"""
Proximity Monitor for Auto-Safe Collision Avoidance.

Monitors proximity sensors (depth cameras, LIDAR, ultrasonic) for objects
that are too close to the rover, automatically triggering safestop to avoid collisions.
"""

import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
from autonomy_interfaces.aoi_tracker import AOIConfig, AOITracker
from autonomy_interfaces.msg import SafetyStatus
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, LaserScan, Range
from std_msgs.msg import String


class ProximitySensorType(Enum):
    """Types of proximity sensors."""

    LIDAR = "LIDAR"
    ULTRASONIC = "ULTRASONIC"
    DEPTH_CAMERA = "DEPTH_CAMERA"


@dataclass
class ProximityReading:
    """Single proximity sensor reading."""

    sensor_type: ProximitySensorType
    sensor_id: str
    timestamp: float
    distance: float
    angle: float  # radians from forward direction
    position: Optional[Point] = None
    confidence: float = 1.0


@dataclass
class ProximityZone:
    """Safety zone definition."""

    name: str
    min_distance: float  # meters
    max_distance: float  # meters
    angle_start: float  # radians
    angle_end: float  # radians
    severity: str = "WARNING"  # "WARNING", "CRITICAL", "EMERGENCY"


class ProximityMonitor(Node):
    """
    Proximity Monitor for Auto-Safe Collision Avoidance.

    Monitors multiple proximity sensors and triggers safety stops when
    objects are detected too close to the rover.
    """

    def __init__(self):
        super().__init__("proximity_monitor")

        self.logger = self.get_logger()
        self.bridge = CvBridge()

        # Auto-safe thresholds (much closer than follow-me)
        self.auto_safe_distance = 0.8  # meters - trigger safestop
        self.warning_distance = 1.5  # meters - trigger warning
        self.follow_me_distance = 2.0  # meters - follow-me threshold

        # Safety zones
        self.safety_zones = self._define_safety_zones()

        # Sensor readings
        self.recent_readings: List[ProximityReading] = []
        self.max_reading_history = 50

        # Proximity violation state
        self.proximity_violation_active = False
        self.last_violation_time = 0.0
        self.violation_cooldown = 1.0  # seconds

        # AoI tracking for safety-critical sensors
        self.aoi_config = AOIConfig(
            acceptable_threshold=0.1,  # 100ms for safety sensors
            optimal_threshold=0.05,  # 50ms optimal
            critical_threshold=0.5,  # 500ms critical
        )
        self.aoi_trackers = {}  # sensor_id -> AOITracker

        # QoS profiles
        self.qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Setup publishers and subscribers
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_timers()

        self.logger.info("Proximity Monitor initialized")

    def _define_safety_zones(self) -> List[ProximityZone]:
        """Define safety zones around the rover."""
        return [
            # Front critical zone - immediate danger
            ProximityZone(
                name="front_critical",
                min_distance=0.0,
                max_distance=0.5,
                angle_start=-math.pi / 4,  # -45 degrees
                angle_end=math.pi / 4,  # +45 degrees
                severity="EMERGENCY",
            ),
            # Front warning zone
            ProximityZone(
                name="front_warning",
                min_distance=0.5,
                max_distance=self.auto_safe_distance,
                angle_start=-math.pi / 3,  # -60 degrees
                angle_end=math.pi / 3,  # +60 degrees
                severity="CRITICAL",
            ),
            # Side zones
            ProximityZone(
                name="left_side",
                min_distance=0.0,
                max_distance=self.auto_safe_distance,
                angle_start=math.pi / 3,
                angle_end=2 * math.pi / 3,
                severity="WARNING",
            ),
            ProximityZone(
                name="right_side",
                min_distance=0.0,
                max_distance=self.auto_safe_distance,
                angle_start=-2 * math.pi / 3,
                angle_end=-math.pi / 3,
                severity="WARNING",
            ),
            # Rear zone
            ProximityZone(
                name="rear",
                min_distance=0.0,
                max_distance=0.3,  # tighter rear zone
                angle_start=2 * math.pi / 3,
                angle_end=-2 * math.pi / 3,
                severity="WARNING",
            ),
        ]

    def _setup_publishers(self):
        """Setup ROS2 publishers."""
        self.proximity_status_pub = self.create_publisher(String, "/safety/proximity_status", self.qos_reliable)

        self.safety_trigger_pub = self.create_publisher(SafetyStatus, "/safety/proximity_trigger", self.qos_reliable)

    def _setup_subscribers(self):
        """Setup ROS2 subscribers for proximity sensors."""
        # LIDAR scanner
        self.create_subscription(
            LaserScan,
            "/scan",
            self._lidar_callback,
            self.qos_sensor,
            callback_group=self.create_callback_group(),
        )

        # Ultrasonic sensors (multiple possible)
        for i in range(4):  # Assume up to 4 ultrasonic sensors
            self.create_subscription(
                Range,
                f"/ultrasonic_{i}/range",
                self._ultrasonic_callback,
                self.qos_sensor,
                callback_group=self.create_callback_group(),
            )

        # Depth camera
        self.create_subscription(
            Image,
            "/camera/depth/image_raw",
            self._depth_callback,
            self.qos_sensor,
            callback_group=self.create_callback_group(),
        )

        self.create_subscription(
            CameraInfo,
            "/camera/camera_info",
            self._camera_info_callback,
            self.qos_reliable,
            callback_group=self.create_callback_group(),
        )

    def _setup_timers(self):
        """Setup monitoring timers."""
        # Proximity check timer - high frequency for safety
        self.proximity_timer = self.create_timer(0.1, self._check_proximity)  # 10Hz

        # Status publishing timer
        self.status_timer = self.create_timer(0.5, self._publish_proximity_status)  # 2Hz

    def _lidar_callback(self, msg: LaserScan):
        """Process LIDAR scan data with AoI tracking."""
        try:
            current_time = time.time()
            sensor_id = "lidar_main"

            # Track AoI for safety sensor
            sensor_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            self._track_sensor_aoi(sensor_id, sensor_timestamp)

            # Convert polar coordinates to readings
            angle = msg.angle_min
            for i, distance in enumerate(msg.ranges):
                if msg.range_min < distance < msg.range_max:
                    reading = ProximityReading(
                        sensor_type=ProximitySensorType.LIDAR,
                        sensor_id=sensor_id,
                        timestamp=current_time,
                        distance=distance,
                        angle=angle,
                        confidence=1.0,
                    )
                    self._add_reading(reading)
                angle += msg.angle_increment

        except Exception as e:
            self.logger.error(f"LIDAR callback error: {e}")

    def _track_sensor_aoi(self, sensor_id: str, sensor_timestamp: float):
        """Track Age of Information for safety-critical sensors."""
        # Initialize tracker if needed
        if sensor_id not in self.aoi_trackers:
            self.aoi_trackers[sensor_id] = AOITracker(self.aoi_config)

        # Update AoI tracking
        current_aoi = self.aoi_trackers[sensor_id].update(sensor_timestamp)

        # Log AoI violations (throttled to prevent spam)
        if current_aoi > self.aoi_config.critical_threshold:
            self.logger.warn(
                f"Safety sensor AoI violation: {sensor_id}={current_aoi:.3f}s",
                throttle_duration_sec=5.0,
            )

    def _ultrasonic_callback(self, msg: Range):
        """Process ultrasonic range data."""
        try:
            current_time = time.time()

            # Extract sensor ID from topic
            sensor_id = msg.header.frame_id or "ultrasonic_unknown"

            # Convert to forward-facing angle (simplified)
            angle = 0.0  # Assume forward-facing, adjust based on mounting

            if msg.range > msg.min_range and msg.range < msg.max_range:
                reading = ProximityReading(
                    sensor_type=ProximitySensorType.ULTRASONIC,
                    sensor_id=sensor_id,
                    timestamp=current_time,
                    distance=msg.range,
                    angle=angle,
                    confidence=0.8,  # Ultrasonic can have more noise
                )
                self._add_reading(reading)

        except Exception as e:
            self.logger.error(f"Ultrasonic callback error: {e}")

    def _depth_callback(self, msg: Image):
        """Process depth camera data."""
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            current_time = time.time()

            # Simple proximity detection - find minimum distance in center region
            height, width = cv_image.shape
            center_y, center_x = height // 2, width // 2

            # Look at center region (reduce processing)
            roi = cv_image[center_y - 50 : center_y + 50, center_x - 50 : center_x + 50]
            if roi.size > 0:
                min_distance = np.min(roi[roi > 0]) / 1000.0  # Convert mm to meters

                if min_distance > 0.1:  # Filter out invalid readings
                    reading = ProximityReading(
                        sensor_type=ProximitySensorType.DEPTH_CAMERA,
                        sensor_id="depth_camera",
                        timestamp=current_time,
                        distance=min_distance,
                        angle=0.0,  # Center of camera view
                        confidence=0.9,
                    )
                    self._add_reading(reading)

        except CvBridgeError as e:
            self.logger.error(f"Depth image conversion error: {e}")
        except Exception as e:
            self.logger.error(f"Depth callback error: {e}")

    def _camera_info_callback(self, msg: CameraInfo):
        """Store camera info for coordinate transformations."""
        # Could be used for more sophisticated depth processing

    def _add_reading(self, reading: ProximityReading):
        """Add a proximity reading to the history."""
        self.recent_readings.append(reading)

        # Keep only recent readings
        if len(self.recent_readings) > self.max_reading_history:
            self.recent_readings = self.recent_readings[-self.max_reading_history :]

    def _check_proximity(self):
        """Check for proximity violations with AoI-aware safety decisions."""
        if not self.recent_readings:
            return

        current_time = time.time()

        # Check cooldown period
        if current_time - self.last_violation_time < self.violation_cooldown:
            return

        # Verify we have fresh sensor data before making safety decisions
        if not self._has_fresh_safety_data():
            self.logger.warn(
                "No fresh safety sensor data available - cannot ensure safety",
                throttle_duration_sec=2.0,
            )
            # In safety-critical situations, we might want to trigger conservative safety
            return

        # Find closest reading in each safety zone (using AoI-filtered readings)
        violations = []
        for zone in self.safety_zones:
            closest_reading = self._find_closest_in_zone_aoi_aware(zone)
            if closest_reading and closest_reading.distance <= zone.max_distance:
                violations.append((zone, closest_reading))

        if violations:
            # Find most severe violation
            most_severe_zone, reading = max(violations, key=lambda x: self._severity_priority(x[0].severity))

            self._trigger_proximity_violation(most_severe_zone, reading)
            self.last_violation_time = current_time

    def _has_fresh_safety_data(self) -> bool:
        """Check if we have sufficiently fresh data from safety-critical sensors."""
        current_time = time.time()

        # Check that all tracked safety sensors have fresh data
        for sensor_id, tracker in self.aoi_trackers.items():
            current_aoi = tracker.get_current_aoi()
            if current_aoi > self.aoi_config.acceptable_threshold:
                return False

        return len(self.aoi_trackers) > 0  # At least one safety sensor must be tracked

    def _find_closest_in_zone_aoi_aware(self, zone: ProximityZone) -> Optional[ProximityReading]:
        """Find the closest reading within a safety zone, considering AoI freshness."""
        candidates = []
        current_time = time.time()

        for reading in self.recent_readings:
            # Enhanced freshness check using AoI tracking
            sensor_aoi = float("inf")
            if reading.sensor_id in self.aoi_trackers:
                sensor_aoi = self.aoi_trackers[reading.sensor_id].get_current_aoi()

            # Only consider readings that are both temporally and AoI fresh
            time_fresh = (current_time - reading.timestamp) <= 0.1  # 100ms time freshness
            aoi_fresh = sensor_aoi <= self.aoi_config.acceptable_threshold

            if time_fresh and aoi_fresh:
                # Check if reading is within zone angle
                if zone.angle_start <= reading.angle <= zone.angle_end:
                    candidates.append(reading)

        return min(candidates, key=lambda r: r.distance) if candidates else None

    def _find_closest_in_zone(self, zone: ProximityZone) -> Optional[ProximityReading]:
        """Find the closest reading within a safety zone."""
        candidates = []

        for reading in self.recent_readings:
            # Check if reading is recent (within 0.5 seconds)
            if time.time() - reading.timestamp > 0.5:
                continue

            # Check if reading is within zone angle
            if zone.angle_start <= reading.angle <= zone.angle_end:
                candidates.append(reading)

        return min(candidates, key=lambda r: r.distance) if candidates else None

    def _severity_priority(self, severity: str) -> int:
        """Get priority value for severity sorting."""
        priorities = {"WARNING": 1, "CRITICAL": 2, "EMERGENCY": 3}
        return priorities.get(severity, 0)

    def _trigger_proximity_violation(self, zone: ProximityZone, reading: ProximityReading):
        """Trigger a proximity violation safety event."""
        self.proximity_violation_active = True

        severity_map = {
            "WARNING": "WARNING",
            "CRITICAL": "CRITICAL",
            "EMERGENCY": "EMERGENCY",
        }

        self.logger.warning(
            f"PROXIMITY VIOLATION: {zone.name} - {reading.distance:.2f}m "
            f"(severity: {zone.severity}) from {reading.sensor_type.value}"
        )

        # Publish safety trigger
        safety_msg = SafetyStatus()
        safety_msg.header.stamp = self.get_clock().now().to_msg()
        safety_msg.header.frame_id = "proximity_monitor"

        safety_msg.is_safe = False
        safety_msg.safety_level = severity_map[zone.severity]
        safety_msg.active_triggers = ["PROXIMITY_VIOLATION"]
        safety_msg.trigger_type = "PROXIMITY_VIOLATION"
        safety_msg.trigger_source = f"proximity_monitor_{reading.sensor_type.value}"
        safety_msg.trigger_time = self.get_clock().now().to_msg()
        safety_msg.trigger_description = (
            f"Object detected in {zone.name} zone at {reading.distance:.2f}m " f"(threshold: {zone.max_distance:.2f}m)"
        )
        safety_msg.requires_manual_intervention = zone.severity == "EMERGENCY"
        safety_msg.can_auto_recover = zone.severity != "EMERGENCY"
        safety_msg.recovery_steps = [
            "Clear the area around the rover",
            "Verify sensor readings are valid",
            "Resume operation when safe",
        ]
        safety_msg.estimated_recovery_time = 30.0
        safety_msg.context_state = "unknown"  # Will be filled by state machine
        safety_msg.mission_phase = "unknown"
        safety_msg.safe_to_retry = True

        self.safety_trigger_pub.publish(safety_msg)

    def _publish_proximity_status(self):
        """Publish current proximity status."""
        status_data = {
            "timestamp": time.time(),
            "violation_active": self.proximity_violation_active,
            "reading_count": len(self.recent_readings),
            "auto_safe_distance": self.auto_safe_distance,
            "warning_distance": self.warning_distance,
            "follow_me_distance": self.follow_me_distance,
            "zones": [],
        }

        # Add zone status
        for zone in self.safety_zones:
            closest = self._find_closest_in_zone(zone)
            zone_status = {
                "name": zone.name,
                "severity": zone.severity,
                "max_distance": zone.max_distance,
                "closest_reading": closest.distance if closest else None,
                "in_violation": (closest.distance <= zone.max_distance if closest else False),
            }
            status_data["zones"].append(zone_status)

        import json

        status_msg = String()
        status_msg.data = json.dumps(status_data, indent=2)
        self.proximity_status_pub.publish(status_msg)

        # Clear violation flag after publishing
        if self.proximity_violation_active:
            self.proximity_violation_active = False

    def get_proximity_status(self) -> Dict[str, Any]:
        """Get comprehensive proximity status."""
        return {
            "violation_active": self.proximity_violation_active,
            "reading_count": len(self.recent_readings),
            "auto_safe_distance": self.auto_safe_distance,
            "warning_distance": self.warning_distance,
            "follow_me_distance": self.follow_me_distance,
            "safety_zones": [
                {
                    "name": zone.name,
                    "severity": zone.severity,
                    "max_distance": zone.max_distance,
                    "closest_reading": self._find_closest_in_zone(zone),
                }
                for zone in self.safety_zones
            ],
        }


def main(args=None):
    """Main entry point for proximity monitor node."""
    rclpy.init(args=args)

    proximity_monitor = ProximityMonitor()

    try:
        rclpy.spin(proximity_monitor)
    except KeyboardInterrupt:
        proximity_monitor.logger.info("Proximity Monitor shutting down")
    finally:
        proximity_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
