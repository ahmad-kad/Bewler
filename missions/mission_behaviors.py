#!/usr/bin/env python3
"""
Mission Behaviors - Specific implementations for each mission type

Contains the detailed logic for:
- Waypoint navigation
- Object detection and approach
- Follow-me behavior
- Delivery missions
- Sample collection

Author: URC 2026 Autonomy Team
"""

import math
import time
from typing import Any, Dict, List, Tuple

import rclpy
from autonomy_interfaces.msg import VisionDetection
from geometry_msgs.msg import PoseStamped, Twist


class WaypointNavigation:
    """
    Waypoint navigation behavior - Navigate between GPS waypoints

    Features:
    - GPS waypoint following
    - Odometry-based dead reckoning
    - Waypoint tolerance checking
    - Timeout handling
    """

    def __init__(self, node: rclpy.node.Node):
        self.node = node
        self.waypoint_tolerance = 1.0  # meters
        self.heading_tolerance = 5.0  # degrees
        self.timeout = 300.0  # 5 minutes per waypoint

    def execute(self, waypoints: List[Dict[str, float]], sensors) -> Dict[str, Any]:
        """
        Execute waypoint navigation mission

        Args:
            waypoints: List of waypoint dictionaries with x, y, heading
            sensors: Sensor interface object

        Returns:
            Mission result dictionary
        """
        self.node.get_logger().info(
            f"Starting waypoint navigation with {len(waypoints)} waypoints"
        )

        completed_waypoints = []
        total_distance = 0.0

        for i, waypoint in enumerate(waypoints):
            self.node.get_logger().info(
                f"Navigating to waypoint {i+1}/{len(waypoints)}"
            )

            start_time = time.time()
            success = False

            while (
                not self._should_abort() and (time.time() - start_time) < self.timeout
            ):
                current_pos = sensors.get_current_position()
                if not current_pos:
                    self.node.get_logger().warn("No position data available")
                    time.sleep(0.5)
                    continue

                # Calculate distance and heading to waypoint
                distance = self._calculate_distance(current_pos, waypoint)
                heading_error = self._calculate_heading_error(current_pos, waypoint)

                self.node.get_logger().debug(".2f")

                # Check if waypoint reached
                if (
                    distance < self.waypoint_tolerance
                    and abs(heading_error) < self.heading_tolerance
                ):
                    self.node.get_logger().info(f"Waypoint {i+1} reached!")
                    completed_waypoints.append(waypoint)
                    total_distance += distance
                    success = True
                    break

                # Compute velocity commands
                vx, vtheta = self._compute_velocity_commands(distance, heading_error)

                # Send commands
                sensors.send_velocity_command(vx, vtheta)

                time.sleep(0.1)

            if not success:
                return {
                    "success": False,
                    "message": f"Failed to reach waypoint {i+1}",
                    "completed_waypoints": len(completed_waypoints),
                    "total_distance": total_distance,
                }

        # Stop at final waypoint
        sensors.send_velocity_command(0.0, 0.0)

        return {
            "success": True,
            "message": f"Completed navigation to {len(completed_waypoints)} waypoints",
            "completed_waypoints": len(completed_waypoints),
            "total_distance": total_distance,
        }

    def _calculate_distance(
        self, current_pos: Tuple[float, float, float], waypoint: Dict[str, float]
    ) -> float:
        """Calculate distance to waypoint"""
        dx = waypoint["x"] - current_pos[0]
        dy = waypoint["y"] - current_pos[1]
        return math.sqrt(dx * dx + dy * dy)

    def _calculate_heading_error(
        self, current_pos: Tuple[float, float, float], waypoint: Dict[str, float]
    ) -> float:
        """Calculate heading error to waypoint (degrees)"""
        dx = waypoint["x"] - current_pos[0]
        dy = waypoint["y"] - current_pos[1]
        target_heading = math.degrees(math.atan2(dy, dx))
        current_heading = math.degrees(current_pos[2])
        error = target_heading - current_heading

        # Normalize to [-180, 180]
        while error > 180:
            error -= 360
        while error < -180:
            error += 360

        return error

    def _compute_velocity_commands(
        self, distance: float, heading_error: float
    ) -> Tuple[float, float]:
        """Compute velocity commands for waypoint approach"""
        # Simple proportional control
        max_speed = 1.0  # m/s
        min_speed = 0.1  # m/s

        # Speed based on distance
        if distance > 5.0:
            vx = max_speed
        elif distance > 1.0:
            vx = max_speed * (distance / 5.0)
        else:
            vx = min_speed

        # Angular velocity based on heading error
        k_angular = 0.02  # rad/s per degree
        vtheta = math.radians(heading_error) * k_angular

        # Limit angular velocity
        vtheta = max(-0.5, min(0.5, vtheta))

        return vx, vtheta

    def _should_abort(self) -> bool:
        """Check if mission should abort"""
        # Check for emergency stop, etc.
        return False


class ObjectDetectionMission:
    """
    Object detection and approach behavior

    Features:
    - Vision-based object detection
    - Precision approach using ArUco tags
    - Search patterns for object location
    - Distance-based approach control
    """

    def __init__(self, node: rclpy.node.Node):
        self.node = node
        self.target_object = None
        self.approach_distance = 0.5  # meters
        self.search_timeout = 300.0  # 5 minutes
        self.vision_timeout = 10.0  # seconds without detection before search

    def execute(self, config: Dict[str, Any], sensors) -> Dict[str, Any]:
        """
        Execute object detection mission

        Args:
            config: Mission configuration
            sensors: Sensor interface

        Returns:
            Mission result
        """
        self.target_object = config.get("target_object", "sample")

        self.node.get_logger().info(
            f"Starting object detection mission for: {self.target_object}"
        )

        start_time = time.time()
        last_detection_time = 0.0
        object_found = False

        while (
            not self._should_abort()
            and (time.time() - start_time) < self.search_timeout
        ):
            # Check vision detections
            detections = sensors.get_vision_detections()

            target_detection = None
            for detection in detections:
                if self._is_target_object(detection):
                    target_detection = detection
                    last_detection_time = time.time()
                    break

            if target_detection:
                object_found = True
                self.node.get_logger().info(
                    f"Target object detected: {self.target_object}"
                )

                # Check distance
                distance = self._calculate_detection_distance(target_detection)

                if distance > self.approach_distance:
                    # Approach the object
                    success = self._approach_object(target_detection, sensors)
                    if success:
                        return {
                            "success": True,
                            "message": f"Successfully approached {self.target_object}",
                            "object_type": self.target_object,
                            "final_distance": distance,
                        }
                else:
                    # Already close enough
                    sensors.send_velocity_command(0.0, 0.0)
                    return {
                        "success": True,
                        "message": f"Already at target distance for {self.target_object}",
                        "object_type": self.target_object,
                        "final_distance": distance,
                    }

            else:
                # No detection - perform search
                time_since_detection = time.time() - last_detection_time
                if time_since_detection > self.vision_timeout:
                    self._perform_search_pattern(sensors)

        if not object_found:
            return {
                "success": False,
                "message": f"Object detection timeout - {self.target_object} not found",
                "search_duration": time.time() - start_time,
            }
        else:
            return {
                "success": False,
                "message": f"Failed to approach {self.target_object}",
                "object_found": True,
            }

    def _is_target_object(self, detection: VisionDetection) -> bool:
        """Check if detection matches target object"""
        # Simple label matching - could be enhanced with ML classification
        target_labels = [self.target_object.lower()]
        if self.target_object == "sample":
            target_labels.extend(["rock", "specimen", "mineral"])

        return any(label in detection.label.lower() for label in target_labels)

    def _calculate_detection_distance(self, detection: VisionDetection) -> float:
        """Estimate distance from detection (simplified)"""
        # This would use depth information from stereo/RGBD camera
        # For now, use bounding box size as distance proxy
        bbox_size = detection.bbox.width * detection.bbox.height
        # Larger bbox = closer object (inverse relationship)
        distance = 10.0 / max(bbox_size, 0.1)  # Rough estimation
        return min(distance, 10.0)  # Cap at 10m

    def _approach_object(self, detection: VisionDetection, sensors) -> bool:
        """Approach detected object using vision guidance"""
        self.node.get_logger().info("Approaching detected object...")

        start_time = time.time()
        approach_timeout = 30.0  # 30 seconds

        while (
            not self._should_abort() and (time.time() - start_time) < approach_timeout
        ):
            # Re-check detection
            detections = sensors.get_vision_detections()
            current_detection = None

            for det in detections:
                if self._is_target_object(det):
                    current_detection = det
                    break

            if not current_detection:
                self.node.get_logger().warn("Lost object detection during approach")
                return False

            distance = self._calculate_detection_distance(current_detection)

            if distance < self.approach_distance:
                # Success!
                sensors.send_velocity_command(0.0, 0.0)
                return True

            # Compute approach velocity
            vx, vtheta = self._compute_approach_velocity(current_detection, distance)
            sensors.send_velocity_command(vx, vtheta)

            time.sleep(0.1)

        return False

    def _compute_approach_velocity(
        self, detection: VisionDetection, distance: float
    ) -> Tuple[float, float]:
        """Compute velocity for object approach"""
        # Center object in frame
        image_center_x = 320  # Assuming 640x480 image
        object_center_x = detection.bbox.center.x

        # Angular velocity to center object
        angular_error = (object_center_x - image_center_x) / 320.0  # Normalized
        vtheta = angular_error * 0.5  # Scale factor

        # Forward velocity based on distance
        if distance > 2.0:
            vx = 0.3
        elif distance > 1.0:
            vx = 0.2
        else:
            vx = 0.1

        return vx, vtheta

    def _perform_search_pattern(self, sensors):
        """Perform search pattern to locate objects"""
        # Spiral search pattern
        self.node.get_logger().info("Performing search pattern...")

        # Move in expanding spiral
        twist = Twist()
        twist.linear.x = 0.2  # Slow forward
        twist.angular.z = 0.3  # Slow turn

        sensors.send_velocity_command(twist.linear.x, twist.angular.z)
        time.sleep(3.0)  # Search for 3 seconds
        sensors.send_velocity_command(0.0, 0.0)

    def _should_abort(self) -> bool:
        """Check if mission should abort"""
        return False


class FollowMeMission:
    """
    Follow-me behavior using ArUco tag tracking

    Features:
    - ArUco tag detection and tracking
    - Adaptive following distance
    - Obstacle avoidance
    - Person following with smooth motion
    """

    def __init__(self, node: rclpy.node.Node):
        self.node = node
        self.target_tag_id = 42
        self.desired_distance = 2.0  # meters
        self.distance_tolerance = 0.5  # meters
        self.max_follow_speed = 1.0  # m/s
        self.tag_timeout = 5.0  # seconds

    def execute(self, config: Dict[str, Any], sensors) -> Dict[str, Any]:
        """
        Execute follow-me mission

        Args:
            config: Mission configuration
            sensors: Sensor interface

        Returns:
            Mission result
        """
        self.target_tag_id = config.get("tag_id", 42)
        follow_duration = config.get("duration", 60.0)  # 1 minute default

        self.node.get_logger().info(
            f"Starting follow-me mission for tag {self.target_tag_id}"
        )

        start_time = time.time()
        last_tag_time = time.time()

        while not self._should_abort() and (time.time() - start_time) < follow_duration:
            # Get ArUco tag pose
            tag_pose = sensors.get_aruco_tag_pose(self.target_tag_id)

            if tag_pose:
                last_tag_time = time.time()

                # Calculate following behavior
                vx, vtheta = self._compute_follow_velocity(tag_pose)

                # Send velocity commands
                sensors.send_velocity_command(vx, vtheta)

                # Log progress
                progress = ((time.time() - start_time) / follow_duration) * 100.0
                self.node.get_logger().debug(".1f")

            else:
                # Tag not detected
                time_since_tag = time.time() - last_tag_time
                if time_since_tag > self.tag_timeout:
                    self.node.get_logger().warn(".1f")
                    # Search behavior
                    self._perform_search_behavior(sensors)
                else:
                    # Slow down and wait
                    sensors.send_velocity_command(0.0, 0.0)

            time.sleep(0.1)

        # Stop following
        sensors.send_velocity_command(0.0, 0.0)

        return {
            "success": True,
            "message": f"Follow-me mission completed for tag {self.target_tag_id}",
            "follow_duration": time.time() - start_time,
            "tag_id": self.target_tag_id,
        }

    def _compute_follow_velocity(self, tag_pose: PoseStamped) -> Tuple[float, float]:
        """Compute velocity commands for following behavior"""
        # Extract position
        tag_x = tag_pose.pose.position.x
        tag_y = tag_pose.pose.position.y

        # Calculate distance and angle
        distance = math.sqrt(tag_x * tag_x + tag_y * tag_y)
        angle = math.atan2(tag_y, tag_x)

        # Distance control
        if distance > self.desired_distance + self.distance_tolerance:
            # Too far - speed up
            vx = min(self.max_follow_speed, (distance - self.desired_distance) * 0.5)
        elif distance < self.desired_distance - self.distance_tolerance:
            # Too close - slow down or reverse
            vx = max(-0.2, (distance - self.desired_distance) * 0.3)
        else:
            # Good distance - maintain
            vx = 0.0

        # Angular control to face the tag
        vtheta = angle * 1.0  # Proportional control
        vtheta = max(-0.5, min(0.5, vtheta))  # Limit angular velocity

        return vx, vtheta

    def _perform_search_behavior(self, sensors):
        """Perform search behavior when tag is lost"""
        # Rotate in place to search
        sensors.send_velocity_command(0.0, 0.3)  # Pure rotation
        time.sleep(1.0)
        sensors.send_velocity_command(0.0, 0.0)

    def _should_abort(self) -> bool:
        """Check if follow mission should abort"""
        return False


class DeliveryMission:
    """
    Delivery mission - Pick up and deliver objects

    Features:
    - Navigate to pickup location
    - Perform pickup operation
    - Navigate to delivery location
    - Perform delivery operation
    - State tracking for carried objects
    """

    def __init__(self, node: rclpy.node.Node):
        self.node = node
        self.has_object = False
        self.pickup_location = None
        self.delivery_location = None

    def execute(self, config: Dict[str, Any], sensors) -> Dict[str, Any]:
        """
        Execute delivery mission

        Args:
            config: Mission configuration with pickup/delivery locations
            sensors: Sensor interface

        Returns:
            Mission result
        """
        self.pickup_location = config.get("pickup_location")
        self.delivery_location = config.get("delivery_location")

        if not self.pickup_location or not self.delivery_location:
            return {
                "success": False,
                "message": "Delivery mission requires both pickup and delivery locations",
            }

        self.node.get_logger().info("Starting delivery mission")

        # Phase 1: Navigate to pickup
        self.node.get_logger().info("Phase 1: Navigating to pickup location")
        pickup_success = sensors.navigate_to_position(self.pickup_location)

        if not pickup_success:
            return {"success": False, "message": "Failed to reach pickup location"}

        # Phase 2: Perform pickup
        self.node.get_logger().info("Phase 2: Performing pickup operation")
        pickup_result = self._perform_pickup_operation(sensors)

        if not pickup_result["success"]:
            return {
                "success": False,
                "message": f'Pickup failed: {pickup_result["message"]}',
            }

        self.has_object = True

        # Phase 3: Navigate to delivery
        self.node.get_logger().info("Phase 3: Navigating to delivery location")
        delivery_nav_success = sensors.navigate_to_position(self.delivery_location)

        if not delivery_nav_success:
            return {"success": False, "message": "Failed to reach delivery location"}

        # Phase 4: Perform delivery
        self.node.get_logger().info("Phase 4: Performing delivery operation")
        delivery_result = self._perform_delivery_operation(sensors)

        if not delivery_result["success"]:
            return {
                "success": False,
                "message": f'Delivery failed: {delivery_result["message"]}',
            }

        return {
            "success": True,
            "message": "Delivery mission completed successfully",
            "pickup_time": pickup_result.get("duration", 0),
            "delivery_time": delivery_result.get("duration", 0),
        }

    def _perform_pickup_operation(self, sensors) -> Dict[str, Any]:
        """Perform pickup operation at current location"""
        # This would interface with the arm/endeffector system
        # For now, simulate the operation

        self.node.get_logger().info("Simulating pickup operation...")

        # Stop movement
        sensors.send_velocity_command(0.0, 0.0)

        # Simulate arm movement and grasping
        time.sleep(3.0)  # Simulate operation time

        return {
            "success": True,
            "message": "Pickup operation completed",
            "duration": 3.0,
        }

    def _perform_delivery_operation(self, sensors) -> Dict[str, Any]:
        """Perform delivery operation at current location"""
        # This would interface with the arm/endeffector system

        self.node.get_logger().info("Simulating delivery operation...")

        # Stop movement
        sensors.send_velocity_command(0.0, 0.0)

        # Simulate arm movement and release
        time.sleep(3.0)  # Simulate operation time

        self.has_object = False

        return {
            "success": True,
            "message": "Delivery operation completed",
            "duration": 3.0,
        }
