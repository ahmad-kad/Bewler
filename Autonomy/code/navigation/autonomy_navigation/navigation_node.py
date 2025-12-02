"""
Main navigation controller for URC 2026 rover.

Coordinates GNSS waypoint navigation, terrain-adaptive path planning,
AR tag precision approaches, obstacle avoidance, and mission progress tracking.
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

import rclpy
from autonomy_interfaces.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..'))

from utilities import (
    AutonomyNode, NodeParameters, MessagePipeline,
    success, failure, ValidationError, ProcessingError,
    OperationResult, Failure
)
from typing import Dict, Any
from .gnss_processor import GNSSProcessor
from .motion_controller import MotionController
from .path_planner import PathPlanner


@dataclass
class Waypoint:
    """Geographic waypoint with navigation metadata."""

    latitude: float
    longitude: float
    altitude: float = 0.0
    name: str = ""
    precision_required: bool = False


@dataclass
class NavigationGoal:
    """Navigation goal with approach constraints."""

    waypoint: Waypoint
    approach_tolerance: float = 1.0  # meters
    orientation_required: bool = False
    target_heading: float = 0.0  # radians


class NavigationNode(AutonomyNode):
    """
    Main navigation controller coordinating all navigation subsystems.

    Integrates GNSS waypoint navigation, terrain-adaptive path planning,
    AR tag precision approaches, obstacle avoidance, and mission tracking.
    """

    def __init__(self) -> None:
        """Initialize navigation node with streamlined infrastructure."""
        super().__init__("navigation_node", NodeParameters.for_navigation())

        # Initialize navigation subsystems
        self.gnss_processor = GNSSProcessor()
        self.path_planner = PathPlanner()
        self.motion_controller = MotionController()

        # Navigation state
        self.current_waypoint: Optional[Waypoint] = None
        self.current_path: List[Tuple[float, float]] = []
        self.current_pose = None
        self.last_imu: Optional[Imu] = None
        self.is_navigating = False

        # Setup interfaces with automatic registration
        self._setup_interfaces()

        # Setup processing pipeline
        self._setup_processing_pipeline()

        self.logger.info("Navigation node initialized with subsystems",
                        subsystems=["gnss", "path_planner", "motion_controller", "terrain"])

    def _setup_interfaces(self) -> None:
        """Setup ROS2 interfaces with automatic registration."""
        # Subscribers
        self.interface_factory.create_subscriber(
            NavSatFix, "/gnss/fix", self._gnss_callback
        )
        self.interface_factory.create_subscriber(
            Imu, "/imu/data", self._imu_callback
        )

        # Publishers
        self.cmd_vel_pub = self.interface_factory.create_publisher(
            Twist, "/cmd_vel"
        )
        self.waypoint_pub = self.interface_factory.create_publisher(
            PoseStamped, "/navigation/current_waypoint"
        )

        # Services
        self.interface_factory.create_service(
            Trigger, "/navigation/stop", self._stop_navigation_callback
        )

        # Action server
        self.navigate_action_server = self.interface_factory.create_action_server(
            NavigateToPose, "/navigate_to_pose", self._navigate_to_pose_callback
        )

        # Control timer
        self.interface_factory.create_timer(
            1.0 / self.params.update_rate, self._control_loop, "control"
        )

    def _setup_processing_pipeline(self) -> None:
        """Setup data processing pipeline."""
        self.navigation_pipeline = (
            MessagePipeline(self.logger, self.trace_data)
            .add_step(self._validate_navigation_data, "validation")
            .add_step(self._process_sensor_fusion, "sensor_fusion")
            .add_step(self._update_navigation_state, "state_update")
        )

    def _validate_navigation_data(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Validate navigation input data."""
        required_fields = ["gnss", "imu", "goal"]
        for field in required_fields:
            if field not in data:
                raise ValueError(f"Missing required field: {field}")
        return data

    def _process_sensor_fusion(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Process sensor fusion for navigation."""
        # Sensor fusion logic
        fused_pose = self.gnss_processor.fuse_sensors(data["gnss"], data["imu"])
        data["fused_pose"] = fused_pose
        return data

    def _update_navigation_state(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Update navigation state based on processed data."""
        if self.is_navigating and self.current_path:
            # Update progress along path
            progress = self._calculate_path_progress(data["fused_pose"])
            data["progress"] = progress
        return data

    def _gnss_callback(self, msg: NavSatFix) -> None:
        """Handle GNSS data updates."""
        self.trace_data("gnss_received", msg)
        # Process GNSS data through pipeline
        pipeline_data = {"gnss": msg, "imu": self.last_imu, "goal": self.current_waypoint}
        result = self.navigation_pipeline.process(pipeline_data)

        if isinstance(result, Failure):
            self.logger.error("GNSS processing failed", error=result.error)
        else:
            self.logger.debug("GNSS data processed successfully")

    def _imu_callback(self, msg: Imu) -> None:
        """Handle IMU data updates."""
        self.last_imu = msg
        self.trace_data("imu_received", msg)

    def _navigate_to_pose_callback(self, goal_handle) -> NavigateToPose.Result:
        """Handle navigation action requests."""
        # Validate operation requirements
        validation = self.validate_operation(
            "navigate_to_pose",
            required_state="idle",
            required_interfaces={"subscribers": 2}  # GNSS and IMU
        )

        if isinstance(validation, Failure):
            self.logger.error("Navigation validation failed", error=validation.error)
            goal_handle.abort()
            return NavigateToPose.Result()

        # Extract goal and start navigation
        goal = goal_handle.request.target_pose
        result = self.start_navigation_to_pose(goal)

        if isinstance(result, Failure):
            self.logger.error("Navigation start failed", error=result.error)
            goal_handle.abort()
            return NavigateToPose.Result()

        goal_handle.succeed()
        return NavigateToPose.Result()

    def _stop_navigation_callback(self, request, response) -> Trigger.Response:
        """Handle navigation stop requests."""
        self.stop_navigation()
        response.success = True
        response.message = "Navigation stopped"
        return response

    def _control_loop(self) -> None:
        """Main navigation control loop."""
        if not self.is_navigating:
            return

        try:
            # Generate velocity commands
            cmd_vel = self.motion_controller.compute_velocity_commands(
                self.current_path, self.current_pose
            )

            # Publish command
            self.cmd_vel_pub.publish(cmd_vel)
            self.trace_data("velocity_command", cmd_vel)

            # Check if goal reached
            if self._is_goal_reached():
                self._complete_navigation()

        except Exception as e:
            self.logger.error("Control loop error", error=e)
            self.transition_to("error", f"Control loop failed: {str(e)}")

    def start_navigation_to_pose(self, target_pose: PoseStamped) -> OperationResult[bool, ProcessingError]:
        """Start navigation to target pose with validation."""
        try:
            # Convert to waypoint
            waypoint = self._pose_to_waypoint(target_pose)

            # Validate waypoint
            validation = self._validate_waypoint(waypoint)
            if isinstance(validation, Failure):
                return validation

            # Plan path
            path_result = self.path_planner.plan_path(
                self.current_pose, waypoint, self.params.node_specific_params
            )

            if not path_result:
                return failure(
                    ProcessingError("path_planning", "no_valid_path_found"),
                    operation="start_navigation"
                )

            # Start navigation
            self.current_waypoint = waypoint
            self.current_path = path_result
            self.is_navigating = True
            self.transition_to("navigating", f"Goal: {waypoint}")

            self.logger.info("Navigation started",
                           waypoint=waypoint.name,
                           path_length=len(self.current_path))

            return success(True, "start_navigation")

        except Exception as e:
            return failure(
                ProcessingError("start_navigation", f"unexpected_error: {str(e)}"),
                operation="start_navigation"
            )

    def stop_navigation(self) -> None:
        """Stop current navigation."""
        if self.is_navigating:
            self.is_navigating = False
            self.current_path = []
            self.transition_to("idle", "Navigation stopped")

            # Publish zero velocity to stop
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)

            self.logger.info("Navigation stopped")

    def _is_goal_reached(self) -> bool:
        """Check if current goal has been reached."""
        if not self.current_waypoint or not self.current_pose:
            return False

        distance = math.sqrt(
            (self.current_waypoint.latitude - self.current_pose[0])**2 +
            (self.current_waypoint.longitude - self.current_pose[1])**2
        )

        return distance < self.params.node_specific_params["waypoint_tolerance"]

    def _complete_navigation(self) -> None:
        """Handle successful navigation completion."""
        self.logger.info("Navigation goal reached",
                        waypoint=self.current_waypoint.name if self.current_waypoint else "unknown")

        self.is_navigating = False
        self.transition_to("completed", "Goal reached")

    def _pose_to_waypoint(self, pose: PoseStamped) -> Waypoint:
        """Convert PoseStamped to Waypoint."""
        # Simplified conversion - would need proper coordinate transformation
        return Waypoint(
            latitude=pose.pose.position.x,  # Simplified
            longitude=pose.pose.position.y, # Simplified
            altitude=pose.pose.position.z,
            name=f"waypoint_{int(pose.header.stamp.sec)}"
        )

    def _validate_waypoint(self, waypoint: Waypoint) -> OperationResult[Waypoint, ValidationError]:
        """Validate waypoint parameters."""
        if not (-90 <= waypoint.latitude <= 90):
            return failure(ValidationError("latitude", waypoint.latitude, "must be between -90 and 90"))

        if not (-180 <= waypoint.longitude <= 180):
            return failure(ValidationError("longitude", waypoint.longitude, "must be between -180 and 180"))

        return success(waypoint)

    def _calculate_path_progress(self, current_pose) -> float:
        """Calculate progress along current path."""
        if not self.current_path:
            return 0.0

        # Simplified progress calculation
        return min(1.0, len(self.current_path) * 0.1)  # Placeholder

    def set_navigation_goal(self, goal: NavigationGoal) -> bool:
        """Set a new navigation goal"""
        if not self.current_position:
            self.get_logger().error("No current position available")
            return False

        self.current_goal = goal
        self.goal_start_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.get_logger().info(f"Set navigation goal: {goal.waypoint.name}")
        return True

    def check_goal_timeout(self) -> bool:
        """Check if current goal has timed out"""
        if not self.goal_start_time:
            return False

        elapsed = self.get_clock().now().seconds_nanoseconds()[0] - self.goal_start_time
        return elapsed > self.goal_timeout

    def publish_velocity_commands(self, linear_vel: float, angular_vel: float):
        """Publish velocity commands to motion controller"""
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_publisher.publish(twist)

    def publish_current_waypoint(self):
        """Publish current waypoint information"""
        if self.current_goal:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = self.current_goal.waypoint.latitude
            pose.pose.position.y = self.current_goal.waypoint.longitude
            pose.pose.position.z = self.current_goal.waypoint.altitude
            self.current_waypoint_publisher.publish(pose)

    def stop_motion(self):
        """Stop all motion"""
        twist = Twist()  # Zero velocities
        self.cmd_vel_publisher.publish(twist)

    def status_callback(self):
        """Publish navigation status"""
        status_msg = String()
        status_msg.data = f"Navigation: {self.current_state.value}"
        if self.current_goal:
            status_msg.data += f" | Goal: {self.current_goal.waypoint.name}"
        if self.current_position:
            status_msg.data += ".2f"
        self.status_publisher.publish(status_msg)

    def calculate_distance_bearing(
        self, pos1: Tuple[float, float, float], pos2: Tuple[float, float, float]
    ) -> Tuple[float, float]:
        """Calculate distance and bearing between two positions"""
        # Simplified calculation - would use proper geodesy in production
        lat1, lon1, alt1 = pos1
        lat2, lon2, alt2 = pos2

        # Convert to radians
        lat1_rad, lon1_rad = math.radians(lat1), math.radians(lon1)
        lat2_rad, lon2_rad = math.radians(lat2), math.radians(lon2)

        # Haversine distance
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        a = (
            math.sin(dlat / 2) ** 2
            + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon / 2) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = 6371000 * c  # Earth radius in meters

        # Bearing calculation
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(
            lat2_rad
        ) * math.cos(dlon)
        bearing = math.atan2(y, x)

        return distance, bearing

    def extract_heading_from_imu(self, imu_msg: Imu) -> float:
        """Extract heading from IMU quaternion"""
        # Simplified quaternion to euler conversion
        # In production, would use proper conversion library
        q = imu_msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        heading = math.atan2(siny_cosp, cosy_cosp)
        return heading


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    navigation_node = NavigationNode()

    try:
        rclpy.spin(navigation_node)
    except KeyboardInterrupt:
        pass
    finally:
        navigation_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
