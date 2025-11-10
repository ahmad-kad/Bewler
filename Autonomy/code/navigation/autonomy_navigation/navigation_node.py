"""
Main navigation controller for URC 2026 rover.

Coordinates GNSS waypoint navigation, terrain-adaptive path planning,
AR tag precision approaches, obstacle avoidance, and mission progress tracking.
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from autonomy_interfaces.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .gnss_processor import GNSSProcessor
from .motion_controller import MotionController
from .path_planner import PathPlanner
from .terrain_classifier import TerrainClassifier


class NavigationState(Enum):
    """Navigation system states."""
    IDLE = "idle"
    PLANNING = "planning"
    NAVIGATING = "navigating"
    AVOIDING_OBSTACLE = "avoiding_obstacle"
    PRECISION_APPROACH = "precision_approach"
    ARRIVED = "arrived"
    ERROR = "error"


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


class NavigationNode(Node):
    """
    Main navigation controller coordinating all navigation subsystems.

    Integrates GNSS waypoint navigation, terrain-adaptive path planning,
    AR tag precision approaches, obstacle avoidance, and mission tracking.
    """

    def __init__(self) -> None:
        """Initialize navigation node with all subsystems."""
        super().__init__('navigation_node')

        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_rate', 10.0),                    # Hz
                ('waypoint_tolerance', 2.0),              # meters
                ('obstacle_avoidance_distance', 5.0),    # meters
                ('max_linear_velocity', 1.0),             # m/s
                ('max_angular_velocity', 1.0),            # rad/s
                ('goal_timeout', 300.0),                  # seconds
                ('terrain_adaptation_enabled', True),
                ('precision_navigation_enabled', True),
            ]
        )

        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.obstacle_avoidance_distance = self.get_parameter('obstacle_avoidance_distance').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.terrain_adaptation_enabled = self.get_parameter('terrain_adaptation_enabled').value
        self.precision_navigation_enabled = self.get_parameter('precision_navigation_enabled').value

        # State variables
        self.current_state = NavigationState.IDLE
        self.current_goal: Optional[NavigationGoal] = None
        self.waypoints: List[Waypoint] = []
        self.current_waypoint_index = 0
        self.goal_start_time: Optional[float] = None

        # Initialize subsystems
        self.gnss_processor = GNSSProcessor()
        self.path_planner = PathPlanner()
        self.terrain_classifier = TerrainClassifier()
        self.motion_controller = MotionController(
            max_linear_velocity=self.max_linear_velocity,
            max_angular_velocity=self.max_angular_velocity
        )

        # Publishers
        self.status_publisher = self.create_publisher(String, 'navigation/status', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.current_waypoint_publisher = self.create_publisher(
            PoseStamped, 'navigation/current_waypoint', 10)
        self.waypoint_reached_publisher = self.create_publisher(
            String, 'navigation/waypoint_reached', 10)

        # Subscribers
        self.gnss_subscription = self.create_subscription(
            NavSatFix, 'gnss/fix', self.gnss_callback, 10)
        self.imu_subscription = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10)
        self.emergency_stop_subscription = self.create_subscription(
            Bool, 'emergency_stop', self.emergency_stop_callback, 10)
        self.waypoint_goal_subscription = self.create_subscription(
            PoseStamped, 'waypoint_goal', self.waypoint_goal_callback, 10)

        # Services
        self.navigate_to_waypoint_service = self.create_service(
            Trigger, 'navigation/navigate_to_waypoint', self.navigate_to_waypoint_callback)
        self.stop_navigation_service = self.create_service(
            Trigger, 'navigation/stop', self.stop_navigation_callback)
        self.get_current_waypoint_service = self.create_service(
            Trigger, 'navigation/get_current_waypoint', self.get_current_waypoint_callback)

        # Action server for navigation goals
        self.navigate_to_pose_action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            self.navigate_to_pose_callback
        )

        # Timers
        self.control_timer = self.create_timer(1.0 / self.update_rate, self.control_loop)
        self.status_timer = self.create_timer(1.0, self.status_callback)

        # Current sensor data
        self.current_position: Optional[Tuple[float, float, float]] = None
        self.current_heading: float = 0.0
        self.last_gnss_time: Optional[float] = None

        self.get_logger().info('Navigation node initialized')

    def gnss_callback(self, msg: NavSatFix):
        """Handle GNSS position updates"""
        self.current_position = (msg.latitude, msg.longitude, msg.altitude)
        self.last_gnss_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Process GNSS data
        self.gnss_processor.process_gnss_data(msg)

    def imu_callback(self, msg: Imu):
        """Handle IMU orientation updates"""
        # Extract heading from IMU quaternion
        # Simplified - in practice would use proper quaternion to euler conversion
        self.current_heading = self.extract_heading_from_imu(msg)

    def emergency_stop_callback(self, msg: Bool):
        """Handle emergency stop commands"""
        if msg.data:
            self.current_state = NavigationState.IDLE
            self.stop_motion()
            self.get_logger().warn('Emergency stop received - navigation halted')

    def waypoint_goal_callback(self, msg: PoseStamped):
        """Handle new waypoint goals from state management"""
        # Convert pose to waypoint
        waypoint = Waypoint(
            latitude=0.0,  # TODO: Convert from local coordinates to GPS
            longitude=0.0,
            altitude=0.0,
            name=f"waypoint_{len(self.waypoints)}",
            tolerance=self.waypoint_tolerance
        )

        # For now, store the pose directly and convert when needed
        waypoint.local_pose = msg.pose

        self.waypoints.append(waypoint)
        self.current_waypoint_index = len(self.waypoints) - 1

        self.get_logger().info(f'Received new waypoint goal: {waypoint.name}')
        self.start_navigation_to_current_waypoint()

    def start_navigation_to_current_waypoint(self):
        """Start navigation to the current waypoint"""
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().warn('No valid waypoint to navigate to')
            return

        waypoint = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f'Starting navigation to {waypoint.name}')

        # Set navigation state
        self.current_state = NavigationState.NAVIGATING

        # Publish current waypoint for monitoring
        self.publish_current_waypoint()

        # Start the navigation execution
        self.execute_navigation()

    def navigate_to_waypoint_callback(self, request, response):
        """Service callback to navigate to next waypoint"""
        if not self.waypoints:
            response.success = False
            response.message = "No waypoints available"
            return response

        if self.current_waypoint_index >= len(self.waypoints):
            response.success = False
            response.message = "All waypoints completed"
            return response

        waypoint = self.waypoints[self.current_waypoint_index]
        goal = NavigationGoal(waypoint=waypoint)

        success = self.set_navigation_goal(goal)
        response.success = success
        response.message = f"Navigation to waypoint {waypoint.name}" if success else "Failed to set navigation goal"

        return response

    def stop_navigation_callback(self, request, response):
        """Service callback to stop navigation"""
        self.current_state = NavigationState.IDLE
        self.stop_motion()
        self.current_goal = None
        self.goal_start_time = None

        response.success = True
        response.message = "Navigation stopped"
        return response

    def get_current_waypoint_callback(self, request, response):
        """Service callback to get current waypoint info"""
        if self.current_goal:
            waypoint = self.current_goal.waypoint
            response.success = True
            response.message = f"Current waypoint: {waypoint.name} ({waypoint.latitude}, {waypoint.longitude})"
        else:
            response.success = False
            response.message = "No active waypoint"

        return response

    def navigate_to_pose_callback(self, goal_handle):
        """Action server callback for navigation goals"""
        self.get_logger().info('Received navigation goal')

        # Extract goal pose and parameters
        target_pose = goal_handle.request.target_pose
        tolerance = goal_handle.request.tolerance
        timeout = goal_handle.request.timeout

        # Convert to waypoint (simplified conversion - would need proper lat/lon conversion)
        waypoint = Waypoint(
            latitude=target_pose.pose.position.x,  # This would need proper conversion
            longitude=target_pose.pose.position.y,
            altitude=target_pose.pose.position.z,
            precision_required=tolerance < 1.0  # Consider precision approach if tight tolerance
        )

        nav_goal = NavigationGoal(
            waypoint=waypoint,
            approach_tolerance=tolerance
        )

        success = self.set_navigation_goal(nav_goal)

        if not success:
            goal_handle.abort()
            result = NavigateToPose.Result()
            result.success = False
            result.message = "Failed to set navigation goal"
            return result

        # Monitor progress until completion or timeout
        start_time = self.get_clock().now().seconds_nanoseconds()[0]
        rate = self.create_rate(5.0)  # 5 Hz feedback

        while rclpy.ok():
            # Check for timeout
            if timeout > 0.0 and (self.get_clock().now().seconds_nanoseconds()[0] - start_time) > timeout:
                goal_handle.abort()
                result = NavigateToPose.Result()
                result.success = False
                result.message = "Navigation timeout"
                return result

            # Check if goal completed
            if self.current_state == NavigationState.ARRIVED:
                goal_handle.succeed()
                result = NavigateToPose.Result()
                result.success = True
                result.message = "Successfully reached goal"
                result.final_pose = target_pose
                result.total_distance_traveled = 0.0  # TODO: Calculate actual distance
                result.total_time = self.get_clock().now().seconds_nanoseconds()[0] - start_time
                return result

            # Check if navigation failed
            if self.current_state == NavigationState.ERROR:
                goal_handle.abort()
                result = NavigateToPose.Result()
                result.success = False
                result.message = "Navigation failed"
                return result

            # Send feedback
            feedback = NavigateToPose.Feedback()
            if self.current_goal and self.current_position:
                distance, _ = self.calculate_distance_bearing(
                    self.current_position,
                    (self.current_goal.waypoint.latitude,
                     self.current_goal.waypoint.longitude,
                     self.current_goal.waypoint.altitude)
                )
                feedback.distance_to_goal = distance
                feedback.estimated_time_remaining = distance / max(self.max_linear_velocity * 0.5, 0.1)  # Rough estimate
                feedback.navigation_state = self.current_state.value

                current_pose_msg = PoseStamped()
                current_pose_msg.header.stamp = self.get_clock().now().to_msg()
                current_pose_msg.header.frame_id = 'map'
                current_pose_msg.pose.position.x = self.current_position[0]
                current_pose_msg.pose.position.y = self.current_position[1]
                current_pose_msg.pose.position.z = self.current_position[2]
                feedback.current_pose = current_pose_msg

                goal_handle.publish_feedback(feedback)

            rate.sleep()

        # Should not reach here, but just in case
        goal_handle.abort()
        result = NavigateToPose.Result()
        result.success = False
        result.message = "Navigation interrupted"
        return result

    def set_navigation_goal(self, goal: NavigationGoal) -> bool:
        """Set a new navigation goal"""
        if not self.current_position:
            self.get_logger().error('No current position available')
            return False

        self.current_goal = goal
        self.current_state = NavigationState.PLANNING
        self.goal_start_time = self.get_clock().now().seconds_nanoseconds()[0]

        self.get_logger().info(f'Set navigation goal: {goal.waypoint.name}')
        return True

    def control_loop(self):
        """Main navigation control loop"""
        if self.current_state == NavigationState.IDLE:
            return

        if not self.current_position or not self.current_goal:
            self.current_state = NavigationState.ERROR
            return

        # Check for timeout
        if self.check_goal_timeout():
            self.current_state = NavigationState.ERROR
            self.get_logger().error('Navigation goal timeout')
            return

        if self.current_state == NavigationState.PLANNING:
            self.plan_navigation_path()

        elif self.current_state == NavigationState.NAVIGATING:
            # Check for obstacles first
            if self.detect_obstacles():
                self.current_state = NavigationState.AVOIDING_OBSTACLE
                self.get_logger().warn('Obstacle detected - switching to avoidance mode')
            else:
                self.execute_navigation()

        elif self.current_state == NavigationState.AVOIDING_OBSTACLE:
            self.execute_obstacle_avoidance()

        elif self.current_state == NavigationState.PRECISION_APPROACH:
            self.execute_precision_approach()

        # Publish current waypoint
        self.publish_current_waypoint()

    def plan_navigation_path(self):
        """Plan path to current goal"""
        if not self.current_goal:
            return

        # Use path planner to generate path
        start_pos = self.current_position
        goal_pos = (self.current_goal.waypoint.latitude,
                   self.current_goal.waypoint.longitude,
                   self.current_goal.waypoint.altitude)

        path = self.path_planner.plan_path(start_pos, goal_pos)

        if path:
            self.current_state = NavigationState.NAVIGATING
            self.get_logger().info('Path planning successful')
        else:
            self.current_state = NavigationState.ERROR
            self.get_logger().error('Path planning failed')

    def execute_navigation(self):
        """Execute waypoint navigation"""
        if not self.current_goal or not self.current_position:
            return

        # Calculate distance and bearing to goal
        distance, bearing = self.calculate_distance_bearing(
            self.current_position,
            (self.current_goal.waypoint.latitude,
             self.current_goal.waypoint.longitude,
             self.current_goal.waypoint.altitude)
        )

        # Check if arrived
        if distance < self.current_goal.approach_tolerance:
            if self.current_goal.waypoint.precision_required and self.precision_navigation_enabled:
                self.current_state = NavigationState.PRECISION_APPROACH
            else:
                self.waypoint_reached()
            return

        # Calculate velocity commands
        linear_vel, angular_vel = self.motion_controller.compute_velocity_commands(
            distance, bearing, self.current_heading
        )

        # Publish velocity commands
        self.publish_velocity_commands(linear_vel, angular_vel)

    def detect_obstacles(self) -> bool:
        """
        Detect obstacles in the robot's path.

        Returns:
            bool: True if obstacle detected that requires avoidance
        """
        # TODO: Implement actual obstacle detection using LIDAR/camera data
        # For now, simulate random obstacle detection (placeholder)

        # Simple simulation: 5% chance of detecting obstacle when moving
        import random
        obstacle_detected = random.random() < 0.05 and self.current_state == NavigationState.NAVIGATING

        if obstacle_detected:
            self.get_logger().warn('Simulated obstacle detected')

        return obstacle_detected

    def execute_obstacle_avoidance(self):
        """
        Execute obstacle avoidance maneuver.

        Simple strategy: Stop, turn, then continue.
        """
        # Simple avoidance: stop and turn 90 degrees
        if not hasattr(self, 'avoidance_start_time'):
            # Start avoidance maneuver
            self.avoidance_start_time = self.get_clock().now().seconds_nanoseconds()[0]
            self.stop_motion()
            self.get_logger().info('Starting obstacle avoidance maneuver')
            return

        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        avoidance_duration = current_time - self.avoidance_start_time

        # Turn for 2 seconds, then check if clear
        if avoidance_duration < 2.0:
            # Turn left (counter-clockwise)
            self.publish_velocity_commands(0.0, 0.5)  # Rotate in place
        elif avoidance_duration < 4.0:
            # Move forward slowly
            self.publish_velocity_commands(0.3, 0.0)
        else:
            # Check if obstacle is cleared
            if not self.detect_obstacles():
                # Obstacle cleared, resume navigation
                self.current_state = NavigationState.NAVIGATING
                delattr(self, 'avoidance_start_time')
                self.get_logger().info('Obstacle avoidance completed, resuming navigation')
            else:
                # Still detecting obstacle, reset avoidance timer
                self.avoidance_start_time = current_time
                self.get_logger().warn('Obstacle still detected, continuing avoidance')

    def execute_precision_approach(self):
        """Execute precision approach to waypoint"""
        # TODO: Implement precision navigation using AR tags, visual servoing, etc.
        # For now, use regular navigation
        self.execute_navigation()

    def waypoint_reached(self):
        """Handle waypoint reached"""
        waypoint_name = self.current_goal.waypoint.name
        self.get_logger().info(f'Waypoint reached: {waypoint_name}')
        self.current_state = NavigationState.ARRIVED
        self.stop_motion()

        # Publish waypoint reached notification for state management
        reached_msg = String()
        reached_msg.data = waypoint_name
        self.waypoint_reached_publisher.publish(reached_msg)

        # Move to next waypoint if available
        self.current_waypoint_index += 1
        if self.current_waypoint_index < len(self.waypoints):
            # Auto-advance to next waypoint
            next_waypoint = self.waypoints[self.current_waypoint_index]
            next_goal = NavigationGoal(waypoint=next_waypoint)
            self.set_navigation_goal(next_goal)
        else:
            # Mission complete - all waypoints reached
            self.current_state = NavigationState.IDLE
            self.get_logger().info('All waypoints completed - navigation mission finished')

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
            pose.header.frame_id = 'map'
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

    def calculate_distance_bearing(self, pos1: Tuple[float, float, float],
                                 pos2: Tuple[float, float, float]) -> Tuple[float, float]:
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
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = 6371000 * c  # Earth radius in meters

        # Bearing calculation
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
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

    def add_waypoints(self, waypoints: List[Waypoint]):
        """Add waypoints to navigation queue"""
        self.waypoints.extend(waypoints)
        self.get_logger().info(f'Added {len(waypoints)} waypoints')

    def clear_waypoints(self):
        """Clear all waypoints"""
        self.waypoints.clear()
        self.current_waypoint_index = 0
        self.get_logger().info('Cleared all waypoints')


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


if __name__ == '__main__':
    main()
