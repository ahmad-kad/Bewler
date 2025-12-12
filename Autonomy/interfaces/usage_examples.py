#!/usr/bin/env python3
"""
ROS2 Interface Usage Examples for URC 2026 Autonomy System

This file demonstrates how to use the custom autonomy_interfaces
in your ROS2 nodes with clean input/output contracts.
"""

import rclpy
from autonomy_interfaces.action import NavigateToPose

# Import your custom interfaces (generated from .msg/.srv/.action files)
from autonomy_interfaces.msg import LedCommand, NavigationStatus, VisionDetection
from autonomy_interfaces.srv import GetSubsystemStatus, SwitchMode

# Standard ROS2 interfaces
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from std_msgs.msg import String


class StateManagementNode(Node):
    """Example: Clean interface usage in state management"""

    def __init__(self):
        super().__init__("state_management_node")

        # Publishers with clear contracts
        self.mission_status_pub = self.create_publisher(String, "mission_status", 10)

        # Services with specific request/response contracts
        self.switch_mode_service = self.create_service(SwitchMode, "switch_mode", self.switch_mode_callback)

        self.get_status_service = self.create_service(
            GetSubsystemStatus, "get_subsystem_status", self.get_subsystem_status_callback
        )

        # Subscribers to subsystem status
        self.navigation_status_sub = self.create_subscription(
            NavigationStatus, "navigation/status", self.navigation_status_callback, 10
        )

    def switch_mode_callback(self, request, response):
        """Clean service contract: specific input → specific output"""
        self.get_logger().info(f"Switching to mode: {request.requested_mode}")

        # Validate input
        valid_modes = ["autonomous", "teleoperation", "manual_override", "idle"]
        if request.requested_mode not in valid_modes:
            response.success = False
            response.message = f"Invalid mode. Must be one of: {valid_modes}"
            response.actual_mode = "unknown"
            response.transition_time = 0.0
            return response

        # Process request (simplified)
        # In real implementation: coordinate with all subsystems
        response.success = True
        response.message = f"Successfully switched to {request.requested_mode}"
        response.actual_mode = request.requested_mode
        response.transition_time = 2.5  # seconds

        return response

    def get_subsystem_status_callback(self, request, response):
        """Clean service contract: query → structured status"""
        subsystem = request.subsystem_name

        if subsystem == "all":
            # Return status for all subsystems
            response.success = True
            response.subsystem_names = ["navigation", "slam", "vision", "typing", "led"]
            response.subsystem_states = ["active", "active", "active", "idle", "active"]
            response.subsystem_health = [0.95, 0.88, 0.92, 0.0, 1.0]
            response.status_messages = [
                "Navigating to waypoint 3",
                "SLAM tracking stable",
                "Vision detecting AR tags",
                "Typing system idle",
                "LED showing ready status",
            ]
        else:
            # Query specific subsystem
            # In real implementation: check actual subsystem status
            response.success = True
            response.subsystem_names = [subsystem]
            response.subsystem_states = ["active"]
            response.subsystem_health = [0.9]
            response.status_messages = [f"{subsystem} is operational"]

        return response

    def navigation_status_callback(self, msg: NavigationStatus):
        """Clean subscriber contract: structured status updates"""
        self.get_logger().info(
            f"Navigation: {msg.state} | Progress: {msg.mission_progress:.1%} | "
            f"Waypoint: {msg.current_waypoint}/{msg.total_waypoints} | "
            f"Distance to goal: {msg.distance_to_goal:.2f}m"
        )

        # Use the structured data for decision making
        if msg.state == "error":
            self.get_logger().error(f"Navigation error: {msg.status_message}")
            # Could trigger emergency protocols

        elif msg.state == "arrived":
            self.get_logger().info("Navigation goal achieved!")
            # Could trigger next mission phase


class NavigationNode(Node):
    """Example: Publishing clean status interface"""

    def __init__(self):
        super().__init__("navigation_node")

        # Publisher with clear contract
        self.status_publisher = self.create_publisher(NavigationStatus, "navigation/status", 10)

        # Action server with Goal-Feedback-Result contract
        self.navigate_action_server = ActionServer(
            self, NavigateToPose, "navigate_to_pose", self.navigate_to_pose_callback
        )

        # Timer to publish status
        self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        """Publish structured navigation status"""
        msg = NavigationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.state = "navigating"
        msg.mission_progress = 0.65
        msg.current_waypoint = 2
        msg.total_waypoints = 5

        # Current pose (simplified)
        msg.current_pose.header = msg.header
        msg.current_pose.pose.position.x = 10.5
        msg.current_pose.pose.position.y = 8.2

        # Goal pose
        msg.goal_pose.header = msg.header
        msg.goal_pose.pose.position.x = 15.0
        msg.goal_pose.pose.position.y = 12.0

        msg.distance_to_goal = 5.8  # meters
        msg.speed = 0.8  # m/s
        msg.heading_error = 0.1  # radians
        msg.status_message = "Approaching target waypoint"

        self.status_publisher.publish(msg)

    def navigate_to_pose_callback(self, goal_handle: ServerGoalHandle):
        """Clean action contract: Goal → Feedback → Result"""
        goal = goal_handle.request

        self.get_logger().info(
            f"Navigating to pose: x={goal.target_pose.pose.position.x:.2f}, "
            f"y={goal.target_pose.pose.position.y:.2f}"
        )

        # Simulate navigation with feedback
        feedback_msg = NavigateToPose.Feedback()
        result_msg = NavigateToPose.Result()

        # Simplified navigation simulation
        for i in range(10):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result_msg.success = False
                result_msg.message = "Navigation canceled"
                return result_msg

            # Send feedback
            feedback_msg.distance_to_goal = 10.0 - i
            feedback_msg.estimated_time_remaining = feedback_msg.distance_to_goal / 0.5
            feedback_msg.navigation_state = "navigating"
            goal_handle.publish_feedback(feedback_msg)

            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.5))

        # Complete successfully
        goal_handle.succeed()
        result_msg.success = True
        result_msg.message = "Successfully reached goal"
        result_msg.final_pose = goal.target_pose
        result_msg.total_distance_traveled = 10.0
        result_msg.total_time = 5.0

        return result_msg


class VisionNode(Node):
    """Example: Publishing detection results with clean interface"""

    def __init__(self):
        super().__init__("vision_node")

        # Publisher for detections (could be array, but showing single for clarity)
        self.detection_publisher = self.create_publisher(VisionDetection, "vision/detections", 10)

        # LED command publisher for status signaling
        self.led_publisher = self.create_publisher(LedCommand, "led/command", 10)

        # Timer to simulate detections
        self.create_timer(2.0, self.publish_detection)

    def publish_detection(self):
        """Publish structured detection results"""
        detection = VisionDetection()
        detection.header.stamp = self.get_clock().now().to_msg()
        detection.header.frame_id = "camera_link"

        detection.class_name = "aruco_marker"
        detection.class_id = 42
        detection.confidence = 0.95

        # Object pose in camera frame
        detection.pose.header = detection.header
        detection.pose.pose.position.x = 0.5
        detection.pose.pose.position.y = 0.1
        detection.pose.pose.position.z = 1.2

        detection.size.x = 0.1  # width
        detection.size.y = 0.1  # height
        detection.size.z = 0.01  # depth

        detection.keypoints = [320.5, 240.8, 325.2, 245.1]  # corner points
        detection.detector_type = "aruco"
        detection.track_id = 1
        detection.age = 3.5

        self.detection_publisher.publish(detection)

        # Signal successful detection via LED
        led_cmd = LedCommand()
        led_cmd.header = detection.header
        led_cmd.status_code = 2  # running
        led_cmd.red = 0.0
        led_cmd.green = 1.0
        led_cmd.blue = 0.0
        led_cmd.pattern = "solid"
        led_cmd.priority = 0

        self.led_publisher.publish(led_cmd)


class LedNode(Node):
    """Example: Consuming LED commands with clean interface"""

    def __init__(self):
        super().__init__("led_node")

        # Subscriber for LED commands
        self.led_subscription = self.create_subscription(LedCommand, "led/command", self.led_command_callback, 10)

    def led_command_callback(self, msg: LedCommand):
        """Process structured LED commands"""
        self.get_logger().info(
            f"LED Command: code={msg.status_code}, "
            f"color=({msg.red:.1f},{msg.green:.1f},{msg.blue:.1f}), "
            f"pattern={msg.pattern}"
        )

        # In real implementation: control actual LED hardware
        # Map status codes to specific colors/patterns per competition rules

        if msg.status_code == 1:  # ready
            self.set_led_color(0.0, 1.0, 0.0)  # green
        elif msg.status_code == 2:  # running
            self.set_led_color(0.0, 0.0, 1.0)  # blue
        elif msg.status_code == 3:  # error
            self.set_led_color(1.0, 0.0, 0.0)  # red
            self.set_led_pattern("blinking")

    def set_led_color(self, r, g, b):
        """Set LED color (hardware interface)"""
        # GPIO control code would go here

    def set_led_pattern(self, pattern):
        """Set LED pattern (hardware interface)"""
        # Pattern control code would go here


def main(args=None):
    """Demonstrate the clean interface contracts"""
    rclpy.init(args=args)

    # Create nodes with clean interfaces
    state_node = StateManagementNode()
    nav_node = NavigationNode()
    vision_node = VisionNode()
    led_node = LedNode()

    try:
        rclpy.spin(state_node)  # Could use MultiThreadedExecutor for all nodes
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown
        state_node.destroy_node()
        nav_node.destroy_node()
        vision_node.destroy_node()
        led_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
