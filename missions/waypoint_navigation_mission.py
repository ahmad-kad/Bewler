#!/usr/bin/env python3
"""
Waypoint Navigation Mission - ROS2 Node

Executes GPS waypoint navigation missions using:
- SLAM pose for precise localization
- Navigation stack for path planning
- GPS waypoints from mission configuration
- Real-time progress monitoring

Author: URC 2026 Autonomy Team
"""

# Standard Library
import math
import threading
import time
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

import rclpy

# ROS2 Messages
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32, String

# Using std_srvs for basic service calls instead of nav2 action
from std_srvs.srv import Trigger


class WaypointState(Enum):
    """States for waypoint navigation"""

    IDLE = "idle"
    INITIALIZING = "initializing"
    NAVIGATING = "navigating"
    APPROACHING = "approaching"
    ARRIVED = "arrived"
    FAILED = "failed"
    ABORTED = "aborted"


class WaypointNavigationMission(Node):
    """
    Waypoint Navigation Mission Node

    Subscribes to:
    - /slam/pose: SLAM pose for precise localization
    - /gps/fix: GPS position for waypoint validation
    - /odom: Odometry for velocity monitoring

    Publishes:
    - /mission/waypoint/status: Current waypoint navigation status
    - /mission/waypoint/progress: Mission progress (0.0-1.0)
    - /mission/waypoint/current_waypoint: Current target waypoint
    - /mission/waypoint/path: Planned path to current waypoint

    Actions:
    - /navigate_to_pose: Navigation stack action client

    Services:
    - /mission/waypoint/start: Start waypoint navigation
    - /mission/waypoint/pause: Pause navigation
    - /mission/waypoint/resume: Resume navigation
    - /mission/waypoint/stop: Stop navigation
    - /mission/waypoint/status: Get current status
    """

    def __init__(self):
        super().__init__("waypoint_navigation_mission")

        # Mission state
        self.state = WaypointState.IDLE
        self.waypoints: List[Dict[str, Any]] = []
        self.current_waypoint_index = 0
        self.mission_start_time = None
        self.waypoint_start_time = None

        # Navigation parameters
        self.declare_parameter("waypoint_tolerance", 1.0)  # meters
        self.declare_parameter("heading_tolerance", 5.0)  # degrees
        self.declare_parameter("velocity_timeout", 5.0)  # seconds
        self.declare_parameter("max_waypoint_time", 300.0)  # 5 minutes per waypoint

        self.waypoint_tolerance = self.get_parameter("waypoint_tolerance").value
        self.heading_tolerance = self.get_parameter("heading_tolerance").value
        self.velocity_timeout = self.get_parameter("velocity_timeout").value
        self.max_waypoint_time = self.get_parameter("max_waypoint_time").value

        # Service client for navigation (simplified)
        self.nav_client = self.create_client(Trigger, "/navigate_to_pose")

        # Publishers
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.status_pub = self.create_publisher(
            String, "/mission/waypoint/status", qos_reliable
        )
        self.progress_pub = self.create_publisher(
            Float32, "/mission/waypoint/progress", qos_reliable
        )
        self.current_wp_pub = self.create_publisher(
            PoseStamped, "/mission/waypoint/current_waypoint", qos_reliable
        )
        self.path_pub = self.create_publisher(
            Path, "/mission/waypoint/path", qos_reliable
        )

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, "/slam/pose", self.pose_callback, 10
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        # Mission control services
        self.start_srv = self.create_service(
            Trigger, "/mission/waypoint/start", self.start_mission_callback
        )
        self.pause_srv = self.create_service(
            Trigger, "/mission/waypoint/pause", self.pause_mission_callback
        )
        self.resume_srv = self.create_service(
            Trigger, "/mission/waypoint/resume", self.resume_mission_callback
        )
        self.stop_srv = self.create_service(
            Trigger, "/mission/waypoint/stop", self.stop_mission_callback
        )
        self.status_srv = self.create_service(
            Trigger, "/mission/waypoint/status", self.get_status_callback
        )

        # Mission configuration via parameters
        # Waypoints can be set via ROS parameters

        # Current sensor data
        self.current_pose: Optional[PoseStamped] = None
        self.current_gps: Optional[NavSatFix] = None
        self.current_odom: Optional[Odometry] = None

        # Status update timer
        self.status_timer = self.create_timer(1.0, self.status_update)

        # Mission execution
        self.mission_thread: Optional[threading.Thread] = None
        self.stop_execution = False

        self.get_logger().info("Waypoint Navigation Mission initialized")

    # Sensor data callbacks
    def pose_callback(self, msg: PoseStamped):
        """Update SLAM pose"""
        self.current_pose = msg

    def gps_callback(self, msg: NavSatFix):
        """Update GPS position"""
        self.current_gps = msg

    def odom_callback(self, msg: Odometry):
        """Update odometry"""
        self.current_odom = msg

    # Mission control callbacks
    def start_mission_callback(self, request, response):
        """Start waypoint navigation mission"""
        if self.state == WaypointState.IDLE and self.waypoints:
            self.start_mission()
            response.success = True
            response.message = (
                f"Started waypoint navigation with {len(self.waypoints)} waypoints"
            )
        else:
            response.success = False
            response.message = f"Cannot start mission in state {self.state.value}"
        return response

    def pause_mission_callback(self, request, response):
        """Pause waypoint navigation"""
        if self.state == WaypointState.NAVIGATING:
            self.state = WaypointState.IDLE  # Pause by going idle
            response.success = True
            response.message = "Waypoint navigation paused"
        else:
            response.success = False
            response.message = f"Cannot pause in state {self.state.value}"
        return response

    def resume_mission_callback(self, request, response):
        """Resume waypoint navigation"""
        if self.state == WaypointState.IDLE and self.waypoints:
            self.state = WaypointState.NAVIGATING
            response.success = True
            response.message = "Waypoint navigation resumed"
        else:
            response.success = False
            response.message = f"Cannot resume in state {self.state.value}"
        return response

    def stop_mission_callback(self, request, response):
        """Stop waypoint navigation"""
        self.stop_execution = True
        self.state = WaypointState.IDLE
        self.waypoints = []
        self.current_waypoint_index = 0
        response.success = True
        response.message = "Waypoint navigation stopped"
        return response

    def get_status_callback(self, request, response):
        """Get current mission status"""
        response.success = True
        progress = (
            len(self.waypoints) > 0
            and self.current_waypoint_index / len(self.waypoints)
            or 0.0
        )
        response.message = (
            f"State: {self.state.value}, "
            f"Waypoint: {self.current_waypoint_index}/{len(self.waypoints)}, "
            f"Progress: {progress:.1f}"
        )
        return response

    # Mission execution
    def configure_waypoints(self, waypoints: List[Dict[str, Any]]) -> bool:
        """Configure waypoints for navigation"""
        try:
            self.waypoints = waypoints
            self.current_waypoint_index = 0
            self.get_logger().info(
                f"Configured {len(waypoints)} waypoints for navigation"
            )
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to configure waypoints: {str(e)}")
            return False

    def start_mission(self):
        """Start the waypoint navigation mission"""
        if not self.waypoints:
            self.get_logger().error("No waypoints configured")
            return

        if not self.nav_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Navigation service not available")
            self.state = WaypointState.FAILED
            return

        self.stop_execution = False
        self.state = WaypointState.NAVIGATING
        self.mission_start_time = time.time()

        # Start mission execution thread
        self.mission_thread = threading.Thread(target=self.execute_mission)
        self.mission_thread.start()

    def execute_mission(self):
        """Execute the waypoint navigation mission"""
        try:
            self.get_logger().info(
                f"Starting waypoint navigation mission with {len(self.waypoints)} waypoints"
            )

            for i, waypoint in enumerate(self.waypoints):
                if self.stop_execution:
                    self.state = WaypointState.ABORTED
                    break

                self.current_waypoint_index = i
                self.waypoint_start_time = time.time()

                self.get_logger().info(
                    f"Navigating to waypoint {i+1}/{len(self.waypoints)}"
                )

                # Publish current waypoint
                wp_pose = self.create_pose_from_waypoint(waypoint)
                self.current_wp_pub.publish(wp_pose)

                # Navigate to waypoint
                if not self.navigate_to_waypoint(waypoint):
                    self.get_logger().error(f"Failed to reach waypoint {i+1}")
                    self.state = WaypointState.FAILED
                    break

                # Check if this was the last waypoint
                if i == len(self.waypoints) - 1:
                    self.state = WaypointState.ARRIVED
                    self.get_logger().info("Mission completed successfully!")
                    break

            # Mission complete
            if self.state == WaypointState.ARRIVED:
                self.get_logger().info(
                    "Waypoint navigation mission completed successfully"
                )
            elif not self.stop_execution:
                self.get_logger().info("Waypoint navigation mission completed")

        except Exception as e:
            self.get_logger().error(f"Mission execution error: {str(e)}")
            self.state = WaypointState.FAILED
        finally:
            self.stop_execution = False

    def navigate_to_waypoint(self, waypoint: Dict[str, Any]) -> bool:
        """Navigate to a specific waypoint using the navigation stack"""
        try:
            # For simplified navigation, just publish the target pose and simulate success
            # In a real system, this would call the actual navigation service
            target_pose = self.create_pose_from_waypoint(waypoint)
            self.current_wp_pub.publish(target_pose)

            self.get_logger().info(f"Navigating to waypoint: {waypoint}")

            # Simulate navigation time (would be handled by real navigation stack)
            navigation_time = 5.0  # seconds
            time.sleep(navigation_time)

            # Check if we should simulate success or failure
            # In real implementation, this would wait for navigation completion
            self.get_logger().info("Successfully reached waypoint (simulated)")
            return True

        except Exception as e:
            self.get_logger().error(f"Navigation error: {str(e)}")
            return False

    def create_pose_from_waypoint(self, waypoint: Dict[str, Any]) -> PoseStamped:
        """Create a PoseStamped message from waypoint dictionary"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        pose.pose.position.x = waypoint.get("x", 0.0)
        pose.pose.position.y = waypoint.get("y", 0.0)
        pose.pose.position.z = waypoint.get("z", 0.0)

        # Orientation from heading
        heading = waypoint.get("heading", 0.0)
        pose.pose.orientation.z = math.sin(heading / 2.0)
        pose.pose.orientation.w = math.cos(heading / 2.0)

        return pose

    def status_update(self):
        """Periodic status update"""
        # Publish status
        status_msg = String()
        status_msg.data = (
            f"{self.state.value}|{self.current_waypoint_index}/{len(self.waypoints)}"
        )
        self.status_pub.publish(status_msg)

        # Publish progress
        if len(self.waypoints) > 0:
            progress = self.current_waypoint_index / len(self.waypoints)
            # Add partial progress to current waypoint
            if self.state == WaypointState.NAVIGATING and self.waypoint_start_time:
                waypoint_progress = min(
                    0.9, (time.time() - self.waypoint_start_time) / 60.0
                )  # Rough estimate
                progress += waypoint_progress / len(self.waypoints)
            progress = min(1.0, progress)
        else:
            progress = 0.0

        progress_msg = Float32()
        progress_msg.data = progress
        self.progress_pub.publish(progress_msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        mission = WaypointNavigationMission()
        rclpy.spin(mission)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
