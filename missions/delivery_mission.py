#!/usr/bin/env python3
"""
Delivery Mission - ROS2 Node

Executes delivery missions with pickup and dropoff:
- Navigate to pickup location
- Perform pickup operation (simulated)
- Navigate to delivery location
- Perform delivery operation
- Return to base or continue

Author: URC 2026 Autonomy Team
"""

# Standard Library
import time
from enum import Enum
from typing import Any, Dict, Optional

import rclpy

# ROS2 Messages
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

# Using std_srvs for basic service calls
from std_srvs.srv import Trigger


class DeliveryState(Enum):
    """States for delivery mission"""

    IDLE = "idle"
    NAVIGATING_PICKUP = "navigating_pickup"
    AT_PICKUP = "at_pickup"
    PICKING_UP = "picking_up"
    NAVIGATING_DELIVERY = "navigating_delivery"
    AT_DELIVERY = "at_delivery"
    DELIVERING = "delivering"
    COMPLETED = "completed"
    FAILED = "failed"


class DeliveryMission(Node):
    """
    Delivery Mission Node

    Subscribes to:
    - /mission/delivery/config: Delivery mission configuration

    Publishes:
    - /mission/delivery/status: Mission status
    - /mission/delivery/progress: Mission progress (0.0-1.0)
    - /mission/delivery/current_location: Current target location

    Actions:
    - /navigate_to_pose: Navigation stack

    Services:
    - /mission/delivery/start: Start delivery mission
    - /mission/delivery/stop: Stop delivery mission
    - /mission/delivery/status: Get current status
    """

    def __init__(self):
        super().__init__("delivery_mission")

        # Mission state
        self.state = DeliveryState.IDLE
        self.pickup_location: Optional[Dict[str, Any]] = None
        self.delivery_location: Optional[Dict[str, Any]] = None
        self.mission_start_time = None
        self.current_location = None

        # Mission parameters
        self.declare_parameter("pickup_timeout", 30.0)  # seconds
        self.declare_parameter("delivery_timeout", 30.0)  # seconds
        self.declare_parameter("navigation_timeout", 300.0)  # 5 minutes

        self.pickup_timeout = self.get_parameter("pickup_timeout").value
        self.delivery_timeout = self.get_parameter("delivery_timeout").value
        self.navigation_timeout = self.get_parameter("navigation_timeout").value

        # Service client for navigation (simplified)
        self.nav_client = self.create_client(Trigger, "/navigate_to_pose")

        # Publishers
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.status_pub = self.create_publisher(
            String, "/mission/delivery/status", qos_reliable
        )
        self.progress_pub = self.create_publisher(
            Float32, "/mission/delivery/progress", qos_reliable
        )
        self.location_pub = self.create_publisher(
            String, "/mission/delivery/current_location", qos_reliable
        )
        self.pickup_complete_pub = self.create_publisher(
            Bool, "/mission/delivery/pickup_complete", qos_reliable
        )
        self.delivery_complete_pub = self.create_publisher(
            Bool, "/mission/delivery/delivery_complete", qos_reliable
        )

        # Mission control services
        self.start_srv = self.create_service(
            Trigger, "/mission/delivery/start", self.start_mission_callback
        )
        self.stop_srv = self.create_service(
            Trigger, "/mission/delivery/stop", self.stop_mission_callback
        )
        self.status_srv = self.create_service(
            Trigger, "/mission/delivery/status", self.get_status_callback
        )

        # Mission configuration via parameters

        # Status update timer
        self.status_timer = self.create_timer(1.0, self.status_update)

        # Mission execution
        self.mission_thread: Optional[threading.Thread] = None
        self.stop_execution = False

        self.get_logger().info("Delivery Mission initialized")

    # Mission control callbacks
    def start_mission_callback(self, request, response):
        """Start delivery mission"""
        if self.state == DeliveryState.IDLE:
            if self.configure_delivery_mission():
                self.start_mission()
                response.success = True
                response.message = "Started delivery mission"
            else:
                response.success = False
                response.message = "Failed to configure delivery mission"
        else:
            response.success = False
            response.message = f"Cannot start mission in state {self.state.value}"
        return response

    def stop_mission_callback(self, request, response):
        """Stop delivery mission"""
        self.stop_execution = True
        self.state = DeliveryState.IDLE
        response.success = True
        response.message = "Delivery mission stopped"
        return response

    def get_status_callback(self, request, response):
        """Get current mission status"""
        response.success = True
        progress = self.get_mission_progress()
        response.message = f"State: {self.state.value}, Progress: {progress:.1f}"
        return response

    # Mission configuration
    def configure_delivery_mission(self) -> bool:
        """Configure delivery mission parameters"""
        try:
            # Get mission configuration from parameter server
            pickup_x = self.declare_parameter("pickup_x", 5.0).value
            pickup_y = self.declare_parameter("pickup_y", 5.0).value
            delivery_x = self.declare_parameter("delivery_x", 15.0).value
            delivery_y = self.declare_parameter("delivery_y", 15.0).value

            self.pickup_location = {"x": pickup_x, "y": pickup_y, "heading": 0.0}

            self.delivery_location = {"x": delivery_x, "y": delivery_y, "heading": 0.0}

            self.get_logger().info(
                f"Configured delivery: pickup({pickup_x}, {pickup_y}) -> delivery({delivery_x}, {delivery_y})"
            )
            return True

        except Exception as e:
            self.get_logger().error(f"Mission configuration error: {str(e)}")
            return False

    # Mission execution
    def start_mission(self):
        """Start the delivery mission"""
        if not self.nav_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Navigation service not available")
            self.state = DeliveryState.FAILED
            return

        if not self.pickup_location or not self.delivery_location:
            self.get_logger().error("Mission locations not configured")
            self.state = DeliveryState.FAILED
            return

        self.stop_execution = False
        self.state = DeliveryState.NAVIGATING_PICKUP
        self.mission_start_time = time.time()

        # Start mission execution thread
        self.mission_thread = threading.Thread(target=self.execute_mission)
        self.mission_thread.start()

    def execute_mission(self):
        """Execute the delivery mission"""
        try:
            self.get_logger().info("Starting delivery mission")

            # Phase 1: Navigate to pickup location
            if not self.navigate_to_location(
                self.pickup_location, DeliveryState.AT_PICKUP
            ):
                self.state = DeliveryState.FAILED
                return

            # Phase 2: Perform pickup
            if not self.perform_pickup():
                self.state = DeliveryState.FAILED
                return

            # Phase 3: Navigate to delivery location
            self.state = DeliveryState.NAVIGATING_DELIVERY
            if not self.navigate_to_location(
                self.delivery_location, DeliveryState.AT_DELIVERY
            ):
                self.state = DeliveryState.FAILED
                return

            # Phase 4: Perform delivery
            if not self.perform_delivery():
                self.state = DeliveryState.FAILED
                return

            # Mission complete
            self.state = DeliveryState.COMPLETED
            self.get_logger().info("Delivery mission completed successfully!")

        except Exception as e:
            self.get_logger().error(f"Mission execution error: {str(e)}")
            self.state = DeliveryState.FAILED
        finally:
            self.stop_execution = False

    def navigate_to_location(
        self, location: Dict[str, Any], target_state: DeliveryState
    ) -> bool:
        """Navigate to a specific location"""
        try:
            self.get_logger().info(f"Navigating to location: {location}")

            # Update current location
            self.current_location = (
                location["name"]
                if "name" in location
                else f"({location['x']:.1f}, {location['y']:.1f})"
            )
            self.location_pub.publish(String(data=self.current_location))

            # Navigate to location (simplified)
            self.get_logger().info(f"Navigating to {self.current_location}...")

            # Simulate navigation (in real system, would call navigation service)
            time.sleep(4.0)  # Simulate navigation time

            self.state = target_state
            self.get_logger().info(
                f"Successfully reached {self.current_location} (simulated)"
            )
            return True

        except Exception as e:
            self.get_logger().error(f"Navigation error: {str(e)}")
            return False

    def perform_pickup(self) -> bool:
        """Perform pickup operation"""
        self.get_logger().info("Performing pickup operation...")
        self.state = DeliveryState.PICKING_UP

        try:
            # Simulate pickup operation
            start_time = time.time()
            while time.time() - start_time < self.pickup_timeout:
                if self.stop_execution:
                    return False

                # Check for pickup completion (simulated)
                # In real implementation, this would check sensors/actuators
                if time.time() - start_time > 5.0:  # 5 second pickup
                    self.pickup_complete_pub.publish(Bool(data=True))
                    self.get_logger().info("Pickup operation completed")
                    return True

                time.sleep(0.5)

            self.get_logger().error("Pickup operation timed out")
            return False

        except Exception as e:
            self.get_logger().error(f"Pickup error: {str(e)}")
            return False

    def perform_delivery(self) -> bool:
        """Perform delivery operation"""
        self.get_logger().info("Performing delivery operation...")
        self.state = DeliveryState.DELIVERING

        try:
            # Simulate delivery operation
            start_time = time.time()
            while time.time() - start_time < self.delivery_timeout:
                if self.stop_execution:
                    return False

                # Check for delivery completion (simulated)
                # In real implementation, this would check sensors/actuators
                if time.time() - start_time > 3.0:  # 3 second delivery
                    self.delivery_complete_pub.publish(Bool(data=True))
                    self.get_logger().info("Delivery operation completed")
                    return True

                time.sleep(0.5)

            self.get_logger().error("Delivery operation timed out")
            return False

        except Exception as e:
            self.get_logger().error(f"Delivery error: {str(e)}")
            return False

    def create_pose_from_location(self, location: Dict[str, Any]) -> PoseStamped:
        """Create a PoseStamped message from location dictionary"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        # Position
        pose.pose.position.x = location.get("x", 0.0)
        pose.pose.position.y = location.get("y", 0.0)
        pose.pose.position.z = location.get("z", 0.0)

        # Orientation from heading
        import math

        heading = location.get("heading", 0.0)
        pose.pose.orientation.z = math.sin(heading / 2.0)
        pose.pose.orientation.w = math.cos(heading / 2.0)

        return pose

    def get_mission_progress(self) -> float:
        """Get current mission progress (0.0-1.0)"""
        state_progress = {
            DeliveryState.IDLE: 0.0,
            DeliveryState.NAVIGATING_PICKUP: 0.1,
            DeliveryState.AT_PICKUP: 0.25,
            DeliveryState.PICKING_UP: 0.4,
            DeliveryState.NAVIGATING_DELIVERY: 0.6,
            DeliveryState.AT_DELIVERY: 0.8,
            DeliveryState.DELIVERING: 0.95,
            DeliveryState.COMPLETED: 1.0,
            DeliveryState.FAILED: 0.0,
        }
        return state_progress.get(self.state, 0.0)

    def status_update(self):
        """Periodic status update"""
        # Publish status
        status_msg = String()
        status_msg.data = self.state.value
        self.status_pub.publish(status_msg)

        # Publish progress
        progress_msg = Float32()
        progress_msg.data = self.get_mission_progress()
        self.progress_pub.publish(progress_msg)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        mission = DeliveryMission()
        rclpy.spin(mission)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
