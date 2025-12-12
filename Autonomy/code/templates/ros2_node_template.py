"""
URC 2026 Autonomy System - ROS 2 Node Template

This template provides a starting point for creating ROS 2 nodes in the autonomy system.
Replace placeholders with your specific implementation.

Author: [Your Name]
Date: [Date]
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# Import your message/service types
from std_srvs.srv import Trigger

# Constants
DEFAULT_UPDATE_RATE = 10.0  # Hz
NODE_NAME = "your_node_name"
NODE_NAMESPACE = ""


class YourNodeName(Node):
    """
    ROS 2 node for [brief description].

    [Detailed description of what this node does, its responsibilities,
    and how it fits into the overall autonomy system.]

    Publishers:
        [topic_name] ([msg_type]): [description]

    Subscribers:
        [topic_name] ([msg_type]): [description]

    Services:
        [service_name] ([srv_type]): [description]

    Parameters:
        [param_name] ([type]): [description] (default: [value])
    """

    def __init__(self):
        """Initialize the node."""
        super().__init__(NODE_NAME, namespace=NODE_NAMESPACE)

        # Declare parameters with defaults
        self.declare_parameters(
            namespace="",
            parameters=[
                ("update_rate", DEFAULT_UPDATE_RATE),
                ("param1", 1.0),
                ("param2", "default_value"),
                ("enable_feature", True),
            ],
        )

        # Get parameter values
        self.update_rate = self.get_parameter("update_rate").value
        self.param1 = self.get_parameter("param1").value
        self.param2 = self.get_parameter("param2").value
        self.enable_feature = self.get_parameter("enable_feature").value

        # Setup QoS profiles
        self._setup_qos_profiles()

        # Create publishers
        self._setup_publishers()

        # Create subscribers
        self._setup_subscribers()

        # Create services
        self._setup_services()

        # Create timers
        self._setup_timers()

        # Initialize state
        self._initialize_state()

        self.get_logger().info(f"{NODE_NAME} node initialized")

    def _setup_qos_profiles(self):
        """Setup QoS profiles for different message types."""
        self.qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.qos_control = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        self.qos_state = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

    def _setup_publishers(self):
        """Setup ROS 2 publishers."""
        # Example publishers - modify as needed
        self.example_pub = self.create_publisher(Twist, "cmd_vel", self.qos_control)

        # Add more publishers here
        # self.another_pub = self.create_publisher(
        #     PoseStamped, 'pose', self.qos_state
        # )

    def _setup_subscribers(self):
        """Setup ROS 2 subscribers."""
        # Example subscribers - modify as needed
        self.example_sub = self.create_subscription(Odometry, "odom", self.example_callback, self.qos_sensor)

        # Add more subscribers here
        # self.scan_sub = self.create_subscription(
        #     LaserScan, 'scan', self.scan_callback,
        #     self.qos_sensor
        # )

    def _setup_services(self):
        """Setup ROS 2 services."""
        # Example service - modify as needed
        self.example_service = self.create_service(Trigger, "example_service", self.example_service_callback)

    def _setup_timers(self):
        """Setup ROS 2 timers."""
        # Main control loop timer
        self.control_timer = self.create_timer(1.0 / self.update_rate, self.control_loop)

        # Optional: Additional timers for different tasks
        # self.health_check_timer = self.create_timer(
        #     5.0, self.health_check_callback
        # )

    def _initialize_state(self):
        """Initialize node state variables."""
        self.is_active = False
        self.last_update_time = time.time()
        self.error_count = 0

        # Initialize your specific state variables here
        # self.current_pose = None
        # self.target_pose = None
        # self.sensor_data = {}

    # Callback methods
    def example_callback(self, msg):
        """Handle example subscriber messages."""
        try:
            # Process the incoming message
            self.get_logger().debug(f"Received message: {type(msg).__name__}")

            # Update state
            self.last_update_time = time.time()

            # Process message data
            # self.process_message_data(msg)

        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f"Error in example_callback: {e}")

    def example_service_callback(self, request, response):
        """Handle example service calls."""
        try:
            # Process service request
            self.get_logger().info("Processing service request")

            # Perform service logic
            success = True
            message = "Service completed successfully"

            if not success:
                response.success = False
                response.message = "Service failed"
                return response

            response.success = True
            response.message = message
            return response

        except Exception as e:
            self.get_logger().error(f"Error in service callback: {e}")
            response.success = False
            response.message = f"Service error: {e}"
            return response

    def control_loop(self):
        """Main control loop executed at update rate."""
        try:
            current_time = time.time()

            # Check if node should be active
            if not self.is_active:
                return

            # Perform main processing logic
            # self.process_control_logic()

            # Publish results
            # self.publish_results()

            # Update timing
            self.last_update_time = current_time

        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f"Error in control loop: {e}")

    def health_check_callback(self):
        """Periodic health check."""
        # Check system health
        time_since_update = time.time() - self.last_update_time

        if time_since_update > 5.0:  # 5 second timeout
            self.get_logger().warn(f"No updates for {time_since_update:.1f} seconds")

        if self.error_count > 10:
            self.get_logger().error(f"High error count: {self.error_count}")

    # Utility methods
    def validate_input(self, data):
        """Validate input data."""
        if data is None:
            raise ValueError("Input data cannot be None")

        # Add your validation logic here
        return True

    def publish_status(self):
        """Publish current node status."""
        # Publish status messages if needed

    def cleanup(self):
        """Clean up resources before shutdown."""
        self.get_logger().info("Cleaning up resources")

        # Stop timers
        if hasattr(self, "control_timer"):
            self.control_timer.cancel()

        # Close connections, save data, etc.
        # self.save_state_to_file()
        # self.disconnect_from_hardware()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    node = None
    try:
        node = YourNodeName()
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Received shutdown signal")
    except Exception as e:
        if node:
            node.get_logger().fatal(f"Node failed: {e}")
        else:
            print(f"Failed to create node: {e}")
    finally:
        if node:
            node.cleanup()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
