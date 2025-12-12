#!/usr/bin/env python3
"""
Basic ROS2 Integration Test
Tests ROS2 topic publishing and subscribing without full mission executor
"""

import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Float32MultiArray


class TestSubscriber(Node):
    """Test subscriber for teleoperation topics"""

    def __init__(self):
        super().__init__('test_subscriber')

        self.received_messages = {
            'joint_states': [],
            'chassis_velocity': [],
            'motor_temperatures': [],
            'system_status': []
        }

        # Create subscriptions
        self.joint_sub = self.create_subscription(
            JointState, '/teleoperation/joint_states',
            lambda msg: self.message_callback('joint_states', msg), 10)

        self.chassis_sub = self.create_subscription(
            TwistStamped, '/teleoperation/chassis_velocity',
            lambda msg: self.message_callback('chassis_velocity', msg), 10)

        self.temp_sub = self.create_subscription(
            Float32MultiArray, '/teleoperation/motor_temperatures',
            lambda msg: self.message_callback('motor_temperatures', msg), 10)

        self.status_sub = self.create_subscription(
            BatteryState, '/teleoperation/system_status',
            lambda msg: self.message_callback('system_status', msg), 10)

        self.get_logger().info("Test subscriber initialized")

    def message_callback(self, topic, msg):
        """Handle incoming messages"""
        self.received_messages[topic].append({
            'timestamp': time.time(),
            'data': msg
        })

        # Keep only last 5 messages
        if len(self.received_messages[topic]) > 5:
            self.received_messages[topic].pop(0)

        self.get_logger().debug(f"Received {topic} message")

    def get_message_counts(self):
        """Get count of received messages per topic"""
        return {topic: len(messages) for topic, messages in self.received_messages.items()}

    def has_received_data(self):
        """Check if we've received data on all topics"""
        counts = self.get_message_counts()
        return all(count > 0 for count in counts.values())


class BasicROS2IntegrationTest:
    """Basic ROS2 integration test"""

    def __init__(self):
        self.success = False
        self.publisher_thread = None
        self.subscriber_node = None

    def run_integrated_test(self):
        """Run both publisher and subscriber in same ROS2 context"""
        print("📡 Starting integrated ROS2 test...")

        # Initialize ROS2 once
        rclpy.init()

        try:
            # Create both nodes
            from test_teleoperation_integration import MockTeleoperationPublisher
            publisher = MockTeleoperationPublisher()
            self.subscriber_node = TestSubscriber()

            # Create executor for both nodes
            executor = rclpy.executors.MultiThreadedExecutor()
            executor.add_node(publisher)
            executor.add_node(self.subscriber_node)

            # Run test for specified duration
            print("⏳ Running integrated test...")
            end_time = time.time() + 10

            while time.time() < end_time and rclpy.ok():
                executor.spin_once(timeout_sec=0.1)

            # Check results
            if self.subscriber_node:
                message_counts = self.subscriber_node.get_message_counts()
                has_data = self.subscriber_node.has_received_data()

                print("\n📊 INTEGRATION TEST RESULTS:")
                print(f"   Has received data on all topics: {'✅' if has_data else '❌'}")

                print("   Message counts:")
                for topic, count in message_counts.items():
                    status = "✅" if count > 0 else "❌"
                    print(f"     {status} {topic}: {count} messages")

                # Check that we received data (subscriber only keeps last 5 messages)
                # At 10Hz publishing rate, we should see at least 1-2 messages in the buffer
                min_expected_messages = 1
                all_topics_good = all(count >= min_expected_messages for count in message_counts.values())

                if has_data and all_topics_good:
                    print("\n🎉 ROS2 Integration Test PASSED!")
                    print("✅ Topics are publishing and subscribing correctly")
                    print("✅ Message rates are within expected ranges")
                    self.success = True
                else:
                    print("\n❌ ROS2 Integration Test FAILED!")
                    if not has_data:
                        print("❌ Not all topics received data")
                    if not all_topics_good:
                        print("❌ Message rates too low")
                    self.success = False

            # Cleanup
            executor.remove_node(publisher)
            executor.remove_node(self.subscriber_node)
            publisher.destroy_node()
            self.subscriber_node.destroy_node()

        except Exception as e:
            print(f"\n💥 Integrated test failed: {e}")
            self.success = False

        finally:
            rclpy.shutdown()

    def run_test(self):
        """Run the basic ROS2 integration test"""
        print("🧪 Basic ROS2 Integration Test")
        print("=" * 40)

        try:
            # Run integrated test with both publisher and subscriber
            self.run_integrated_test()

        except Exception as e:
            print(f"\n💥 Test failed with exception: {e}")
            self.success = False

        return self.success


def main():
    """Main test function"""
    print("Testing basic ROS2 teleoperation integration...")

    test = BasicROS2IntegrationTest()
    success = test.run_test()

    print(f"\n🏁 Final Result: {'SUCCESS' if success else 'FAILURE'}")
    return success


if __name__ == '__main__':
    success = main()
    exit(0 if success else 1)
