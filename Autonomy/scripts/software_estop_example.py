#!/usr/bin/env python3
"""
Software ESTOP Example

Demonstrates how to trigger a software emergency stop that behaves
exactly like a hardware emergency stop button.
"""

import rclpy
from rclpy.node import Node
from autonomy_interfaces.srv import SoftwareEstop


class SoftwareEstopExample(Node):
    """Example node showing how to trigger software ESTOP."""

    def __init__(self):
        super().__init__('software_estop_example')

        # Create client for software ESTOP service
        self.estop_client = self.create_client(SoftwareEstop, '/state_machine/software_estop')

        # Wait for service to be available
        while not self.estop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for software ESTOP service...')

        self.get_logger().info('Software ESTOP service available')

    def trigger_estop(self, operator_id: str = "example_operator",
                      reason: str = "Emergency situation detected",
                      force_immediate: bool = False):
        """Trigger software emergency stop."""

        # Create request
        request = SoftwareEstop.Request()
        request.operator_id = operator_id
        request.reason = reason
        request.acknowledge_criticality = True  # Must acknowledge this is critical
        request.force_immediate = force_immediate

        self.get_logger().critical(f'TRIGGERING SOFTWARE ESTOP: {reason}')

        # Call service
        future = self.estop_client.call_async(request)

        # Wait for response
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().critical(
                    f'SOFTWARE ESTOP SUCCESSFUL: {response.message} '
                    f'(ID: {response.estop_id})'
                )
            else:
                self.get_logger().error(f'SOFTWARE ESTOP FAILED: {response.message}')
        else:
            self.get_logger().error('Service call failed')


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    estop_example = SoftwareEstopExample()

    try:
        # Example: Trigger software ESTOP for critical situation
        estop_example.trigger_estop(
            operator_id="mission_control",
            reason="Critical navigation failure - potential collision risk",
            force_immediate=True  # Force immediate shutdown
        )

    except KeyboardInterrupt:
        pass
    finally:
        estop_example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()







