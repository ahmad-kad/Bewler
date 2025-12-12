"""
URC 2026 Autonomy System - ROS 2 Action Server Template

Template for implementing ROS 2 action servers for long-running tasks.

Author: [Your Name]
Date: [Date]
"""

# Standard imports
import time
from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

# Import your action types
from your_package.action import YourAction


class YourActionServer(Node):
    """
    ROS 2 Action Server for [describe the long-running task].

    This action server handles [specific task] with progress feedback,
    goal cancellation, and result reporting.
    """

    def __init__(self):
        super().__init__("your_action_server")

        # Use ReentrantCallbackGroup for action servers
        self.callback_group = ReentrantCallbackGroup()

        # Create action server
        self.action_server = ActionServer(
            self,
            YourAction,
            "your_action",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        # State tracking
        self.current_goal_handle: Optional[ActionServer.GoalHandle] = None
        self.is_executing = False

        self.get_logger().info("Action server initialized")

    def goal_callback(self, goal_request):
        """Handle incoming goal requests."""
        self.get_logger().info("Received goal request")

        # Validate goal
        if not self._validate_goal(goal_request):
            self.get_logger().warn("Goal validation failed")
            return GoalResponse.REJECT

        # Check if we can accept the goal
        if self.is_executing:
            self.get_logger().warn("Already executing a goal, rejecting")
            return GoalResponse.REJECT

        self.get_logger().info("Goal accepted")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle goal cancellation requests."""
        self.get_logger().info("Received cancel request")

        # Check if we can cancel
        if goal_handle != self.current_goal_handle:
            self.get_logger().warn("Cancel request for unknown goal")
            return CancelResponse.REJECT

        self.get_logger().info("Cancel request accepted")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the accepted goal."""
        self.get_logger().info("Executing goal")

        self.current_goal_handle = goal_handle
        self.is_executing = True

        # Extract goal data
        goal = goal_handle.request

        # Create feedback and result
        feedback = YourAction.Feedback()
        result = YourAction.Result()

        try:
            # Execute the long-running task
            success = self._execute_task(goal, feedback, result)

            if success:
                goal_handle.succeed()
                self.get_logger().info("Goal succeeded")
            else:
                goal_handle.abort()
                self.get_logger().warn("Goal aborted")

        except Exception as e:
            self.get_logger().error(f"Goal execution failed: {e}")
            goal_handle.abort()
            result.error_message = str(e)

        finally:
            self.is_executing = False
            self.current_goal_handle = None

        return result

    def _validate_goal(self, goal_request) -> bool:
        """Validate the incoming goal request."""
        # Implement your goal validation logic
        # Check parameters, constraints, etc.

        goal = goal_request.your_goal_field

        # Example validations
        if goal.some_parameter < 0:
            return False

        if goal.another_parameter > 100:
            return False

        return True

    def _execute_task(self, goal, feedback, result) -> bool:
        """Execute the main task logic."""
        try:
            # Initialize progress
            total_steps = 100
            current_step = 0

            # Main execution loop
            while current_step < total_steps:
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Goal cancelled during execution")
                    result.error_message = "Goal was cancelled"
                    return False

                # Perform work
                success = self._perform_work_step(goal, current_step)

                if not success:
                    result.error_message = "Work step failed"
                    return False

                # Update progress
                current_step += 1
                progress = float(current_step) / total_steps

                # Publish feedback
                feedback.progress = progress
                feedback.current_step = current_step
                feedback.status_message = f"Step {current_step}/{total_steps}"

                goal_handle.publish_feedback(feedback)

                # Small delay to prevent overwhelming the system
                time.sleep(0.1)

            # Set final result
            result.success = True
            result.final_message = "Task completed successfully"
            result.final_value = 42.0  # Example result data

            return True

        except Exception as e:
            self.get_logger().error(f"Task execution error: {e}")
            result.error_message = str(e)
            return False

    def _perform_work_step(self, goal, step) -> bool:
        """Perform a single step of work."""
        try:
            # Implement your work step logic here
            # This could be:
            # - Moving a motor
            # - Processing sensor data
            # - Computing a trajectory
            # - etc.

            # Simulate work
            time.sleep(0.05)

            # Return success/failure
            return True

        except Exception as e:
            self.get_logger().error(f"Work step {step} failed: {e}")
            return False


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        node = YourActionServer()

        # Use MultiThreadedExecutor for action servers
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info("Action server shutting down")
        finally:
            executor.shutdown()

    except Exception as e:
        print(f"Failed to start action server: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
