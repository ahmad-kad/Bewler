#!/usr/bin/env python3
"""
🚀 Autonomous Typing Subsystem Node

URC 2026 Autonomous Typing Subsystem implementation.
Integrates keyboard detection, arm control, and sequence execution.
"""


import numpy as np
import rclpy
from autonomy_autonomous_typing.arm_controller import ArmController
from autonomy_autonomous_typing.aruco_detection import TypingArUcoDetector
from autonomy_autonomous_typing.typing_executor import TypingExecutor
from autonomy_interfaces.action import PerformTyping
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String


class AutonomousTypingNode(Node):
    """
    Main autonomous typing node orchestrating all subsystems.

    Coordinates:
    - Keyboard localization (ArUco markers via vision)
    - Arm control (motion planning and execution)
    - Typing execution (sequence coordination)
    """

    def __init__(self):
        super().__init__("autonomous_typing_node")

        # Initialize subsystems
        self.arm_controller = ArmController(self)
        self.typing_executor = TypingExecutor(self, self.arm_controller)
        self.aruco_detector = TypingArUcoDetector(self)

        # Publishers
        self.status_publisher = self.create_publisher(String, "autonomous_typing/status", 10)
        self.typing_result_publisher = self.create_publisher(String, "typing_result", 10)

        # Subscribers
        self.keyboard_pose_sub = self.create_subscription(
            PoseStamped, "keyboard_pose", self._keyboard_pose_callback, 10
        )

        # Action server for typing tasks
        self.typing_action_server = ActionServer(self, PerformTyping, "perform_typing", self.execute_typing_callback)

        # Status tracking
        self.keyboard_pose_received = False
        self.last_keyboard_pose: PoseStamped = None
        self.aruco_alignment_ready = False
        self.last_aruco_result = None

        # Status timer
        self.status_timer = self.create_timer(1.0, self._status_callback)

        self.get_logger().info("Autonomous typing node initialized")

    def _keyboard_pose_callback(self, msg: PoseStamped) -> None:
        """Handle keyboard pose updates from localization node."""
        try:
            self.last_keyboard_pose = msg
            self.keyboard_pose_received = True

            # Extract position and orientation
            pos = msg.pose.position
            ori = msg.pose.orientation

            position = np.array([pos.x, pos.y, pos.z])
            orientation = np.array([ori.x, ori.y, ori.z, ori.w])

            # Update typing executor with keyboard pose
            self.typing_executor.set_keyboard_pose(position, orientation)

            self.get_logger().debug(f"Keyboard pose updated: pos={position}, ori={orientation}")

        except Exception as e:
            self.get_logger().error(f"Error processing keyboard pose: {e}")

    def detect_keyboard_with_aruco(self, min_tags: int = 3) -> bool:
        """
        Detect keyboard using ArUco tags for alignment.

        Args:
            min_tags: Minimum number of tags required for alignment

        Returns:
            True if keyboard detected and aligned successfully
        """
        try:
            self.get_logger().info("Detecting keyboard with ArUco tags...")

            # Detect ArUco tags for typing alignment
            result = self.aruco_detector.detect_typing_tags_with_minimum(
                min_tags=min_tags,
                target_depth=0.3,  # 30cm from keyboard
                timeout=5.0,
                max_detection_distance=3.0,
            )

            if result["success"]:
                self.last_aruco_result = result
                self.aruco_alignment_ready = result["mission_ready"]

                # Update typing executor with ArUco-based keyboard pose
                if result["alignment_available"]:
                    target_pos = result["arm_target_position"]
                    target_ori = result["alignment_orientation"]

                    position = np.array([target_pos.x, target_pos.y, target_pos.z])
                    orientation = np.array([target_ori.x, target_ori.y, target_ori.z, target_ori.w])

                    self.typing_executor.set_keyboard_pose(position, orientation)
                    self.keyboard_pose_received = True

                    self.get_logger().info(
                        f"Keyboard detected with ArUco alignment: "
                        f"quality={result['alignment_quality']:.2f}, "
                        f"tags={len(result['detected_tag_ids'])}"
                    )

                    # Log keyboard information
                    keyboard_info = result.get("keyboard_info", {})
                    if keyboard_info:
                        self.get_logger().info(
                            f"Keyboard info: size={keyboard_info.get('estimated_keyboard_size', 0):.2f}m, "
                            f"aspect_ratio={keyboard_info.get('aspect_ratio', 0):.2f}"
                        )

                    return True
                else:
                    self.get_logger().warn("ArUco detection successful but alignment not available")
                    return False
            else:
                self.get_logger().warn(f"ArUco detection failed: {result['message']}")
                return False

        except Exception as e:
            self.get_logger().error(f"ArUco detection error: {e}")
            return False

    def get_aruco_typing_status(self) -> dict:
        """Get current ArUco typing status."""
        if self.last_aruco_result is None:
            return {"status": "not_detected", "message": "No ArUco detection performed"}

        return self.aruco_detector.get_typing_status(self.last_aruco_result)

    def is_aruco_typing_ready(self) -> bool:
        """Check if ArUco-based typing is ready."""
        return self.aruco_alignment_ready

    def execute_typing_callback(self, goal_handle):
        """
        Execute typing action for competition mission.

        Goal: PerformTyping with sequence string
        Feedback: characters_typed, current_character
        Result: success, message, characters_typed
        """
        try:
            typing_sequence = goal_handle.request.sequence
            self.get_logger().info(f'Received typing goal: "{typing_sequence}"')

            # Validate preconditions - try ArUco detection if keyboard not detected
            if not self.keyboard_pose_received:
                self.get_logger().info("Keyboard pose not available, attempting ArUco detection...")

                # Try ArUco detection
                if not self.detect_keyboard_with_aruco(min_tags=3):
                    self.get_logger().error("ArUco detection failed")
                    result = PerformTyping.Result()
                    result.success = False
                    result.message = "Keyboard not detected via ArUco tags"
                    result.characters_typed = 0
                    goal_handle.abort()
                    goal_handle.set_aborted(result)
                    return result
                else:
                    self.get_logger().info("Keyboard detected via ArUco tags")

            if not self.arm_controller.is_ready():
                self.get_logger().error("Arm controller not ready")
                result = PerformTyping.Result()
                result.success = False
                result.message = "Arm controller not ready"
                result.characters_typed = 0
                goal_handle.abort()
                goal_handle.set_aborted(result)
                return result

            # Prepare feedback message
            feedback_msg = PerformTyping.Feedback()

            # Define feedback callback for typing executor
            def feedback_callback(char_index: int, char: str, success: bool):
                feedback_msg.characters_typed = char_index + (1 if success else 0)
                feedback_msg.current_character = char
                goal_handle.publish_feedback(feedback_msg)

            # Execute typing sequence
            exec_result = self.typing_executor.execute_sequence(typing_sequence, feedback_callback=feedback_callback)

            # Prepare result message
            result = PerformTyping.Result()
            result.success = exec_result["success"]
            result.characters_typed = exec_result["characters_typed"]
            result.message = (
                f'Successfully typed {exec_result["characters_typed"]}' f'/{exec_result["total_characters"]} characters'
            )

            # Handle result
            if result.success:
                self.get_logger().info(result.message)
                goal_handle.succeed()
                goal_handle.set_succeeded(result)
            else:
                self.get_logger().warn(result.message)
                goal_handle.abort()
                goal_handle.set_aborted(result)

            return result

        except Exception as e:
            self.get_logger().error(f"Typing execution error: {e}")
            result = PerformTyping.Result()
            result.success = False
            result.message = f"Execution error: {str(e)}"
            result.characters_typed = 0
            goal_handle.abort()
            goal_handle.set_aborted(result)
            return result

    def _status_callback(self) -> None:
        """Publish system status periodically."""
        try:
            status_msg = String()

            keyboard_status = "detected" if self.keyboard_pose_received else "not detected"
            arm_status = "ready" if self.arm_controller.is_ready() else "not ready"
            exec_status = self.typing_executor.get_status()["status"]

            # Add ArUco status if available
            aruco_status = ""
            if self.last_aruco_result is not None:
                aruco_info = self.get_aruco_typing_status()
                aruco_status = f" | ArUco: {aruco_info.get('detected_tags', 0)} tags, quality: {aruco_info.get('alignment_quality', 0):.2f}"

            status_msg.data = (
                f"Typing System | Keyboard: {keyboard_status} | "
                f"Arm: {arm_status} | Status: {exec_status}{aruco_status}"
            )

            self.status_publisher.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    # Create and spin the node
    node = AutonomousTypingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
