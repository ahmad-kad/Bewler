#!/usr/bin/env python3
"""
Typing Executor Module

Orchestrates typing sequences, handles error recovery, and validates key presses
for autonomous keyboard interaction.
"""

import time
from enum import Enum
from typing import Callable, Dict, Optional

import numpy as np
from autonomy_autonomous_typing.arm_controller import ArmController
from autonomy_autonomous_typing.keyboard_dictionary import (
    KeyboardDictionary,
    KeyboardTransformManager,
)
from rclpy.node import Node


class KeyPressStatus(Enum):
    """Status of individual key press."""

    SUCCESS = "success"
    FAILED = "failed"
    RETRY_NEEDED = "retry_needed"
    INVALID_KEY = "invalid_key"


class TypingSequenceStatus(Enum):
    """Status of typing sequence execution."""

    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class TypingExecutor:
    """
    Executes typing sequences with error handling and validation.
    """

    def __init__(self, node: Node, arm_controller: ArmController):
        """
        Initialize typing executor.

        Args:
            node: ROS 2 node for logging
            arm_controller: ArmController instance
        """
        self.node = node
        self.logger = node.get_logger()
        self.arm_controller = arm_controller

        # Keyboard management
        self.keyboard_dict = KeyboardDictionary()
        self.transform_manager = KeyboardTransformManager()

        # Sequence state
        self.current_sequence: Optional[str] = None
        self.current_index = 0
        self.current_status = TypingSequenceStatus.PENDING

        # Statistics
        self.typing_stats = {
            "total_keys_attempted": 0,
            "successful_presses": 0,
            "failed_presses": 0,
            "retried_presses": 0,
        }

        # Execution parameters
        self.max_retries_per_key = 2
        self.retry_delay = 0.5  # seconds between retries
        self.validation_method = "position"  # 'position' or 'force'

        # Keyboard pose (to be updated externally)
        self.keyboard_pose_position: Optional[np.ndarray] = None
        self.keyboard_pose_orientation: Optional[np.ndarray] = None

        self.logger.info("Typing executor initialized")

    def set_keyboard_pose(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """
        Set current keyboard pose (called by localization node).

        Args:
            position: Keyboard frame origin in world [x, y, z]
            orientation: Keyboard frame rotation as quaternion [x, y, z, w]
        """
        self.keyboard_pose_position = position
        self.keyboard_pose_orientation = orientation

    def execute_sequence(
        self,
        sequence: str,
        feedback_callback: Optional[Callable[[int, str, bool], None]] = None,
    ) -> Dict:
        """
        Execute typing sequence character by character.

        Args:
            sequence: String to type (e.g., "HELLO")
            feedback_callback: Optional callback for progress feedback
                              Called as: feedback_callback(char_index, char, success)

        Returns:
            Dict with execution results: {
                'success': bool,
                'sequence': str,
                'characters_typed': int,
                'total_characters': int,
                'failed_characters': List[str],
                'statistics': dict
            }
        """
        try:
            # Validation
            if not self._validate_preconditions(sequence):
                return {
                    "success": False,
                    "sequence": sequence,
                    "characters_typed": 0,
                    "total_characters": len(sequence),
                    "failed_characters": list(sequence),
                    "statistics": self.typing_stats,
                }

            self.current_sequence = sequence
            self.current_index = 0
            self.current_status = TypingSequenceStatus.IN_PROGRESS
            failed_chars = []

            # Execute each character
            for i, char in enumerate(sequence):
                self.current_index = i

                self.logger.info(f'Typing character {i+1}/{len(sequence)}: "{char}"')

                # Attempt to press character
                success = self._press_character(char)

                if success:
                    self.typing_stats["successful_presses"] += 1
                else:
                    self.typing_stats["failed_presses"] += 1
                    failed_chars.append(char)

                # Invoke feedback callback if provided
                if feedback_callback:
                    feedback_callback(i, char, success)

                # Small delay between characters for system response
                time.sleep(0.3)

            # Determine overall success (80% threshold)
            success_rate = self.typing_stats["successful_presses"] / len(sequence) if sequence else 0
            overall_success = success_rate >= 0.8

            self.current_status = TypingSequenceStatus.COMPLETED if overall_success else TypingSequenceStatus.FAILED

            result = {
                "success": overall_success,
                "sequence": sequence,
                "characters_typed": self.typing_stats["successful_presses"],
                "total_characters": len(sequence),
                "success_rate": success_rate,
                "failed_characters": failed_chars,
                "statistics": self.typing_stats.copy(),
            }

            self.logger.info(
                f'Sequence execution {"completed" if overall_success else "failed"}: '
                f'{result["characters_typed"]}/{result["total_characters"]} success'
            )

            return result

        except Exception as e:
            self.logger.error(f"Sequence execution error: {e}")
            self.current_status = TypingSequenceStatus.FAILED
            return {
                "success": False,
                "sequence": sequence,
                "characters_typed": self.typing_stats["successful_presses"],
                "total_characters": len(sequence),
                "failed_characters": list(sequence[self.current_index :]),
                "statistics": self.typing_stats,
                "error": str(e),
            }

    def _press_character(self, char: str, retry_count: int = 0) -> bool:
        """
        Attempt to press a single character with retries.

        Args:
            char: Character to press
            retry_count: Current retry attempt number

        Returns:
            True if press succeeded, False otherwise
        """
        try:
            # Validate key exists
            if not self.keyboard_dict.is_valid_key(char):
                self.logger.warn(f'Unsupported character: "{char}"')
                return False

            # Get key world position
            key_world_pos = self._get_key_world_position(char)
            if key_world_pos is None:
                self.logger.error(f'Could not determine world position for key: "{char}"')
                return False

            self.logger.debug(f'Key "{char}" world position: {key_world_pos}')

            # Get press parameters
            press_params = self.keyboard_dict.get_press_params()

            # Execute press with arm controller
            success = self.arm_controller.press_key(
                key_world_pos,
                approach_height=press_params["approach_height"],
                press_depth=press_params["press_depth"],
                dwell_time=press_params["dwell_time"],
            )

            if success:
                self.logger.info(f'Successfully pressed key: "{char}"')
                return True
            elif retry_count < self.max_retries_per_key:
                self.logger.warn(f"Key press failed, retrying ({retry_count + 1}/{self.max_retries_per_key})")
                self.typing_stats["retried_presses"] += 1
                time.sleep(self.retry_delay)
                return self._press_character(char, retry_count + 1)
            else:
                self.logger.error(f"Key press failed after {self.max_retries_per_key} retries")
                return False

        except Exception as e:
            self.logger.error(f'Error pressing character "{char}": {e}')
            return False

    def _get_key_world_position(self, char: str) -> Optional[np.ndarray]:
        """
        Get world frame position of a key.

        Args:
            char: Character/key name

        Returns:
            3D position in world frame or None
        """
        # Check preconditions
        if self.keyboard_pose_position is None or self.keyboard_pose_orientation is None:
            self.logger.error("Keyboard pose not available")
            return None

        # Get key position relative to keyboard frame
        key_keyboard_pos = self.keyboard_dict.get_key_3d_position(char, z_offset=0.0)
        if key_keyboard_pos is None:
            return None

        # Transform to world frame
        key_world_pos = self.transform_manager.keyboard_to_world(
            key_keyboard_pos,
            self.keyboard_pose_position,
            self.keyboard_pose_orientation,
        )

        return key_world_pos

    def _validate_preconditions(self, sequence: str) -> bool:
        """
        Validate that system is ready for typing.

        Args:
            sequence: Sequence to type

        Returns:
            True if all preconditions met, False otherwise
        """
        checks = []

        # Check arm is ready
        if not self.arm_controller.is_ready():
            self.logger.error("Arm controller not ready")
            checks.append(False)
        else:
            checks.append(True)

        # Check keyboard pose is available
        if self.keyboard_pose_position is None or self.keyboard_pose_orientation is None:
            self.logger.error("Keyboard pose not available")
            checks.append(False)
        else:
            checks.append(True)

        # Check all characters are valid
        for char in sequence:
            if not self.keyboard_dict.is_valid_key(char):
                self.logger.warn(f'Unsupported character in sequence: "{char}"')
                checks.append(False)
                break
        else:
            checks.append(True)

        # Check sequence is not empty
        if not sequence:
            self.logger.error("Sequence is empty")
            checks.append(False)
        else:
            checks.append(True)

        return all(checks)

    def validate_key_press(self, char: str, validation_method: str = "position") -> bool:
        """
        Validate that a key press was successful.

        Args:
            char: Character that was pressed
            validation_method: Method to use ('position' or 'force')

        Returns:
            True if press validation succeeds, False otherwise
        """
        if validation_method == "position":
            # Check end-effector position near key
            expected_pos = self._get_key_world_position(char)
            if expected_pos is None:
                return False

            # Would compare with actual arm position from feedback
            # This is a placeholder - in practice would check joint states
            return True

        elif validation_method == "force":
            # Check force feedback indicates key was pressed
            # This would require force/torque sensor integration
            # Placeholder for now
            return True

        else:
            self.logger.error(f"Unknown validation method: {validation_method}")
            return False

    def get_status(self) -> Dict:
        """Get current execution status."""
        return {
            "status": self.current_status.value,
            "sequence": self.current_sequence,
            "current_index": self.current_index,
            "statistics": self.typing_stats.copy(),
        }

    def reset_statistics(self) -> None:
        """Reset typing statistics."""
        self.typing_stats = {
            "total_keys_attempted": 0,
            "successful_presses": 0,
            "failed_presses": 0,
            "retried_presses": 0,
        }


# Standalone testing
if __name__ == "__main__":
    print("Typing Executor Module")
    print("=" * 50)

    # Test keyboard dictionary
    kb = KeyboardDictionary()
    test_sequence = "hello"

    print(f"\nValidating sequence: '{test_sequence}'")
    for char in test_sequence:
        if kb.is_valid_key(char):
            pos = kb.get_key_position(char)
            traj = kb.get_key_press_trajectory(char)
            print(f"  '{char}' -> pos: {pos}, traj keys: {traj.keys() if traj else 'None'}")
        else:
            print(f"  '{char}' -> INVALID")

    print("\nTest completed successfully")
