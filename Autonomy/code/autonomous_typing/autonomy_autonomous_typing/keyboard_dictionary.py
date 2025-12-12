#!/usr/bin/env python3
"""
Keyboard Dictionary Module

Manages keyboard layout with 3D positions relative to keyboard frame origin.
Provides utilities for transforming key positions to world coordinates.
"""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import numpy as np


@dataclass
class KeyInfo:
    """Information about a keyboard key."""

    name: str
    xy_offset: Tuple[float, float]  # Position relative to keyboard origin
    description: str
    key_width: float = 0.013  # ~13mm standard key width
    key_height: float = 0.013  # ~13mm standard key height


class KeyboardDictionary:
    """
    Manages keyboard layout with 3D positions relative to keyboard frame.

    The keyboard frame origin is assumed to be at one corner of the keyboard,
    typically where markers are placed. All key positions are relative to this origin.
    """

    def __init__(self):
        """Initialize keyboard layout dictionary."""
        self.keyboard_layout = self._create_keyboard_layout()

        # Pressing parameters (in meters)
        self.press_params = {
            "approach_height": 0.02,  # 2cm above keyboard surface
            "press_depth": 0.015,  # 1.5cm into keyboard
            "press_force": 3.0,  # Newtons (if force controlled)
            "dwell_time": 0.1,  # seconds at bottom of press
            "retract_time": 0.3,  # seconds to lift off
        }

    def _create_keyboard_layout(self) -> Dict[str, KeyInfo]:
        """
        Create complete QWERTY keyboard layout with positions relative to
        keyboard frame origin (typically top-left marker position).

        Standard keyboard dimensions:
        - Key spacing: 13.5mm
        - Row spacing: 19mm
        - Standard offset from edge: 10mm
        """
        # Base offsets (origin at top-left marker position)
        # First key position (Q key) relative to origin
        first_key_x = 0.02  # 2cm from left edge
        first_key_y = 0.015  # 1.5cm from top edge
        key_spacing_x = 0.0135  # 13.5mm horizontal spacing
        key_spacing_y = 0.019  # 19mm vertical (row) spacing

        layout = {}

        # Number row (1-0, -, =, backspace)
        number_row_y = first_key_y
        number_keys = [
            "`",
            "1",
            "2",
            "3",
            "4",
            "5",
            "6",
            "7",
            "8",
            "9",
            "0",
            "-",
            "=",
            "backspace",
        ]
        for i, key in enumerate(number_keys):
            layout[key] = KeyInfo(
                name=key,
                xy_offset=(first_key_x + i * key_spacing_x, number_row_y),
                description=f"Number row: {key}",
            )

        # QWERTY row
        qwerty_y = first_key_y + key_spacing_y
        qwerty_keys = [
            "tab",
            "q",
            "w",
            "e",
            "r",
            "t",
            "y",
            "u",
            "i",
            "o",
            "p",
            "[",
            "]",
            "\\",
        ]
        for i, key in enumerate(qwerty_keys):
            layout[key] = KeyInfo(
                name=key,
                xy_offset=(first_key_x + i * key_spacing_x, qwerty_y),
                description=f"QWERTY row: {key}",
            )

        # ASDF row (home row)
        asdf_y = first_key_y + 2 * key_spacing_y
        asdf_keys = [
            "caps",
            "a",
            "s",
            "d",
            "f",
            "g",
            "h",
            "j",
            "k",
            "l",
            ";",
            "'",
            "enter",
        ]
        for i, key in enumerate(asdf_keys):
            layout[key] = KeyInfo(
                name=key,
                xy_offset=(first_key_x + i * key_spacing_x, asdf_y),
                description=f"Home row: {key}",
            )

        # ZXCV row
        zxcv_y = first_key_y + 3 * key_spacing_y
        zxcv_keys = [
            "shift_l",
            "z",
            "x",
            "c",
            "v",
            "b",
            "n",
            "m",
            ",",
            ".",
            "/",
            "shift_r",
        ]
        for i, key in enumerate(zxcv_keys):
            layout[key] = KeyInfo(
                name=key,
                xy_offset=(first_key_x + i * key_spacing_x, zxcv_y),
                description=f"Lower row: {key}",
            )

        # Space bar (centered)
        space_y = first_key_y + 4 * key_spacing_y
        space_center_x = first_key_x + 4.5 * key_spacing_x
        layout["space"] = KeyInfo(
            name="space",
            xy_offset=(space_center_x, space_y),
            description="Space bar (center)",
            key_width=0.06,  # Wider key
        )

        return layout

    def get_key_position(self, key: str) -> Optional[Tuple[float, float]]:
        """
        Get (x, y) position of a key relative to keyboard frame origin.

        Args:
            key: Key name (e.g., 'q', 'space', 'backspace')

        Returns:
            Tuple of (x, y) in meters, or None if key not found
        """
        key_lower = key.lower()
        if key_lower not in self.keyboard_layout:
            return None

        return self.keyboard_layout[key_lower].xy_offset

    def get_key_3d_position(self, key: str, z_offset: float = 0.0) -> Optional[np.ndarray]:
        """
        Get 3D position of a key relative to keyboard frame origin.

        Args:
            key: Key name
            z_offset: Z position in keyboard frame (0 = at surface, positive = above)

        Returns:
            3D numpy array [x, y, z] or None if key not found
        """
        xy_pos = self.get_key_position(key)
        if xy_pos is None:
            return None

        return np.array([xy_pos[0], xy_pos[1], z_offset])

    def get_key_press_trajectory(self, key: str) -> Optional[Dict[str, np.ndarray]]:
        """
        Get 3D trajectory for pressing a key (approach -> press -> retract).

        Args:
            key: Key name

        Returns:
            Dict with 'approach', 'press', and 'retract' positions, or None
        """
        xy_pos = self.get_key_position(key)
        if xy_pos is None:
            return None

        approach_z = self.press_params["approach_height"]
        press_z = self.press_params["press_depth"]

        return {
            "approach": np.array([xy_pos[0], xy_pos[1], approach_z]),
            "press": np.array([xy_pos[0], xy_pos[1], press_z]),
            "retract": np.array([xy_pos[0], xy_pos[1], approach_z]),
        }

    def get_all_keys(self) -> list:
        """Get list of all supported keys."""
        return list(self.keyboard_layout.keys())

    def is_valid_key(self, key: str) -> bool:
        """Check if a key is in the keyboard layout."""
        return key.lower() in self.keyboard_layout

    def get_press_params(self) -> Dict:
        """Get keyboard pressing parameters."""
        return self.press_params.copy()

    def set_press_params(self, **kwargs) -> None:
        """Update pressing parameters."""
        for key, value in kwargs.items():
            if key in self.press_params:
                self.press_params[key] = value


class KeyboardTransformManager:
    """
    Manages coordinate transforms between keyboard frame and world frame.
    """

    def __init__(self):
        """Initialize transform manager."""
        self.keyboard_dict = KeyboardDictionary()

    def keyboard_to_world(
        self,
        keyboard_frame_pos: np.ndarray,
        keyboard_pose_position: np.ndarray,
        keyboard_pose_orientation: np.ndarray,  # Quaternion [x, y, z, w]
    ) -> np.ndarray:
        """
        Transform a position from keyboard frame to world frame.

        Args:
            keyboard_frame_pos: Position in keyboard frame [x, y, z]
            keyboard_pose_position: Keyboard frame origin in world frame [x, y, z]
            keyboard_pose_orientation: Keyboard frame rotation as quaternion [x, y, z, w]

        Returns:
            Position in world frame [x, y, z]
        """
        # Convert quaternion to rotation matrix
        R = self._quaternion_to_rotation_matrix(keyboard_pose_orientation)

        # Apply rotation and translation
        world_pos = R @ keyboard_frame_pos + keyboard_pose_position

        return world_pos

    def get_key_world_position(
        self,
        key: str,
        keyboard_pose_position: np.ndarray,
        keyboard_pose_orientation: np.ndarray,
        z_offset: float = 0.0,
    ) -> Optional[np.ndarray]:
        """
        Get world position of a key given keyboard pose.

        Args:
            key: Key name
            keyboard_pose_position: Keyboard origin in world frame
            keyboard_pose_orientation: Keyboard rotation as quaternion
            z_offset: Z offset from keyboard surface

        Returns:
            3D position in world frame or None
        """
        # Get key position in keyboard frame
        key_pos = self.keyboard_dict.get_key_3d_position(key, z_offset)
        if key_pos is None:
            return None

        # Transform to world frame
        return self.keyboard_to_world(key_pos, keyboard_pose_position, keyboard_pose_orientation)

    @staticmethod
    def _quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
        """
        Convert quaternion [x, y, z, w] to 3x3 rotation matrix.
        """
        x, y, z, w = q

        R = np.array(
            [
                [1 - 2 * (y**2 + z**2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x**2 + z**2), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x**2 + y**2)],
            ]
        )

        return R


# Example usage and testing
if __name__ == "__main__":
    # Create keyboard dictionary
    kb = KeyboardDictionary()

    # Print some key positions
    test_keys = ["q", "space", "backspace", "a"]
    print("Keyboard Layout (relative to origin):")
    for key in test_keys:
        pos = kb.get_key_position(key)
        if pos:
            print(f"  {key:12} -> x={pos[0]:.4f}m, y={pos[1]:.4f}m")

    # Test press trajectory
    print("\nKey Press Trajectory for 'q':")
    traj = kb.get_key_press_trajectory("q")
    if traj:
        print(f"  Approach: {traj['approach']}")
        print(f"  Press:    {traj['press']}")
        print(f"  Retract:  {traj['retract']}")

    # Test transform manager
    print("\nTransform Example:")
    tm = KeyboardTransformManager()

    # Assume keyboard is at (0.5, 0.3, 0.1) with identity rotation
    keyboard_pos = np.array([0.5, 0.3, 0.1])
    keyboard_quat = np.array([0.0, 0.0, 0.0, 1.0])  # Identity quaternion [x, y, z, w]

    world_pos = tm.get_key_world_position("q", keyboard_pos, keyboard_quat)
    print(f"  Keyboard origin in world: {keyboard_pos}")
    print(f"  'q' key in world frame: {world_pos}")
