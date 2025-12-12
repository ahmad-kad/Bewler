"""
ArUco Alignment Calculator for Autonomous Typing.

Calculates optimal arm positioning for coplanar alignment with ArUco tags
for autonomous typing missions. Works with any ArUco tag IDs.
"""

import math
from typing import Any, Dict, List, Tuple

import numpy as np
import structlog
from geometry_msgs.msg import Point, Quaternion

logger = structlog.get_logger(__name__)


class TypingAlignmentCalculator:
    """
    Calculator for ArUco tag-based arm alignment for typing missions.

    Provides methods to calculate optimal arm positioning for typing operations
    requiring coplanar alignment with any ArUco tags detected.
    """

    def __init__(self):
        """Initialize the typing alignment calculator."""
        self.logger = structlog.get_logger(__name__)

        # Typing-specific configuration
        self.typing_config = {
            "min_tags": 3,  # Minimum tags for alignment (3 corners)
            "optimal_tags": 4,  # Optimal tags for alignment (4 corners)
            "target_depth": 0.3,  # 30cm from keyboard surface
            "alignment_tolerance": 0.05,  # 5cm position tolerance
            "orientation_tolerance": 0.1,  # ~6 degrees
            "keyboard_size_estimate": 0.3,  # Estimated keyboard size (30cm)
        }

    def calculate_typing_alignment(
        self, detected_tags: List[Dict[str, Any]], target_depth: float = None
    ) -> Dict[str, Any]:
        """
        Calculate alignment for autonomous typing with any ArUco tags.

        Args:
            detected_tags: List of detected tag information (any IDs)
            target_depth: Override target depth (optional)

        Returns:
            Dictionary with alignment calculation results
        """
        self.logger.info("Calculating typing alignment", num_tags=len(detected_tags))

        # Check minimum tag requirements
        if len(detected_tags) < self.typing_config["min_tags"]:
            return self._create_error_result(
                f"Insufficient tags for typing alignment: {len(detected_tags)} < {self.typing_config['min_tags']}"
            )

        # Use configured target depth if not provided
        if target_depth is None:
            target_depth = self.typing_config["target_depth"]

        try:
            # Calculate rectangular alignment for keyboard
            result = self._calculate_keyboard_alignment(detected_tags, target_depth)

            # Add typing-specific information
            result["mission_type"] = "AUTONOMOUS_TYPING"
            result["mission_ready"] = self._evaluate_typing_readiness(result)
            result["alignment_warnings"] = self._generate_typing_warnings(result)
            result["typing_recommendations"] = self._generate_typing_recommendations(result)

            return result

        except Exception as e:
            self.logger.error("Typing alignment calculation failed", error=str(e))
            return self._create_error_result(f"Alignment calculation failed: {str(e)}")

    def _calculate_keyboard_alignment(self, detected_tags: List[Dict[str, Any]], target_depth: float) -> Dict[str, Any]:
        """Calculate alignment for keyboard typing (rectangular layout)."""
        if len(detected_tags) < 3:
            return self._create_error_result("Need at least 3 tags for keyboard alignment")

        # Extract positions
        positions = [tag["position"] for tag in detected_tags]

        # Calculate center point
        center = self._calculate_center_point(positions)

        # Calculate plane normal using PCA
        normal, orientation = self._calculate_plane_normal(positions)

        # Calculate target position (offset by target_depth along normal)
        target_position = Point()
        target_position.x = center.x + normal[0] * target_depth
        target_position.y = center.y + normal[1] * target_depth
        target_position.z = center.z + normal[2] * target_depth

        # Calculate alignment quality
        quality = self._calculate_keyboard_alignment_quality(positions, center, normal)

        # Calculate individual tag errors
        tag_errors = self._calculate_tag_errors(positions, center, normal)

        # Estimate keyboard orientation and size
        keyboard_info = self._estimate_keyboard_properties(positions, center, normal)

        return {
            "success": True,
            "alignment_available": True,
            "alignment_center": center,
            "alignment_orientation": orientation,
            "arm_target_position": target_position,
            "alignment_quality": quality,
            "alignment_errors": tag_errors,
            "detected_tag_ids": [tag["id"] for tag in detected_tags],
            "tag_positions": positions,
            "keyboard_info": keyboard_info,
        }

    def _calculate_center_point(self, positions: List[Point]) -> Point:
        """Calculate the center point of a list of positions."""
        center = Point()
        center.x = sum(pos.x for pos in positions) / len(positions)
        center.y = sum(pos.y for pos in positions) / len(positions)
        center.z = sum(pos.z for pos in positions) / len(positions)
        return center

    def _calculate_plane_normal(self, positions: List[Point]) -> Tuple[np.ndarray, Quaternion]:
        """Calculate plane normal using Principal Component Analysis."""
        # Convert to numpy array
        points = np.array([[pos.x, pos.y, pos.z] for pos in positions])

        # Center the points
        centroid = np.mean(points, axis=0)
        centered_points = points - centroid

        # Calculate covariance matrix
        cov_matrix = np.cov(centered_points.T)

        # Find eigenvalues and eigenvectors
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

        # The normal is the eigenvector with smallest eigenvalue
        normal = eigenvectors[:, 0]

        # Ensure normal points towards camera (negative z component)
        if normal[2] > 0:
            normal = -normal

        # Convert to quaternion
        orientation = self._normal_to_quaternion(normal)

        return normal, orientation

    def _normal_to_quaternion(self, normal: np.ndarray) -> Quaternion:
        """Convert normal vector to quaternion orientation."""
        # Default orientation (facing forward)
        default_normal = np.array([0, 0, 1])

        # Calculate rotation axis
        rotation_axis = np.cross(default_normal, normal)
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)

        # Calculate rotation angle
        cos_angle = np.dot(default_normal, normal)
        angle = math.acos(np.clip(cos_angle, -1.0, 1.0))

        # Convert to quaternion
        quat = Quaternion()
        quat.w = math.cos(angle / 2)
        quat.x = rotation_axis[0] * math.sin(angle / 2)
        quat.y = rotation_axis[1] * math.sin(angle / 2)
        quat.z = rotation_axis[2] * math.sin(angle / 2)

        return quat

    def _calculate_keyboard_alignment_quality(self, positions: List[Point], center: Point, normal: np.ndarray) -> float:
        """Calculate alignment quality score for keyboard typing (0.0-1.0)."""
        if len(positions) < 3:
            return 0.0

        # Calculate how well points fit the plane
        plane_errors = []
        for pos in positions:
            # Distance from point to plane
            point_vec = np.array([pos.x - center.x, pos.y - center.y, pos.z - center.z])
            distance = abs(np.dot(point_vec, normal))
            plane_errors.append(distance)

        # Calculate quality based on plane fit
        max_error = max(plane_errors)
        tolerance = self.typing_config["alignment_tolerance"]
        quality = max(0.0, 1.0 - (max_error / tolerance))

        # Bonus for having more tags (up to optimal count)
        detected_tags = len(positions)
        optimal_tags = self.typing_config["optimal_tags"]
        if detected_tags >= optimal_tags:
            tag_bonus = 1.0
        else:
            tag_bonus = detected_tags / optimal_tags

        quality *= tag_bonus

        # Check if keyboard size is reasonable
        keyboard_size = self._estimate_keyboard_size(positions)
        expected_size = self.typing_config["keyboard_size_estimate"]
        size_ratio = min(keyboard_size, expected_size) / max(keyboard_size, expected_size)
        quality *= 0.5 + 0.5 * size_ratio  # 50-100% based on size match

        return min(1.0, max(0.0, quality))

    def _calculate_tag_errors(self, positions: List[Point], center: Point, normal: np.ndarray) -> List[float]:
        """Calculate individual tag alignment errors."""
        errors = []
        for pos in positions:
            point_vec = np.array([pos.x - center.x, pos.y - center.y, pos.z - center.z])
            distance = abs(np.dot(point_vec, normal))
            errors.append(distance)
        return errors

    def _estimate_keyboard_properties(
        self, positions: List[Point], center: Point, normal: np.ndarray
    ) -> Dict[str, Any]:
        """Estimate keyboard properties from detected tags."""
        if len(positions) < 2:
            return {"width": 0.0, "height": 0.0, "aspect_ratio": 1.0}

        # Project points onto the plane
        projected_points = []
        for pos in positions:
            point_vec = np.array([pos.x - center.x, pos.y - center.y, pos.z - center.z])
            # Project onto plane (remove normal component)
            projected = point_vec - np.dot(point_vec, normal) * normal
            projected_points.append(projected)

        # Calculate bounding box
        if len(projected_points) >= 2:
            x_coords = [p[0] for p in projected_points]
            y_coords = [p[1] for p in projected_points]
            z_coords = [p[2] for p in projected_points]

            width = max(x_coords) - min(x_coords)
            height = max(y_coords) - min(y_coords)
            depth = max(z_coords) - min(z_coords)

            aspect_ratio = width / height if height > 0 else 1.0

            return {
                "width": width,
                "height": height,
                "depth": depth,
                "aspect_ratio": aspect_ratio,
                "estimated_keyboard_size": max(width, height),
            }
        else:
            return {"width": 0.0, "height": 0.0, "aspect_ratio": 1.0}

    def _estimate_keyboard_size(self, positions: List[Point]) -> float:
        """Estimate keyboard size from tag positions."""
        if len(positions) < 2:
            return 0.0

        # Calculate maximum distance between any two points
        max_distance = 0.0
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                pos1 = positions[i]
                pos2 = positions[j]
                distance = math.sqrt((pos2.x - pos1.x) ** 2 + (pos2.y - pos1.y) ** 2 + (pos2.z - pos1.z) ** 2)
                max_distance = max(max_distance, distance)

        return max_distance

    def _evaluate_typing_readiness(self, result: Dict[str, Any]) -> bool:
        """Evaluate if typing mission can proceed based on alignment results."""
        if not result.get("success", False):
            return False

        # Check alignment quality threshold
        quality = result.get("alignment_quality", 0.0)
        min_quality = 0.6  # 60% minimum quality for typing
        if quality < min_quality:
            return False

        # Check if we have enough tags
        detected_tags = len(result.get("detected_tag_ids", []))
        min_tags = self.typing_config["min_tags"]
        if detected_tags < min_tags:
            return False

        # Check keyboard size reasonableness
        keyboard_info = result.get("keyboard_info", {})
        estimated_size = keyboard_info.get("estimated_keyboard_size", 0.0)
        expected_size = self.typing_config["keyboard_size_estimate"]

        # Allow 50% size variation
        size_ratio = estimated_size / expected_size
        if size_ratio < 0.5 or size_ratio > 2.0:
            return False

        return True

    def _generate_typing_warnings(self, result: Dict[str, Any]) -> List[str]:
        """Generate warnings about typing alignment quality."""
        warnings = []

        quality = result.get("alignment_quality", 0.0)
        if quality < 0.8:
            warnings.append(f"Low typing alignment quality: {quality:.2f}")

        detected_tags = len(result.get("detected_tag_ids", []))
        optimal_tags = self.typing_config["optimal_tags"]
        if detected_tags < optimal_tags:
            missing = optimal_tags - detected_tags
            warnings.append(f"Missing {missing} optimal tags for typing")

        # Check for high individual tag errors
        errors = result.get("alignment_errors", [])
        if errors:
            max_error = max(errors)
            tolerance = self.typing_config["alignment_tolerance"]
            if max_error > tolerance:
                warnings.append(f"High alignment error: {max_error:.3f}m > {tolerance:.3f}m")

        # Check keyboard size
        keyboard_info = result.get("keyboard_info", {})
        estimated_size = keyboard_info.get("estimated_keyboard_size", 0.0)
        expected_size = self.typing_config["keyboard_size_estimate"]
        size_ratio = estimated_size / expected_size
        if size_ratio < 0.7 or size_ratio > 1.5:
            warnings.append(f"Unusual keyboard size: {estimated_size:.2f}m (expected ~{expected_size:.2f}m)")

        return warnings

    def _generate_typing_recommendations(self, result: Dict[str, Any]) -> List[str]:
        """Generate recommendations for improving typing mission success."""
        recommendations = []

        quality = result.get("alignment_quality", 0.0)
        if quality < 0.9:
            recommendations.append("Improve camera positioning for better tag visibility")
            recommendations.append("Ensure adequate lighting on ArUco tags")
            recommendations.append("Position tags at keyboard corners for better alignment")

        detected_tags = len(result.get("detected_tag_ids", []))
        optimal_tags = self.typing_config["optimal_tags"]
        if detected_tags < optimal_tags:
            recommendations.append("Add more ArUco tags at keyboard corners")
            recommendations.append("Ensure tags are clearly visible and not occluded")

        # Check keyboard orientation
        keyboard_info = result.get("keyboard_info", {})
        aspect_ratio = keyboard_info.get("aspect_ratio", 1.0)
        if aspect_ratio < 2.0 or aspect_ratio > 4.0:
            recommendations.append("Ensure keyboard is properly oriented (landscape)")

        recommendations.append("Verify keyboard is flat and stable")
        recommendations.append("Check that all corner tags are visible")

        return recommendations

    def _create_error_result(self, message: str) -> Dict[str, Any]:
        """Create an error result dictionary."""
        return {
            "success": False,
            "message": message,
            "alignment_available": False,
            "mission_ready": False,
            "alignment_quality": 0.0,
            "alignment_warnings": [message],
            "typing_recommendations": [],
        }
