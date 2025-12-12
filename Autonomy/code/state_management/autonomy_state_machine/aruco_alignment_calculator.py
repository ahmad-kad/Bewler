"""
ArUco Alignment Calculator for Mission-Specific Operations.

Calculates optimal arm positioning for coplanar alignment with ArUco tags
for autonomous typing and USB connection missions.
"""

import math
from typing import Any, Dict, List, Tuple

import numpy as np
import structlog
from autonomy_interfaces.msg import ArmAlignmentCommand
from geometry_msgs.msg import Point, Quaternion

logger = structlog.get_logger(__name__)


class ArUcoAlignmentCalculator:
    """
    Calculator for ArUco tag-based arm alignment.

    Provides methods to calculate optimal arm positioning for mission-specific
    operations requiring coplanar alignment with ArUco tags.
    """

    def __init__(self):
        """Initialize the alignment calculator."""
        self.logger = structlog.get_logger(__name__)

        # Mission-specific tag configurations
        self.mission_configs = {
            "AUTONOMOUS_TYPING": {
                "required_tags": 4,  # 4 corners of keyboard
                "tag_layout": "rectangular",
                "min_tags": 3,  # Minimum tags for alignment
                "target_depth": 0.3,  # 30cm from tag plane
                "alignment_tolerance": 0.05,  # 5cm position tolerance
                "orientation_tolerance": 0.1,  # ~6 degrees
            },
            "USB_CONNECTION": {
                "required_tags": 2,  # 2 tags for USB port alignment
                "tag_layout": "linear",
                "min_tags": 2,
                "target_depth": 0.2,  # 20cm from tag plane
                "alignment_tolerance": 0.02,  # 2cm position tolerance
                "orientation_tolerance": 0.05,  # ~3 degrees
            },
            "PANEL_OPERATIONS": {
                "required_tags": 4,  # 4 corners of panel
                "tag_layout": "rectangular",
                "min_tags": 3,
                "target_depth": 0.4,  # 40cm from tag plane
                "alignment_tolerance": 0.08,  # 8cm position tolerance
                "orientation_tolerance": 0.15,  # ~9 degrees
            },
        }

    def calculate_alignment(
        self,
        detected_tags: List[Dict[str, Any]],
        mission_type: str,
        target_depth: float = None,
    ) -> Dict[str, Any]:
        """
        Calculate alignment for mission-specific ArUco tag detection.

        Args:
            detected_tags: List of detected tag information
            mission_type: Type of mission (AUTONOMOUS_TYPING, USB_CONNECTION, etc.)
            target_depth: Override target depth (optional)

        Returns:
            Dictionary with alignment calculation results
        """
        self.logger.info(
            "Calculating alignment",
            mission_type=mission_type,
            num_tags=len(detected_tags),
        )

        # Get mission configuration
        config = self.mission_configs.get(mission_type)
        if not config:
            return self._create_error_result(f"Unknown mission type: {mission_type}")

        # Check minimum tag requirements
        if len(detected_tags) < config["min_tags"]:
            return self._create_error_result(f"Insufficient tags detected: {len(detected_tags)} < {config['min_tags']}")

        # Use mission-specific target depth if not provided
        if target_depth is None:
            target_depth = config["target_depth"]

        try:
            # Calculate alignment based on mission type
            if config["tag_layout"] == "rectangular":
                result = self._calculate_rectangular_alignment(detected_tags, target_depth, config)
            elif config["tag_layout"] == "linear":
                result = self._calculate_linear_alignment(detected_tags, target_depth, config)
            else:
                return self._create_error_result(f"Unknown tag layout: {config['tag_layout']}")

            # Add mission-specific information
            result["mission_type"] = mission_type
            result["mission_ready"] = self._evaluate_mission_readiness(result, config)
            result["alignment_warnings"] = self._generate_alignment_warnings(result, config)
            result["mission_recommendations"] = self._generate_mission_recommendations(result, config)

            return result

        except Exception as e:
            self.logger.error("Alignment calculation failed", error=str(e))
            return self._create_error_result(f"Alignment calculation failed: {str(e)}")

    def _calculate_rectangular_alignment(
        self,
        detected_tags: List[Dict[str, Any]],
        target_depth: float,
        config: Dict[str, Any],
    ) -> Dict[str, Any]:
        """Calculate alignment for rectangular tag layout (typing, panels)."""
        if len(detected_tags) < 3:
            return self._create_error_result("Need at least 3 tags for rectangular alignment")

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
        quality = self._calculate_alignment_quality(positions, center, normal, config)

        # Calculate individual tag errors
        tag_errors = self._calculate_tag_errors(positions, center, normal)

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
        }

    def _calculate_linear_alignment(
        self,
        detected_tags: List[Dict[str, Any]],
        target_depth: float,
        config: Dict[str, Any],
    ) -> Dict[str, Any]:
        """Calculate alignment for linear tag layout (USB connection)."""
        if len(detected_tags) < 2:
            return self._create_error_result("Need at least 2 tags for linear alignment")

        # Extract positions
        positions = [tag["position"] for tag in detected_tags]

        # Calculate center point
        center = self._calculate_center_point(positions)

        # Calculate direction vector between tags
        if len(positions) >= 2:
            direction = self._calculate_direction_vector(positions[0], positions[1])
        else:
            direction = np.array([1, 0, 0])  # Default direction

        # Calculate orientation (perpendicular to direction vector)
        orientation = self._calculate_linear_orientation(direction)

        # Calculate target position
        target_position = Point()
        target_position.x = center.x + direction[0] * target_depth
        target_position.y = center.y + direction[1] * target_depth
        target_position.z = center.z + direction[2] * target_depth

        # Calculate alignment quality
        quality = self._calculate_linear_alignment_quality(positions, center, direction, config)

        # Calculate individual tag errors
        tag_errors = self._calculate_linear_tag_errors(positions, center, direction)

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

    def _calculate_direction_vector(self, pos1: Point, pos2: Point) -> np.ndarray:
        """Calculate normalized direction vector between two points."""
        direction = np.array([pos2.x - pos1.x, pos2.y - pos1.y, pos2.z - pos1.z])
        return direction / np.linalg.norm(direction)

    def _calculate_linear_orientation(self, direction: np.ndarray) -> Quaternion:
        """Calculate orientation quaternion for linear alignment."""
        # Create a rotation that aligns the z-axis with the direction vector
        z_axis = np.array([0, 0, 1])

        # Calculate rotation axis (cross product)
        rotation_axis = np.cross(z_axis, direction)
        rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)

        # Calculate rotation angle
        cos_angle = np.dot(z_axis, direction)
        angle = math.acos(np.clip(cos_angle, -1.0, 1.0))

        # Convert to quaternion
        quat = Quaternion()
        quat.w = math.cos(angle / 2)
        quat.x = rotation_axis[0] * math.sin(angle / 2)
        quat.y = rotation_axis[1] * math.sin(angle / 2)
        quat.z = rotation_axis[2] * math.sin(angle / 2)

        return quat

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

    def _calculate_alignment_quality(
        self,
        positions: List[Point],
        center: Point,
        normal: np.ndarray,
        config: Dict[str, Any],
    ) -> float:
        """Calculate alignment quality score (0.0-1.0)."""
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
        tolerance = config["alignment_tolerance"]
        quality = max(0.0, 1.0 - (max_error / tolerance))

        # Penalize for missing tags
        expected_tags = config["required_tags"]
        detected_tags = len(positions)
        tag_penalty = (expected_tags - detected_tags) / expected_tags
        quality *= 1.0 - tag_penalty * 0.3  # 30% penalty for missing tags

        return min(1.0, max(0.0, quality))

    def _calculate_linear_alignment_quality(
        self,
        positions: List[Point],
        center: Point,
        direction: np.ndarray,
        config: Dict[str, Any],
    ) -> float:
        """Calculate alignment quality for linear layout."""
        if len(positions) < 2:
            return 0.0

        # Calculate how well points fit the line
        line_errors = []
        for pos in positions:
            # Distance from point to line
            point_vec = np.array([pos.x - center.x, pos.y - center.y, pos.z - center.z])
            cross_product = np.cross(point_vec, direction)
            distance = np.linalg.norm(cross_product)
            line_errors.append(distance)

        # Calculate quality based on line fit
        max_error = max(line_errors)
        tolerance = config["alignment_tolerance"]
        quality = max(0.0, 1.0 - (max_error / tolerance))

        return min(1.0, max(0.0, quality))

    def _calculate_tag_errors(self, positions: List[Point], center: Point, normal: np.ndarray) -> List[float]:
        """Calculate individual tag alignment errors."""
        errors = []
        for pos in positions:
            point_vec = np.array([pos.x - center.x, pos.y - center.y, pos.z - center.z])
            distance = abs(np.dot(point_vec, normal))
            errors.append(distance)
        return errors

    def _calculate_linear_tag_errors(self, positions: List[Point], center: Point, direction: np.ndarray) -> List[float]:
        """Calculate individual tag errors for linear alignment."""
        errors = []
        for pos in positions:
            point_vec = np.array([pos.x - center.x, pos.y - center.y, pos.z - center.z])
            cross_product = np.cross(point_vec, direction)
            distance = np.linalg.norm(cross_product)
            errors.append(distance)
        return errors

    def _evaluate_mission_readiness(self, result: Dict[str, Any], config: Dict[str, Any]) -> bool:
        """Evaluate if mission can proceed based on alignment results."""
        if not result.get("success", False):
            return False

        # Check alignment quality threshold
        quality = result.get("alignment_quality", 0.0)
        min_quality = 0.7  # 70% minimum quality
        if quality < min_quality:
            return False

        # Check if we have enough tags
        detected_tags = len(result.get("detected_tag_ids", []))
        min_tags = config["min_tags"]
        if detected_tags < min_tags:
            return False

        return True

    def _generate_alignment_warnings(self, result: Dict[str, Any], config: Dict[str, Any]) -> List[str]:
        """Generate warnings about alignment quality."""
        warnings = []

        quality = result.get("alignment_quality", 0.0)
        if quality < 0.8:
            warnings.append(f"Low alignment quality: {quality:.2f}")

        detected_tags = len(result.get("detected_tag_ids", []))
        required_tags = config["required_tags"]
        if detected_tags < required_tags:
            missing = required_tags - detected_tags
            warnings.append(f"Missing {missing} required tags")

        # Check for high individual tag errors
        errors = result.get("alignment_errors", [])
        if errors:
            max_error = max(errors)
            tolerance = config["alignment_tolerance"]
            if max_error > tolerance:
                warnings.append(f"High alignment error: {max_error:.3f}m > {tolerance:.3f}m")

        return warnings

    def _generate_mission_recommendations(self, result: Dict[str, Any], config: Dict[str, Any]) -> List[str]:
        """Generate recommendations for improving mission success."""
        recommendations = []

        quality = result.get("alignment_quality", 0.0)
        if quality < 0.9:
            recommendations.append("Improve camera positioning for better tag visibility")
            recommendations.append("Ensure adequate lighting on ArUco tags")

        detected_tags = len(result.get("detected_tag_ids", []))
        required_tags = config["required_tags"]
        if detected_tags < required_tags:
            recommendations.append("Move rover closer to target area")
            recommendations.append("Check for obstructions blocking tag visibility")

        # Check for specific mission recommendations
        mission_type = result.get("mission_type", "")
        if mission_type == "AUTONOMOUS_TYPING":
            recommendations.append("Ensure keyboard is flat and well-lit")
            recommendations.append("Verify all 4 corner tags are visible")
        elif mission_type == "USB_CONNECTION":
            recommendations.append("Align USB port with detected tags")
            recommendations.append("Ensure USB port is accessible")

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
            "mission_recommendations": [],
        }

    def create_arm_alignment_command(self, alignment_result: Dict[str, Any], mission_type: str) -> ArmAlignmentCommand:
        """Create arm alignment command from alignment result."""
        cmd = ArmAlignmentCommand()

        # Set mission information
        cmd.mission_type = mission_type
        cmd.alignment_id = f"{mission_type}_{int(time.time())}"

        # Set target information
        cmd.target_position = alignment_result["arm_target_position"]
        cmd.target_orientation = alignment_result["alignment_orientation"]
        cmd.approach_distance = 0.1  # 10cm approach distance
        cmd.final_distance = 0.05  # 5cm final distance

        # Set alignment parameters
        cmd.alignment_quality = alignment_result["alignment_quality"]
        cmd.max_position_error = 0.02  # 2cm position tolerance
        cmd.max_orientation_error = 0.05  # ~3 degrees
        cmd.alignment_timeout = 30.0  # 30 second timeout

        # Set safety parameters
        cmd.max_approach_speed = 0.1  # 10cm/s
        cmd.max_rotation_speed = 0.2  # ~11 degrees/s
        cmd.enable_collision_avoidance = True
        cmd.safety_zones = ["workspace_boundaries"]

        # Set feedback requirements
        cmd.require_position_feedback = True
        cmd.require_force_feedback = True
        cmd.feedback_rate = 10.0  # 10Hz

        # Set mission-specific parameters
        cmd.required_aruco_tags = [str(tag_id) for tag_id in alignment_result.get("detected_tag_ids", [])]
        cmd.tag_visibility_timeout = 5.0  # 5 seconds
        cmd.allow_realignment = True

        return cmd
