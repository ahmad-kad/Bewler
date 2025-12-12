"""
Typing-Specific ArUco Detection Service.

Provides ArUco detection and alignment calculation for autonomous typing missions
using any ArUco tag IDs detected in the scene.
"""

from typing import Any, Dict, Optional

import structlog
from autonomy_interfaces.srv import DetectAruco
from rclpy.client import Client
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from .typing_alignment_calculator import TypingAlignmentCalculator

logger = structlog.get_logger(__name__)


class TypingArUcoDetector:
    """
    Typing-specific ArUco detection service.

    Provides detection and alignment calculation for autonomous typing missions
    using any ArUco tags detected in the scene.
    """

    def __init__(self, node: Node):
        """
        Initialize typing ArUco detector.

        Args:
            node: Parent ROS2 node
        """
        self.node = node

        # Initialize alignment calculator
        self.alignment_calculator = TypingAlignmentCalculator()

        # QoS profile for reliable communication
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Service clients
        self.aruco_detection_client: Client = node.create_client(DetectAruco, "/aruco_detection/detect")

        logger.info("Typing ArUco detector initialized")

    def detect_typing_tags(
        self,
        target_depth: float = None,
        timeout: float = 5.0,
        max_detection_distance: float = 3.0,
    ) -> Dict[str, Any]:
        """
        Detect any ArUco tags for typing alignment.

        Args:
            target_depth: Target depth for alignment (optional)
            timeout: Detection timeout in seconds
            max_detection_distance: Maximum detection distance in meters

        Returns:
            Dictionary with detection and alignment results
        """
        logger.info(
            "Detecting typing tags",
            target_depth=target_depth,
            timeout=timeout,
            max_distance=max_detection_distance,
        )

        # Create detection request - detect ANY tags
        request = DetectAruco.Request()
        request.target_tag_ids = []  # Empty list = detect any tags
        request.detection_timeout = timeout
        request.require_distance_estimate = True
        request.max_detection_distance = max_detection_distance
        request.calculate_alignment = True
        request.target_depth = target_depth or 0.3  # Default 30cm
        request.mission_type = "AUTONOMOUS_TYPING"

        try:
            # Call detection service
            if not self.aruco_detection_client.wait_for_service(timeout_sec=2.0):
                return self._create_error_result("ArUco detection service not available")

            future = self.aruco_detection_client.call_async(request)
            self.node.get_executor().spin_until_future_complete(future, timeout_sec=timeout + 1.0)

            if not future.done():
                return self._create_error_result("Detection request timed out")

            response = future.result()

            if not response.success:
                return self._create_error_result(f"Detection failed: {response.message}")

            # Process detection results
            return self._process_typing_detection_results(response, target_depth)

        except Exception as e:
            logger.error("Typing detection failed", error=str(e))
            return self._create_error_result(f"Detection failed: {str(e)}")

    def detect_typing_tags_with_minimum(
        self,
        min_tags: int = 3,
        target_depth: float = None,
        timeout: float = 5.0,
        max_detection_distance: float = 3.0,
    ) -> Dict[str, Any]:
        """
        Detect ArUco tags for typing with minimum tag requirement.

        Args:
            min_tags: Minimum number of tags required
            target_depth: Target depth for alignment (optional)
            timeout: Detection timeout in seconds
            max_detection_distance: Maximum detection distance in meters

        Returns:
            Dictionary with detection and alignment results
        """
        logger.info(
            "Detecting typing tags with minimum requirement",
            min_tags=min_tags,
            target_depth=target_depth,
        )

        # First attempt: detect any tags
        result = self.detect_typing_tags(target_depth, timeout, max_detection_distance)

        if not result["success"]:
            return result

        # Check if we have enough tags
        detected_count = len(result.get("detected_tag_ids", []))
        if detected_count < min_tags:
            return self._create_error_result(f"Insufficient tags detected: {detected_count} < {min_tags}")

        return result

    def _process_typing_detection_results(
        self, response: DetectAruco.Response, target_depth: float = None
    ) -> Dict[str, Any]:
        """Process detection results and calculate typing alignment."""
        # Extract detected tag information
        detected_tags = []
        for i, tag_id in enumerate(response.detected_tag_ids):
            tag_info = {
                "id": tag_id,
                "position": response.tag_positions[i],
                "distance": response.tag_distances[i],
                "angle": response.tag_angles[i],
            }
            detected_tags.append(tag_info)

        # Calculate typing alignment
        alignment_result = self.alignment_calculator.calculate_typing_alignment(detected_tags, target_depth)

        # Create result
        result = {
            "success": alignment_result["success"],
            "message": alignment_result.get("message", "Detection successful"),
            "mission_type": "AUTONOMOUS_TYPING",
            "detected_tag_ids": response.detected_tag_ids,
            "tag_positions": response.tag_positions,
            "tag_distances": response.tag_distances,
            "tag_angles": response.tag_angles,
            "detection_time": response.detection_time,
            "alignment_available": alignment_result.get("alignment_available", False),
            "alignment_center": alignment_result.get("alignment_center"),
            "alignment_orientation": alignment_result.get("alignment_orientation"),
            "arm_target_position": alignment_result.get("arm_target_position"),
            "alignment_quality": alignment_result.get("alignment_quality", 0.0),
            "alignment_errors": alignment_result.get("alignment_errors", []),
            "mission_ready": alignment_result.get("mission_ready", False),
            "keyboard_info": alignment_result.get("keyboard_info", {}),
            "alignment_warnings": alignment_result.get("alignment_warnings", []),
            "typing_recommendations": alignment_result.get("typing_recommendations", []),
        }

        return result

    def get_typing_alignment_command(self, detection_result: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """Get typing alignment command from detection result."""
        if not detection_result.get("alignment_available", False):
            return None

        return {
            "target_position": detection_result["arm_target_position"],
            "target_orientation": detection_result["alignment_orientation"],
            "alignment_quality": detection_result["alignment_quality"],
            "keyboard_center": detection_result["alignment_center"],
            "keyboard_info": detection_result["keyboard_info"],
            "typing_ready": detection_result["mission_ready"],
        }

    def is_typing_ready(self, detection_result: Dict[str, Any]) -> bool:
        """Check if typing mission is ready based on detection result."""
        return detection_result.get("mission_ready", False)

    def get_typing_status(self, detection_result: Dict[str, Any]) -> Dict[str, Any]:
        """Get typing status information from detection result."""
        return {
            "alignment_quality": detection_result.get("alignment_quality", 0.0),
            "detected_tags": len(detection_result.get("detected_tag_ids", [])),
            "typing_ready": detection_result.get("mission_ready", False),
            "warnings": detection_result.get("alignment_warnings", []),
            "recommendations": detection_result.get("typing_recommendations", []),
            "keyboard_info": detection_result.get("keyboard_info", {}),
        }

    def _create_error_result(self, message: str) -> Dict[str, Any]:
        """Create an error result dictionary."""
        return {
            "success": False,
            "message": message,
            "mission_type": "AUTONOMOUS_TYPING",
            "detected_tag_ids": [],
            "tag_positions": [],
            "tag_distances": [],
            "tag_angles": [],
            "alignment_available": False,
            "mission_ready": False,
            "keyboard_info": {},
            "alignment_warnings": [message],
            "typing_recommendations": [],
        }
