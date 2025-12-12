"""
Mission-Specific ArUco Detection Service.

Provides ArUco detection and alignment calculation for specific missions
like autonomous typing and USB connection.
"""

from typing import Any, Dict, List, Optional

import structlog
from autonomy_interfaces.msg import ArmAlignmentCommand
from autonomy_interfaces.srv import DetectAruco, DetectMissionAruco
from rclpy.client import Client
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

from .aruco_alignment_calculator import ArUcoAlignmentCalculator

logger = structlog.get_logger(__name__)


class MissionArUcoDetector:
    """
    Mission-specific ArUco detection service.

    Provides detection and alignment calculation for specific missions
    requiring precise arm positioning relative to ArUco tags.
    """

    def __init__(self, node: Node):
        """
        Initialize mission ArUco detector.

        Args:
            node: Parent ROS2 node
        """
        self.node = node

        # Initialize alignment calculator
        self.alignment_calculator = ArUcoAlignmentCalculator()

        # QoS profile for reliable communication
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Service clients
        self.aruco_detection_client: Client = node.create_client(DetectAruco, "/aruco_detection/detect")

        # Service servers
        self.mission_detection_service = node.create_service(
            DetectMissionAruco, "/mission_aruco/detect", self._handle_mission_detection
        )

        # Publishers
        self.alignment_command_publisher = node.create_publisher(
            ArmAlignmentCommand, "/arm/alignment_command", self.qos
        )

        logger.info("Mission ArUco detector initialized")

    def detect_mission_tags(
        self,
        mission_type: str,
        required_tag_ids: List[int],
        optional_tag_ids: List[int] = None,
        target_depth: float = None,
        timeout: float = 5.0,
    ) -> Dict[str, Any]:
        """
        Detect ArUco tags for a specific mission with alignment calculation.

        Args:
            mission_type: Type of mission (AUTONOMOUS_TYPING, USB_CONNECTION, etc.)
            required_tag_ids: List of required ArUco tag IDs
            optional_tag_ids: List of optional ArUco tag IDs
            target_depth: Target depth for alignment (optional)
            timeout: Detection timeout in seconds

        Returns:
            Dictionary with detection and alignment results
        """
        logger.info(
            "Detecting mission tags",
            mission_type=mission_type,
            required_tags=required_tag_ids,
            optional_tags=optional_tag_ids,
        )

        if optional_tag_ids is None:
            optional_tag_ids = []

        # Combine all tag IDs
        all_tag_ids = required_tag_ids + optional_tag_ids

        # Create detection request
        request = DetectAruco.Request()
        request.target_tag_ids = all_tag_ids
        request.detection_timeout = timeout
        request.require_distance_estimate = True
        request.max_detection_distance = 5.0  # 5m max detection distance
        request.calculate_alignment = True
        request.target_depth = target_depth or 0.3  # Default 30cm
        request.mission_type = mission_type

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
            return self._process_detection_results(response, mission_type, required_tag_ids, optional_tag_ids)

        except Exception as e:
            logger.error("Mission detection failed", error=str(e))
            return self._create_error_result(f"Detection failed: {str(e)}")

    def _process_detection_results(
        self,
        response: DetectAruco.Response,
        mission_type: str,
        required_tag_ids: List[int],
        optional_tag_ids: List[int],
    ) -> Dict[str, Any]:
        """Process detection results and calculate alignment."""
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

        # Calculate alignment
        alignment_result = self.alignment_calculator.calculate_alignment(detected_tags, mission_type)

        # Determine which tags were detected
        detected_required = [tag_id for tag_id in required_tag_ids if tag_id in response.detected_tag_ids]
        detected_optional = [tag_id for tag_id in optional_tag_ids if tag_id in response.detected_tag_ids]
        missing_required = [tag_id for tag_id in required_tag_ids if tag_id not in response.detected_tag_ids]

        # Create result
        result = {
            "success": alignment_result["success"],
            "message": alignment_result.get("message", "Detection successful"),
            "mission_type": mission_type,
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
            "missing_required_tags": missing_required,
            "detected_optional_tags": detected_optional,
            "alignment_warnings": alignment_result.get("alignment_warnings", []),
            "mission_recommendations": alignment_result.get("mission_recommendations", []),
        }

        return result

    def create_arm_alignment_command(self, detection_result: Dict[str, Any]) -> Optional[ArmAlignmentCommand]:
        """Create arm alignment command from detection result."""
        if not detection_result.get("alignment_available", False):
            return None

        mission_type = detection_result.get("mission_type", "UNKNOWN")

        # Create alignment command
        cmd = self.alignment_calculator.create_arm_alignment_command(detection_result, mission_type)

        # Set header
        cmd.header = Header()
        cmd.header.stamp = self.node.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"

        return cmd

    def publish_alignment_command(self, detection_result: Dict[str, Any]) -> bool:
        """Publish arm alignment command based on detection result."""
        cmd = self.create_arm_alignment_command(detection_result)
        if cmd is None:
            logger.warning("Cannot create alignment command from detection result")
            return False

        self.alignment_command_publisher.publish(cmd)
        logger.info("Published arm alignment command", mission_type=cmd.mission_type)
        return True

    def _handle_mission_detection(
        self, request: DetectMissionAruco.Request, response: DetectMissionAruco.Response
    ) -> DetectMissionAruco.Response:
        """Handle mission-specific ArUco detection service requests."""
        logger.info(
            "Mission detection request",
            mission_type=request.mission_type,
            required_tags=request.required_tag_ids,
            optional_tags=request.optional_tag_ids,
        )

        try:
            # Perform detection
            result = self.detect_mission_tags(
                mission_type=request.mission_type,
                required_tag_ids=list(request.required_tag_ids),
                optional_tag_ids=list(request.optional_tag_ids),
                target_depth=request.target_depth,
                timeout=request.detection_timeout,
            )

            # Populate response
            response.success = result["success"]
            response.message = result["message"]
            response.mission_type = result["mission_type"]
            response.detected_tag_ids = result["detected_tag_ids"]
            response.tag_positions = result["tag_positions"]
            response.tag_distances = result["tag_distances"]
            response.tag_angles = result["tag_angles"]
            response.detection_time = result["detection_time"]
            response.alignment_available = result["alignment_available"]
            response.alignment_center = result["alignment_center"]
            response.alignment_orientation = result["alignment_orientation"]
            response.arm_target_position = result["arm_target_position"]
            response.alignment_quality = result["alignment_quality"]
            response.alignment_errors = result["alignment_errors"]
            response.mission_ready = result["mission_ready"]
            response.missing_required_tags = result["missing_required_tags"]
            response.detected_optional_tags = result["detected_optional_tags"]
            response.alignment_warnings = result["alignment_warnings"]
            response.mission_recommendations = result["mission_recommendations"]

            # Publish alignment command if successful
            if result["success"] and result["alignment_available"]:
                self.publish_alignment_command(result)

        except Exception as e:
            logger.error("Mission detection service failed", error=str(e))
            response.success = False
            response.message = f"Service error: {str(e)}"

        return response

    def _create_error_result(self, message: str) -> Dict[str, Any]:
        """Create an error result dictionary."""
        return {
            "success": False,
            "message": message,
            "mission_type": "UNKNOWN",
            "detected_tag_ids": [],
            "tag_positions": [],
            "tag_distances": [],
            "tag_angles": [],
            "alignment_available": False,
            "mission_ready": False,
            "missing_required_tags": [],
            "detected_optional_tags": [],
            "alignment_warnings": [message],
            "mission_recommendations": [],
        }
