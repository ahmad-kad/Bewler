#!/usr/bin/env python3
"""
Calibration Manager Node for URC 2026
Manages camera calibration, parameter loading, and status monitoring
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from autonomy_interfaces.srv import CalibrateCamera, LoadCalibrationParameters, ValidateCalibration
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo

import yaml
import os
import json
from datetime import datetime

from .calibration_data_manager import CalibrationDataManager


class CalibrationManager(Node):
    """Main calibration management node with parameter management and status monitoring."""

    def __init__(self):
        super().__init__("calibration_manager")

        # Create callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters
        self.declare_parameter("calibration_data_directory", "/home/robotics2025/Autonomy/calibration_data")
        self.declare_parameter("camera_intrinsics_file", "camera_intrinsics.yaml")
        self.declare_parameter("hand_eye_transform_file", "hand_eye_transform.npz")
        self.declare_parameter("calibration_status_topic", "/calibration/status")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")

        # Get parameter values
        self.calibration_data_dir = self.get_parameter("calibration_data_directory").value
        self.status_topic = self.get_parameter("calibration_status_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value

        # Initialize data manager
        self.data_manager = CalibrationDataManager(self.calibration_data_dir)

        # Create services
        self.calibrate_camera_srv = self.create_service(
            CalibrateCamera,
            "calibration/calibrate_camera",
            self.calibrate_camera_callback,
            callback_group=self.callback_group,
        )

        self.load_parameters_srv = self.create_service(
            LoadCalibrationParameters,
            "calibration/load_parameters",
            self.load_parameters_callback,
            callback_group=self.callback_group,
        )

        self.validate_calibration_srv = self.create_service(
            ValidateCalibration,
            "calibration/validate_calibration",
            self.validate_calibration_callback,
            callback_group=self.callback_group,
        )

        # Publishers
        self.camera_info_pub = self.create_publisher(
            CameraInfo, self.camera_info_topic, 10, callback_group=self.callback_group
        )

        self.status_pub = self.create_publisher(String, self.status_topic, 10, callback_group=self.callback_group)

        # Initialize calibration state
        self.current_calibration = None
        self.calibration_loaded = False
        self.last_calibration_check = datetime.now()
        self.calibration_health_score = 0.0
        self.drift_detected = False

        # Status monitoring parameters
        self.declare_parameter("enable_health_monitoring", True)
        self.declare_parameter("status_check_interval", 30.0)
        self.declare_parameter("drift_detection_threshold", 0.5)  # pixels

        # Start status monitoring timer
        status_interval = self.get_parameter("status_check_interval").value
        self.create_timer(status_interval, self.status_check_callback, callback_group=self.callback_group)

        self.get_logger().info("Calibration Manager node initialized")
        self.publish_status("Calibration manager initialized")

    def calibrate_camera_callback(self, request, response):
        """Handle camera calibration requests."""
        try:
            self.get_logger().info(f"Starting camera calibration from directory: {request.image_directory}")

            # Import calibration tools dynamically
            import sys

            sys.path.append("/home/robotics2025/Autonomy/calibration")

            from charuco_board.camera_calibrator import CameraCalibrator

            # Create calibrator and run calibration
            calibrator = CameraCalibrator()
            results = calibrator.calibrate_from_images(
                request.image_directory,
                (request.squares_x, request.squares_y),
                request.square_size,
                request.marker_size,
            )

            if results["calibration_success"]:
                # Save calibration data using data manager
                output_file = self.data_manager.save_calibration(
                    results,
                    "camera_intrinsics",
                    f"ChArUco calibration: {results['squares_x']}x{results['squares_y']} board",
                )

                # Store current calibration
                self.current_calibration = results
                self.calibration_loaded = True

                response.success = True
                response.result_file = output_file
                response.calibration_summary = (
                    f"Calibration successful: RMS={results['reprojection_error']:.4f}px, "
                    f"Quality={results['quality_rating'].upper()}"
                )

                self.publish_status(f"Camera calibration completed: {response.calibration_summary}")
                self.get_logger().info(f"Calibration completed and saved to: {output_file}")
            else:
                response.success = False
                response.error_message = "Calibration failed - check image quality and board detection"
                self.publish_status("Camera calibration failed")
                self.get_logger().error("Camera calibration failed")

        except Exception as e:
            response.success = False
            response.error_message = str(e)
            self.publish_status(f"Camera calibration error: {str(e)}")
            self.get_logger().error(f"Camera calibration error: {str(e)}")

        return response

    def load_parameters_callback(self, request, response):
        """Load calibration parameters into ROS."""
        try:
            self.get_logger().info(f"Loading calibration parameters from: {request.calibration_file}")

            # Load calibration data using data manager if it's a managed file
            if os.path.basename(request.calibration_file).startswith(("camera_intrinsics", "hand_eye")):
                # It's a managed calibration file
                cal_type = "camera_intrinsics" if "camera" in request.calibration_file else "hand_eye"
                calib_data = self.data_manager.load_calibration(cal_type, os.path.basename(request.calibration_file))
                if calib_data is None:
                    response.success = False
                    response.error_message = f"Could not load managed calibration: {request.calibration_file}"
                    return response
            else:
                # Load directly from file
                with open(request.calibration_file, "r") as f:
                    calib_data = yaml.safe_load(f)

            # Create CameraInfo message
            camera_info = CameraInfo()
            camera_info.width = calib_data["image_width"]
            camera_info.height = calib_data["image_height"]
            camera_info.k = calib_data["camera_matrix"]["data"]  # 3x3 row-major
            camera_info.d = calib_data["distortion_coefficients"]["data"]  # distortion coefficients

            # Publish camera info
            self.camera_info_pub.publish(camera_info)

            # Store current calibration
            self.current_calibration = calib_data
            self.calibration_loaded = True

            # Set ROS parameters if namespace provided
            if request.parameter_namespace:
                self.set_parameters(
                    [
                        rclpy.parameter.Parameter(
                            f"{request.parameter_namespace}.camera_matrix",
                            rclpy.parameter.Parameter.Type.DOUBLE_ARRAY,
                            calib_data["camera_matrix"]["data"],
                        ),
                        rclpy.parameter.Parameter(
                            f"{request.parameter_namespace}.distortion_coefficients",
                            rclpy.parameter.Parameter.Type.DOUBLE_ARRAY,
                            calib_data["distortion_coefficients"]["data"],
                        ),
                        rclpy.parameter.Parameter(
                            f"{request.parameter_namespace}.image_width",
                            rclpy.parameter.Parameter.Type.INTEGER,
                            calib_data["image_width"],
                        ),
                        rclpy.parameter.Parameter(
                            f"{request.parameter_namespace}.image_height",
                            rclpy.parameter.Parameter.Type.INTEGER,
                            calib_data["image_height"],
                        ),
                    ]
                )

            response.success = True
            self.publish_status(f"Calibration parameters loaded from {os.path.basename(request.calibration_file)}")
            self.get_logger().info("Calibration parameters loaded successfully")

        except Exception as e:
            response.success = False
            response.error_message = str(e)
            self.publish_status(f"Parameter loading error: {str(e)}")
            self.get_logger().error(f"Parameter loading error: {str(e)}")

        return response

    def validate_calibration_callback(self, request, response):
        """Validate existing calibration quality."""
        try:
            self.get_logger().info(f"Validating calibration: {request.calibration_file}")

            # Import validation tools
            import sys

            sys.path.append("/home/robotics2025/Autonomy/calibration")

            from calibration_validator import CalibrationValidator

            validator = CalibrationValidator()
            calibration = validator.load_calibration(request.calibration_file)

            # Test reprojection error
            if request.test_images:
                results = validator.test_reprojection_error(
                    calibration, request.test_images, (10, 7), 0.020  # Assume ChArUco board  # Assume 20mm squares
                )

                response.success = True
                response.reprojection_error = results.get("mean_reprojection_error", float("inf"))
                response.quality_assessment = results.get("quality_assessment", "unknown")

                # Generate recommendations
                recommendations = []
                if response.reprojection_error > 1.0:
                    recommendations.append("High reprojection error - recalibrate with better images")
                if response.quality_assessment in ["poor", "acceptable"]:
                    recommendations.append("Consider recalibrating with more/better images")

                response.recommendations = "; ".join(recommendations)

                self.publish_status(
                    f"Calibration validation: {response.quality_assessment.upper()}, RMS={response.reprojection_error:.4f}px"
                )
                self.get_logger().info(f"Calibration validation completed: {response.quality_assessment}")
            else:
                response.success = False
                response.error_message = "No test images provided for validation"
                self.get_logger().error("No test images provided for validation")

        except Exception as e:
            response.success = False
            response.error_message = str(e)
            self.publish_status(f"Calibration validation error: {str(e)}")
            self.get_logger().error(f"Calibration validation error: {str(e)}")

        return response

    def status_check_callback(self):
        """Periodic status check and health monitoring."""
        try:
            # Get calibration statistics
            stats = self.data_manager.get_calibration_stats()

            # Assess calibration health
            health_info = self._assess_calibration_health()

            status_info = {
                "timestamp": datetime.now().isoformat(),
                "node_status": "active",
                "calibration_loaded": self.calibration_loaded,
                "calibration_health_score": self.calibration_health_score,
                "drift_detected": self.drift_detected,
                "data_stats": stats,
                "health_assessment": health_info,
                "last_check": self.last_calibration_check.isoformat(),
            }

            if self.current_calibration:
                status_info.update(
                    {
                        "current_calibration_type": self.current_calibration.get("board_type", "unknown"),
                        "quality_rating": self.current_calibration.get("quality_rating", "unknown"),
                        "reprojection_error": self.current_calibration.get("reprojection_error", None),
                        "calibration_date": self.current_calibration.get("calibration_date", None),
                    }
                )

            # Publish status
            status_msg = String()
            status_msg.data = json.dumps(status_info, default=str)
            self.status_pub.publish(status_msg)

            self.last_calibration_check = datetime.now()

            # Log status periodically (less frequent than publishing)
            if self.get_clock().now().nanoseconds % (300 * 1e9) < (30 * 1e9):  # Every 5 minutes
                self.get_logger().info(
                    f"Calibration health: score={self.calibration_health_score:.2f}, "
                    f"drift={self.drift_detected}, loaded={self.calibration_loaded}"
                )

                # Alert on critical issues
                if self.calibration_health_score < 0.3:
                    self.get_logger().warn("Calibration health is CRITICAL - recalibration recommended")
                elif self.drift_detected:
                    self.get_logger().warn("Calibration drift detected - validation recommended")

        except Exception as e:
            self.get_logger().error(f"Status check error: {str(e)}")
            # Publish error status
            error_status = {"timestamp": datetime.now().isoformat(), "node_status": "error", "error": str(e)}
            status_msg = String()
            status_msg.data = json.dumps(error_status)
            self.status_pub.publish(status_msg)

    def _assess_calibration_health(self) -> Dict[str, Any]:
        """Assess the overall health of the calibration system."""
        health = {"overall_score": 0.0, "issues": [], "recommendations": [], "last_validation": None}

        try:
            score_components = []
            issues = []
            recommendations = []

            # Check if calibration is loaded
            if not self.calibration_loaded:
                issues.append("No calibration data loaded")
                recommendations.append("Load or perform camera calibration")
                score_components.append(0.0)
            else:
                score_components.append(1.0)

            # Check calibration quality
            if self.current_calibration:
                quality = self.current_calibration.get("quality_rating", "unknown")
                if quality == "excellent":
                    score_components.append(1.0)
                elif quality == "good":
                    score_components.append(0.8)
                elif quality == "acceptable":
                    score_components.append(0.6)
                elif quality == "poor":
                    score_components.append(0.2)
                    issues.append("Poor calibration quality")
                    recommendations.append("Recalibrate camera with better conditions")
                else:
                    score_components.append(0.5)

                # Check reprojection error
                error = self.current_calibration.get("reprojection_error", float("inf"))
                if error > 1.0:
                    issues.append(f"High reprojection error: {error:.4f}px")
                    recommendations.append("Recalibrate to reduce projection errors")

                # Check calibration age (older than 30 days?)
                cal_date = self.current_calibration.get("calibration_date")
                if cal_date:
                    try:
                        cal_datetime = datetime.fromisoformat(cal_date.replace("Z", "+00:00"))
                        age_days = (datetime.now() - cal_datetime).days
                        if age_days > 30:
                            issues.append(f"Calibration is {age_days} days old")
                            recommendations.append("Consider recalibration for accuracy")
                    except BaseException:
                        pass
            else:
                score_components.append(0.0)

            # Check data integrity
            try:
                validation = self.data_manager.validate_calibration(self.current_calibration or {})
                if not validation["valid"]:
                    issues.extend(validation["errors"])
                    recommendations.append("Check calibration data integrity")
                    score_components.append(0.5)
                else:
                    score_components.append(1.0)
            except BaseException:
                score_components.append(0.5)

            # Calculate overall score
            if score_components:
                health["overall_score"] = sum(score_components) / len(score_components)
            else:
                health["overall_score"] = 0.0

            health["issues"] = issues
            health["recommendations"] = recommendations

            # Update instance variables
            self.calibration_health_score = health["overall_score"]

            # Check for drift (simplified - would need actual validation)
            self.drift_detected = health["overall_score"] < 0.5

        except Exception as e:
            health["issues"].append(f"Health assessment error: {str(e)}")
            health["overall_score"] = 0.0

        return health

    def publish_status(self, message: str):
        """Publish a status message."""
        try:
            status_msg = String()
            status_msg.data = json.dumps(
                {"timestamp": datetime.now().isoformat(), "message": message, "node": "calibration_manager"}
            )
            self.status_pub.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f"Status publish error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    node = CalibrationManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info("Calibration Manager starting...")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down calibration manager")
    except Exception as e:
        node.get_logger().error(f"Calibration manager error: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
