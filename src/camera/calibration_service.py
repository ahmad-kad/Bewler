"""
ROS 2 calibration service for distributed architecture.
Runs on Pi 5 to provide camera intrinsics to Pi Zero 2 W nodes.
"""

import os
from typing import Optional

import rclpy
from rclpy.node import Node

# Optional ROS imports for different environments
try:
    from urc_interfaces.srv import GetCalibration  # Custom service interface
    CUSTOM_INTERFACE_AVAILABLE = True
except ImportError:
    CUSTOM_INTERFACE_AVAILABLE = False

# Fallback to standard ROS 2 service if custom interface not available
try:
    from std_srvs.srv import Trigger
    from sensor_msgs.msg import CameraInfo
    STD_SERVICES_AVAILABLE = True
except ImportError:
    Trigger = None
    CameraInfo = None
    STD_SERVICES_AVAILABLE = False


class CalibrationService(Node):
    """
    ROS 2 service node that provides camera calibration data to distributed nodes.

    This service runs on the Pi 5 (calibration master) and serves calibration
    data to Pi Zero 2 W nodes upon request. Supports both custom service interface
    and fallback to standard ROS services.
    """

    def __init__(self):
        super().__init__('calibration_service')

        # Declare parameters
        self.declare_parameter('calibration_dir', 'calibrations')
        self.declare_parameter('use_custom_interface', True)

        # Get parameters
        self.calibration_dir = self.get_parameter('calibration_dir').value
        self.use_custom_interface = self.get_parameter('use_custom_interface').value

        # Validate dependencies
        if not self._validate_dependencies():
            raise RuntimeError("Missing required dependencies for calibration service")

        # Create service
        if self.use_custom_interface and CUSTOM_INTERFACE_AVAILABLE:
            self.service = self.create_service(
                GetCalibration,
                '/get_camera_calibration',
                self.get_calibration_callback
            )
            self.get_logger().info('Calibration service initialized with custom interface')
        elif STD_SERVICES_AVAILABLE:
            # Fallback to standard Trigger service
            self.service = self.create_service(
                Trigger,
                '/get_camera_calibration',
                self.get_calibration_trigger_callback
            )
            self.get_logger().info('Calibration service initialized with standard Trigger interface')
        else:
            raise RuntimeError("No suitable service interface available")

        # Publisher for continuous calibration updates
        self.calib_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        self.get_logger().info(f'Calibration service ready, serving from: {self.calibration_dir}')

    def _validate_dependencies(self) -> bool:
        """Validate that required dependencies are available."""
        if self.use_custom_interface and not CUSTOM_INTERFACE_AVAILABLE:
            self.get_logger().warn("Custom interface requested but not available, will use standard services")
            self.use_custom_interface = False

        if not STD_SERVICES_AVAILABLE:
            self.get_logger().error("Standard ROS services not available")
            return False

        return True

    def get_calibration_callback(self, request, response):
        """
        Handle calibration requests using custom service interface.

        Args:
            request: GetCalibration request with camera_name
            response: GetCalibration response

        Returns:
            GetCalibration response with calibration data or error
        """
        camera_name = request.camera_name
        self.get_logger().info(f'Calibration request for camera: {camera_name}')

        try:
            # Load calibration data
            calib_data = self.load_calibration_data(camera_name)

            if calib_data:
                # Convert to CameraInfo message
                response.camera_info = self.calib_data_to_msg(calib_data, camera_name)
                response.success = True
                response.error_message = ""
                self.get_logger().info(f'Successfully served calibration for {camera_name}')
            else:
                response.success = False
                response.error_message = f"Calibration file not found for camera: {camera_name}"
                self.get_logger().warn(response.error_message)

        except Exception as e:
            response.success = False
            response.error_message = f"Error loading calibration: {str(e)}"
            self.get_logger().error(response.error_message)

        return response

    def get_calibration_trigger_callback(self, request, response):
        """
        Handle calibration requests using standard Trigger service.
        This is a fallback when custom interface is not available.

        Note: Trigger service doesn't support parameters, so this will
        return calibration for the default camera only.
        """
        self.get_logger().info('Calibration request via Trigger service')

        # For Trigger service, we need to know which camera is requested
        # This is a limitation - custom interface is preferred
        camera_name = self.get_parameter('default_camera').value or 'camera'

        try:
            calib_data = self.load_calibration_data(camera_name)

            if calib_data:
                # Publish to topic instead of returning in response
                msg = self.calib_data_to_msg(calib_data, camera_name)
                self.calib_pub.publish(msg)

                response.success = True
                response.message = f"Calibration published for {camera_name}"
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = f"Calibration not found for camera: {camera_name}"

        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"

        return response

    def load_calibration_data(self, camera_name: str) -> Optional[dict]:
        """
        Load calibration data from JSON file.

        Args:
            camera_name: Name of the camera

        Returns:
            Calibration data dictionary or None if not found
        """
        try:
            from ..utils.camera import load_calibration_file
            return load_calibration_file(camera_name, self.calibration_dir)
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration for {camera_name}: {e}")
            return None

    def calib_data_to_msg(self, calib_data: dict, camera_name: str) -> CameraInfo:
        """
        Convert calibration data dictionary to CameraInfo message.

        Args:
            calib_data: Calibration data dictionary
            camera_name: Name of the camera for frame_id

        Returns:
            CameraInfo message
        """
        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{calib_data.get('sensor', 'camera')}_{camera_name}_link"

        msg.height = calib_data['image_height']
        msg.width = calib_data['image_width']
        msg.distortion_model = "plumb_bob"
        msg.d = calib_data['distortion_coefficients']
        msg.k = calib_data['camera_matrix']

        # Add rectification matrix (identity if not present)
        if 'rectification_matrix' in calib_data:
            msg.r = calib_data['rectification_matrix']
        else:
            msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        # Add projection matrix
        if 'projection_matrix' in calib_data:
            msg.p = calib_data['projection_matrix']
        else:
            # Create basic projection matrix from intrinsics
            fx = msg.k[0]
            fy = msg.k[4]
            cx = msg.k[2]
            cy = msg.k[5]
            msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        return msg

    def list_available_calibrations(self) -> list:
        """
        List all available camera calibrations.

        Returns:
            List of camera names with available calibrations
        """
        try:
            from ..utils.camera import list_calibrated_cameras
            return list_calibrated_cameras(self.calibration_dir)
        except Exception as e:
            self.get_logger().error(f"Failed to list calibrations: {e}")
            return []

    def publish_all_calibrations(self):
        """Publish all available calibrations to ROS network."""
        cameras = self.list_available_calibrations()

        for camera_name in cameras:
            try:
                calib_data = self.load_calibration_data(camera_name)
                if calib_data:
                    msg = self.calib_data_to_msg(calib_data, camera_name)
                    self.calib_pub.publish(msg)
                    self.get_logger().info(f"Published calibration for {camera_name}")
            except Exception as e:
                self.get_logger().error(f"Failed to publish {camera_name}: {e}")


def main(args=None):
    """Main entry point for the calibration service."""
    rclpy.init(args=args)

    try:
        # Create and spin the service node
        service_node = CalibrationService()

        # Optionally publish all calibrations at startup
        service_node.publish_all_calibrations()

        rclpy.spin(service_node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Service error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
