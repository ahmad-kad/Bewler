"""
Dual pipeline ROS 2 node for Pi Zero 2 W.
Runs TFLite INT8 inference and H.264 hardware encoding on the same frame buffer.
Optimized for 512MB RAM constraint with Zenoh RMW.
"""

import os
import subprocess
import time
from typing import Optional, Dict, Any, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np

# Optional imports for different environments
try:
    import cv2

    OPENCV_AVAILABLE = True
except ImportError:
    cv2 = None
    OPENCV_AVAILABLE = False

try:
    import tflite_runtime.interpreter as tflite

    TFLITE_AVAILABLE = True
except ImportError:
    tflite = None
    TFLITE_AVAILABLE = False

try:
    from cv_bridge import CvBridge

    CV_BRIDGE_AVAILABLE = True
except ImportError:
    CvBridge = None
    CV_BRIDGE_AVAILABLE = False

try:
    from sensor_msgs.msg import Image
    from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
    from tf2_ros import Buffer, TransformListener

    ROS_MSGS_AVAILABLE = True
except ImportError:
    Image = None
    CameraInfo = None
    PoseStamped = None
    Detection2DArray = None
    Detection2D = None
    BoundingBox2D = None
    Buffer = None
    TransformListener = None
    TransformException = None
    ROS_MSGS_AVAILABLE = False


class DualPipelineNode(Node):
    """
    ROS 2 node implementing dual pipeline architecture for Pi Zero 2 W.

    Pipeline 1: TFLite INT8 inference for object detection
    Pipeline 2: H.264 hardware encoding for compressed streaming

    Both pipelines share the same frame buffer to minimize memory usage.
    """

    def __init__(self):
        super().__init__("dual_pipeline_node")

        # Declare parameters
        self.declare_parameter("camera_device", "/dev/video0")
        self.declare_parameter("h264_device", "/dev/video11")
        self.declare_parameter("model_path", "")
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("frame_rate", 15.0)
        self.declare_parameter("h264_bitrate", "1M")
        self.declare_parameter("camera_name", "camera")
        self.declare_parameter("calibration_file", "")

        # Get parameters
        self.camera_device = self.get_parameter("camera_device").value
        self.h264_device = self.get_parameter("h264_device").value
        self.model_path = self.get_parameter("model_path").value
        self.frame_width = self.get_parameter("frame_width").value
        self.frame_height = self.get_parameter("frame_height").value
        self.frame_rate = self.get_parameter("frame_rate").value
        self.h264_bitrate = self.get_parameter("h264_bitrate").value
        self.camera_name = self.get_parameter("camera_name").value
        self.calibration_file = self.get_parameter("calibration_file").value

        # Validate dependencies
        if not self._validate_dependencies():
            raise RuntimeError("Missing required dependencies")

        # Initialize components
        self.cv_bridge = CvBridge()
        self.cap = None
        self.encoder_process = None
        self.interpreter = None
        self.tf_buffer = None
        self.tf_listener = None

        # Initialize camera and pipelines
        self._initialize_camera()
        self._initialize_h264_encoder()
        self._initialize_inference()
        self._initialize_tf()

        # Setup QoS for real-time performance
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        # Create publishers
        self.image_pub = self.create_publisher(
            Image, f"/camera/{self.camera_name}/image_raw", qos
        )
        self.compressed_pub = self.create_publisher(
            Image, f"/camera/{self.camera_name}/image_raw/compressed", qos
        )
        self.detection_pub = self.create_publisher(
            Detection2DArray, f"/camera/{self.camera_name}/detections", qos
        )

        # Create timer for frame processing
        self.timer = self.create_timer(1.0 / self.frame_rate, self.process_frame)

        # Load calibration if provided
        if self.calibration_file:
            self.load_calibration()

        self.get_logger().info(
            f"Dual pipeline node initialized for {self.camera_device}"
        )

    def _validate_dependencies(self) -> bool:
        """Validate that all required dependencies are available."""
        missing = []

        if not OPENCV_AVAILABLE:
            missing.append("OpenCV")
        if not TFLITE_AVAILABLE:
            missing.append("TFLite Runtime")
        if not CV_BRIDGE_AVAILABLE:
            missing.append("cv_bridge")
        if not ROS_MSGS_AVAILABLE:
            missing.append("ROS 2 messages")

        if missing:
            self.get_logger().error(f"Missing dependencies: {', '.join(missing)}")
            return False

        return True

    def _initialize_camera(self):
        """Initialize camera capture with optimized settings for Pi Zero."""
        try:
            self.cap = cv2.VideoCapture(self.camera_device)
            if not self.cap.isOpened():
                raise RuntimeError(f"Cannot open camera device {self.camera_device}")

            # Configure for low latency and memory efficiency
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimal buffer

            # Verify settings
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

            self.get_logger().info(
                f"Camera initialized: {actual_width}x{actual_height} @ {self.frame_rate}fps"
            )

        except Exception as e:
            self.get_logger().error(f"Camera initialization failed: {e}")
            raise

    def _initialize_h264_encoder(self):
        """Initialize H.264 hardware encoder process."""
        try:
            ffmpeg_cmd = [
                "ffmpeg",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "bgr24",
                "-s",
                f"{self.frame_width}x{self.frame_height}",
                "-r",
                str(self.frame_rate),
                "-i",
                "-",
                "-c:v",
                "h264_v4l2m2m",
                "-b:v",
                self.h264_bitrate,
                "-maxrate",
                self.h264_bitrate,
                "-f",
                "v4l2",
                self.h264_device,
            ]

            self.encoder_process = subprocess.Popen(
                ffmpeg_cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                bufsize=0,  # Unbuffered for low latency
            )

            self.get_logger().info(
                f"H.264 encoder initialized for device {self.h264_device}"
            )

        except Exception as e:
            self.get_logger().error(f"H.264 encoder initialization failed: {e}")
            raise

    def _initialize_inference(self):
        """Initialize TFLite INT8 interpreter."""
        if not self.model_path:
            self.get_logger().info("No model path provided, inference disabled")
            return

        try:
            self.interpreter = tflite.Interpreter(model_path=self.model_path)
            self.interpreter.allocate_tensors()

            self.input_details = self.interpreter.get_input_details()
            self.output_details = self.interpreter.get_output_details()

            # Log model info
            input_shape = self.input_details[0]["shape"]
            self.get_logger().info(f"TFLite model loaded: input shape {input_shape}")

        except Exception as e:
            self.get_logger().error(f"TFLite initialization failed: {e}")
            raise

    def _initialize_tf(self):
        """Initialize TF2 buffer and listener for coordinate transforms."""
        try:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.get_logger().info("TF2 listener initialized")
        except Exception as e:
            self.get_logger().error(f"TF2 initialization failed: {e}")
            raise

    def load_calibration(self):
        """Load camera intrinsics from JSON file."""
        if not self.calibration_file or not os.path.exists(self.calibration_file):
            self.get_logger().warn(
                f"Calibration file not found: {self.calibration_file}"
            )
            return

        try:
            import json

            with open(self.calibration_file, "r") as f:
                self.calib_data = json.load(f)
            self.get_logger().info(f"Calibration loaded from {self.calibration_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration: {e}")
            self.calib_data = None

    def preprocess_int8(self, frame: np.ndarray) -> np.ndarray:
        """
        Preprocess frame for INT8 quantized model.
        Optimized for memory efficiency and speed.
        """
        # Resize to model input size (assume 224x224 for MobileNet)
        frame = cv2.resize(frame, (224, 224))

        # Normalize to [-1, 1] range for INT8
        frame = frame.astype(np.float32)
        frame = (frame - 128.0) / 128.0

        # Add batch dimension
        return frame[np.newaxis, ...].astype(np.int8)

    def run_inference(self, frame: np.ndarray) -> List[Dict[str, Any]]:
        """Run TFLite inference and return detections."""
        if not self.interpreter:
            return []

        try:
            # Preprocess
            input_tensor = self.preprocess_int8(frame)

            # Run inference
            self.interpreter.set_tensor(self.input_details[0]["index"], input_tensor)
            self.interpreter.invoke()

            # Get results (simplified for demo - adapt to your model output)
            output = self.interpreter.get_tensor(self.output_details[0]["index"])

            # Parse detections (this is model-specific)
            detections = self.parse_detections(output)
            return detections

        except Exception as e:
            self.get_logger().error(f"Inference failed: {e}")
            return []

    def parse_detections(self, output: np.ndarray) -> List[Dict[str, Any]]:
        """Parse model output into detection format (model-specific)."""
        # This is a placeholder - adapt to your specific model's output format
        detections = []

        # Example for SSD MobileNet style output
        # Adapt this based on your actual model
        for i in range(min(10, len(output))):  # Limit to 10 detections
            detection = {
                "class_id": int(output[i][0]),
                "confidence": float(output[i][1]),
                "bbox": {
                    "x": float(output[i][2]),
                    "y": float(output[i][3]),
                    "width": float(output[i][4]),
                    "height": float(output[i][5]),
                },
            }
            detections.append(detection)

        return detections

    def create_detection_msg(
        self, detections: List[Dict[str, Any]], timestamp
    ) -> Detection2DArray:
        """Convert detections to ROS Detection2DArray message."""
        detection_array = Detection2DArray()
        detection_array.header.stamp = timestamp
        detection_array.header.frame_id = f"{self.camera_name}_link"

        for det in detections:
            detection = Detection2D()
            detection.header = detection_array.header

            # Bounding box
            bbox = BoundingBox2D()
            bbox.center.x = det["bbox"]["x"]
            bbox.center.y = det["bbox"]["y"]
            bbox.size_x = det["bbox"]["width"]
            bbox.size_y = det["bbox"]["height"]

            detection.bbox = bbox
            detection_array.detections.append(detection)

        return detection_array

    def process_frame(self):
        """Main frame processing loop - dual pipeline execution."""
        try:
            # Capture frame (single buffer for both pipelines)
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Failed to capture frame")
                return

            timestamp = self.get_clock().now().to_msg()

            # Pipeline 1: Publish raw image for remote processing
            try:
                img_msg = self.cv_bridge.cv2_to_imgmsg(frame, "bgr8")
                img_msg.header.stamp = timestamp
                img_msg.header.frame_id = f"{self.camera_name}_link"
                self.image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Image publishing failed: {e}")

            # Pipeline 2: H.264 encoding (write to encoder process)
            try:
                if self.encoder_process and self.encoder_process.poll() is None:
                    self.encoder_process.stdin.write(frame.tobytes())
            except Exception as e:
                self.get_logger().error(f"H.264 encoding failed: {e}")

            # Pipeline 3: Inference (only if model loaded)
            if self.interpreter:
                try:
                    detections = self.run_inference(frame)
                    if detections:
                        detection_msg = self.create_detection_msg(detections, timestamp)
                        self.detection_pub.publish(detection_msg)
                except Exception as e:
                    self.get_logger().error(f"Inference processing failed: {e}")

        except Exception as e:
            self.get_logger().error(f"Frame processing error: {e}")

    def destroy_node(self):
        """Clean up resources."""
        super().destroy_node()

        # Clean up camera
        if self.cap:
            self.cap.release()

        # Clean up encoder process
        if self.encoder_process:
            try:
                self.encoder_process.stdin.close()
                self.encoder_process.wait(timeout=1.0)
            except Exception:
                self.encoder_process.kill()


def main(args=None):
    """Main entry point for the dual pipeline node."""
    # Set Zenoh RMW for lightweight communication
    os.environ["RMW_IMPLEMENTATION"] = "rmw_zenoh_cpp"

    rclpy.init(args=args)

    try:
        node = DualPipelineNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
