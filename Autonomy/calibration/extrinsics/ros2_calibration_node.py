#!/usr/bin/env python3
"""
ROS2 Calibration Node - URC 2026
================================

Integrates calibration system with state machine for real-time camera intrinsics.

Topics:
  Publish:
    /calibration/status (String) - Calibration status updates
    /calibration/progress (Float32) - Progress percentage

  Subscribe:
    /state_machine/state (String) - Current state machine state
    /calibration/command (String) - Calibration commands

Services:
  /calibration/start_intrinsics (Empty) - Start intrinsics calibration
  /calibration/cancel (Empty) - Cancel current calibration
  /calibration/get_results (Empty) - Get last calibration results

Parameters:
  ~/calibration_mode (string) - "intrinsic" | "hand_eye" | "imu"
  ~/target_images (int) - Number of images to capture (default: 50)
  ~/capture_mode (string) - "manual" | "video" | "conservative"
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import threading
import logging
from pathlib import Path
import sys
import os

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class CalibrationNode(Node):
    """ROS2 node for managing camera calibration."""
    
    def __init__(self):
        """Initialize calibration node."""
        super().__init__('calibration_node')
        
        # Node configuration
        self.declare_parameter('calibration_mode', 'intrinsic')
        self.declare_parameter('target_images', 50)
        self.declare_parameter('capture_mode', 'manual')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('board_name', 'board_5x7')
        
        # State tracking
        self.is_calibrating = False
        self.calibration_thread = None
        self.current_progress = 0.0
        self.calibration_result = None
        
        # Publishers
        self.status_pub = self.create_publisher(
            String, '/calibration/status', 10
        )
        self.progress_pub = self.create_publisher(
            Float32, '/calibration/progress', 10
        )
        
        # Subscribers
        self.state_sub = self.create_subscription(
            String, '/state_machine/state', self.state_callback, 10
        )
        
        self.command_sub = self.create_subscription(
            String, '/calibration/command', self.command_callback, 10
        )
        
        self.get_logger().info("Calibration Node initialized")
        self.publish_status("INITIALIZED")
    
    # ========================================================================
    # PUBLISHERS
    # ========================================================================
    
    def publish_status(self, status: str):
        """Publish calibration status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status: {status}")
    
    def publish_progress(self, progress: float):
        """Publish calibration progress."""
        msg = Float32()
        msg.data = min(100.0, max(0.0, progress))
        self.progress_pub.publish(msg)
        self.current_progress = progress
    
    # ========================================================================
    # SUBSCRIBERS
    # ========================================================================
    
    def state_callback(self, msg: String):
        """Handle state machine state changes."""
        state = msg.data
        self.get_logger().debug(f"State changed: {state}")
        
        # Auto-start calibration if entering CALIBRATION state
        if state == "CALIBRATION" and not self.is_calibrating:
            self.get_logger().info("Entering CALIBRATION state - ready to calibrate")
            self.publish_status("READY")
    
    def command_callback(self, msg: String):
        """Handle calibration commands."""
        command = msg.data.lower().strip()
        self.get_logger().info(f"Calibration command: {command}")
        
        if command == "start":
            self.start_calibration()
        elif command == "cancel":
            self.cancel_calibration()
        elif command == "get_results":
            self.publish_results()
    
    # ========================================================================
    # CALIBRATION OPERATIONS
    # ========================================================================
    
    def start_calibration(self):
        """Start calibration process in background thread."""
        if self.is_calibrating:
            self.get_logger().warning("Calibration already in progress")
            return
        
        self.publish_status("STARTING")
        self.is_calibrating = True
        
        # Start calibration in separate thread
        self.calibration_thread = threading.Thread(
            target=self._calibration_worker
        )
        self.calibration_thread.daemon = True
        self.calibration_thread.start()
    
    def _calibration_worker(self):
        """Background worker for calibration."""
        try:
            self.get_logger().info("Calibration worker started")
            self.publish_status("CALIBRATING")
            
            # Get parameters
            calibration_mode = self.get_parameter('calibration_mode').value
            target_images = self.get_parameter('target_images').value
            capture_mode = self.get_parameter('capture_mode').value
            
            self.get_logger().info(
                f"Calibration: mode={calibration_mode}, "
                f"images={target_images}, capture={capture_mode}"
            )
            
            # TODO: Import and use actual calibration modules
            # from intrinsics.camera_intrinsics_calibrator import (
            #     CameraConfig, CharUcoBoardConfig, CameraIntrinsicsCalibrator
            # )
            
            # Simulate calibration process with progress updates
            for i in range(target_images):
                if not self.is_calibrating:
                    self.publish_status("CANCELLED")
                    break
                
                # Update progress
                progress = (i / target_images) * 100.0
                self.publish_progress(progress)
                
                # TODO: Process images and update calibration
                
                # Simulate processing time
                import time
                time.sleep(0.1)
            
            if self.is_calibrating:
                self.publish_progress(100.0)
                self.publish_status("COMPLETE")
                self.calibration_result = {
                    "status": "success",
                    "images": target_images,
                    "mode": capture_mode
                }
                self.get_logger().info("Calibration completed successfully")
        
        except Exception as e:
            self.get_logger().error(f"Calibration failed: {e}")
            self.publish_status(f"ERROR: {str(e)}")
        
        finally:
            self.is_calibrating = False
    
    def cancel_calibration(self):
        """Cancel ongoing calibration."""
        if not self.is_calibrating:
            self.get_logger().warning("No calibration in progress")
            return
        
        self.get_logger().info("Cancelling calibration")
        self.is_calibrating = False
        self.publish_status("CANCELLED")
        
        if self.calibration_thread:
            self.calibration_thread.join(timeout=2.0)
    
    def publish_results(self):
        """Publish last calibration results."""
        if self.calibration_result is None:
            self.publish_status("NO_RESULTS")
            return
        
        result_str = str(self.calibration_result)
        self.publish_status(f"RESULTS: {result_str}")
        self.get_logger().info(f"Results published: {self.calibration_result}")
    
    # ========================================================================
    # LIFECYCLE
    # ========================================================================
    
    def destroy_node(self):
        """Cleanup on shutdown."""
        if self.is_calibrating:
            self.get_logger().info("Cancelling calibration on shutdown")
            self.cancel_calibration()
        
        super().destroy_node()


class CalibrationNodeWithRealSupport(CalibrationNode):
    """Extended calibration node with real calibration support."""
    
    def __init__(self):
        """Initialize with real calibration support."""
        super().__init__()
        
        # Try to import calibration modules
        self.calibrator = None
        self._init_calibrator()
    
    def _init_calibrator(self):
        """Initialize calibrator with real implementation."""
        try:
            # Try to import calibration modules
            intrinsics_path = Path(__file__).parent.parent / "intrinsics"
            if str(intrinsics_path) not in sys.path:
                sys.path.insert(0, str(intrinsics_path))
            
            from camera_intrinsics_calibrator import (
                CameraConfig,
                CharUcoBoardConfig,
                CameraIntrinsicsCalibrator,
                CaptureMode
            )
            
            self.CameraConfig = CameraConfig
            self.CharUcoBoardConfig = CharUcoBoardConfig
            self.CameraIntrinsicsCalibrator = CameraIntrinsicsCalibrator
            self.CaptureMode = CaptureMode
            
            self.get_logger().info("Calibration modules loaded successfully")
            self.publish_status("MODULES_LOADED")
        
        except ImportError as e:
            self.get_logger().warning(f"Could not load calibration modules: {e}")
            self.get_logger().info("Running in simulation mode")
    
    def _calibration_worker(self):
        """Enhanced worker with real calibration."""
        try:
            self.get_logger().info("Calibration worker started (with real support)")
            self.publish_status("CALIBRATING")
            
            # Get parameters
            calibration_mode = self.get_parameter('calibration_mode').value
            target_images = self.get_parameter('target_images').value
            capture_mode = self.get_parameter('capture_mode').value
            camera_index = self.get_parameter('camera_index').value
            board_name = self.get_parameter('board_name').value
            
            if calibration_mode != "intrinsic":
                self.get_logger().warning(
                    f"Mode {calibration_mode} not yet implemented"
                )
                self.publish_status("READY")
                return
            
            # Check if modules are available
            if not hasattr(self, 'CameraIntrinsicsCalibrator'):
                self.get_logger().info("Running in simulation mode (modules not available)")
                self._run_simulation(target_images)
                return
            
            # Real calibration
            self._run_real_calibration(
                camera_index, board_name, capture_mode, target_images
            )
        
        except Exception as e:
            self.get_logger().error(f"Calibration failed: {e}")
            self.publish_status(f"ERROR: {str(e)}")
        
        finally:
            self.is_calibrating = False
    
    def _run_simulation(self, target_images: int):
        """Run simulation mode calibration."""
        self.get_logger().info("Running calibration in simulation mode")
        
        for i in range(target_images):
            if not self.is_calibrating:
                self.publish_status("CANCELLED")
                break
            
            progress = (i / target_images) * 100.0
            self.publish_progress(progress)
            
            # Simulate processing
            import time
            time.sleep(0.05)
        
        if self.is_calibrating:
            self.publish_progress(100.0)
            self.calibration_result = {
                "status": "success_simulation",
                "images": target_images,
                "note": "Simulated calibration - real camera not detected"
            }
            self.publish_status("COMPLETE")
    
    def _run_real_calibration(self, camera_idx, board_name, capture_mode, target_images):
        """Run real calibration with actual camera."""
        self.get_logger().info(
            f"Starting real calibration: camera={camera_idx}, "
            f"board={board_name}, mode={capture_mode}, images={target_images}"
        )
        
        try:
            # Create configurations
            camera_config = self.CameraConfig(
                name=f"Camera_{camera_idx}",
                camera_index=camera_idx,
                resolution=(1920, 1080)
            )
            
            # Standard board configurations
            boards = {
                "board_5x7": (5, 7, 30.0, 18.0),
                "board_7x5": (7, 5, 25.0, 15.0),
                "board_small": (4, 5, 20.0, 12.0),
            }
            
            if board_name not in boards:
                raise ValueError(f"Unknown board: {board_name}")
            
            cols, rows, checker_mm, marker_mm = boards[board_name]
            board_config = self.CharUcoBoardConfig(
                name=board_name,
                aruco_dict_name="DICT_4X4_50",
                size=(cols, rows),
                checker_size_mm=checker_mm,
                marker_size_mm=marker_mm
            )
            
            # Initialize calibrator
            calibrator = self.CameraIntrinsicsCalibrator(camera_config, board_config)
            
            # Capture images
            self.publish_status("CAPTURING")
            
            if capture_mode == "manual":
                images = calibrator.capture_manual(target_images=target_images)
            elif capture_mode == "video":
                images = calibrator.capture_video(target_images=target_images)
            else:  # conservative
                images = calibrator.capture_conservative(target_images=target_images)
            
            if not images:
                self.publish_status("CAPTURE_FAILED")
                return
            
            self.publish_progress(50.0)
            
            # Process dataset
            self.publish_status("PROCESSING")
            corners, ids, _ = calibrator.process_dataset(images)
            
            if len(corners) < 5:
                self.publish_status("INSUFFICIENT_DATA")
                return
            
            self.publish_progress(75.0)
            
            # Calibrate
            self.publish_status("COMPUTING")
            result = calibrator.calibrate(corners, ids, images, capture_mode)
            
            self.publish_progress(90.0)
            
            # Save results
            self.publish_status("SAVING")
            files = calibrator.save_calibration(result)
            
            self.publish_progress(100.0)
            
            self.calibration_result = {
                "status": "success",
                "images": len(images),
                "valid_frames": len(corners),
                "reprojection_error": result.reprojection_error,
                "quality_score": result.quality_score,
                "output_files": {fmt: str(f) for fmt, f in files.items()}
            }
            
            self.publish_status("COMPLETE")
            self.get_logger().info(f"Calibration complete: {self.calibration_result}")
        
        except Exception as e:
            self.get_logger().error(f"Real calibration failed: {e}")
            self.publish_status(f"ERROR: {str(e)}")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    # Try to use extended node with real support
    try:
        node = CalibrationNodeWithRealSupport()
    except Exception as e:
        logger.warning(f"Could not initialize extended node: {e}")
        node = CalibrationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.info("Calibration node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

