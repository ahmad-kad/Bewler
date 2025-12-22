"""
ROS 2 launch file for Pi Zero 2 W dual pipeline system.
Launches camera capture, inference, and H.264 streaming nodes.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    """Generate launch description for Pi Zero 2 W dual pipeline."""

    # Declare launch arguments
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device path'
    )

    h264_device_arg = DeclareLaunchArgument(
        'h264_device',
        default_value='/dev/video11',
        description='H.264 encoder virtual device path'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Path to TFLite INT8 model file'
    )

    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='camera',
        description='Name of the camera'
    )

    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value='',
        description='Path to camera calibration file'
    )

    enable_inference_arg = DeclareLaunchArgument(
        'enable_inference',
        default_value='true',
        description='Enable TFLite inference pipeline'
    )

    enable_streaming_arg = DeclareLaunchArgument(
        'enable_streaming',
        default_value='true',
        description='Enable H.264 streaming pipeline'
    )

    frame_width_arg = DeclareLaunchArgument(
        'frame_width',
        default_value='640',
        description='Frame width for camera capture'
    )

    frame_height_arg = DeclareLaunchArgument(
        'frame_height',
        default_value='480',
        description='Frame height for camera capture'
    )

    frame_rate_arg = DeclareLaunchArgument(
        'frame_rate',
        default_value='15.0',
        description='Frame rate for camera capture'
    )

    ros_domain_id_arg = DeclareLaunchArgument(
        'ros_domain_id',
        default_value='42',
        description='ROS domain ID for network isolation'
    )

    # Define nodes
    dual_pipeline_node = Node(
        package='urc_camera',
        executable='dual_pipeline_node',
        name='dual_pipeline',
        parameters=[{
            'camera_device': LaunchConfiguration('camera_device'),
            'h264_device': LaunchConfiguration('h264_device'),
            'model_path': LaunchConfiguration('model_path'),
            'camera_name': LaunchConfiguration('camera_name'),
            'calibration_file': LaunchConfiguration('calibration_file'),
            'frame_width': LaunchConfiguration('frame_width'),
            'frame_height': LaunchConfiguration('frame_height'),
            'frame_rate': LaunchConfiguration('frame_rate'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_inference')),
        arguments=['--ros-args', '--log-level', 'info']
    )

    # v4l2_camera node for H.264 streaming
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[{
            'video_device': LaunchConfiguration('h264_device'),
            'output_encoding': 'h264',
            'image_transport': 'compressed',
            'publish_rate': LaunchConfiguration('frame_rate'),
        }],
        remappings=[
            ('image_raw', f'/camera/{LaunchConfiguration("camera_name")}/image_raw/compressed'),
            ('image_raw/compressed', f'/camera/{LaunchConfiguration("camera_name")}/image_raw/compressed'),
        ],
        condition=IfCondition(LaunchConfiguration('enable_streaming')),
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Static transform publisher for camera frame
    camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_publisher',
        arguments=[
            '0.0', '0.0', '0.0',  # translation
            '0.0', '0.0', '0.0',  # rotation (rpy)
            'base_link',
            LaunchConfiguration('camera_name') + '_link'
        ],
        parameters=[{
            'use_sim_time': False
        }]
    )

    # Log launch configuration
    log_launch = LogInfo(
        msg=[
            'Launching Pi Zero 2 W dual pipeline with:\n',
            '  Camera device: ', LaunchConfiguration('camera_device'), '\n',
            '  H.264 device: ', LaunchConfiguration('h264_device'), '\n',
            '  Camera name: ', LaunchConfiguration('camera_name'), '\n',
            '  Model path: ', LaunchConfiguration('model_path'), '\n',
            '  Inference enabled: ', LaunchConfiguration('enable_inference'), '\n',
            '  Streaming enabled: ', LaunchConfiguration('enable_streaming'), '\n',
            '  Resolution: ', LaunchConfiguration('frame_width'), 'x', LaunchConfiguration('frame_height'), '\n',
            '  Frame rate: ', LaunchConfiguration('frame_rate'), ' fps\n',
            '  ROS Domain ID: ', LaunchConfiguration('ros_domain_id')
        ]
    )

    # Create launch description
    ld = LaunchDescription([
        # Launch arguments
        camera_device_arg,
        h264_device_arg,
        model_path_arg,
        camera_name_arg,
        calibration_file_arg,
        enable_inference_arg,
        enable_streaming_arg,
        frame_width_arg,
        frame_height_arg,
        frame_rate_arg,
        ros_domain_id_arg,

        # Log configuration
        log_launch,

        # Nodes
        dual_pipeline_node,
        v4l2_camera_node,
        camera_tf_node,
    ])

    return ld
