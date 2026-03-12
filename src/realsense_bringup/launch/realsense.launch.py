"""
Launch the Intel RealSense D4xx camera node.

Topics published (used downstream):
  /camera/color/image_raw          - RGB image
  /camera/color/camera_info        - RGB camera intrinsics
  /camera/depth/image_rect_raw     - Aligned depth image
  /camera/depth/camera_info        - Depth camera intrinsics
  /camera/aligned_depth_to_color/image_raw  - Depth aligned to RGB frame
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_no = LaunchConfiguration("serial_no", default="")
    enable_pointcloud = LaunchConfiguration("enable_pointcloud", default="false")

    return LaunchDescription([
        DeclareLaunchArgument("serial_no", default_value="",
                              description="RealSense device serial number (empty = first found)"),
        DeclareLaunchArgument("enable_pointcloud", default_value="false",
                              description="Enable native RealSense point cloud topic"),

        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera",
            namespace="camera",
            parameters=[{
                "serial_no": serial_no,
                # Enable RGB and depth streams
                "enable_color": True,
                "enable_depth": True,
                # Align depth to color frame for pixel-accurate projection
                "align_depth.enable": True,
                # Resolution and FPS — balance quality vs. latency
                "color_width": 640,
                "color_height": 480,
                "color_fps": 30.0,
                "depth_width": 640,
                "depth_height": 480,
                "depth_fps": 30.0,
                # Emit organized point cloud if requested
                "enable_pointcloud": enable_pointcloud,
                "pointcloud.enable": enable_pointcloud,
            }],
            output="screen",
            emulate_tty=True,
        ),
    ])
