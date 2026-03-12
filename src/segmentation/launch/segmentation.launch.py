from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="yolov8n-seg.pt",
                              description="Path to YOLOv8-seg .pt model file"),
        DeclareLaunchArgument("confidence", default_value="0.5"),
        DeclareLaunchArgument("target_class", default_value="-1",
                              description="COCO class id to detect (-1 = all)"),
        DeclareLaunchArgument("device", default_value="cpu"),

        Node(
            package="segmentation",
            executable="yolo_segmentation_node.py",
            name="yolo_segmentation_node",
            parameters=[{
                "model_path": LaunchConfiguration("model_path"),
                "confidence": LaunchConfiguration("confidence"),
                "target_class": LaunchConfiguration("target_class"),
                "device": LaunchConfiguration("device"),
            }],
            output="screen",
            emulate_tty=True,
        ),
    ])
