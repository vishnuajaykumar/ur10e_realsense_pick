"""
Full system launch: RealSense → YOLOv8-seg → PointCloud Processor → MoveIt Pick & Place

Usage:
  ros2 launch bringup pick_and_place.launch.py
  ros2 launch bringup pick_and_place.launch.py model_path:=/path/to/yolov8n-seg.pt
  ros2 launch bringup pick_and_place.launch.py target_class:=39  # bottle
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Arguments ─────────────────────────────────────────────────────────────
    model_path    = LaunchConfiguration("model_path",    default="yolov8n-seg.pt")
    confidence    = LaunchConfiguration("confidence",    default="0.5")
    target_class  = LaunchConfiguration("target_class",  default="-1")
    device        = LaunchConfiguration("device",        default="cpu")
    target_instance = LaunchConfiguration("target_instance", default="1")

    # ── 1. RealSense camera (starts immediately) ───────────────────────────
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("realsense_bringup"), "launch", "realsense.launch.py"
            ])
        ]),
    )

    # ── 2. YOLOv8-seg node (after camera has time to initialise) ──────────
    segmentation_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("segmentation"), "launch", "segmentation.launch.py"
                    ])
                ]),
                launch_arguments={
                    "model_path":   model_path,
                    "confidence":   confidence,
                    "target_class": target_class,
                    "device":       device,
                }.items(),
            )
        ],
    )

    # ── 3. PointCloud processor (after segmentation is publishing masks) ───
    perception_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("perception"), "launch", "perception.launch.py"
                    ])
                ]),
                launch_arguments={
                    "target_instance_id": target_instance,
                }.items(),
            )
        ],
    )

    # ── 4. MoveIt pick & place server (after robot driver is up) ──────────
    manipulation_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("manipulation"), "launch", "manipulation.launch.py"
                    ])
                ]),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("model_path",    default_value="yolov8n-seg.pt"),
        DeclareLaunchArgument("confidence",    default_value="0.5"),
        DeclareLaunchArgument("target_class",  default_value="-1"),
        DeclareLaunchArgument("device",        default_value="cpu"),
        DeclareLaunchArgument("target_instance", default_value="1"),

        realsense_launch,
        segmentation_launch,
        perception_launch,
        manipulation_launch,
    ])
