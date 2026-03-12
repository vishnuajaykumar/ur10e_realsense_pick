from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("target_instance_id", default_value="1",
                              description="Mask instance id to process (1-indexed)"),
        DeclareLaunchArgument("target_frame", default_value="base_link"),
        DeclareLaunchArgument("min_depth_m", default_value="0.1"),
        DeclareLaunchArgument("max_depth_m", default_value="2.0"),

        Node(
            package="perception",
            executable="pointcloud_processor",
            name="pointcloud_processor",
            parameters=[{
                "target_instance_id": LaunchConfiguration("target_instance_id"),
                "target_frame":       LaunchConfiguration("target_frame"),
                "min_depth_m":        LaunchConfiguration("min_depth_m"),
                "max_depth_m":        LaunchConfiguration("max_depth_m"),
                "depth_scale":        0.001,
            }],
            output="screen",
            emulate_tty=True,
        ),
    ])
