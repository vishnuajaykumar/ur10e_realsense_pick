from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("arm_planning_group",     default_value="ur_manipulator"),
        DeclareLaunchArgument("gripper_planning_group", default_value="gripper"),
        DeclareLaunchArgument("approach_distance_m",    default_value="0.10"),
        DeclareLaunchArgument("place_x", default_value="0.4"),
        DeclareLaunchArgument("place_y", default_value="0.3"),
        DeclareLaunchArgument("place_z", default_value="0.3"),

        Node(
            package="manipulation",
            executable="pick_place_server",
            name="pick_place_server",
            parameters=[{
                "arm_planning_group":     LaunchConfiguration("arm_planning_group"),
                "gripper_planning_group": LaunchConfiguration("gripper_planning_group"),
                "approach_distance_m":    LaunchConfiguration("approach_distance_m"),
                "place_x": LaunchConfiguration("place_x"),
                "place_y": LaunchConfiguration("place_y"),
                "place_z": LaunchConfiguration("place_z"),
            }],
            output="screen",
            emulate_tty=True,
        ),
    ])
