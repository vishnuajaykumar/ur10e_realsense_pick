"""
Full Gazebo simulation — physics + RealSense camera plugin.

Starts:
  - Gazebo with pick_world (table + objects)
  - robot_state_publisher (UR10e + RealSense URDF)
  - Spawn robot into Gazebo
  - joint_state_broadcaster + joint_trajectory_controller
  - MoveIt2 move_group
  - RViz

Camera topics match real hardware:
  /camera/color/image_raw
  /camera/color/camera_info
  /camera/aligned_depth_to_color/image_raw

Usage:
  ros2 launch simulation sim_gazebo.launch.py
  ros2 launch simulation sim_gazebo.launch.py paused:=true  # start paused
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, file_path):
    pkg = get_package_share_directory(package_name)
    full_path = os.path.join(pkg, file_path)
    with open(full_path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    ur_type     = LaunchConfiguration("ur_type",     default="ur10e")
    prefix      = LaunchConfiguration("prefix",      default="")
    paused      = LaunchConfiguration("paused",      default="false")
    launch_rviz = LaunchConfiguration("launch_rviz", default="true")

    # ── World file ─────────────────────────────────────────────────────────
    world_file = PathJoinSubstitution(
        [FindPackageShare("simulation"), "worlds", "pick_world.world"]
    )

    # ── Robot description (Gazebo hardware — no fake) ──────────────────────
    robot_description_content = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("simulation"), "urdf",
                              "ur10e_with_realsense.urdf.xacro"]),
        " ur_type:=", ur_type,
        " prefix:=", prefix,
        " use_fake_hardware:=false",
    ]), value_type=str)
    robot_description = {"robot_description": robot_description_content}

    # ── MoveIt SRDF ────────────────────────────────────────────────────────
    robot_description_semantic_content = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf",
                              "ur.srdf.xacro"]),
        " name:=ur prefix:=", prefix,
    ]), value_type=str)
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # ── MoveIt config ──────────────────────────────────────────────────────
    kinematics_yaml  = load_yaml("ur_moveit_config", "config/kinematics.yaml")
    ompl_yaml        = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")

    ompl_pipeline = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_pipeline["move_group"].update(ompl_yaml)

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # ── 1. Gazebo ──────────────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch",
                                  "gazebo.launch.py"])
        ]),
        launch_arguments={
            "world": world_file,
            "paused": paused,
            "verbose": "false",
        }.items(),
    )

    # ── 2. Robot state publisher ───────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ── 3. Spawn robot in Gazebo ───────────────────────────────────────────
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-topic", "robot_description",
                    "-entity", "ur10e_realsense",
                    "-x", "0", "-y", "0", "-z", "0",
                ],
                output="screen",
            )
        ],
    )

    # ── 4. Controllers ─────────────────────────────────────────────────────
    spawn_jsb = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "control", "load_controller",
                     "--set-state", "start", "joint_state_broadcaster"],
                output="screen",
            )
        ],
    )
    spawn_jtc = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=["ros2", "control", "load_controller",
                     "--set-state", "start", "ur_joint_trajectory_controller"],
                output="screen",
            )
        ],
    )

    # ── 5. MoveIt move_group ───────────────────────────────────────────────
    move_group = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    {"robot_description_kinematics": kinematics_yaml},
                    ompl_pipeline,
                    trajectory_execution,
                    moveit_controllers,
                    planning_scene_monitor,
                ],
            )
        ],
    )

    # ── 6. RViz ────────────────────────────────────────────────────────────
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "rviz", "view_robot.rviz"]
    )
    rviz = TimerAction(
        period=12.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    ompl_pipeline,
                    {"robot_description_kinematics": kinematics_yaml},
                ],
                output="log",
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("ur_type",     default_value="ur10e"),
        DeclareLaunchArgument("prefix",      default_value=""),
        DeclareLaunchArgument("paused",      default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),

        gazebo,
        robot_state_publisher,
        spawn_robot,
        spawn_jsb,
        spawn_jtc,
        move_group,
        rviz,
    ])
