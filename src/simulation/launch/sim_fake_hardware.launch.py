"""
Fake-hardware simulation — no Gazebo, no physics.

Starts:
  - robot_state_publisher (UR10e URDF)
  - ur_ros2_control with use_fake_hardware=true
  - joint_state_broadcaster
  - joint_trajectory_controller
  - MoveIt2 move_group
  - RViz with MoveIt MotionPlanning panel

Use this to verify MoveIt planning and pick_place_server logic
before spinning up Gazebo. Camera topics are NOT available here.

Usage:
  ros2 launch simulation sim_fake_hardware.launch.py
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
)
from launch.event_handlers import OnProcessExit
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
    ur_type      = LaunchConfiguration("ur_type",      default="ur10e")
    prefix       = LaunchConfiguration("prefix",       default="")
    launch_rviz  = LaunchConfiguration("launch_rviz",  default="true")

    # ── Robot description (fake hardware) ─────────────────────────────────
    robot_description_content = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("simulation"), "urdf",
                              "ur10e_with_realsense.urdf.xacro"]),
        " ur_type:=", ur_type,
        " prefix:=", prefix,
        " use_fake_hardware:=true",
        " fake_sensor_commands:=true",
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
    kinematics_yaml = load_yaml("ur_moveit_config", "config/kinematics.yaml")
    ompl_yaml       = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
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

    # ── Nodes ──────────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    move_group = Node(
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

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "rviz", "view_robot.rviz"]
    )
    rviz = Node(
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

    # Spawn controllers via ros2 control
    spawn_jsb = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "start",
             "joint_state_broadcaster"],
        output="screen",
    )
    spawn_jtc = ExecuteProcess(
        cmd=["ros2", "control", "load_controller", "--set-state", "start",
             "ur_joint_trajectory_controller"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("ur_type",     default_value="ur10e"),
        DeclareLaunchArgument("prefix",      default_value=""),
        DeclareLaunchArgument("launch_rviz", default_value="true"),

        robot_state_publisher,
        move_group,
        TimerAction(period=2.0, actions=[spawn_jsb]),
        TimerAction(period=4.0, actions=[spawn_jtc]),
        TimerAction(period=3.0, actions=[rviz]),
    ])
