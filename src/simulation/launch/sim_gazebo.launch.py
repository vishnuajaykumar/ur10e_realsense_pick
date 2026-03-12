"""
Full Gazebo simulation — UR10e + Robotiq + RealSense camera plugin + MoveIt + RViz.

Notes:
  - ros2_control_node uses /opt/ros/foxy binary explicitly to avoid
    booksros2_ws controller_manager override
  - use_fake_hardware=true so fake_components drives the arm (no real UR needed)
  - Gazebo provides visuals + depth camera plugin on /camera/* topics
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    OpaqueFunction, SetEnvironmentVariable, TimerAction
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
    with open(os.path.join(pkg, file_path)) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    ur_type     = LaunchConfiguration("ur_type",     default="ur10e")
    prefix      = LaunchConfiguration("prefix",      default="")
    paused      = LaunchConfiguration("paused",      default="false")

    # ── Robot description ──────────────────────────────────────────────────
    robot_description_content = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("simulation"), "urdf",
                              "ur10e_with_realsense.urdf.xacro"]),
        " ur_type:=", ur_type, " prefix:=", prefix,
        " use_fake_hardware:=true fake_sensor_commands:=true",
    ]), value_type=str)
    robot_description = {"robot_description": robot_description_content}

    # ── MoveIt SRDF + config ───────────────────────────────────────────────
    robot_description_semantic_content = ParameterValue(Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]),
        " name:=ur prefix:=", prefix,
    ]), value_type=str)
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    kinematics_yaml  = load_yaml("ur_moveit_config", "config/kinematics.yaml")
    ompl_yaml        = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")

    ompl_pipeline = {"move_group": {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": (
            "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints"
        ),
        "start_state_max_bounds_error": 0.1,
    }}
    ompl_pipeline["move_group"].update(ompl_yaml)

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager":
            "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    sim_controllers_path = os.path.join(
        get_package_share_directory("simulation"), "config", "sim_controllers.yaml"
    )

    # ── Nodes ──────────────────────────────────────────────────────────────
    world_file = PathJoinSubstitution(
        [FindPackageShare("simulation"), "worlds", "pick_world.world"]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"])
        ]),
        launch_arguments={"world": world_file, "paused": paused, "verbose": "false"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn_robot = TimerAction(period=3.0, actions=[
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-topic", "robot_description", "-entity", "ur10e_realsense",
                       "-x", "0", "-y", "0", "-z", "0"],
            output="screen",
        )
    ])

    # Use system Foxy ros2_control_node via OpaqueFunction — evaluates xacro
    # at launch time and writes a param YAML, bypassing booksros2_ws override
    def launch_ros2_control_node(context, *args, **kwargs):
        import subprocess
        import tempfile

        xacro_file = os.path.join(
            get_package_share_directory("simulation"),
            "urdf", "ur10e_with_realsense.urdf.xacro"
        )
        result = subprocess.run(
            ["xacro", xacro_file,
             "use_fake_hardware:=true", "fake_sensor_commands:=true"],
            capture_output=True, text=True, check=True
        )
        import re
        urdf_string = result.stdout
        # Strip <gpio> blocks — not supported by Foxy ros2_control
        urdf_string = re.sub(r'<gpio\b[^>]*>.*?</gpio>', '', urdf_string, flags=re.DOTALL)
        # Robotiq macro uses mock_components (newer ros2_control name);
        # Foxy only has fake_components — replace it
        urdf_string = urdf_string.replace(
            'mock_components/GenericSystem', 'fake_components/GenericSystem'
        )

        # Write robot_description param to temp file
        param_file = tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", delete=False
        )
        import yaml as _yaml
        _yaml.dump(
            {"/**": {"ros__parameters": {"robot_description": urdf_string}}},
            param_file
        )
        param_file.flush()

        return [
            TimerAction(period=5.0, actions=[
                ExecuteProcess(
                    cmd=[
                        "/opt/ros/foxy/lib/controller_manager/ros2_control_node",
                        "--ros-args",
                        "--params-file", param_file.name,
                        "--params-file", sim_controllers_path,
                    ],
                    output="screen",
                )
            ])
        ]

    ros2_control_node = OpaqueFunction(function=launch_ros2_control_node)

    spawn_jsb = TimerAction(period=9.0, actions=[
        ExecuteProcess(
            cmd=["ros2", "control", "load_controller", "--set-state", "start",
                 "joint_state_broadcaster"],
            output="screen",
        )
    ])

    spawn_jtc = TimerAction(period=11.0, actions=[
        ExecuteProcess(
            cmd=["ros2", "control", "load_controller", "--set-state", "start",
                 "ur_joint_trajectory_controller"],
            output="screen",
        )
    ])

    move_group = TimerAction(period=13.0, actions=[
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                {"robot_description_kinematics": kinematics_yaml},
                ompl_pipeline,
                {"moveit_manage_controllers": False,
                 "trajectory_execution.allowed_execution_duration_scaling": 1.2,
                 "trajectory_execution.allowed_goal_duration_margin": 0.5,
                 "trajectory_execution.allowed_start_tolerance": 0.01},
                moveit_controllers,
                {"publish_planning_scene": True,
                 "publish_geometry_updates": True,
                 "publish_state_updates": True,
                 "publish_transforms_updates": True},
            ],
        )
    ])

    rviz = TimerAction(period=16.0, actions=[
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", PathJoinSubstitution(
                [FindPackageShare("ur_moveit_config"), "rviz", "view_robot.rviz"]
            )],
            parameters=[
                robot_description,
                robot_description_semantic,
                ompl_pipeline,
                {"robot_description_kinematics": kinematics_yaml},
            ],
            output="log",
        )
    ])

    return LaunchDescription([
        SetEnvironmentVariable("GAZEBO_MODEL_PATH",
                               "/usr/share/gazebo-11/models:/usr/local/share/gazebo-11/models"),
        DeclareLaunchArgument("ur_type",  default_value="ur10e"),
        DeclareLaunchArgument("prefix",   default_value=""),
        DeclareLaunchArgument("paused",   default_value="false"),

        gazebo,
        robot_state_publisher,
        spawn_robot,
        ros2_control_node,
        spawn_jsb,
        spawn_jtc,
        move_group,
        rviz,
    ])
