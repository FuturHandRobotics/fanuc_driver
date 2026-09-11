# SPDX-FileCopyrightText: 2025, FANUC America Corporation
# SPDX-FileCopyrightText: 2025, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
import os
import tempfile
import yaml as _yaml

from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_series = LaunchConfiguration("robot_series")
    robot_ip = LaunchConfiguration("robot_ip")
    ros2_control_config = LaunchConfiguration("ros2_control_config")
    gpio_configuration = LaunchConfiguration("gpio_configuration")
    launch_rviz = LaunchConfiguration("launch_rviz")
    hand_type = LaunchConfiguration("hand_type")
    prefix = LaunchConfiguration("prefix")
    hand_ros2_control = LaunchConfiguration("hand_ros2_control")
    no_calib = LaunchConfiguration("no_calib")
    calib_mode = LaunchConfiguration("calib_mode")

    robot_model_str = robot_model.perform(context)
    robot_series_str = robot_series.perform(context)
    hand_ros2_control_str = hand_ros2_control.perform(context)

    if robot_series_str == "crx":
        urdf_xacro_file = robot_model_str + ".urdf.xacro"
    else:
        urdf_xacro_file = "6dof_robot.urdf.xacro"

    robot_description = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("fanuc_hardware_interface"), "robot", urdf_xacro_file]
            ),
            " ",
            "robot_series:=",
            robot_series,
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "gpio_configuration:=",
            gpio_configuration,
            " ",
            "robot_model:=",
            robot_model,
            " ",
            "hand_type:=",
            hand_type,
            " ",
            "prefix:=",
            prefix,
            " ",
            "hand_ros2_control:=",
            hand_ros2_control,
            " ",
            "no_calib:=",
            no_calib,
            " ",
            "calib_mode:=",
            calib_mode,
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description, value_type=str)
    }

    # ── Hand JTC — read motor list from futur_hand_driver yaml ────────────────
    _hand_type_str = hand_type.perform(context)
    _prefix_str = prefix.perform(context)
    try:
        from ament_index_python.packages import get_package_share_directory as _gpsd

        _hand_yaml = os.path.join(
            _gpsd("futur_hand_driver"), "config", "hands", f"{_hand_type_str}.yaml"
        )
        _motors = (
            (_yaml.safe_load(open(_hand_yaml)) or {}).get("hand", {}).get("motors", [])
        )
    except (FileNotFoundError, KeyError):
        _motors = []

    # Only claim the hand's hardware interfaces/controller when this CM owns
    # the hand (hand_ros2_control=true). When a dedicated hand controller
    # manager is launched separately, the URDF built above excludes the
    # hand's <ros2_control> block entirely, so this CM must not reference
    # hand joints it no longer has interfaces for.
    _drive_hand = bool(_motors) and hand_ros2_control_str == "true"

    ros_parameters = [robot_description, ros2_control_config]

    if _drive_hand:
        _tendon_joints = [f"{_prefix_str}_{m['joint']}" for m in _motors]
        # launch_ros wraps a plain dict under /**→ros__parameters, which puts the
        # controller's joints/interfaces at the wrong param path.  Write a proper
        # two-section YAML (CM type-registration + controller params) to a temp
        # file and pass it as a path so launch_ros leaves the structure untouched.
        _hand_jtc_yaml = {
            "controller_manager": {
                "ros__parameters": {
                    "hand_trajectory_controller": {
                        "type": "joint_trajectory_controller/JointTrajectoryController",
                    },
                },
            },
            "hand_trajectory_controller": {
                "ros__parameters": {
                    "joints": _tendon_joints,
                    "command_interfaces": ["position"],
                    "state_interfaces": ["position", "velocity"],
                    "allow_partial_joints_goal": False,
                    "interpolate_from_desired_state": True,
                    "constraints": {
                        "goal_time": 0.0,
                        "stopped_velocity_tolerance": 0.0,
                    },
                },
            },
        }
        _hand_jtc_tmp = tempfile.NamedTemporaryFile(
            mode="w", suffix=".yaml", delete=False, prefix=f"hand_jtc_{_hand_type_str}_"
        )
        _yaml.dump(_hand_jtc_yaml, _hand_jtc_tmp)
        _hand_jtc_tmp.close()
        ros_parameters.append(_hand_jtc_tmp.name)

    nodes_to_launch = []
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=ros_parameters,
        output="both",
    )
    nodes_to_launch.append(control_node)

    if _drive_hand:
        nodes_to_launch.append(
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "hand_trajectory_controller",
                            "--controller-manager",
                            "/controller_manager",
                            "--controller-manager-timeout",
                            "30",
                        ],
                        output="screen",
                    )
                ],
            )
        )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    nodes_to_launch.append(robot_state_pub_node)

    rviz_file = PathJoinSubstitution(
        [
            FindPackageShare(
                PythonExpression(['"fanuc_" + "', robot_series, '" + "_description"'])
            ),
            "rviz",
            PythonExpression(['"view_" + "', robot_series, '" + ".rviz"']),
        ]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        arguments=["--display-config", rviz_file],
        condition=IfCondition(launch_rviz),
    )
    nodes_to_launch.append(rviz_node)

    controller_spawner_processes = [
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 joint_state_broadcaster"
            ],
            shell=True,
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 joint_trajectory_controller"
            ],
            shell=True,
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 fanuc_gpio_controller"
            ],
            shell=True,
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 fanuc_force_sensor_broadcaster"
            ],
            shell=True,
            output="screen",
        ),
        ExecuteProcess(
            cmd=[
                "ros2 run controller_manager spawner --controller-manager-timeout 180 force_torque_sensor_broadcaster"
            ],
            shell=True,
            output="screen",
        ),
    ]

    return nodes_to_launch + controller_spawner_processes


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_model",
            description="The robot model (required).",
        ),
        DeclareLaunchArgument(
            "robot_series",
            default_value="crx",
            description='The robot series such as "crx" (required).',
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.100",
            description="The robot's IP address.",
        ),
        DeclareLaunchArgument(
            "ros2_control_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "config",
                    "ros2_controllers.yaml",
                ]
            ),
            description="ROS 2 control configuration file the controllers.",
        ),
        DeclareLaunchArgument(
            "gpio_configuration",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "config",
                    "example_gpio_config.yaml",
                ]
            ),
            description="YAML file configuration to specify the IO streaming data.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Specify whether or not to open RVIZ.",
        ),
        DeclareLaunchArgument(
            "hand_ros2_control",
            default_value="true",
            description=(
                "Include the hand ros2_control hardware block in the arm URDF "
                "and spawn its trajectory controller on this CM. Set false "
                "when a dedicated hand controller manager is launched "
                "separately (e.g. futur_hand_driver's hand_control.launch.py)."
            ),
        ),
        DeclareLaunchArgument(
            "no_calib",
            default_value="false",
            description=(
                "Skip the hand's current-sensing calibration routine on real "
                "hardware and use whatever position the motors are currently "
                "at as the 'open' reference instead. Ignored when "
                "hand_ros2_control:=false (nothing to calibrate here)."
            ),
        ),
        DeclareLaunchArgument(
            "calib_mode",
            default_value="auto",
            description=(
                "Hand calibration mode: auto (current-sensing hard-stop), skip "
                "(use current position as open reference), or manual (load "
                "init_pos/max_pos from the manual_calibrate cache file). "
                "Supersedes no_calib when set to anything other than 'auto'."
            ),
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
