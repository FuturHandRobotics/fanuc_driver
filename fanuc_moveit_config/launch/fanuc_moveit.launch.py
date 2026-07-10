# SPDX-FileCopyrightText: 2025, FANUC America Corporation
# SPDX-FileCopyrightText: 2025, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model")
    robot_ip = LaunchConfiguration("robot_ip")
    ros2_control_config = LaunchConfiguration("ros2_control_config")
    gpio_configuration = LaunchConfiguration("gpio_configuration")
    use_mock = LaunchConfiguration("use_mock")
    hand_type = LaunchConfiguration("hand_type")
    prefix = LaunchConfiguration("prefix")
    hand_ros2_control = LaunchConfiguration("hand_ros2_control")
    no_calib = LaunchConfiguration("no_calib")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_file_path = LaunchConfiguration("rviz_file_path")
    warehouse_plugin = LaunchConfiguration("warehouse_plugin").perform(context)
    warehouse_host = LaunchConfiguration("warehouse_host").perform(context)
    warehouse_port = int(LaunchConfiguration(
        "warehouse_port").perform(context) or 0)

    # Load hand joint limits from futur_hand_description and apply prefix.
    # This keeps hand-specific limits out of the arm's MoveIt config package.
    hand_joint_limits_file = LaunchConfiguration(
        "hand_joint_limits_file").perform(context)
    prefix_str = prefix.perform(context)
    hand_moveit_params: dict = {}

    # Joint limits
    if hand_joint_limits_file and os.path.isfile(hand_joint_limits_file):
        with open(hand_joint_limits_file) as f:
            raw = yaml.safe_load(f) or {}
        prefixed = {f"{prefix_str}_{k}": v for k, v in raw.items()}
        hand_moveit_params["robot_description_planning"] = {"joint_limits": prefixed}

    # Controller config — add hand controller to the arm's controller list
    hand_controller_file = LaunchConfiguration("hand_controller_file").perform(context)
    if hand_controller_file and os.path.isfile(hand_controller_file):
        with open(hand_controller_file) as f:
            raw_ctrl = yaml.safe_load(f) or {}
        prefixed_ctrl: dict = {}
        for ctrl_name, ctrl_cfg in raw_ctrl.items():
            cfg = dict(ctrl_cfg)
            if "joints" in cfg:
                cfg["joints"] = [f"{prefix_str}_{j}" for j in cfg["joints"]]
            prefixed_ctrl[ctrl_name] = cfg
        hand_moveit_params["moveit_simple_controller_manager"] = {
            # Real-hardware HandHardwareInterface runs Dynamixel calibration
            # on activate, which can take 30+ s. 90 s gives enough margin.
            "wait_for_servers": 45.0,
            "controller_names": ["joint_trajectory_controller"] + list(prefixed_ctrl.keys()),
            **prefixed_ctrl,
        }
    hand_joint_limits_params = hand_moveit_params

    nodes_to_launch = []

    # Conditionally include the appropriate control launch file
    include_fanuc_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "launch",
                    "fanuc_physical_control.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "robot_model": robot_model,
            "robot_series": "crx",
            "robot_ip": robot_ip,
            "gpio_configuration": gpio_configuration,
            "ros2_control_config": ros2_control_config,
            "launch_rviz": "false",
            "use_mock": use_mock,
            "hand_type": hand_type,
            "prefix": prefix,
            "hand_ros2_control": hand_ros2_control,
            "no_calib": no_calib,
        }.items(),
        condition=UnlessCondition(use_mock),
    )
    nodes_to_launch.append(include_fanuc_control)

    include_fanuc_mock_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("fanuc_hardware_interface"),
                    "launch",
                    "fanuc_mock_control.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "robot_model": robot_model,
            "robot_series": "crx",
            "gpio_configuration": gpio_configuration,
            "ros2_control_config": ros2_control_config,
            "launch_rviz": "false",
            "hand_type": hand_type,
            "prefix": prefix,
            "hand_ros2_control": hand_ros2_control,
            "no_calib": no_calib,
        }.items(),
        condition=IfCondition(use_mock),
    )
    nodes_to_launch.append(include_fanuc_mock_control)

    hand_srdf_file = os.path.join(
        get_package_share_directory("futur_hand_description"),
        "hands", f"hand_{hand_type.perform(context)}", "srdf", f"hand_{hand_type.perform(context)}.srdf.xacro",
    )

    hand_type_str = hand_type.perform(context)

    description_arguments = {
        "robot_ip": robot_ip.perform(context),
        "use_mock": use_mock.perform(context),
        "gpio_configuration": gpio_configuration.perform(context),
        "hand_type": hand_type_str,
        "prefix": prefix.perform(context),
    }

    urdf_full_path = os.path.join(
        get_package_share_directory("fanuc_hardware_interface"),
        "robot",
        f"{robot_model.perform(context)}.urdf.xacro",
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_model.perform(context), package_name="fanuc_moveit_config"
        )
        .robot_description(file_path=urdf_full_path, mappings=description_arguments)
        .robot_description_semantic(
            file_path=f"srdf/{robot_model.perform(context)}.srdf.xacro",
            mappings={
                "hand_type": hand_type.perform(context),
                "prefix": prefix.perform(context),
                "hand_srdf_file": hand_srdf_file,
            },
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl", "stomp", "taskspace_birrt"])
        .to_moveit_configs()
    )

    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="log",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
            {
                "warehouse_plugin": warehouse_plugin,
                "warehouse_host": warehouse_host,
                "warehouse_port": warehouse_port,
            },
            hand_joint_limits_params,
        ],
    )
    # Delay move_group until after the hand_trajectory_controller action server is
    # up.  The hand spawner fires 3 s after the control launch; give it 3 more
    # seconds of margin so the action client connects on the first attempt.
    nodes_to_launch.append(TimerAction(period=6.0, actions=[move_group_node]))

    rviz_file = rviz_file_path
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="both",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {
                "warehouse_plugin": warehouse_plugin,
                "warehouse_host": warehouse_host,
                "warehouse_port": warehouse_port,
            },
            moveit_config.joint_limits,
            hand_joint_limits_params,
        ],
        arguments=["--display-config", rviz_file],
        condition=IfCondition(start_rviz),
    )
    nodes_to_launch.append(TimerAction(period=10.0, actions=[rviz_node]))

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_model",
            description="The robot model (required).",
            choices=[
                "crx3ia",
                "crx5ia",
                "crx10ia",
                "crx10ia_l",
                "crx20ia_l",
                "crx30ia",
            ],
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
            description="YAML file configuration to specify the GPIO configuration..",
        ),
        DeclareLaunchArgument(
            "use_mock",
            default_value="false",
            description="Whether to use a mock hardware interface.",
        ),
        DeclareLaunchArgument(
            "hand_type",
            default_value="minimal",
            description="Hand/tool xacro variant to attach.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="fanuc",
            description="Prefix for robot/hand frames and joints.",
        ),
        DeclareLaunchArgument(
            "hand_ros2_control",
            default_value="true",
            description=(
                "Include the hand ros2_control hardware block in the arm URDF "
                "and spawn its trajectory controller on the arm's controller "
                "manager. Set false when a dedicated hand controller manager "
                "is launched separately (e.g. futur_hand_driver's "
                "hand_control.launch.py) — the hand's geometry/SRDF groups "
                "stay attached for collision/planning either way."
            ),
        ),
        DeclareLaunchArgument(
            "no_calib",
            default_value="false",
            description=(
                "Skip the hand's current-sensing calibration routine on real "
                "hardware and use whatever position the motors are currently "
                "at as the 'open' reference instead. No effect in mock mode "
                "or when hand_ros2_control:=false."
            ),
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Whether to start RViz.",
        ),
        DeclareLaunchArgument(
            "rviz_file_path",
            default_value=PathJoinSubstitution(
                [FindPackageShare("fanuc_moveit_config"),
                 "rviz", "view_robot.rviz"]
            ),
            description="Path to the RViz config file.",
        ),
        DeclareLaunchArgument(
            "warehouse_plugin",
            default_value="",
            description="warehouse_ros plugin class (empty = no warehouse).",
        ),
        DeclareLaunchArgument(
            "warehouse_host",
            default_value="",
            description="Warehouse database host or file path.",
        ),
        DeclareLaunchArgument(
            "warehouse_port",
            default_value="0",
            description="Warehouse database port.",
        ),
        DeclareLaunchArgument(
            "hand_joint_limits_file",
            default_value="",
            description=(
                "Path to a YAML file with MoveIt joint limits for the hand. "
                "Keys are bare joint names (without prefix); the prefix is "
                "prepended at launch time."
            ),
        ),
        DeclareLaunchArgument(
            "hand_controller_file",
            default_value="",
            description=(
                "Path to a YAML file with MoveIt controller config for the hand. "
                "Joint names are bare (without prefix); the prefix is prepended "
                "at launch time."
            ),
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
