from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

from iiwa_utils import converter, setting_loader

import yaml
import tempfile

def wrap_for_ros2_params(yaml_path: str, namespace: str) -> str:
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    wrapped = {namespace: {"ros__parameters": data}}

    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False
    )
    yaml.dump(wrapped, tmp, default_flow_style=False)
    tmp.close()
    return tmp.name


def _runtime_setup(context, *args, **kwargs):
    settings = setting_loader.build_settings(
        settings_path=LaunchConfiguration("setting").perform(context), check_files=True
    )

    xacro_args = {
        "initial_positions_file": settings.controller.moveit.initial_positions,
        "robot_ip": settings.robot.ip,
        "fri_port": str(settings.robot.port),
        "simulate": "false",
        "command_mode": settings.robot.command_mode,
    }

    robot_description = converter.load_robot_description(
        model_path=settings.robot.description,
        robot_name=settings.robot.name,
        xacro_args=xacro_args,
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, 
                     "use_sim_time": False}],
    )

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("iiwa_bringup"),
                    "launch",
                    "supported",
                    "iiwa_controllers.launch.py"
                ]
            )
        ),
        launch_arguments={
            "robot_name": str(settings.robot.name),
            "description": str(settings.robot.description),
            "initial_positions_file": str(settings.controller.moveit.initial_positions),
            "controller_path": str(settings.controller.controller_path)
        }.items()
    )

    # Moveit
    moveit_configs = (
        MoveItConfigsBuilder("iiwa7", package_name="iiwa_config")
        .robot_description(
            file_path=settings.robot.description,
            mappings={
                "initial_positions_file": settings.controller.moveit.initial_positions
            },
        )
        .robot_description_semantic(file_path=settings.controller.moveit.srdf)
        .robot_description_kinematics(file_path=settings.controller.moveit.kinematics)
        .joint_limits(file_path=settings.controller.moveit.joint_limits)
        .pilz_cartesian_limits(file_path=settings.controller.moveit.pilz_limits)
        .trajectory_execution(file_path=settings.controller.moveit.moveit_controllers)
        .moveit_cpp(file_path=settings.controller.moveit.moveit_cpp)
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_configs.to_dict(),
            {"robot_description": robot_description},
        ],
    )


    joint_limits_ros2  = wrap_for_ros2_params(
        settings.controller.moveit.joint_limits,
        "robot_description_planning"
    )
    kinematics_ros2 = wrap_for_ros2_params(
        settings.controller.moveit.kinematics,
        "robot_description_kinematics"
    )

    rviz_launch = Node(
        condition=IfCondition(LaunchConfiguration("rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", settings.digital_twin.rviz.config],
        output="log",
        parameters=[
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
            # moveit_configs.robot_description_kinematics,
            moveit_configs.planning_pipelines,
            # moveit_configs.joint_limits,
            joint_limits_ros2,
            kinematics_ros2, 
        ],
    )

    shutdown_on_rviz_exit = RegisterEventHandler(
        OnProcessExit(target_action=rviz_launch, on_exit=[EmitEvent(event=Shutdown())])
    )
    
    return [
        rsp_node,
        controllers_launch,
        move_group,
        rviz_launch,
        shutdown_on_rviz_exit
    ]


def generate_launch_description():
    declare_rviz = DeclareLaunchArgument(
        name="rviz",
        default_value="0",
        description="If true|1|yes then launch RViz/MoveIt branch (instead of controllers branch)",
    )

    declacre_setting = DeclareLaunchArgument(
        name="setting",
        default_value=PathJoinSubstitution(
            [FindPackageShare("iiwa_config"), "config", "setting.yaml"]
        ),
        description="Absolute path to settings file",
    )

    runtime_setup = OpaqueFunction(function=_runtime_setup)

    return LaunchDescription(
        [
            declare_rviz,
            declacre_setting,
            runtime_setup,
        ]
    )