import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from iiwa_utils import converter, setting_loader

from supported.moveit_nodes import make_moveit_nodes
from supported.rviz_nodes import make_rviz_nodes
from supported.simulation_nodes import make_simulation_nodes
from supported.optional_nodes import make_foxglove_node, make_web_server_node


def _runtime_setup(context, *args, **kwargs):
    setup = []

    simulate = LaunchConfiguration("simulate").perform(context) in ("true", "1", "yes")

    settings = setting_loader.build_settings(
        settings_path=LaunchConfiguration("setting").perform(context),
        check_files=True,
    )

    joint_limits_ros2 = converter.wrap_for_ros2_params(
        settings.controller.moveit.joint_limits,
        "robot_description_planning",
    )
    kinematics_ros2 = converter.wrap_for_ros2_params(
        settings.controller.moveit.kinematics,
        "robot_description_kinematics",
    )

    description_path = settings.robot.description
    use_sim_time = simulate

    if simulate:
        xacro_args = {
            "initial_positions_file": settings.controller.moveit.initial_positions,
            "simulate": "true",
        }
    else:
        xacro_args = {
            "initial_positions_file": settings.controller.moveit.initial_positions,
            "robot_ip": settings.robot.ip,
            "fri_port": str(settings.robot.port),
            "simulate": "false",
            "joint_position_tau": str(settings.robot.joint_position_tau),
        }

    robot_description = converter.load_robot_description(
        model_path=description_path,
        robot_name=settings.robot.name,
        xacro_args=xacro_args,
    )

    # Robot State Publisher
    setup.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": use_sim_time},
        ],
    ))

    # Webots симуляция
    if simulate:
        setup.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("iiwa_bringup"), "launch", "supported", "webots_spawn.launch.py",
                ])
            ),
            launch_arguments={
                "robot_name": str(settings.robot.name),
                "description": str(settings.robot.description),
                "world": str(settings.digital_twin.webots.world),
                "transform": str(settings.digital_twin.webots.transform),
                "rotation": str(settings.digital_twin.webots.rotation),
                "controller_timer": str(settings.digital_twin.webots.controller_timer),
                "controller": str(settings.controller.controller_path),
                "initial_positions_file": str(settings.controller.moveit.initial_positions),
            }.items(),
        ))

        setup += make_simulation_nodes(settings)

    # Controllers
    if simulate:
        controller_args = {
            "robot_name": settings.robot.name,
            "description": description_path,
            "initial_positions_file": settings.controller.moveit.initial_positions,
            "controller_path": settings.controller.controller_path,
            "simulate": "true",
            "transform": str(settings.digital_twin.webots.transform),
            "rotation": str(settings.digital_twin.webots.rotation),
            "controller_timer": str(settings.digital_twin.webots.controller_timer),
        }
    else:
        controller_args = {
            "robot_name": settings.robot.name,
            "description": description_path,
            "initial_positions_file": settings.controller.moveit.initial_positions,
            "controller_path": settings.controller.controller_path,
            "simulate": "false",
            "transform": str(settings.digital_twin.webots.transform),
            "rotation": str(settings.digital_twin.webots.rotation),
            "controller_timer": str(settings.digital_twin.webots.controller_timer),
            "fri_cycle_ms": str(settings.robot.fri_cycle_ms),
            "joint_position_tau": str(settings.robot.joint_position_tau),
            "controller": settings.robot.active_controller,
        }

    setup.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("iiwa_bringup"), "launch", "supported", "controllers.launch.py",
            ])
        ),
        launch_arguments={k: str(v) for k, v in controller_args.items()}.items(),
    ))

    # MoveIt
    moveit_configs, moveit_nodes = make_moveit_nodes(settings, robot_description, use_sim_time)
    setup += moveit_nodes

    # RViz
    setup += make_rviz_nodes(settings, moveit_configs, joint_limits_ros2, kinematics_ros2, use_sim_time)

    # Опциональные сервисы
    if settings.foxglove.enabled:
        setup.append(make_foxglove_node(settings, use_sim_time))

    if settings.web.enabled:
        setup.append(make_web_server_node(settings, use_sim_time))

    return setup


def generate_launch_description():
    declare_simulate = DeclareLaunchArgument(
        name="simulate",
        default_value="false",
        description="true = Webots симуляция, false = реальный робот через FRI",
    )

    declare_rviz = DeclareLaunchArgument(
        name="rviz",
        default_value="false",
        description="true = запустить RViz",
    )

    declare_setting = DeclareLaunchArgument(
        name="setting",
        default_value=PathJoinSubstitution(
            [FindPackageShare("iiwa_config"), "config", "setting.yaml"]
        ),
        description="Путь к файлу настроек",
    )

    return LaunchDescription([
        declare_simulate,
        declare_rviz,
        declare_setting,
        OpaqueFunction(function=_runtime_setup),
    ])
