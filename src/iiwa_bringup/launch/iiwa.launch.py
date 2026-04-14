import json
from dataclasses import asdict

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
from webots_ros2_driver.webots_controller import WebotsController

from iiwa_utils import converter, setting_loader
from iiwa_utils.camera_spawner import load_camera_config, build_ros_urdf  # type: ignore


def _foxglove_params(fg, use_sim_time: bool) -> dict:
    params = asdict(fg)
    params.pop("enabled")
    params["use_sim_time"] = use_sim_time

    return params


def _runtime_setup(context, *args, **kwargs):
    setup = []

    # Настройка параметров
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

    if simulate:
        xacro_args = {
            "initial_positions_file": settings.controller.moveit.initial_positions,
            "simulate": "true",
        }
        use_sim_time = True
    else:
        xacro_args = {
            "initial_positions_file": settings.controller.moveit.initial_positions,
            "robot_ip": settings.robot.ip,
            "fri_port": str(settings.robot.port),
            "simulate": "false",
            "command_mode": settings.robot.command_mode,
        }
        use_sim_time = False

    robot_description = converter.load_robot_description(
        model_path=description_path,
        robot_name=settings.robot.name,
        xacro_args=xacro_args,
    )

    # Вызов нод
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": use_sim_time},
        ],
    )

    setup += [rsp_node]

    # webots spawn
    if simulate:
        webots_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("iiwa_bringup"),
                        "launch",
                        "supported",
                        "webots_spawn.launch.py",
                    ]
                )
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
        )

        setup += [webots_launch]

    # Controller launch
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

        if settings.digital_twin.webots.cameras:
            # Спавн камеры
            camera_spawner_node = Node(
                package="iiwa_utils",
                executable="camera_spawner",
                name="camera_spawner",
                output="screen",
                parameters=[{
                    "camera_configs": json.dumps(settings.digital_twin.webots.cameras)
                }],
            )
            setup.append(camera_spawner_node)

            # WebotsController для каждой камеры
            for cam_path in settings.digital_twin.webots.cameras:
                cam_cfg = load_camera_config(cam_path)
                urdf = build_ros_urdf(cam_cfg)

                camera_controller = WebotsController(
                    robot_name=f"{cam_cfg.name}_robot",
                    parameters=[{
                        "robot_description": urdf,
                        "use_sim_time": True,
                        "set_robot_state_publisher": False,
                    }],
                    respawn=True,
                )
                setup.append(camera_controller)

    else:
        controller_args = {
            "robot_name": settings.robot.name,
            "description": description_path,
            "initial_positions_file": settings.controller.moveit.initial_positions,
            "controller_path": settings.controller.controller_path,
            "simulate": "false",
            "command_mode": settings.robot.command_mode,
        }

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("iiwa_bringup"),
                    "launch",
                    "supported",
                    "controllers.launch.py",
                ]
            )
        ),
        launch_arguments={k: str(v) for k, v in controller_args.items()}.items(),
    )

    # Moveit launch
    moveit_configs = (
        MoveItConfigsBuilder("iiwa7", package_name="iiwa_config")
        .robot_description(
            file_path=description_path,
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
            {"use_sim_time": use_sim_time},
        ],
    )

    # Rviz launch
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
            moveit_configs.planning_pipelines,
            joint_limits_ros2,
            kinematics_ros2,
            {"use_sim_time": use_sim_time},
        ],
    )

    shutdown_on_rviz_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz_launch,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )

    setup += [
        controllers_launch, 
        move_group, 
        rviz_launch, 
        shutdown_on_rviz_exit
    ]

    if settings.foxglove.enabled:
        foxglove_bridge = Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            output="screen",
            name="foxglove_bridge",
            parameters=[_foxglove_params(settings.foxglove, use_sim_time)]
        )

        setup += [foxglove_bridge]

    return setup

def generate_launch_description():
    declare_simulate = DeclareLaunchArgument(
        name="simulate",
        default_value="false",
        description="true = Gazebo симуляция, false = реальный робот через FRI",
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

    runtime_setup = OpaqueFunction(function=_runtime_setup)

    return LaunchDescription([
        declare_simulate,
        declare_rviz,
        declare_setting,
        runtime_setup,
    ])

