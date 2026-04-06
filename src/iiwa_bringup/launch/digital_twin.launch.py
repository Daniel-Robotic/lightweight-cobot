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


def _runtime_setup(context, *args, **kwatgs):
    setup = []

    settings = setting_loader.build_settings(
        settings_path=LaunchConfiguration("setting").perform(context), check_files=True
    )

    robot_description = converter.load_robot_description(
        model_path=settings.digital_twin.description,
        robot_name=settings.robot.name,
        xacro_args={
            "initial_positions_file": settings.controller.moveit.initial_positions
        },
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

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
            "description": str(settings.digital_twin.description),
            "world": str(settings.digital_twin.webots.world),
            "transform": str(settings.digital_twin.webots.transform),
            "rotation": str(settings.digital_twin.webots.rotation),
            "controller_timer": str(settings.digital_twin.webots.controller_timer),
            "controller": str(settings.controller.controller_path),
            "initial_positions_file": str(settings.controller.moveit.initial_positions),
        }.items(),
    )

    moveit_configs = (
        MoveItConfigsBuilder("iiwa7", package_name="iiwa_config")
        .robot_description(
            file_path=settings.digital_twin.description,
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
            {"use_sim_time": True},
        ],
    )

    # TODO: не забудь поменять правильное название и имя пакета
    # moveit_py_node = Node(
    #     # name="motion_planning_node",
    #     package="iiwa_planning",
    #     executable="motion_planning",
    #     output="both",
    #     parameters=[moveit_configs.to_dict()],
    # )


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
            moveit_configs.planning_scene_monitor,
            {"use_sim_time": True},
        ],
    )

    shutdown_on_rviz_exit = RegisterEventHandler(
        OnProcessExit(target_action=rviz_launch, on_exit=[EmitEvent(event=Shutdown())])
    )

    setup += [
        rsp_node,
        webots_launch,
        move_group,
        # moveit_py_node,
        rviz_launch,
        shutdown_on_rviz_exit,
    ]

    return setup


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
