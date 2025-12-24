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

from iiwa_bringup.utils import converter


PACKAGE = "iiwa_bringup"
DESCRIPTION_PKG = "iiwa_description"


def _arg(name: str, default, desc: str):
    """Short helper to declare a launch argument."""
    return DeclareLaunchArgument(name=name, default_value=default, description=desc)


def _share_file(pkg_arg_name: str, *relpath: str):
    """Join <share>/<relpath...> where <share> is FindPackageShare(LaunchConfiguration(pkg_arg_name))."""
    return PathJoinSubstitution([FindPackageShare(LaunchConfiguration(pkg_arg_name)), *relpath])


def _runtime_setup(context, *args, **kwatgs):
    setup = []

    robot_name = LaunchConfiguration("robot_name").perform(context)
    world_path = LaunchConfiguration("world").perform(context)
    transform = LaunchConfiguration("transform").perform(context)
    rotation = LaunchConfiguration("rotation").perform(context)
    timer = LaunchConfiguration("controller_timer").perform(context)

    xacro_file = LaunchConfiguration("xacro_file").perform(context)
    srdf_file = LaunchConfiguration("srdf_file").perform(context)

    # ros2_controllers_file = LaunchConfiguration("controller").perform(context)
    initial_positions_file = LaunchConfiguration("initial_positions_file").perform(context)

    kinematics_yaml = LaunchConfiguration("kinematics_yaml").perform(context)
    joint_limits_yaml = LaunchConfiguration("joint_limits_yaml").perform(context)
    pilz_limits_yaml = LaunchConfiguration("pilz_limits_yaml").perform(context)
    moveit_controllers_yaml = LaunchConfiguration("moveit_controllers_yaml").perform(context)

    robot_description = converter.load_robot_description(
        model_path=xacro_file, robot_name=robot_name
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, 
                     "use_sim_time": False}],
    )

    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare(PACKAGE),
                    "launch",
                    "supported",
                    "webots_spawn.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_name": robot_name,
            "xacro_file": xacro_file,
            "world": world_path,
            "transform": transform,
            "rotation": rotation,
            "controller_timer": timer,
        }.items(),
    )

    moveit_configs = (
        MoveItConfigsBuilder("iiwa7", package_name=LaunchConfiguration("config_pkg").perform(context))
        .robot_description(file_path=xacro_file, mappings={
            "initial_positions_file": initial_positions_file,
        })
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=kinematics_yaml)
        .joint_limits(file_path=joint_limits_yaml)
        .pilz_cartesian_limits(file_path=pilz_limits_yaml)
        .trajectory_execution(file_path=moveit_controllers_yaml)
        .moveit_cpp(file_path="/home/daniel/dev/ros2_iiwa7/src/iiwa_bringup/config/motion_planing.yaml") # TODO: сделать подстановочным значением
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_configs.to_dict(),
            robot_description,
        ],
    )

    # TODO: не забудь поменять правильное название и имя пакета
    moveit_py_node = Node(
        name="moveit_py",
        package="iiwa_object_spawner",
        executable="motion_planning_test",
        output="both",
        parameters=[moveit_configs.to_dict()],
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare(PACKAGE), "config", "rviz_iiwa.rviz"]
    )

    rviz_launch = Node(
        condition=IfCondition(LaunchConfiguration("rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="log",
        parameters=[
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
            moveit_configs.robot_description_kinematics,
            moveit_configs.planning_pipelines,
            moveit_configs.joint_limits,
        ],
    )

    shutdown_on_rviz_exit = RegisterEventHandler(
        OnProcessExit(target_action=rviz_launch, 
                      on_exit=[EmitEvent(event=Shutdown())])
    )

    setup += [rsp_node, 
              webots_launch, 
              move_group,
              moveit_py_node, 
              rviz_launch, 
              shutdown_on_rviz_exit
            ]

    return setup


def generate_launch_description():
    # Объявление аргументов командной строки
    declare_description_pkg = _arg(
        "description_pkg",
        DESCRIPTION_PKG,
        "Package containing URDF/Xacro (and optionally SRDF/worlds).",
    )
    declare_config_pkg = _arg(
        "config_pkg",
        PACKAGE,
        "Package containing MoveIt/ros2_control config YAML files (config/*).",
    )

    declare_robot_name = _arg(
        "robot_name",
        "iiwa7",
        "Robot name (used for TF and naming).",
    )
    declare_world = _arg(
        "world",
        _share_file("description_pkg", "worlds", "iiwa.wbt"),
        "Path to Webots world (.wbt).",
    )
    declare_controller = _arg(
        "controller",
        _share_file("config_pkg", "config", "iiwa_controller.yaml"),
        "Controllers YAML (spawner/controller_manager).",
    )
    declare_transform = _arg(
        "transform",
        "-0.25 0 0.79",
        "Spawn translation in Webots (x y z).",
    )
    declare_rotation = _arg(
        "rotation",
        "0 0 1 0",
        "Spawn rotation axis-angle in Webots.",
    )
    declare_rviz = _arg(
        "rviz",
        "0",
        "If true|1|yes then launch RViz/MoveIt branch (instead of controllers branch).",
    )
    declare_controller_timer = _arg(
        "controller_timer",
        "50",
        "Timeout (seconds) for controller_manager spawners.",
    )

    # URDF/SRDF
    declare_xacro_file = _arg(
        "xacro_file",
        _share_file("description_pkg", "urdf", "iiwa7.urdf.xacro"),
        "Xacro used by MoveIt robot_description.",
    )
    declare_srdf_file = _arg(
        "srdf_file",
        _share_file("config_pkg", "config", "iiwa7.srdf"),
        "SRDF path.",
    )

    # YAML конфиги
    declare_initial_positions = _arg(
        "initial_positions_file",
        _share_file("config_pkg", "config", "initial_positions.yaml"),
        "initial_positions.yaml passed into xacro arg initial_positions_file.",
    )
    declare_kinematics_yaml = _arg(
        "kinematics_yaml",
        _share_file("config_pkg", "config", "kinematics.yaml"),
        "MoveIt kinematics.yaml",
    )
    declare_joint_limits_yaml = _arg(
        "joint_limits_yaml",
        _share_file("config_pkg", "config", "joint_limits.yaml"),
        "MoveIt joint_limits.yaml",
    )
    declare_pilz_limits_yaml = _arg(
        "pilz_limits_yaml",
        _share_file("config_pkg", "config", "pilz_cartesian_limits.yaml"),
        "Pilz cartesian limits yaml.",
    )
    declare_moveit_controllers_yaml = _arg(
        "moveit_controllers_yaml",
        _share_file("config_pkg", "config", "moveit_controllers.yaml"),
        "MoveIt controllers (trajectory_execution / simple_controller_manager).",
    )


    runtime_setup = OpaqueFunction(function=_runtime_setup)

    return LaunchDescription(
        [
            declare_description_pkg,
            declare_config_pkg,

            declare_robot_name,
            declare_world,
            declare_controller,
            declare_transform,
            declare_rotation,
            declare_rviz,
            declare_controller_timer,

            declare_xacro_file,
            declare_srdf_file,
            declare_initial_positions,
            declare_kinematics_yaml,
            declare_joint_limits_yaml,
            declare_pilz_limits_yaml,
            declare_moveit_controllers_yaml,

            runtime_setup,
        ]
    )
