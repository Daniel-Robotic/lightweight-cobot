from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from iiwa_bringup.utils import converter

PACKAGE = "iiwa_bringup"
DESCRIPTION_PKG = "iiwa_description"


def _runtime_setup(context, *args, **kwatgs):
    setup = []

    model_path = LaunchConfiguration("model").perform(context)
    robot_name = LaunchConfiguration("robot_name").perform(context)
    world_path = LaunchConfiguration("world").perform(context)
    rviz_status = LaunchConfiguration("rviz").perform(context).lower() in [
        "true",
        "1",
        "yes",
    ]

    transform = LaunchConfiguration("transform").perform(context)
    rotation = LaunchConfiguration("rotation").perform(context)
    timer = LaunchConfiguration("controller_timer").perform(context)

    robot_description = converter.load_robot_description(
        model_path=model_path, robot_name=robot_name
    )

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
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
            "model": model_path,
            "world": world_path,
            "transform": transform,
            "rotation": rotation,
            "controller_timer": timer,
        }.items(),
    )

    setup += [rsp_node, webots_launch]

    if rviz_status:
        rviz_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare(PACKAGE), "launch", "supported", "rviz.launch.py"]
                )
            ),
            launch_arguments={"package_name": PACKAGE}.items(),
        )

        setup.append(rviz_launch)

    return setup


def generate_launch_description():
    # Объявление аргументов командной строки
    declare_model_arg = DeclareLaunchArgument(
        name="model",
        default_value=PathJoinSubstitution(
            [FindPackageShare(DESCRIPTION_PKG), "urdf", "iiwa7.urdf.xacro"]
        ),
        description="Path to robot xacro or urdf file (used to build robot_description).",
    )

    declare_robot_name_arg = DeclareLaunchArgument(
        name="robot_name",
        default_value="iiwa7",
        description="Robot name (used for TF and naming spawned robot).",
    )

    declare_world_arg = DeclareLaunchArgument(
        name="world",
        default_value=PathJoinSubstitution(
            [FindPackageShare(DESCRIPTION_PKG), "worlds", "iiwa.wbt"]
        ),
        description="Path to the Webots world (.wbt) to launch.",
    )

    declare_controller_arg = DeclareLaunchArgument(
        name="controller",
        default_value=PathJoinSubstitution(
            [FindPackageShare(PACKAGE), "config", "iiwa_controller.yaml"]
        ),
        description="Path to controllers YAML file (used by spawner and controller_manager).",
    )

    declare_transform_arg = DeclareLaunchArgument(
        name="transform",
        default_value="-0.25 0 0.79",
        description="Translation applied when spawning the robot in the Webots world (x y z).",
    )

    declare_rotation_arg = DeclareLaunchArgument(
        name="rotation",
        default_value="0 0 1 0",
        description="Rotation (axis-angle) applied when spawning the robot in the Webots world.",
    )

    declare_rviz_arg = DeclareLaunchArgument(
        name="rviz",
        default_value="0",
        description="If true|1|yes then launch RViz and joint_state_publisher_gui instead of controllers.",
    )

    declare_controller_manager_arg = DeclareLaunchArgument(
        name="controller_timer",
        default_value="50",
        description="Timeout (seconds) for controller_manager spawners (--controller-manager-timeout).",
    )

    runtime_setup = OpaqueFunction(function=_runtime_setup)

    return LaunchDescription(
        [
            declare_model_arg,
            declare_robot_name_arg,
            declare_world_arg,
            declare_controller_arg,
            declare_transform_arg,
            declare_rotation_arg,
            declare_rviz_arg,
            declare_controller_manager_arg,
            runtime_setup,
        ]
    )
