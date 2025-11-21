import xacro
from pathlib import Path
from launch.events import Shutdown
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    OpaqueFunction,
    EmitEvent,
    TimerAction
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.urdf_spawner import URDFSpawner
from webots_ros2_driver.webots_controller import WebotsController


def _runtime_controller(robot_name: str,
                        robot_description_str: str,
                        transform_proto: str = "0 0 0",
                        rotation_proto: str = "0 0 1 0",
                        controller_manager_timer: int = 50):
    tmo = ['--controller-manager-timeout', str(controller_manager_timer)]

    jsb = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'] + tmo,
        parameters=[{'use_sim_time': False}],
    )

    jtc = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_trajectory_controller'] + tmo, # название с yaml файла
        parameters=[{'use_sim_time': False}],
    )

    spawner_urdf = URDFSpawner(
        name=robot_name,
        robot_description=robot_description_str,
        translation=transform_proto,
        rotation=rotation_proto
    )

    return [jsb, jtc, spawner_urdf]


def _runtime_setup(context, *args, **kwargs):
    # Получение параметров от пользователя при запуске launch
    model = LaunchConfiguration('model').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    transform_proto = LaunchConfiguration('transform_proto').perform(context)
    rotation_proto = LaunchConfiguration('rotation_proto').perform(context)
    world_path = LaunchConfiguration('world').perform(context)
    rviz_status = LaunchConfiguration('rviz').perform(context).lower() in ['true', '1', 'yes']
    controller_manager = LaunchConfiguration('controller_manager').perform(context)

    # Загрузка описания робота
    model_path = Path(model)
    suffix = model_path.suffix.lower()
    if suffix == ".xacro":
        robot_description_str = xacro.process_file(model, mappings={'name': str(robot_name)}).toxml()
    elif suffix==".urdf":
        robot_description_str = Path(model).read_text(encoding="utf-8")
    else:
        raise FileNotFoundError(f"Поддерживаются форматы файла: xacro/urdf")

    # Запуск узлов
    webots = WebotsLauncher(world=world_path, 
                            ros2_supervisor=True)

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_str,
            "use_sim_time": False
        }]
    )

    # работа с драйверами
    driver = WebotsController(
        robot_name=robot_name,
        parameters=[
            {
                "robot_description": model,
                "use_sim_time": False,
                "set_robot_state_publisher": False
            },
            LaunchConfiguration('controller').perform(context)
        ],
        respawn=True
    )

    # Во время запуска драйвера спавниться робот и ros2_controllers
    spawn_on_driver_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=driver,
            on_start=lambda evt, ctx: [
                TimerAction(period=2.0,
                            actions=_runtime_controller(robot_name=robot_name,
                                                        robot_description_str=robot_description_str,
                                                        transform_proto=transform_proto,
                                                        rotation_proto=rotation_proto,
                                                        controller_manager_timer=controller_manager))
            ]
        )
    )

    # При закрытии webots закрываем все
    shutdown_on_webots_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=webots,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    setup = [webots, 
             webots._supervisor, 
             rsp_node, 
             driver,
             spawn_on_driver_start,
             shutdown_on_webots_exit
            ]


    # Настройка rviz
    if rviz_status:

        rviz_config = PathJoinSubstitution([
            FindPackageShare(kwargs['package_name']),
            'config',
            'rviz_iiwa.rviz'
        ])

        rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='log'
        )

        # При закрытии rviz закрываем все
        shutdown_on_rviz_exit = RegisterEventHandler(
            OnProcessExit(
                target_action=rviz,
                on_exit=[EmitEvent(event=Shutdown())]
            )
        )

        setup += [rviz, shutdown_on_rviz_exit]

    return setup


def generate_launch_description():
    package_name="iiwa_bringup"
    description_pkg = "iiwa_description"

    # Объявление аргументов командной строки
    declare_model_arg = DeclareLaunchArgument(
        name="model",
        default_value=PathJoinSubstitution([
            FindPackageShare(description_pkg),
            "urdf",
            "iiwa7.urdf.xacro"
        ]),
        description="Path to robot xacro or urdf file (used to build robot_description)."
    )

    declare_robot_name_arg = DeclareLaunchArgument(
        name="robot_name",
        default_value="iiwa7",
        description="Robot name (used for TF and naming spawned robot)."
    )

    declare_world_arg = DeclareLaunchArgument(
        name="world",
        default_value=PathJoinSubstitution([
            FindPackageShare(description_pkg),
            "worlds",
            "iiwa.wbt"
        ]),
        description="Path to the Webots world (.wbt) to launch."
    )

    declare_controller_arg = DeclareLaunchArgument(
        name="controller",
        default_value=PathJoinSubstitution([
            FindPackageShare(package_name),
            "config",
            "iiwa_controller.yaml"
        ]),
        description="Path to controllers YAML file (used by spawner and controller_manager)."
    )

    declare_transform_arg = DeclareLaunchArgument(
        name="transform_proto",
        default_value="-0.25 0 0.79",
        description="Translation applied when spawning the robot in the Webots world (x y z)."
    )

    declare_rotation_arg = DeclareLaunchArgument(
        name="rotation_proto",
        default_value="0 0 1 0",
        description="Rotation (axis-angle) applied when spawning the robot in the Webots world."
    )

    declare_rviz_arg = DeclareLaunchArgument(
        name="rviz",
        default_value="0",
        description="If true|1|yes then launch RViz and joint_state_publisher_gui instead of controllers."
    )

    declare_controller_manager_arg = DeclareLaunchArgument(
        name="controller_manager",
        default_value="50",
        description="Timeout (seconds) for controller_manager spawners (--controller-manager-timeout)."
    )

    runtime_setup = OpaqueFunction(
        function=_runtime_setup,
        kwargs={'package_name': package_name}
    )

    return LaunchDescription([
        declare_model_arg,
        declare_robot_name_arg,
        declare_world_arg,
        declare_controller_arg,
        declare_transform_arg,
        declare_rotation_arg,
        declare_rviz_arg,
        declare_controller_manager_arg,
        runtime_setup,
    ])
