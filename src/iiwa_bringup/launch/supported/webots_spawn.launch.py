from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def _spawn_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    description = LaunchConfiguration("description").perform(context)
    world = LaunchConfiguration("world").perform(context)
    transform = LaunchConfiguration("transform").perform(context)
    rotation = LaunchConfiguration("rotation").perform(context)
    controller_timer = LaunchConfiguration("controller_timer").perform(context)
    controller = LaunchConfiguration("controller").perform(context)
    initial_positions_file = LaunchConfiguration("initial_positions_file").perform(
        context
    )

    webots = WebotsLauncher(world=world, ros2_supervisor=True)

    driver = WebotsController(
        robot_name=robot_name,
        parameters=[
            {
                "robot_description": description,
                "use_sim_time": False,
                "set_robot_state_publisher": False,
            },
            controller,
        ],
        respawn=True,
    )

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("iiwa_bringup"),
                    "launch",
                    "supported",
                    "webots_controllers.launch.py",
                ]
            )
        ),
        launch_arguments={
            "robot_name": robot_name,
            "description": description,
            "transform": transform,
            "rotation": rotation,
            "controller_timer": controller_timer,
            "initial_positions_file": initial_positions_file,
        }.items(),
    )

    spawn_on_driver_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=driver,
            on_start=lambda evt, ctx: [
                TimerAction(period=5.0, actions=[controllers_launch])
            ],
        )
    )

    shutdown_on_webots_exit = RegisterEventHandler(
        OnProcessExit(target_action=webots, on_exit=[EmitEvent(event=Shutdown())])
    )

    return [
        webots,
        webots._supervisor,
        driver,
        spawn_on_driver_start,
        shutdown_on_webots_exit,
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_spawn_setup)])
