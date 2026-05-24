from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher

from iiwa_utils import converter


def _spawn_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    description = LaunchConfiguration("description").perform(context)
    world = LaunchConfiguration("world").perform(context)
    controller = LaunchConfiguration("controller").perform(context)
    initial_positions_file = LaunchConfiguration("initial_positions_file").perform(context)

    robot_description = converter.load_robot_description(
        model_path=description,
        robot_name=robot_name,
        xacro_args={
            "simulate": "true",
            "initial_positions_file": initial_positions_file,
        },
    )

    webots = WebotsLauncher(world=world, ros2_supervisor=True)

    driver = WebotsController(
        robot_name=robot_name,
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
                "set_robot_state_publisher": False,
            },
            controller,
        ],
        respawn=True,
    )

    shutdown_on_webots_exit = RegisterEventHandler(
        OnProcessExit(target_action=webots, on_exit=[EmitEvent(event=Shutdown())])
    )

    return [
        webots,
        webots._supervisor,
        driver,
        shutdown_on_webots_exit,
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_spawn_setup)])
