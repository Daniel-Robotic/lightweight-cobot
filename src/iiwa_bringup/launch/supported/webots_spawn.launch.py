import tempfile

import yaml
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

    # robot_description cannot be passed via -p key:=value: rcl's YAML parser
    # chokes on XML content (quotes, angle brackets). Merge all params into one
    # temp file and pass via --params-file. A minimal dict triggers --ros-args
    # so WebotsController includes the required prefix in the command.
    with open(controller, "r") as f:
        controller_cfg = yaml.safe_load(f) or {}

    combined = {
        "/**": {"ros__parameters": {
            "robot_description": robot_description,
            "set_robot_state_publisher": False,
        }}
    }
    combined.update(controller_cfg)

    with tempfile.NamedTemporaryFile(
        mode="w", suffix=".yaml", delete=False, prefix="iiwa7_webots_"
    ) as f:
        yaml.dump(combined, f, default_flow_style=False, allow_unicode=True)
        params_file = f.name

    webots = WebotsLauncher(world=world, ros2_supervisor=True)

    driver = WebotsController(
        robot_name=robot_name,
        parameters=[
            {"use_sim_time": True},  # dict → triggers --ros-args prefix
            params_file,             # file → --params-file (robot_description + controllers)
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
