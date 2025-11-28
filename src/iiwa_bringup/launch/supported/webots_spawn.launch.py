from launch.events import Shutdown
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (IncludeLaunchDescription, 
                            OpaqueFunction,
                            TimerAction,
                            RegisterEventHandler,
                            EmitEvent)


from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def _spawn_setup(context, *args, **kwargs):
    model_path = LaunchConfiguration('model').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    world_path = LaunchConfiguration('world').perform(context)
    transform = LaunchConfiguration('transform').perform(context)
    rotation = LaunchConfiguration('rotation').perform(context)
    timer = LaunchConfiguration('controller_timer').perform(context)

    webots = WebotsLauncher(world=world_path, 
                            ros2_supervisor=True)
    
    driver = WebotsController(
        robot_name=robot_name,
        parameters=[
            {
                "robot_description": model_path,
                "use_sim_time": True,
                "set_robot_state_publisher": False
            },
            LaunchConfiguration('controller').perform(context)
        ],
        respawn=True
    )

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('iiwa_bringup'),
                'launch',
                'supported',
                'webots_controllers.launch.py'
            ])
        ),
        launch_arguments={
            'robot_name': robot_name,
            'model': model_path,
            'transform': transform,
            'rotation': rotation,
            'controller_timer': timer
        }.items()
    )

    spawn_on_driver_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=driver,
            on_start=lambda evt, ctx: [
                TimerAction(period=5.0,
                            actions=[controllers_launch]
                            )
            ]
        )
    )

    shutdown_on_webots_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=webots,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    return [webots,
            webots._supervisor, 
            driver,
            spawn_on_driver_start,
            shutdown_on_webots_exit]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_spawn_setup)
    ])