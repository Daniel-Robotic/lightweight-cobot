from launch.actions import EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_rviz_nodes(settings, moveit_configs, joint_limits_ros2, kinematics_ros2, use_sim_time: bool):
    """Возвращает [rviz_launch, shutdown_on_rviz_exit]."""
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

    return [rviz_launch, shutdown_on_rviz_exit]
