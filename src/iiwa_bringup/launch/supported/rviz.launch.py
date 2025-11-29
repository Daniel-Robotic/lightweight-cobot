from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_package_arg = DeclareLaunchArgument(
        "package_name",
        default_value="iiwa_bringup",
        description="Package name where RViz config is stored",
    )

    package_name = LaunchConfiguration("package_name")

    rviz_config = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "rviz_iiwa.rviz"]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="log",
    )

    # При закрытии rviz закрываем все
    shutdown_on_rviz_exit = RegisterEventHandler(
        OnProcessExit(target_action=rviz, on_exit=[EmitEvent(event=Shutdown())])
    )

    return LaunchDescription([declare_package_arg, rviz, shutdown_on_rviz_exit])
