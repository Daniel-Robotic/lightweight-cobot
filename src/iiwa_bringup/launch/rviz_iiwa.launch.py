from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    package_name="iiwa_bringup"
    description_package_name = "iiwa_description"

    declare_model_arg = DeclareLaunchArgument(
        name="model",
        default_value=PathJoinSubstitution([
            FindPackageShare(description_package_name),
            "urdf",
            "iiwa7.urdf.xacro"
        ]),
        description="Path to the robot URDF/Xacro file"
    )

    declare_robot_name_arg = DeclareLaunchArgument(
        name="robot_name",
        default_value="iiwa7",
        description="Robot name fro the TF tree"
    )

    robot_description = Command([
        "xacro ", LaunchConfiguration('model'),
        " robot_name:=", LaunchConfiguration("robot_name")
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False,
        }]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{"use_sim_time": False}]
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare(package_name),
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

    shutdown_on_rviz_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rviz,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    )

    return LaunchDescription([
        declare_model_arg,
        declare_robot_name_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
        shutdown_on_rviz_exit
    ])