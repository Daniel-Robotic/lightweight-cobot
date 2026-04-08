from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import OpaqueFunction
from launch import LaunchDescription
from launch_ros.actions import Node


from iiwa_utils import converter


def _setup_controllers(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    description = LaunchConfiguration("description").perform(context)
    initial_positions_file = LaunchConfiguration("initial_positions_file").perform(
        context
    )
    controller_path = LaunchConfiguration("controller_path").perform(context)

    robot_description = converter.load_robot_description(
        model_path=description,
        robot_name=robot_name,
        xacro_args={"initial_positions_file": initial_positions_file},
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            controller_path,
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["iiwa_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    return [
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_after_jsb,
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_setup_controllers)])