from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node

from iiwa_utils import converter


def _setup_controllers(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    description = LaunchConfiguration("description").perform(context)
    initial_positions_file = LaunchConfiguration("initial_positions_file").perform(context)
    controller_path = LaunchConfiguration("controller_path").perform(context)
    simulate = LaunchConfiguration("simulate").perform(context).lower() in ("true", "1", "yes")

    robot_description = converter.load_robot_description(
        model_path=description,
        robot_name=robot_name,
        xacro_args={"initial_positions_file": initial_positions_file},
    )

    # СИМУЛЯЦИЯ (Gazebo)
    if simulate:
        jsb = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "30",
            ],
            parameters=[{"use_sim_time": True}],
        )

        jtc = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "iiwa_arm_controller",
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "30",
            ],
            parameters=[{"use_sim_time": True}],
        )

        jtc_after_jsb = RegisterEventHandler(
            OnProcessExit(
                target_action=jsb,
                on_exit=[jtc],
            )
        )

        return [jsb, jtc_after_jsb]

    # РЕАЛЬНЫЙ РОБОТ (FRI)
    else:
        ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                controller_path,
            ],
        )

        jsb = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "/controller_manager",
            ],
        )

        jtc = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "iiwa_arm_controller",
                "--controller-manager", "/controller_manager",
            ],
        )

        torque_controller = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "forward_torque_controller",
                "--controller-manager", "/controller_manager",
                "--inactive",
            ],
        )

        jtc_after_jsb = RegisterEventHandler(
            OnProcessExit(
                target_action=jsb,
                on_exit=[jtc, torque_controller],
            )
        )

        return [
            ros2_control_node,
            jsb,
            jtc_after_jsb,
        ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_setup_controllers)])