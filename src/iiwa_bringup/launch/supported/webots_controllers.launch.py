from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.urdf_spawner import URDFSpawner

from iiwa_utils import converter


def _setup_controllers(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    description = LaunchConfiguration("description").perform(context)
    transform = LaunchConfiguration("transform").perform(context)
    rotation = LaunchConfiguration("rotation").perform(context)
    controller_timer = LaunchConfiguration("controller_timer").perform(context)
    initial_positions_file = LaunchConfiguration("initial_positions_file").perform(
        context
    )

    robot_description = converter.load_robot_description(
        model_path=description,
        robot_name=robot_name,
        xacro_args={"initial_positions_file": initial_positions_file},
    )

    tmo = ["--controller-manager-timeout", str(controller_timer)]

    jsb = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster"] + tmo,
        parameters=[{"use_sim_time": True}],
    )

    jtc = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["iiwa_arm_controller"] + tmo,
        parameters=[{"use_sim_time": True}],
    )

    torque_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["forward_torque_controller",
                   "--inactive"] + tmo,
        parameters=[{"use_sim_time": True}]
    )

    spawner_urdf = URDFSpawner(
        name=robot_name,
        robot_description=robot_description,
        translation=transform,
        rotation=rotation,
    )

    return [jsb, jtc, torque_controller_spawner, spawner_urdf]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_setup_controllers)])
