from launch_ros.actions import Node
from webots_ros2_driver.urdf_spawner import URDFSpawner

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from iiwa_bringup.utils import converter


def _setup_controllers(context, *args, **kwargs):
    model_path = LaunchConfiguration('model').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    transform = LaunchConfiguration('transform').perform(context)
    rotation = LaunchConfiguration('rotation').perform(context)
    timer = LaunchConfiguration('controller_timer').perform(context)

    robot_description = converter.load_robot_description(model_path=model_path,
                                                         robot_name=robot_name)
    
    tmo = ['--controller-manager-timeout', str(timer)]

    jsb = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'] + tmo,
        parameters=[{'use_sim_time': False}],
    )

    jtc = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['iiwa_arm_controller'] + tmo,
        parameters=[{'use_sim_time': False}],
    )

    spawner_urdf = URDFSpawner(
        name=robot_name,
        robot_description=robot_description,
        translation=transform,
        rotation=rotation
    )

    return [jsb, jtc, spawner_urdf]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_setup_controllers)
    ])