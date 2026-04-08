from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _spawn_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    world = LaunchConfiguration("world").perform(context)
    gazebo_config = LaunchConfiguration("gazebo_config").perform(context)
    simulate = LaunchConfiguration("simulate").perform(context).lower() in ("true", "1", "yes")
    # transform = LaunchConfiguration("transform").perform(context)
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": f"-r {world} --render-engine ogre",
            "on_exit_shutdown": "true",
            }.items(),
    )

    # Table Spawner
    spawn_table = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_table",
        output="screen",
        arguments=[
            "-file", str(
                get_package_share_directory("iiwa_description")
                + "/urdf/table/table.urdf"
            ),
            "-name",  "iiwa_table",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.0",
        ],
    )

    # Robot Spawner
    # TODO: добавить аргументы для transform и rotation
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_iiwa7",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", robot_name,
            "-x", "-0.25",
            "-y", "0.0",
            "-z", "0.81"
        ],
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{
            "config_file": gazebo_config,
            "use_sim_time": simulate
        }],
    )

    return [
        gazebo,
        spawn_table,
        spawn_robot, 
        gz_bridge
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_spawn_setup)])
