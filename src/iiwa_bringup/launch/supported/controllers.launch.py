from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch_ros.actions import Node
from webots_ros2_driver.urdf_spawner import URDFSpawner


from iiwa_utils import converter


def _setup_controllers(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    description = LaunchConfiguration("description").perform(context)
    transform = LaunchConfiguration("transform").perform(context)
    rotation = LaunchConfiguration("rotation").perform(context)
    initial_positions_file = LaunchConfiguration("initial_positions_file").perform(context)
    controller_timer = LaunchConfiguration("controller_timer").perform(context)
    controller_path = LaunchConfiguration("controller_path").perform(context)
    simulate = LaunchConfiguration("simulate").perform(context).lower() in ("true", "1", "yes")
    fri_cycle_ms = int(LaunchConfiguration("fri_cycle_ms").perform(context))
    if not 1 <= fri_cycle_ms <= 100:
        raise ValueError("fri_cycle_ms must be within the FRI 1.16 range 1..100 ms")
    update_rate = 1000 // fri_cycle_ms

    xacro_args = {"initial_positions_file": initial_positions_file}

    if simulate:
        xacro_args["simulate"] = "true"
    else:
        xacro_args["robot_ip"] = LaunchConfiguration("robot_ip").perform(context)
        xacro_args["fri_port"] = LaunchConfiguration("fri_port").perform(context)

    robot_description = converter.load_robot_description(
        model_path=description,
        robot_name=robot_name,
        xacro_args=xacro_args,
    )

    # Webots
    if simulate:
        tmo = ["--controller-manager-timeout", str(controller_timer)]

        spawner_urdf = URDFSpawner(
            name=robot_name,
            robot_description=robot_description,
            translation=transform,
            rotation=rotation,
        )

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

        jtc_after_jsb = RegisterEventHandler(
            OnProcessExit(
                target_action=jsb,
                on_exit=[jtc],
            )
        )

        return [spawner_urdf, jsb, jtc_after_jsb]

    # FRI
    else:
        ros2_control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                controller_path,
                {"update_rate": update_rate},
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

        cm = ["--controller-manager", "/controller_manager"]

        jtc = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=["iiwa_arm_controller"] + cm,
        )

        jtc_after_jsb = RegisterEventHandler(
            OnProcessExit(
                target_action=jsb,
                on_exit=[jtc],
            )
        )

        return [
            ros2_control_node,
            jsb,
            jtc_after_jsb,
        ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("fri_cycle_ms", default_value="5"),
        DeclareLaunchArgument("robot_ip", default_value="192.170.10.2"),
        DeclareLaunchArgument("fri_port", default_value="30200"),
        OpaqueFunction(function=_setup_controllers),
    ])
