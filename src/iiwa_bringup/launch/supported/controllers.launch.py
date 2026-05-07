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
    command_mode = LaunchConfiguration("command_mode").perform(context)

    xacro_args = {"initial_positions_file": initial_positions_file}
    
    if simulate:
        xacro_args["simulate"] = "true"

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

        torque_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=["iiwa_arm_torque_controller", "--inactive"] + tmo,
            parameters=[{"use_sim_time": True}]
        )

        jtc_after_jsb = RegisterEventHandler(
            OnProcessExit(
                target_action=jsb,
                on_exit=[jtc, torque_controller_spawner],
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

        jtc_args = ["iiwa_arm_controller", "--controller-manager", "/controller_manager"]
        torque_args = ["iiwa_arm_torque_controller", "--controller-manager", "/controller_manager"]

        if command_mode == "torque":
            jtc_args += ["--inactive"]
        else:
            torque_args += ["--inactive"]

        jtc = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=jtc_args,
        )

        torque_controller = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=torque_args,
        )

        state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=["iiwa_state_broadcaster", "--controller-manager", "/controller_manager"],
        )

        jtc_after_jsb = RegisterEventHandler(
            OnProcessExit(
                target_action=jsb,
                on_exit=[jtc, torque_controller, state_broadcaster],
            )
        )

        return [
            ros2_control_node,
            jsb,
            jtc_after_jsb,
        ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("command_mode", default_value="position"),
        OpaqueFunction(function=_setup_controllers),
    ])
