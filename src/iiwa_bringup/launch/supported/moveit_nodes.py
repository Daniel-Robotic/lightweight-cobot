from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def make_moveit_nodes(settings, robot_description: str, use_sim_time: bool):
    """Возвращает (moveit_configs, [move_group, move_to_pose_server])."""
    moveit_configs = (
        MoveItConfigsBuilder("iiwa7", package_name="iiwa_config")
        .robot_description(
            file_path=settings.robot.description,
            mappings={
                "initial_positions_file": settings.controller.moveit.initial_positions
            },
        )
        .robot_description_semantic(file_path=settings.controller.moveit.srdf)
        .robot_description_kinematics(file_path=settings.controller.moveit.kinematics)
        .joint_limits(file_path=settings.controller.moveit.joint_limits)
        .pilz_cartesian_limits(file_path=settings.controller.moveit.pilz_limits)
        .trajectory_execution(file_path=settings.controller.moveit.moveit_controllers)
        .moveit_cpp(file_path=settings.controller.moveit.moveit_cpp)
        .to_moveit_configs()
    )

    common_params = [
        moveit_configs.to_dict(),
        {"robot_description": robot_description},
        {"use_sim_time": use_sim_time},
    ]

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=common_params,
    )

    move_to_pose_server = Node(
        package="iiwa_planning",
        executable="move_to_pose_server",
        output="screen",
        parameters=[
            *common_params,
            {
                "pose_link": settings.planning.pose_link,
                "planning_group": settings.planning.planning_group,
                "default_frame": settings.planning.default_frame,
                "default_planner": settings.planning.default_planner,
                "planning_attempts": settings.planning.planning_attempts,
            },
        ],
    )

    return moveit_configs, [move_group, move_to_pose_server]
