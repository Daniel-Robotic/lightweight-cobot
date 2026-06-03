from dataclasses import asdict

from launch_ros.actions import Node


def make_foxglove_node(settings, use_sim_time: bool) -> Node:
    params = asdict(settings.foxglove)
    params.pop("enabled")
    params["use_sim_time"] = use_sim_time

    return Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        output="screen",
        name="foxglove_bridge",
        parameters=[params],
    )


def make_web_server_node(settings, use_sim_time: bool) -> Node:
    return Node(
        package="iiwa_web",
        executable="iiwa_web_server",
        output="screen",
        name="iiwa_web_server",
        parameters=[{
            "host": settings.web.host,
            "port": settings.web.port,
            "endpoints_path": settings.web.endpoints,
            "joint_limits_path": settings.web.joint_limits,
            "use_sim_time": use_sim_time,
        }],
    )
