import json
from dataclasses import asdict

from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController

from iiwa_utils.camera_spawner import load_camera_config, build_ros_urdf  # type: ignore


def make_simulation_nodes(settings) -> list:
    """Create the configured object spawner and camera nodes for Webots."""
    nodes = []
    webots = settings.digital_twin.webots
    if webots.objects.enabled and webots.objects.count:
        nodes.append(Node(
            package="iiwa_utils",
            executable="object_spawner",
            name="object_spawner",
            output="screen",
            parameters=[{
                "objects_config": json.dumps(asdict(webots.objects)),
                "robot_translation": webots.transform,
                "robot_rotation": webots.rotation,
            }],
        ))
    if not settings.digital_twin.webots.cameras:
        return nodes

    camera_spawner = Node(
        package="iiwa_utils",
        executable="camera_spawner",
        name="camera_spawner",
        output="screen",
        parameters=[{
            "camera_configs": json.dumps(settings.digital_twin.webots.cameras)
        }],
    )

    camera_controllers = []
    for cam_path in settings.digital_twin.webots.cameras:
        cam_cfg = load_camera_config(cam_path)
        camera_controllers.append(WebotsController(
            robot_name=f"{cam_cfg.name}_robot",
            parameters=[{
                "robot_description": build_ros_urdf(cam_cfg),
                "use_sim_time": True,
                "set_robot_state_publisher": False,
            }],
            respawn=True,
        ))

    return [*nodes, camera_spawner, *camera_controllers]
