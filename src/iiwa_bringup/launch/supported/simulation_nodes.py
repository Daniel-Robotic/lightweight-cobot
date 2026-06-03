import json

from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController

from iiwa_utils.camera_spawner import load_camera_config, build_ros_urdf  # type: ignore


def make_simulation_nodes(settings) -> list:
    """Возвращает ноды камер для симуляции: [camera_spawner, *camera_controllers].

    Если камеры не заданы в настройках — возвращает пустой список.
    """
    if not settings.digital_twin.webots.cameras:
        return []

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

    return [camera_spawner, *camera_controllers]
