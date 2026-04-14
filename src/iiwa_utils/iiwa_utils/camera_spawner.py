"""
Документация Webots:
  Camera:      https://cyberbotics.com/doc/reference/camera
  RangeFinder: https://cyberbotics.com/doc/reference/rangefinder
"""

import json
import os
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import yaml
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from webots_ros2_msgs.srv import SpawnNodeFromString


_SKIP_FIELDS = {"ros"}
_CAMERA_BOOL_FIELDS = {"antiAliasing"}
_RANGE_FINDER_BOOL_FIELDS: set = set()


@dataclass(frozen=True)
class RosCfg:
    topic: str
    update_rate: int


@dataclass(frozen=True)
class CameraDeviceCfg:
    webots_fields: Dict[str, Any]
    ros: RosCfg


@dataclass(frozen=True)
class RangeFinderDeviceCfg:
    webots_fields: Dict[str, Any]
    ros: RosCfg


@dataclass(frozen=True)
class CameraConfig:
    name: str
    translation: str
    rotation: str
    camera: Optional[CameraDeviceCfg]
    range_finder: Optional[RangeFinderDeviceCfg]


def _parse_device_block(
    raw: Optional[Dict[str, Any]],
    bool_fields: set,
    default_topic: str,
    default_rate: int,
) -> Optional[Dict[str, Any]]:
    """
    Парсит блок camera или range_finder из YAML.
    Возвращает None если блок отсутствует.
    """
    if raw is None:
        return None

    ros_raw = raw.get("ros", {})
    ros = RosCfg(
        topic=str(ros_raw.get("topic", default_topic)),
        update_rate=int(ros_raw.get("update_rate", default_rate)),
    )

    webots_fields: Dict[str, Any] = {}
    for key, value in raw.items():
        if key in _SKIP_FIELDS:
            continue
        if key in bool_fields:
            webots_fields[key] = "TRUE" if value else "FALSE"
        else:
            webots_fields[key] = value

    return {"webots_fields": webots_fields, "ros": ros}


def load_camera_config(path: str) -> CameraConfig:
    """Загружает camera YAML и возвращает CameraConfig."""
    path = os.path.abspath(path)
    if not os.path.exists(path):
        raise FileNotFoundError(f"Camera config not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        raw: Dict[str, Any] = yaml.safe_load(f) or {}

    name = str(raw["name"])
    translation = str(raw["translation"])
    rotation = str(raw["rotation"])

    cam_raw = _parse_device_block(
        raw.get("camera"),
        _CAMERA_BOOL_FIELDS,
        default_topic=f"/{name}/image_raw",
        default_rate=30,
    )
    rf_raw = _parse_device_block(
        raw.get("range_finder"),
        _RANGE_FINDER_BOOL_FIELDS,
        default_topic=f"/{name}/depth/image_raw",
        default_rate=15,
    )

    camera = (
        CameraDeviceCfg(webots_fields=cam_raw["webots_fields"], ros=cam_raw["ros"])
        if cam_raw is not None else None
    )
    range_finder = (
        RangeFinderDeviceCfg(webots_fields=rf_raw["webots_fields"], ros=rf_raw["ros"])
        if rf_raw is not None else None
    )

    return CameraConfig(
        name=name,
        translation=translation,
        rotation=rotation,
        camera=camera,
        range_finder=range_finder,
    )



def _build_device_proto(device_type: str, device_name: str, fields: Dict[str, Any]) -> str:
    """Строит PROTO-строку для Camera или RangeFinder."""
    parts = [f'name "{device_name}"']
    for key, value in fields.items():
        if isinstance(value, str) and value not in ("TRUE", "FALSE"):
            parts.append(f'{key} "{value}"')
        else:
            parts.append(f'{key} {value}')
    return f'{device_type} {{ {" ".join(parts)} }}'


def build_robot_proto(cfg: CameraConfig) -> str:
    """
    Строит Webots PROTO-строку Robot-ноды, содержащей Camera и/или RangeFinder.
    Результат передаётся в SpawnNodeFromString.Request.data.
    """
    children: List[str] = []

    if cfg.camera is not None:
        children.append(
            _build_device_proto("Camera", cfg.name, cfg.camera.webots_fields)
        )

    if cfg.range_finder is not None:
        children.append(
            _build_device_proto("RangeFinder", f"{cfg.name}_depth", cfg.range_finder.webots_fields)
        )

    children_str = " ".join(children)

    return (
        f'Robot {{'
        f' name "{cfg.name}_robot"'
        f' translation {cfg.translation}'
        f' rotation {cfg.rotation}'
        f' children [ {children_str} ]'
        f' controller "<extern>"'
        f' }}'
    )


def build_ros_urdf(cfg: CameraConfig) -> str:
    """
    Строит URDF-строку для WebotsController.
    Описывает ROS2-интерфейс Camera и/или RangeFinder.
    """
    devices: List[str] = []

    if cfg.camera is not None:
        ros = cfg.camera.ros
        devices.append(
            f'    <device reference="{cfg.name}" type="Camera">\n'
            f'      <ros>\n'
            f'        <topicName>{ros.topic}</topicName>\n'
            f'        <updateRate>{ros.update_rate}</updateRate>\n'
            f'        <alwaysOn>True</alwaysOn>\n'
            f'        <frameName>{cfg.name}_link</frameName>\n'
            f'      </ros>\n'
            f'    </device>'
        )

    if cfg.range_finder is not None:
        ros = cfg.range_finder.ros
        devices.append(
            f'    <device reference="{cfg.name}_depth" type="RangeFinder">\n'
            f'      <ros>\n'
            f'        <topicName>{ros.topic}</topicName>\n'
            f'        <updateRate>{ros.update_rate}</updateRate>\n'
            f'        <alwaysOn>True</alwaysOn>\n'
            f'        <frameName>{cfg.name}_link</frameName>\n'
            f'      </ros>\n'
            f'    </device>'
        )

    devices_str = "\n".join(devices)

    return (
        f'<?xml version="1.0"?>\n'
        f'<robot name="{cfg.name}_robot">\n'
        f'  <link name="{cfg.name}_link"/>\n'
        f'  <webots>\n'
        f'{devices_str}\n'
        f'  </webots>\n'
        f'</robot>'
    )


class CameraSpawner(Node):
    """
    Спавнит камеры в Webots через /Ros2Supervisor/spawn_node_from_string.

    Параметры ROS2 ноды:
        camera_configs (string): JSON-массив абсолютных путей до camera YAML файлов.
    """

    def __init__(self):
        super().__init__("camera_spawner")

        self.declare_parameter("camera_configs", "[]")
        configs_json = (
            self.get_parameter("camera_configs")
            .get_parameter_value()
            .string_value
        )
        config_paths: List[str] = json.loads(configs_json)

        self._pending: List[CameraConfig] = []
        for path in config_paths:
            try:
                cfg = load_camera_config(path)
                self._pending.append(cfg)
                self.get_logger().info(f"Loaded camera config: '{cfg.name}' from {path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load camera config '{path}': {e}")

        if not self._pending:
            self.get_logger().info("No cameras to spawn.")
            return

        self._in_flight = False

        self._client = self.create_client(
            SpawnNodeFromString, "/Ros2Supervisor/spawn_node_from_string"
        )

        self.get_logger().info("Waiting for /Ros2Supervisor/spawn_node_from_string...")
        while not self._client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warning(
                "Service /Ros2Supervisor/spawn_node_from_string not available, retrying..."
            )

        self._timer = self.create_timer(0.1, self._tick)

    def _tick(self):
        if not self._pending:
            self.get_logger().info("All cameras spawned.")
            self._timer.cancel()
            return

        if self._in_flight:
            return

        cfg = self._pending[0]
        proto = build_robot_proto(cfg)

        self.get_logger().info(f"Spawning camera '{cfg.name}'...")
        self.get_logger().debug(f"Proto:\n{proto}")

        req = SpawnNodeFromString.Request(data=proto, check_fields=True)
        self._in_flight = True
        future = self._client.call_async(req)
        future.add_done_callback(lambda f: self._on_spawned(f, cfg))

    def _on_spawned(self, future: Future, cfg: CameraConfig):
        try:
            response = future.result()
            self.get_logger().info(f"Camera '{cfg.name}' spawned: {response}")
        except Exception as e:
            self.get_logger().error(f"Failed to spawn camera '{cfg.name}': {e}")
        finally:
            self._pending.pop(0)
            self._in_flight = False



def main(args=None):
    try:
        rclpy.init(args=args)
        node = CameraSpawner()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
