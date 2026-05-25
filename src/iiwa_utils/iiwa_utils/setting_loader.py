import os
from dataclasses import dataclass, fields, is_dataclass
from typing import Any, Dict, List, Optional, Type, TypeVar

import yaml
from ament_index_python.packages import get_package_share_directory

T = TypeVar("T")


@dataclass(frozen=True)
class RobotCfg:
    name: str
    ip: str
    port: int
    description: str
    fri_cycle_ms: int
    joint_position_tau: float
    active_controller: str  # "jtc" | "forward"


@dataclass(frozen=True)
class WebotsCfg:
    world: str
    transform: str
    rotation: str
    controller_timer: str
    cameras: List[str]


@dataclass(frozen=True)
class RvizCfg:
    config: str


@dataclass(frozen=True)
class DigitalTwinCfg:
    webots: WebotsCfg
    rviz: RvizCfg


@dataclass(frozen=True)
class MoveitCfg:
    srdf: str
    kinematics: str
    joint_limits: str
    pilz_limits: str
    initial_positions: str
    moveit_controllers: str
    moveit_cpp: str


@dataclass(frozen=True)
class ControllerCfg:
    controller_path: str
    moveit: MoveitCfg


@dataclass(frozen=True)
class PlanningCfg:
    pose_link: str
    planning_group: str
    default_frame: str
    default_planner: str
    planning_attempts: int


@dataclass(frozen=True)
class FoxgloveCfg:
    enabled: bool                       # Запускать ли foxglove_bridge
    port: int                           # WebSocket-порт (обычно 8765)
    address: str                        # Адрес прослушивания (0.0.0.0 = все интерфейсы)
    debug: bool                         # Подробное логирование bridge
    tls: bool                           # Включить TLS-шифрование
    certfile: str                       # Путь к SSL-сертификату (при tls=true)
    keyfile: str                        # Путь к приватному ключу SSL (при tls=true)
    topic_whitelist: List[str]          # Regex топиков, публикуемых клиенту
    param_whitelist: List[str]          # Regex ROS-параметров, видимых клиенту
    service_whitelist: List[str]        # Regex сервисов, доступных клиенту
    client_topic_whitelist: List[str]   # Regex топиков, в которые клиент может писать
    min_qos_depth: int                  # Минимальная глубина QoS-очереди
    max_qos_depth: int                  # Максимальная глубина QoS-очереди
    num_threads: int                    # Потоки bridge (0 = авто по CPU)
    send_buffer_limit: int              # Лимит буфера отправки в байтах
    use_sim_time: bool                  # Использовать /clock вместо системного времени
    capabilities: List[str]             # Возможности, открытые клиенту
    include_hidden: bool                # Показывать скрытые топики/сервисы
    asset_uri_allowlist: List[str]      # Разрешённые package://-URI для отдачи ассетов
    ignore_unresponsive_param_nodes: bool  # Не падать при зависших param-нодах


class SettingsError(RuntimeError):
    pass


@dataclass(frozen=True)
class Settings:
    robot: RobotCfg
    digital_twin: DigitalTwinCfg
    controller: ControllerCfg
    planning: PlanningCfg
    foxglove: FoxgloveCfg

    def to_dict(self) -> Dict[str, Any]:
        def _convert(obj: Any) -> Any:
            if is_dataclass(obj):
                return {f.name: _convert(getattr(obj, f.name)) for f in fields(obj)}
            if isinstance(obj, dict):
                return {k: _convert(v) for k, v in obj.items()}
            if isinstance(obj, (list, tuple)):
                return [_convert(x) for x in obj]
            return obj

        return _convert(self)

    def to_yaml(self) -> str:
        return yaml.safe_dump(self.to_dict(), sort_keys=False, allow_unicode=True)

    def save_yaml(self, path: str) -> None:
        path = os.path.abspath(path)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(self.to_yaml())

    @classmethod
    def from_yaml(cls: Type[T], settings_path: str, check_files: bool = True) -> T:
        return build_settings(settings_path=settings_path, check_files=check_files)


# Helpers
def load_yaml(path: str) -> Dict[str, Any]:
    path = os.path.abspath(path)
    if not os.path.exists(path):
        raise SettingsError(f"settings yaml not found: {path}")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise SettingsError("settings yaml root must be a mapping (dict)")
    return data


def resolve_path(value: str, settings_dir: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise SettingsError("empty path value in settings")
    value = value.strip()
    if value.startswith("pkg://"):
        rest = value[len("pkg://"):]
        if "/" not in rest:
            raise SettingsError(f"bad pkg uri: {value} (need pkg://<pkg>/<path>)")
        pkg, rel = rest.split("/", 1)
        share = get_package_share_directory(pkg)
        return os.path.normpath(os.path.join(share, rel))
    if os.path.isabs(value):
        return os.path.normpath(value)
    return os.path.normpath(os.path.join(settings_dir, value))


def require(d: Dict[str, Any], key: str) -> Any:
    if key not in d:
        raise SettingsError(f"missing key '{key}'")
    return d[key]


def assert_file(path: str, key: str) -> None:
    if not os.path.exists(path):
        raise SettingsError(f"file for '{key}' does not exist: {path}")
    if not os.path.isfile(path):
        raise SettingsError(f"path for '{key}' is not a file: {path}")


# Foxglove defaults
_FOXGLOVE_DEFAULTS: Dict[str, Any] = {
    "enabled": False,
    "port": 8765,
    "address": "0.0.0.0",
    "debug": False,
    "tls": False,
    "certfile": "",
    "keyfile": "",
    "topic_whitelist": [".*"],
    "param_whitelist": [".*"],
    "service_whitelist": [".*"],
    "client_topic_whitelist": [".*"],
    "min_qos_depth": 1,
    "max_qos_depth": 10,
    "num_threads": 0,
    "send_buffer_limit": 10_000_000,
    "use_sim_time": False,
    "capabilities": [
        "clientPublish",
        "parameters",
        "parametersSubscribe",
        "services",
        "connectionGraph",
        "assets",
    ],
    "include_hidden": False,
    "asset_uri_allowlist": [
        r"^package://(?:[-\w%]+/)*[-\w%.]+\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$"
    ],
    "ignore_unresponsive_param_nodes": True,
}


def _parse_foxglove(raw: Optional[Dict[str, Any]]) -> FoxgloveCfg:
    """Парсит секцию foxglove, подставляя дефолты для пропущенных полей."""
    if raw is None:
        raw = {}

    def get(key: str) -> Any:
        return raw.get(key, _FOXGLOVE_DEFAULTS[key])

    return FoxgloveCfg(
        enabled=bool(get("enabled")),
        port=int(get("port")),
        address=str(get("address")),
        debug=bool(get("debug")),
        tls=bool(get("tls")),
        certfile=str(get("certfile")),
        keyfile=str(get("keyfile")),
        topic_whitelist=list(get("topic_whitelist")),
        param_whitelist=list(get("param_whitelist")),
        service_whitelist=list(get("service_whitelist")),
        client_topic_whitelist=list(get("client_topic_whitelist")),
        min_qos_depth=int(get("min_qos_depth")),
        max_qos_depth=int(get("max_qos_depth")),
        num_threads=int(get("num_threads")),
        send_buffer_limit=int(get("send_buffer_limit")),
        use_sim_time=bool(get("use_sim_time")),
        capabilities=list(get("capabilities")),
        include_hidden=bool(get("include_hidden")),
        asset_uri_allowlist=list(get("asset_uri_allowlist")),
        ignore_unresponsive_param_nodes=bool(get("ignore_unresponsive_param_nodes")),
    )


def build_settings(settings_path: str, check_files: bool = True) -> Settings:
    settings_path = os.path.abspath(settings_path)
    settings_dir = os.path.dirname(settings_path)

    raw = load_yaml(settings_path)

    # robot
    robot_raw = require(raw, "robot")
    robot = RobotCfg(
        name=str(require(robot_raw, "name")),
        ip=str(require(robot_raw, "ip")),
        port=int(require(robot_raw, "port")),
        description=resolve_path(str(require(robot_raw, "description")), settings_dir),
        fri_cycle_ms=int(robot_raw.get("fri_cycle_ms", 5)),
        joint_position_tau=float(robot_raw.get("joint_position_tau", 0.04)),
        active_controller=str(robot_raw.get("active_controller", "jtc")),
    )

    # digital_twin
    dt_raw = require(raw, "digital_twin")
    webots_raw = require(dt_raw, "webots")
    cameras_raw = webots_raw.get("cameras", [])
    rviz_raw = require(dt_raw, "rviz")

    cameras = [resolve_path(str(c), settings_dir) for c in cameras_raw]

    webots = WebotsCfg(
        world=resolve_path(str(require(webots_raw, "world")), settings_dir),
        transform=str(require(webots_raw, "transform")),
        rotation=str(require(webots_raw, "rotation")),
        controller_timer=str(int(require(webots_raw, "controller_timer"))),
        cameras=cameras,
    )
    rviz = RvizCfg(
        config=resolve_path(str(require(rviz_raw, "config")), settings_dir)
    )
    digital_twin = DigitalTwinCfg(webots=webots, rviz=rviz)

    # controller + moveit
    ctrl_raw = require(raw, "controller")
    moveit_raw = require(ctrl_raw, "moveit")

    moveit = MoveitCfg(
        srdf=resolve_path(str(require(moveit_raw, "srdf")), settings_dir),
        kinematics=resolve_path(str(require(moveit_raw, "kinematics")), settings_dir),
        joint_limits=resolve_path(str(require(moveit_raw, "joint_limits")), settings_dir),
        pilz_limits=resolve_path(str(require(moveit_raw, "pilz_limits")), settings_dir),
        initial_positions=resolve_path(str(require(moveit_raw, "initial_positions")), settings_dir),
        moveit_controllers=resolve_path(str(require(moveit_raw, "moveit_controllers")), settings_dir),
        moveit_cpp=resolve_path(str(require(moveit_raw, "moveit_cpp")), settings_dir),
    )
    controller = ControllerCfg(
        controller_path=resolve_path(str(require(ctrl_raw, "controller_path")), settings_dir),
        moveit=moveit,
    )

    # planning
    planning_raw = raw.get("planning", {})
    planning = PlanningCfg(
        pose_link=str(planning_raw.get("pose_link", "link_ee")),
        planning_group=str(planning_raw.get("planning_group", "iiwa_arm")),
        default_frame=str(planning_raw.get("default_frame", "base_link")),
        default_planner=str(planning_raw.get("default_planner", "ompl")),
        planning_attempts=int(planning_raw.get("planning_attempts", 3)),
    )

    # foxglove
    foxglove = _parse_foxglove(raw.get("foxglove"))

    s = Settings(
        robot=robot,
        digital_twin=digital_twin,
        controller=controller,
        planning=planning,
        foxglove=foxglove,
    )

    if check_files:
        for i, cam_path in enumerate(s.digital_twin.webots.cameras):
            assert_file(cam_path, f"digital_twin.webots.cameras[{i}]")

        assert_file(s.robot.description, "robot.description")
        assert_file(s.digital_twin.webots.world, "digital_twin.webots.world")
        assert_file(s.digital_twin.rviz.config, "digital_twin.rviz.config")
        assert_file(s.controller.controller_path, "controller.controller_path")
        assert_file(s.controller.moveit.srdf, "controller.moveit.srdf")
        assert_file(s.controller.moveit.kinematics, "controller.moveit.kinematics")
        assert_file(s.controller.moveit.joint_limits, "controller.moveit.joint_limits")
        assert_file(s.controller.moveit.pilz_limits, "controller.moveit.pilz_limits")
        assert_file(s.controller.moveit.initial_positions, "controller.moveit.initial_positions")
        assert_file(s.controller.moveit.moveit_controllers, "controller.moveit.moveit_controllers")
        assert_file(s.controller.moveit.moveit_cpp, "controller.moveit.moveit_cpp")

    return s
