import os
from dataclasses import dataclass, fields, is_dataclass
from typing import Any, Dict, Type, TypeVar

import yaml
from ament_index_python.packages import get_package_share_directory

T = TypeVar("T")


@dataclass(frozen=True)
class RobotCfg:
    name: str
    ip: str
    port: int
    command_mode: str
    description: str


@dataclass(frozen=True)
class WebotsCfg:
    world: str
    transform: str
    rotation: str
    controller_timer: str


@dataclass(frozen=True)
class RvizCfg:
    config: str


@dataclass(frozen=True)
class DigitalTwinCfg:
    webots: WebotsCfg
    rviz: RvizCfg
    description: str


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


class SettingsError(RuntimeError):
    pass


@dataclass(frozen=True)
class Settings:
    robot: RobotCfg
    digital_twin: DigitalTwinCfg
    controller: ControllerCfg

    def to_dict(self) -> Dict[str, Any]:
        """Serialize dataclass tree -> dict (recursive)."""

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
    """
    Supported:
      - pkg://<pkg>/<rel>
      - /abs/path
      - ./relative or relative (relative to settings.yaml dir)
    Returns absolute normalized path.
    """
    if not isinstance(value, str) or not value.strip():
        raise SettingsError("empty path value in settings")

    value = value.strip()

    if value.startswith("pkg://"):
        rest = value[len("pkg://") :]
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
        command_mode=str(require(robot_raw, "command_mode")),
        description=resolve_path(str(require(robot_raw, "description")), settings_dir),
    )

    # digital_twin
    dt_raw = require(raw, "digital_twin")
    webots_raw = require(dt_raw, "webots")
    rviz_raw = require(dt_raw, "rviz")

    webots = WebotsCfg(
        world=resolve_path(str(require(webots_raw, "world")), settings_dir),
        transform=str(require(webots_raw, "transform")),
        rotation=str(require(webots_raw, "rotation")),
        controller_timer=int(require(webots_raw, "controller_timer")),
    )

    rviz = RvizCfg(config=resolve_path(str(require(rviz_raw, "config")), settings_dir))

    digital_twin = DigitalTwinCfg(
        webots=webots,
        rviz=rviz,
        description=resolve_path(str(require(dt_raw, "description")), settings_dir),
    )

    # controller, moveit
    ctrl_raw = require(raw, "controller")
    moveit_raw = require(ctrl_raw, "moveit")

    moveit = MoveitCfg(
        srdf=resolve_path(str(require(moveit_raw, "srdf")), settings_dir),
        kinematics=resolve_path(str(require(moveit_raw, "kinematics")), settings_dir),
        joint_limits=resolve_path(
            str(require(moveit_raw, "joint_limits")), settings_dir
        ),
        pilz_limits=resolve_path(str(require(moveit_raw, "pilz_limits")), settings_dir),
        initial_positions=resolve_path(
            str(require(moveit_raw, "initial_positions")), settings_dir
        ),
        moveit_controllers=resolve_path(
            str(require(moveit_raw, "moveit_controllers")), settings_dir
        ),
        moveit_cpp=resolve_path(str(require(moveit_raw, "moveit_cpp")), settings_dir),
    )

    controller = ControllerCfg(
        controller_path=resolve_path(
            str(require(ctrl_raw, "controller_path")), settings_dir
        ),
        moveit=moveit,
    )

    s = Settings(robot=robot, digital_twin=digital_twin, controller=controller)

    if check_files:
        assert_file(s.robot.description, "robot.description")
        assert_file(s.digital_twin.webots.world, "digital_twin.webots.world")
        assert_file(s.digital_twin.rviz.config, "digital_twin.rviz.config")
        assert_file(s.digital_twin.description, "digital_twin.description")
        assert_file(s.controller.controller_path, "controller.controller_path")

        assert_file(s.controller.moveit.srdf, "controller.moveit.srdf")
        assert_file(s.controller.moveit.kinematics, "controller.moveit.kinematics")
        assert_file(s.controller.moveit.joint_limits, "controller.moveit.joint_limits")
        assert_file(s.controller.moveit.pilz_limits, "controller.moveit.pilz_limits")
        assert_file(
            s.controller.moveit.initial_positions, "controller.moveit.initial_positions"
        )
        assert_file(
            s.controller.moveit.moveit_controllers,
            "controller.moveit.moveit_controllers",
        )
        assert_file(s.controller.moveit.moveit_cpp, "controller.moveit.moveit_cpp")

    return s


