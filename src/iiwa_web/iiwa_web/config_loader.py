from __future__ import annotations

import yaml
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional


@dataclass
class FieldDef:
    name: str
    type: str  # string | float | int | bool | float_array
    required: bool = False
    default: Any = None
    description: str = ""
    min: Optional[float] = None
    max: Optional[float] = None
    length: Optional[int] = None
    choices: Optional[list] = None
    normalize: Optional[str] = None   # "lower" | "upper"
    joint_limits: bool = False


@dataclass
class EndpointDef:
    path: str
    method: str           # GET | POST
    type: str             # topic | service | action | tf
    ros_name: str = ""
    msg_type: str = ""
    summary: str = ""
    description: str = ""
    tags: list = field(default_factory=list)
    fields: list = field(default_factory=list)          # topic: поля ответа
    response_fields: list = field(default_factory=list) # service/action: поля ответа
    request_fields: list[FieldDef] = field(default_factory=list)
    timeout: float = 5.0
    deprecated: bool = False
    enabled: bool = True
    parent_frame: str = ""  # tf: родительский фрейм
    child_frame: str = ""   # tf: дочерний фрейм


def _resolve_path(package: str, relative: str) -> Path:
    """Ищет файл сначала через ament_index, затем по пути относительно src/."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(package)) / relative
    except Exception:
        src = Path(__file__).parents[3]  # .../src/iiwa_web/iiwa_web/ -> .../src/
        return src / package / relative


def _parse_joint_limits_data(data: dict) -> tuple[list[str], list[tuple[float, float]]]:
    joints = data["joint_limits"]
    names: list[str] = []
    limits: list[tuple[float, float]] = []
    i = 1
    while f"joint{i}" in joints:
        j = joints[f"joint{i}"]
        names.append(f"joint{i}")
        limits.append((j["min_position"], j["max_position"]))
        i += 1
    return names, limits


def load_joint_names(
    package: str = "iiwa_config",
    relative: str = "config/moveit/joint_limits.yaml",
) -> list[str]:
    """Возвращает упорядоченный список имён суставов из joint_limits.yaml."""
    path = _resolve_path(package, relative)
    with open(path) as f:
        data = yaml.safe_load(f)
    names, _ = _parse_joint_limits_data(data)
    return names


def load_joint_limits(
    package: str = "iiwa_config",
    relative: str = "config/moveit/joint_limits.yaml",
) -> list[tuple[float, float]]:
    """Возвращает список (min, max) для каждого сустава по порядку joint1..jointN."""
    path = _resolve_path(package, relative)
    with open(path) as f:
        data = yaml.safe_load(f)
    _, limits = _parse_joint_limits_data(data)
    return limits


def load_api_config(
    package: str = "iiwa_config",
    relative: str = "config/api_endpoints.yaml",
) -> list[EndpointDef]:
    """Загружает описания эндпоинтов из YAML и возвращает список EndpointDef."""
    path = _resolve_path(package, relative)
    with open(path) as f:
        data = yaml.safe_load(f)

    endpoints: list[EndpointDef] = []
    for ep in data.get("endpoints", []):
        if not ep.get("enabled", True):
            continue

        request_fields = [
            FieldDef(
                name=rf["name"],
                type=rf["type"],
                required=rf.get("required", False),
                default=rf.get("default"),
                description=rf.get("description", ""),
                min=rf.get("min"),
                max=rf.get("max"),
                length=rf.get("length"),
                choices=rf.get("choices"),
                normalize=rf.get("normalize"),
                joint_limits=rf.get("joint_limits", False),
            )
            for rf in ep.get("request_fields", [])
        ]

        endpoints.append(EndpointDef(
            path=ep["path"],
            method=ep["method"].upper(),
            type=ep["type"],
            ros_name=ep.get("ros_name", ""),
            msg_type=ep.get("msg_type", ""),
            summary=ep.get("summary", ""),
            description=ep.get("description", ""),
            tags=ep.get("tags", []),
            fields=ep.get("fields", []),
            response_fields=ep.get("response_fields", []),
            request_fields=request_fields,
            timeout=ep.get("timeout", 5.0),
            deprecated=ep.get("deprecated", False),
            parent_frame=ep.get("parent_frame", ""),
            child_frame=ep.get("child_frame", ""),
        ))

    return endpoints
