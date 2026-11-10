"""Create the per-launch world and settings copies.

The installed world template keeps the two local PROTO references for source
tree use.  Webots must open a copy whose local EXTERNPROTO paths are absolute,
because the temporary world lives outside the installed package.  The same
launch also needs a settings copy that points at that world and disables the
legacy random object spawner.
"""
from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Any

import yaml


EXPERIMENT_IDS = tuple(range(1, 21))
MODES = ('view', 'execute')
DEFAULT_START_DELAY_SEC = 3.0
DEFAULT_TRACE_DIR = '~/.ros/iiwa_experement/runs'
LOCAL_PROTOS = (
    'CollaborativeRobotTable.proto',
    'WorkspaceLimiter.proto',
)
_EXTERNPROTO_RE = re.compile(r'(EXTERNPROTO\s+")([^"]+)(")')


def validate_experiment_id(raw: str) -> int:
    """Accept only a decimal experiment number in 1..20."""
    text = str(raw).strip()
    if not text.isdigit():
        raise ValueError(f'experiment_id must be an integer in 1..20, got {raw!r}')
    value = int(text)
    if text != str(value):
        raise ValueError(f'experiment_id must be a decimal integer, got {raw!r}')
    if value not in EXPERIMENT_IDS:
        raise ValueError(f'experiment_id {value} is outside 1..20')
    return value


def validate_mode(raw: str) -> str:
    """Accept only the documented view/execute modes."""
    text = str(raw).strip().lower()
    if text not in MODES:
        raise ValueError(f'mode must be one of {list(MODES)}, got {raw!r}')
    return text


def validate_start_delay(raw: str) -> float:
    """Return a finite non-negative simulation-time delay."""
    text = str(raw).strip()
    try:
        value = float(text)
    except (TypeError, ValueError) as error:
        raise ValueError(
            f'start_delay_sec must be a finite non-negative number, got {raw!r}'
        ) from error
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(
            f'start_delay_sec must be a finite non-negative number, got {raw!r}'
        )
    return value


def validate_trace_dir(raw: str) -> str:
    text = str(raw).strip()
    if not text:
        raise ValueError('trace_dir must not be empty')
    return text


def _absolute_proto(source: str, description_share: Path) -> str:
    if source.startswith(('http://', 'https://')):
        return source
    proto_dir = description_share / 'resource' / 'protos'
    candidate = proto_dir / Path(source).name
    if not candidate.is_file():
        raise FileNotFoundError(
            f'EXTERNPROTO {source!r} was not found in {proto_dir}'
        )
    return str(candidate.resolve())


def _materialize_world(
    world_template: Path,
    description_share: Path,
    output_dir: Path,
) -> Path:
    if not world_template.is_file():
        raise FileNotFoundError(f'world template does not exist: {world_template}')
    text = world_template.read_text(encoding='utf-8')

    def replace(match: re.Match[str]) -> str:
        return match.group(1) + _absolute_proto(match.group(2), description_share) + match.group(3)

    resolved = _EXTERNPROTO_RE.sub(replace, text)
    for name in LOCAL_PROTOS:
        expected = description_share / 'resource' / 'protos' / name
        if str(expected.resolve()) not in resolved:
            raise FileNotFoundError(
                f'world template does not reference the installed {name}'
            )
    world_path = output_dir / world_template.name
    world_path.write_text(resolved, encoding='utf-8')
    return world_path


def _mapping(value: Any, label: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f'{label} must be a mapping')
    return value


def _materialize_setting(
    base_setting: Path,
    world_path: Path,
    output_dir: Path,
    moveit_cpp_path: Path | None = None,
) -> Path:
    if not base_setting.is_file():
        raise FileNotFoundError(f'base settings file does not exist: {base_setting}')
    data = _mapping(
        yaml.safe_load(base_setting.read_text(encoding='utf-8')) or {},
        'settings',
    )
    digital_twin = _mapping(data.get('digital_twin'), 'digital_twin')
    webots = _mapping(digital_twin.get('webots'), 'digital_twin.webots')
    objects = _mapping(webots.setdefault('objects', {}), 'digital_twin.webots.objects')
    objects['enabled'] = False
    objects['count'] = 0
    webots['world'] = str(world_path.resolve())
    # The interactive TCP gizmo must not exist in reference runs: it draws an
    # overlay ball and can preempt the reference route.  cobot-setting.yaml
    # only applies to `cobot run`, not to this launch.
    gizmo = _mapping(
        digital_twin.setdefault('tcp_gizmo', {}), 'digital_twin.tcp_gizmo',
    )
    gizmo['enabled'] = False

    tool = _mapping(data.setdefault('tool', {}), 'tool')
    tool['active'] = 'patron'

    # The reference executor runs many rest-to-rest plans back to back.  The
    # Webots position controller keeps a heavy tool oscillating at a few
    # hundredths of rad/s for seconds after a stop, which trips the default
    # 0.01 rad/s start-state guard and aborts otherwise valid plans.  Loosen
    # the guard for these launches only; the runner still waits for the arm
    # to settle before every plan and verifies the reached pose afterwards.
    planning = _mapping(data.setdefault('planning', {}), 'planning')
    planning['stopped_velocity_tolerance'] = 0.08

    if moveit_cpp_path is not None:
        controller = _mapping(data.setdefault('controller', {}), 'controller')
        moveit = _mapping(controller.setdefault('moveit', {}), 'controller.moveit')
        moveit['moveit_cpp'] = str(Path(moveit_cpp_path).resolve())

    setting_path = output_dir / 'setting.yaml'
    setting_path.write_text(
        yaml.safe_dump(data, sort_keys=False, allow_unicode=True),
        encoding='utf-8',
    )
    return setting_path


def prepare_runtime_files(
    base_setting: Path | str,
    world_template: Path | str,
    description_share: Path | str,
    output_dir: Path | str,
    moveit_cpp_path: Path | str | None = None,
) -> tuple[Path, Path]:
    """Write an isolated world/settings pair and return ``(world, setting)``.

    Both local PROTO references are resolved against the installed
    ``iiwa_description`` share.  External https PROTO references are preserved
    verbatim.  The base settings keep their cameras, robot transform and
    controller paths; only the world path and the legacy object spawner change.
    """
    base_setting = Path(base_setting)
    world_template = Path(world_template)
    description_share = Path(description_share)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    world_path = _materialize_world(world_template, description_share, output_dir)
    setting_path = _materialize_setting(
        base_setting,
        world_path,
        output_dir,
        None if moveit_cpp_path is None else Path(moveit_cpp_path),
    )
    return world_path, setting_path
