"""Strict loader for the deterministic grant experiment manifest.

The manifest in ``config/scenes.yaml`` is the only source of object poses for
the twenty experiments.  Everything in this module is pure Python so that the
same validation runs in unit tests and at launch time.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml


CAP_STATES = ('absent', 'free', 'installed')
TOOL_NAMES = ('vacuum', 'magnetic_tool', 'sealant_tool', 'torque_tool')
EXPERIMENT_IDS = tuple(range(1, 21))
SEALANT_SIDES = ('side_1', 'side_2', 'side_3', 'side_4')
SCREW_HOLES = ('hole_1', 'hole_2', 'hole_3', 'hole_4')

# Bounds of lotok.obj after its local Y axis is mapped to Webots Z. They are
# repeated here so validation does not need a Webots process or the mesh file.
TRAY_HALF_LENGTH = 0.060
TRAY_HALF_WIDTH = 0.035
SCREW_RADIUS = 0.00275

# Failure metadata is consumed by the reference executor. It never changes
# the initial pose of an object. Screws are handed to the virtual operator:
# there is deliberately no tightening or seating failure.
_ALLOWED_FAILURES = {
    'grab': {'grasp_failed'},
    'verify': {'not_ok'},
    'dispense': {'interrupted'},
}


class ScenarioError(ValueError):
    """Raised when the manifest or a requested experiment is invalid."""


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    yaw_deg: float

    @property
    def xy(self) -> tuple[float, float]:
        return self.x, self.y


@dataclass(frozen=True)
class Tool:
    name: str
    pose: Pose


@dataclass(frozen=True)
class Screw:
    name: str
    pose: Pose


@dataclass(frozen=True)
class Failure:
    skill: str
    outcome: str
    target: str | None = None
    attempt: int | None = None
    details: tuple[tuple[str, Any], ...] = ()

    def as_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {'skill': self.skill, 'outcome': self.outcome}
        if self.target is not None:
            result['target'] = self.target
        if self.attempt is not None:
            result['attempt'] = self.attempt
        if self.details:
            result['details'] = dict(self.details)
        return result


@dataclass(frozen=True)
class ScenarioSpec:
    experiment_id: int
    fixture: Pose
    body: Pose
    rack: Pose
    cap_state: str
    cap_pose: Pose | None
    tools: tuple[Tool, ...]
    tray_pose: Pose | None
    screws: tuple[Screw, ...]
    failures: tuple[Failure, ...]
    sealant_sides: tuple[str, ...] = ()
    screw_holes: tuple[str, ...] = ()

    @property
    def tool_names(self) -> tuple[str, ...]:
        return tuple(tool.name for tool in self.tools)

    @property
    def delivered_screw_count(self) -> int:
        """Number of screws handed to the operator in this experiment."""
        return len(self.screw_holes)


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ScenarioError(f'{label} must be a mapping')
    return value


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ScenarioError(f'{label} must be a finite number')
    result = float(value)
    if not math.isfinite(result):
        raise ScenarioError(f'{label} must be a finite number')
    return result


def _pose(raw: Any, label: str) -> Pose:
    data = _mapping(raw, label)
    for key in ('x', 'y', 'z'):
        if key not in data:
            raise ScenarioError(f'{label}.{key} is required')
    return Pose(
        x=_number(data['x'], f'{label}.x'),
        y=_number(data['y'], f'{label}.y'),
        z=_number(data['z'], f'{label}.z'),
        yaw_deg=_number(data.get('yaw_deg', 0.0), f'{label}.yaw_deg'),
    )


def _merge_pose(base: Pose, override: Any, label: str) -> Pose:
    if override is None:
        return base
    data = _mapping(override, label)
    merged = {
        'x': base.x,
        'y': base.y,
        'z': base.z,
        'yaw_deg': base.yaw_deg,
    }
    merged.update(data)
    return _pose(merged, label)


def _same_angle(first: float, second: float) -> bool:
    delta = (first - second) % 360.0
    return math.isclose(min(delta, 360.0 - delta), 0.0, abs_tol=1e-9)


def _require_mapping(data: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    if key not in data:
        raise ScenarioError(f"manifest is missing '{key}'")
    return _mapping(data[key], key)


def _experiment_table(data: Mapping[str, Any]) -> dict[int, Mapping[str, Any]]:
    raw = _require_mapping(data, 'experiments')
    parsed: dict[int, Mapping[str, Any]] = {}
    for raw_id, entry in raw.items():
        text = str(raw_id).strip()
        if isinstance(raw_id, bool) or not text.isdigit():
            raise ScenarioError(f'invalid experiment id {raw_id!r} in manifest')
        experiment_id = int(text)
        if experiment_id not in EXPERIMENT_IDS or text != str(experiment_id):
            raise ScenarioError(f'experiment id {raw_id!r} is outside 1..20')
        if experiment_id in parsed:
            raise ScenarioError(f'duplicate experiment id {experiment_id} in manifest')
        parsed[experiment_id] = _mapping(entry, f'experiments.{experiment_id}')
    if set(parsed) != set(EXPERIMENT_IDS):
        missing = sorted(set(EXPERIMENT_IDS) - set(parsed))
        extra = sorted(set(parsed) - set(EXPERIMENT_IDS))
        raise ScenarioError(
            f'manifest must define experiments 1..20; missing={missing}, extra={extra}'
        )
    return parsed


def _tool_poses(poses: Mapping[str, Any]) -> dict[str, Pose]:
    raw = _mapping(poses.get('tools'), 'poses.tools')
    missing = [name for name in TOOL_NAMES if name not in raw]
    extra = sorted(set(raw) - set(TOOL_NAMES))
    if missing or extra:
        raise ScenarioError(
            f'poses.tools must define {list(TOOL_NAMES)}; missing={missing}, extra={extra}'
        )
    return {name: _pose(raw[name], f'poses.tools.{name}') for name in TOOL_NAMES}


def _tools(raw: Any, poses: Mapping[str, Pose]) -> tuple[Tool, ...]:
    if not isinstance(raw, list) or not raw:
        raise ScenarioError('experiment tools must be a nonempty list')
    tools: list[Tool] = []
    seen: set[str] = set()
    for index, name in enumerate(raw):
        if not isinstance(name, str):
            raise ScenarioError(f'tools[{index}] must be a tool name')
        if name not in TOOL_NAMES:
            raise ScenarioError(
                f'unknown tool {name!r}; expected one of {list(TOOL_NAMES)}'
            )
        if name in seen:
            raise ScenarioError(f'duplicate tool {name!r}')
        seen.add(name)
        tools.append(Tool(name, poses[name]))
    return tuple(tools)


def _rotate_offset(dx: float, dy: float, yaw_deg: float) -> tuple[float, float]:
    angle = math.radians(yaw_deg)
    cosine, sine = math.cos(angle), math.sin(angle)
    return dx * cosine - dy * sine, dx * sine + dy * cosine


def _screws(
    raw_entry: Mapping[str, Any],
    raw_manifest: Mapping[str, Any],
    experiment_id: int,
    tray_pose: Pose | None,
) -> tuple[Screw, ...]:
    explicit = raw_entry.get('screws')
    if tray_pose is None:
        if explicit is not None:
            raise ScenarioError(
                f'experiment {experiment_id} has screws but no tray'
            )
        return ()

    if explicit is None:
        screw_data = _mapping(raw_manifest.get('screws'), 'screws')
        layout = screw_data.get('layout')
        if not isinstance(layout, list):
            raise ScenarioError('screws.layout must be a list')
        z = _number(screw_data.get('z'), 'screws.z')
    else:
        if not isinstance(explicit, list):
            raise ScenarioError(f'experiments.{experiment_id}.screws must be a list')
        layout = explicit
        screw_data = _mapping(raw_manifest.get('screws'), 'screws')
        z = _number(screw_data.get('z'), 'screws.z')

    if len(layout) != 10:
        raise ScenarioError(
            f'experiment {experiment_id} has {len(layout)} screws; '
            'a tray scenario must contain exactly 10 screws'
        )

    result: list[Screw] = []
    names: set[str] = set()
    for index, raw_screw in enumerate(layout):
        label = f'experiments.{experiment_id}.screws[{index}]'
        data = _mapping(raw_screw, label)
        name = data.get('name')
        if not isinstance(name, str) or not name:
            raise ScenarioError(f'{label}.name must be a nonempty string')
        if name in names:
            raise ScenarioError(f'duplicate screw name {name!r}')
        names.add(name)
        dx = _number(data.get('dx'), f'{label}.dx')
        dy = _number(data.get('dy'), f'{label}.dy')
        if abs(dx) > TRAY_HALF_LENGTH - SCREW_RADIUS or abs(dy) > TRAY_HALF_WIDTH - SCREW_RADIUS:
            raise ScenarioError(
                f'screw {name!r} at offset ({dx}, {dy}) is outside the tray'
            )
        world_dx, world_dy = _rotate_offset(dx, dy, tray_pose.yaw_deg)
        result.append(Screw(
            name=name,
            pose=Pose(tray_pose.x + world_dx, tray_pose.y + world_dy, z, tray_pose.yaw_deg),
        ))
    return tuple(result)


def _named_list(
    raw: Any,
    label: str,
    allowed: tuple[str, ...],
    experiment_id: int,
) -> tuple[str, ...]:
    if raw is None:
        return ()
    if not isinstance(raw, list):
        raise ScenarioError(f'{label} must be a list')
    result: list[str] = []
    for index, name in enumerate(raw):
        if not isinstance(name, str) or name not in allowed:
            raise ScenarioError(
                f'{label}[{index}] must be one of {list(allowed)}'
            )
        if name in result:
            raise ScenarioError(f'{label} contains duplicate {name!r}')
        result.append(name)
    return tuple(result)


def _failures(raw: Any, experiment_id: int) -> tuple[Failure, ...]:
    if raw is None:
        return ()
    if not isinstance(raw, list):
        raise ScenarioError(f'experiments.{experiment_id}.failures must be a list')
    result: list[Failure] = []
    for index, entry in enumerate(raw):
        label = f'experiments.{experiment_id}.failures[{index}]'
        data = _mapping(entry, label)
        skill = data.get('skill')
        outcome = data.get('outcome')
        if skill not in _ALLOWED_FAILURES:
            raise ScenarioError(f'{label}.skill must be one of {sorted(_ALLOWED_FAILURES)}')
        if outcome not in _ALLOWED_FAILURES[skill]:
            raise ScenarioError(
                f'{label}.outcome {outcome!r} is not valid for skill {skill!r}'
            )
        target = data.get('target')
        if target is not None and not isinstance(target, str):
            raise ScenarioError(f'{label}.target must be a string')
        attempt = data.get('attempt')
        if attempt is not None:
            if isinstance(attempt, bool) or not isinstance(attempt, int) or attempt < 1:
                raise ScenarioError(f'{label}.attempt must be a positive integer')
        details_raw = data.get('details', {})
        details = _mapping(details_raw, f'{label}.details')
        result.append(Failure(
            skill=skill,
            outcome=outcome,
            target=target,
            attempt=attempt,
            details=tuple(sorted(details.items())),
        ))
    return tuple(result)


def load_scenario(experiment_id: int, manifest_path: Path | str) -> ScenarioSpec:
    """Load and fully validate one experiment from the YAML manifest."""
    if isinstance(experiment_id, bool) or not isinstance(experiment_id, int):
        raise ScenarioError(f'experiment id must be an integer in 1..20, got {experiment_id!r}')
    if experiment_id not in EXPERIMENT_IDS:
        raise ScenarioError(f'experiment id {experiment_id} is outside 1..20')

    path = Path(manifest_path)
    if not path.is_file():
        raise FileNotFoundError(f'scenario manifest does not exist: {path}')
    try:
        raw = yaml.safe_load(path.read_text(encoding='utf-8'))
    except yaml.YAMLError as error:
        raise ScenarioError(f'invalid YAML in {path}: {error}') from error
    data = _mapping(raw, 'manifest')
    if data.get('schema_version') != 1:
        raise ScenarioError('manifest schema_version must be 1')

    experiments = _experiment_table(data)
    entry = experiments[experiment_id]
    poses = _require_mapping(data, 'poses')
    fixture = _merge_pose(
        _pose(poses.get('fixture'), 'poses.fixture'),
        entry.get('fixture'),
        f'experiments.{experiment_id}.fixture',
    )
    body = _merge_pose(
        _pose(poses.get('body'), 'poses.body'),
        entry.get('body'),
        f'experiments.{experiment_id}.body',
    )
    if not (
        math.isclose(fixture.x, body.x, abs_tol=1e-9)
        and math.isclose(fixture.y, body.y, abs_tol=1e-9)
        and _same_angle(fixture.yaw_deg, body.yaw_deg)
    ):
        raise ScenarioError(
            f'experiment {experiment_id}: fixture and body must share X, Y and yaw'
        )
    rack = _pose(poses.get('rack'), 'poses.rack')
    tool_poses = _tool_poses(poses)

    cap_state = entry.get('cap', 'free')
    if cap_state not in CAP_STATES:
        raise ScenarioError(
            f'experiment {experiment_id}: cap must be one of {list(CAP_STATES)}'
        )
    if cap_state == 'absent':
        if entry.get('cap_pose') is not None:
            raise ScenarioError(f'experiment {experiment_id}: an absent cap cannot have a pose')
        cap_pose = None
    elif cap_state == 'free':
        cap_pose = _merge_pose(
            _pose(poses.get('cap_free'), 'poses.cap_free'),
            entry.get('cap_pose'),
            f'experiments.{experiment_id}.cap_pose',
        )
    else:
        cap_pose = _merge_pose(
            _pose(poses.get('cap_installed'), 'poses.cap_installed'),
            entry.get('cap_pose'),
            f'experiments.{experiment_id}.cap_pose',
        )
        if not (
            math.isclose(cap_pose.x, body.x, abs_tol=1e-9)
            and math.isclose(cap_pose.y, body.y, abs_tol=1e-9)
            and _same_angle(cap_pose.yaw_deg, body.yaw_deg)
        ):
            raise ScenarioError(
                f'experiment {experiment_id}: an installed cap must share the body pose'
            )

    tray_enabled = entry.get('tray', False)
    if not isinstance(tray_enabled, bool):
        raise ScenarioError(f'experiments.{experiment_id}.tray must be true or false')
    if tray_enabled:
        tray_pose = _merge_pose(
            _pose(poses.get('tray'), 'poses.tray'),
            entry.get('tray_pose'),
            f'experiments.{experiment_id}.tray_pose',
        )
    else:
        if entry.get('tray_pose') is not None:
            raise ScenarioError(f'experiment {experiment_id}: tray_pose requires tray: true')
        tray_pose = None

    sealant_sides = _named_list(
        entry.get('sealant'), f'experiments.{experiment_id}.sealant',
        SEALANT_SIDES, experiment_id,
    )
    if sealant_sides and 'sealant_tool' not in entry.get('tools', []):
        raise ScenarioError(
            f'experiment {experiment_id}: sealant sides require the sealant_tool'
        )

    screw_holes = _named_list(
        entry.get('screw_holes'), f'experiments.{experiment_id}.screw_holes',
        SCREW_HOLES, experiment_id,
    )
    if tray_pose is None and screw_holes:
        raise ScenarioError(
            f'experiment {experiment_id}: screw holes require a screw tray'
        )
    if tray_pose is not None and not screw_holes:
        raise ScenarioError(
            f'experiment {experiment_id}: a screw tray requires screw_holes'
        )
    if screw_holes and 'magnetic_tool' not in entry.get('tools', []):
        raise ScenarioError(
            f'experiment {experiment_id}: screw holes require the magnetic_tool'
        )

    return ScenarioSpec(
        experiment_id=experiment_id,
        fixture=fixture,
        body=body,
        rack=rack,
        cap_state=cap_state,
        cap_pose=cap_pose,
        tools=_tools(entry.get('tools'), tool_poses),
        tray_pose=tray_pose,
        screws=_screws(entry, data, experiment_id, tray_pose),
        failures=_failures(entry.get('failures'), experiment_id),
        sealant_sides=sealant_sides,
        screw_holes=screw_holes,
    )
