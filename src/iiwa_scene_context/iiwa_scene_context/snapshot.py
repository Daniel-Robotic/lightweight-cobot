"""Pure snapshot builders for the external planner contract.

The snapshot is a live view of the selected experiment: only objects that the
Webots supervisor really created are listed, every pose carries its frame,
source and validity stamp, and derived goals use the same geometry module as
the graph executor.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

import numpy as np
from scipy.spatial.transform import Rotation

from iiwa_experement.geometry import (
    GeometrySpec,
    Transform,
    load_geometry,
    object_pose_transform,
    sealant_point,
    tcp_goal,
    world_target,
)
from iiwa_experement.scenario import ScenarioSpec

from .catalog import (
    CATALOG_VERSION,
    CONTRACT_VERSION,
    SCENE_SCHEMA_VERSION,
    SkillCatalog,
    build_component_entry,
    destination_entry,
    tool_entry,
)


DEFAULT_CAMERAS = {
    'top': {
        'topic': '/d455_top/image_raw/image_color',
        'role': 'overview',
        'required_for': ['detect'],
    },
    'hand': {
        'topic': '/camera/color/image_color',
        'role': 'close_inspection',
        'required_for': ['detect', 'capture'],
    },
}

SERVICE_OBJECTS = {
    'robot': 'робот',
    'workspace': 'рабочая зона',
    'camera': 'камера',
    'operator': 'виртуальный оператор',
}

POSE_CONVENTIONS = {
    'frame_id': 'world',
    'length_unit': 'm',
    'orientation': 'quaternion_xyzw',
}

_OBJECT_DEFS = {
    'fixture': 'grant_fixture',
    'body': 'grant_body',
    'tray': 'grant_tray',
    'cap': 'grant_cap',
}


def _default_geometry() -> GeometrySpec:
    """Locate geometry.yaml in an installed or source checkout."""
    candidates: list[Path] = []
    try:
        from ament_index_python.packages import get_package_share_directory

        candidates.append(
            Path(get_package_share_directory('iiwa_experement'))
            / 'config' / 'geometry.yaml'
        )
    except Exception:
        pass
    candidates.append(
        Path(__file__).resolve().parents[2]
        / 'iiwa_experement' / 'config' / 'geometry.yaml'
    )
    for path in candidates:
        if path.is_file():
            return load_geometry(path)
    raise FileNotFoundError('geometry.yaml is unavailable for the scene snapshot')


def _state_mapping(supervisor_state: Mapping[str, Any] | None) -> dict[str, Any]:
    if supervisor_state is None:
        return {}
    if not isinstance(supervisor_state, Mapping):
        raise TypeError('supervisor_state must be a mapping')
    return dict(supervisor_state)


def _transform(entry: Any, child: str) -> Transform | None:
    if not isinstance(entry, Mapping):
        return None
    xyz = entry.get('xyz')
    rotation = entry.get('rotation')
    if not isinstance(xyz, (list, tuple)) or len(xyz) != 3:
        return None
    if not isinstance(rotation, (list, tuple)) or len(rotation) != 3:
        return None
    if any(not isinstance(row, (list, tuple)) or len(row) != 3 for row in rotation):
        return None
    try:
        values = np.asarray(rotation, dtype=float)
        translation = np.asarray(xyz, dtype=float)
    except (TypeError, ValueError):
        return None
    if not np.all(np.isfinite(values)) or not np.all(np.isfinite(translation)):
        return None
    return Transform(
        xyz=tuple(float(value) for value in translation),
        rotation=tuple(tuple(float(value) for value in row) for row in values),
        parent='world',
        child=child,
    )


def _pose_json(
    transform: Transform | None,
    *,
    source: str,
    stamp: float,
) -> dict[str, Any] | None:
    if transform is None:
        return None
    position = np.asarray(transform.xyz, dtype=float)
    matrix = np.asarray(transform.rotation, dtype=float)
    if position.shape != (3,) or matrix.shape != (3, 3):
        raise ValueError('pose must contain a 3-vector and a 3x3 rotation')
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(matrix)):
        raise ValueError('pose contains a non-finite number')
    quaternion = Rotation.from_matrix(matrix).as_quat()
    norm = float(np.linalg.norm(quaternion))
    if not np.isfinite(norm) or norm < 1e-9:
        raise ValueError('pose quaternion is not normalizable')
    quaternion = quaternion / norm
    return {
        'frame_id': 'world',
        'position': {
            'x': float(position[0]),
            'y': float(position[1]),
            'z': float(position[2]),
        },
        'orientation': {
            'x': float(quaternion[0]),
            'y': float(quaternion[1]),
            'z': float(quaternion[2]),
            'w': float(quaternion[3]),
        },
        'source': source,
        'stamp': float(stamp),
    }


def _flags(state: Mapping[str, Any]) -> list[str]:
    flags: list[str] = []
    mounted = state.get('mounted_tool')
    flags.append('tool_docked' if not mounted else f'tool_mounted:{mounted}')
    held = state.get('held_object')
    flags.append('gripper_empty' if not held else f'holding:{held}')
    cap_state = (state.get('cap') or {}).get('state')
    if cap_state:
        flags.append(f'hatch_{cap_state}')
    return flags


def _component_state(
    catalog: SkillCatalog,
    spec: ScenarioSpec,
    state: Mapping[str, Any],
) -> dict[str, Any]:
    components: dict[str, Any] = {}
    for object_id in catalog.component_ids(spec):
        kind = catalog.component_kind(object_id, spec)
        if kind == 'hatch':
            components[object_id] = str(
                (state.get('cap') or {}).get('state', 'unknown')
            )
        elif kind == 'screw':
            components[object_id] = str(
                (state.get('screws') or {}).get(object_id, {}).get(
                    'state', 'unknown',
                )
            )
        else:
            components[object_id] = 'present'
    return components


def _tool_state(
    catalog: SkillCatalog,
    spec: ScenarioSpec,
    state: Mapping[str, Any],
) -> dict[str, str]:
    return {
        name: str(
            (state.get('tools') or {}).get(name, {}).get(
                'state', 'unknown',
            )
        )
        for name in catalog.tool_ids(spec)
    }


def _hole_state(
    spec: ScenarioSpec,
    state: Mapping[str, Any],
) -> dict[str, Any]:
    del spec
    holes = state.get('holes') or {}
    return {
        name: {
            'state': str(entry.get('state', 'unknown')),
            'screw': entry.get('screw'),
        }
        for name, entry in holes.items()
        if isinstance(entry, Mapping)
    }


def _object_pose(
    state: Mapping[str, Any],
    definition: str,
    fallback: Transform,
    *,
    stamp: float,
) -> tuple[Transform, str]:
    live = _transform((state.get('objects') or {}).get(definition), definition)
    if live is not None:
        return live, 'webots_supervisor'
    return fallback, 'scenario_manifest'


def _component_pose(
    spec: ScenarioSpec,
    geometry: GeometrySpec,
    state: Mapping[str, Any],
    object_id: str,
    kind: str,
    stamp: float,
) -> tuple[Transform | None, str]:
    if kind == 'fixture':
        fallback = object_pose_transform(
            spec.fixture.x, spec.fixture.y, spec.fixture.z,
            spec.fixture.yaw_deg, child='fixture',
        )
        return _object_pose(state, 'grant_fixture', fallback, stamp=stamp)
    if kind == 'body':
        fallback = object_pose_transform(
            spec.body.x, spec.body.y, spec.body.z,
            spec.body.yaw_deg, child='body',
        )
        return _object_pose(state, 'grant_body', fallback, stamp=stamp)
    if kind == 'tray':
        assert spec.tray_pose is not None
        fallback = object_pose_transform(
            spec.tray_pose.x, spec.tray_pose.y, spec.tray_pose.z,
            spec.tray_pose.yaw_deg, child='tray',
        )
        return _object_pose(state, 'grant_tray', fallback, stamp=stamp)
    if kind == 'hatch':
        live = _transform((state.get('cap') or {}).get('pose'), 'cap')
        if live is not None:
            return live, 'webots_supervisor'
        return world_target(spec, geometry, 'cap'), 'scenario_manifest'
    if kind == 'screw':
        live = _transform(
            (state.get('screws') or {}).get(object_id, {}).get('pose'),
            object_id,
        )
        if live is not None:
            return live, 'webots_supervisor'
        return world_target(spec, geometry, f'screw:{object_id}'), 'scenario_manifest'
    return None, 'scenario_manifest'


def _tool_pose(
    spec: ScenarioSpec,
    geometry: GeometrySpec,
    state: Mapping[str, Any],
    tool_name: str,
    stamp: float,
) -> tuple[Transform | None, str]:
    live = _transform(
        (state.get('tools') or {}).get(tool_name, {}).get('pose'),
        tool_name,
    )
    if live is not None:
        return live, 'webots_supervisor'
    fallback = world_target(spec, geometry, f'tool:{tool_name}')
    return _object_pose(state, f'grant_{tool_name}', fallback, stamp=stamp)


def _destination_poses(
    spec: ScenarioSpec,
    geometry: GeometrySpec,
    destination_id: str,
    kind: str,
    stamp: float,
) -> dict[str, Any]:
    source = 'executor_geometry'
    if kind == 'workspace':
        work = geometry.safe_poses['work']
        pose = _pose_json(work, source=source, stamp=stamp)
        return {'pose': pose, 'tcp_pose': pose}
    if kind == 'seat':
        target = world_target(spec, geometry, 'cap_seat')
        return {
            'pose': _pose_json(target, source=source, stamp=stamp),
            'tcp_pose': _pose_json(
                tcp_goal(target, geometry.cap_tcp_to_node),
                source=source, stamp=stamp,
            ),
        }
    if kind == 'hole':
        target = world_target(spec, geometry, f'hole:{destination_id}')
        return {
            'pose': _pose_json(target, source=source, stamp=stamp),
            'tcp_pose': _pose_json(
                tcp_goal(target, geometry.screw_tcp_to_node),
                source=source, stamp=stamp,
            ),
        }
    if kind == 'tool_socket':
        tool = destination_id[: -len('_socket')]
        target = world_target(spec, geometry, f'tool:{tool}')
        return {
            'pose': _pose_json(target, source=source, stamp=stamp),
            'tcp_pose': _pose_json(
                tcp_goal(target, geometry.tool_tcp_to_nodes[tool]),
                source=source, stamp=stamp,
            ),
        }
    if kind == 'contour':
        start = sealant_point(geometry, spec, destination_id, 0.0)
        end = sealant_point(geometry, spec, destination_id, 1.0)
        tip = geometry.tool_work_tips['sealant_tool']
        return {
            'start': _pose_json(start, source=source, stamp=stamp),
            'end': _pose_json(end, source=source, stamp=stamp),
            'start_tcp': _pose_json(
                tcp_goal(start, tip), source=source, stamp=stamp,
            ),
            'end_tcp': _pose_json(
                tcp_goal(end, tip), source=source, stamp=stamp,
            ),
        }
    return {}


def build_scene(
    spec: ScenarioSpec,
    supervisor_state: Mapping[str, Any],
    catalog: SkillCatalog,
    geometry: GeometrySpec | None = None,
    *,
    stamp: float | None = None,
) -> dict[str, Any]:
    """Build the live ``scene_json`` object for one snapshot."""
    state = _state_mapping(supervisor_state)
    catalog.validate_ids(spec)
    if geometry is None:
        geometry = _default_geometry()
    if stamp is None:
        try:
            stamp = float(state.get('simulation_time', 0.0))
        except (TypeError, ValueError):
            stamp = 0.0
    stamp = float(stamp)
    mounted = state.get('mounted_tool')
    held = state.get('held_object')

    components = []
    for object_id in catalog.component_ids(spec):
        kind = catalog.component_kind(object_id, spec)
        entry = build_component_entry(catalog, spec, object_id, state)
        pose, source = _component_pose(
            spec, geometry, state, object_id, kind, stamp,
        )
        entry['pose'] = _pose_json(pose, source=source, stamp=stamp)
        components.append(entry)

    destinations = []
    for destination_id in catalog.destination_ids(spec):
        kind = catalog.destination_kind(destination_id, spec)
        entry = destination_entry(catalog, spec, destination_id, state)
        entry.update(_destination_poses(
            spec, geometry, destination_id, kind, stamp,
        ))
        destinations.append(entry)

    tools = []
    for tool in spec.tools:
        entry = tool_entry(catalog, spec, tool, state)
        pose, source = _tool_pose(spec, geometry, state, tool.name, stamp)
        entry['pose'] = _pose_json(pose, source=source, stamp=stamp)
        tools.append(entry)

    initial_state = {
        'tool_mounted': mounted if isinstance(mounted, str) else None,
        'held_object': held if isinstance(held, str) else None,
        'flags': _flags(state),
        'components': _component_state(catalog, spec, state),
        'tools': _tool_state(catalog, spec, state),
        'holes': _hole_state(spec, state),
        'scene_ready': bool(state.get('ready', True)),
        'stamp': stamp,
    }

    return {
        'schema_version': SCENE_SCHEMA_VERSION,
        'contract_version': CONTRACT_VERSION,
        'catalog_version': CATALOG_VERSION,
        'profile': catalog.profile,
        'experiment_id': int(spec.experiment_id),
        'domain': 'grant_assembly',
        'pose_conventions': dict(POSE_CONVENTIONS),
        'components': components,
        'destinations': destinations,
        'tools': tools,
        'initial_state': initial_state,
        'cameras': DEFAULT_CAMERAS,
        'available_skills': list(catalog.skill_names(spec)),
        'skill_contracts': catalog.skill_contracts(spec),
        'action_rules': catalog.action_rules(spec),
        'service_objects': dict(SERVICE_OBJECTS),
        'recovery_attempts_per_component': 1,
    }


def build_envelope(
    scene: Mapping[str, Any],
    *,
    state_version: int,
    stamp: float,
    experiment_id: int | None = None,
) -> dict[str, Any]:
    """Wrap ``scene_json`` in the transport envelope used by every interface."""
    if state_version < 0:
        raise ValueError('state_version must be non-negative')
    envelope = {
        'schema_version': SCENE_SCHEMA_VERSION,
        'contract_version': CONTRACT_VERSION,
        'catalog_version': CATALOG_VERSION,
        'experiment_id': (
            int(experiment_id)
            if experiment_id is not None
            else int(scene.get('experiment_id', 0))
        ),
        'state_version': int(state_version),
        'timestamp': float(stamp),
        'scene_json': dict(scene),
    }
    return envelope
