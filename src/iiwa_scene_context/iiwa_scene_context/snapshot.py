"""Pure snapshot builders for the external planner contract.

Coordinates never leave this module's inputs: the model receives logical ids,
logical ownership and camera names only.  Exact poses are resolved at skill
execution time from the live supervisor.
"""
from __future__ import annotations

from typing import Any, Mapping

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
        'topic': '/d455_top/image_raw',
        'role': 'overview',
        'required_for': ['detect'],
    },
    'hand': {
        'topic': '/camera/color',
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


def _state_mapping(supervisor_state: Mapping[str, Any] | None) -> dict[str, Any]:
    if supervisor_state is None:
        return {}
    if not isinstance(supervisor_state, Mapping):
        raise TypeError('supervisor_state must be a mapping')
    return dict(supervisor_state)


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


def build_scene(
    spec: ScenarioSpec,
    supervisor_state: Mapping[str, Any],
    catalog: SkillCatalog,
) -> dict[str, Any]:
    """Build the coordinate-free ``scene_json`` object for one snapshot."""
    state = _state_mapping(supervisor_state)
    catalog.validate_ids(spec)
    mounted = state.get('mounted_tool')
    held = state.get('held_object')

    components = [
        build_component_entry(catalog, spec, object_id, state)
        for object_id in catalog.component_ids(spec)
    ]
    destinations = [
        destination_entry(catalog, spec, destination_id, state)
        for destination_id in catalog.destination_ids(spec)
    ]
    tools = [
        tool_entry(catalog, spec, tool, state)
        for tool in spec.tools
    ]

    initial_state = {
        'tool_mounted': mounted if isinstance(mounted, str) else None,
        'held_object': held if isinstance(held, str) else None,
        'flags': _flags(state),
        'components': _component_state(catalog, spec, state),
        'tools': _tool_state(catalog, spec, state),
        'holes': _hole_state(spec, state),
        'scene_ready': bool(state.get('ready', True)),
    }

    return {
        'schema_version': SCENE_SCHEMA_VERSION,
        'contract_version': CONTRACT_VERSION,
        'catalog_version': CATALOG_VERSION,
        'profile': catalog.profile,
        'experiment_id': int(spec.experiment_id),
        'domain': 'grant_assembly',
        'components': components,
        'destinations': destinations,
        'tools': tools,
        'initial_state': initial_state,
        'cameras': DEFAULT_CAMERAS,
        'available_skills': list(catalog.skill_names()),
        'skill_contracts': catalog.skill_contracts(),
        'service_objects': dict(SERVICE_OBJECTS),
        'recovery_attempts_per_component': 1,
    }


def build_envelope(
    scene: Mapping[str, Any],
    *,
    scene_revision: int,
    state_version: int,
    stamp: float,
    experiment_id: int | None = None,
) -> dict[str, Any]:
    """Wrap ``scene_json`` in the transport envelope used by every interface."""
    if scene_revision < 0:
        raise ValueError('scene_revision must be non-negative')
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
        'scene_revision': int(scene_revision),
        'timestamp': float(stamp),
        'scene_json': dict(scene),
    }
    return envelope
