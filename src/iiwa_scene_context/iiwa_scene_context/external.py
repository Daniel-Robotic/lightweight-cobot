"""Adapt the live scene-v2 contract to the external graph-v3 context bundle.

The trained planner sees a public scene/catalog pair derived here, never the
training workspace.  Only ids and semantics that exist in the live scene are
advertised; unsupported concepts are omitted and later rejected by the
executor instead of being silently rewritten.
"""
from __future__ import annotations

import copy
from typing import Any, Mapping

from .catalog import (
    CATALOG_VERSION,
    CONTRACT_VERSION,
    NATIVE_SKILLS,
    SKILL_DESCRIPTIONS,
    SkillCatalog,
)
from .graph_contract import (
    GRAPH_CONTRACT_VERSION,
    GRAPH_SCHEMA_VERSION,
    NORMALIZED_EVENTS,
    response_schema,
)


EXTERNAL_SCENE_SCHEMA_VERSION = 1
EXTERNAL_CATALOG_SCHEMA_VERSION = 1
SUPPORTED_GOAL_PREDICATES = (
    'installed',
    'handoff',
    'tool_docked',
    'held',
    'tool_mounted',
    'at',
    'visible',
)
SUPPORTED_OBSERVATIONS = (
    'installed',
    'handoff',
    'tool_docked',
    'held',
    'tool_mounted',
    'at',
    'visible',
    'hole_empty',
)
UNSUPPORTED_GOAL_PREDICATES = (
    'placed',
    'dispensed',
    'inspected',
    'defective',
)
EXTERNAL_UNSUPPORTED_SKILLS = ('inspect_state',)
# Normalized event -> raw executor outcome.  Only exact, decidable mappings
# are advertised.  ``already_held`` / ``already_placed`` need an extra
# inspection node before they can be proven, so they are not offered.
SUPPORTED_EVENTS: dict[str, dict[str, str]] = {
    'detect': {
        'object_unavailable': 'not_found',
        'target_unavailable': 'not_found',
    },
    'grab': {'object_unavailable': 'grasp_failed'},
    'pick': {'object_unavailable': 'grasp_failed'},
    'transfer': {'target_unavailable': 'failed'},
    'move_to_storage': {'target_unavailable': 'failed'},
    'release': {'target_unavailable': 'failed'},
}

_COMPONENT_TYPES = {
    'fixture': 'fixture',
    'body': 'component',
    'hatch': 'part',
    'cap': 'part',
    'tray': 'component',
    'screw': 'part',
}
_LOCATION_TYPES = {
    'workspace': 'workspace',
    'seat': 'seat',
    'hole': 'hole',
    'tool_socket': 'tool_socket',
    'contour': 'contour',
}
_TOOL_TYPES = {
    'suction_cup': 'suction',
    'magnet': 'magnet',
    'dispenser': 'dispenser',
    'torque_wrench': 'gripper',
}


class ContextError(ValueError):
    """The live snapshot cannot be offered as planner context."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


def supported_contract(envelope: Mapping[str, Any]) -> bool:
    scene = envelope.get('scene_json')
    if not isinstance(scene, Mapping):
        return False
    try:
        return (
            int(envelope.get('schema_version')) == 2
            and int(envelope.get('contract_version')) == CONTRACT_VERSION
            and int(envelope.get('catalog_version')) == CATALOG_VERSION
            and int(scene.get('schema_version')) == 2
            and int(scene.get('contract_version')) == CONTRACT_VERSION
            and int(scene.get('catalog_version')) == CATALOG_VERSION
        )
    except (TypeError, ValueError):
        return False


def _component_type(kind: str) -> str:
    return _COMPONENT_TYPES.get(kind, 'component')


def _location_type(kind: str) -> str:
    return _LOCATION_TYPES.get(kind, 'workspace')


def _tool_type(kind: str) -> str:
    return _TOOL_TYPES.get(kind, 'gripper')


def _public_pose(pose: Any) -> dict[str, Any] | None:
    """Keep only the dataset scene-v1 pose fields."""
    if not isinstance(pose, Mapping):
        return None
    frame_id = pose.get('frame_id')
    position = pose.get('position')
    orientation = pose.get('orientation')
    if not isinstance(position, Mapping) or not isinstance(orientation, Mapping):
        return None
    return {
        'frame_id': str(frame_id or 'world'),
        'position': dict(position),
        'orientation': dict(orientation),
    }


def _scene_objects(scene: Mapping[str, Any]) -> list[dict[str, Any]]:
    objects = []
    for entry in scene.get('components', []):
        if not isinstance(entry, Mapping) or not isinstance(entry.get('id'), str):
            continue
        attributes = entry.get('attributes') or {}
        kind = str(attributes.get('kind', 'component'))
        objects.append({
            'id': entry['id'],
            'type': _component_type(kind),
            'kind': kind,
            'name': str(entry.get('name', entry['id'])),
            'movable': bool(attributes.get('movable', False)),
            'pose': _public_pose(entry.get('pose')),
            'state': entry.get('state'),
        })
    return objects


def _scene_locations(scene: Mapping[str, Any]) -> list[dict[str, Any]]:
    locations = []
    for entry in scene.get('destinations', []):
        if not isinstance(entry, Mapping) or not isinstance(entry.get('id'), str):
            continue
        kind = str(entry.get('kind', 'destination'))
        pose = entry.get('pose')
        if not isinstance(pose, Mapping):
            pose = entry.get('start')
        locations.append({
            'id': entry['id'],
            'type': _location_type(kind),
            'kind': kind,
            'name': str(entry.get('name', entry['id'])),
            'capacity': int(entry.get('capacity', 1)),
            'state': str(entry.get('state', 'available')),
            'pose': _public_pose(pose),
            'screw': entry.get('screw'),
            'tool': entry.get('tool'),
        })
    return locations


def _tool_compatibility(
    scene: Mapping[str, Any],
    tool: Mapping[str, Any],
) -> tuple[list[str], list[str]]:
    kind = str(tool.get('kind', ''))
    components = [
        entry for entry in scene.get('components', [])
        if isinstance(entry, Mapping)
    ]
    destinations = [
        entry for entry in scene.get('destinations', [])
        if isinstance(entry, Mapping)
    ]

    def component_kind(entry: Mapping[str, Any]) -> str:
        return str((entry.get('attributes') or {}).get('kind', ''))

    def destination_kind(entry: Mapping[str, Any]) -> str:
        return str(entry.get('kind', ''))

    objects: list[str] = []
    locations: list[str] = []
    if kind == 'suction_cup':
        objects = [e['id'] for e in components if component_kind(e) == 'hatch']
        locations = [
            e['id'] for e in destinations
            if destination_kind(e) in ('workspace', 'seat')
        ]
    elif kind == 'magnet':
        objects = [e['id'] for e in components if component_kind(e) == 'screw']
        locations = [
            e['id'] for e in destinations
            if destination_kind(e) in ('workspace', 'hole')
        ]
    elif kind == 'dispenser':
        locations = [
            e['id'] for e in destinations
            if destination_kind(e) == 'contour'
        ]
    elif kind == 'torque_wrench':
        # The executor has no tighten skill for this tool.
        objects = []
        locations = []
    else:
        objects = [e['id'] for e in components if component_kind(e) != 'fixture']
        locations = [
            e['id'] for e in destinations
            if destination_kind(e) in ('workspace', 'seat', 'hole')
        ]
    return objects, locations


def _scene_tools(scene: Mapping[str, Any]) -> list[dict[str, Any]]:
    tools = []
    for entry in scene.get('tools', []):
        if not isinstance(entry, Mapping) or not isinstance(entry.get('id'), str):
            continue
        compatible_objects, compatible_locations = _tool_compatibility(scene, entry)
        tools.append({
            'id': entry['id'],
            'type': _tool_type(str(entry.get('kind', ''))),
            'kind': str(entry.get('kind', '')),
            'name': str(entry.get('name', entry['id'])),
            'socket': str(entry.get('socket', f"{entry['id']}_socket")),
            'state': str(entry.get('state', 'unknown')),
            'compatible_objects': compatible_objects,
            'compatible_locations': compatible_locations,
            'pose': _public_pose(entry.get('pose')),
        })
    return tools


def _initial_facts(
    scene: Mapping[str, Any],
    objects: list[dict[str, Any]],
    locations: list[dict[str, Any]],
    tools: list[dict[str, Any]],
) -> list[dict[str, Any]]:
    initial = scene.get('initial_state') or {}
    held = initial.get('held_object')
    mounted = initial.get('tool_mounted')
    component_states = initial.get('components') or {}
    facts: list[dict[str, Any]] = []

    def add(predicate: str, **args: str) -> None:
        facts.append({'predicate': predicate, 'args': args})

    for entry in objects:
        object_id = entry['id']
        state = str(entry.get('state') or component_states.get(object_id, ''))
        add('visible', object=object_id)
        if entry.get('movable') and object_id != held and state not in (
            'installed', 'delivered',
        ):
            add('available', object=object_id)
    for entry in tools:
        tool_id = entry['id']
        if entry.get('state') == 'docked':
            add('available', object=tool_id)
    if mounted:
        add('mounted', tool=str(mounted))
    if not held:
        add('gripper_empty')
    if held:
        add('held', object=str(held))
    for entry in locations:
        location_id = entry['id']
        if entry.get('kind') == 'hole' and entry.get('state') == 'empty':
            add('hole_empty', hole=location_id)
        if (
            entry.get('kind') == 'seat'
            and entry.get('state') == 'occupied'
            and 'cap' in {item['id'] for item in objects}
        ):
            add('installed', part='cap', destination=location_id)
            add('at', object='cap', destination=location_id)
    return facts


def build_external_scene(scene: Mapping[str, Any]) -> dict[str, Any]:
    """Convert the live scene-v2 object into the graph-v3 public scene-v1."""
    if int(scene.get('schema_version', 0)) != 2:
        raise ContextError(
            'unsupported_contract',
            'live scene is not schema_version 2',
        )
    objects = _scene_objects(scene)
    locations = _scene_locations(scene)
    tools = _scene_tools(scene)
    return {
        'schema_version': EXTERNAL_SCENE_SCHEMA_VERSION,
        'frame_id': 'world',
        'experiment_id': scene.get('experiment_id'),
        'objects': objects,
        'locations': locations,
        'tools': tools,
        'initial_facts': _initial_facts(scene, objects, locations, tools),
        'available_skills': [
            str(name) for name in scene.get('available_skills', [])
            if name not in EXTERNAL_UNSUPPORTED_SKILLS
        ],
    }


def _skill_parameters(name: str) -> dict[str, str]:
    if name in ('detect', 'capture', 'inspect_state', 'stop', 'notify_operator',
                'wait_scene_clear', 'verify', 'inspect'):
        return {'object': 'object'}
    if name in ('dispense',):
        return {'object': 'object', 'destination': 'location'}
    return {'object': 'object', 'destination': 'location', 'tool': 'tool'}


def _catalog_skill(
    name: str,
    scene: Mapping[str, Any],
) -> dict[str, Any]:
    rules = (scene.get('action_rules') or {}).get(name, {})
    return {
        'description': SKILL_DESCRIPTIONS.get(name, ''),
        'parameters': _skill_parameters(name),
        'preconditions': list(rules.get('preconditions', [])),
        'events': sorted(SUPPORTED_EVENTS.get(name, {})),
        'target_types': list(rules.get('target_types', [])),
        'object_types': list(rules.get('object_types', [])),
        'result': rules.get('result', ''),
    }


def build_external_catalog(scene: Mapping[str, Any]) -> dict[str, Any]:
    """Adapt the server skill catalog to the graph-v3 public catalog."""
    available = [
        str(name) for name in scene.get('available_skills', [])
        if name in SKILL_DESCRIPTIONS
    ]
    object_types = sorted({
        _component_type(str((entry.get('attributes') or {}).get('kind', 'component')))
        for entry in scene.get('components', [])
        if isinstance(entry, Mapping)
    } | {'part', 'component', 'fixture'})
    location_types = sorted({
        _location_type(str(entry.get('kind', 'destination')))
        for entry in scene.get('destinations', [])
        if isinstance(entry, Mapping)
    } | {'workspace', 'seat', 'hole', 'tool_socket', 'contour'})
    tool_types = sorted({
        _tool_type(str(entry.get('kind', '')))
        for entry in scene.get('tools', [])
        if isinstance(entry, Mapping)
    } | {'suction', 'magnet', 'dispenser'})
    predicates = {
        'installed': {
            'args': {'part': 'object', 'destination': 'location'},
            'public': True,
            'dynamic': False,
            'description': 'Part is fixed in a seat; confirmed by inspect_state.',
        },
        'handoff': {
            'args': {'screw': 'object', 'hole': 'location'},
            'public': True,
            'dynamic': False,
            'description': 'Screw was handed to the virtual operator at a hole.',
        },
        'tool_docked': {
            'args': {'tool': 'tool', 'socket': 'location'},
            'public': True,
            'dynamic': False,
            'description': 'Tool is back in its own socket.',
        },
        'held': {
            'args': {'object': 'object'},
            'public': True,
            'dynamic': False,
            'description': 'Object is held by the gripper.',
        },
        'tool_mounted': {
            'args': {'tool': 'tool'},
            'public': True,
            'dynamic': False,
            'description': 'Tool is mounted in the gripper.',
        },
        'at': {
            'args': {'object': 'object', 'destination': 'location'},
            'public': True,
            'dynamic': False,
            'description': 'Object is installed or handed to a destination.',
        },
        'visible': {
            'args': {'object': 'object'},
            'public': True,
            'dynamic': False,
            'description': 'Object is present in the live scene.',
        },
        'hole_empty': {
            'args': {'hole': 'location'},
            'public': True,
            'dynamic': False,
            'description': 'Hole has no delivered screw.',
        },
    }
    observations = {
        name: {
            'args': dict(predicates[name]['args']),
            'outcomes': 'deterministic',
            'requires': [],
            'description': predicates[name]['description'],
        }
        for name in SUPPORTED_OBSERVATIONS
    }
    goal_skills = {
        'installed': ['release', 'verify', 'inspect'],
        'handoff': ['handoff_screw'],
        'tool_docked': ['dock_tool'],
        'held': ['grab', 'pick'],
        'tool_mounted': ['grab', 'pick'],
        'at': ['release', 'handoff_screw'],
        'visible': ['detect'],
    }
    return {
        'schema_version': EXTERNAL_CATALOG_SCHEMA_VERSION,
        'name': 'grant-runtime',
        'catalog_version': CATALOG_VERSION,
        'object_types': object_types,
        'location_types': location_types,
        'tool_types': tool_types,
        'predicates': predicates,
        'parameters': {
            'object': {'type': 'object'},
            'destination': {'type': 'location'},
            'tool': {'type': 'tool'},
            'count': {'type': 'value'},
            'mapping': {'type': 'value'},
        },
        'skills': {
            name: _catalog_skill(name, scene)
            for name in available
            if name in NATIVE_SKILLS or name in ('pick', 'move_to_storage', 'inspect')
            if name != 'inspect_state'
        },
        'observations': observations,
        'goal_skills': goal_skills,
        'placement': {},
        'tool_compatibility': {},
        'blocked_reasons': {
            'unsupported_skill': 'The live executor does not implement this skill.',
            'unsupported_goal': 'The live executor cannot confirm this goal predicate.',
            'unsupported_predicate': 'The live executor cannot evaluate this observation.',
            'unknown_id': 'The id is absent from the current scene.',
        },
    }


def render_system_prompt(scene: Mapping[str, Any], catalog: Mapping[str, Any]) -> str:
    """Deterministic graph-v3 instruction for the external model."""
    skills = ', '.join(sorted(catalog.get('skills', {})))
    goals = ', '.join(SUPPORTED_GOAL_PREDICATES)
    observations = ', '.join(SUPPORTED_OBSERVATIONS)
    events = ', '.join(NORMALIZED_EVENTS)
    return (
        'Ты — планировщик робота KUKA iiwa 7. Верни РОВНО один JSON-объект '
        'graph-v3 и ничего больше.\n'
        'Поле schema_version всегда 3. Допустимые status: ready, '
        'clarification_required, blocked.\n'
        'ready: entry указывает на узел плана, goals и plan непустые.\n'
        'clarification_required: entry="", plan=[], добавь question и missing '
        '(список машинных имён параметров).\n'
        'blocked: entry="", plan=[], добавь reason. Не применяй blocked к '
        'неизвестному будущему результату обнаружения или захвата.\n\n'
        'Узел skill: {id, kind:"skill", skill, object, target, tool?, '
        'params?, next, on?, max_retries?}.\n'
        'Узел observe: {id, kind:"observe", predicate:{name,args}, '
        'on:{true,false,unknown}, max_retries?}.\n'
        'Переходы указывают на id узла либо на терминал completed, '
        'replan_required, operator_required.\n'
        'completed допустим только после подтверждения всех goals. '
        'operator_required сообщает оператору подтверждённые и оставшиеся '
        'цели. replan_required — для новой попытки планирования.\n'
        f'Поле on у skill допускает только события: {events}. Для каждого '
        'навыка допустимый набор указан в catalog.skills[].events; остальные '
        'события отклоняются до движения.\n'
        'У observe обязательны все три перехода true/false/unknown. Ветка '
        'unknown не разрешает движение и должна вести к operator_required '
        'или replan_required.\n\n'
        f'goals — проверяемые предикаты, не свободный текст. Доступны: {goals}. '
        'Неподдерживаемый предикат отклоняет весь граф до движения.\n'
        f'Доступные наблюдения: {observations}.\n\n'
        f'Навыки этой сцены: {skills}. Используй только ID и навыки из '
        'scene/objects, scene/locations, scene/tools и scene/available_skills. '
        'Не придумывай объекты, позы, координаты, углы или команды суставам.\n'
        'Люк берётся присоской vacuum. Винт берётся magnetic_tool. Герметик '
        'наносится только sealant_tool по существующему участку side_i. '
        'handoff_screw передаёт винт оператору у отверстия; вставки и затяжки '
        'нет. torque_tool не используется.\n'
        'При неизвестном состоянии захвата план обязан остановиться, проверить '
        'состояние и завершиться operator_required. Повторы ограничены полем '
        'max_retries; бесконечные циклы запрещены.\n'
        'Не добавляй пояснений, markdown, комментариев или нескольких '
        'JSON-объектов.\n'
    )


def build_planner_context(
    envelope: Mapping[str, Any],
    *,
    schema: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Build the single consistent model-input package from one snapshot."""
    if not supported_contract(envelope):
        raise ContextError(
            'unsupported_contract',
            'live scene or catalog versions are not supported by the '
            'graph-v3 executor',
        )
    scene_json = envelope.get('scene_json')
    if not isinstance(scene_json, Mapping):
        raise ContextError('scene_not_ready', 'snapshot has no scene_json')
    if not scene_json.get('components'):
        raise ContextError('scene_not_ready', 'snapshot contains no objects')
    scene = build_external_scene(scene_json)
    catalog = build_external_catalog(scene_json)
    if not catalog['skills']:
        raise ContextError(
            'unsupported_contract',
            'the live catalog advertises no executable skill',
        )
    return {
        'schema_version': GRAPH_SCHEMA_VERSION,
        'contract_version': GRAPH_CONTRACT_VERSION,
        'catalog_version': CATALOG_VERSION,
        'state_version': int(envelope.get('state_version', 0)),
        'scene_schema_version': EXTERNAL_SCENE_SCHEMA_VERSION,
        'external_catalog_version': EXTERNAL_CATALOG_SCHEMA_VERSION,
        'experiment_id': envelope.get('experiment_id'),
        'timestamp': envelope.get('timestamp'),
        'system_prompt': render_system_prompt(scene, catalog),
        'scene': scene,
        'catalog': catalog,
        'response_schema': copy.deepcopy(
            dict(schema) if schema is not None else response_schema(),
        ),
    }
