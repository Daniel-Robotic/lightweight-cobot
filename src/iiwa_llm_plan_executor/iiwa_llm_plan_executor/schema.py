"""Strict graph-v3 parsing and conversion to the executable node model.

Graph-v3 is the only accepted external format.  The parser keeps the original
response for canonical hashing, checks the wire schema, references, reachability,
bounded retry cycles and converts supported skill/observe nodes into the
internal ``PlanGraph`` used by the engine.
"""
from __future__ import annotations

import copy
import json
from dataclasses import dataclass, field
from typing import Any, Mapping

from iiwa_scene_context.catalog import SKILL_OUTCOMES
from iiwa_scene_context.external import SUPPORTED_EVENTS
from iiwa_scene_context.graph_contract import (
    GRAPH_SCHEMA_VERSION,
    GRAPH_TERMINALS,
    ID_PATTERN,
    MAX_NODES,
    MAX_RESPONSE_BYTES,
    NORMALIZED_EVENTS,
)


ALLOWED_STATUSES = ('ready', 'blocked', 'clarification_required')
TOP_LEVEL_KEYS = {
    'ready': {'schema_version', 'status', 'entry', 'goals', 'plan'},
    'clarification_required': {
        'schema_version', 'status', 'entry', 'goals', 'plan', 'question',
        'missing',
    },
    'blocked': {
        'schema_version', 'status', 'entry', 'goals', 'plan', 'reason',
        'reason_code', 'entities', 'evidence',
    },
}
SKILL_NODE_KEYS = {
    'id', 'kind', 'skill', 'object', 'target', 'tool', 'params', 'next',
    'on', 'max_retries',
}
OBSERVE_NODE_KEYS = {'id', 'kind', 'predicate', 'on', 'max_retries'}
PROGRESS_EVENTS = {
    'success', 'ok', 'found', 'clear', 'empty', 'holding', 'placed',
}
SAFE_FAILURE_TERMINAL = {
    'collision': 'operator_required',
    'human_in_scene': 'operator_required',
    'grasp_failed': 'operator_required',
    'not_ok': 'operator_required',
    'timeout': 'operator_required',
    'unknown': 'operator_required',
    'empty': 'operator_required',
    'holding': 'operator_required',
    'placed': 'operator_required',
    'not_found': 'replan_required',
    'failed': 'replan_required',
}
# A graph-v3 skill node cannot express the four inspect_state outcomes; the
# model must use an observe node instead.  Other skills have one nominal
# success event.
UNSUPPORTED_SKILL_NODES = {'inspect_state'}


class PlanSchemaError(ValueError):
    """The response is not a well-formed graph-v3 object."""

    def __init__(
        self,
        message: str,
        *,
        code: str = 'invalid',
        details: dict[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.details = dict(details or {})


@dataclass(frozen=True)
class Transition:
    event: str
    next: str

    def as_dict(self) -> dict[str, str]:
        return {'event': self.event, 'next': self.next}


@dataclass(frozen=True)
class Goal:
    id: str
    predicate: str
    args: dict[str, str] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            'id': self.id,
            'predicate': self.predicate,
            'args': dict(self.args),
        }

    def canonical(self) -> str:
        args = ','.join(f'{key}={self.args[key]}' for key in sorted(self.args))
        return f'{self.predicate}({args})'


@dataclass(frozen=True)
class PlanNode:
    id: str
    skill: str
    object: str = ''
    target: str = ''
    params: dict[str, Any] = field(default_factory=dict)
    transitions: tuple[Transition, ...] = ()
    kind: str = 'skill'
    tool: str = ''
    max_retries: int | None = None

    def transition(self, event: str) -> str | None:
        for item in self.transitions:
            if item.event == event:
                return item.next
        return None

    def as_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            'id': self.id,
            'kind': self.kind,
            'transitions': [entry.as_dict() for entry in self.transitions],
        }
        if self.kind == 'observe':
            payload['predicate'] = dict(self.params.get('predicate') or {})
        else:
            payload.update({
                'skill': self.skill,
                'object': self.object,
                'target': self.target,
            })
            if self.tool:
                payload['tool'] = self.tool
            if self.params:
                payload['params'] = dict(self.params)
        if self.max_retries is not None:
            payload['max_retries'] = self.max_retries
        return payload


@dataclass(frozen=True)
class PlanGraph:
    status: str
    entry: str
    nodes: tuple[PlanNode, ...]
    goals: tuple[Goal, ...] = ()
    raw: dict[str, Any] = field(default_factory=dict)
    status_details: dict[str, Any] = field(default_factory=dict)

    def node(self, node_id: str) -> PlanNode:
        for item in self.nodes:
            if item.id == node_id:
                return item
        raise KeyError(node_id)

    @property
    def ids(self) -> tuple[str, ...]:
        return tuple(item.id for item in self.nodes)

    def as_dict(self) -> dict[str, Any]:
        return copy.deepcopy(self.raw)


def _string(value: Any, label: str, *, allow_empty: bool = True) -> str:
    if not isinstance(value, str):
        raise PlanSchemaError(f'{label} must be a string')
    text = value.strip()
    if not allow_empty and not text:
        raise PlanSchemaError(f'{label} must not be empty')
    return text


def _id(value: Any, label: str) -> str:
    text = _string(value, label, allow_empty=False)
    if not ID_PATTERN.match(text):
        raise PlanSchemaError(f'{label} {text!r} is not a valid graph-v3 id')
    return text


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise PlanSchemaError(f'{label} must be an object')
    return value


def _string_map(value: Any, label: str) -> dict[str, str]:
    raw = _mapping(value, label)
    result: dict[str, str] = {}
    for key, item in raw.items():
        if not isinstance(key, str) or not key:
            raise PlanSchemaError(f'{label} keys must be non-empty strings')
        if not isinstance(item, str):
            raise PlanSchemaError(f'{label}.{key} must be a string')
        result[key] = item
    return result


def _goal(raw: Any, index: int) -> Goal:
    label = f'goals[{index}]'
    entry = _mapping(raw, label)
    extra = set(entry) - {'id', 'predicate', 'args'}
    if extra:
        raise PlanSchemaError(f'{label} has unknown keys {sorted(extra)}')
    args_raw = entry.get('args', {})
    return Goal(
        id=_id(entry.get('id'), f'{label}.id'),
        predicate=_string(
            entry.get('predicate'), f'{label}.predicate', allow_empty=False,
        ),
        args=_string_map(args_raw, f'{label}.args'),
    )


def _predicate(raw: Any, label: str) -> tuple[str, dict[str, str]]:
    entry = _mapping(raw, label)
    extra = set(entry) - {'name', 'args'}
    if extra:
        raise PlanSchemaError(f'{label} has unknown keys {sorted(extra)}')
    return (
        _string(entry.get('name'), f'{label}.name', allow_empty=False),
        _string_map(entry.get('args', {}), f'{label}.args'),
    )


def _max_retries(raw: Any, label: str) -> int | None:
    if raw is None:
        return None
    if isinstance(raw, bool) or not isinstance(raw, int):
        raise PlanSchemaError(f'{label} must be an integer or null')
    if not 0 <= raw <= 20:
        raise PlanSchemaError(f'{label} must be within 0..20')
    return raw


def _nominal_success(skill: str) -> str:
    return {
        'detect': 'found',
        'verify': 'ok',
        'inspect': 'ok',
        'wait_scene_clear': 'clear',
    }.get(skill, 'success')


def _skill_transitions(
    skill: str,
    node_id: str,
    next_id: str,
    on: dict[str, str],
) -> tuple[Transition, ...]:
    if skill in UNSUPPORTED_SKILL_NODES:
        raise PlanSchemaError(
            f'node {node_id!r}: skill {skill!r} must be expressed as an '
            'observe node',
            code='unsupported_contract',
            details={'node_id': node_id, 'skill': skill},
        )
    supported = SUPPORTED_EVENTS.get(skill, {})
    mapped: dict[str, str] = {}
    for event, target in on.items():
        if event not in NORMALIZED_EVENTS:
            raise PlanSchemaError(
                f'node {node_id!r} has unknown normalized event {event!r}'
            )
        raw_event = supported.get(event)
        if raw_event is None:
            raise PlanSchemaError(
                f'node {node_id!r}: event {event!r} has no exact runtime '
                f'mapping for skill {skill!r}',
                code='unsupported_contract',
                details={'node_id': node_id, 'skill': skill, 'event': event},
            )
        existing = mapped.get(raw_event)
        if existing is not None and existing != target:
            raise PlanSchemaError(
                f'node {node_id!r}: normalized events resolve to the same '
                f'runtime outcome {raw_event!r} but different targets'
            )
        mapped[raw_event] = target
    outcomes = SKILL_OUTCOMES.get(skill)
    if not outcomes:
        raise PlanSchemaError(
            f'node {node_id!r} uses unknown or unavailable skill {skill!r}'
        )
    nominal = _nominal_success(skill)
    transitions: dict[str, str] = {}
    for event in outcomes:
        if event == nominal or event in PROGRESS_EVENTS:
            target = next_id
        elif event in mapped:
            target = mapped[event]
        else:
            target = SAFE_FAILURE_TERMINAL.get(event, 'operator_required')
        transitions[event] = target
    for raw_event, target in mapped.items():
        if raw_event in transitions and transitions[raw_event] != target:
            raise PlanSchemaError(
                f'node {node_id!r}: event {raw_event!r} is already mapped to '
                f'{transitions[raw_event]!r}'
            )
        transitions[raw_event] = target
    return tuple(
        Transition(event, target)
        for event, target in sorted(transitions.items())
    )


def _skill_node(raw: Mapping[str, Any], index: int) -> PlanNode:
    label = f'plan[{index}]'
    extra = set(raw) - SKILL_NODE_KEYS
    if extra:
        raise PlanSchemaError(f'{label} has unknown keys {sorted(extra)}')
    node_id = _id(raw.get('id'), f'{label}.id')
    skill = _string(raw.get('skill'), f'{label}.skill', allow_empty=False)
    object_id = _string(raw.get('object', ''), f'{label}.object')
    target = _string(raw.get('target', ''), f'{label}.target')
    tool_raw = raw.get('tool')
    tool = '' if tool_raw is None else _string(tool_raw, f'{label}.tool')
    next_id = _string(raw.get('next'), f'{label}.next', allow_empty=False)
    on = _string_map(raw.get('on', {}), f'{label}.on')
    params_raw = _mapping(raw.get('params', {}), f'{label}.params')
    params: dict[str, Any] = {}
    for key, value in params_raw.items():
        if not isinstance(key, str) or not key:
            raise PlanSchemaError(f'{label}.params keys must be non-empty strings')
        if isinstance(value, bool) or not isinstance(value, (int, float, str)):
            raise PlanSchemaError(
                f'{label}.params.{key} must be a number or string'
            )
        params[key] = value
    transitions = _skill_transitions(skill, node_id, next_id, on)
    return PlanNode(
        id=node_id,
        skill=skill,
        object=object_id,
        target=target,
        params=params,
        transitions=transitions,
        kind='skill',
        tool=tool,
        max_retries=_max_retries(raw.get('max_retries'), f'{label}.max_retries'),
    )


def _observe_node(raw: Mapping[str, Any], index: int) -> PlanNode:
    label = f'plan[{index}]'
    extra = set(raw) - OBSERVE_NODE_KEYS
    if extra:
        raise PlanSchemaError(f'{label} has unknown keys {sorted(extra)}')
    node_id = _id(raw.get('id'), f'{label}.id')
    name, args = _predicate(raw.get('predicate'), f'{label}.predicate')
    on = _mapping(raw.get('on'), f'{label}.on')
    extra_on = set(on) - {'true', 'false', 'unknown'}
    if extra_on:
        raise PlanSchemaError(f'{label}.on has unknown keys {sorted(extra_on)}')
    transitions = []
    for outcome in ('true', 'false', 'unknown'):
        if outcome not in on:
            raise PlanSchemaError(f'{label}.on must contain {outcome!r}')
        transitions.append(Transition(
            outcome,
            _string(on[outcome], f'{label}.on.{outcome}', allow_empty=False),
        ))
    return PlanNode(
        id=node_id,
        skill='observe',
        params={'predicate': {'name': name, 'args': args}},
        transitions=tuple(transitions),
        kind='observe',
        max_retries=_max_retries(raw.get('max_retries'), f'{label}.max_retries'),
    )


def _check_bounded_cycles(nodes: Mapping[str, PlanNode]) -> None:
    color: dict[str, int] = {}
    stack: list[str] = []

    def visit(node_id: str) -> None:
        color[node_id] = 1
        stack.append(node_id)
        for transition in nodes[node_id].transitions:
            target = transition.next
            if target not in nodes:
                continue
            if color.get(target, 0) == 1:
                cycle = stack[stack.index(target):]
                if not any((nodes[item].max_retries or 0) > 0 for item in cycle):
                    raise PlanSchemaError(
                        f'cycle {cycle + [target]} has no max_retries bound'
                    )
            elif color.get(target, 0) == 0:
                visit(target)
        stack.pop()
        color[node_id] = 2

    for node_id in nodes:
        if color.get(node_id, 0) == 0:
            visit(node_id)


def _check_references(graph: PlanGraph) -> None:
    ids = set(graph.ids)
    allowed = ids | set(GRAPH_TERMINALS)
    if len(graph.ids) != len(ids):
        raise PlanSchemaError('duplicate node ids')
    for node in graph.nodes:
        if node.id in GRAPH_TERMINALS:
            raise PlanSchemaError(f'node id {node.id!r} is a reserved terminal')
    if graph.entry not in ids:
        raise PlanSchemaError(
            f'entry {graph.entry!r} does not reference a plan node'
        )
    for node in graph.nodes:
        for transition in node.transitions:
            if transition.next not in allowed:
                raise PlanSchemaError(
                    f'node {node.id!r} references unknown target '
                    f'{transition.next!r}'
                )
        if node.kind == 'observe':
            unknown = node.transition('unknown')
            if unknown not in GRAPH_TERMINALS:
                raise PlanSchemaError(
                    f'observe node {node.id!r}: unknown must lead to a safe '
                    f'terminal, got {unknown!r}',
                    code='unsafe_unknown',
                    details={'node_id': node.id, 'unknown': unknown},
                )
    reachable: set[str] = set()
    stack = [graph.entry]
    by_id = {node.id: node for node in graph.nodes}
    while stack:
        current = stack.pop()
        if current in reachable or current not in by_id:
            continue
        reachable.add(current)
        stack.extend(item.next for item in by_id[current].transitions)
    unreachable = ids - reachable
    if unreachable:
        raise PlanSchemaError(f'unreachable nodes {sorted(unreachable)}')
    _check_bounded_cycles(by_id)


def parse_response(raw: str | Mapping[str, Any]) -> PlanGraph:
    """Parse a graph-v3 response or raise :class:`PlanSchemaError`."""
    if isinstance(raw, str):
        if len(raw.encode('utf-8')) > MAX_RESPONSE_BYTES:
            raise PlanSchemaError(
                f'response exceeds {MAX_RESPONSE_BYTES} bytes'
            )
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError as error:
            raise PlanSchemaError(
                f'response is not valid JSON: {error}', code='invalid',
            ) from error
    elif isinstance(raw, Mapping):
        payload = dict(raw)
    else:
        raise PlanSchemaError('response must be a JSON string or object')
    if not isinstance(payload, Mapping):
        raise PlanSchemaError('response must be a JSON object')
    if payload.get('schema_version') != GRAPH_SCHEMA_VERSION:
        raise PlanSchemaError(
            'only graph-v3 responses with schema_version=3 are accepted',
            code='unsupported_contract',
            details={'schema_version': payload.get('schema_version')},
        )
    status = _string(payload.get('status'), 'status', allow_empty=False)
    if status not in ALLOWED_STATUSES:
        raise PlanSchemaError(
            f'status must be one of {list(ALLOWED_STATUSES)}, got {status!r}'
        )
    extra = set(payload) - TOP_LEVEL_KEYS[status]
    if extra:
        raise PlanSchemaError(f'response has unknown keys {sorted(extra)}')
    goals_raw = payload.get('goals', [])
    if not isinstance(goals_raw, list):
        raise PlanSchemaError('goals must be a list')
    goals = tuple(_goal(entry, index) for index, entry in enumerate(goals_raw))
    goal_ids = [goal.id for goal in goals]
    if len(goal_ids) != len(set(goal_ids)):
        raise PlanSchemaError('duplicate goal ids')
    plan_raw = payload.get('plan', [])
    if not isinstance(plan_raw, list):
        raise PlanSchemaError('plan must be a list')

    if status != 'ready':
        details: dict[str, Any] = {'status': status}
        if plan_raw:
            raise PlanSchemaError(
                f'{status} must not contain an executable plan',
                code='invalid',
                details={'status': status},
            )
        if status == 'clarification_required':
            question = _string(
                payload.get('question'), 'question', allow_empty=False,
            )
            missing_raw = payload.get('missing')
            if not isinstance(missing_raw, list) or not missing_raw:
                raise PlanSchemaError('missing must be a non-empty list')
            missing = [
                _string(value, f'missing[{index}]', allow_empty=False)
                for index, value in enumerate(missing_raw)
            ]
            details.update({'question': question, 'missing': missing})
        else:
            reason = _string(
                payload.get('reason'), 'reason', allow_empty=False,
            )
            reason_code = payload.get('reason_code')
            if reason_code is not None:
                reason_code = _string(reason_code, 'reason_code')
            entities = payload.get('entities', [])
            if not isinstance(entities, list):
                raise PlanSchemaError('entities must be a list')
            evidence = payload.get('evidence', {})
            if not isinstance(evidence, Mapping):
                raise PlanSchemaError('evidence must be an object')
            details.update({
                'reason': reason,
                'reason_code': reason_code,
                'entities': [_string(value, 'entities[]') for value in entities],
                'evidence': dict(evidence),
            })
        return PlanGraph(
            status=status,
            entry='',
            nodes=(),
            goals=goals,
            raw=copy.deepcopy(payload),
            status_details=details,
        )

    entry = _string(payload.get('entry'), 'entry', allow_empty=False)
    if not goals:
        raise PlanSchemaError('a ready graph must contain at least one goal')
    if not plan_raw:
        raise PlanSchemaError('a ready graph must contain at least one node')
    if len(plan_raw) > MAX_NODES:
        raise PlanSchemaError(
            f'plan contains {len(plan_raw)} nodes; limit is {MAX_NODES}'
        )
    nodes: list[PlanNode] = []
    for index, raw_node in enumerate(plan_raw):
        node_raw = _mapping(raw_node, f'plan[{index}]')
        kind = node_raw.get('kind')
        if kind == 'skill':
            nodes.append(_skill_node(node_raw, index))
        elif kind == 'observe':
            nodes.append(_observe_node(node_raw, index))
        else:
            raise PlanSchemaError(
                f'plan[{index}].kind must be "skill" or "observe", '
                f'got {kind!r}'
            )
    ids = [node.id for node in nodes]
    if len(ids) != len(set(ids)):
        raise PlanSchemaError('duplicate node ids')
    graph = PlanGraph(
        status='ready',
        entry=entry,
        nodes=tuple(nodes),
        goals=goals,
        raw=copy.deepcopy(payload),
    )
    _check_references(graph)
    return graph
