"""Strict parsing of the externally supplied Qwen3 graph response."""
from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any, Mapping


MAX_NODES = 256
MAX_RESPONSE_BYTES = 200_000
ALLOWED_STATUSES = ('ready', 'blocked')
TERMINALS = ('completed', 'replan_required', 'operator_required')
TOP_LEVEL_KEYS = ('status', 'entry', 'plan', 'transitions')
NODE_KEYS = ('id', 'skill', 'object', 'target', 'transitions', 'params')


class PlanSchemaError(ValueError):
    """The response is not a well-formed plan graph."""


@dataclass(frozen=True)
class Transition:
    event: str
    next: str

    def as_dict(self) -> dict[str, str]:
        return {'event': self.event, 'next': self.next}


@dataclass(frozen=True)
class PlanNode:
    id: str
    skill: str
    object: str = ''
    target: str = ''
    params: dict[str, Any] = field(default_factory=dict)
    transitions: tuple[Transition, ...] = ()

    def as_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            'id': self.id,
            'skill': self.skill,
            'object': self.object,
            'target': self.target,
            'transitions': [entry.as_dict() for entry in self.transitions],
        }
        if self.params:
            payload['params'] = dict(self.params)
        return payload


@dataclass(frozen=True)
class PlanGraph:
    status: str
    entry: str
    nodes: tuple[PlanNode, ...]
    top_level_transitions: tuple[tuple[str, Transition], ...] = ()

    def node(self, node_id: str) -> PlanNode:
        for item in self.nodes:
            if item.id == node_id:
                return item
        raise KeyError(node_id)

    @property
    def ids(self) -> tuple[str, ...]:
        return tuple(item.id for item in self.nodes)

    def as_dict(self) -> dict[str, Any]:
        return {
            'status': self.status,
            'entry': self.entry,
            'plan': [item.as_dict() for item in self.nodes],
        }


def _string(value: Any, label: str, *, allow_empty: bool = True) -> str:
    if not isinstance(value, str):
        raise PlanSchemaError(f'{label} must be a string')
    text = value.strip()
    if not allow_empty and not text:
        raise PlanSchemaError(f'{label} must not be empty')
    return text


def _params(raw: Any, label: str) -> dict[str, Any]:
    if raw is None:
        return {}
    if not isinstance(raw, Mapping):
        raise PlanSchemaError(f'{label}.params must be an object')
    result: dict[str, Any] = {}
    for key, value in raw.items():
        if not isinstance(key, str) or not key:
            raise PlanSchemaError(f'{label}.params keys must be non-empty strings')
        if isinstance(value, bool) or not isinstance(value, (int, float, str)):
            raise PlanSchemaError(
                f'{label}.params.{key} must be a number or string'
            )
        result[key] = value
    return result


def _transition(raw: Any, label: str) -> Transition:
    if not isinstance(raw, Mapping):
        raise PlanSchemaError(f'{label} must be an object')
    extra = set(raw) - {'event', 'next'}
    if extra:
        raise PlanSchemaError(f'{label} has unknown keys {sorted(extra)}')
    event = _string(raw.get('event'), f'{label}.event', allow_empty=False)
    next_id = _string(raw.get('next'), f'{label}.next', allow_empty=False)
    return Transition(event, next_id)


def _node(raw: Any, index: int) -> PlanNode:
    label = f'plan[{index}]'
    if not isinstance(raw, Mapping):
        raise PlanSchemaError(f'{label} must be an object')
    extra = set(raw) - set(NODE_KEYS)
    if extra:
        raise PlanSchemaError(f'{label} has unknown keys {sorted(extra)}')
    node_id = _string(raw.get('id'), f'{label}.id', allow_empty=False)
    skill = _string(raw.get('skill'), f'{label}.skill', allow_empty=False)
    obj = _string(raw.get('object', ''), f'{label}.object')
    target = _string(raw.get('target', ''), f'{label}.target')
    transitions_raw = raw.get('transitions', [])
    if not isinstance(transitions_raw, list):
        raise PlanSchemaError(f'{label}.transitions must be a list')
    transitions: list[Transition] = []
    seen: set[str] = set()
    for t_index, entry in enumerate(transitions_raw):
        transition = _transition(entry, f'{label}.transitions[{t_index}]')
        if transition.event in seen:
            raise PlanSchemaError(
                f'{label} has duplicate event {transition.event!r}'
            )
        seen.add(transition.event)
        transitions.append(transition)
    return PlanNode(
        id=node_id,
        skill=skill,
        object=obj,
        target=target,
        params=_params(raw.get('params'), label),
        transitions=tuple(transitions),
    )


def _merge_top_level(
    nodes: list[PlanNode],
    raw: Any,
) -> tuple[list[PlanNode], tuple[tuple[str, Transition], ...]]:
    if raw is None:
        return nodes, ()
    if not isinstance(raw, list):
        raise PlanSchemaError('transitions must be a list')
    by_id = {node.id: node for node in nodes}
    merged: dict[str, list[Transition]] = {
        node.id: list(node.transitions) for node in nodes
    }
    top_level: list[tuple[str, Transition]] = []
    for index, entry in enumerate(raw):
        label = f'transitions[{index}]'
        if not isinstance(entry, Mapping):
            raise PlanSchemaError(f'{label} must be an object')
        extra = set(entry) - {'from', 'event', 'to', 'next'}
        if extra:
            raise PlanSchemaError(f'{label} has unknown keys {sorted(extra)}')
        source = _string(entry.get('from'), f'{label}.from', allow_empty=False)
        next_id = entry.get('to', entry.get('next'))
        transition = Transition(
            _string(entry.get('event'), f'{label}.event', allow_empty=False),
            _string(next_id, f'{label}.to', allow_empty=False),
        )
        if source not in by_id:
            raise PlanSchemaError(f'{label}.from references unknown node {source!r}')
        existing = {item.event for item in merged[source]}
        if transition.event in existing:
            raise PlanSchemaError(
                f'{label}: node {source!r} already has event '
                f'{transition.event!r}'
            )
        merged[source].append(transition)
        top_level.append((source, transition))
    result = [
        PlanNode(
            id=node.id,
            skill=node.skill,
            object=node.object,
            target=node.target,
            params=node.params,
            transitions=tuple(merged[node.id]),
        )
        for node in nodes
    ]
    return result, tuple(top_level)


def parse_response(raw: str | Mapping[str, Any]) -> PlanGraph:
    """Parse a model response or raise :class:`PlanSchemaError`."""
    if isinstance(raw, str):
        if len(raw.encode('utf-8')) > MAX_RESPONSE_BYTES:
            raise PlanSchemaError(
                f'response exceeds {MAX_RESPONSE_BYTES} bytes'
            )
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError as error:
            raise PlanSchemaError(f'response is not valid JSON: {error}') from error
    elif isinstance(raw, Mapping):
        payload = dict(raw)
    else:
        raise PlanSchemaError('response must be a JSON string or object')
    if not isinstance(payload, Mapping):
        raise PlanSchemaError('response must be a JSON object')
    extra = set(payload) - set(TOP_LEVEL_KEYS)
    if extra:
        raise PlanSchemaError(f'response has unknown keys {sorted(extra)}')
    status = _string(payload.get('status'), 'status', allow_empty=False)
    if status not in ALLOWED_STATUSES:
        raise PlanSchemaError(
            f'status must be one of {list(ALLOWED_STATUSES)}, got {status!r}'
        )
    entry = _string(payload.get('entry', ''), 'entry')
    plan_raw = payload.get('plan')
    if not isinstance(plan_raw, list):
        raise PlanSchemaError('plan must be a list')
    if len(plan_raw) > MAX_NODES:
        raise PlanSchemaError(
            f'plan contains {len(plan_raw)} nodes; limit is {MAX_NODES}'
        )
    nodes: list[PlanNode] = []
    seen_ids: set[str] = set()
    for index, raw_node in enumerate(plan_raw):
        node = _node(raw_node, index)
        if node.id in seen_ids:
            raise PlanSchemaError(f'duplicate node id {node.id!r}')
        if node.id in TERMINALS:
            raise PlanSchemaError(
                f'node id {node.id!r} is a reserved terminal'
            )
        seen_ids.add(node.id)
        nodes.append(node)
    nodes, top_level = _merge_top_level(nodes, payload.get('transitions'))
    if status == 'ready':
        if not nodes:
            raise PlanSchemaError('a ready graph must contain at least one node')
        if not entry:
            raise PlanSchemaError('a ready graph must have a non-empty entry')
    elif nodes or entry:
        raise PlanSchemaError('a blocked response must not contain an executable plan')
    return PlanGraph(status, entry, tuple(nodes), top_level)
