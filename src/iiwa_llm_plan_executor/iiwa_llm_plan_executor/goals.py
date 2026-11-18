"""Confirm graph-v3 goals and observations from the live supervisor state."""
from __future__ import annotations

from typing import Any, Mapping

from iiwa_experement.scenario import ScenarioSpec

from .schema import Goal


def _tool_state(state: Mapping[str, Any], tool: str) -> str | None:
    entry = (state.get('tools') or {}).get(tool)
    if isinstance(entry, Mapping) and 'state' in entry:
        return str(entry.get('state'))
    return None


def _screw_entry(state: Mapping[str, Any], screw: str) -> Mapping[str, Any] | None:
    entry = (state.get('screws') or {}).get(screw)
    return entry if isinstance(entry, Mapping) else None


def _hole_entry(state: Mapping[str, Any], hole: str) -> Mapping[str, Any] | None:
    entry = (state.get('holes') or {}).get(hole)
    return entry if isinstance(entry, Mapping) else None


def _cap_state(state: Mapping[str, Any]) -> str | None:
    entry = state.get('cap')
    if isinstance(entry, Mapping) and 'state' in entry:
        return str(entry.get('state'))
    return None


def _object_exists(object_id: str, state: Mapping[str, Any], spec: ScenarioSpec) -> bool:
    if object_id == 'cap':
        return spec.cap_state != 'absent' and _cap_state(state) not in ('absent',)
    if object_id in spec.tool_names:
        return True
    if object_id in {screw.name for screw in spec.screws}:
        entry = _screw_entry(state, object_id)
        return entry is not None and entry.get('state') != 'missing'
    return object_id in {'fixture', 'body', 'tray'}


def _installed(args: Mapping[str, str], state: Mapping[str, Any]) -> bool:
    if args.get('part') != 'cap' or args.get('destination') != 'hatch_seat':
        return False
    return _cap_state(state) == 'installed'


def _handoff(args: Mapping[str, str], state: Mapping[str, Any]) -> bool:
    screw = args.get('screw', '')
    hole = args.get('hole', '')
    entry = _screw_entry(state, screw)
    if entry is not None and entry.get('state') == 'delivered':
        return entry.get('hole') in (hole, None)
    hole_entry = _hole_entry(state, hole)
    return bool(hole_entry and hole_entry.get('screw') == screw)


def _at(args: Mapping[str, str], state: Mapping[str, Any]) -> bool:
    obj = args.get('object', '')
    destination = args.get('destination', '')
    if obj == 'cap' and destination == 'hatch_seat':
        return _installed({'part': 'cap', 'destination': destination}, state)
    return _handoff({'screw': obj, 'hole': destination}, state)


def observe_predicate(
    name: str,
    args: Mapping[str, str],
    state: Mapping[str, Any],
    spec: ScenarioSpec,
) -> str:
    """Return true/false/unknown for one graph-v3 observation."""
    if name == 'installed':
        return 'true' if _installed(args, state) else 'false'
    if name == 'handoff':
        return 'true' if _handoff(args, state) else 'false'
    if name == 'at':
        return 'true' if _at(args, state) else 'false'
    if name == 'held':
        return 'true' if state.get('held_object') == args.get('object') else 'false'
    if name == 'tool_mounted':
        return 'true' if state.get('mounted_tool') == args.get('tool') else 'false'
    if name == 'tool_docked':
        tool = args.get('tool', '')
        socket = args.get('socket', '')
        if socket != f'{tool}_socket':
            return 'unknown'
        status = _tool_state(state, tool)
        if status is None:
            return 'unknown'
        return 'true' if status == 'docked' else 'false'
    if name == 'visible':
        return 'true' if _object_exists(args.get('object', ''), state, spec) else 'false'
    if name == 'hole_empty':
        entry = _hole_entry(state, args.get('hole', ''))
        if entry is None:
            return 'unknown'
        return 'true' if entry.get('state') == 'empty' else 'false'
    return 'unknown'


def goal_holds(
    goal: Goal,
    state: Mapping[str, Any],
    spec: ScenarioSpec,
) -> bool:
    return observe_predicate(goal.predicate, goal.args, state, spec) == 'true'


def evaluate_goals(
    goals: tuple[Goal, ...],
    state: Mapping[str, Any],
    spec: ScenarioSpec,
) -> tuple[list[str], list[str]]:
    confirmed: list[str] = []
    remaining: list[str] = []
    for goal in goals:
        target = confirmed if goal_holds(goal, state, spec) else remaining
        target.append(goal.canonical())
    return confirmed, remaining
