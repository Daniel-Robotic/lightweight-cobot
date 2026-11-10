"""Validate a parsed graph against the live scene and the server catalog."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable, Mapping

from iiwa_scene_context.catalog import ALLOWED_PARAMS, SkillCatalog

from .schema import TERMINALS, PlanGraph, PlanNode


SAFE_COLLISION_SKILLS = {'stop', 'notify_operator'}
SAFE_GRASP_FAILURE_SKILLS = {'inspect_state', 'stop'}
SAFE_FAILURE_TERMINALS = {'replan_required', 'operator_required'}
PROGRESS_EVENTS = {
    'success', 'ok', 'found', 'clear', 'empty', 'holding', 'placed',
}
REQUIRES_OBJECT = {
    'detect', 'approach', 'lower', 'grab', 'lift', 'transfer', 'align',
    'release', 'retreat', 'verify', 'dispense', 'dock_tool',
    'handoff_screw', 'pick', 'move_to_storage', 'inspect', 'inspect_state',
    'capture',
}


@dataclass(frozen=True)
class ValidationResult:
    ok: bool
    code: str
    reason: str
    details: dict[str, Any] = field(default_factory=dict)

    @property
    def accepted(self) -> bool:
        return self.ok

    def as_dict(self) -> dict[str, Any]:
        return {
            'ok': self.ok,
            'code': self.code,
            'reason': self.reason,
            'details': dict(self.details),
        }


@dataclass(frozen=True)
class _State:
    mounted: str | None = None
    held: str | None = None
    unknown: bool = False
    transfer_target: str | None = None


@dataclass(frozen=True)
class _Facts:
    components: dict[str, str]
    tools: dict[str, str]
    destinations: dict[str, str]
    cameras: set[str]
    service_objects: set[str]
    initial: _State
    available_skills: set[str]
    envelope: Mapping[str, Any]


def _scene_payload(scene: Mapping[str, Any]) -> tuple[Mapping[str, Any], Mapping[str, Any]]:
    if 'scene_json' in scene:
        payload = scene['scene_json']
        if not isinstance(payload, Mapping):
            raise ValueError('scene_json must be an object')
        return scene, payload
    return {}, scene


def _facts(scene: Mapping[str, Any]) -> _Facts:
    envelope, payload = _scene_payload(scene)
    components = {
        entry['id']: str((entry.get('attributes') or {}).get('kind', 'component'))
        for entry in payload.get('components', [])
        if isinstance(entry, Mapping) and isinstance(entry.get('id'), str)
    }
    tools = {
        entry['id']: str(entry.get('socket', f"{entry['id']}_socket"))
        for entry in payload.get('tools', [])
        if isinstance(entry, Mapping) and isinstance(entry.get('id'), str)
    }
    destinations = {
        entry['id']: str(entry.get('kind', 'destination'))
        for entry in payload.get('destinations', [])
        if isinstance(entry, Mapping) and isinstance(entry.get('id'), str)
    }
    cameras = set(payload.get('cameras', {}))
    service_objects = set(payload.get('service_objects', {}))
    initial_raw = payload.get('initial_state') or {}
    initial = _State(
        mounted=initial_raw.get('tool_mounted') or None,
        held=initial_raw.get('held_object') or None,
    )
    available = set(payload.get('available_skills', []))
    return _Facts(
        components=components,
        tools=tools,
        destinations=destinations,
        cameras=cameras,
        service_objects=service_objects,
        initial=initial,
        available_skills=available,
        envelope=envelope,
    )


def _reject(reason: str, *, code: str = 'rejected', **details) -> ValidationResult:
    return ValidationResult(False, code, reason, details)


def _replan(reason: str, **details) -> ValidationResult:
    return ValidationResult(False, 'replan_required', reason, details)


def _node_or_terminal(graph: PlanGraph, node_id: str) -> PlanNode | None:
    if node_id in TERMINALS:
        return None
    return graph.node(node_id)


def _structural_checks(graph: PlanGraph, facts: _Facts) -> ValidationResult | None:
    ids = set(graph.ids)
    for node in graph.nodes:
        if not node.transitions:
            return _reject(f'node {node.id!r} has no transitions')
        if node.skill not in facts.available_skills:
            return _reject(
                f'node {node.id!r} uses unknown or unavailable skill '
                f'{node.skill!r}'
            )
        if node.object:
            known = (
                node.object in facts.components
                or node.object in facts.tools
                or node.object in facts.cameras
                or node.object in facts.service_objects
                or node.object in facts.destinations
            )
            if not known:
                return _reject(
                    f'node {node.id!r} references unknown object '
                    f'{node.object!r}'
                )
        if node.target:
            known = (
                node.target in facts.destinations
                or node.target in facts.components
                or node.target in facts.tools
            )
            if not known:
                return _reject(
                    f'node {node.id!r} references unknown target '
                    f'{node.target!r}'
                )
        if node.params:
            allowed_params = set(ALLOWED_PARAMS.get(node.skill, ()))
            unknown_params = set(node.params) - allowed_params
            if unknown_params:
                return _reject(
                    f'node {node.id!r} has unsupported params '
                    f'{sorted(unknown_params)}'
                )
            if node.skill == 'dispense':
                for key in ('start_fraction', 'end_fraction'):
                    if key in node.params:
                        value = node.params[key]
                        if (
                            isinstance(value, bool)
                            or not isinstance(value, (int, float))
                            or not 0.0 <= float(value) <= 1.0
                        ):
                            return _reject(
                                f'node {node.id!r} param {key} must be '
                                'within 0..1'
                            )
                start = float(node.params.get('start_fraction', 0.0))
                end = float(node.params.get('end_fraction', 1.0))
                if start > end:
                    return _reject(
                        f'node {node.id!r} dispense start_fraction exceeds '
                        'end_fraction'
                    )
        allowed_events = set(facts.envelope.get('skill_contracts', {}).get(
            node.skill, {},
        ).get('outcomes', {}))
        if not allowed_events:
            # The server catalog remains authoritative even if the envelope
            # was produced by an older snapshot.
            from iiwa_scene_context.catalog import SKILL_OUTCOMES
            allowed_events = set(SKILL_OUTCOMES.get(node.skill, {}))
        events = {transition.event for transition in node.transitions}
        missing = allowed_events - events
        extra = events - allowed_events
        if missing:
            return _reject(
                f'node {node.id!r} misses required transitions {sorted(missing)}'
            )
        if extra:
            return _reject(
                f'node {node.id!r} has unknown events {sorted(extra)}'
            )
        for transition in node.transitions:
            if transition.next not in ids and transition.next not in TERMINALS:
                return _reject(
                    f'node {node.id!r} points to unknown node '
                    f'{transition.next!r}'
                )
            if transition.next == node.id:
                # A self-loop that does not inspect state would spin forever.
                if node.skill not in ('inspect_state', 'wait_scene_clear'):
                    return _reject(
                        f'node {node.id!r} has a direct self-loop'
                    )
    if graph.entry not in ids:
        return _reject(f'entry {graph.entry!r} is not a plan node')

    reachable: set[str] = set()
    stack = [graph.entry]
    while stack:
        current = stack.pop()
        if current in reachable or current not in ids:
            continue
        reachable.add(current)
        stack.extend(item.next for item in graph.node(current).transitions)
    unreachable = ids - reachable
    if unreachable:
        return _reject(f'unreachable nodes {sorted(unreachable)}')

    reverse: dict[str, set[str]] = {node_id: set() for node_id in ids}
    for node in graph.nodes:
        for transition in node.transitions:
            if transition.next in ids:
                reverse[transition.next].add(node.id)
    can_finish = set(TERMINALS)
    changed = True
    while changed:
        changed = False
        for node_id in ids - can_finish:
            if any(item in can_finish for item in _targets(graph.node(node_id))):
                can_finish.add(node_id)
                changed = True
    dead = ids - can_finish
    if dead:
        return _reject(
            f'nodes cannot reach a terminal: {sorted(dead)}'
        )

    for node in graph.nodes:
        for transition in node.transitions:
            target = _node_or_terminal(graph, transition.next)
            if transition.event == 'collision':
                if target is None and transition.next not in SAFE_FAILURE_TERMINALS:
                    return _reject(
                        f'collision in {node.id!r} must stop the operation'
                    )
                if target is not None and target.skill not in SAFE_COLLISION_SKILLS:
                    return _reject(
                        f'collision in {node.id!r} must reach stop/notify, '
                        f'got {target.skill!r}'
                    )
            if transition.event == 'grasp_failed':
                if target is None and transition.next not in SAFE_FAILURE_TERMINALS:
                    return _reject(
                        f'grasp_failed in {node.id!r} must not continue blindly'
                    )
                if target is not None and target.skill not in SAFE_GRASP_FAILURE_SKILLS:
                    return _reject(
                        f'grasp_failed in {node.id!r} must reach inspect_state '
                        f'or stop, got {target.skill!r}'
                    )
            if transition.event in (
                'not_found', 'failed', 'not_ok', 'timeout', 'unknown',
            ) and transition.next == 'completed':
                return _reject(
                    f'{transition.event} in {node.id!r} cannot map to completed'
                )
    return None


def _targets(node: PlanNode) -> list[str]:
    return [item.next for item in node.transitions]


def _precondition(
    node: PlanNode,
    state: _State,
    facts: _Facts,
) -> ValidationResult | None:
    skill = node.skill
    obj = node.object
    target = node.target
    if node.id in ('',) or skill not in facts.available_skills:
        return _reject(f'unknown skill {skill!r}')

    if skill in REQUIRES_OBJECT and not obj:
        return _reject(f'{skill} in {node.id!r} requires an object')

    if state.unknown and skill not in (
        'inspect_state', 'stop', 'notify_operator', 'wait_scene_clear',
    ):
        return _replan(
            f'{node.id!r} follows an unconfirmed grasp; inspect_state is required',
        )

    if skill in ('grab', 'pick'):
        if obj in facts.tools:
            if state.mounted is not None:
                return _replan(
                    f'{node.id!r} cannot grab {obj!r}: tool '
                    f'{state.mounted!r} is already mounted'
                )
            if state.held is not None:
                return _replan(
                    f'{node.id!r} cannot grab {obj!r} while holding '
                    f'{state.held!r}'
                )
            return None
        if obj == 'cap':
            if 'cap' not in facts.components:
                return _reject(f'{node.id!r}: hatch is absent from this scene')
            if state.mounted != 'vacuum':
                return _replan(
                    f'{node.id!r} requires the vacuum tool in patron; '
                    f'mounted tool is {state.mounted!r}'
                )
            if state.held is not None:
                return _replan(
                    f'{node.id!r} cannot grab the hatch while holding '
                    f'{state.held!r}'
                )
            return None
        if obj in facts.components and facts.components[obj] == 'screw':
            if state.mounted != 'magnetic_tool':
                return _replan(
                    f'{node.id!r} requires magnetic_tool in patron; '
                    f'mounted tool is {state.mounted!r}'
                )
            if state.held is not None:
                return _replan(
                    f'{node.id!r} cannot grab a screw while holding '
                    f'{state.held!r}'
                )
            return None
        return _reject(f'{node.id!r}: object {obj!r} cannot be grasped')

    if skill == 'release':
        if obj in facts.tools:
            socket = facts.tools[obj]
            if state.mounted != obj:
                return _replan(
                    f'{node.id!r}: {obj!r} is not mounted '
                    f'(mounted={state.mounted!r})'
                )
            if target != socket:
                return _replan(
                    f'{node.id!r}: release({obj}) must name its socket '
                    f'{socket!r}'
                )
            if state.transfer_target != socket:
                return _replan(
                    f'{node.id!r}: release({obj},{socket}) requires an explicit '
                    'preceding transfer to that socket'
                )
            return None
        if obj == 'cap':
            if state.held != 'cap':
                return _replan(f'{node.id!r}: hatch is not held')
            if facts.destinations.get(target) != 'seat':
                return _replan(
                    f'{node.id!r}: hatch release requires hatch_seat, '
                    f'got {target!r}'
                )
            return None
        if obj in facts.components and facts.components[obj] == 'screw':
            if state.held != obj:
                return _replan(f'{node.id!r}: {obj!r} is not held')
            if facts.destinations.get(target) != 'hole':
                return _replan(
                    f'{node.id!r}: screw handoff requires a hole target'
                )
            return None
        return _reject(f'{node.id!r}: object {obj!r} cannot be released')

    if skill in ('transfer', 'move_to_storage'):
        if obj in facts.tools:
            if state.mounted != obj:
                return _replan(f'{node.id!r}: {obj!r} is not mounted')
            if target not in ('work', facts.tools[obj]):
                return _replan(
                    f'{node.id!r}: tool transfer must target work or its socket'
                )
            return None
        if obj == 'cap':
            if state.held != 'cap':
                return _replan(f'{node.id!r}: hatch is not held')
            if target != 'hatch_seat' and target != 'work':
                return _replan(
                    f'{node.id!r}: hatch transfer must target hatch_seat or work'
                )
            return None
        if obj in facts.components and facts.components[obj] == 'screw':
            if state.held != obj:
                return _replan(f'{node.id!r}: {obj!r} is not held')
            if facts.destinations.get(target) != 'hole' and target != 'work':
                return _replan(
                    f'{node.id!r}: screw transfer must target a hole or work'
                )
            return None
        return _reject(f'{node.id!r}: object {obj!r} cannot be transferred')

    if skill == 'align':
        if state.held is None and state.mounted is None:
            return _replan(f'{node.id!r}: align requires a held part or tool')
        return None

    if skill == 'dispense':
        if state.mounted != 'sealant_tool':
            return _replan(
                f'{node.id!r}: dispense requires sealant_tool in patron'
            )
        side = obj or target
        if facts.destinations.get(side) != 'contour':
            return _reject(
                f'{node.id!r}: dispense target {side!r} is not a sealant side'
            )
        return None

    if skill == 'dock_tool':
        socket = facts.tools.get(obj)
        if socket is None:
            return _reject(f'{node.id!r}: {obj!r} is not a present tool')
        if state.mounted != obj:
            return _replan(
                f'{node.id!r}: {obj!r} is not mounted '
                f'(mounted={state.mounted!r})'
            )
        if target != socket:
            return _replan(
                f'{node.id!r}: dock_tool must target socket {socket!r}'
            )
        if state.transfer_target != socket:
            return _replan(
                f'{node.id!r}: dock_tool requires an explicit preceding '
                'transfer to that socket'
            )
        return None

    if skill == 'handoff_screw':
        if obj not in facts.components or facts.components[obj] != 'screw':
            return _reject(f'{node.id!r}: {obj!r} is not a present screw')
        if state.held != obj:
            return _replan(f'{node.id!r}: {obj!r} is not held')
        if state.mounted != 'magnetic_tool':
            return _replan(
                f'{node.id!r}: handoff_screw requires magnetic_tool'
            )
        if facts.destinations.get(target) != 'hole':
            return _replan(
                f'{node.id!r}: handoff_screw target must be a hole'
            )
        return None

    if skill in ('capture',):
        if obj not in facts.cameras:
            return _reject(f'{node.id!r}: unknown camera {obj!r}')
    return None


def _apply_event(
    node: PlanNode,
    event: str,
    state: _State,
    facts: _Facts,
) -> _State:
    skill = node.skill
    obj = node.object
    if skill == 'inspect_state' and event in ('empty', 'holding', 'placed', 'unknown'):
        if event == 'empty':
            return _State(state.mounted, None, False, state.transfer_target)
        if event == 'holding':
            return _State(state.mounted, obj or None, False, state.transfer_target)
        if event == 'placed':
            return _State(state.mounted, None, False, state.transfer_target)
        return _State(state.mounted, state.held, True, state.transfer_target)
    if event == 'grasp_failed':
        return _State(state.mounted, state.held, True, state.transfer_target)
    if event not in PROGRESS_EVENTS:
        return state
    if skill in ('grab', 'pick'):
        if obj in facts.tools:
            return _State(obj, state.held, False, state.transfer_target)
        if obj == 'cap':
            return _State(state.mounted, 'cap', False, state.transfer_target)
        if obj and facts.components.get(obj) == 'screw':
            return _State(state.mounted, obj, False, state.transfer_target)
        return state
    if skill == 'release':
        if obj in facts.tools:
            return _State(None, state.held, False, state.transfer_target)
        if obj:
            return _State(state.mounted, None, False, state.transfer_target)
        return state
    if skill == 'dock_tool':
        return _State(None, state.held, False, state.transfer_target)
    if skill == 'handoff_screw':
        return _State(state.mounted, None, False, state.transfer_target)
    if skill == 'transfer':
        return _State(
            state.mounted, state.held, False, node.target or state.transfer_target,
        )
    return state


def _ownership_checks(graph: PlanGraph, facts: _Facts) -> ValidationResult | None:
    initial = facts.initial
    seen: set[tuple[str, _State]] = set()
    queue: list[tuple[str, _State]] = [(graph.entry, initial)]
    visited_states: dict[str, set[_State]] = {node_id: set() for node_id in graph.ids}
    while queue:
        node_id, state = queue.pop()
        if (node_id, state) in seen:
            continue
        seen.add((node_id, state))
        node = graph.node(node_id)
        failure = _precondition(node, state, facts)
        if failure is not None:
            failure.details.update({
                'node_id': node.id,
                'skill': node.skill,
                'mounted_tool': state.mounted,
                'held_object': state.held,
            })
            return failure
        visited_states[node.id].add(state)
        for transition in node.transitions:
            if transition.next not in graph.ids:
                continue
            next_state = _apply_event(node, transition.event, state, facts)
            if (transition.next, next_state) not in seen:
                queue.append((transition.next, next_state))
    return None


def validate_graph(
    graph: PlanGraph,
    scene: Mapping[str, Any],
    catalog: SkillCatalog,
    *,
    expected_scene_revision: int | None = None,
) -> ValidationResult:
    """Return an admission decision; never mutates the graph or the scene."""
    if graph.status == 'blocked':
        return ValidationResult(
            False, 'blocked', 'the model reported status=blocked; no motion',
            {'status': graph.status},
        )
    if graph.status != 'ready':
        return _reject(f'unsupported graph status {graph.status!r}')
    if len(graph.nodes) > 256:
        return _reject('graph exceeds the 256-node limit')
    facts = _facts(scene)
    if expected_scene_revision is not None:
        revision = facts.envelope.get('scene_revision')
        if revision is not None and int(revision) != int(expected_scene_revision):
            return _reject(
                f'graph was built for scene_revision {expected_scene_revision}, '
                f'current revision is {revision}',
                code='stale_scene',
            )
    structural = _structural_checks(graph, facts)
    if structural is not None:
        return structural
    ownership = _ownership_checks(graph, facts)
    if ownership is not None:
        return ownership
    return ValidationResult(
        True,
        'ok',
        'graph accepted',
        {
            'nodes': len(graph.nodes),
            'entry': graph.entry,
            'scene_revision': facts.envelope.get('scene_revision'),
            'available_skills': len(facts.available_skills),
        },
    )
