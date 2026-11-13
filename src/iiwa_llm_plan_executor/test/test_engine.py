"""Graph engine: admission, one-edge transitions, terminals and cancellation."""
from __future__ import annotations

import threading
from pathlib import Path
from types import SimpleNamespace

from iiwa_experement.geometry import load_geometry
from iiwa_experement.scenario import load_scenario
from iiwa_llm_plan_executor.engine import GraphEngine
from iiwa_llm_plan_executor.schema import parse_response
from iiwa_scene_context.catalog import DEFAULT_CATALOG
from iiwa_scene_context.snapshot import build_envelope, build_scene


PACKAGE = Path(__file__).resolve().parents[1]
SCENES = PACKAGE.parent / 'iiwa_experement' / 'config' / 'scenes.yaml'
GEOMETRY = load_geometry(PACKAGE.parent / 'iiwa_experement' / 'config' / 'geometry.yaml')
SAFE = {
    'collision': 'operator_required',
    'grasp_failed': 'operator_required',
    'human_in_scene': 'operator_required',
    'not_found': 'replan_required',
    'failed': 'replan_required',
    'not_ok': 'operator_required',
    'timeout': 'operator_required',
    'unknown': 'operator_required',
}
PROGRESS = {'success', 'ok', 'found', 'clear', 'empty', 'holding', 'placed'}


class FakeContext:
    def __init__(self, scene_json):
        self.scene_json = scene_json
        self.mounted_tool = None
        self.held_object = None
        self.scene_revision = 0
        self.ownership_unknown = False


class FakeSkills:
    def __init__(self):
        self.calls = []
        self.events = {}
        self.details = {}
        self.on_execute = None
        self.block = False
        self.started = threading.Event()
        self.release = threading.Event()

    def execute(self, node, context):
        if self.on_execute is not None:
            self.on_execute(node, context)
        if self.block:
            self.started.set()
            self.release.wait(timeout=5.0)
        self.calls.append(node.id)
        return SimpleNamespace(
            event=self.events.get(node.id, 'success'),
            details=self.details.get(node.id, {}),
        )


def envelope(revision=1, stamp=5.0, experiment_id=1):
    spec = load_scenario(experiment_id, SCENES)
    tools = {tool.name: {'state': 'docked'} for tool in spec.tools}
    scene = build_scene(
        spec,
        {
            'version': revision,
            'mounted_tool': None,
            'held_object': None,
            'ready': True,
            'cap': {'state': spec.cap_state},
            'tools': tools,
            'screws': {},
            'holes': {},
        },
        DEFAULT_CATALOG,
        GEOMETRY,
        stamp=stamp,
    )
    return build_envelope(
        scene,
        state_version=revision,
        stamp=stamp,
        experiment_id=experiment_id,
    )


def node(node_id, skill, obj='', target='', next_on=None):
    events = DEFAULT_CATALOG.outcomes(skill)
    next_on = next_on or {}
    transitions = []
    for event in events:
        if event in next_on:
            destination = next_on[event]
        elif event in PROGRESS:
            destination = 'completed'
        else:
            destination = SAFE.get(event, 'operator_required')
        transitions.append({'event': event, 'next': destination})
    return {
        'id': node_id,
        'skill': skill,
        'object': obj,
        'target': target,
        'transitions': transitions,
    }


def graph(entry, *nodes):
    return parse_response({
        'status': 'ready',
        'entry': entry,
        'plan': list(nodes),
    })


STOP_GRAPH = lambda: graph(
    'a',
    node('a', 'stop', 'robot', next_on={'success': 'b'}),
    node('b', 'stop', 'robot', next_on={'success': 'completed'}),
)

GRAB_GRAPH = lambda: graph(
    'g',
    node('g', 'grab', 'vacuum', next_on={'success': 't'}),
    node('t', 'transfer', 'vacuum', 'work',
         next_on={'success': 'completed'}),
)

BAD_GRAPH = lambda: graph(
    'g',
    node('g', 'grab', 'vacuum', next_on={'success': 'ghost'}),
)


def make_engine(skills=None, *, clock=5.0, provider=None, threaded=False,
                max_steps=512, **kwargs):
    skills = skills or FakeSkills()
    clock_box = {'now': clock}
    state = {'envelope': envelope(1, clock_box['now'])}
    if provider is not None:
        state['provider'] = provider

    def default_provider():
        return state['envelope']
    events = {
        'plans': [],
        'statuses': [],
        'replans': [],
        'stops': [],
    }
    engine = GraphEngine(
        state.get('provider', default_provider),
        lambda env, scene: FakeContext(scene),
        skills,
        clock=lambda: clock_box['now'],
        on_plan=lambda payload: events['plans'].append(payload),
        on_status=lambda payload: events['statuses'].append(payload),
        on_replan=lambda payload: events['replans'].append(payload),
        on_stop=lambda reason: events['stops'].append(reason),
        robot_fresh=lambda: (True, 'ok'),
        threaded=threaded,
        max_steps=max_steps,
        **kwargs,
    )
    engine.update_readiness(scene_ready=True, ready_at=0.0, motion_ready=True)
    return engine, skills, events, state, clock_box


def test_submission_waits_for_scene_and_the_three_second_delay():
    engine, _, _, _, _ = make_engine(clock=2.0)
    rejected = engine.submit('r1', STOP_GRAPH().as_dict())
    assert rejected.accepted is False
    assert rejected.code == 'not_ready'
    assert 'start delay' in rejected.message
    assert engine.get_status('r1').found is False

    engine, _, _, _, _ = make_engine(clock=3.01)
    accepted = engine.submit('r1', STOP_GRAPH().as_dict())
    assert accepted.accepted is True
    engine.run_accepted('r1')

    engine.update_readiness(scene_ready=False, ready_at=0.0, motion_ready=False)
    rejected = engine.submit('r2', STOP_GRAPH().as_dict())
    assert rejected.code == 'not_ready'


def test_rejected_graphs_are_not_registered_published_or_executed():
    engine, skills, events, _, _ = make_engine()
    blocked = engine.submit('r1', {'status': 'blocked', 'entry': '', 'plan': []})
    assert blocked.code == 'blocked'
    rejected = engine.submit('r2', BAD_GRAPH().as_dict())
    assert rejected.accepted is False
    assert rejected.code == 'rejected'
    assert events['plans'] == []
    assert skills.calls == []
    assert engine.get_status('r1').found is False
    assert engine.get_status('r2').found is False
    assert not any(
        event.get('state') == 'rejected' for event in events['statuses']
    )


def test_rejected_run_id_can_be_reused_with_a_corrected_graph():
    engine, skills, events, _, _ = make_engine()
    rejected = engine.submit('same-id', BAD_GRAPH().as_dict())
    assert rejected.accepted is False
    assert engine.get_status('same-id').found is False

    accepted = engine.submit('same-id', STOP_GRAPH().as_dict())
    assert accepted.accepted is True
    assert accepted.code == 'accepted'
    assert len(events['plans']) == 1
    engine.run_accepted('same-id')
    assert engine.get_status('same-id').state == 'completed'


def test_cap_grab_without_vacuum_is_admitted_and_follows_grasp_failed():
    engine, skills, events, _, _ = make_engine()
    value = graph(
        'g',
        node('g', 'grab', 'cap', next_on={
            'success': 'completed',
            'grasp_failed': 'operator_required',
        }),
    )
    accepted = engine.submit('r1', value.as_dict())
    assert accepted.accepted is True
    assert len(events['plans']) == 1
    skills.events = {'g': 'grasp_failed'}
    skills.details = {'g': {'reason': 'required tool vacuum is not mounted'}}
    engine.run_accepted('r1')
    status = engine.get_status('r1')
    assert status.state == 'operator_required'
    assert status.events[0]['event'] == 'grasp_failed'
    assert status.events[0]['next'] == 'operator_required'
    assert status.events[0]['reason']
    assert status.mounted_tool is None
    assert status.held_object is None


def test_accepted_graph_is_published_once_before_the_first_motion():
    engine, skills, events, _, _ = make_engine()
    order = []

    def on_plan(payload):
        events['plans'].append(payload)
        order.append(('plan', payload['run_id']))

    engine.on_plan = on_plan
    skills.on_execute = lambda node, context: order.append(('move', node.id))
    accepted = engine.submit('r1', STOP_GRAPH().as_dict())
    assert accepted.accepted
    engine.run_accepted('r1')
    assert order[0] == ('plan', 'r1')
    assert order[1] == ('move', 'a')
    assert len(events['plans']) == 1
    status = engine.get_status('r1')
    assert status.state == 'completed'
    assert status.found is True
    assert 'scene_revision' not in status.as_dict()


def test_same_run_id_is_idempotent_and_a_different_graph_conflicts():
    engine, _, events, _, _ = make_engine()
    first = engine.submit('r1', STOP_GRAPH().as_dict())
    assert first.accepted
    duplicate = engine.submit('r1', STOP_GRAPH().as_dict())
    assert duplicate.accepted is True
    assert duplicate.code == 'duplicate'
    assert len(events['plans']) == 1

    conflict = engine.submit('r1', GRAB_GRAPH().as_dict())
    assert conflict.accepted is False
    assert conflict.code == 'conflict'
    assert len(events['plans']) == 1

    busy = engine.submit('r2', STOP_GRAPH().as_dict())
    assert busy.accepted is False
    assert busy.code == 'busy'
    assert len(events['plans']) == 1


def test_accepted_run_id_stays_bound_after_a_terminal_state():
    engine, _, events, _, _ = make_engine()
    assert engine.submit('r1', STOP_GRAPH().as_dict()).accepted
    engine.run_accepted('r1')
    assert engine.get_status('r1').state == 'completed'

    duplicate = engine.submit('r1', STOP_GRAPH().as_dict())
    assert duplicate.accepted is True
    assert duplicate.code == 'duplicate'
    conflict = engine.submit('r1', GRAB_GRAPH().as_dict())
    assert conflict.accepted is False
    assert conflict.code == 'conflict'
    assert len(events['plans']) == 1


def test_plan_publication_failure_does_not_register_the_run():
    engine, skills, _, _, _ = make_engine()

    def broken(_payload):
        raise RuntimeError('publisher unavailable')

    engine.on_plan = broken
    rejected = engine.submit('r1', STOP_GRAPH().as_dict())
    assert rejected.accepted is False
    assert rejected.code == 'rejected'
    assert 'publisher unavailable' in rejected.message
    assert engine.get_status('r1').found is False
    assert skills.calls == []


def test_each_event_selects_exactly_one_edge_and_is_visible_in_status():
    engine, skills, _, _, _ = make_engine()
    engine.submit('r1', STOP_GRAPH().as_dict())
    skills.events = {'a': 'success', 'b': 'success'}
    engine.run_accepted('r1')
    status = engine.get_status('r1')
    assert status.state == 'completed'
    assert [event['node_id'] for event in status.events] == ['a', 'b']
    assert [event['next'] for event in status.events] == ['b', 'completed']
    assert all(event['run_id'] == 'r1' for event in status.events)
    assert all('scene_revision' not in event for event in status.events)


def test_replan_returns_a_fresh_scene_and_operator_required_stops():
    engine, skills, events, _, _ = make_engine()
    replan_graph = graph(
        'd',
        node('d', 'detect', 'cap',
             next_on={'found': 'replan_required'}),
    )
    engine.submit('r1', replan_graph.as_dict())
    skills.events = {'d': 'found'}
    engine.run_accepted('r1')
    assert engine.get_status('r1').state == 'replan_required'
    assert len(events['replans']) == 1
    assert events['replans'][0]['state_version'] == 1

    engine, skills, events, _, _ = make_engine()
    operator_graph = graph(
        'd',
        node('d', 'detect', 'cap',
             next_on={'found': 'operator_required'}),
    )
    engine.submit('r2', operator_graph.as_dict())
    skills.events = {'d': 'found'}
    engine.run_accepted('r2')
    assert engine.get_status('r2').state == 'operator_required'


def test_unknown_skill_outcome_stops_execution():
    engine, skills, events, _, _ = make_engine()
    engine.submit('r1', STOP_GRAPH().as_dict())
    skills.events = {'a': 'bogus'}
    engine.run_accepted('r1')
    status = engine.get_status('r1')
    assert status.state == 'failed'
    assert 'unknown skill outcome' in status.reason
    assert events['stops']


def test_lost_freshness_and_missing_scene_stop_motion():
    engine, _, events, _, _ = make_engine()
    engine.robot_fresh = lambda: (False, 'joint_states is stale')
    engine.submit('r1', STOP_GRAPH().as_dict())
    engine.run_accepted('r1')
    assert engine.get_status('r1').state == 'failed'
    assert 'joint_states' in engine.get_status('r1').reason
    assert events['stops']

    engine, skills, events, state, _ = make_engine()
    engine.submit('r2', STOP_GRAPH().as_dict())
    skills.on_execute = lambda node, context: state.update(envelope=None)
    engine.run_accepted('r2')
    assert engine.get_status('r2').state == 'failed'
    assert 'scene snapshot' in engine.get_status('r2').reason


def test_scene_state_change_stops_non_ownership_nodes_but_not_own_graph_changes():
    engine, skills, events, state, _ = make_engine()
    engine.submit('r1', STOP_GRAPH().as_dict())
    skills.on_execute = lambda node, context: state.update(envelope=envelope(2))
    engine.run_accepted('r1')
    assert engine.get_status('r1').state == 'failed'
    assert 'changed externally' in engine.get_status('r1').reason

    engine, skills, events, state, _ = make_engine()
    engine.submit('r2', GRAB_GRAPH().as_dict())

    def own_change(node, context):
        if node.id == 'g':
            context.mounted_tool = 'vacuum'
            state.update(envelope=envelope(2))

    skills.on_execute = own_change
    engine.run_accepted('r2')
    status = engine.get_status('r2')
    assert status.state == 'completed'
    assert status.mounted_tool == 'vacuum'


def test_changed_scene_before_the_first_motion_stops_the_run():
    engine, skills, events, state, _ = make_engine()
    accepted = engine.submit('r1', STOP_GRAPH().as_dict())
    assert accepted.accepted
    state['envelope'] = envelope(2)
    engine.run_accepted('r1')
    status = engine.get_status('r1')
    assert status.state == 'failed'
    assert 'before the first motion' in status.reason
    assert skills.calls == []
    assert events['stops']


def test_step_limit_stops_a_recovering_cycle():
    engine, skills, _, _, _ = make_engine(max_steps=3)
    cycle = graph(
        'a',
        node('a', 'wait_scene_clear', 'workspace',
             next_on={'clear': 'b'}),
        node('b', 'wait_scene_clear', 'workspace',
             next_on={'clear': 'a'}),
    )
    engine.submit('r1', cycle.as_dict())
    skills.events = {'a': 'clear', 'b': 'clear'}
    engine.run_accepted('r1')
    status = engine.get_status('r1')
    assert status.state == 'failed'
    assert 'step limit' in status.reason


def test_cancellation_stops_the_active_graph_and_preserves_ownership():
    engine, skills, events, _, _ = make_engine(threaded=True)
    skills.block = True
    skills.on_execute = (
        lambda node, context: setattr(context, 'mounted_tool', node.object)
        if node.skill == 'grab' else None
    )
    engine.submit('r1', GRAB_GRAPH().as_dict())
    assert skills.started.wait(timeout=2.0)
    result = engine.cancel('r1')
    assert result.accepted
    assert events['stops']
    skills.release.set()
    assert engine.wait('r1', timeout=2.0)
    status = engine.get_status('r1')
    assert status.state == 'canceled'
    assert status.mounted_tool == 'vacuum'


def test_cancel_rejects_unknown_or_finished_runs():
    engine, _, _, _, _ = make_engine()
    assert engine.cancel('missing').accepted is False
    engine.submit('r1', STOP_GRAPH().as_dict())
    engine.run_accepted('r1')
    assert engine.cancel('r1').accepted is False


def test_status_query_marks_unknown_ids():
    engine, _, _, _, _ = make_engine()
    status = engine.get_status('nope')
    assert status.found is False
    assert status.state == 'unknown'


def test_not_ready_or_unavailable_scene_is_rejected_without_registration():
    engine, _, _, state, _ = make_engine()
    state['envelope'] = None
    rejected = engine.submit('r1', STOP_GRAPH().as_dict())
    assert rejected.code == 'not_ready'
    assert engine.get_status('r1').found is False


def test_stale_scene_timestamp_is_rejected_before_execution():
    engine, _, _, state, _ = make_engine(clock=10.0)
    state['envelope'] = envelope(1, stamp=1.0)
    rejected = engine.submit('r1', STOP_GRAPH().as_dict())
    assert rejected.code == 'stale_scene'
    assert engine.get_status('r1').found is False
