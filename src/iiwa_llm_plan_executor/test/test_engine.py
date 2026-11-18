"""Graph-v3 engine: admission, goals, retries, terminals and cancellation."""
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


def skill_node(
    node_id,
    skill,
    *,
    obj='',
    target='',
    next_id='completed',
    on=None,
    max_retries=None,
):
    payload = {
        'id': node_id,
        'kind': 'skill',
        'skill': skill,
        'object': obj,
        'target': target,
        'next': next_id,
        'on': on or {},
    }
    if max_retries is not None:
        payload['max_retries'] = max_retries
    return payload


def observe_node(node_id, name, args, *, true='completed',
                 false='replan_required', unknown='operator_required'):
    return {
        'id': node_id,
        'kind': 'observe',
        'predicate': {'name': name, 'args': args},
        'on': {'true': true, 'false': false, 'unknown': unknown},
    }


def graph(entry, *nodes, goals=None):
    return parse_response({
        'schema_version': 3,
        'status': 'ready',
        'entry': entry,
        'goals': goals or [{
            'id': 'g1',
            'predicate': 'visible',
            'args': {'object': 'fixture'},
        }],
        'plan': list(nodes),
    })


STOP_GRAPH = lambda: graph(
    'a',
    skill_node('a', 'stop', obj='robot', next_id='b'),
    skill_node('b', 'stop', obj='robot', next_id='completed'),
)

GRAB_GRAPH = lambda: graph(
    'g',
    skill_node('g', 'grab', obj='vacuum', next_id='t'),
    skill_node('t', 'transfer', obj='vacuum', target='work'),
)

BAD_GRAPH = {
    'schema_version': 3,
    'status': 'ready',
    'entry': 'g',
    'goals': [{
        'id': 'g1', 'predicate': 'visible', 'args': {'object': 'fixture'},
    }],
    'plan': [skill_node('g', 'grab', obj='vacuum', next_id='ghost')],
}


def make_engine(skills=None, *, clock=5.0, provider=None, threaded=False,
                max_steps=512, goal_evaluator=None, **kwargs):
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
        goal_evaluator=goal_evaluator,
        threaded=threaded,
        max_steps=max_steps,
        **kwargs,
    )
    engine.update_readiness(scene_ready=True, ready_at=0.0, motion_ready=True)
    return engine, skills, events, state, clock_box


def submit(engine, run_id, value, version='1'):
    return engine.submit(run_id, value.as_dict(), version)


def test_submission_requires_context_version_and_the_start_delay():
    engine, _, _, _, _ = make_engine(clock=2.0)
    rejected = submit(engine, 'r1', STOP_GRAPH())
    assert rejected.code == 'not_ready'
    assert 'start delay' in rejected.message
    assert engine.get_status('r1').found is False

    engine, _, _, _, _ = make_engine(clock=5.0)
    missing = engine.submit('r1', STOP_GRAPH().as_dict())
    assert missing.code == 'invalid'
    assert 'context_state_version' in missing.message

    accepted = submit(engine, 'r1', STOP_GRAPH())
    assert accepted.accepted is True
    engine.run_accepted('r1')


def test_stale_context_version_is_rejected_without_registration():
    engine, _, events, _, _ = make_engine()
    rejected = submit(engine, 'r1', STOP_GRAPH(), version='9')
    assert rejected.accepted is False
    assert rejected.code == 'stale_scene'
    assert rejected.details['state_version'] == 1
    assert engine.get_status('r1').found is False
    assert events['plans'] == []


def test_legacy_format_is_rejected_as_unsupported_contract():
    engine, _, events, _, _ = make_engine()
    rejected = engine.submit(
        'r1',
        {'status': 'ready', 'entry': 'a', 'plan': []},
        '1',
    )
    assert rejected.code == 'unsupported_contract'
    assert engine.get_status('r1').found is False
    assert events['plans'] == []


def test_not_executable_statuses_do_not_register_or_publish():
    engine, skills, events, _, _ = make_engine()
    blocked = engine.submit('r1', {
        'schema_version': 3,
        'status': 'blocked',
        'entry': '',
        'plan': [],
        'reason': 'vacuum is absent',
    }, '1')
    assert blocked.code == 'not_executable'
    assert blocked.details['status'] == 'blocked'
    clarification = engine.submit('r2', {
        'schema_version': 3,
        'status': 'clarification_required',
        'entry': '',
        'plan': [],
        'question': 'Сколько винтов?',
        'missing': ['count'],
    }, '1')
    assert clarification.code == 'not_executable'
    assert clarification.details['missing'] == ['count']
    assert events['plans'] == []
    assert skills.calls == []
    assert engine.get_status('r1').found is False
    assert engine.get_status('r2').found is False


def test_rejected_graphs_are_not_registered_published_or_executed():
    engine, skills, events, _, _ = make_engine()
    rejected = engine.submit('r2', BAD_GRAPH, '1')
    assert rejected.accepted is False
    assert rejected.code == 'invalid'
    assert events['plans'] == []
    assert skills.calls == []
    assert engine.get_status('r2').found is False


def test_rejected_run_id_can_be_reused_with_a_corrected_graph():
    engine, skills, events, _, _ = make_engine()
    rejected = engine.submit('same-id', BAD_GRAPH, '1')
    assert rejected.accepted is False
    assert engine.get_status('same-id').found is False

    accepted = submit(engine, 'same-id', STOP_GRAPH())
    assert accepted.accepted is True
    assert len(events['plans']) == 1
    engine.run_accepted('same-id')
    assert engine.get_status('same-id').state == 'completed'


def test_accepted_graph_is_published_once_before_the_first_motion():
    engine, skills, events, _, _ = make_engine()
    order = []

    def on_plan(payload):
        events['plans'].append(payload)
        order.append(('plan', payload['run_id']))

    engine.on_plan = on_plan
    skills.on_execute = lambda node, context: order.append(('move', node.id))
    accepted = submit(engine, 'r1', STOP_GRAPH())
    assert accepted.accepted
    engine.run_accepted('r1')
    assert order[0] == ('plan', 'r1')
    assert order[1] == ('move', 'a')
    assert len(events['plans']) == 1


def test_skill_failure_follows_the_safe_terminal():
    engine, skills, _, _, _ = make_engine()
    value = graph(
        'g',
        skill_node('g', 'grab', obj='cap', next_id='completed'),
        goals=[{
            'id': 'g', 'predicate': 'installed',
            'args': {'part': 'cap', 'destination': 'hatch_seat'},
        }],
    )
    accepted = submit(engine, 'r1', value)
    assert accepted.accepted is True
    skills.events = {'g': 'grasp_failed'}
    skills.details = {'g': {'reason': 'required tool vacuum is not mounted'}}
    engine.run_accepted('r1')
    status = engine.get_status('r1')
    assert status.state == 'operator_required'
    assert status.events[0]['event'] == 'grasp_failed'
    assert status.events[0]['next'] == 'operator_required'
    assert list(status.goals_remaining) == ['installed(destination=hatch_seat,part=cap)']


def test_completed_is_downgraded_when_goals_are_not_confirmed():
    engine, _, _, _, _ = make_engine()
    value = graph(
        'a',
        skill_node('a', 'stop', obj='robot'),
        goals=[{
            'id': 'g', 'predicate': 'installed',
            'args': {'part': 'cap', 'destination': 'hatch_seat'},
        }],
    )
    assert submit(engine, 'r1', value).accepted
    engine.run_accepted('r1')
    status = engine.get_status('r1')
    assert status.state == 'operator_required'
    assert 'goals are not confirmed' in status.reason
    assert list(status.goals_remaining) == ['installed(destination=hatch_seat,part=cap)']


def test_explicit_goal_evaluator_confirms_completion():
    def evaluator(goals, scene):
        del scene
        return [goal.canonical() for goal in goals], []

    engine, _, _, _, _ = make_engine(goal_evaluator=evaluator)
    value = graph(
        'a',
        skill_node('a', 'stop', obj='robot'),
        goals=[{
            'id': 'g', 'predicate': 'installed',
            'args': {'part': 'cap', 'destination': 'hatch_seat'},
        }],
    )
    assert submit(engine, 'r1', value).accepted
    engine.run_accepted('r1')
    assert engine.get_status('r1').state == 'completed'


def test_observe_node_selects_true_branch_and_unknown_stops():
    engine, skills, _, _, _ = make_engine()
    value = graph(
        'check',
        observe_node(
            'check', 'installed',
            {'part': 'cap', 'destination': 'hatch_seat'},
            true='completed', false='replan_required',
            unknown='operator_required',
        ),
        goals=[{
            'id': 'g', 'predicate': 'installed',
            'args': {'part': 'cap', 'destination': 'hatch_seat'},
        }],
    )

    def only_completed(goals, scene):
        del scene
        return [goal.canonical() for goal in goals], []

    engine.goal_evaluator = only_completed
    assert submit(engine, 'r1', value).accepted
    skills.events = {'check': 'true'}
    engine.run_accepted('r1')
    assert engine.get_status('r1').state == 'completed'

    engine, skills, _, _, _ = make_engine()
    engine.goal_evaluator = only_completed
    assert submit(engine, 'r2', value).accepted
    skills.events = {'check': 'unknown'}
    engine.run_accepted('r2')
    assert engine.get_status('r2').state == 'operator_required'


def test_retry_limit_stops_at_operator_required():
    engine, _, _, _, _ = make_engine()
    value = graph(
        'a',
        skill_node('a', 'stop', obj='robot', next_id='a', max_retries=1),
    )
    assert submit(engine, 'r1', value).accepted
    engine.run_accepted('r1')
    status = engine.get_status('r1')
    assert status.state == 'operator_required'
    assert 'retry limit' in status.reason


def test_step_limit_stops_a_bounded_cycle():
    engine, _, _, _, _ = make_engine(max_steps=3)
    value = graph(
        'a',
        skill_node('a', 'stop', obj='robot', next_id='b'),
        skill_node('b', 'stop', obj='robot', next_id='a', max_retries=20),
    )
    assert submit(engine, 'r1', value).accepted
    engine.run_accepted('r1')
    assert 'step limit' in engine.get_status('r1').reason


def test_duplicate_conflict_and_busy_rules():
    engine, skills, _, _, _ = make_engine()
    assert submit(engine, 'r1', STOP_GRAPH()).accepted
    duplicate = submit(engine, 'r1', STOP_GRAPH())
    assert duplicate.accepted is True
    assert duplicate.code == 'duplicate'

    conflict = submit(engine, 'r1', GRAB_GRAPH())
    assert conflict.code == 'conflict'
    busy = submit(engine, 'r2', GRAB_GRAPH())
    assert busy.code == 'busy'

    engine.run_accepted('r1')
    assert engine.get_status('r1').state == 'completed'
    reused = submit(engine, 'r1', GRAB_GRAPH())
    assert reused.code == 'conflict'
    assert skills.calls == ['a', 'b']


def test_cancellation_stops_the_active_graph_and_preserves_state():
    engine, skills, _, _, _ = make_engine(threaded=True)
    skills.block = True
    accepted = submit(engine, 'r1', STOP_GRAPH())
    assert accepted.accepted
    assert skills.started.wait(timeout=2.0)
    result = engine.cancel('r1')
    assert result.accepted is True
    skills.release.set()
    assert engine.wait('r1', 3.0) is True
    status = engine.get_status('r1')
    assert status.state == 'canceled'
    assert engine.cancel('r1').accepted is False
    assert engine.cancel('unknown').accepted is False


def test_not_ready_or_unavailable_scene_is_rejected_without_registration():
    engine, _, events, state, _ = make_engine()
    state['envelope'] = None
    rejected = submit(engine, 'r1', STOP_GRAPH())
    assert rejected.code == 'not_ready'
    assert engine.get_status('r1').found is False

    engine.update_readiness(scene_ready=False, ready_at=0.0, motion_ready=False)
    rejected = submit(engine, 'r2', STOP_GRAPH())
    assert rejected.code == 'not_ready'
    assert events['plans'] == []


def test_stale_scene_timestamp_is_rejected_before_execution():
    engine, _, events, state, clock = make_engine()
    state['envelope'] = envelope(1, stamp=1.0)
    rejected = submit(engine, 'r1', STOP_GRAPH())
    assert rejected.code == 'stale_scene'
    assert engine.get_status('r1').found is False
    assert events['plans'] == []
