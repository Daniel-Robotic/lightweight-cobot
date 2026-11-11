"""Integration tests with the complete Qwen graph submitted by the user."""
from __future__ import annotations

import copy
import json
from pathlib import Path

from iiwa_experement.geometry import load_geometry, world_target
from iiwa_experement.object_state import transform_json
from iiwa_experement.scenario import load_scenario
from iiwa_llm_plan_executor.engine import GraphEngine
from iiwa_llm_plan_executor.moveit_backend import MotionOutcome
from iiwa_llm_plan_executor.schema import parse_response
from iiwa_llm_plan_executor.skill_context import SkillContext
from iiwa_llm_plan_executor.skills import SkillRegistry
from iiwa_llm_plan_executor.validator import validate_graph
from iiwa_scene_context.catalog import DEFAULT_CATALOG
from iiwa_scene_context.snapshot import build_envelope, build_scene


PACKAGE = Path(__file__).resolve().parents[1]
ASSETS = PACKAGE.parent / 'iiwa_experement' / 'config'
SCENES = ASSETS / 'scenes.yaml'
GRAPH_PATH = Path(__file__).resolve().parent / 'data' / 'qwen_cap_full_graph.json'
GEOMETRY = load_geometry(ASSETS / 'geometry.yaml')
SPEC = load_scenario(20, SCENES)
FULL_GRAPH = parse_response(json.loads(GRAPH_PATH.read_text(encoding='utf-8')))


class FakeMotion:
    def __init__(self):
        self.calls = []
        self.stop_calls = []

    def plan_and_execute(self, goal, phase, held_object):
        self.calls.append((goal, phase, held_object))
        return MotionOutcome(True, 'ok', 'ok')

    def stop(self, reason='stop'):
        self.stop_calls.append(reason)


class FakeSupervisor:
    def __init__(self):
        self.commands = []
        self.state = {
            'version': 1,
            'mounted_tool': None,
            'held_object': None,
            'cap': {
                'state': 'free',
                'pose': transform_json(world_target(SPEC, GEOMETRY, 'cap')),
            },
            'tools': {
                tool.name: {'state': 'docked'}
                for tool in SPEC.tools
            },
            'screws': {},
            'holes': {},
            'objects': {},
        }

    def latest(self):
        return copy.deepcopy(self.state)

    def command(self, *args, **kwargs):
        self.commands.append((args, kwargs))
        raise AssertionError(
            'no supervisor command is allowed on the missing-tool path'
        )


def envelope(*, mounted: str | None = None):
    tools = {tool.name: {'state': 'docked'} for tool in SPEC.tools}
    if mounted is not None:
        tools[mounted] = {'state': 'mounted'}
    scene = build_scene(
        SPEC,
        {
            'version': 1,
            'mounted_tool': mounted,
            'held_object': None,
            'ready': True,
            'cap': {'state': 'free'},
            'tools': tools,
            'screws': {},
            'holes': {},
        },
        DEFAULT_CATALOG,
    )
    return build_envelope(
        scene,
        scene_revision=1,
        state_version=1,
        stamp=5.0,
        experiment_id=20,
    )


def make_engine(*, mounted: str | None = None):
    scene_envelope = envelope(mounted=mounted)
    motion = FakeMotion()
    supervisor = FakeSupervisor()
    registry = SkillRegistry(motion, supervisor)

    def context_factory(env, scene_json):
        del scene_json
        context = SkillContext(
            spec=SPEC,
            geometry=GEOMETRY,
            scene_revision=int(env['scene_revision']),
        )
        context.frame_provider = lambda name: 5.0
        context.notifier = lambda reason, details: True
        return context

    engine = GraphEngine(
        lambda: scene_envelope,
        context_factory,
        registry,
        clock=lambda: 5.0,
        threaded=False,
        min_start_delay_sec=3.0,
    )
    engine.update_readiness(scene_ready=True, ready_at=0.0, motion_ready=True)
    return engine, motion, supervisor, scene_envelope


def test_full_qwen_graph_empty_patron_follows_actual_failure_path():
    engine, motion, supervisor, scene_envelope = make_engine(mounted=None)
    result = validate_graph(
        FULL_GRAPH,
        scene_envelope,
        DEFAULT_CATALOG,
        expected_scene_revision=1,
    )
    assert result.ok is True, result.reason

    accepted = engine.submit('full-empty', FULL_GRAPH.as_dict(), 1)
    assert accepted.accepted is True
    engine.run_accepted('full-empty')
    status = engine.get_status('full-empty')
    assert status.state == 'operator_required'
    assert [event['node_id'] for event in status.events] == [
        'a_find_part',
        'a_find_target',
        'a_approach',
        'a_pick',
        'a_stop_recover',
        'a_state',
        'a_retry_find_part',
        'a_retry_approach',
        'a_retry_pick',
        'stop_operator',
        'notify',
    ]
    pick_events = [
        event for event in status.events if event['node_id'] == 'a_pick'
    ]
    state_events = [
        event for event in status.events if event['node_id'] == 'a_state'
    ]
    retry_pick = [
        event for event in status.events if event['node_id'] == 'a_retry_pick'
    ]
    assert pick_events[0]['event'] == 'grasp_failed'
    assert pick_events[0]['reason'] == 'required tool vacuum is not mounted'
    assert pick_events[0]['next'] == 'a_stop_recover'
    assert state_events[0]['event'] == 'empty'
    assert state_events[0]['next'] == 'a_retry_find_part'
    assert retry_pick[0]['event'] == 'grasp_failed'
    assert retry_pick[0]['next'] == 'stop_operator'
    assert status.mounted_tool is None
    assert status.held_object is None
    assert supervisor.state['cap']['state'] == 'free'
    assert supervisor.state['held_object'] is None
    assert supervisor.commands == []
    assert not any(phase in ('lower', 'contact') for _, phase, _ in motion.calls)


def test_full_qwen_graph_with_vacuum_has_a_success_cycle_without_completed():
    # The nominal successful branch is inspect ok -> state -> placed -> inspect.
    a_inspect = FULL_GRAPH.node('a_inspect')
    a_state = FULL_GRAPH.node('a_state')
    assert {
        transition.event: transition.next
        for transition in a_inspect.transitions
    }['ok'] == 'a_state'
    assert {
        transition.event: transition.next
        for transition in a_state.transitions
    }['placed'] == 'a_inspect'

    all_nexts = {
        transition.next
        for node in FULL_GRAPH.nodes
        for transition in node.transitions
    }
    assert 'completed' not in all_nexts

    # With the vacuum mounted the graph is still not executable as a whole:
    # the generic grasp_failed branch leads through inspect_state holding to a
    # retry pick while the hatch is held, which the ownership check rejects.
    result = validate_graph(
        FULL_GRAPH,
        envelope(mounted='vacuum'),
        DEFAULT_CATALOG,
        expected_scene_revision=1,
    )
    assert result.ok is False
    assert 'a_retry_pick' in result.reason
    assert 'holding' in result.reason
