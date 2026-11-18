"""Integration tests with a complete graph-v3 cap-install response."""
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
        GEOMETRY,
        stamp=5.0,
    )
    return build_envelope(
        scene,
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
            scene_revision=int(env['state_version']),
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


def test_graph_v3_cap_install_is_admitted_and_validated():
    scene_envelope = envelope(mounted=None)
    result = validate_graph(FULL_GRAPH, scene_envelope, DEFAULT_CATALOG)
    assert result.ok is True, result.reason
    assert FULL_GRAPH.entry == 'a_detect_cap'
    assert FULL_GRAPH.goals[0].canonical() == (
        'installed(destination=hatch_seat,part=cap)'
    )
    observe = FULL_GRAPH.node('a_observe_installed')
    assert observe.kind == 'observe'
    assert observe.transition('unknown') == 'operator_required'


def test_full_graph_without_vacuum_follows_the_runtime_grasp_failure():
    engine, motion, supervisor, _ = make_engine(mounted=None)
    accepted = engine.submit('full-empty', FULL_GRAPH.as_dict(), '1')
    assert accepted.accepted is True
    engine.run_accepted('full-empty')
    status = engine.get_status('full-empty')
    assert status.state == 'operator_required'
    assert [event['node_id'] for event in status.events] == [
        'a_detect_cap', 'a_grab_cap',
    ]
    grab = status.events[1]
    assert grab['event'] == 'grasp_failed'
    assert grab['reason'] == 'required tool vacuum is not mounted'
    assert grab['next'] == 'operator_required'
    assert status.mounted_tool is None
    assert status.held_object is None
    assert supervisor.state['cap']['state'] == 'free'
    assert supervisor.state['held_object'] is None
    assert supervisor.commands == []
    assert not any(phase in ('lower', 'contact') for _, phase, _ in motion.calls)


def test_full_graph_with_vacuum_is_still_goal_checked():
    scene_envelope = envelope(mounted='vacuum')
    result = validate_graph(FULL_GRAPH, scene_envelope, DEFAULT_CATALOG)
    assert result.ok is True, result.reason
    engine, _, _, _ = make_engine(mounted='vacuum')
    accepted = engine.submit('full-vacuum', FULL_GRAPH.as_dict(), '1')
    assert accepted.accepted is True
