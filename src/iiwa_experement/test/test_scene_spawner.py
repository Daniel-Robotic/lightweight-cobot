"""The spawner sends every node once and only signals readiness at the end."""
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory

from iiwa_experement.scenario import ScenarioError, load_scenario
from iiwa_experement.scene_spawner import (
    Readiness,
    ScenePlan,
    SpawnError,
    build_plan,
    execute_plan,
    node_name,
    spawn_nodes,
)
from iiwa_experement.webots_nodes import build_nodes


PACKAGE = Path(__file__).resolve().parents[1]
SCENES = PACKAGE / 'config' / 'scenes.yaml'
ASSET_ROOT = (
    Path(get_package_share_directory('iiwa_description')) / 'objects' / 'grant'
)


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self):
        self.logger = FakeLogger()

    def get_logger(self):
        return self.logger


class FakeFuture:
    def __init__(self, behavior):
        self.behavior = behavior
        self.completed = behavior != 'timeout'

    def done(self):
        return self.completed

    def result(self):
        if isinstance(self.behavior, Exception):
            raise self.behavior
        if self.behavior == 'reject':
            return type('Response', (), {'success': False})()
        return type('Response', (), {'success': True})()


class FakeClient:
    def __init__(self, behaviors=(), available=True):
        self.behaviors = list(behaviors)
        self.available = available
        self.calls = []
        self.readiness_during_calls = []
        self.readiness = None

    def wait_for_service(self, timeout_sec):
        return self.available

    def call_async(self, request):
        self.calls.append(request.data)
        if self.readiness is not None:
            self.readiness_during_calls.append(self.readiness.ready)
        index = len(self.calls) - 1
        behavior = self.behaviors[index] if index < len(self.behaviors) else 'ok'
        return FakeFuture(behavior)


def fake_spin(node, future, timeout_sec):
    return None


def run_plan(nodes, client, readiness=None, **kwargs):
    plan = ScenePlan(
        experiment_id=1,
        spec=load_scenario(1, SCENES),
        nodes=list(nodes),
        readiness=readiness or Readiness(),
    )
    client.readiness = plan.readiness
    execute_plan(
        plan,
        client,
        FakeNode(),
        spin_until_future_complete=fake_spin,
        **kwargs,
    )
    return plan


def test_successful_spawn_sends_build_nodes_in_order_and_signals_at_the_end():
    nodes = build_nodes(load_scenario(6, SCENES), ASSET_ROOT)
    client = FakeClient()
    plan = run_plan(nodes, client)
    assert client.calls == nodes
    assert client.readiness_during_calls == [False] * len(nodes)
    assert plan.readiness.ready is True
    assert plan.readiness.message == f'experiment 1: {len(nodes)} objects spawned'


def test_node_name_is_used_for_progress_and_errors():
    assert node_name('Solid {\n  name "grant_cap"\n}') == 'grant_cap'
    assert node_name('Solid {}') == 'unnamed object'


def test_missing_supervisor_fails_without_spawning():
    client = FakeClient(available=False)
    with pytest.raises(SpawnError, match='unavailable'):
        run_plan(['Solid { name "grant_cap" }'], client)
    assert client.calls == []


def test_rejected_object_names_the_last_object_and_keeps_readiness_false():
    client = FakeClient(['ok', 'reject'])
    readiness = Readiness()
    with pytest.raises(SpawnError, match='grant_body'):
        run_plan(
            ['Solid { name "grant_fixture" }', 'Solid { name "grant_body" }'],
            client,
            readiness,
        )
    assert len(client.calls) == 2
    assert readiness.ready is False
    assert 'grant_body' in readiness.message


def test_exception_names_the_last_object():
    client = FakeClient(['ok', RuntimeError('service exploded')])
    with pytest.raises(SpawnError, match='grant_body.*service exploded'):
        run_plan(
            ['Solid { name "grant_fixture" }', 'Solid { name "grant_body" }'],
            client,
        )


def test_response_timeout_is_not_retried():
    client = FakeClient(['ok', 'timeout', 'ok'])
    with pytest.raises(SpawnError, match='grant_body.*timed out.*no retry'):
        run_plan(
            ['Solid { name "grant_fixture" }', 'Solid { name "grant_body" }'],
            client,
        )
    assert len(client.calls) == 2


def test_duplicate_name_rejection_is_not_hidden():
    client = FakeClient(['reject'])
    with pytest.raises(SpawnError, match='grant_fixture.*rejected'):
        run_plan(['Solid { name "grant_fixture" }'], client)


def test_build_plan_rejects_an_invalid_id_before_spawning():
    with pytest.raises(ScenarioError):
        build_plan(0, SCENES, ASSET_ROOT)
    with pytest.raises(ScenarioError):
        build_plan(21, SCENES, ASSET_ROOT)


def test_build_plan_uses_the_full_build_nodes_list():
    plan = build_plan(19, SCENES, ASSET_ROOT)
    assert plan.nodes == build_nodes(plan.spec, ASSET_ROOT)
    assert plan.readiness.ready is False
