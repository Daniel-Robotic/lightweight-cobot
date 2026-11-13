"""Readiness, freshness and internal scene version rules of the snapshot node."""
from __future__ import annotations

from pathlib import Path

from std_srvs.srv import Trigger

from iiwa_experement.geometry import load_geometry
from iiwa_experement.scenario import load_scenario
from iiwa_scene_context.node import SceneStore, SceneContextNode, scene_qos
from iiwa_scene_context.snapshot import build_scene


PACKAGE = Path(__file__).resolve().parents[1]
SCENES = PACKAGE.parent / 'iiwa_experement' / 'config' / 'scenes.yaml'
GEOMETRY = load_geometry(PACKAGE.parent / 'iiwa_experement' / 'config' / 'geometry.yaml')


def state(version: int, *, mounted=None, held=None):
    return {
        'version': version,
        'mounted_tool': mounted,
        'held_object': held,
        'ready': True,
        'cap': {'state': 'free'},
        'tools': {'vacuum': {'state': 'docked'}},
        'screws': {},
        'holes': {},
    }


def make_store():
    spec = load_scenario(1, SCENES)
    return SceneStore(spec, freshness_sec=2.0, geometry=GEOMETRY)


def test_snapshot_requires_scene_ready_and_fresh_state():
    store = make_store()
    store.update(state(1), received_at=10.0)
    envelope, reason = store.snapshot(10.1)
    assert envelope is None
    assert 'not ready' in reason

    store.set_scene_ready(True, 'scene 1: 9 objects spawned')
    envelope, reason = store.snapshot(10.1)
    assert reason is None
    assert envelope['scene_json']['experiment_id'] == 1

    envelope, reason = store.snapshot(13.0)
    assert envelope is None
    assert 'stale' in reason


def test_internal_version_advances_on_state_version_but_not_on_pose_updates():
    store = make_store()
    store.set_scene_ready(True)
    store.update(state(1), received_at=10.0)
    assert store.scene_revision == 1
    assert 'scene_revision' not in store.snapshot(10.1)[0]

    # A pose-only publish of the same held object carries the same version.
    store.update(state(1, mounted='vacuum', held='cap'), received_at=10.5)
    assert store.scene_revision == 1

    # A confirmed ownership change increments the supervisor version.
    store.update(state(2, mounted='vacuum', held='cap'), received_at=11.0)
    assert store.scene_revision == 2

    store.update(state(3, mounted='vacuum', held=None), received_at=11.5)
    assert store.scene_revision == 3


def test_late_subscriber_qos_is_transient_local_and_depth_one():
    qos = scene_qos()
    assert qos.durability.name == 'TRANSIENT_LOCAL'
    assert qos.depth == 1
    assert qos.reliability.name == 'RELIABLE'


def test_scene_service_returns_json_or_a_reason():
    store = make_store()
    node = object.__new__(SceneContextNode)
    node.store = store
    node._now = lambda: 10.1

    response = Trigger.Response()
    result = SceneContextNode._handle_get_scene(node, None, response)
    assert result.success is False
    assert 'not ready' in result.message

    store.set_scene_ready(True)
    store.update(state(1), received_at=10.0)
    response = Trigger.Response()
    result = SceneContextNode._handle_get_scene(node, None, response)
    assert result.success is True
    assert '"scene_json"' in result.message
    assert '"scene_revision"' not in result.message
    assert '"pose_conventions"' in result.message


def test_snapshot_scene_is_the_catalog_scene_not_the_state():
    store = make_store()
    store.set_scene_ready(True)
    store.update(state(1), received_at=10.0)
    envelope, _ = store.snapshot(10.1)
    expected = build_scene(
        store.spec, store.state, store.catalog, store.geometry,
        stamp=store.received_at,
    )
    assert envelope['scene_json'] == expected
    assert 'objects' not in envelope['scene_json']


def test_scene_topic_is_republished_to_keep_the_timestamp_fresh():
    store = make_store()
    store.set_scene_ready(True)
    store.update(state(1), received_at=10.0)
    node = object.__new__(SceneContextNode)
    node.store = store
    node._last_published = None
    node._last_publish_time = -1.0
    node._publisher = type('Publisher', (), {
        'messages': [],
        'publish': lambda self, message: self.messages.append(message),
    })()
    clock = {'now': 10.1}
    node._now = lambda: clock['now']
    node.get_parameter = lambda name: type('P', (), {
        'value': 0.5 if name == 'publish_period_sec' else None,
    })()
    SceneContextNode._publish_if_new(node)
    SceneContextNode._publish_if_new(node)
    assert len(node._publisher.messages) == 1
    clock['now'] = 10.7
    SceneContextNode._publish_if_new(node)
    assert len(node._publisher.messages) == 2
