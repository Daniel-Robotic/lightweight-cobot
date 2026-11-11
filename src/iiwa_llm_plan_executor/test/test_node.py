"""ROS wrapper helpers: request parsing, plan/status publication and logs."""
from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from iiwa_experement.geometry import load_geometry
from iiwa_experement.scenario import load_scenario
from iiwa_llm_plan_executor.engine import AdmissionResult, RunStatus
from iiwa_llm_plan_executor.node import (
    PlannerExecutorNode,
    RunLog,
    _safe_name,
    plan_qos,
)
from iiwa_msgs.srv import GetRunStatus


PACKAGE = Path(__file__).resolve().parents[1]
ASSETS = PACKAGE.parent / 'iiwa_experement' / 'config'


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


class FakeEngine:
    def __init__(self, admission=None, status=None, cancel=None):
        self.admission = admission or AdmissionResult(True, 'accepted', 'ok', 'r1')
        self.status = status
        self.cancel_result = cancel
        self.submitted = []
        self.cancelled = []

    def submit(self, run_id, graph, revision):
        self.submitted.append((run_id, graph, revision))
        return self.admission

    def get_status(self, run_id):
        return self.status or RunStatus(
            True, run_id, 'completed', 'done', 'a', 'stop', 'success',
            'completed', 1, 'completed', 'vacuum', None, (),
        )

    def cancel(self, run_id):
        self.cancelled.append(run_id)
        return self.cancel_result or SimpleNamespace(
            accepted=True, message='cancel requested',
        )


def fake_node(*, engine=None):
    node = object.__new__(PlannerExecutorNode)
    node.engine = engine or FakeEngine()
    node._plan_publisher = FakePublisher()
    node._status_publisher = FakePublisher()
    node._last_plan = None
    node._last_status = None
    node.log = RunLog('/tmp/iiwa-plan-log-test')
    node.get_logger = lambda: SimpleNamespace(
        info=lambda *a, **k: None,
        warning=lambda *a, **k: None,
        error=lambda *a, **k: None,
    )
    node._now = lambda: 10.0
    node._lock = __import__('threading').RLock()
    return node


def test_plan_qos_is_transient_local_depth_one():
    qos = plan_qos()
    assert qos.durability.name == 'TRANSIENT_LOCAL'
    assert qos.depth == 1


def test_stationary_wait_rejects_a_robot_that_keeps_moving():
    node = object.__new__(PlannerExecutorNode)
    node._last_joint_velocity = (0.2,) * 7
    with pytest.raises(RuntimeError, match='did not become stationary'):
        PlannerExecutorNode._wait_stationary(
            node, timeout_sec=0.02, tolerance=0.01,
        )


def test_safe_name_and_run_log_are_machine_readable(tmp_path):
    assert _safe_name('run/one 1') == 'run_one_1'
    log = RunLog(str(tmp_path))
    log.write({'run_id': 'r1', 'event': 'plan_accepted', 'scene_revision': 1})
    log.write({'run_id': 'r1', 'event': 'completed', 'terminal': 'completed'})
    lines = (tmp_path / 'r1.jsonl').read_text().splitlines()
    assert [json.loads(line)['event'] for line in lines] == [
        'plan_accepted', 'completed',
    ]


def test_plan_is_published_once_and_kept_for_late_subscribers():
    node = fake_node()
    payload = {'run_id': 'r1', 'scene_revision': 2, 'response': '{}'}
    PlannerExecutorNode._publish_plan(node, payload)
    PlannerExecutorNode._publish_plan(node, payload)
    assert len(node._plan_publisher.messages) == 2
    assert node._last_plan == payload
    assert json.loads(node._plan_publisher.messages[-1])['run_id'] == 'r1'


def test_submit_parses_json_and_forwards_the_untouched_response():
    node = fake_node()
    published = []
    node._publish_status = lambda payload: published.append(payload)
    request = SimpleNamespace(value=json.dumps({
        'run_id': 'r1',
        'scene_revision': 3,
        'response': {'status': 'ready', 'entry': 'a', 'plan': []},
    }))
    response = SimpleNamespace(success=False)
    PlannerExecutorNode._handle_submit(node, request, response)
    assert response.success is True
    assert node.engine.submitted[0][0] == 'r1'
    assert node.engine.submitted[0][1] == {
        'status': 'ready', 'entry': 'a', 'plan': [],
    }
    assert node.engine.submitted[0][2] == 3


def test_rejected_submit_returns_false_and_publishes_the_reason():
    node = fake_node(engine=FakeEngine(
        admission=AdmissionResult(False, 'replan_required', 'need vacuum', 'r1'),
    ))
    published = []
    node._publish_status = lambda payload: published.append(payload)
    request = SimpleNamespace(value=json.dumps({
        'run_id': 'r1', 'scene_revision': 1,
        'response': {'status': 'blocked', 'entry': '', 'plan': []},
    }))
    response = SimpleNamespace(success=True)
    PlannerExecutorNode._handle_submit(node, request, response)
    assert response.success is False
    assert published[-1]['state'] == 'rejected'
    assert 'replan_required' in published[-1]['reason']


def test_replan_scene_status_keeps_the_run_id():
    node = fake_node()
    node._last_status = {'run_id': 'run-42', 'state': 'running'}
    envelope = {'scene_revision': 7, 'scene_json': {'components': []}}
    PlannerExecutorNode._on_replan(node, envelope)
    payload = json.loads(node._status_publisher.messages[-1])
    assert payload['run_id'] == 'run-42'
    assert payload['scene_revision'] == 7
    assert payload['scene'] == envelope


def test_invalid_submit_json_is_rejected_without_calling_the_engine():
    node = fake_node()
    request = SimpleNamespace(value='{broken')
    response = SimpleNamespace(success=True)
    PlannerExecutorNode._handle_submit(node, request, response)
    assert response.success is False
    assert node.engine.submitted == []


def test_status_service_returns_found_and_json():
    node = fake_node()
    request = GetRunStatus.Request(run_id='r1')
    response = GetRunStatus.Response()
    PlannerExecutorNode._handle_get_status(node, request, response)
    assert response.found is True
    payload = json.loads(response.status_json)
    assert payload['run_id'] == 'r1'
    assert payload['state'] == 'completed'
    assert payload['mounted_tool'] == 'vacuum'


def test_cancel_accepts_json_and_plain_run_ids():
    node = fake_node()
    response = SimpleNamespace(success=False)
    PlannerExecutorNode._handle_cancel(
        node, SimpleNamespace(value=json.dumps({'run_id': 'r1'})), response,
    )
    assert response.success is True
    response = SimpleNamespace(success=False)
    PlannerExecutorNode._handle_cancel(
        node, SimpleNamespace(value='r2'), response,
    )
    assert node.engine.cancelled == ['r1', 'r2']


def test_scene_provider_enforces_freshness():
    node = fake_node()
    node._now = lambda: 10.0
    node.scene_max_age = 2.0
    node._latest_scene = {
        'timestamp': 9.0, 'scene_revision': 1, 'scene_json': {},
    }
    assert PlannerExecutorNode._scene_provider(node) is not None
    node._latest_scene['timestamp'] = 7.0
    assert PlannerExecutorNode._scene_provider(node) is None


def test_context_factory_sets_sources_and_inspection_configuration():
    node = fake_node()
    node.spec = load_scenario(20, ASSETS / 'scenes.yaml')
    node.geometry = load_geometry(ASSETS / 'geometry.yaml')
    node.adapter = 'adapter'
    node.close_inspection = frozenset({'cap'})
    node._frame_provider = lambda name: 1.0
    node._notify_operator = lambda reason, details: True
    context = PlannerExecutorNode._context_factory(
        node, {'scene_revision': 4}, {},
    )
    assert context.scene_revision == 4
    assert context.scene_adapter == 'adapter'
    assert context.close_inspection_objects == frozenset({'cap'})
    assert context.capture_frame('top') == 1.0
    assert context.notify('x', {}) is True


def test_future_scene_timestamps_cannot_delay_the_start_gate():
    from std_msgs.msg import String

    node = fake_node()
    node._scene_ready_at = None
    node._latest_scene = None
    message = String()
    message.data = json.dumps({'timestamp': 500.0, 'scene_revision': 1})
    PlannerExecutorNode._on_scene(node, message)
    assert node._scene_ready_at == 10.0
    message.data = json.dumps({'timestamp': 5.0, 'scene_revision': 1})
    PlannerExecutorNode._on_scene(node, message)
    assert node._scene_ready_at == 5.0


def test_robot_freshness_uses_the_joint_limit_age():
    node = fake_node()
    node.limits = SimpleNamespace(joint_state_max_age_sec=1.0)
    node._last_joint = None
    node._last_joint_at = None
    assert PlannerExecutorNode._robot_fresh(node)[0] is False
    node._last_joint = (0.0,) * 7
    node._last_joint_at = 9.5
    assert PlannerExecutorNode._robot_fresh(node)[0] is True
    node._last_joint_at = 8.0
    assert PlannerExecutorNode._robot_fresh(node)[0] is False
