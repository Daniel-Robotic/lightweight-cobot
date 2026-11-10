"""HTTP adapter tests for scene, submit, status and cancellation."""
from __future__ import annotations

import json
from types import SimpleNamespace

import pytest
from fastapi import APIRouter, FastAPI
from fastapi.testclient import TestClient

from iiwa_web import main, planner


def envelope():
    return {
        'experiment_id': 20,
        'scene_revision': 4,
        'state_version': 2,
        'timestamp': 10.0,
        'scene_json': {'experiment_id': 20, 'components': []},
    }


class FakeBridge:
    def __init__(self):
        self.calls = []
        self.submit_success = True
        self.cancel_success = True
        self.scene_success = True
        self.status_payload = {
            'found': True,
            'run_id': 'r1',
            'state': 'running',
            'reason': '',
        }
        self.raise_on = set()

    def call_service(self, srv_type, srv_name, request):
        self.calls.append((srv_name, request))
        if srv_name in self.raise_on:
            raise RuntimeError(f"{srv_name} unavailable")
        if srv_name == planner.SCENE_SERVICE:
            return SimpleNamespace(
                success=self.scene_success,
                message=json.dumps(envelope()) if self.scene_success else 'scene not ready',
            )
        if srv_name == planner.SUBMIT_SERVICE:
            return SimpleNamespace(success=self.submit_success)
        if srv_name == planner.CANCEL_SERVICE:
            return SimpleNamespace(success=self.cancel_success)
        if srv_name == planner.STATUS_SERVICE:
            if self.status_payload is None:
                return SimpleNamespace(found=False, status_json='{}')
            return SimpleNamespace(
                found=bool(self.status_payload.get('found', True)),
                status_json=json.dumps(self.status_payload),
            )
        raise AssertionError(srv_name)


@pytest.fixture
def bridge(monkeypatch):
    fake = FakeBridge()
    monkeypatch.setattr(planner, 'get_bridge', lambda: fake)
    planner._active_run_id = None
    app = FastAPI()
    app.include_router(planner.router)
    return fake, TestClient(app)


def test_scene_returns_the_envelope(bridge):
    fake, client = bridge
    response = client.get('/planner/scene')
    assert response.status_code == 200
    assert response.json()['scene_revision'] == 4
    assert fake.calls[0][0] == planner.SCENE_SERVICE


def test_scene_not_ready_and_service_failure(bridge):
    fake, client = bridge
    fake.scene_success = False
    assert client.get('/planner/scene').status_code == 409
    fake.raise_on.add(planner.SCENE_SERVICE)
    assert client.get('/planner/scene').status_code == 503


def test_submit_accepts_without_waiting_for_completion(bridge):
    fake, client = bridge
    response = client.post('/planner/runs', json={
        'run_id': 'r1',
        'scene_revision': 4,
        'response': {
            'status': 'ready',
            'entry': 'a',
            'plan': [],
        },
    })
    assert response.status_code == 200
    assert response.json()['status'] == 'accepted'
    service, request = fake.calls[0]
    assert service == planner.SUBMIT_SERVICE
    payload = json.loads(request.value)
    assert payload['run_id'] == 'r1'
    assert payload['scene_revision'] == 4
    assert payload['response']['entry'] == 'a'
    assert planner._active_run_id == 'r1'


def test_submit_conflict_and_not_ready_mapping(bridge):
    fake, client = bridge
    fake.submit_success = False
    fake.status_payload = {
        'found': True, 'run_id': 'r1', 'state': 'rejected',
        'reason': 'busy: another run is active',
    }
    response = client.post('/planner/runs', json={
        'run_id': 'r1', 'scene_revision': 4,
        'response': {'status': 'blocked', 'entry': '', 'plan': []},
    })
    assert response.status_code == 409
    assert 'busy' in response.json()['detail']

    fake.status_payload['reason'] = 'not_ready: scene is not ready'
    response = client.post('/planner/runs', json={
        'run_id': 'r2', 'scene_revision': 4,
        'response': {'status': 'blocked', 'entry': '', 'plan': []},
    })
    assert response.status_code == 503

    fake.raise_on.add(planner.SUBMIT_SERVICE)
    assert client.post('/planner/runs', json={
        'run_id': 'r3', 'scene_revision': 4,
        'response': {'status': 'blocked', 'entry': '', 'plan': []},
    }).status_code == 503


def test_status_returns_404_for_unknown_run(bridge):
    fake, client = bridge
    response = client.get('/planner/runs/r1')
    assert response.status_code == 200
    assert response.json()['state'] == 'running'
    fake.status_payload = None
    assert client.get('/planner/runs/nope').status_code == 404


def test_status_clears_the_active_run_on_terminal(bridge):
    fake, client = bridge
    planner._active_run_id = 'r1'
    fake.status_payload = {
        'found': True, 'run_id': 'r1', 'state': 'completed',
        'reason': 'done',
    }
    assert client.get('/planner/runs/r1').json()['state'] == 'completed'
    assert planner._active_run_id is None


def test_cancel_and_unknown_cancel(bridge):
    fake, client = bridge
    response = client.post('/planner/runs/r1/cancel')
    assert response.status_code == 200
    service, request = fake.calls[0]
    assert service == planner.CANCEL_SERVICE
    assert json.loads(request.value)['run_id'] == 'r1'

    fake.cancel_success = False
    fake.status_payload = None
    assert client.post('/planner/runs/nope/cancel').status_code == 404


def test_stop_cancels_the_active_graph_before_the_robot_stop(monkeypatch):
    order = []
    fake = FakeBridge()

    def call_service(srv_type, srv_name, request):
        order.append(srv_name)
        return SimpleNamespace(success=True, message='stopped')

    fake.call_service = call_service
    monkeypatch.setattr(main, 'get_bridge', lambda: fake)
    monkeypatch.setattr(main.planner, 'cancel_active_run', lambda: order.append('cancel_graph'))
    monkeypatch.setattr(main.runner, 'stop_if_running', lambda: order.append('runner'))
    monkeypatch.setattr(main.trajectory, 'send_stop_trajectory', lambda: order.append('trajectory'))
    result = main.stop_all()
    assert result['status'] == 'stopped'
    assert order == ['cancel_graph', 'runner', 'trajectory', 'cobot/stop']


def test_schema_app_and_serving_app_share_the_planner_router(monkeypatch):
    import asyncio
    from fastmcp import FastMCP

    monkeypatch.setattr(main, 'build_dynamic_router', lambda *args: APIRouter())
    schema = main.build_schema_app(None, None)
    paths = {route.path for route in schema.routes}
    assert {
        '/planner/scene', '/planner/runs', '/planner/runs/{run_id}',
        '/planner/runs/{run_id}/cancel', '/stop',
    } <= paths

    mcp = FastMCP.from_fastapi(app=schema)
    tools = asyncio.run(mcp.list_tools())
    names = {tool.name for tool in tools}
    assert 'get_scene_planner_scene_get' in names
    assert 'submit_run_planner_runs_post' in names
    assert 'get_run_planner_runs' in names
    assert 'cancel_run_planner_runs' in names

    app = FastAPI()
    main.add_application_routes(app, None, None)
    served = {route.path for route in app.routes}
    assert paths <= served
