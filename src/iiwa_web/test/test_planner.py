"""HTTP adapter tests for graph-v3 context, submit, status and cancellation."""
from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from fastapi import APIRouter, FastAPI
from fastapi.testclient import TestClient

from iiwa_experement.geometry import load_geometry
from iiwa_experement.scenario import load_scenario
from iiwa_scene_context.catalog import DEFAULT_CATALOG
from iiwa_scene_context.snapshot import build_envelope, build_scene
from iiwa_web import main, planner


PACKAGE = Path(__file__).resolve().parents[1]
ASSETS = PACKAGE.parent / 'iiwa_experement' / 'config'
SCENES = ASSETS / 'scenes.yaml'
GEOMETRY = load_geometry(ASSETS / 'geometry.yaml')


def envelope(experiment_id=20):
    spec = load_scenario(experiment_id, SCENES)
    tools = {tool.name: {'state': 'docked'} for tool in spec.tools}
    scene = build_scene(
        spec,
        {
            'version': 2,
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
        stamp=10.0,
    )
    return build_envelope(
        scene,
        state_version=2,
        stamp=10.0,
        experiment_id=experiment_id,
    )


def graph_v3():
    return {
        'schema_version': 3,
        'status': 'ready',
        'entry': 'a',
        'goals': [{
            'id': 'g', 'predicate': 'visible',
            'args': {'object': 'fixture'},
        }],
        'plan': [{
            'id': 'a',
            'kind': 'skill',
            'skill': 'stop',
            'object': 'robot',
            'next': 'completed',
        }],
    }


class FakeBridge:
    def __init__(self):
        self.calls = []
        self.submit_accepted = True
        self.submit_code = 'accepted'
        self.submit_reason = 'graph admitted'
        self.submit_details = ''
        self.cancel_success = True
        self.scene_success = True
        self.scene_payload = envelope()
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
                message=(
                    json.dumps(self.scene_payload)
                    if self.scene_success else 'scene not ready'
                ),
            )
        if srv_name == planner.SUBMIT_SERVICE:
            return SimpleNamespace(
                accepted=self.submit_accepted,
                code=self.submit_code,
                reason=self.submit_reason,
                status_json='{}',
                details_json=self.submit_details,
            )
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


def test_context_returns_one_consistent_graph_v3_package(bridge):
    fake, client = bridge
    response = client.get('/planner/context')
    assert response.status_code == 200
    payload = response.json()
    assert payload['schema_version'] == 3
    assert payload['contract_version'] == 3
    assert payload['catalog_version'] == 2
    assert payload['state_version'] == 2
    assert payload['scene']['schema_version'] == 1
    assert payload['catalog']['skills']
    assert payload['response_schema']['$defs']['ReadyResponse']
    assert 'graph-v3' in payload['system_prompt']
    assert fake.calls[0][0] == planner.SCENE_SERVICE


def test_context_rejects_unready_or_unsupported_scene(bridge):
    fake, client = bridge
    fake.scene_success = False
    response = client.get('/planner/context')
    assert response.status_code == 409
    assert response.json()['detail']['code'] == 'scene_not_ready'

    fake.scene_success = True
    fake.scene_payload = envelope()
    fake.scene_payload['contract_version'] = 1
    response = client.get('/planner/context')
    assert response.status_code == 409
    assert response.json()['detail']['code'] == 'unsupported_contract'


def test_scene_returns_the_envelope_without_scene_revision(bridge):
    fake, client = bridge
    response = client.get('/planner/scene')
    assert response.status_code == 200
    assert response.json()['state_version'] == 2
    assert 'scene_revision' not in response.json()
    assert fake.calls[0][0] == planner.SCENE_SERVICE


def test_scene_not_ready_and_service_failure(bridge):
    fake, client = bridge
    fake.scene_success = False
    assert client.get('/planner/scene').status_code == 409
    fake.raise_on.add(planner.SCENE_SERVICE)
    assert client.get('/planner/scene').status_code == 503


def test_submit_forwards_graph_v3_and_context_state_version(bridge):
    fake, client = bridge
    response = client.post('/planner/runs', json={
        'run_id': 'r1',
        'context_state_version': '2',
        'response': graph_v3(),
    })
    assert response.status_code == 200
    assert response.json()['status'] == 'accepted'
    service, request = fake.calls[0]
    assert service == planner.SUBMIT_SERVICE
    assert request.run_id == 'r1'
    assert request.context_state_version == '2'
    assert json.loads(request.response)['schema_version'] == 3
    assert planner._active_run_id == 'r1'


def test_submit_requires_context_state_version(bridge):
    fake, client = bridge
    response = client.post('/planner/runs', json={
        'run_id': 'r1',
        'response': graph_v3(),
    })
    assert response.status_code == 422
    assert fake.calls == []


def test_duplicate_submit_does_not_start_a_second_motion(bridge):
    fake, client = bridge
    fake.submit_code = 'duplicate'
    response = client.post('/planner/runs', json={
        'run_id': 'r1',
        'context_state_version': '2',
        'response': graph_v3(),
    })
    assert response.status_code == 200
    assert response.json()['status'] == 'duplicate'


@pytest.mark.parametrize('code,http_status', [
    ('busy', 409),
    ('conflict', 409),
    ('stale_scene', 409),
    ('not_ready', 503),
    ('invalid', 422),
    ('unsupported_contract', 422),
    ('not_executable', 422),
    ('rejected', 422),
    ('replan_required', 422),
])
def test_submit_errors_include_the_exact_code_and_reason(
    bridge, code, http_status,
):
    fake, client = bridge
    fake.submit_accepted = False
    fake.submit_code = code
    fake.submit_reason = f'{code} because of a concrete reason'
    fake.submit_details = json.dumps({
        'status': 'blocked',
        'question': 'Сколько винтов?',
        'missing': ['count'],
    })
    response = client.post('/planner/runs', json={
        'run_id': 'r1',
        'context_state_version': '2',
        'response': graph_v3(),
    })
    assert response.status_code == http_status
    detail = response.json()['detail']
    assert detail['code'] == code
    assert detail['reason'] == f'{code} because of a concrete reason'
    assert detail['run_id'] == 'r1'
    assert detail['missing'] == ['count']


def test_submit_service_failure_is_503(bridge):
    fake, client = bridge
    fake.raise_on.add(planner.SUBMIT_SERVICE)
    assert client.post('/planner/runs', json={
        'run_id': 'r3',
        'context_state_version': '2',
        'response': graph_v3(),
    }).status_code == 503


def test_status_returns_404_for_unknown_run(bridge):
    fake, client = bridge
    response = client.get('/planner/runs/r1')
    assert response.status_code == 200
    assert response.json()['state'] == 'running'
    assert 'scene_revision' not in response.json()
    fake.status_payload = None
    assert client.get('/planner/runs/nope').status_code == 404


def test_status_clears_the_active_run_on_terminal(bridge):
    fake, client = bridge
    planner._active_run_id = 'r1'
    fake.status_payload = {
        'found': True, 'run_id': 'r1', 'state': 'completed',
        'reason': 'done', 'goals_confirmed': ['visible(object=fixture)'],
        'goals_remaining': [],
    }
    body = client.get('/planner/runs/r1').json()
    assert body['state'] == 'completed'
    assert body['goals_remaining'] == []
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
        '/planner/context', '/planner/scene', '/planner/runs',
        '/planner/runs/{run_id}', '/planner/runs/{run_id}/cancel', '/stop',
    } <= paths

    mcp = FastMCP.from_fastapi(app=schema)
    tools = asyncio.run(mcp.list_tools())
    names = {tool.name for tool in tools}
    assert 'get_context_planner_context_get' in names
    assert 'get_scene_planner_scene_get' in names
    assert 'submit_run_planner_runs_post' in names
    assert 'get_run_planner_runs' in names
    assert 'cancel_run_planner_runs' in names

    app = FastAPI()
    main.add_application_routes(app, None, None)
    served = {route.path for route in app.routes}
    assert paths <= served
