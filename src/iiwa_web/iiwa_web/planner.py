"""Thin HTTP/MCP adapter for the scene snapshot and graph executor services.

This module contains no motion logic and no local scene revision.  It only
forwards JSON to the ROS services using the existing web bridge.
"""
from __future__ import annotations

import json
import threading
from typing import Any

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from std_srvs.srv import Trigger
import webots_ros2_msgs.srv
from iiwa_msgs.srv import GetRunStatus, SubmitPlan
from iiwa_scene_context.external import ContextError, build_planner_context

from .ros_node import get_bridge


SCENE_SERVICE = '/iiwa_scene_context/get_scene'
SUBMIT_SERVICE = '/iiwa_llm_plan_executor/submit_plan'
LEGACY_SUBMIT_SERVICE = '/iiwa_llm_plan_executor/submit'
CANCEL_SERVICE = '/iiwa_llm_plan_executor/cancel'
STATUS_SERVICE = '/iiwa_llm_plan_executor/get_status'

router = APIRouter(prefix='/planner', tags=['planner'])
_lock = threading.Lock()
_active_run_id: str | None = None


class RunRequest(BaseModel):
    run_id: str = Field(..., min_length=1, max_length=128)
    context_state_version: str = Field(
        ...,
        min_length=1,
        max_length=64,
        description='state_version from GET /planner/context',
    )
    response: dict[str, Any] | str = Field(
        ..., description='The untouched graph-v3 model response',
    )


def _status(run_id: str) -> dict[str, Any] | None:
    try:
        response = get_bridge().call_service(
            GetRunStatus,
            STATUS_SERVICE,
            GetRunStatus.Request(run_id=run_id),
        )
    except (RuntimeError, TimeoutError):
        return None
    if not response.found:
        return None
    try:
        payload = json.loads(response.status_json)
    except json.JSONDecodeError:
        return None
    return payload if isinstance(payload, dict) else None


def _http_status(code: str) -> int:
    if code in ('busy', 'conflict', 'stale_scene', 'scene_not_ready'):
        return 409
    if code in ('not_ready', 'ros_unavailable'):
        return 503
    if code == 'invalid_scene':
        return 502
    if code in ('invalid', 'unsupported_contract', 'not_executable', 'rejected'):
        return 422
    return 422


def _scene_envelope() -> dict[str, Any]:
    try:
        response = get_bridge().call_service(
            Trigger, SCENE_SERVICE, Trigger.Request(),
        )
    except (RuntimeError, TimeoutError) as error:
        raise HTTPException(
            503,
            detail={'code': 'ros_unavailable', 'reason': str(error)},
        ) from error
    if not response.success:
        raise HTTPException(
            409,
            detail={
                'code': 'scene_not_ready',
                'reason': response.message or 'scene is not ready',
            },
        )
    try:
        payload = json.loads(response.message)
    except json.JSONDecodeError as error:
        raise HTTPException(
            502,
            detail={'code': 'invalid_scene', 'reason': f'invalid scene JSON: {error}'},
        ) from error
    if not isinstance(payload, dict):
        raise HTTPException(
            502,
            detail={'code': 'invalid_scene', 'reason': 'scene envelope must be an object'},
        )
    return payload


@router.get('/context', summary='Пакет контекста graph-v3 для внешней модели')
def get_context():
    envelope = _scene_envelope()
    try:
        return build_planner_context(envelope)
    except ContextError as error:
        raise HTTPException(
            409 if error.code in ('unsupported_contract', 'scene_not_ready')
            else _http_status(error.code),
            detail={'code': error.code, 'reason': error.message},
        ) from error


@router.get('/scene', summary='Текущий scene_json для внешнего планировщика')
def get_scene():
    return _scene_envelope()


@router.post('/runs', summary='Передать готовый graph-v3 исполнителю')
def submit_run(request: RunRequest):
    global _active_run_id
    graph = (
        request.response
        if isinstance(request.response, str)
        else json.dumps(request.response, ensure_ascii=False)
    )
    try:
        response = get_bridge().call_service(
            SubmitPlan,
            SUBMIT_SERVICE,
            SubmitPlan.Request(
                run_id=request.run_id,
                context_state_version=request.context_state_version,
                response=graph,
            ),
        )
    except (RuntimeError, TimeoutError) as error:
        raise HTTPException(
            503,
            detail={'code': 'ros_unavailable', 'reason': str(error)},
        ) from error
    if not response.accepted:
        details: dict[str, Any] = {}
        raw_details = getattr(response, 'details_json', '') or ''
        if raw_details:
            try:
                parsed = json.loads(raw_details)
            except json.JSONDecodeError:
                parsed = {}
            if isinstance(parsed, dict):
                details = parsed
        raise HTTPException(
            _http_status(response.code),
            detail={
                'code': response.code,
                'reason': response.reason,
                'run_id': request.run_id,
                **details,
            },
        )
    with _lock:
        _active_run_id = request.run_id
    return {
        'status': 'duplicate' if response.code == 'duplicate' else 'accepted',
        'code': response.code,
        'run_id': request.run_id,
    }


@router.get('/runs/{run_id}', summary='Статус одного запуска графа')
def get_run(run_id: str):
    global _active_run_id
    status = _status(run_id)
    if status is None:
        raise HTTPException(404, f'run {run_id!r} was not submitted')
    if status.get('state') in (
        'completed', 'replan_required', 'operator_required', 'failed',
        'canceled',
    ):
        with _lock:
            if _active_run_id == run_id:
                _active_run_id = None
    return status


@router.post('/runs/{run_id}/cancel', summary='Отменить запуск графа')
def cancel_run(run_id: str):
    global _active_run_id
    try:
        response = get_bridge().call_service(
            webots_ros2_msgs.srv.SetString,
            CANCEL_SERVICE,
            webots_ros2_msgs.srv.SetString.Request(
                value=json.dumps({'run_id': run_id}),
            ),
        )
    except (RuntimeError, TimeoutError) as error:
        raise HTTPException(503, str(error)) from error
    if not response.success:
        status = _status(run_id)
        if status is None:
            raise HTTPException(404, f'run {run_id!r} was not submitted')
        raise HTTPException(409, status.get('reason') or 'run is not active')
    with _lock:
        if _active_run_id == run_id:
            _active_run_id = None
    return {'status': 'cancel_requested', 'run_id': run_id}


def cancel_active_run() -> None:
    """Stop the accepted graph before the regular cobot/stop command."""
    global _active_run_id
    with _lock:
        run_id = _active_run_id
        _active_run_id = None
    if not run_id:
        return
    try:
        get_bridge().call_service(
            webots_ros2_msgs.srv.SetString,
            CANCEL_SERVICE,
            webots_ros2_msgs.srv.SetString.Request(
                value=json.dumps({'run_id': run_id}),
            ),
        )
    except (RuntimeError, TimeoutError):
        return
