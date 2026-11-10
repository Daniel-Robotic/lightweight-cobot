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
from iiwa_msgs.srv import GetRunStatus

from .ros_node import get_bridge


SCENE_SERVICE = '/iiwa_scene_context/get_scene'
SUBMIT_SERVICE = '/iiwa_llm_plan_executor/submit'
CANCEL_SERVICE = '/iiwa_llm_plan_executor/cancel'
STATUS_SERVICE = '/iiwa_llm_plan_executor/get_status'

router = APIRouter(prefix='/planner', tags=['planner'])
_lock = threading.Lock()
_active_run_id: str | None = None


class RunRequest(BaseModel):
    run_id: str = Field(..., min_length=1, max_length=128)
    scene_revision: int = Field(..., ge=0)
    response: dict[str, Any] | str = Field(
        ..., description='The untouched Qwen3 graph response',
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


def _rejection(run_id: str) -> tuple[int, str, dict[str, Any] | None]:
    status = _status(run_id)
    reason = '' if status is None else str(status.get('reason', ''))
    lowered = reason.lower()
    if 'busy' in lowered:
        return 409, reason or 'another graph is already active', status
    if 'not_ready' in lowered:
        return 503, reason or 'the executor is not ready yet', status
    return 422, reason or 'graph was rejected', status


@router.get('/scene', summary='Текущий scene_json для внешнего планировщика')
def get_scene():
    try:
        response = get_bridge().call_service(
            Trigger, SCENE_SERVICE, Trigger.Request(),
        )
    except (RuntimeError, TimeoutError) as error:
        raise HTTPException(503, str(error)) from error
    if not response.success:
        raise HTTPException(409, response.message or 'scene is not ready')
    try:
        payload = json.loads(response.message)
    except json.JSONDecodeError as error:
        raise HTTPException(502, f'invalid scene JSON: {error}') from error
    if not isinstance(payload, dict):
        raise HTTPException(502, 'scene envelope must be a JSON object')
    return payload


@router.post('/runs', summary='Передать готовый граф Qwen3 исполнителю')
def submit_run(request: RunRequest):
    global _active_run_id
    payload = {
        'run_id': request.run_id,
        'scene_revision': int(request.scene_revision),
        'response': request.response,
    }
    try:
        response = get_bridge().call_service(
            webots_ros2_msgs.srv.SetString,
            SUBMIT_SERVICE,
            webots_ros2_msgs.srv.SetString.Request(
                value=json.dumps(payload, ensure_ascii=False),
            ),
        )
    except (RuntimeError, TimeoutError) as error:
        raise HTTPException(503, str(error)) from error
    if not response.success:
        code, reason, status = _rejection(request.run_id)
        raise HTTPException(code, detail=reason)
    with _lock:
        _active_run_id = request.run_id
    return {
        'status': 'accepted',
        'run_id': request.run_id,
        'scene_revision': request.scene_revision,
    }


@router.get('/runs/{run_id}', summary='Статус одного запуска графа')
def get_run(run_id: str):
    global _active_run_id
    status = _status(run_id)
    if status is None:
        raise HTTPException(404, f'run {run_id!r} was not submitted')
    if status.get('state') in (
        'completed', 'replan_required', 'operator_required', 'failed',
        'canceled', 'rejected',
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
