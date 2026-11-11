"""ROS-free graph engine: admission, edge selection, terminals and status."""
from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping

from iiwa_scene_context.catalog import DEFAULT_CATALOG, SkillCatalog

from .schema import TERMINALS, PlanGraph, PlanSchemaError, parse_response
from .validator import ValidationResult, validate_graph


ACTIVE_STATES = ('admitted', 'running')
TERMINAL_STATES = (
    'completed', 'replan_required', 'operator_required', 'failed', 'canceled',
)
OWNERSHIP_SKILLS = ('grab', 'pick', 'release', 'dock_tool', 'handoff_screw')
DEFAULT_MIN_START_DELAY_SEC = 3.0
DEFAULT_SCENE_MAX_AGE_SEC = 2.0
DEFAULT_MAX_STEPS = 512


@dataclass(frozen=True)
class AdmissionResult:
    accepted: bool
    code: str
    message: str
    run_id: str

    def as_dict(self) -> dict[str, Any]:
        return {
            'accepted': self.accepted,
            'code': self.code,
            'message': self.message,
            'run_id': self.run_id,
        }


@dataclass
class RunRecord:
    run_id: str
    state: str = 'new'
    reason: str = ''
    scene_revision: int = 0
    response: str = ''
    node_id: str | None = None
    skill: str | None = None
    event: str | None = None
    next: str | None = None
    mounted_tool: str | None = None
    held_object: str | None = None
    terminal: str | None = None
    events: list[dict[str, Any]] = field(default_factory=list)
    context: Any = None
    cancel_requested: bool = False
    done: threading.Event = field(default_factory=threading.Event)
    thread: threading.Thread | None = None
    details: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RunStatus:
    found: bool
    run_id: str
    state: str
    reason: str
    node_id: str | None
    skill: str | None
    event: str | None
    next: str | None
    scene_revision: int
    terminal: str | None
    mounted_tool: str | None
    held_object: str | None
    events: tuple[dict[str, Any], ...] = ()

    def as_dict(self) -> dict[str, Any]:
        payload = {
            'found': self.found,
            'run_id': self.run_id,
            'state': self.state,
            'reason': self.reason,
            'node_id': self.node_id,
            'skill': self.skill,
            'event': self.event,
            'next': self.next,
            'scene_revision': self.scene_revision,
            'terminal': self.terminal,
            'mounted_tool': self.mounted_tool,
            'held_object': self.held_object,
            'events': list(self.events),
        }
        return payload

    def to_json(self) -> str:
        return json.dumps(self.as_dict(), ensure_ascii=False)


@dataclass(frozen=True)
class CancelResult:
    accepted: bool
    message: str
    run_id: str


class GraphEngine:
    def __init__(
        self,
        scene_provider: Callable[[], Mapping[str, Any] | None],
        context_factory: Callable[[Mapping[str, Any], Mapping[str, Any]], Any],
        skills: Any,
        *,
        catalog: SkillCatalog = DEFAULT_CATALOG,
        clock: Callable[[], float] | None = None,
        on_plan: Callable[[dict[str, Any]], None] | None = None,
        on_status: Callable[[dict[str, Any]], None] | None = None,
        on_replan: Callable[[dict[str, Any] | None], None] | None = None,
        on_stop: Callable[[str], None] | None = None,
        robot_fresh: Callable[[], tuple[bool, str]] | None = None,
        threaded: bool = True,
        min_start_delay_sec: float = DEFAULT_MIN_START_DELAY_SEC,
        scene_max_age_sec: float = DEFAULT_SCENE_MAX_AGE_SEC,
        max_steps: int = DEFAULT_MAX_STEPS,
    ) -> None:
        self.scene_provider = scene_provider
        self.context_factory = context_factory
        self.skills = skills
        self.catalog = catalog
        self._clock = clock or time.monotonic
        self.on_plan = on_plan
        self.on_status = on_status
        self.on_replan = on_replan
        self.on_stop = on_stop
        self.robot_fresh = robot_fresh
        self.threaded = bool(threaded)
        self.min_start_delay_sec = float(min_start_delay_sec)
        self.scene_max_age_sec = float(scene_max_age_sec)
        self.max_steps = int(max_steps)
        self.scene_ready = False
        self.ready_at: float | None = None
        self.motion_ready = False
        self.readiness_reason = 'not ready'
        self._runs: dict[str, RunRecord] = {}
        self._lock = threading.RLock()

    # ------------------------------------------------------------------ readiness
    def update_readiness(
        self,
        *,
        scene_ready: bool,
        ready_at: float | None,
        motion_ready: bool,
        reason: str = '',
    ) -> None:
        self.scene_ready = bool(scene_ready)
        self.ready_at = None if ready_at is None else float(ready_at)
        self.motion_ready = bool(motion_ready)
        self.readiness_reason = reason or (
            'ready' if self.scene_ready and self.motion_ready else 'not ready'
        )

    def _not_ready(self, now: float) -> str | None:
        if not self.scene_ready:
            return f'scene is not ready: {self.readiness_reason}'
        if not self.motion_ready:
            return f'motion is not ready: {self.readiness_reason}'
        if self.ready_at is None:
            return 'scene_ready time is unknown'
        remaining = self.min_start_delay_sec - (now - self.ready_at)
        if remaining > 0.0:
            return (
                'start delay has not elapsed '
                f'({remaining:.2f} s of {self.min_start_delay_sec:.1f} s left)'
            )
        return None

    # ------------------------------------------------------------------ admission
    def submit(
        self,
        run_id: str,
        response_json: str | Mapping[str, Any],
        scene_revision: int,
    ) -> AdmissionResult:
        run_id = str(run_id).strip()
        if not run_id:
            return AdmissionResult(False, 'rejected', 'run_id must not be empty', '')
        raw_response = (
            response_json
            if isinstance(response_json, str)
            else json.dumps(response_json, ensure_ascii=False)
        )
        with self._lock:
            existing = self._runs.get(run_id)
            if existing is not None:
                if existing.state in ACTIVE_STATES:
                    return AdmissionResult(
                        True, 'duplicate',
                        f'run {run_id!r} is already active; no second motion',
                        run_id,
                    )
                if existing.state == 'rejected':
                    return AdmissionResult(
                        False, existing.reason.split(':', 1)[0] or 'rejected',
                        existing.reason, run_id,
                    )
                return AdmissionResult(
                    True, 'duplicate',
                    f'run {run_id!r} was already admitted; no second motion',
                    run_id,
                )
            active = [
                record for record in self._runs.values()
                if record.state in ACTIVE_STATES
            ]
            now = self._clock()
            record = RunRecord(
                run_id=run_id,
                response=raw_response,
                scene_revision=int(scene_revision),
            )
            self._runs[run_id] = record
            if active:
                record.state = 'rejected'
                record.reason = 'busy: another run is active'
                return AdmissionResult(
                    False, 'busy', 'another graph is already active', run_id,
                )
            reason = self._not_ready(now)
            if reason is not None:
                record.state = 'rejected'
                record.reason = f'not_ready: {reason}'
                return AdmissionResult(False, 'not_ready', reason, run_id)
            envelope = self._envelope()
            if envelope is None:
                record.state = 'rejected'
                record.reason = 'not_ready: no fresh scene snapshot'
                return AdmissionResult(
                    False, 'not_ready', 'no fresh scene snapshot', run_id,
                )
            envelope_revision = int(envelope.get('scene_revision', -1))
            if envelope_revision != int(scene_revision):
                record.state = 'rejected'
                record.reason = (
                    f'stale_scene: submitted revision {scene_revision} does not '
                    f'match current revision {envelope_revision}'
                )
                return AdmissionResult(
                    False, 'stale_scene',
                    f'submitted scene_revision {scene_revision} does not match '
                    f'the current revision {envelope_revision}',
                    run_id,
                )
            stale = self._scene_age(envelope, now)
            if stale is not None:
                record.state = 'rejected'
                record.reason = f'stale_scene: {stale}'
                return AdmissionResult(False, 'stale_scene', stale, run_id)
            try:
                graph = parse_response(raw_response)
            except PlanSchemaError as error:
                record.state = 'rejected'
                record.reason = f'rejected: {error}'
                return AdmissionResult(False, 'rejected', str(error), run_id)
            validation: ValidationResult = validate_graph(
                graph,
                envelope,
                self.catalog,
                expected_scene_revision=scene_revision,
            )
            if not validation.ok:
                record.state = 'rejected'
                record.reason = f'{validation.code}: {validation.reason}'
                record.details = validation.details
                return AdmissionResult(
                    False, validation.code, validation.reason, run_id,
                )
            record.state = 'admitted'
            record.scene_revision = envelope_revision
            try:
                if self.on_plan is not None:
                    self.on_plan({
                        'run_id': run_id,
                        'scene_revision': envelope_revision,
                        'response': raw_response,
                    })
            except Exception as error:
                record.state = 'rejected'
                record.reason = f'rejected: plan publication failed: {error}'
                return AdmissionResult(
                    False, 'rejected',
                    f'plan publication failed: {error}', run_id,
                )
            if self.threaded:
                thread = threading.Thread(
                    target=self._execute,
                    args=(run_id, graph, envelope),
                    name=f'graph-run-{run_id}',
                    daemon=True,
                )
                record.thread = thread
                thread.start()
            return AdmissionResult(True, 'accepted', 'graph admitted', run_id)

    # ------------------------------------------------------------------ helpers
    def _envelope(self) -> Mapping[str, Any] | None:
        try:
            envelope = self.scene_provider()
        except Exception:
            return None
        if not isinstance(envelope, Mapping):
            return None
        if not isinstance(envelope.get('scene_json'), Mapping):
            return None
        if 'scene_revision' not in envelope:
            return None
        return envelope

    def _scene_age(self, envelope: Mapping[str, Any], now: float) -> str | None:
        try:
            stamp = float(envelope.get('timestamp'))
        except (TypeError, ValueError):
            return 'scene snapshot has no timestamp'
        if now - stamp > self.scene_max_age_sec:
            return (
                f'scene snapshot is stale ({now - stamp:.2f} s > '
                f'{self.scene_max_age_sec:.2f} s)'
            )
        return None

    def _context(self, run: RunRecord, envelope: Mapping[str, Any]) -> Any:
        context = self.context_factory(envelope, envelope['scene_json'])
        initial = (envelope.get('scene_json') or {}).get('initial_state') or {}
        if hasattr(context, 'mounted_tool'):
            context.mounted_tool = initial.get('tool_mounted') or None
        if hasattr(context, 'held_object'):
            context.held_object = initial.get('held_object') or None
        if hasattr(context, 'scene_revision'):
            context.scene_revision = int(envelope.get('scene_revision', 0))
        run.context = context
        return context

    def _status_payload(self, run: RunRecord, **extra) -> dict[str, Any]:
        context = run.context
        if context is not None:
            if hasattr(context, 'mounted_tool'):
                run.mounted_tool = context.mounted_tool
            if hasattr(context, 'held_object'):
                run.held_object = context.held_object
        payload = {
            'run_id': run.run_id,
            'state': run.state,
            'reason': run.reason,
            'node_id': run.node_id,
            'skill': run.skill,
            'event': run.event,
            'next': run.next,
            'scene_revision': run.scene_revision,
            'terminal': run.terminal,
            'mounted_tool': run.mounted_tool,
            'held_object': run.held_object,
            'timestamp': self._clock(),
        }
        payload.update(extra)
        return payload

    def _emit_status(self, run: RunRecord, **extra) -> None:
        payload = self._status_payload(run, **extra)
        if self.on_status is not None:
            try:
                self.on_status(payload)
            except Exception:
                pass

    def _record_event(
        self,
        run: RunRecord,
        node,
        event: str,
        next_id: str,
        outcome: Any,
    ) -> None:
        context = run.context
        if context is not None:
            if hasattr(context, 'mounted_tool'):
                run.mounted_tool = context.mounted_tool
            if hasattr(context, 'held_object'):
                run.held_object = context.held_object
        details = getattr(outcome, 'details', {}) or {}
        entry = {
            'run_id': run.run_id,
            'node_id': node.id,
            'skill': node.skill,
            'event': event,
            'reason': details.get('reason', ''),
            'next': next_id,
            'scene_revision': run.scene_revision,
            'mounted_tool': run.mounted_tool,
            'held_object': run.held_object,
            'details': details,
            'timestamp': self._clock(),
        }
        run.events.append(entry)
        self._emit_status(run, event_detail=entry)

    def _stop_motion(self, reason: str) -> None:
        if self.on_stop is not None:
            try:
                self.on_stop(reason)
            except Exception:
                pass

    def _terminal(self, run: RunRecord, state: str, reason: str) -> None:
        run.state = state
        run.reason = reason
        run.terminal = state
        if state in ('failed', 'canceled'):
            self._stop_motion(reason)
        self._emit_status(run, terminal_reason=reason)
        run.done.set()

    # ------------------------------------------------------------------ execution
    def run_accepted(self, run_id: str, graph: PlanGraph | None = None,
                     envelope: Mapping[str, Any] | None = None) -> None:
        """Synchronously execute an already admitted run (tests, tools)."""
        run = self._runs.get(str(run_id).strip())
        if run is None or run.state != 'admitted':
            return
        graph = graph or parse_response(run.response)
        envelope = envelope or self._envelope()
        if envelope is None:
            self._terminal(run, 'failed', 'scene snapshot is unavailable')
            return
        self._execute(run.run_id, graph, envelope)

    def _execute(self, run_id: str, graph: PlanGraph, first_envelope: Mapping[str, Any]) -> None:
        with self._lock:
            run = self._runs[run_id]
            run.state = 'running'
        try:
            context = self._context(run, first_envelope)
            envelope = first_envelope
            node_id = graph.entry
            steps = 0
            while True:
                if run.cancel_requested:
                    self._terminal(run, 'canceled', 'cancel requested')
                    return
                if steps >= self.max_steps:
                    self._terminal(
                        run, 'failed',
                        f'executed step limit of {self.max_steps} reached',
                    )
                    return
                now = self._clock()
                if self.robot_fresh is not None:
                    try:
                        fresh, message = self.robot_fresh()
                    except Exception as error:
                        fresh, message = False, str(error)
                    if not fresh:
                        self._terminal(run, 'failed', f'robot state: {message}')
                        return
                current = self._envelope()
                if current is None:
                    self._terminal(run, 'failed', 'fresh scene snapshot is unavailable')
                    return
                age = self._scene_age(current, now)
                if age is not None:
                    self._terminal(run, 'failed', age)
                    return
                revision = int(current.get('scene_revision', -1))
                if revision != run.scene_revision:
                    previous_node = graph.node(node_id) if node_id in graph.ids else None
                    acknowledged = (
                        run.skill in OWNERSHIP_SKILLS
                        and run.event == 'success'
                        and previous_node is not None
                        and revision > run.scene_revision
                    )
                    if not acknowledged:
                        self._terminal(
                            run, 'failed',
                            'scene_revision changed externally '
                            f'({run.scene_revision} -> {revision}); motion stopped',
                        )
                        return
                    run.scene_revision = revision
                    if hasattr(context, 'scene_revision'):
                        context.scene_revision = revision
                    envelope = current
                node = graph.node(node_id)
                outcome = self.skills.execute(node, context)
                event = str(outcome.event)
                transitions = {
                    transition.event: transition.next for transition in node.transitions
                }
                run.node_id = node.id
                run.skill = node.skill
                run.event = event
                if event not in transitions:
                    run.next = None
                    self._record_event(run, node, event, '', outcome)
                    self._terminal(
                        run, 'failed',
                        f'unknown skill outcome {event!r} for node {node.id!r}',
                    )
                    return
                next_id = transitions[event]
                run.next = next_id
                self._record_event(run, node, event, next_id, outcome)
                steps += 1
                if run.cancel_requested:
                    self._terminal(run, 'canceled', 'cancel requested')
                    return
                if next_id in TERMINALS:
                    if next_id == 'replan_required':
                        fresh_scene = self._envelope()
                        if self.on_replan is not None:
                            try:
                                self.on_replan(
                                    None if fresh_scene is None else dict(fresh_scene),
                                )
                            except Exception:
                                pass
                    if next_id == 'completed' and hasattr(context, 'held_object'):
                        if context.held_object is not None:
                            # The graph deliberately leaves an object in the
                            # gripper only if it never asked to release it.
                            pass
                    self._terminal(run, next_id, f'graph reached {next_id}')
                    return
                node_id = next_id
        except Exception as error:
            self._terminal(run, 'failed', f'execution error: {error}')

    # ------------------------------------------------------------------ queries
    def get_status(self, run_id: str) -> RunStatus:
        run_id = str(run_id).strip()
        with self._lock:
            run = self._runs.get(run_id)
            if run is None:
                return RunStatus(
                    False, run_id, 'unknown', 'run_id was not submitted',
                    None, None, None, None, 0, None, None, None, (),
                )
            return RunStatus(
                True,
                run.run_id,
                run.state,
                run.reason,
                run.node_id,
                run.skill,
                run.event,
                run.next,
                run.scene_revision,
                run.terminal,
                run.mounted_tool,
                run.held_object,
                tuple(run.events),
            )

    def cancel(self, run_id: str) -> CancelResult:
        run_id = str(run_id).strip()
        with self._lock:
            run = self._runs.get(run_id)
            if run is None:
                return CancelResult(False, 'run_id is unknown', run_id)
            if run.state not in ACTIVE_STATES:
                return CancelResult(
                    False, f'run is not active (state={run.state})', run_id,
                )
            run.cancel_requested = True
            if run.state == 'admitted' and not self.threaded:
                self._terminal(run, 'canceled', 'cancel requested')
            self._stop_motion('cancel requested')
            self._emit_status(run, cancel_requested=True)
            return CancelResult(True, 'cancel requested', run_id)

    def wait(self, run_id: str, timeout: float | None = None) -> bool:
        run = self._runs.get(str(run_id).strip())
        if run is None:
            return False
        return run.done.wait(timeout)

    def active_run_id(self) -> str | None:
        with self._lock:
            for run in self._runs.values():
                if run.state in ACTIVE_STATES:
                    return run.run_id
        return None
