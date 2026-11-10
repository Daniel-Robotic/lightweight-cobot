"""Reference operations: tools, hatch, sealant contour and screw handoff."""
from __future__ import annotations

import json
import math
import uuid
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from webots_ros2_msgs.srv import SetString

from .geometry import (
    GeometrySpec,
    Transform,
    object_pose_transform,
    sealant_point,
    tcp_goal,
    world_target,
)
from .motion import MotionAdapter, MotionResult
from .planning_scene import (
    PlanningSceneBridge,
    attached_cap,
    attached_screw,
    attached_tool,
    cap_world_box,
    placed_cap_pose,
    screw_object_id,
)
from .scenario import ScenarioSpec


COMMAND_SERVICE = '/iiwa_experement/object_command'
STATE_TOPIC = '/iiwa_experement/object_state'


class OperationError(RuntimeError):
    """An operation could not be completed safely."""


@dataclass(frozen=True)
class OperationResult:
    success: bool
    operation: str
    message: str
    events: tuple[dict[str, Any], ...] = field(default_factory=tuple)


def _translation(xyz) -> Transform:
    return Transform.from_xyz_rpy(
        [float(value) for value in xyz], [0.0, 0.0, 0.0],
        parent='tcp', child='offset',
    )


class SupervisorClient:
    """Synchronous facade over the Webots attachment supervisor."""

    def __init__(self, node: Node, *, timeout_sec: float = 15.0) -> None:
        self.node = node
        self.timeout_sec = float(timeout_sec)
        self.latest: dict[str, Any] | None = None
        self._client = node.create_client(SetString, COMMAND_SERVICE)
        self._subscription = node.create_subscription(
            String, STATE_TOPIC, self._on_state, 10,
        )

    def _on_state(self, message: String) -> None:
        try:
            self.latest = json.loads(message.data)
        except Exception:
            pass

    def _spin(self, seconds: float = 0.05) -> None:
        import time

        finish = time.monotonic() + seconds
        while time.monotonic() < finish:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait_ready(self, timeout_sec: float = 60.0) -> dict[str, Any]:
        import time

        end = time.monotonic() + timeout_sec
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.latest and self.latest.get('ready'):
                return self.latest
        raise OperationError('supervisor did not become ready')

    def wait_version(self, minimum: int, timeout_sec: float | None = None) -> dict[str, Any]:
        import time

        end = time.monotonic() + (self.timeout_sec if timeout_sec is None else timeout_sec)
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.latest and self.latest.get('version', -1) >= minimum:
                return self.latest
        raise OperationError(f'supervisor state version {minimum} was not published')

    def command(
        self,
        op: str,
        object_id: str = '',
        *,
        target: Transform | None = None,
        target_name: str = '',
        expected_state: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        if not self._client.wait_for_service(timeout_sec=5.0):
            raise OperationError(f'{COMMAND_SERVICE} is unavailable')
        request_id = uuid.uuid4().hex
        payload: dict[str, Any] = {
            'request_id': request_id,
            'op': op,
            'object': object_id,
            'target_name': target_name,
        }
        if target is not None:
            payload['target'] = _transform_payload(target)
        if expected_state is not None:
            payload['expected_state'] = expected_state
        request = SetString.Request()
        request.value = json.dumps(payload)
        future = self._client.call_async(request)
        self._spin(0.0)
        import time

        end = time.monotonic() + self.timeout_sec
        while not future.done() and time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        if not future.done():
            raise OperationError(f'supervisor command {op} timed out')
        accepted = bool(future.result().success)
        # The detailed result is published atomically after the transition.
        state = self.latest
        last = (state or {}).get('last_command_result') or {}
        if last.get('request_id') != request_id:
            end = time.monotonic() + 2.0
            while time.monotonic() < end:
                rclpy.spin_once(self.node, timeout_sec=0.02)
                state = self.latest
                last = (state or {}).get('last_command_result') or {}
                if last.get('request_id') == request_id:
                    break
        result = last
        if not accepted or not result.get('accepted', False):
            reason = result.get('error') or 'supervisor rejected the command'
            raise OperationError(f'{op}({object_id or "-"}) rejected: {reason}')
        return state or {}

    @property
    def objects(self) -> dict[str, Any]:
        return dict((self.latest or {}).get('objects') or {})

    def object_pose(self, name: str) -> Transform:
        entry = self.objects.get(name)
        if not isinstance(entry, dict):
            raise OperationError(f'object {name!r} is not present in the supervisor state')
        return _transform_from_payload(entry, child=name)

    def logical_cap(self) -> dict[str, Any]:
        return dict((self.latest or {}).get('cap') or {})

    def logical_screws(self) -> dict[str, Any]:
        return dict((self.latest or {}).get('screws') or {})


def _transform_payload(transform: Transform) -> dict[str, Any]:
    return {
        'xyz': [float(value) for value in transform.xyz],
        'rotation': [
            [float(value) for value in row] for row in transform.rotation
        ],
    }


def _transform_from_payload(data: dict[str, Any], child: str = 'object') -> Transform:
    xyz = data.get('xyz')
    rotation = data.get('rotation')
    if not isinstance(xyz, (list, tuple)) or len(xyz) != 3:
        raise OperationError('object pose has no valid xyz')
    if not isinstance(rotation, (list, tuple)) or len(rotation) != 3:
        raise OperationError('object pose has no valid rotation')
    if any(not isinstance(row, (list, tuple)) or len(row) != 3 for row in rotation):
        raise OperationError('object pose rotation must be a 3x3 matrix')
    return Transform(
        xyz=tuple(float(value) for value in xyz),
        rotation=tuple(tuple(float(value) for value in row) for row in rotation),
        parent='world',
        child=child,
    )


class ReferenceOperations:
    """Sequence of visible robot operations for one experiment."""

    def __init__(
        self,
        node: Node,
        spec: ScenarioSpec,
        geometry: GeometrySpec,
        supervisor: SupervisorClient,
        bridge: PlanningSceneBridge,
        motion: MotionAdapter,
        *,
        event_sink: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self.node = node
        self.spec = spec
        self.geometry = geometry
        self.supervisor = supervisor
        self.bridge = bridge
        self.motion = motion
        self._event_sink = event_sink
        self.events: list[dict[str, Any]] = []
        self.completed_sealant: set[str] = set()
        self.last_sealant_point: tuple[str, float] | None = None
        self.cap_relative: Transform | None = None

    # ------------------------------------------------------------------ helpers
    def _emit(self, operation: str, **payload) -> None:
        event = {'operation': operation, **payload}
        self.events.append(event)
        if self._event_sink is not None:
            self._event_sink(event)

    def _require(self, condition: bool, message: str) -> None:
        if not condition:
            raise OperationError(message)

    def _move(
        self,
        goal: Transform,
        planner: str,
        speed: float,
        label: str,
        *,
        allow_fallback: bool = True,
    ) -> MotionResult:
        attempts = 3
        result = None
        planning_failed = False
        for attempt in range(attempts):
            # Rest-to-rest planning: the server's response adapter rejects any
            # plan whose start state is still moving.  Wait for the arm to
            # settle before every attempt instead of rapid-firing retries
            # into the same non-stationary window.
            self.motion.wait_stationary()
            result = self.motion.move_pose(
                goal,
                planner=planner,
                speed=speed,
                label=label if attempt == 0 else f'{label}:retry{attempt}',
            )
            if result.success:
                if attempt:
                    self._emit('motion_retry', label=label, planner=planner,
                               attempt=attempt, reason=result.message)
                return result
            planning_failed = (
                'Планирование' in result.message
                or 'planning' in result.message.lower()
                or 'недостижима' in result.message
            )
            if not planning_failed:
                break
            if attempt + 1 < attempts:
                # A failed search executed no motion; replan from the same
                # measured, still-synchronized state.
                self.motion.reset_stop()
                continue
        if allow_fallback and planner != 'ompl' and planning_failed:
            self.motion.reset_stop()
            self.motion.wait_stationary()
            fallback = self.motion.move_pose(
                goal, planner='ompl', speed=speed, label=f'{label}:ompl',
            )
            if fallback.success:
                self._emit(
                    'motion_fallback', label=label, original=planner,
                    reason=result.message if result is not None else '',
                )
                return fallback
            result = fallback
        raise OperationError(f'{label}: {result.message if result is not None else "motion failed"}')

    def _above(self, goal: Transform, gap: float) -> Transform:
        return goal.compose_relaxed(_translation([0.0, 0.0, -gap]))

    def _contact(
        self,
        goal: Transform,
        label: str,
        *,
        approach_gap: float | None = None,
        transfer_gap: float | None = None,
        remove_before_contact: str | None = None,
        remove_phase: str = 'transfer',
    ) -> None:
        approach_gap = (
            self.geometry.approach_gap if approach_gap is None else approach_gap
        )
        transfer_gap = (
            self.geometry.transfer_gap if transfer_gap is None else transfer_gap
        )
        if remove_before_contact is not None and remove_phase == 'transfer':
            # The object is the intended contact target.  Remove it from the
            # world scene for the whole approach so the attached copy cannot
            # duplicate it and the short axial moves are not rejected by a
            # false contact with the target itself.
            self.bridge.remove(remove_before_contact)
            self._require_synchronized()
        self.motion.wait_stationary()
        transfer = self._above(goal, transfer_gap)
        approach = self._above(goal, approach_gap)
        # Transfers and approaches use Pilz PTP: deterministic joint-space
        # interpolation instead of OMPL's random wide elbow sweeps.  OMPL
        # remains the automatic fallback of _move when PTP cannot plan.
        self._move(transfer, 'ptp', self.geometry.speeds['transfer'], f'{label}:transfer')
        try:
            self._move(
                approach, 'ptp', self.geometry.speeds['approach'],
                f'{label}:approach',
            )
        except OperationError as error:
            message = str(error)
            if 'Планирование' not in message and 'недостижима' not in message:
                raise
            # A pre-contact point can sit on a different IK branch.  The
            # collision-checked direct path to the contact is safer than
            # forcing an infeasible axial segment.
            self._emit('approach_skipped', label=f'{label}:approach',
                       reason=message)
            self.motion.reset_stop()
        if remove_before_contact is not None and remove_phase == 'contact':
            self.bridge.remove(remove_before_contact)
            self._require_synchronized()
        self._move(goal, 'lin', self.geometry.speeds['contact'], f'{label}:contact')

    def _retreat(self, goal: Transform, label: str) -> None:
        """Lift axially away from a contact pose before any lateral motion.

        On real hardware a sideways move straight out of a contact pose drags
        the tool or the part across the fixture, so the lift is mandatory:
        the gaps only shrink, and if every axial segment is rejected the
        operation stops instead of continuing at contact depth.
        """
        lifted = False
        last_error: OperationError | None = None
        for index, gap in enumerate((self.geometry.approach_gap, 0.03, 0.015)):
            lift_label = f'{label}:retreat' if index == 0 else f'{label}:retreat:{gap:.3f}'
            try:
                self._move(
                    self._above(goal, gap), 'lin',
                    self.geometry.speeds['approach'], lift_label,
                )
                lifted = True
                break
            except OperationError as error:
                if 'Планирование' not in str(error):
                    raise
                last_error = error
                self.motion.reset_stop()
        if not lifted:
            raise OperationError(
                f'{label}: the axial retreat was rejected ({last_error})'
            )
        self._move(
            self._above(goal, self.geometry.transfer_gap), 'ptp',
            self.geometry.speeds['transfer'], f'{label}:clear',
        )

    def _exit_rack(self, goal: Transform, label: str) -> None:
        """Lift straight up out of the rack, then step sideways out of it.

        The lift is mandatory: a lateral move at socket depth would drag the
        tool (or the empty patron) through the magazine on real hardware.
        The sideways step starts from the height that was actually reached.
        """
        height = None
        last_error: OperationError | None = None
        for index, gap in enumerate((
            self.geometry.transfer_gap, self.geometry.approach_gap, 0.03, 0.015,
        )):
            lift_label = f'{label}:lift' if index == 0 else f'{label}:lift:{gap:.3f}'
            try:
                self._move(
                    self._above(goal, gap), 'lin',
                    self.geometry.speeds['approach'], lift_label,
                )
                height = gap
                break
            except OperationError as error:
                if 'Планирование' not in str(error):
                    raise
                last_error = error
                self.motion.reset_stop()
        if height is None:
            raise OperationError(
                f'{label}: the axial lift out of the rack was rejected '
                f'({last_error})'
            )
        base = self._above(goal, height)
        for offset in (0.10, 0.06, 0.03):
            exit_pose = Transform.from_xyz_rpy(
                [0.0, offset, 0.0], [0.0, 0.0, 0.0],
                parent='world', child='rack_exit_offset',
            ).compose_relaxed(base)
            exit_label = (
                f'{label}:exit' if offset == 0.10 else f'{label}:exit:{offset:.2f}'
            )
            try:
                self._move(
                    exit_pose, 'lin', self.geometry.speeds['transfer'], exit_label,
                )
                return
            except OperationError as error:
                if 'Планирование' not in str(error):
                    raise
                last_error = error
                self.motion.reset_stop()
        raise OperationError(
            f'{label}: no lateral rack exit was plannable ({last_error})'
        )

    def _contact_with_recovery(
        self, goal: Transform, label: str, *, prepare=None, **kwargs,
    ) -> None:
        """Contact a target, reconfiguring through 'work' if planning fails.

        'work' is the only neutral pose the routes are allowed to use:
        'transport' is forbidden entirely and 'home' is reserved for the end
        of the route.  A single retry from 'work' switches the IK branch
        without leaving the working area.
        """
        try:
            self._contact(goal, label, **kwargs)
        except OperationError as error:
            if 'Планирование' not in str(error):
                raise
            self._emit('reconfigure', label=label, target='work',
                       reason=str(error))
            if prepare is not None:
                prepare()
            self.motion.reset_stop()
            named = self.motion.move_named('work', 0.3)
            if not getattr(named, 'success', False):
                raise error
            self.motion.reset_stop()
            self._contact(goal, f'{label}:recovery', **kwargs)

    def _require_synchronized(self) -> None:
        try:
            self.bridge.assert_synchronized(*self.bridge.expected_sets())
        except Exception as error:
            raise OperationError(f'planning scene is not synchronized: {error}') from error
        # The relay to /planning_scene is asynchronous for the MoveItPy
        # monitor.  Give it a short, bounded window before planning.
        if self.node is None:
            return
        import time

        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _expect_latest(self, timeout: float = 2.0) -> dict[str, Any]:
        import time

        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
            if self.supervisor.latest is not None:
                return self.supervisor.latest
        raise OperationError('supervisor state is unavailable')

    # ------------------------------------------------------------------ tools
    def take_tool(self, tool_name: str) -> OperationResult:
        self._emit('take_tool', object=tool_name, phase='start')
        try:
            self._require_synchronized()
            tool_pose = self.supervisor.object_pose(f'grant_{tool_name}')
            goal = tcp_goal(tool_pose, self.geometry.tool_tcp_to_nodes[tool_name])
            self._contact_with_recovery(
                goal, f'take_tool({tool_name})',
                remove_before_contact=f'grant_{tool_name}',
            )
            self.supervisor.command('attach_tool', tool_name)
            # The held tool becomes part of the robot in the planning scene,
            # so the lift, the rack exit and every later transfer are planned
            # with the tool body instead of dragging it through the magazine.
            self.bridge.attach(attached_tool(tool_name, self.geometry))
            self._require_synchronized()
            self._exit_rack(goal, f'take_tool({tool_name})')
            self._emit('take_tool', object=tool_name, phase='done',
                       tool=tool_name, tcp=goal.xyz)
            return OperationResult(True, 'take_tool', f'{tool_name} mounted', tuple(self.events))
        except Exception as error:
            self._emit('take_tool', object=tool_name, phase='failed', error=str(error))
            raise

    def dock_tool(self, tool_name: str) -> OperationResult:
        self._emit('dock_tool', object=tool_name, phase='start')
        try:
            self._require_synchronized()
            rack_pose = world_target(self.spec, self.geometry, f'tool:{tool_name}')
            goal = tcp_goal(rack_pose, self.geometry.tool_tcp_to_nodes[tool_name])
            # Remove the held tool from MoveIt before entering the socket
            # region.  The supervisor keeps following tcp, and the contact
            # pose is verified by both the motion adapter and supervisor.
            self.bridge.remove(f'grant_{tool_name}')
            self._require_synchronized()
            self._contact_with_recovery(goal, f'dock_tool({tool_name})')
            self.supervisor.command('dock_tool', tool_name)
            # The tool is back in its calibrated socket: restore the exact
            # initial box pose instead of reusing the rotated node pose.
            self.bridge.restore_world(f'grant_{tool_name}')
            self._require_synchronized()
            self._exit_rack(goal, f'dock_tool({tool_name})')
            self._emit('dock_tool', object=tool_name, phase='done')
            return OperationResult(True, 'dock_tool', f'{tool_name} docked', tuple(self.events))
        except Exception as error:
            self._emit('dock_tool', object=tool_name, phase='failed', error=str(error))
            raise

    # ------------------------------------------------------------------ hatch
    def pick_cap(self) -> OperationResult:
        self._emit('pick_cap', object='cap', phase='start')
        try:
            self._require_synchronized()
            cap_pose = self.supervisor.object_pose('grant_cap')
            # Use a reachable wrist yaw; the suction cup is symmetric and the
            # measured relative rotation is used when the hatch is placed.
            goal = Transform.from_xyz_rpy(
                [cap_pose.xyz[0], cap_pose.xyz[1], cap_pose.xyz[2] + 0.146],
                [180.0, 0.0, 45.0],
                parent='world',
                child='tcp',
            )
            self.cap_relative = goal.inverse().compose_relaxed(cap_pose)
            self._contact(
                goal, 'pick_cap', remove_before_contact='grant_cap',
            )
            self.supervisor.command('grab_cap', 'cap')
            # Plan the retreat and the transfer with the hatch attached to the
            # robot, not as a blind spot removed from the scene.
            self.bridge.attach(attached_cap(self.geometry))
            self._require_synchronized()
            self._retreat(goal, 'pick_cap')
            self._emit('pick_cap', object='cap', phase='done', tcp=goal.xyz)
            return OperationResult(True, 'pick_cap', 'hatch grasped', tuple(self.events))
        except Exception as error:
            self._emit('pick_cap', object='cap', phase='failed', error=str(error))
            raise

    def place_cap(self, target_pose: Transform | None = None) -> OperationResult:
        self._emit('place_cap', object='cap', phase='start')
        try:
            self._require_synchronized()
            target = target_pose or world_target(self.spec, self.geometry, 'cap_seat')
            relative = self.cap_relative or self.geometry.cap_tcp_to_node
            goal = tcp_goal(target, relative)
            self._contact(
                goal, 'place_cap',
                remove_before_contact='grant_cap',
                remove_phase='contact',
            )
            self.supervisor.command('place_cap', 'cap', target=target)
            # The collision box of the installed hatch is flat, thinner than
            # the plate and centred on it: it must clear both the body box
            # below and the suction-cup box above at the contact pose.
            self.bridge.add_world(
                'grant_cap', cap_world_box(self.geometry),
                placed_cap_pose(target, self.spec.body.yaw_deg),
            )
            self._require_synchronized()
            self._retreat(goal, 'place_cap')
            self._emit('place_cap', object='cap', phase='done', target=target.xyz)
            return OperationResult(True, 'place_cap', 'hatch placed', tuple(self.events))
        except Exception as error:
            self._emit('place_cap', object='cap', phase='failed', error=str(error))
            raise

    # ------------------------------------------------------------------ sealant
    def dispense(
        self,
        segment_name: str,
        start_fraction: float = 0.0,
        end_fraction: float = 1.0,
        *,
        lift: bool = True,
    ) -> OperationResult:
        self._emit('dispense', object=segment_name, phase='start')
        try:
            self._require_synchronized()
            start = sealant_point(
                self.geometry, self.spec, segment_name, start_fraction,
            )
            end = sealant_point(
                self.geometry, self.spec, segment_name, end_fraction,
            )
            tip = self.geometry.tool_work_tips['sealant_tool']
            start_goal = tcp_goal(start, tip)
            end_goal = tcp_goal(end, tip)
            if start_fraction <= 0.0 and end_fraction >= 1.0:
                self._contact(start_goal, f'dispense({segment_name})')
            else:
                self._contact(start_goal, f'dispense({segment_name}):resume')
            self._move(
                end_goal, 'lin', self.geometry.speeds['contour'],
                f'dispense({segment_name}):path', allow_fallback=False,
            )
            self.last_sealant_point = (segment_name, end_fraction)
            self.completed_sealant.add(segment_name)
            if lift:
                self._retreat(end_goal, f'dispense({segment_name})')
            self._emit('dispense', object=segment_name, phase='done',
                       start_fraction=start_fraction, end_fraction=end_fraction)
            return OperationResult(True, 'dispense', f'{segment_name} completed', tuple(self.events))
        except Exception as error:
            self._emit('dispense', object=segment_name, phase='failed', error=str(error))
            raise

    def dispense_contour(self, segments: Iterable[str]) -> OperationResult:
        """Traverse adjacent sides continuously without lifting the nozzle."""
        segments = list(segments)
        self._emit('dispense_contour', segments=list(segments), phase='start')
        try:
            self._require_synchronized()
            tip = self.geometry.tool_work_tips['sealant_tool']
            first = sealant_point(self.geometry, self.spec, segments[0], 0.0)
            first_goal = tcp_goal(first, tip)
            self._contact(first_goal, 'dispense_contour:start')
            for segment in segments:
                end = sealant_point(self.geometry, self.spec, segment, 1.0)
                end_goal = tcp_goal(end, tip)
                self._move(
                    end_goal, 'lin', self.geometry.speeds['contour'],
                    f'dispense_contour:{segment}', allow_fallback=False,
                )
                self.completed_sealant.add(segment)
                self.last_sealant_point = (segment, 1.0)
                self._emit('dispense', object=segment, phase='done')
            self._retreat(end_goal, 'dispense_contour')
            self._emit('dispense_contour', segments=list(segments), phase='done')
            return OperationResult(True, 'dispense_contour', 'contour completed', tuple(self.events))
        except Exception as error:
            self._emit('dispense_contour', phases='failed', error=str(error))
            raise

    # ------------------------------------------------------------------ screws
    def pick_screw(self, screw_id: str) -> OperationResult:
        def_name = screw_object_id(screw_id)
        self._emit('pick_screw', object=screw_id, phase='start')
        try:
            self._require_synchronized()
            screw_pose = self.supervisor.object_pose(def_name)
            goal = tcp_goal(screw_pose, self.geometry.screw_tcp_to_node)
            def restore_screw() -> None:
                # The first attempt removed the screw from the planning scene
                # before its transfer was rejected.  Restore its calibrated
                # tray pose for the retry.
                self.bridge.restore_world(def_name)
                self._require_synchronized()

            self._contact_with_recovery(
                goal, f'pick_screw({screw_id})',
                remove_before_contact=def_name,
                prepare=restore_screw,
            )
            self.supervisor.command('grab_screw', screw_id)
            self.bridge.attach(attached_screw(screw_id, self.geometry))
            self._require_synchronized()
            # The axial lift is mandatory even at far tray cells: continuing
            # laterally with the screw at tray depth would drag it across the
            # tray walls on real hardware.
            self._retreat(goal, f'pick_screw({screw_id})')
            self._emit('pick_screw', object=screw_id, phase='done')
            return OperationResult(True, 'pick_screw', f'{screw_id} grasped', tuple(self.events))
        except Exception as error:
            self._emit('pick_screw', object=screw_id, phase='failed', error=str(error))
            raise

    def handoff_screw(self, screw_id: str, hole_id: str) -> OperationResult:
        def_name = screw_object_id(screw_id)
        self._emit('handoff_screw', object=screw_id, target=hole_id, phase='start')
        try:
            self._require_synchronized()
            hole = world_target(self.spec, self.geometry, f'hole:{hole_id}')
            goal = tcp_goal(hole, self.geometry.screw_tcp_to_node)
            self._contact_with_recovery(
                goal, f'handoff_screw({screw_id},{hole_id})',
                remove_before_contact=def_name,
                remove_phase='contact',
            )
            self.supervisor.command(
                'handoff_screw', screw_id, target=hole, target_name=hole_id,
            )
            self._require_synchronized()
            # Always lift axially out of the handoff pose before the route
            # continues: a joint-space or lateral move at hole height would
            # drag the magnet across the body on real hardware.
            self._retreat(goal, f'handoff_screw({screw_id},{hole_id})')
            self._emit('delivered', object=screw_id, target=hole_id,
                       phase='done', hole=hole_id)
            return OperationResult(
                True, 'handoff_screw', f'{screw_id} delivered to {hole_id}',
                tuple(self.events),
            )
        except Exception as error:
            self._emit('handoff_screw', object=screw_id, target=hole_id,
                       phase='failed', error=str(error))
            raise

    # ------------------------------------------------------------------ tightening
    def tighten_hole(self, hole_id: str, twist_deg: float = 45.0) -> OperationResult:
        """Simulate tightening at an empty hole with the torque tool.

        The screw has already been handed to the virtual operator and removed
        from the scene.  The torque tool approaches the hole, rotates around
        its own axis, returns and retracts.  Nothing is inserted or retained,
        so the hole stays empty.
        """
        self._emit('tighten_simulated', object=hole_id, phase='start',
                   twist_deg=twist_deg)
        try:
            self._require_synchronized()
            hole = world_target(self.spec, self.geometry, f'hole:{hole_id}')
            # Reuse the calibrated handoff pose: an upright tool node 4 mm
            # above the hole with the same body yaw as the screw delivery.
            target = object_pose_transform(
                hole.xyz[0], hole.xyz[1], hole.xyz[2] + 0.004,
                self.spec.body.yaw_deg, child='tighten_target',
            )
            tip = self.geometry.tool_tcp_to_nodes['torque_tool']
            goal = tcp_goal(target, tip)
            self._contact_with_recovery(goal, f'tighten_hole({hole_id})')
            twist_error = None
            twist_done = False
            joint_twist = False
            twisted_goal = goal
            # Prefer a pure flange rotation: it twists the tool about its own
            # axis without moving the arm to another IK branch.  A Cartesian
            # twist can push joint 6 into its limit, after which 'work' is no
            # longer reachable.
            joints = None
            provider = getattr(self.motion, 'joint_positions', None)
            if provider is not None:
                joints = provider()
            if joints is not None and len(joints) == 7:
                joints = tuple(float(value) for value in joints)
                for sign in (1.0, -1.0):
                    target_joints = list(joints)
                    target_joints[6] += math.radians(sign * twist_deg)
                    self.motion.wait_stationary()
                    result = self.motion.move_joints(
                        target_joints, self.geometry.speeds['contour'],
                        f'tighten_hole({hole_id}):joint_twist',
                    )
                    if getattr(result, 'success', False):
                        twist_done = True
                        joint_twist = True
                        self._emit('tighten_twist', object=hole_id,
                                   joint=7, delta_deg=sign * twist_deg)
                        break
                    reset = getattr(self.motion, 'reset_stop', None)
                    if reset is not None:
                        reset()
            if not twist_done:
                for suffix, offset in (
                    ('twist:positive', twist_deg),
                    ('twist:negative', -twist_deg),
                ):
                    candidate = goal.compose_relaxed(Transform.from_xyz_rpy(
                        [0.0, 0.0, 0.0], [0.0, 0.0, offset],
                        parent='tcp', child='tighten_twist',
                    ))
                    self.motion.wait_stationary()
                    try:
                        self._move(
                            candidate, 'lin', self.geometry.speeds['contour'],
                            f'tighten_hole({hole_id}):{suffix}',
                        )
                        twist_done = True
                        twisted_goal = candidate
                        break
                    except OperationError as error:
                        # Holes on the far side of the body can sit against
                        # the wrist limit in one direction.  A failed plan
                        # executes no motion, so the opposite twist is safe.
                        if 'Планирование' not in str(error):
                            raise
                        self.motion.reset_stop()
                        twist_error = error
                if not twist_done:
                    raise twist_error
            # Lift axially out of the hole BEFORE undoing the twist: rotating
            # the flange back at the contact point would visibly unscrew the
            # simulated tightening.
            self._retreat(twisted_goal, f'tighten_hole({hole_id})')
            if joint_twist:
                # Return only joint 7 to its pre-twist value; the other
                # joints stay where the retreat left them.
                current = provider() if provider is not None else None
                self._require(
                    current is not None and len(current) == 7,
                    f'tighten_hole({hole_id}): joint state unavailable for '
                    'the untwist',
                )
                untwist_joints = list(current)
                untwist_joints[6] = joints[6]
                self.motion.wait_stationary()
                untwist = self.motion.move_joints(
                    untwist_joints, self.geometry.speeds['contour'],
                    f'tighten_hole({hole_id}):joint_untwist',
                )
                if not getattr(untwist, 'success', False):
                    raise OperationError(
                        f'tighten_hole({hole_id}): the joint untwist failed: '
                        f'{untwist.message}'
                    )
            else:
                # Pure rotation about the tool axis at the safe height; the
                # raised untwisted pose shares the retreat position.
                self._move(
                    self._above(goal, self.geometry.transfer_gap), 'lin',
                    self.geometry.speeds['contour'],
                    f'tighten_hole({hole_id}):untwist',
                )
            self._emit('tighten_simulated', object=hole_id, phase='done',
                       twist_deg=twist_deg)
            return OperationResult(
                True, 'tighten_simulated',
                f'{hole_id} tightening simulated', tuple(self.events),
            )
        except Exception as error:
            self._emit('tighten_simulated', object=hole_id, phase='failed',
                       error=str(error))
            raise

    # ------------------------------------------------------------------ recovery
    # ------------------------------------------------------------------ recovery
    def simulate_grasp_failure(self, kind: str, name: str = '') -> OperationResult:
        """Move to a grasp pose, do not attach, and restore the object."""
        label = f'grasp_failed({kind}{":" + name if name else ""})'
        self._emit('grasp_failed', object=name or kind, phase='start')
        try:
            if kind == 'cap':
                pose = self.supervisor.object_pose('grant_cap')
                goal = tcp_goal(pose, self.geometry.cap_tcp_to_node)
                self._contact(goal, label, remove_before_contact='grant_cap')
                self._retreat(goal, label)
                self.bridge.restore_world('grant_cap')
                self._require_synchronized()
            elif kind == 'screw':
                def_name = screw_object_id(name)
                pose = self.supervisor.object_pose(def_name)
                goal = tcp_goal(pose, self.geometry.screw_tcp_to_node)
                self._contact(goal, label, remove_before_contact=def_name)
                self._retreat(goal, label)
                self.bridge.restore_world(def_name)
                self._require_synchronized()
            else:
                raise OperationError(f'unknown grasp target {kind!r}')
            self._emit('grasp_failed', object=name or kind, phase='done')
            return OperationResult(True, 'grasp_failed', label, tuple(self.events))
        except Exception as error:
            self._emit('grasp_failed', object=name or kind,
                       phase='failed', error=str(error))
            raise

    def place_cap_shifted(self, dx: float = 0.003) -> OperationResult:
        target = world_target(self.spec, self.geometry, 'cap_seat')
        shifted = Transform(
            xyz=(target.xyz[0] + dx, target.xyz[1], target.xyz[2]),
            rotation=target.rotation,
            parent=target.parent,
            child=target.child,
        )
        self._emit('place_cap_shifted', object='cap', phase='start', dx=dx)
        try:
            result = self.place_cap(shifted)
            self._emit('place_cap_shifted', object='cap', phase='done', dx=dx)
            return result
        except Exception as error:
            self._emit('place_cap_shifted', object='cap', phase='failed',
                       dx=dx, error=str(error))
            raise

    # ------------------------------------------------------------------ verifies
    def inspect_cap(
        self,
        target_pose: Transform | None = None,
        tolerance: float | None = None,
    ) -> dict[str, Any]:
        self.supervisor.command('inspect')
        target = target_pose or world_target(self.spec, self.geometry, 'cap_seat')
        state = self._expect_latest()
        cap = state.get('cap') or {}
        if cap.get('state') != 'installed':
            return {'ok': False, 'reason': f'hatch state is {cap.get("state")!r}'}
        pose_entry = cap.get('pose')
        if not pose_entry:
            return {'ok': False, 'reason': 'hatch pose is unknown'}
        pose = _transform_from_payload(pose_entry, child='cap')
        distance = float(
            (pose.position - target.position) @ (pose.position - target.position)
        ) ** 0.5
        limit = (
            self.geometry.position_tolerance if tolerance is None else float(tolerance)
        )
        ok = distance <= limit
        return {
            'ok': ok,
            'distance': distance,
            'target': target.xyz,
            'actual': pose.xyz,
        }

    def inspect_sealant(self) -> dict[str, Any]:
        expected = list(self.spec.sealant_sides)
        missing = [side for side in expected if side not in self.completed_sealant]
        return {
            'ok': not missing,
            'completed': sorted(self.completed_sealant),
            'missing': missing,
        }

    def inspect_state(self) -> dict[str, Any]:
        self.supervisor.command('inspect')
        return self._expect_latest()
