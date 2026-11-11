"""Constrained MoveItPy adapter used by every graph skill.

The adapter differs from the existing ``MoveToPose`` action on purpose: it
seeds IK from the measured current joints, sets a joint-7 corridor and a
working-orientation path constraint, checks the *whole* planned trajectory
with :class:`MotionPolicy` and only then asks the trajectory execution manager
to run it.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping

import numpy as np
from scipy.spatial.transform import Rotation

from iiwa_experement.geometry import GeometrySpec, Transform

from .motion_policy import MotionCheck, MotionPolicy, tcp_tilt_deg


class PlannerError(RuntimeError):
    """The MoveIt plan could not be produced or executed."""


@dataclass
class PlannedMotion:
    points: list[tuple[float, ...]]
    tcp_poses: list[Transform]
    current_joints: tuple[float, ...] | None = None
    start_tcp_pose: Transform | None = None
    handle: Any = None
    details: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class MotionOutcome:
    success: bool
    code: str
    message: str
    details: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            'success': self.success,
            'code': self.code,
            'message': self.message,
            'details': dict(self.details),
        }


_POSE_TO_MATRIX = None


def _pose_message(transform: Transform, frame_id: str):
    from geometry_msgs.msg import Pose, PoseStamped

    quaternion = Rotation.from_matrix(transform.matrix).as_quat()
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (
        float(transform.xyz[0]),
        float(transform.xyz[1]),
        float(transform.xyz[2]),
    )
    pose.orientation.x = float(quaternion[0])
    pose.orientation.y = float(quaternion[1])
    pose.orientation.z = float(quaternion[2])
    pose.orientation.w = float(quaternion[3])
    stamped = PoseStamped()
    stamped.header.frame_id = frame_id
    stamped.pose = pose
    return stamped


def _matrix_transform(matrix, parent: str, child: str) -> Transform:
    values = np.asarray(matrix, dtype=float)
    return Transform(
        xyz=tuple(float(value) for value in values[:3, 3]),
        rotation=tuple(
            tuple(float(value) for value in row)
            for row in values[:3, :3]
        ),
        parent=parent,
        child=child,
    )


def _error_code_name(value: int | None) -> str:
    if value is None:
        return 'unknown'
    from moveit_msgs.msg import MoveItErrorCodes

    for name in dir(MoveItErrorCodes):
        if name.isupper() and getattr(MoveItErrorCodes, name) == value:
            return f'{name}({value})'
    return f'code {value}'


def _format_contacts(result: Any) -> str:
    contacts = getattr(result, 'contacts', None)
    pairs: list[str] = []
    try:
        for key in contacts.keys():
            pairs.append(str(key))
    except Exception:
        try:
            for item in contacts:
                pairs.append(str(item))
        except Exception:
            pass
    return ', '.join(sorted(set(pairs))) if pairs else 'unknown pair'


class MoveItPyPlanner:
    """Thin wrapper around MoveItPy with the constrained plan request."""

    PHASES = {
        # PTP: free-space changes between working points, objects, tools,
        # placement points and safe alignments.  No J7 path constraint; the
        # full sampled trajectory is checked afterwards.
        'transfer': ('pilz_industrial_motion_planner', 'PTP', 0.25, 2.0),
        'approach': ('pilz_industrial_motion_planner', 'PTP', 0.18, 2.0),
        'observe': ('pilz_industrial_motion_planner', 'PTP', 0.18, 2.0),
        'align': ('pilz_industrial_motion_planner', 'PTP', 0.10, 2.0),
        'escape': ('pilz_industrial_motion_planner', 'PTP', 0.15, 2.0),
        # LIN: axial contact/retreat and the continuous dispense contour.
        'contact': ('pilz_industrial_motion_planner', 'LIN', 0.06, 2.0),
        'lower': ('pilz_industrial_motion_planner', 'LIN', 0.06, 2.0),
        'retreat': ('pilz_industrial_motion_planner', 'LIN', 0.12, 2.0),
        'dispense': ('pilz_industrial_motion_planner', 'LIN', 0.09, 2.0),
    }

    def __init__(
        self,
        geometry: GeometrySpec,
        policy: MotionPolicy,
        *,
        planning_group: str = 'iiwa_arm',
        pose_link: str = 'tcp',
        pose_frame: str = 'base_link',
        moveit_factory: Callable[[], Any] | None = None,
        joint_limits: Mapping[str, tuple[float, float]] | None = None,
    ) -> None:
        self.geometry = geometry
        self.policy = policy
        self.planning_group = planning_group
        self.pose_link = pose_link
        self.pose_frame = pose_frame
        self._moveit_factory = moveit_factory
        self.joint_limits = dict(joint_limits or {})
        self._moveit = None
        self._arm = None
        self._model = None
        self._group = None
        self._scene_monitor = None
        self._stopped = False

    # ------------------------------------------------------------------ setup
    def ensure_ready(self) -> None:
        self._ensure()

    def _ensure(self) -> None:
        if self._arm is not None:
            return
        try:
            if self._moveit_factory is not None:
                self._moveit = self._moveit_factory()
            else:
                from moveit.planning import MoveItPy

                self._moveit = MoveItPy(node_name='iiwa_llm_plan_executor_moveit')
            self._model = self._moveit.get_robot_model()
            self._arm = self._moveit.get_planning_component(self.planning_group)
            self._group = self._model.get_joint_model_group(self.planning_group)
            self._scene_monitor = self._moveit.get_planning_scene_monitor()
        except Exception as error:
            raise PlannerError(f'MoveItPy is unavailable: {error}') from error

    def _goal_in_pose_frame(self, goal: Transform) -> Transform:
        if goal.parent == self.pose_frame:
            return goal
        if goal.parent == 'world':
            return self.geometry.world_to_base.inverse().compose_relaxed(goal)
        raise PlannerError(
            f'goal frame {goal.parent!r} is not world or {self.pose_frame!r}'
        )

    def _constraints(self, goal: Transform, *, include_orientation: bool = True):
        del goal, include_orientation
        from moveit_msgs.msg import Constraints

        return Constraints()

    def _parameters(self, phase: str):
        from moveit.planning import PlanRequestParameters

        pipeline, planner_id, speed, plan_time = self.PHASES.get(
            phase, self.PHASES['transfer'],
        )
        parameters = PlanRequestParameters(self._moveit, self.planning_group)
        parameters.planning_pipeline = pipeline
        parameters.planner_id = planner_id
        parameters.planning_time = plan_time
        parameters.planning_attempts = 10 if pipeline == 'ompl' else 3
        parameters.max_velocity_scaling_factor = speed
        parameters.max_acceleration_scaling_factor = speed
        return parameters

    # ------------------------------------------------------------------ plan
    def plan(
        self,
        goal: Transform,
        phase: str = 'transfer',
        held_object: str | None = None,
    ) -> PlannedMotion:
        self._ensure()
        if self._stopped:
            raise PlannerError('planner is stopped')
        base_goal = self._goal_in_pose_frame(goal)
        effective_phase = phase
        print(
            '[iiwa_llm_plan_executor] plan request '
            f'phase={phase!r} goal_xyz={tuple(round(float(v), 6) for v in base_goal.xyz)} '
            f'goal_quat={tuple(round(float(v), 6) for v in Rotation.from_matrix(base_goal.matrix).as_quat())}',
            flush=True,
        )
        try:
            self._arm.set_start_state_to_current_state()
            start_state = self._arm.get_start_state()
            names = list(self._group.active_joint_model_names)
            measured = tuple(
                float(value)
                for value in start_state.get_joint_group_positions(
                    self.planning_group,
                )
            )
            bounds = list(self._group.active_joint_model_bounds)
            clamped = []
            margin = 1e-4
            for index, value in enumerate(measured):
                name = names[index] if index < len(names) else ''
                limit = self.joint_limits.get(name)
                if limit is not None:
                    low, high = limit
                else:
                    try:
                        low, high = bounds[index]
                    except (IndexError, TypeError, ValueError):
                        low, high = -3.2, 3.2
                low = float(low) + margin
                high = float(high) - margin
                clamped.append(min(max(value, low), high))
            if any(abs(a - b) > 1e-9 for a, b in zip(clamped, measured)):
                print(
                    '[iiwa_llm_plan_executor] clamping start state '
                    f'measured={measured} clamped={clamped} bounds={bounds}',
                    flush=True,
                )
            from moveit.core.robot_state import RobotState

            clamped_state = RobotState(self._model)
            clamped_state.set_joint_group_positions(
                self.planning_group, clamped,
            )
            clamped_state.update()
            self._arm.set_start_state(robot_state=clamped_state)
            start_state = self._arm.get_start_state()
            current = tuple(clamped)
            print(
                '[iiwa_llm_plan_executor] plan start '
                f'measured={tuple(round(float(v), 6) for v in measured)} '
                f'clamped={tuple(round(float(v), 6) for v in current)}',
                flush=True,
            )
            start_matrix = start_state.get_global_link_transform(self.pose_link)
            start_tcp = _matrix_transform(
                start_matrix, self.pose_frame, self.pose_link,
            )
            reorientation = (
                tcp_tilt_deg(start_tcp) > self.policy.limits.max_tilt_deg
                and held_object is None
            )
            effective_phase = phase
            # J7 has no artificial +/-90 deg corridor and PTP/LIN goal IK is
            # allowed to use the whole joint range.  Safety comes from the
            # dense post-plan joint/self/world/orientation checks below.
            from moveit_msgs.msg import Constraints
            self._arm.set_path_constraints(Constraints())
            phase_params = self.PHASES.get(
                effective_phase, self.PHASES['transfer'],
            )
            print(
                '[iiwa_llm_plan_executor] plan pipeline '
                f'phase={phase!r} pipeline={phase_params[0]!r} '
                f'planner_id={phase_params[1]!r} planning_time={phase_params[3]}',
                flush=True,
            )
            self._arm.set_goal_state(
                pose_stamped_msg=_pose_message(base_goal, self.pose_frame),
                pose_link=self.pose_link,
            )
            result = self._arm.plan(
                single_plan_parameters=self._parameters(effective_phase),
            )
        except PlannerError:
            raise
        except Exception as error:
            print(
                '[iiwa_llm_plan_executor] plan exception '
                f'phase={phase!r} effective_phase={effective_phase!r} '
                f'error={error!r}',
                flush=True,
            )
            raise PlannerError(f'constrained planning failed: {error}') from error
        if result is None or result.trajectory is None:
            error_code = (
                None if result is None
                else getattr(result.error_code, 'val', None)
            )
            error_name = _error_code_name(error_code)
            print(
                '[iiwa_llm_plan_executor] plan empty '
                f'phase={phase!r} pipeline={effective_phase!r} '
                f'error_code={error_name} goal_xyz={base_goal.xyz}',
                flush=True,
            )
            raise PlannerError(
                f'planning failed for phase {phase!r} '
                f'({effective_phase}, error {error_name}); '
                f'goal_xyz={tuple(round(float(v), 6) for v in base_goal.xyz)} '
                f'start={tuple(round(float(v), 6) for v in current)}'
            )
        message = result.trajectory.get_robot_trajectory_msg()
        names = list(message.joint_trajectory.joint_names)
        expected = list(self._group.active_joint_model_names)
        if set(expected) - set(names):
            raise PlannerError(
                f'planned trajectory misses joints {sorted(set(expected) - set(names))}'
            )
        index = {name: position for position, name in enumerate(names)}
        raw_points: list[tuple[float, ...]] = []
        for waypoint in message.joint_trajectory.points:
            raw_points.append(tuple(
                float(waypoint.positions[index[name]]) for name in expected
            ))
        points = self._densify(
            raw_points, self.policy.limits.max_sample_step_rad,
        )
        poses: list[Transform] = []
        from moveit.core.robot_state import RobotState

        state = RobotState(self._model)
        for ordered in points:
            state.set_joint_group_positions(self.planning_group, ordered)
            state.update()
            matrix = state.get_global_link_transform(self.pose_link)
            poses.append(_matrix_transform(matrix, self.pose_frame, self.pose_link))
        clearance = self._check_collisions(points)
        return PlannedMotion(
            points=points,
            tcp_poses=poses,
            current_joints=current,
            start_tcp_pose=start_tcp,
            handle=result,
            details={
                'phase': phase,
                'planner_phase': effective_phase,
                'planner': self.PHASES.get(
                    effective_phase, self.PHASES['transfer'],
                )[:2],
                'reorientation': reorientation,
                'raw_points': len(raw_points),
                'sampled_points': len(points),
                'min_clearance_m': self.policy.limits.min_clearance_m,
                'measured_clearance_m': clearance,
            },
        )

    @staticmethod
    def _densify(
        points: list[tuple[float, ...]],
        max_step: float,
    ) -> list[tuple[float, ...]]:
        if not points:
            return []
        result = [points[0]]
        for previous, current_point in zip(points, points[1:]):
            delta = max(
                abs(a - b) for a, b in zip(current_point, previous)
            )
            steps = max(1, int(math.ceil(delta / max_step)))
            for step in range(1, steps + 1):
                fraction = step / steps
                result.append(tuple(
                    a + (b - a) * fraction
                    for a, b in zip(previous, current_point)
                ))
        return result

    def _check_collisions(self, points: list[tuple[float, ...]]) -> float | None:
        if self._scene_monitor is None:
            raise PlannerError('planning scene monitor is unavailable')
        from moveit.core.collision_detection import (
            CollisionRequest,
            CollisionResult,
        )
        from moveit.core.robot_state import RobotState

        requirement = float(self.policy.limits.min_clearance_m)
        measured_min: float | None = None
        state = RobotState(self._model)
        with self._scene_monitor.read_only() as scene:
            acm = getattr(scene, 'allowed_collision_matrix', None)
            for index, sample in enumerate(points):
                state.set_joint_group_positions(self.planning_group, sample)
                state.update()
                request = CollisionRequest()
                request.distance = True
                request.contacts = True
                request.max_contacts = 20
                request.max_contacts_per_pair = 2
                collided = False
                checked = False
                last_error = None
                distances: list[float] = []
                contacts: list[str] = []
                for function in (
                    'check_collision_unpadded', 'check_self_collision',
                ):
                    for arguments in (
                        (request, CollisionResult(), state, acm),
                        (request, CollisionResult(), state),
                    ):
                        if acm is None and len(arguments) == 4:
                            continue
                        try:
                            result = arguments[1]
                            hit = bool(getattr(scene, function)(*arguments))
                            checked = True
                            collided = collided or hit
                        except TypeError as error:
                            last_error = error
                            continue
                        try:
                            distances.append(float(result.distance))
                        except Exception:
                            pass
                        formatted = _format_contacts(result)
                        if formatted != 'unknown pair':
                            contacts.append(formatted)
                if not checked:
                    raise PlannerError(
                        'collision checking is unavailable: '
                        f'{last_error}'
                    )
                distance = min(distances) if distances else None
                if distance is not None:
                    measured_min = (
                        distance if measured_min is None
                        else min(measured_min, distance)
                    )
                if collided or (
                    distance is not None and distance < requirement
                ):
                    pairs = ', '.join(sorted(set(contacts))) or 'unknown pair'
                    measured = (
                        'unknown' if distance is None else f'{distance:.4f} m'
                    )
                    raise PlannerError(
                        f'collision/clearance failure at sample {index}: '
                        f'{pairs}; minimum distance {measured}, '
                        f'required {requirement:.4f} m'
                    )
        return measured_min

    def execute(self, planned: PlannedMotion) -> bool:
        self._ensure()
        try:
            execution = self._moveit.execute(
                planned.handle.trajectory, controllers=[],
            )
            return bool(execution)
        except Exception as error:
            raise PlannerError(f'trajectory execution failed: {error}') from error

    def stop(self) -> None:
        self._stopped = True
        if self._moveit is None:
            return
        try:
            self._moveit.get_trajectory_execution_manager().stop_execution()
        except Exception:
            pass

    def reset_stop(self) -> None:
        self._stopped = False


class MoveItBackend:
    """The single execution path used by every skill."""

    def __init__(
        self,
        geometry: GeometrySpec,
        policy: MotionPolicy,
        *,
        planner: Any | None = None,
        freshness_guard: Callable[[], tuple[bool, str]] | None = None,
        scene_guard: Callable[[], tuple[bool, str]] | None = None,
        current_joints: Callable[[], tuple[float, ...] | None] | None = None,
        before_plan: Callable[[], None] | None = None,
        collision_checker: Callable[[Any], str | None] | None = None,
        on_event: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self.geometry = geometry
        self.policy = policy
        self.planner = planner or MoveItPyPlanner(geometry, policy)
        self._freshness_guard = freshness_guard
        self._scene_guard = scene_guard
        self._current_joints = current_joints
        self._before_plan = before_plan
        self._collision_checker = collision_checker
        self._on_event = on_event
        self.stopped = False
        self.stop_reason: str | None = None

    def _emit(self, **payload) -> None:
        if self._on_event is not None:
            self._on_event(payload)

    def _guard(self) -> MotionOutcome | None:
        if self.stopped:
            return MotionOutcome(
                False, 'stopped',
                f'motion is blocked after stop: {self.stop_reason}',
            )
        for name, guard in (
            ('joint_states', self._freshness_guard),
            ('planning_scene', self._scene_guard),
        ):
            if guard is None:
                continue
            try:
                ok, message = guard()
            except Exception as error:
                return self._fail(f'{name} guard failed: {error}', 'stale_state')
            if not ok:
                return self._fail(message, 'stale_state')
        return None

    def _fail(self, message: str, code: str = 'failed', **details) -> MotionOutcome:
        self.stop(message)
        self._emit(event='motion_failed', code=code, message=message, **details)
        return MotionOutcome(False, code, message, details)

    def plan_and_execute(
        self,
        goal: Transform,
        phase: str = 'transfer',
        held_object: str | None = None,
    ) -> MotionOutcome:
        blocked = self._guard()
        if blocked is not None:
            return blocked
        if self._before_plan is not None:
            try:
                self._before_plan()
            except Exception as error:
                return self._fail(f'settle wait failed: {error}', 'stale_state')
        planned = None
        check: MotionCheck | None = None
        last_error = ''
        for attempt in range(3):
            try:
                candidate = self.planner.plan(goal, phase, held_object)
            except Exception as error:
                last_error = str(error)
                if attempt + 1 < 3:
                    try:
                        self._before_plan and self._before_plan()
                    except Exception:
                        pass
                    continue
                return self._fail(last_error, 'planning_failed')
            current = None
            if self._current_joints is not None:
                try:
                    current = self._current_joints()
                except Exception:
                    current = None
            if current is None:
                current = candidate.current_joints
            candidate_check = self.policy.validate_trajectory(
                candidate.points, candidate.tcp_poses, current,
                start_tcp_pose=candidate.start_tcp_pose,
                held_object=held_object,
            )
            if candidate_check.ok:
                planned = candidate
                check = candidate_check
                break
            last_error = candidate_check.reason
            if attempt + 1 < 3:
                try:
                    self._before_plan and self._before_plan()
                except Exception:
                    pass
        if planned is None or check is None:
            try:
                self.planner.stop()
            except Exception:
                pass
            return self._fail(
                f'motion policy rejected every planned trajectory: {last_error}',
                'motion_limit',
            )
        if self._collision_checker is not None:
            try:
                collision_reason = self._collision_checker(planned)
            except Exception as error:
                collision_reason = f'collision check failed: {error}'
            if collision_reason:
                try:
                    self.planner.stop()
                except Exception:
                    pass
                return self._fail(collision_reason, 'collision')
        target = (
            goal
            if goal.parent == 'base_link'
            else self.geometry.world_to_base.inverse().compose_relaxed(goal)
        )
        final = planned.tcp_poses[-1]
        distance = float(np.linalg.norm(final.position - target.position))
        cosine = (np.trace(final.matrix.T @ target.matrix) - 1.0) / 2.0
        angle = float(math.acos(max(-1.0, min(1.0, cosine))))
        if (
            distance > self.geometry.position_tolerance
            or angle > self.geometry.orientation_tolerance_rad
        ):
            return self._fail(
                'planned final tcp is outside the goal tolerance '
                f'({distance:.4f} m, {angle:.4f} rad)',
                'goal_mismatch',
            )
        self._emit(
            event='motion_planned', phase=phase, held_object=held_object,
            planner_details=dict(planned.details), **check.details,
        )
        try:
            executed = self.planner.execute(planned)
        except Exception as error:
            return self._fail(str(error), 'execution_failed')
        if not executed:
            return self._fail('trajectory execution did not succeed', 'execution_failed')
        self._emit(event='motion_executed', phase=phase, **check.details)
        return MotionOutcome(
            True,
            'ok',
            'motion executed',
            {'phase': phase, **check.details},
        )

    def stop(self, reason: str = 'stop requested') -> None:
        self.stopped = True
        if self.stop_reason is None:
            self.stop_reason = reason
        try:
            self.planner.stop()
        except Exception:
            pass

    def reset_stop(self) -> None:
        self.stopped = False
        self.stop_reason = None
        reset = getattr(self.planner, 'reset_stop', None)
        if reset is not None:
            reset()
