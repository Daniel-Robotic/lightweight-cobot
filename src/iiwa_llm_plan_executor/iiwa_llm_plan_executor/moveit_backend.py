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
from typing import Any, Callable

import numpy as np
from scipy.spatial.transform import Rotation

from iiwa_experement.geometry import GeometrySpec, Transform

from .motion_policy import MotionCheck, MotionPolicy


class PlannerError(RuntimeError):
    """The MoveIt plan could not be produced or executed."""


@dataclass
class PlannedMotion:
    points: list[tuple[float, ...]]
    tcp_poses: list[Transform]
    current_joints: tuple[float, ...] | None = None
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


class MoveItPyPlanner:
    """Thin wrapper around MoveItPy with the constrained plan request."""

    PHASES = {
        'transfer': ('pilz_industrial_motion_planner', 'PTP', 0.35, 2.0),
        'approach': ('pilz_industrial_motion_planner', 'PTP', 0.18, 2.0),
        'retreat': ('pilz_industrial_motion_planner', 'PTP', 0.18, 2.0),
        'observe': ('pilz_industrial_motion_planner', 'PTP', 0.18, 2.0),
        'contact': ('pilz_industrial_motion_planner', 'LIN', 0.06, 2.0),
        'lower': ('pilz_industrial_motion_planner', 'LIN', 0.06, 2.0),
        'align': ('pilz_industrial_motion_planner', 'LIN', 0.06, 2.0),
        'dispense': ('pilz_industrial_motion_planner', 'LIN', 0.09, 2.0),
        'escape': ('ompl', 'RRTConnectkConfigDefault', 0.20, 10.0),
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
    ) -> None:
        self.geometry = geometry
        self.policy = policy
        self.planning_group = planning_group
        self.pose_link = pose_link
        self.pose_frame = pose_frame
        self._moveit_factory = moveit_factory
        self._moveit = None
        self._arm = None
        self._model = None
        self._group = None
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

    def _constraints(self, goal: Transform):
        from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint

        limits = self.policy.limits
        constraints = Constraints()
        joint = JointConstraint()
        joint.joint_name = 'joint_7'
        joint.position = float(limits.work_joint7_rad)
        joint.tolerance_above = float(limits.joint7_corridor_rad)
        joint.tolerance_below = float(limits.joint7_corridor_rad)
        joint.weight = 1.0
        constraints.joint_constraints.append(joint)

        # The model uses the same joint order as the Webots controller.  With
        # a hard joint-7 corridor, this constraint prevents the IK branch that
        # would flip joint 7 by about -180 degrees.
        orientation = OrientationConstraint()
        orientation.header.frame_id = self.pose_frame
        orientation.link_name = self.pose_link
        quaternion = Rotation.from_matrix(goal.matrix).as_quat()
        orientation.orientation.x = float(quaternion[0])
        orientation.orientation.y = float(quaternion[1])
        orientation.orientation.z = float(quaternion[2])
        orientation.orientation.w = float(quaternion[3])
        tilt = math.radians(float(limits.max_tilt_deg))
        orientation.absolute_x_axis_tolerance = tilt
        orientation.absolute_y_axis_tolerance = tilt
        orientation.absolute_z_axis_tolerance = tilt
        orientation.weight = 1.0
        constraints.orientation_constraints.append(orientation)
        return constraints

    def _parameters(self, phase: str):
        from moveit.planning import PlanRequestParameters

        pipeline, planner_id, speed, plan_time = self.PHASES.get(
            phase, self.PHASES['transfer'],
        )
        parameters = PlanRequestParameters(self._moveit, self.planning_group)
        parameters.planning_pipeline = pipeline
        parameters.planner_id = planner_id
        parameters.planning_time = plan_time
        parameters.planning_attempts = 3
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
        del held_object  # attached geometry is maintained by the scene bridge
        self._ensure()
        if self._stopped:
            raise PlannerError('planner is stopped')
        base_goal = self._goal_in_pose_frame(goal)
        try:
            self._arm.set_start_state_to_current_state()
            start_state = self._arm.get_start_state()
            current = tuple(
                float(value)
                for value in start_state.get_joint_group_positions(
                    self.planning_group,
                )
            )
            self._arm.set_path_constraints(self._constraints(base_goal))
            self._arm.set_goal_state(
                pose_stamped_msg=_pose_message(base_goal, self.pose_frame),
                pose_link=self.pose_link,
            )
            result = self._arm.plan(
                single_plan_parameters=self._parameters(phase),
            )
        except PlannerError:
            raise
        except Exception as error:
            raise PlannerError(f'constrained planning failed: {error}') from error
        if not result or result.trajectory is None:
            raise PlannerError(
                f'no constrained trajectory for phase {phase!r} '
                f'(joint7 corridor {self.policy.limits.joint7_min_rad:.3f}..'
                f'{self.policy.limits.joint7_max_rad:.3f} rad)'
            )
        message = result.trajectory.get_robot_trajectory_msg()
        names = list(message.joint_trajectory.joint_names)
        expected = list(self._group.active_joint_model_names)
        if set(expected) - set(names):
            raise PlannerError(
                f'planned trajectory misses joints {sorted(set(expected) - set(names))}'
            )
        index = {name: position for position, name in enumerate(names)}
        points: list[tuple[float, ...]] = []
        poses: list[Transform] = []
        from moveit.core.robot_state import RobotState

        state = RobotState(self._model)
        for waypoint in message.joint_trajectory.points:
            ordered = tuple(
                float(waypoint.positions[index[name]]) for name in expected
            )
            points.append(ordered)
            state.set_joint_group_positions(self.planning_group, ordered)
            state.update()
            matrix = state.get_global_link_transform(self.pose_link)
            poses.append(_matrix_transform(matrix, self.pose_frame, self.pose_link))
        return PlannedMotion(
            points=points,
            tcp_poses=poses,
            current_joints=current,
            handle=result,
            details={
                'phase': phase,
                'planner': self.PHASES.get(phase, self.PHASES['transfer'])[:2],
            },
        )

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
        on_event: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        self.geometry = geometry
        self.policy = policy
        self.planner = planner or MoveItPyPlanner(geometry, policy)
        self._freshness_guard = freshness_guard
        self._scene_guard = scene_guard
        self._current_joints = current_joints
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
        try:
            planned = self.planner.plan(goal, phase, held_object)
        except Exception as error:
            return self._fail(str(error), 'planning_failed')
        current = None
        if self._current_joints is not None:
            try:
                current = self._current_joints()
            except Exception:
                current = None
        if current is None:
            current = planned.current_joints
        check: MotionCheck = self.policy.validate_trajectory(
            planned.points, planned.tcp_poses, current,
        )
        if not check.ok:
            try:
                self.planner.stop()
            except Exception:
                pass
            return self._fail(
                f'motion policy rejected the trajectory: {check.reason}',
                'motion_limit',
                **check.details,
            )
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
            **check.details,
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
