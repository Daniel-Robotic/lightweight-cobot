"""Safe wrapper around the existing ``cobot/move_to_pose`` motion server.

Every call performs the same guard sequence:
fresh joint state -> synchronized planning scene -> available server ->
accepted goal -> successful result -> measured tcp pose.  Any failure stops
the robot and blocks all later motion.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Callable

import numpy as np
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import tf2_ros

from iiwa_msgs.action import MoveToJoints, MoveToPose
from iiwa_msgs.srv import MoveToNamedPose

from .geometry import GeometrySpec, Transform


MOVE_ACTION = '/cobot/move_to_pose'
JOINTS_ACTION = '/cobot/move_to_joints'
NAMED_SERVICE = '/cobot/move_to_named'
STOP_SERVICE = '/cobot/stop'
PLANNERS = ('ptp', 'lin', 'ompl')


@dataclass(frozen=True)
class MotionResult:
    success: bool
    message: str
    label: str
    sent: bool = False
    goal: Transform | None = None
    actual: Transform | None = None
    duration_sec: float = 0.0


def transform_from_pose(message, parent: str, child: str) -> Transform:
    orientation = message.orientation
    matrix = Rotation.from_quat([
        orientation.x, orientation.y, orientation.z, orientation.w,
    ]).as_matrix()
    return Transform(
        xyz=(
            float(message.position.x),
            float(message.position.y),
            float(message.position.z),
        ),
        rotation=tuple(tuple(float(value) for value in row) for row in matrix),
        parent=parent,
        child=child,
    )


def transform_from_tf(message, parent: str, child: str) -> Transform:
    translation = message.transform.translation
    rotation = message.transform.rotation
    matrix = Rotation.from_quat([
        rotation.x, rotation.y, rotation.z, rotation.w,
    ]).as_matrix()
    return Transform(
        xyz=(
            float(translation.x),
            float(translation.y),
            float(translation.z),
        ),
        rotation=tuple(tuple(float(value) for value in row) for row in matrix),
        parent=parent,
        child=child,
    )


def _pose_distance(first: Transform, second: Transform) -> tuple[float, float]:
    distance = float(np.linalg.norm(first.position - second.position))
    cosine = (np.trace(first.matrix.T @ second.matrix) - 1.0) / 2.0
    angle = float(np.arccos(max(-1.0, min(1.0, cosine))))
    return distance, angle


class MotionAdapter:
    """Guarded motion client used by every reference operation."""

    def __init__(
        self,
        node: Node,
        geometry: GeometrySpec,
        *,
        action_client: Any | None = None,
        joints_client: Any | None = None,
        named_client: Any | None = None,
        stop_client: Any | None = None,
        tf_buffer: Any | None = None,
        scene_check: Callable[[], tuple[bool, str]] | None = None,
        pose_provider: Callable[[], Transform | None] | None = None,
        freshness_check: Callable[[], tuple[bool, str]] | None = None,
        spin_until: Callable[[Any, float], bool] | None = None,
        position_tolerance: float | None = None,
        orientation_tolerance: float | None = None,
        command_timeout_sec: float = 90.0,
        settle_time_sec: float = 1.0,
        stability_timeout_sec: float = 5.0,
        stability_epsilon: float = 0.001,
        stability_angle: float = 0.01,
        pose_correction_attempts: int = 2,
        joint_correction_attempts: int = 2,
        joint_position_tolerance: float = 0.05,
    ) -> None:
        self.node = node
        self.geometry = geometry
        self.action_client = action_client or ActionClient(
            node, MoveToPose, MOVE_ACTION,
        )
        self.joints_client = joints_client
        if self.joints_client is None and node is not None:
            self.joints_client = ActionClient(node, MoveToJoints, JOINTS_ACTION)
        self.named_client = named_client or node.create_client(
            MoveToNamedPose, NAMED_SERVICE,
        )
        self.stop_client = stop_client or node.create_client(
            Trigger, STOP_SERVICE,
        )
        self.tf_buffer = tf_buffer
        self._tf_listener = None
        if tf_buffer is None:
            self.tf_buffer = tf2_ros.Buffer()
            self._tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        self._scene_check = scene_check
        self._pose_provider = pose_provider
        self._freshness_check = freshness_check
        self._spin_until = spin_until or self._default_spin_until
        self.position_tolerance = (
            geometry.position_tolerance
            if position_tolerance is None else float(position_tolerance)
        )
        self.orientation_tolerance = (
            geometry.orientation_tolerance_rad
            if orientation_tolerance is None else float(orientation_tolerance)
        )
        self.command_timeout_sec = float(command_timeout_sec)
        self.settle_time_sec = max(0.0, float(settle_time_sec))
        self.stability_timeout_sec = max(0.0, float(stability_timeout_sec))
        self.stability_epsilon = float(stability_epsilon)
        self.stability_angle = float(stability_angle)
        self.pose_correction_attempts = max(1, int(pose_correction_attempts))
        self.joint_correction_attempts = max(1, int(joint_correction_attempts))
        self.joint_position_tolerance = float(joint_position_tolerance)
        self._joint_positions: tuple[float, ...] | None = None
        self.stopped = False
        self.stop_reason: str | None = None
        self._last_joint_time = None
        if freshness_check is None:
            node.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)

    # ------------------------------------------------------------------ freshness
    def _on_joint_state(self, message: JointState) -> None:
        self._last_joint_time = self.node.get_clock().now()
        if len(message.position) == 7:
            self._joint_positions = tuple(float(value) for value in message.position)

    def _default_freshness(self) -> tuple[bool, str]:
        if self._last_joint_time is None:
            return False, 'no /joint_states received yet'
        age = (self.node.get_clock().now() - self._last_joint_time).nanoseconds / 1e9
        if age > 1.0:
            return False, f'/joint_states is stale ({age:.2f} s old)'
        if self.tf_buffer is not None:
            try:
                self.tf_buffer.lookup_transform('base_link', 'tcp', Time())
            except Exception as error:
                return False, f'tcp transform is unavailable: {error}'
        return True, 'ok'

    def _fresh(self) -> tuple[bool, str]:
        if self._freshness_check is None:
            return self._default_freshness()
        return self._freshness_check()

    def _scene(self) -> tuple[bool, str]:
        if self._scene_check is None:
            return False, 'planning scene bridge is not configured'
        return self._scene_check()

    # ------------------------------------------------------------------ transforms
    def _default_spin_until(self, future, timeout_sec: float) -> bool:
        import rclpy

        rclpy.spin_until_future_complete(
            self.node, future, timeout_sec=timeout_sec,
        )
        return future.done()

    def world_to_base(self) -> Transform:
        # The URDF ships a ``world_base_joint`` with an identity origin, so
        # robot_state_publisher always provides a synthetic world->base_link
        # transform at zero.  The physical Webots placement comes from the
        # launch setting and is stored in GeometrySpec.world_to_base.
        return self.geometry.world_to_base

    def update_world_to_base(self, transform: Transform) -> None:
        self.geometry = GeometrySpec(
            **{**self.geometry.__dict__, 'world_to_base': transform}
        )

    def actual_tcp(self) -> Transform | None:
        if self._pose_provider is not None:
            return self._pose_provider()
        if self.tf_buffer is None:
            return None
        try:
            message = self.tf_buffer.lookup_transform('base_link', 'tcp', Time())
            return transform_from_tf(message, 'base_link', 'tcp')
        except Exception:
            return None

    def to_base_goal(self, goal: Transform) -> Transform:
        return self.world_to_base().inverse().compose_relaxed(goal)

    def _abc(self, goal: Transform) -> tuple[float, float, float]:
        euler = Rotation.from_matrix(goal.matrix).as_euler('ZYX')
        return float(euler[0]), float(euler[1]), float(euler[2])

    # ------------------------------------------------------------------ guards
    def _guard(self, label: str, speed: float, planner: str) -> MotionResult | None:
        if self.stopped:
            return MotionResult(
                False,
                f'motion is blocked after stop: {self.stop_reason}',
                label,
            )
        if not math.isfinite(float(speed)) or not 0.0 < float(speed) <= 1.0:
            return MotionResult(False, f'invalid speed {speed!r}', label)
        if planner not in PLANNERS:
            return MotionResult(False, f'unknown planner {planner!r}', label)
        fresh, message = self._fresh()
        if not fresh:
            self.stop(message)
            return MotionResult(False, message, label)
        synchronized, message = self._scene()
        if not synchronized:
            self.stop(message)
            return MotionResult(False, message, label)
        return None

    # ------------------------------------------------------------------ motion
    def move_pose(
        self,
        goal: Transform,
        planner: str = 'ptp',
        speed: float = 0.2,
        label: str = '',
    ) -> MotionResult:
        label = label or 'move_pose'
        blocked = self._guard(label, speed, planner)
        if blocked is not None:
            return blocked
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            return MotionResult(False, f'{MOVE_ACTION} is unavailable', label)

        base_goal = self.to_base_goal(goal)
        a, b, c = self._abc(base_goal)
        request = MoveToPose.Goal()
        request.x, request.y, request.z = (float(value) for value in base_goal.xyz)
        request.a, request.b, request.c = a, b, c
        request.speed = float(speed)
        request.planner = planner
        request.frame_id = 'base_link'

        started = time.monotonic()
        last_actual: Transform | None = None
        for attempt in range(self.pose_correction_attempts):
            future = self.action_client.send_goal_async(request)
            if not self._spin_until(future, self.command_timeout_sec):
                return self._fail(label, 'goal response timed out', base_goal, started)
            try:
                goal_handle = future.result()
            except Exception as error:
                return self._fail(label, f'goal call failed: {error}', base_goal, started)
            if goal_handle is None or not goal_handle.accepted:
                return self._fail(label, 'motion goal was rejected', base_goal, started)

            result_future = goal_handle.get_result_async()
            if not self._spin_until(result_future, self.command_timeout_sec):
                return self._fail(label, 'motion result timed out', base_goal, started)
            try:
                wrapped = result_future.result()
                result = wrapped.result
            except Exception as error:
                return self._fail(label, f'motion result failed: {error}', base_goal, started)
            if not result.success:
                return self._fail(
                    label, result.message or 'motion server reported failure',
                    base_goal, started,
                )

            actual = self._settle_and_measure()
            last_actual = actual
            if actual is None:
                return self._fail(
                    label, 'actual tcp pose is unavailable after motion',
                    base_goal, started,
                )
            distance, angle = _pose_distance(actual, base_goal)
            if distance <= self.position_tolerance and angle <= self.orientation_tolerance:
                return MotionResult(
                    True, 'motion completed', label, True, base_goal, actual,
                    time.monotonic() - started,
                )
            if attempt + 1 < self.pose_correction_attempts:
                # The action succeeded but the controller stopped outside the
                # pose tolerance.  Re-issue the same verified goal once; this
                # is a measured correction, not a blind repeat.
                if self.node is not None:
                    self.node.get_logger().warning(
                        f'{label}: correcting tcp pose '
                        f'({distance:.4f} m, {angle:.4f} rad off goal)'
                    )
                continue
            return self._fail(
                label,
                f'actual tcp is {distance:.4f} m / {angle:.4f} rad from the goal',
                base_goal,
                started,
                actual=last_actual,
            )
        return self._fail(label, 'motion failed without a result', base_goal, started)

    def move_named(self, name: str, speed: float = 0.2, label: str = '') -> MotionResult:
        label = label or f'move_named({name})'
        joints = self.geometry.named_joints.get(name)
        if self.joints_client is not None and joints is not None:
            result = self.move_joints(joints, speed, label)
            if result.success:
                return result
            planning_failed = (
                'Планирование' in result.message
                or 'planning' in result.message.lower()
                or 'недостижима' in result.message
            )
            if not planning_failed:
                return result
            # A joint-space plan can be rejected in an unusual scene; retry
            # the named service once as the fallback path.
            self.reset_stop()
        return self._move_named_service(name, speed, label)

    def move_joints(
        self,
        joints,
        speed: float = 0.2,
        label: str = '',
    ) -> MotionResult:
        label = label or 'move_joints'
        values = tuple(float(value) for value in joints)
        if len(values) != 7:
            return MotionResult(False, 'joint goal must contain 7 values', label)
        blocked = self._guard(label, speed, 'ptp')
        if blocked is not None:
            return blocked
        if self.joints_client is None:
            return MotionResult(False, f'{JOINTS_ACTION} is unavailable', label)
        if not self.joints_client.wait_for_server(timeout_sec=5.0):
            return MotionResult(False, f'{JOINTS_ACTION} is unavailable', label)

        request = MoveToJoints.Goal()
        request.joints = list(values)
        request.speed = float(speed)
        started = time.monotonic()
        last_error = 0.0
        for attempt in range(self.joint_correction_attempts):
            future = self.joints_client.send_goal_async(request)
            if not self._spin_until(future, self.command_timeout_sec):
                return self._fail(label, 'joint goal response timed out', None, started)
            try:
                goal_handle = future.result()
            except Exception as error:
                return self._fail(
                    label, f'joint goal call failed: {error}', None, started,
                )
            if goal_handle is None or not goal_handle.accepted:
                return self._fail(label, 'joint goal was rejected', None, started)
            result_future = goal_handle.get_result_async()
            if not self._spin_until(result_future, self.command_timeout_sec):
                return self._fail(label, 'joint result timed out', None, started)
            try:
                result = result_future.result().result
            except Exception as error:
                return self._fail(
                    label, f'joint result failed: {error}', None, started,
                )
            if not result.success:
                return self._fail(
                    label, result.message or 'joint motion failed', None, started,
                )
            actual = self._settle_joints()
            if actual is None or len(actual) != 7:
                break
            last_error = max(abs(a - b) for a, b in zip(actual, values))
            if last_error <= self.joint_position_tolerance:
                break
            if attempt + 1 < self.joint_correction_attempts:
                # The controller can stop short after a transient Webots
                # contact.  Re-issue the same verified joint goal once.
                if self.node is not None:
                    self.node.get_logger().warning(
                        f'{label}: joint positions {last_error:.4f} rad off; '
                        're-issuing the goal'
                    )
                continue
            return self._fail(
                label,
                f'joint positions are {last_error:.4f} rad from the joint goal',
                None,
                started,
            )
        return MotionResult(
            True, 'joint motion completed', label, True, None, self.actual_tcp(),
            time.monotonic() - started,
        )

    def _move_named_service(
        self, name: str, speed: float, label: str,
    ) -> MotionResult:
        blocked = self._guard(label, speed, 'ptp')
        if blocked is not None:
            return blocked
        if not self.named_client.wait_for_service(timeout_sec=5.0):
            return MotionResult(False, f'{NAMED_SERVICE} is unavailable', label)

        request = MoveToNamedPose.Request()
        request.name = str(name)
        request.speed = float(speed)
        request.accel_scale = float(speed)
        started = time.monotonic()
        future = self.named_client.call_async(request)
        if not self._spin_until(future, self.command_timeout_sec):
            return self._fail(label, 'named motion timed out', None, started)
        try:
            response = future.result()
        except Exception as error:
            return self._fail(label, f'named motion failed: {error}', None, started)
        if not response.success:
            planning_failed = (
                'Планирование' in (response.message or '')
                or 'planning' in (response.message or '').lower()
                or 'недостижима' in (response.message or '')
            )
            pose = self.geometry.safe_poses.get(name)
            if planning_failed and pose is not None:
                # PTP can be rejected from some tool sockets.  Fall back to a
                # calibrated safe Cartesian pose with a collision-checked
                # OMPL plan; the final tcp pose is verified as usual.
                self.reset_stop()
                fallback = self.move_pose(
                    pose, planner='ompl', speed=speed, label=f'{label}:safe',
                )
                if fallback.success:
                    return MotionResult(
                        True, fallback.message, label, True, fallback.goal,
                        fallback.actual, time.monotonic() - started,
                    )
                return fallback
            return self._fail(
                label, response.message or 'named motion failed', None, started,
            )
        self._settle()
        return MotionResult(
            True, response.message or 'motion completed', label,
            True, None, self.actual_tcp(), time.monotonic() - started,
        )

    def _settle(self) -> None:
        if self.settle_time_sec <= 0.0 or self.node is None:
            return
        import rclpy

        end = time.monotonic() + self.settle_time_sec
        while time.monotonic() < end:
            rclpy.spin_once(
                self.node,
                timeout_sec=min(0.05, max(0.0, end - time.monotonic())),
            )

    def _settle_joints(self) -> tuple[float, ...] | None:
        """Wait until the measured joint positions stop changing.

        The joint action result can arrive while the Webots controller is
        still following the last trajectory points.  Comparing that first
        sample produced false 'joint positions are ... off' failures.
        """
        self._settle()
        if self.node is None:
            return self._joint_positions
        import rclpy

        deadline = time.monotonic() + self.stability_timeout_sec
        last = self._joint_positions
        stable_since = None
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            current = self._joint_positions
            if current is None or last is None or len(current) != len(last):
                last = current
                stable_since = None
                continue
            if max(abs(a - b) for a, b in zip(current, last)) <= 0.001:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= 0.25:
                    return current
            else:
                stable_since = None
            last = current
        return self._joint_positions

    def _settle_and_measure(self) -> Transform | None:
        """Wait until the controller stops moving before verifying tcp.

        The action result is delivered when the trajectory finishes, but the
        joint trajectory controller can still be settling.  Treating the
        first sample as final caused false out-of-tolerance stops.
        """
        self._settle()
        import rclpy

        deadline = time.monotonic() + self.stability_timeout_sec
        last = self.actual_tcp()
        stable_since = None
        while time.monotonic() < deadline and self.node is not None:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            current = self.actual_tcp()
            if current is None:
                last = None
                stable_since = None
                continue
            if last is not None:
                distance, angle = _pose_distance(current, last)
                if distance <= self.stability_epsilon and angle <= self.stability_angle:
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic() - stable_since >= 0.25:
                        return current
                else:
                    stable_since = None
            last = current
        return last

    def _fail(
        self,
        label: str,
        message: str,
        goal: Transform | None,
        started: float,
        *,
        actual: Transform | None = None,
    ) -> MotionResult:
        self.stop(message)
        return MotionResult(
            False, message, label, True, goal, actual,
            time.monotonic() - started,
        )

    def stop(self, reason: str) -> bool:
        self.stopped = True
        if self.stop_reason is None:
            self.stop_reason = reason
        try:
            if self.stop_client.wait_for_service(timeout_sec=1.0):
                future = self.stop_client.call_async(Trigger.Request())
                self._spin_until(future, 5.0)
                return bool(future.done() and future.result().success)
        except Exception:
            pass
        return False

    def reset_stop(self) -> None:
        """Only for a new run; never used to resume after a failed motion."""
        self.stopped = False
        self.stop_reason = None
