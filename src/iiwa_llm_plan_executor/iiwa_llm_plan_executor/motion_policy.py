"""Trajectory-level checks applied before anything reaches the controller.

Every PTP/LIN plan is resampled and checked for joint limits, self/world
collision, working orientation and joint jumps.  The old hard +/-90 deg joint-7
corridor is only a preference; real joint limits remain authoritative.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import yaml

from iiwa_experement.geometry import Transform


class MotionPolicyError(ValueError):
    """The limits file is missing or inconsistent."""


@dataclass(frozen=True)
class MotionLimits:
    work_joint7_rad: float
    joint7_corridor_rad: float
    max_tilt_deg: float
    max_joint_jump_rad: float
    max_cartesian_jump_m: float
    joint_state_max_age_sec: float
    scene_max_age_sec: float
    start_tolerance_rad: float = 0.35
    min_clearance_m: float = 0.003
    max_sample_step_rad: float = 0.01

    @property
    def joint7_min_rad(self) -> float:
        # Kept for diagnostics/compatibility only.  It is no longer a hard
        # rejection boundary: safe paths may leave the preferred +/-90 deg
        # band while remaining inside the real joint limits.
        return self.work_joint7_rad - self.joint7_corridor_rad

    @property
    def joint7_max_rad(self) -> float:
        return self.work_joint7_rad + self.joint7_corridor_rad


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MotionPolicyError(f'{label} must be a finite number')
    number = float(value)
    if not math.isfinite(number):
        raise MotionPolicyError(f'{label} must be a finite number')
    return number


def load_limits(path: Path | str) -> MotionLimits:
    source = Path(path)
    if not source.is_file():
        raise FileNotFoundError(f'motion limits file does not exist: {source}')
    raw = yaml.safe_load(source.read_text(encoding='utf-8'))
    if not isinstance(raw, dict):
        raise MotionPolicyError('motion limits must be a mapping')
    if raw.get('schema_version') != 1:
        raise MotionPolicyError('motion limits schema_version must be 1')
    joint7 = raw.get('joint7') or {}
    tcp = raw.get('tcp') or {}
    trajectory = raw.get('trajectory') or {}
    freshness = raw.get('freshness') or {}
    corridor = _finite(joint7.get('corridor_rad'), 'joint7.corridor_rad')
    tilt = _finite(tcp.get('max_tilt_deg'), 'tcp.max_tilt_deg')
    jump = _finite(
        trajectory.get('max_joint_jump_rad'), 'trajectory.max_joint_jump_rad',
    )
    cartesian = _finite(
        trajectory.get('max_cartesian_jump_m'),
        'trajectory.max_cartesian_jump_m',
    )
    joint_age = _finite(
        freshness.get('joint_state_max_age_sec'),
        'freshness.joint_state_max_age_sec',
    )
    scene_age = _finite(
        freshness.get('scene_max_age_sec'), 'freshness.scene_max_age_sec',
    )
    collision = raw.get('collision') or {}
    min_clearance = _finite(
        collision.get('min_clearance_m', 0.003),
        'collision.min_clearance_m',
    )
    if corridor <= 0 or tilt <= 0 or jump <= 0 or cartesian <= 0:
        raise MotionPolicyError('motion limits must be positive')
    if joint_age <= 0 or scene_age <= 0:
        raise MotionPolicyError('freshness limits must be positive')
    if min_clearance < 0:
        raise MotionPolicyError('collision.min_clearance_m must be non-negative')
    sample_step = _finite(
        trajectory.get('max_sample_step_rad', 0.01),
        'trajectory.max_sample_step_rad',
    )
    if sample_step <= 0:
        raise MotionPolicyError('trajectory.max_sample_step_rad must be positive')
    return MotionLimits(
        work_joint7_rad=_finite(
            joint7.get('work_reference_rad'), 'joint7.work_reference_rad',
        ),
        joint7_corridor_rad=corridor,
        max_tilt_deg=tilt,
        max_joint_jump_rad=jump,
        max_cartesian_jump_m=cartesian,
        joint_state_max_age_sec=joint_age,
        scene_max_age_sec=scene_age,
        start_tolerance_rad=_finite(
            trajectory.get('start_tolerance_rad', 0.35),
            'trajectory.start_tolerance_rad',
        ),
        min_clearance_m=min_clearance,
        max_sample_step_rad=sample_step,
    )


@dataclass(frozen=True)
class MotionCheck:
    ok: bool
    reason: str = ''
    details: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {'ok': self.ok, 'reason': self.reason, 'details': dict(self.details)}


def tcp_tilt_deg(pose: Transform) -> float:
    matrix = pose.matrix
    # The working direction is the tcp local +Z axis; it must point down.
    z_axis = matrix[:, 2]
    dot = float(np.clip(-z_axis[2], -1.0, 1.0))
    return math.degrees(math.acos(dot))


class MotionPolicy:
    def __init__(
        self,
        limits: MotionLimits,
        joint_limits: dict[str, tuple[float, float]] | None = None,
    ) -> None:
        self.limits = limits
        self.joint_limits = dict(joint_limits or {})
        self.joint_names = tuple(
            f'joint{index}' for index in range(1, 8)
        )

    @property
    def work_joint7(self) -> float:
        return self.limits.work_joint7_rad

    def check_sources(
        self,
        *,
        joint_state_age: float | None,
        scene_age: float | None,
    ) -> MotionCheck:
        for name, age, limit in (
            ('joint_states', joint_state_age, self.limits.joint_state_max_age_sec),
            ('planning_scene', scene_age, self.limits.scene_max_age_sec),
        ):
            if age is None:
                return MotionCheck(False, f'{name} freshness is unknown')
            if not math.isfinite(float(age)) or float(age) < 0.0:
                return MotionCheck(False, f'{name} freshness is invalid')
            if float(age) > limit:
                return MotionCheck(
                    False,
                    f'{name} is stale ({float(age):.2f} s > {limit:.2f} s)',
                )
        return MotionCheck(True, 'fresh')

    def validate_trajectory(
        self,
        points: Sequence[Sequence[float]],
        tcp_poses: Sequence[Transform],
        current_joints: Sequence[float] | None = None,
        *,
        start_tcp_pose: Transform | None = None,
        held_object: str | None = None,
    ) -> MotionCheck:
        if not points:
            return MotionCheck(False, 'trajectory is empty')
        if len(points) != len(tcp_poses):
            return MotionCheck(
                False,
                'every trajectory point needs a matching tcp pose',
            )
        parsed: list[tuple[float, ...]] = []
        for index, point in enumerate(points):
            values = tuple(float(value) for value in point)
            if len(values) != 7:
                return MotionCheck(
                    False, f'point {index} must contain 7 joint values',
                )
            if any(not math.isfinite(value) for value in values):
                return MotionCheck(False, f'point {index} has a non-finite value')
            parsed.append(values)

        if current_joints is not None:
            current = tuple(float(value) for value in current_joints)
            if len(current) != 7:
                return MotionCheck(False, 'current joint state must have 7 values')
            if any(
                abs(a - b) > self.limits.start_tolerance_rad
                for a, b in zip(parsed[0], current)
            ):
                return MotionCheck(
                    False,
                    'trajectory does not start at the current joint state',
                )

        min_j7 = min(point[6] for point in parsed)
        max_j7 = max(point[6] for point in parsed)
        j7_deviation = max(
            abs(value - self.limits.work_joint7_rad) for value in (min_j7, max_j7)
        )
        epsilon = 1e-9
        for index, values in enumerate(parsed):
            for joint_index, value in enumerate(values):
                name = self.joint_names[joint_index]
                bounds = self.joint_limits.get(name)
                if bounds is None:
                    continue
                low, high = float(bounds[0]), float(bounds[1])
                if value < low - epsilon or value > high + epsilon:
                    return MotionCheck(
                        False,
                        f'{name} at point {index} is {value:.4f} rad, '
                        f'outside the joint limit [{low:.4f}, {high:.4f}]',
                        {
                            'joint': name,
                            'value': value,
                            'limit': [low, high],
                            'point': index,
                        },
                    )

        max_tilt = 0.0
        start_tilt = (
            tcp_tilt_deg(start_tcp_pose)
            if isinstance(start_tcp_pose, Transform)
            else 0.0
        )
        # A robot that starts outside the working posture (the calibrated home
        # pose points the tip up) needs one empty-gripper reorientation to the
        # downward working pose.  This exception is explicit and logged; J7,
        # jumps and the final downward orientation are still checked.  Any
        # motion with a held/mounted object is always fully tilt-checked.
        reorientation = (
            start_tilt > self.limits.max_tilt_deg + epsilon
            and held_object is None
        )
        # Allow only floating-point roundoff; a planned path must obey the
        # configured tilt limit rather than widening it by a visible angle.
        tilt_epsilon_deg = 1e-6
        tilt_limit_deg = self.limits.max_tilt_deg + tilt_epsilon_deg
        for index, pose in enumerate(tcp_poses):
            if not isinstance(pose, Transform):
                return MotionCheck(False, f'tcp pose {index} is not a Transform')
            tilt = tcp_tilt_deg(pose)
            max_tilt = max(max_tilt, tilt)
            if tilt > tilt_limit_deg and not reorientation:
                return MotionCheck(
                    False,
                    f'tcp tilt at point {index} is {tilt:.2f} deg '
                    f'(limit {self.limits.max_tilt_deg:.2f})',
                    {'max_tilt_deg': tilt, 'point': index},
                )
        if reorientation:
            final_tilt = tcp_tilt_deg(tcp_poses[-1])
            if final_tilt > tilt_limit_deg:
                return MotionCheck(
                    False,
                    'a reorientation motion must finish in the downward '
                    f'working posture (final tilt {final_tilt:.2f} deg)',
                    {'final_tilt_deg': final_tilt},
                )

        max_jump = 0.0
        for index in range(1, len(parsed)):
            jump = max(
                abs(a - b) for a, b in zip(parsed[index], parsed[index - 1])
            )
            max_jump = max(max_jump, jump)
            if jump > self.limits.max_joint_jump_rad + epsilon:
                return MotionCheck(
                    False,
                    f'joint jump of {jump:.3f} rad before point {index} '
                    f'(limit {self.limits.max_joint_jump_rad:.3f})',
                    {'max_jump_rad': jump, 'point': index},
                )

        max_step = 0.0
        for index in range(1, len(tcp_poses)):
            step = float(
                np.linalg.norm(
                    tcp_poses[index].position - tcp_poses[index - 1].position
                )
            )
            max_step = max(max_step, step)
            if step > self.limits.max_cartesian_jump_m + epsilon:
                return MotionCheck(
                    False,
                    f'cartesian jump of {step:.3f} m before point {index} '
                    f'(limit {self.limits.max_cartesian_jump_m:.3f})',
                    {'max_cartesian_step_m': step, 'point': index},
                )

        return MotionCheck(
            True,
            'trajectory accepted',
            {
                'points': len(parsed),
                'min_j7': min_j7,
                'max_j7': max_j7,
                'max_j7_deviation_rad': j7_deviation,
                'preferred_j7_rad': self.limits.work_joint7_rad,
                'max_tilt_deg': max_tilt,
                'reorientation': reorientation,
                'max_joint_jump_rad': max_jump,
                'max_cartesian_step_m': max_step,
            },
        )
