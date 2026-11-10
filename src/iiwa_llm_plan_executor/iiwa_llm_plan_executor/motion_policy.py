"""Trajectory-level TCP/J7 limits applied before anything reaches the controller.

The MoveToPose action cannot express a joint-7 corridor or a working tilt
limit, so every plan produced by the planner adapter is checked here, point
by point, before execution.
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

    @property
    def joint7_min_rad(self) -> float:
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
    if corridor <= 0 or tilt <= 0 or jump <= 0 or cartesian <= 0:
        raise MotionPolicyError('motion limits must be positive')
    if joint_age <= 0 or scene_age <= 0:
        raise MotionPolicyError('freshness limits must be positive')
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
    )


@dataclass(frozen=True)
class MotionCheck:
    ok: bool
    reason: str = ''
    details: dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {'ok': self.ok, 'reason': self.reason, 'details': dict(self.details)}


def _tcp_tilt_deg(pose: Transform) -> float:
    matrix = pose.matrix
    # The working direction is the tcp local +Z axis; it must point down.
    z_axis = matrix[:, 2]
    dot = float(np.clip(-z_axis[2], -1.0, 1.0))
    return math.degrees(math.acos(dot))


class MotionPolicy:
    def __init__(self, limits: MotionLimits) -> None:
        self.limits = limits

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
        epsilon = 1e-9
        if (
            min_j7 < self.limits.joint7_min_rad - epsilon
            or max_j7 > self.limits.joint7_max_rad + epsilon
        ):
            return MotionCheck(
                False,
                'joint7 leaves the hard corridor '
                f'[{self.limits.joint7_min_rad:.3f}, '
                f'{self.limits.joint7_max_rad:.3f}] rad '
                f'(trajectory {min_j7:.3f}..{max_j7:.3f})',
                {'min_j7': min_j7, 'max_j7': max_j7},
            )

        max_tilt = 0.0
        for index, pose in enumerate(tcp_poses):
            if not isinstance(pose, Transform):
                return MotionCheck(False, f'tcp pose {index} is not a Transform')
            tilt = _tcp_tilt_deg(pose)
            max_tilt = max(max_tilt, tilt)
            if tilt > self.limits.max_tilt_deg + epsilon:
                return MotionCheck(
                    False,
                    f'tcp tilt at point {index} is {tilt:.2f} deg '
                    f'(limit {self.limits.max_tilt_deg:.2f})',
                    {'max_tilt_deg': tilt, 'point': index},
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
                'max_tilt_deg': max_tilt,
                'max_joint_jump_rad': max_jump,
                'max_cartesian_step_m': max_step,
            },
        )
