"""Trajectory-level limits: J7 corridor, tilt, jumps and freshness."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from iiwa_experement.geometry import Transform
from iiwa_llm_plan_executor.motion_policy import MotionPolicy, load_limits


PACKAGE = Path(__file__).resolve().parents[1]
LIMITS = PACKAGE / 'config' / 'motion_limits.yaml'
BASE = (0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0)


def policy() -> MotionPolicy:
    return MotionPolicy(load_limits(LIMITS))


def point(j7: float, base=BASE):
    values = list(base)
    values[6] = j7
    return tuple(values)


def down_pose(tilt_deg: float = 0.0, xyz=(0.4, 0.0, 1.2)) -> Transform:
    rotation = (
        Rotation.from_euler('x', 180.0, degrees=True).as_matrix()
        @ Rotation.from_euler('y', tilt_deg, degrees=True).as_matrix()
    )
    return Transform(
        xyz=tuple(float(value) for value in xyz),
        rotation=tuple(tuple(float(value) for value in row) for row in rotation),
        parent='world',
        child='tcp',
    )


def test_work_joint7_is_zero_and_corridor_is_inclusive():
    limits = load_limits(LIMITS)
    assert limits.work_joint7_rad == 0.0
    assert limits.joint7_min_rad == -math.pi / 2
    assert limits.joint7_max_rad == math.pi / 2
    checker = MotionPolicy(limits)
    for boundary in (-math.pi / 2, math.pi / 2):
        result = checker.validate_trajectory(
            [point(boundary)], [down_pose()],
        )
        assert result.ok, result.reason


def test_point_outside_the_corridor_is_rejected():
    result = policy().validate_trajectory(
        [point(-math.pi / 2 - 0.01)], [down_pose()],
    )
    assert result.ok is False
    assert 'joint7' in result.reason


def test_intermediate_point_outside_the_corridor_rejects_the_whole_path():
    result = policy().validate_trajectory(
        [point(-1.2), point(-1.6), point(-1.2)],
        [down_pose(), down_pose(), down_pose()],
    )
    assert result.ok is False
    assert 'joint7' in result.reason


def test_near_pi_joint_jump_is_rejected():
    result = policy().validate_trajectory(
        [point(-1.4), point(1.4)],
        [down_pose(), down_pose()],
    )
    assert result.ok is False
    assert 'jump' in result.reason


def test_tcp_tilt_above_ten_degrees_is_rejected():
    checker = policy()
    assert checker.validate_trajectory(
        [point(0.0)], [down_pose(9.9)],
    ).ok
    rejected = checker.validate_trajectory(
        [point(0.0)], [down_pose(10.5)],
    )
    assert rejected.ok is False
    assert 'tilt' in rejected.reason
    assert checker.validate_trajectory(
        [point(0.0)], [down_pose(10.05)],
    ).ok is False


def test_an_empty_gripper_may_reorient_to_the_working_posture():
    checker = policy()
    up = Transform.from_xyz_rpy(
        [0.4, 0.0, 1.2], [0.0, 0.0, 0.0], parent='world', child='tcp',
    )
    accepted = checker.validate_trajectory(
        [point(0.0), point(0.0)],
        [up, down_pose()],
        start_tcp_pose=up,
        held_object=None,
    )
    assert accepted.ok is True
    assert accepted.details['reorientation'] is True

    held = checker.validate_trajectory(
        [point(0.0), point(0.0)],
        [up, down_pose()],
        start_tcp_pose=up,
        held_object='cap',
    )
    assert held.ok is False
    assert 'tilt' in held.reason

    stuck_up = checker.validate_trajectory(
        [point(0.0), point(0.0)],
        [up, up],
        start_tcp_pose=up,
        held_object=None,
    )
    assert stuck_up.ok is False
    assert 'working posture' in stuck_up.reason


def test_stale_joint_state_and_scene_are_rejected_before_motion():
    checker = policy()
    assert checker.check_sources(
        joint_state_age=0.1, scene_age=0.2,
    ).ok
    assert checker.check_sources(
        joint_state_age=1.5, scene_age=0.2,
    ).ok is False
    assert checker.check_sources(
        joint_state_age=0.1, scene_age=2.5,
    ).ok is False
    assert checker.check_sources(
        joint_state_age=None, scene_age=0.2,
    ).ok is False


def test_malformed_or_jumping_trajectories_are_rejected():
    checker = policy()
    assert checker.validate_trajectory(
        [point(0.0)[:6]], [down_pose()],
    ).ok is False
    assert checker.validate_trajectory(
        [point(0.0)], [],
    ).ok is False
    assert checker.validate_trajectory(
        [point(0.0), point(0.0)], [down_pose(), down_pose()],
        current_joints=(0.5, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0),
    ).ok is False
    far = down_pose(xyz=(0.9, 0.0, 1.2))
    assert checker.validate_trajectory(
        [point(0.0), point(0.0)], [down_pose(), far],
    ).ok is False


def test_details_report_the_checked_extremes():
    result = policy().validate_trajectory(
        [point(-0.2), point(0.1)],
        [down_pose(2.0), down_pose(4.0)],
    )
    assert result.ok
    assert result.details['min_j7'] == pytest.approx(-0.2)
    assert result.details['max_j7'] == pytest.approx(0.1)
    assert result.details['max_tilt_deg'] == pytest.approx(4.0, abs=1e-6)
