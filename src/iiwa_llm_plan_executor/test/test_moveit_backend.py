"""The MoveIt backend rejects unsafe plans before asking for execution."""
from __future__ import annotations

import math
from pathlib import Path

from scipy.spatial.transform import Rotation

from iiwa_experement.geometry import Transform, load_geometry
from iiwa_llm_plan_executor.motion_policy import MotionPolicy, load_limits
from iiwa_llm_plan_executor.moveit_backend import (
    MotionOutcome,
    MoveItBackend,
    PlannedMotion,
)


PACKAGE = Path(__file__).resolve().parents[1]
ASSETS = PACKAGE.parent / 'iiwa_experement' / 'config'
LIMITS = load_limits(PACKAGE / 'config' / 'motion_limits.yaml')
GEOMETRY = load_geometry(ASSETS / 'geometry.yaml')
BASE = (0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0)


def point(j7: float):
    values = list(BASE)
    values[6] = j7
    return tuple(values)


def down_pose(tilt=0.0, xyz=(0.4, 0.0, 1.2)) -> Transform:
    rotation = (
        Rotation.from_euler('x', 180.0, degrees=True).as_matrix()
        @ Rotation.from_euler('y', tilt, degrees=True).as_matrix()
    )
    return Transform(
        xyz=tuple(float(value) for value in xyz),
        rotation=tuple(tuple(float(value) for value in row) for row in rotation),
        parent='world',
        child='tcp',
    )


GOAL = down_pose()
BASE_GOAL = GEOMETRY.world_to_base.inverse().compose_relaxed(GOAL)


class FakePlanner:
    def __init__(self, *, planned=None, execute_ok=True, plan_error=None,
                 execute_error=None):
        self.planned = planned
        self.execute_ok = execute_ok
        self.plan_error = plan_error
        self.execute_error = execute_error
        self.plan_calls = []
        self.execute_calls = []
        self.stopped = 0
        self.reset_calls = 0

    def plan(self, goal, phase, held_object):
        self.plan_calls.append((goal, phase, held_object))
        if self.plan_error is not None:
            raise self.plan_error
        return self.planned

    def execute(self, planned):
        self.execute_calls.append(planned)
        if self.execute_error is not None:
            raise self.execute_error
        return self.execute_ok

    def stop(self):
        self.stopped += 1

    def reset_stop(self):
        self.reset_calls += 1


def planned(points, poses, current=None):
    if current is None:
        current = tuple(points[0])
    return PlannedMotion(
        points=list(points),
        tcp_poses=list(poses),
        current_joints=current,
        handle='handle',
    )


def backend(planner, **kwargs):
    checker = MotionPolicy(LIMITS)
    return MoveItBackend(
        GEOMETRY,
        checker,
        planner=planner,
        freshness_guard=kwargs.pop('freshness_guard', lambda: (True, 'ok')),
        scene_guard=kwargs.pop('scene_guard', lambda: (True, 'ok')),
        **kwargs,
    )


def test_safe_plan_is_executed_once_and_the_final_pose_is_verified():
    planner = FakePlanner(planned=planned(
        [point(0.0), point(0.1)],
        [BASE_GOAL, BASE_GOAL],
    ))
    result = backend(planner).plan_and_execute(GOAL, 'transfer', 'vacuum')
    assert result.success is True
    assert result.code == 'ok'
    assert len(planner.plan_calls) == 1
    assert planner.plan_calls[0][2] == 'vacuum'
    assert len(planner.execute_calls) == 1


def test_safe_joint7_beyond_the_old_corridor_reaches_execution():
    planner = FakePlanner(planned=planned(
        [point(-1.5), point(-1.7), point(-1.5)],
        [BASE_GOAL, BASE_GOAL, BASE_GOAL],
    ))
    result = backend(planner).plan_and_execute(GOAL, 'transfer')
    assert result.success is True
    assert len(planner.execute_calls) == 1


def test_motion_phase_selection_is_ptp_for_working_points_and_lin_for_axes():
    from iiwa_llm_plan_executor.moveit_backend import MoveItPyPlanner

    phases = MoveItPyPlanner.PHASES
    for phase in ('transfer', 'approach', 'align', 'observe', 'escape'):
        assert phases[phase][0] == 'pilz_industrial_motion_planner'
        assert phases[phase][1] == 'PTP'
    for phase in ('contact', 'lower', 'retreat', 'dispense'):
        assert phases[phase][0] == 'pilz_industrial_motion_planner'
        assert phases[phase][1] == 'LIN'


def test_dense_resampling_covers_steps_between_waypoints():
    from iiwa_llm_plan_executor.moveit_backend import MoveItPyPlanner

    samples = MoveItPyPlanner._densify(
        [(0.0,) * 7, (0.05,) * 7], 0.01,
    )
    assert len(samples) == 6
    assert samples[0] == (0.0,) * 7
    assert samples[-1] == (0.05,) * 7
    assert max(
        abs(a - b) for a, b in zip(samples[1], samples[0])
    ) <= 0.01 + 1e-12


def test_camera_self_collision_is_rejected_before_execution():
    planner = FakePlanner(planned=planned([point(0.0)], [BASE_GOAL]))
    result = backend(
        planner,
        collision_checker=lambda planned_motion: (
            'collision/clearance failure: camera_hand <-> link6; '
            'minimum distance 0.0005 m, required 0.0030 m'
        ),
    ).plan_and_execute(GOAL, 'transfer')
    assert result.success is False
    assert result.code == 'collision'
    assert 'camera_hand' in result.message
    assert 'link6' in result.message
    assert planner.execute_calls == []


def test_excessive_tilt_and_jump_are_rejected_without_execution():
    tilted_goal = GEOMETRY.world_to_base.inverse().compose_relaxed(down_pose(20.0))
    for points, poses in (
        ([point(0.0)], [tilted_goal]),
        ([point(-1.4), point(1.4)], [BASE_GOAL, BASE_GOAL]),
    ):
        planner = FakePlanner(planned=planned(points, poses))
        result = backend(planner).plan_and_execute(GOAL, 'transfer')
        assert result.code == 'motion_limit'
        assert planner.execute_calls == []


def test_stale_sources_reject_before_planning():
    planner = FakePlanner(planned=planned([point(0.0)], [BASE_GOAL]))
    result = backend(
        planner, freshness_guard=lambda: (False, 'joint_states is stale'),
    ).plan_and_execute(GOAL, 'transfer')
    assert result.success is False
    assert result.code == 'stale_state'
    assert planner.plan_calls == []
    assert planner.execute_calls == []


def test_planning_and_execution_failures_are_reported_and_stop_motion():
    planner = FakePlanner(plan_error=RuntimeError('no IK branch'))
    result = backend(planner).plan_and_execute(GOAL, 'transfer')
    assert result.code == 'planning_failed'
    assert planner.execute_calls == []
    assert planner.stopped >= 1

    planner = FakePlanner(
        planned=planned([point(0.0)], [BASE_GOAL]), execute_ok=False,
    )
    result = backend(planner).plan_and_execute(GOAL, 'transfer')
    assert result.code == 'execution_failed'

    planner = FakePlanner(
        planned=planned([point(0.0)], [BASE_GOAL]),
        execute_error=RuntimeError('controller lost'),
    )
    result = backend(planner).plan_and_execute(GOAL, 'transfer')
    assert result.code == 'execution_failed'


def test_mismatched_final_pose_is_rejected():
    wrong = down_pose(tilt=0.0, xyz=(0.42, 0.0, 1.2))
    planner = FakePlanner(planned=planned([point(0.0)], [wrong]))
    result = backend(planner).plan_and_execute(GOAL, 'transfer')
    assert result.code == 'goal_mismatch'
    assert planner.execute_calls == []


def test_stop_blocks_all_later_motion():
    planner = FakePlanner(planned=planned([point(0.0)], [BASE_GOAL]))
    path = backend(planner)
    path.stop('operator pressed stop')
    result = path.plan_and_execute(GOAL, 'transfer')
    assert result.success is False
    assert result.code == 'stopped'
    assert planner.plan_calls == []
    path.reset_stop()
    assert path.plan_and_execute(GOAL, 'transfer').success is True


def test_source_freshness_is_checked_for_non_finite_values():
    checker = MotionPolicy(LIMITS)
    assert checker.check_sources(
        joint_state_age=math.nan, scene_age=0.1,
    ).ok is False
    assert checker.check_sources(
        joint_state_age=0.1, scene_age=-1.0,
    ).ok is False
