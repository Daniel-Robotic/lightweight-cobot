"""Regressions for coordinate conversion and rejected/stale drag targets."""
import math

import numpy as np
import pytest

from iiwa_utils import tcp_gizmo_model as model


def pose(x=0.0):
    result = np.eye(4)
    result[0, 3] = x
    return result


def test_world_to_base_includes_rotated_robot_mount():
    mount = model.webots_pose([1, 2, 3], [0, 0, 1, math.pi / 2])
    target = model.webots_pose([1, 3, 3], [0, 0, 1, math.pi / 2])
    np.testing.assert_allclose(np.linalg.inv(mount) @ target, pose(1), atol=1e-12)


def test_invalid_axis_or_nonfinite_translation_rejected():
    for xyz, rotation in [([float('nan'), 0, 0], [0, 0, 1, 0]),
                          ([0, 0, 0], [0, 0, 0, 1])]:
        with pytest.raises(ValueError):
            model.webots_pose(xyz, rotation)


def test_drag_retains_target_until_actual_tcp_reaches_it():
    drag = model.DragTarget()
    drag.update_feedback(pose(), True, 1)
    shown, command = drag.step(pose(), False)
    assert command is None
    shown, command = drag.step(pose(0.1), True)
    assert command is None
    drag.update_feedback(pose(0.05), True, 1)
    shown, command = drag.step(shown, False)
    np.testing.assert_array_equal(command, pose(0.1))
    # A release is a single committed goal, not a stream of new commands.
    shown, command = drag.step(shown, False)
    assert command is None
    drag.update_feedback(pose(0.1), True, 1)
    shown, command = drag.step(shown, False)
    assert command is None
    np.testing.assert_array_equal(shown, pose(0.1))


def test_rejection_resets_to_actual_and_requires_release_before_new_drag():
    drag = model.DragTarget()
    drag.update_feedback(pose(), True, 1)
    drag.step(pose(), False)
    drag.step(pose(0.5), True)
    drag.update_feedback(pose(0.01), True, 2)
    shown, command = drag.step(pose(0.5), True)
    assert command is None
    np.testing.assert_array_equal(shown, pose(0.01))
    drag.step(pose(0.8), True)
    shown, command = drag.step(shown, False)
    assert command is None
    shown, command = drag.step(pose(0.02), True)
    assert command is None
    shown, command = drag.step(shown, False)
    np.testing.assert_array_equal(command, pose(0.02))


def test_high_priority_motion_discards_old_target_after_completion():
    drag = model.DragTarget()
    drag.update_feedback(pose(), True, 1)
    drag.step(pose(), False)
    drag.step(pose(0.5), True)
    drag.update_feedback(pose(0.2), False, 2)
    shown, command = drag.step(pose(0.5), False)
    assert command is None
    drag.update_feedback(pose(0.3), True, 2)
    shown, command = drag.step(shown, False)
    assert command is None
    np.testing.assert_array_equal(shown, pose(0.3))


def test_stale_feedback_cancels_target():
    drag = model.DragTarget()
    drag.update_feedback(pose(), True, 1)
    drag.step(pose(), False)
    drag.step(pose(0.5), True)
    drag.invalidate()
    shown, command = drag.step(pose(0.5), True)
    assert command is None
    np.testing.assert_array_equal(shown, pose())


@pytest.mark.parametrize('offset', [0.00001, 0.02, 0.5])
def test_idle_marker_readback_difference_never_starts_motion(offset):
    """Delayed Webots field reads or settling are not mouse input."""
    drag = model.DragTarget()
    drag.update_feedback(pose(1), True, 1)
    drag.step(pose(), False)
    shown, command = drag.step(pose(1 - offset), False)
    assert command is None
    np.testing.assert_array_equal(shown, pose(1))


def test_idle_tcp_feedback_changes_do_not_turn_into_drag_commands():
    drag = model.DragTarget()
    previous = pose()
    for index in range(100):
        actual = pose(0.1 + index * 0.002)
        drag.update_feedback(actual, True, 1)
        shown, command = drag.step(previous, False)
        assert command is None
        np.testing.assert_array_equal(shown, actual)
        previous = shown


def test_readback_after_mouse_release_does_not_replace_active_target():
    drag = model.DragTarget()
    drag.update_feedback(pose(), True, 1)
    drag.step(pose(), False)
    drag.step(pose(0.1), True)
    shown, command = drag.step(pose(0.08), False)
    np.testing.assert_array_equal(command, pose(0.1))


def test_idle_rotated_marker_readback_never_starts_motion():
    drag = model.DragTarget()
    actual = model.webots_pose([0.1, 0.2, 0.3], [1, 0, 0, 0.7])
    drag.update_feedback(actual, True, 1)
    drag.step(pose(), False)
    shown, command = drag.step(pose(), False)
    assert command is None
    np.testing.assert_allclose(shown, actual)


def test_no_motion_during_long_drag_and_only_final_pose_is_committed():
    drag = model.DragTarget()
    drag.update_feedback(pose(), True, 1)
    drag.step(pose(), False)
    for index in range(1, 100):
        shown, command = drag.step(pose(index / 1000), True)
        assert command is None
    shown, command = drag.step(shown, False)
    np.testing.assert_allclose(command, pose(.099))
    for _ in range(100):
        shown, command = drag.step(shown, False)
        assert command is None


def test_next_drag_waits_for_controller_completion_epoch():
    drag = model.DragTarget()
    drag.update_feedback(pose(), True, 1)
    drag.step(pose(), False)
    drag.step(pose(.1), True)
    drag.step(pose(.1), False)
    # Looking close enough is not proof that the controller has finished.
    drag.update_feedback(pose(.1), True, 1)
    shown, command = drag.step(pose(.2), True)
    assert command is None
    drag.update_feedback(pose(.1), True, 2)
    drag.step(shown, False)
    shown, command = drag.step(pose(.2), True)
    assert command is None
    shown, command = drag.step(shown, False)
    np.testing.assert_allclose(command, pose(.2))
