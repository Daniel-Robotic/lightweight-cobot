"""Webots marker geometry and drag lifecycle, independent of ROS/Webots APIs."""
import numpy as np
from scipy.spatial.transform import Rotation


def webots_pose(translation, axis_angle):
    xyz = np.asarray(translation, dtype=float)
    aa = np.asarray(axis_angle, dtype=float)
    if xyz.shape != (3,) or aa.shape != (4,) or not np.isfinite(xyz).all() or not np.isfinite(aa).all():
        raise ValueError('Pose must contain finite translation and axis-angle')
    norm = np.linalg.norm(aa[:3])
    if norm < 1e-12 and abs(aa[3]) > 1e-12:
        raise ValueError('Rotation axis must be nonzero')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(aa[:3] * aa[3] / max(norm, 1e-12)).as_matrix()
    result[:3, 3] = xyz
    return result


def to_webots(matrix):
    vector = Rotation.from_matrix(matrix[:3, :3]).as_rotvec()
    angle = np.linalg.norm(vector)
    axis = vector / angle if angle > 1e-12 else np.array([0., 0., 1.])
    return matrix[:3, 3].tolist(), [*axis.tolist(), float(angle)]


def close_pose(left, right, linear=0.001, angular=0.01):
    delta = left[:3, :3].T @ right[:3, :3]
    return (np.linalg.norm(left[:3, 3] - right[:3, 3]) < linear
            and Rotation.from_matrix(delta).magnitude() < angular)


class DragTarget:
    """Separate mouse edits from marker writes and never replay rejected targets."""

    def __init__(self):
        self.actual = None
        self.epoch = None
        self.enabled = False
        self.shown = None
        self.target = None
        self._reset = True
        self._wait_release = False
        self._mouse_down = False
        self._submitted = False

    def update_feedback(self, actual, enabled, epoch):
        if epoch != self.epoch or not enabled:
            self._reset = True
            self.target = None
        self.actual = actual.copy()
        self.enabled = enabled
        self.epoch = epoch

    def invalidate(self):
        self.enabled = False
        self.target = None
        self._reset = True

    def step(self, observed, mouse_down):
        was_down = self._mouse_down
        self._mouse_down = bool(mouse_down)
        if self.actual is None:
            self.shown = observed.copy()
            return self.shown, None
        if self._reset or not self.enabled:
            self.target = None
            self._submitted = False
            self._wait_release = mouse_down
            self._reset = False
            self.shown = self.actual.copy()
            return self.shown, None
        if self._wait_release:
            self._wait_release = mouse_down
            self.shown = self.actual.copy()
            return self.shown, None
        if self._submitted:
            # A committed goal owns the marker until controller completion (a
            # new feedback epoch). Do not queue another mouse gesture behind it.
            self.shown = self.target.copy()
            return self.shown, None
        # Supervisor field writes/readbacks are not mouse commands. In
        # particular, startup synchronisation can read an older marker pose.
        if mouse_down and self.shown is not None and not close_pose(observed, self.shown, 1e-6, 1e-5):
            self.target = observed.copy()
        command = None
        if was_down and not mouse_down and self.target is not None:
            if close_pose(self.target, self.actual):
                self.target = None
            else:
                self._submitted = True
                command = self.target.copy()
        self.shown = (self.target if self.target is not None else self.actual).copy()
        return self.shown, command
