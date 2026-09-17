"""Native Webots TCP marker. Loaded as a webots_ros2_driver Python plugin."""
import time

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

from iiwa_msgs.msg import GizmoCommand, GizmoState
from iiwa_utils.tcp_gizmo_model import DragTarget, to_webots, webots_pose


def pose_matrix(pose):
    result = np.eye(4)
    result[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    result[:3, :3] = Rotation.from_quat([
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
    ]).as_matrix()
    if not np.isfinite(result).all():
        raise ValueError('Non-finite TCP pose')
    return result


def set_pose(message, matrix):
    message.position.x, message.position.y, message.position.z = matrix[:3, 3].tolist()
    q = Rotation.from_matrix(matrix[:3, :3]).as_quat().tolist()
    message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w = q


class WebotsTcpGizmo:
    def init(self, webots_node, properties):
        self._robot = webots_node.robot
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node('tcp_gizmo_webots')
        self._frame = properties.get('frame', 'base_link')
        self._mount = webots_pose(
            [float(x) for x in properties['translation'].split()],
            [float(x) for x in properties['rotation'].split()],
        )
        self._inverse_mount = np.linalg.inv(self._mount)
        self._state_timeout = float(properties.get('state_timeout', '0.5'))
        self._drag = DragTarget()
        self._last_state = 0.0
        self._last_stamp = None
        marker = self._robot.getSelf()
        self._translation = marker.getField('translation')
        self._rotation = marker.getField('rotation')
        self._mouse = self._robot.getMouse()
        self._mouse.enable(int(self._robot.getBasicTimeStep()))
        self._publisher = self._node.create_publisher(GizmoCommand, '/cobot/tcp_gizmo/command', 1)
        self._node.create_subscription(GizmoState, '/cobot/tcp_gizmo/state', self._feedback, 1)
        self._node.get_logger().info('TCP gizmo: position/rotate the marker, then release the mouse to move')

    def _feedback(self, message):
        # The launch validates the frame: the Webots mount maps base_link to world.
        if message.actual.header.frame_id != self._frame:
            self._drag.invalidate()
            return
        try:
            actual = self._mount @ pose_matrix(message.actual.pose)
        except ValueError:
            self._drag.invalidate()
            return
        self._last_state = time.monotonic()
        self._last_stamp = message.actual.header.stamp
        self._drag.update_feedback(actual, message.enabled, message.epoch)

    def step(self):
        rclpy.spin_once(self._node, timeout_sec=0)
        if time.monotonic() - self._last_state > self._state_timeout:
            self._drag.invalidate()
        try:
            observed = webots_pose(self._translation.getSFVec3f(), self._rotation.getSFRotation())
        except ValueError:
            self._drag.invalidate()
            return
        # Webots may not expose a 3-D handle drag through getSelected().  A
        # command still requires the marker fields themselves to change, so a
        # mouse press elsewhere cannot start TCP motion.
        dragging = self._mouse.getState().left
        shown, target = self._drag.step(observed, dragging)
        xyz, aa = to_webots(shown)
        self._translation.setSFVec3f(xyz)
        self._rotation.setSFRotation(aa)
        if target is not None:
            command = GizmoCommand()
            command.epoch = self._drag.epoch
            command.target.header.frame_id = self._frame
            command.target.header.stamp = self._last_stamp
            set_pose(command.target.pose, self._inverse_mount @ target)
            self._publisher.publish(command)
