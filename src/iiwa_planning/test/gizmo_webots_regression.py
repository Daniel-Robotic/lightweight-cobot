"""Manual regression against Webots: release commits exactly one TCP action.

Exercises the marker model with real ROS/controller feedback. Physical mouse
input is covered separately by the Webots plugin adapter tests.
"""
import copy
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatusArray
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from trajectory_msgs.msg import JointTrajectoryPoint
from iiwa_msgs.msg import GizmoCommand, GizmoState
from iiwa_utils.tcp_gizmo import pose_matrix
from iiwa_utils.tcp_gizmo_model import DragTarget
from iiwa_planning.motion_priority import MotionPriorityClient


def main():
    rclpy.init()
    node = rclpy.create_node('gizmo_webots_regression')
    state = {}
    drag = DragTarget()

    def capture(msg):
        state['gizmo'] = msg
        if msg.actual.header.frame_id:
            drag.update_feedback(pose_matrix(msg.actual.pose), msg.enabled, msg.epoch)

    node.create_subscription(GizmoState, '/cobot/tcp_gizmo/state', capture, 10)
    node.create_subscription(
        GoalStatusArray, '/cobot/tcp_gizmo/follow_joint_trajectory/_action/status',
        lambda msg: state.update(status=msg),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publisher = node.create_publisher(GizmoCommand, '/cobot/tcp_gizmo/command', 1)

    def pump(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)

    def wait(predicate, timeout=30.):
        end = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < end, f'Timeout: {state.get("gizmo")}'
            rclpy.spin_once(node, timeout_sec=.01)

    def ids():
        return {bytes(g.goal_info.goal_id.uuid) for g in state['status'].status_list} if 'status' in state else set()

    def error(goal):
        diff = np.linalg.inv(pose_matrix(goal.pose)) @ pose_matrix(state['gizmo'].actual.pose)
        return np.linalg.norm(diff[:3, 3]), Rotation.from_matrix(diff[:3, :3]).magnitude()

    def release(goal):
        initial = copy.deepcopy(state['gizmo'].actual)
        epoch = state['gizmo'].epoch
        before = ids()
        drag.step(pose_matrix(initial.pose), False)
        for _ in range(8):
            shown, command = drag.step(pose_matrix(goal.pose), True)
            assert command is None, 'Mouse hold must only edit the preview'
            pump(.02)
        assert ids() == before, 'Controller received an action before release'
        assert error(initial)[0] < .001, 'Robot moved while mouse was held'
        shown, command = drag.step(shown, False)
        assert command is not None
        msg = GizmoCommand(target=copy.deepcopy(goal), epoch=epoch)
        msg.target.header.stamp = state['gizmo'].actual.header.stamp
        publisher.publish(msg)
        return epoch, before

    def finish(goal, epoch, before):
        # No command keepalive: one release must complete even beyond timeout.
        wait(lambda: state['gizmo'].epoch != epoch)
        assert state['gizmo'].reason == 'target reached', state['gizmo'].reason
        wait(lambda: len(ids() - before) == 1, 2.)
        wait(lambda: error(goal)[0] < .002 and error(goal)[1] < .01, 2.)
        pump(.15)
        assert len(ids() - before) == 1, 'A release generated repeated actions'
        assert not any(g.status in (1, 2, 3) for g in state['status'].status_list)
        shown, command = drag.step(pose_matrix(goal.pose), False)
        assert command is None
        np.testing.assert_allclose(shown, pose_matrix(state['gizmo'].actual.pose), atol=1e-8)

    try:
        params = AsyncParameterClient(node, '/tcp_gizmo')
        assert params.wait_for_services(timeout_sec=30.)
        future = params.get_parameters(['simulation', 'use_sim_time', 'pose_link', 'robot_description_semantic',
                                        'max_linear_speed'])
        wait(future.done)
        values = future.result().values
        assert values[0].bool_value and values[1].bool_value, 'Simulation only'
        assert values[2].string_value == 'tcp', 'Gizmo must use active tool TCP'
        srdf = ET.fromstring(values[3].string_value)
        controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
        assert controllers.wait_for_service(timeout_sec=30.)
        deadline = time.monotonic() + 30.
        while True:
            listing = controllers.call_async(ListControllers.Request())
            wait(listing.done)
            if any(c.name == 'iiwa_arm_controller' and c.state == 'active' for c in listing.result().controller):
                break
            assert time.monotonic() < deadline
            pump(.1)
        wait(lambda: 'gizmo' in state and state['gizmo'].enabled)
        action = ActionClient(node, FollowJointTrajectory, '/iiwa_arm_controller/follow_joint_trajectory')
        assert action.wait_for_server(timeout_sec=5.)

        def ordinary(name):
            joints = srdf.find(f"group_state[@name='{name}']")
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = [j.attrib['name'] for j in joints]
            point = JointTrajectoryPoint(positions=[float(j.attrib['value']) for j in joints])
            point.time_from_start.sec = 3
            goal.trajectory.points = [point]
            future = action.send_goal_async(goal)
            wait(future.done)
            assert future.result().accepted
            return future.result().get_result_async()

        done = ordinary('home')
        wait(done.done)
        assert done.result().result.error_code == 0
        wait(lambda: state['gizmo'].enabled)
        goal = copy.deepcopy(state['gizmo'].actual)
        goal.pose.position.z -= .02
        finish(goal, *release(goal))
        print('PASS: straight posture -> 2 cm descent; no action during drag, exactly one on release', flush=True)

        done = ordinary('work')
        wait(done.done)
        assert done.result().result.error_code == 0
        wait(lambda: state['gizmo'].enabled)
        for kind in ('translation', 'rotation'):
            for axis in range(3):
                for sign in (1., -1.):
                    goal = copy.deepcopy(state['gizmo'].actual)
                    if kind == 'translation':
                        key = ('x', 'y', 'z')[axis]
                        setattr(goal.pose.position, key, getattr(goal.pose.position, key) + sign * .01)
                    else:
                        rotvec = np.zeros(3)
                        rotvec[axis] = sign * .03
                        q = (Rotation.from_rotvec(rotvec) * Rotation.from_matrix(pose_matrix(goal.pose)[:3, :3])).as_quat()
                        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = q.tolist()
                    finish(goal, *release(goal))
                    print(f'PASS: {kind} axis={axis} sign={sign:+.0f}; exactly one action', flush=True)

        goal = copy.deepcopy(state['gizmo'].actual)
        goal.pose.position.x = 10.
        epoch, before = release(goal)
        wait(lambda: state['gizmo'].epoch != epoch)
        assert 'unreachable' in state['gizmo'].reason
        assert ids() == before, 'Invalid target dispatched an action'
        shown, command = drag.step(pose_matrix(goal.pose), False)
        np.testing.assert_allclose(shown, pose_matrix(state['gizmo'].actual.pose), atol=1e-8)
        assert command is None
        print('PASS: unreachable released target resets marker; no controller action', flush=True)

        goal = copy.deepcopy(state['gizmo'].actual)
        goal.pose.position.x -= .1
        epoch, before = release(goal)
        wait(lambda: len(ids() - before) == 1)
        low_id = next(iter(ids() - before))
        assert any(bytes(g.goal_info.goal_id.uuid) == low_id and g.status == 2 for g in state['status'].status_list)
        done = ordinary('work')
        # The high request may be accepted upstream, but must not be dispatched
        # to JTC until the complete gizmo action has succeeded.
        deadline = time.monotonic() + 20.
        while not done.done():
            assert time.monotonic() < deadline, 'Queued ordinary command did not complete'
            pump(.01)
            if len(ids() - before) > 1:
                assert any(bytes(g.goal_info.goal_id.uuid) == low_id and g.status == 4
                           for g in state['status'].status_list), 'Ordinary motion interrupted gizmo'
        assert done.result().result.error_code == 0
        wait(lambda: state['gizmo'].enabled)
        assert len(ids() - before) == 2
        goal = copy.deepcopy(state['gizmo'].actual)
        goal.pose.position.x -= .01
        finish(goal, *release(goal))
        print('PASS: ordinary action waits for completed gizmo action; next release works', flush=True)

        # Pose/joints/file planners first request a lease, before planning from
        # the robot's current state. Exercise their actual client as well.
        goal = copy.deepcopy(state['gizmo'].actual)
        goal.pose.position.x -= .1
        epoch, before = release(goal)
        wait(lambda: len(ids() - before) == 1)
        low_id = next(iter(ids() - before))
        priority = MotionPriorityClient()
        completed = threading.Event()
        failures = []
        def acquire():
            try:
                priority.exchange('release-regression', True, 5.)
            except Exception as error:
                failures.append(error)
            finally:
                completed.set()
        started = time.monotonic()
        worker = threading.Thread(target=acquire, daemon=True)
        worker.start()
        try:
            wait(completed.is_set)
            assert not failures, failures
            elapsed = time.monotonic() - started
            assert any(bytes(g.goal_info.goal_id.uuid) == low_id and g.status == 4
                       for g in state['status'].status_list), 'Lease granted before gizmo completed'
            if values[4].double_value <= .03:
                assert elapsed > 2., 'Slow trajectory did not exercise the old 2 s RPC timeout'
            priority.exchange('release-regression', False, 5.)
            wait(lambda: state['gizmo'].enabled)
            print(f'PASS: planner lease waits for full gizmo action ({elapsed:.2f} s)', flush=True)
        finally:
            priority.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
