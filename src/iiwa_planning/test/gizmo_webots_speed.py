"""Manual Webots speed regression: work -> 10 cm inward TCP translation.

Run against iiwa.launch.py simulate:=true. Checks actual feedback, both peak
TCP speed and convergence, and prints elapsed simulation/wall time.
"""
import copy
import math
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter_client import AsyncParameterClient
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from trajectory_msgs.msg import JointTrajectoryPoint
from iiwa_msgs.msg import GizmoCommand, GizmoState


def main():
    rclpy.init()
    node = rclpy.create_node('gizmo_webots_speed')
    states = []
    node.create_subscription(GizmoState, '/cobot/tcp_gizmo/state', states.append, 10)
    publisher = node.create_publisher(GizmoCommand, '/cobot/tcp_gizmo/command', 1)

    def wait(predicate, timeout=30.):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, 'Timed out waiting for simulation'
            rclpy.spin_once(node, timeout_sec=.01)

    def xyz(state):
        p = state.actual.pose.position
        return p.x, p.y, p.z

    def stamp(state):
        t = state.actual.header.stamp
        return t.sec + t.nanosec * 1e-9

    try:
        params = AsyncParameterClient(node, '/tcp_gizmo')
        assert params.wait_for_services(timeout_sec=30.)
        future = params.get_parameters([
            'simulation', 'use_sim_time', 'max_linear_speed', 'robot_description_semantic'])
        wait(future.done)
        values = future.result().values
        assert values[0].bool_value and values[1].bool_value, 'Webots simulation only'
        speed_limit = values[2].double_value
        work = ET.fromstring(values[3].string_value).find("group_state[@name='work']")
        assert work is not None
        controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
        assert controllers.wait_for_service(timeout_sec=30.)
        deadline = time.monotonic() + 30.
        while True:
            listing = controllers.call_async(ListControllers.Request())
            wait(listing.done)
            if any(c.name == 'iiwa_arm_controller' and c.state == 'active'
                   for c in listing.result().controller):
                break
            assert time.monotonic() < deadline, 'Controller did not activate'
            rclpy.spin_once(node, timeout_sec=.1)
        wait(lambda: states and states[-1].enabled)
        action = ActionClient(node, FollowJointTrajectory, '/iiwa_arm_controller/follow_joint_trajectory')
        assert action.wait_for_server(timeout_sec=5.)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [joint.attrib['name'] for joint in work]
        point = JointTrajectoryPoint(positions=[float(joint.attrib['value']) for joint in work])
        point.time_from_start.sec = 3
        goal.trajectory.points = [point]
        future = action.send_goal_async(goal)
        wait(future.done)
        assert future.result().accepted
        done = future.result().get_result_async()
        wait(done.done)
        assert done.result().result.error_code == 0
        wait(lambda: states[-1].enabled)
        initial = states[-1]
        target = copy.deepcopy(initial.actual)
        target.pose.position.x -= .1
        start = len(states) - 1
        epoch = initial.epoch
        wall_start = time.monotonic()
        publisher.publish(GizmoCommand(target=target, epoch=epoch))
        while states[-1].epoch == epoch:
            assert time.monotonic() - wall_start < 30., 'TCP did not reach 10 cm target'
            end = time.monotonic() + .025
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.005)
        assert states[-1].reason == 'target reached', states[-1].reason
        wait(lambda: initial.actual.pose.position.x - states[-1].actual.pose.position.x >= .099, 2.)
        samples = states[start:]
        speeds = [math.dist(xyz(a), xyz(b)) / (stamp(b) - stamp(a))
                  for a, b in zip(samples, samples[1:]) if stamp(b) > stamp(a)]
        elapsed = stamp(samples[-1]) - stamp(initial)
        peak = max(speeds)
        distance = math.dist(xyz(initial), xyz(samples[-1]))
        print(f'limit={speed_limit:.3f} m/s; distance={distance:.6f} m; '
              f'simulation_time={elapsed:.3f} s; wall_time={time.monotonic() - wall_start:.3f} s; '
              f'mean={distance / elapsed:.4f} m/s; peak={peak:.4f} m/s', flush=True)
        assert peak <= speed_limit * 1.05 + .002, 'Measured TCP speed exceeds configured cap'
        # At 0.50 m/s, a 10 cm work-pose move must not consist of hundreds of
        # tiny acceleration-limited 64 ms steps. Keep margin for simulator load.
        if speed_limit >= .5:
            assert elapsed < 3., f'Artificial short-step speed bottleneck: {elapsed:.3f} s'
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
