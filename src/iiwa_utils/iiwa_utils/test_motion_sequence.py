#!/usr/bin/env python3
import json
import threading
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.serialization import serialize_message

import rosbag2_py
from rosidl_runtime_py.utilities import get_message

from iiwa_msgs.action import MoveToJoints, MoveToPose


class IiwaTestRunner(Node):
    def __init__(self):
        super().__init__('iiwa_test_runner')

        self.declare_parameter('n_iterations', 3)
        self.declare_parameter('bag_path', '/tmp/iiwa_test')
        self.declare_parameter('config_path', '')
        self.declare_parameter('topics', [
            '/joint_states',
            '/iiwa/joint_states',
            '/tf',
            '/tf_static',
        ])
        self.declare_parameter("delay_between_iterations", 5.0)

        self._n_iter = self.get_parameter('n_iterations').value
        self._delay_between_iterations = self.get_parameter("delay_between_iterations").value
        self._bag_path = self.get_parameter('bag_path').value
        self._topics_param = self.get_parameter('topics').value
        config_path = self.get_parameter('config_path').value

        cfg = self._load_config(config_path)
        self._home_joints: list[float] = cfg['home_joints']
        self._poses: list[dict] = cfg['poses']

        self._cb_group = ReentrantCallbackGroup()
        self._joints_client = ActionClient(
            self, MoveToJoints, '/iiwa/move_to_joints',
            callback_group=self._cb_group,
        )
        self._pose_client = ActionClient(
            self, MoveToPose, '/iiwa/move_to_pose',
            callback_group=self._cb_group,
        )

        self._writer: rosbag2_py.SequentialWriter | None = None
        self._registered_topics: set[str] = set()
        self._subs = []

        self._init_bag()
        self._init_subscribers()

    # Config
    def _load_config(self, config_path: str) -> dict:
        path = Path(config_path) if config_path else Path(__file__).parent / 'motion_config.json'
        self.get_logger().info(f'Loading config from {path}')
        with open(path) as f:
            return json.load(f)

    # Bag files
    def _init_bag(self):
        storage_opts = rosbag2_py.StorageOptions(uri=self._bag_path, storage_id='mcap')
        converter_opts = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        )
        self._writer = rosbag2_py.SequentialWriter()
        self._writer.open(storage_opts, converter_opts)
        self.get_logger().info(f'Bag opened at {self._bag_path}')

    def _init_subscribers(self):
        self.get_logger().info('Discovering topics (2 s)...')
        time.sleep(2.0)
        available = dict(self.get_topic_names_and_types())

        for topic in self._topics_param:
            if topic not in available:
                self.get_logger().warn(f'Topic {topic} not available, skipping')
                continue

            type_str = available[topic][0]
            try:
                msg_type = get_message(type_str)
            except Exception as exc:
                self.get_logger().warn(f'Cannot load type {type_str} for {topic}: {exc}')
                continue

            self._writer.create_topic(rosbag2_py.TopicMetadata(
                id=len(self._registered_topics),
                name=topic,
                type=type_str,
                serialization_format='cdr',
            ))
            self._registered_topics.add(topic)

            def _make_cb(t: str):
                def cb(msg):
                    if self._writer:
                        self._writer.write(
                            t,
                            serialize_message(msg),
                            self.get_clock().now().nanoseconds,
                        )
                return cb

            self._subs.append(self.create_subscription(
                msg_type, topic, _make_cb(topic), 10,
                callback_group=self._cb_group,
            ))
            self.get_logger().info(f'  subscribed: {topic} [{type_str}]')

    def close_bag(self):
        if self._writer:
            del self._writer
            self._writer = None
            self.get_logger().info(f'[BAG] closed → {self._bag_path}')

    # Action helpers
    def _send_joints_goal(self, joints: list[float], speed: float = 0.1) -> bool:
        goal = MoveToJoints.Goal()
        goal.joints = joints
        goal.speed = speed

        self.get_logger().info(f'[MOVE] joints → {joints}')
        if not self._joints_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveToJoints server unavailable')
            return False

        done = threading.Event()
        success_holder: list[bool] = [False]

        def _on_result(future):
            res = future.result().result
            success_holder[0] = res.success
            self.get_logger().info(f'[MOVE] joints done: success={res.success}')
            done.set()

        def _on_goal(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error('MoveToJoints goal rejected')
                done.set()
                return
            gh.get_result_async().add_done_callback(_on_result)

        self._joints_client.send_goal_async(goal).add_done_callback(_on_goal)
        done.wait()
        return success_holder[0]

    def _send_pose_goal(self, *, x, y, z, a, b, c,
                        speed: float = 0.1, 
                        planner: str = 'lin',
                        id: int = None) -> bool:
        goal = MoveToPose.Goal()
        goal.x, goal.y, goal.z = x, y, z
        goal.a, goal.b, goal.c = a, b, c
        goal.speed = speed
        goal.planner = planner

        self.get_logger().info(f'[MOVE] {id if id is not None else ""} pose → x={x} y={y} z={z} planner={planner}')
        if not self._pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveToPose server unavailable')
            return False

        done = threading.Event()
        success_holder: list[bool] = [False]

        def _on_result(future):
            res = future.result().result
            success_holder[0] = res.success
            self.get_logger().info(f'[MOVE] pose done: success={res.success}')
            done.set()

        def _on_goal(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error('MoveToPose goal rejected')
                done.set()
                return
            gh.get_result_async().add_done_callback(_on_result)

        self._pose_client.send_goal_async(goal).add_done_callback(_on_goal)
        done.wait()
        return success_holder[0]

    # Main sequence
    def run(self, done_event: threading.Event):
        try:
            for i in range(self._n_iter):
                self.get_logger().info(f'======= Iteration {i + 1}/{self._n_iter} =======')

                self._send_joints_goal(self._home_joints)

                for i, pose in enumerate(self._poses):
                    self._send_pose_goal(id=i, **pose)

                self._send_joints_goal(self._home_joints)
                time.sleep(self._delay_between_iterations)

            self.get_logger().info('======= Sequence complete =======')
        finally:
            self.close_bag()
            done_event.set()


def main():
    rclpy.init()
    node = IiwaTestRunner()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    done_event = threading.Event()
    threading.Thread(target=node.run, args=(done_event,), daemon=True).start()

    try:
        while not done_event.is_set():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_bag()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
