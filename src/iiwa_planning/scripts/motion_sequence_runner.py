#!/usr/bin/env python3
"""
Запуск заданной последовательности движений манипулятора iiwa.

Каждая точка в конфиге может быть либо суставной (type: joints),
либо декартовой (type: pose). Тип определяется автоматически по наличию
ключа "joints" или координат "x/y/z".

Запуск:
  ros2 run iiwa_planning motion_sequence_runner \
    --ros-args -p config_path:=/path/to/config.json -p n_iterations:=2
"""

import json
import shutil
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


def _is_joints_waypoint(wp: dict) -> bool:
    return "joints" in wp


class MotionSequenceRunner(Node):
    def __init__(self):
        super().__init__('motion_sequence_runner')

        self.declare_parameter('n_iterations', 3)
        self.declare_parameter('bag_path', '')
        self.declare_parameter('config_path', '')
        self.declare_parameter('topics', [''])
        self.declare_parameter('delay_between_iterations', 5.0)
        self.declare_parameter('joints_action', 'cobot/move_to_joints')
        self.declare_parameter('pose_action', 'cobot/move_to_pose')

        self._n_iter = self.get_parameter('n_iterations').value
        self._delay = self.get_parameter('delay_between_iterations').value
        self._bag_path = self.get_parameter('bag_path').value
        topics_param = self.get_parameter('topics').value
        self._topics_param: list[str] = [t for t in topics_param if t]
        config_path = self.get_parameter('config_path').value
        joints_action = self.get_parameter('joints_action').value
        pose_action = self.get_parameter('pose_action').value

        cfg = self._load_config(config_path)
        self._home: dict = cfg['home']
        self._waypoints: list[dict] = cfg['waypoints']

        self._cb_group = ReentrantCallbackGroup()
        self._joints_client = ActionClient(
            self, MoveToJoints, joints_action,
            callback_group=self._cb_group,
        )
        self._pose_client = ActionClient(
            self, MoveToPose, pose_action,
            callback_group=self._cb_group,
        )
        self.get_logger().info(f'joints_action={joints_action}  pose_action={pose_action}')

        self._writer: rosbag2_py.SequentialWriter | None = None
        self._registered_topics: set[str] = set()
        self._subs = []

        if self._bag_path:
            self._init_bag()
            self._init_subscribers()
        else:
            self.get_logger().info('bag_path not set — recording disabled')

    # ── Config ──────────────────────────────────────────────────────────────

    def _load_config(self, config_path: str) -> dict:
        path = (
            Path(config_path) if config_path
            else Path(__file__).parent / 'motion_sequence_config.json'
        )
        self.get_logger().info(f'Loading config from {path}')
        with open(path) as f:
            return json.load(f)

    # ── Bag recording ────────────────────────────────────────────────────────

    def _init_bag(self):
        bag_dir = Path(self._bag_path)
        if bag_dir.exists():
            shutil.rmtree(bag_dir)
            self.get_logger().info(f'Removed existing bag at {self._bag_path}')

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

        topics = self._topics_param if self._topics_param else list(available.keys())
        if not self._topics_param:
            self.get_logger().info(
                f'topics not set — recording all {len(topics)} available topics'
            )

        for topic in topics:
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
            self.get_logger().info(f'Bag closed → {self._bag_path}')

    # ── Action helpers ────────────────────────────────────────────────────────

    def _send_joints_goal(self, wp: dict) -> bool:
        goal = MoveToJoints.Goal()
        goal.joints = wp['joints']
        goal.speed = float(wp.get('speed', 0.1))

        self.get_logger().info(f'[JOINTS] → {goal.joints}')
        if not self._joints_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveToJoints server unavailable')
            return False

        done = threading.Event()
        result_holder: list[bool] = [False]

        def _on_result(future):
            result_holder[0] = future.result().result.success
            self.get_logger().info(f'[JOINTS] done: success={result_holder[0]}')
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
        return result_holder[0]

    def _send_pose_goal(self, wp: dict, idx: int | None = None) -> bool:
        goal = MoveToPose.Goal()
        goal.x, goal.y, goal.z = wp['x'], wp['y'], wp['z']
        goal.a, goal.b, goal.c = wp['a'], wp['b'], wp['c']
        goal.speed = float(wp.get('speed', 0.1))
        goal.planner = wp.get('planner', 'lin')

        label = f'#{idx} ' if idx is not None else ''
        self.get_logger().info(
            f'[POSE] {label}→ x={goal.x} y={goal.y} z={goal.z} planner={goal.planner}'
        )
        if not self._pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveToPose server unavailable')
            return False

        done = threading.Event()
        result_holder: list[bool] = [False]

        def _on_result(future):
            result_holder[0] = future.result().result.success
            self.get_logger().info(f'[POSE] done: success={result_holder[0]}')
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
        return result_holder[0]

    def _send_waypoint(self, wp: dict, idx: int | None = None) -> bool:
        if _is_joints_waypoint(wp):
            return self._send_joints_goal(wp)
        return self._send_pose_goal(wp, idx=idx)

    # ── Main sequence ─────────────────────────────────────────────────────────

    def run(self, done_event: threading.Event):
        try:
            for i in range(self._n_iter):
                self.get_logger().info(f'======= Iteration {i + 1}/{self._n_iter} =======')

                self._send_waypoint(self._home)

                for idx, wp in enumerate(self._waypoints):
                    self._send_waypoint(wp, idx=idx)

                self._send_waypoint(self._home)
                time.sleep(self._delay)

            self.get_logger().info('======= Sequence complete =======')
        finally:
            self.close_bag()
            done_event.set()


def main():
    rclpy.init()
    node = MotionSequenceRunner()

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
