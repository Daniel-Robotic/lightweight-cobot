"""Spawn the configured batch once through the Webots supervisor service."""
import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from webots_ros2_msgs.srv import SpawnNodeFromString

from iiwa_utils.object_scene import build_objects, parse_objects


def spawn_objects(node):
    raw = json.loads(node.declare_parameter('objects_config', '{}').value)
    translation = node.declare_parameter('robot_translation', '0 0 0').value
    rotation = node.declare_parameter('robot_rotation', '0 0 1 0').value
    config = parse_objects(raw, lambda path: path)
    objects = build_objects(config, translation, rotation)
    if not objects:
        node.get_logger().info('Object spawning disabled or count is zero')
        return
    client = node.create_client(SpawnNodeFromString, '/Ros2Supervisor/spawn_node_from_string')
    if not client.wait_for_service(timeout_sec=60.0):
        raise RuntimeError('Webots supervisor unavailable; no objects spawned')
    for index, data in enumerate(objects):
        future = client.call_async(SpawnNodeFromString.Request(data=data))
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        if not future.done():
            # A timeout does not prove rejection: retrying could create duplicates.
            raise RuntimeError(f'Object {index + 1}: spawn timed out; result unknown, no retry')
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f'Object {index + 1} rejected; {index}/{len(objects)} spawned')
    node.get_logger().info(f'All {len(objects)} objects spawned')


def main(args=None):
    rclpy.init(args=args)
    node = Node('object_spawner')
    try:
        spawn_objects(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
