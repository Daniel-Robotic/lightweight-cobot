"""Spawn the nonphysical external supervisor marker without editing the world."""
import rclpy
from rclpy.node import Node
from webots_ros2_msgs.srv import SpawnNodeFromString


MARKER = '''DEF TCP_GIZMO Robot {
  name "tcp_gizmo"
  controller "<extern>"
  supervisor TRUE
  children [
    Shape {
      appearance PBRAppearance { baseColor 0.1 0.9 0.65 roughness 0.6 metalness 0 }
      geometry Sphere { radius 0.018 subdivision 2 }
    }
    Shape {
      appearance Appearance { material Material { emissiveColor 1 0.15 0.15 } }
      geometry IndexedLineSet { coord Coordinate { point [0 0 0, 0.08 0 0] } coordIndex [0 1 -1] }
    }
    Shape {
      appearance Appearance { material Material { emissiveColor 0.15 1 0.15 } }
      geometry IndexedLineSet { coord Coordinate { point [0 0 0, 0 0.08 0] } coordIndex [0 1 -1] }
    }
    Shape {
      appearance Appearance { material Material { emissiveColor 0.15 0.35 1 } }
      geometry IndexedLineSet { coord Coordinate { point [0 0 0, 0 0 0.08] } coordIndex [0 1 -1] }
    }
  ]
}'''


def main(args=None):
    rclpy.init(args=args)
    node = Node('tcp_gizmo_spawner')
    try:
        client = node.create_client(SpawnNodeFromString, '/Ros2Supervisor/spawn_node_from_string')
        if not client.wait_for_service(timeout_sec=60.0):
            raise RuntimeError('Webots supervisor unavailable; TCP marker was not spawned')
        future = client.call_async(SpawnNodeFromString.Request(data=MARKER, check_fields=True))
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        if not future.done() or not future.result().success:
            raise RuntimeError('Webots could not spawn TCP marker')
        node.get_logger().info('TCP marker spawned')
    finally:
        node.destroy_node()
        rclpy.shutdown()
