from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.task import Future
from webots_ros2_msgs.srv import SpawnNodeFromString
from rclpy.executors import ExternalShutdownException


@dataclass(frozen=True)
class CameraSpawnParams:
    camera_name: str
    translation: str
    rotation: str
    



class CameraSpawner(Node):

    def __init__(self, camera_params: CameraSpawnParams):
        super().__init__("camera_spawner")

        self._camera_params = camera_params
        self._urdf_template = self._generate_camera_urdf()
        self._proto_template = self._generate_camera_proto()



        # self.cli = self.create_client(
        #     SpawnNodeFromString, "/Ros2Supervisor/spawn_node_from_string"
        # )

        # while not self.cli.wait_for_service(timeout_sec=10):
        #     self.get_logger().warning(
        #         "service /Ros2Supervisor/spawn_node_from_string  not available, waiting again..."
        #     )

        # data = 'Camera { name "camera" translation 0 0 1.5 rotation 1 0 0 -1.5708 }'
        # req = SpawnNodeFromString.Request(data=data, check_fields=True)
        # self.get_logger().info(f"Calling spawn service for camera")
        # future = self.cli.call_async(req)
        # future.add_done_callback(self._response_callback)

    # def _response_callback(self, future: Future):
    #     try:
    #         response = future.result()
    #         self.get_logger().info("Camera spawned successfully")
    #     except Exception as e:
    #         self.get_logger().error(f"Service call failed: {e}")

    def _generate_camera_proto(self) -> str:
        ...

    def _generate_camera_urdf(self) -> str:
        ...