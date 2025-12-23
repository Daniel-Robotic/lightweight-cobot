# /Ros2Supervisor/spawn_node_from_string / webots_ros2_msgs/srv/SpawnNodeFromString
# /Ros2Supervisor/spawn_urdf_robot


from random import randint

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.task import Future
from webots_ros2_msgs.srv import SpawnNodeFromString


class ObjectSpawner(Node):
    def __init__(self):
        super().__init__("object_spawner")

        # параметры передаваемые
        self.declare_parameter('objects_path', '')

        # переменные
        self._object_count: int = randint(1, 5)
        self._spawned_count: int = 0
        self._call_in_progress: bool = False

        # Реализация
        self.cli = self.create_client(
            SpawnNodeFromString, "/Ros2Supervisor/spawn_node_from_string"
        )

        while not self.cli.wait_for_service(timeout_sec=10):
            self.get_logger().warning(
                "service /Ros2Supervisor/spawn_node_from_string  not available, waiting again..."
            )

        self._timer = self.create_timer(0.1, self.timer_callback)

    # TODO: переделать процесс спавна, должен сформировать все proto файлы, после чего заспавнить все объекты
    def timer_callback(self):
        # TODO: здесь должен получать объекты
        if self._spawned_count >= self._object_count:
            self.get_logger().info(f"All {self._object_count} objects spawned")
            self._timer.cancel()
            return

        if self._call_in_progress:
            return

        data = 'Solid { name "test_box2" translation 0 1 0.5 children [ Shape { appearance PBRAppearance { baseColor 0.901961 0.380392 0 } geometry Box { size 0.1 0.1 0.1 } } ] boundingObject Box { size 0.1 0.1 0.1 } physics Physics { } }'
        data = self._create_group(children=[data])

        req = SpawnNodeFromString.Request(data=data, check_fields=True)
        self.get_logger().info(
            f"Calling spawn service #{self._spawned_count + 1}/{self._object_count}"
        )
        self._call_in_progress = True
        future = self.cli.call_async(req)
        future.add_done_callback(self._response_callback)

    def _response_callback(self, future: Future):
        try:
            response = future.result()
            self.get_logger().info(f"Spawn service response: {response}")
        except Exception as e:
            self.get_logger().error(f"Service call field: {e}")
        finally:
            self._call_in_progress = False
            self._spawned_count += 1

    # Вспомогательные методы формирования кода для спавна объектов
    def _create_group(self, children: list[str]) -> str:
        return f"DEF OBJECTS Group {{children [{" ".join(children)}]}}"


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = ObjectSpawner()
            rclpy.spin(node=node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
