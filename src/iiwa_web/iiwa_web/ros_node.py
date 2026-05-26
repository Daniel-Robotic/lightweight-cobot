import time
import rclpy
import threading

from rclpy.node import Node
from rclpy.action import ActionClient


class CobotWebNode(Node):
    def __init__(self):
        super().__init__('cobot_web_node')

        self._topic_cache: dict = {}
        self._publishers: dict = {}
        self._service_clients: dict = {}
        self._action_clients: dict = {}
        self._lock = threading.Lock()

    def subscribe(self, topic_name: str, msg_type):
        if topic_name not in self._topic_cache:
            self._topic_cache[topic_name] = None
            self.create_subscription(
                msg_type,
                topic_name,
                lambda msg, t=topic_name: self._handle_message(t, msg),
                10
            )
            self.get_logger().info(f'Subscribed to topic: {topic_name}')

    def publish(self, topic_name: str, message_type, msg):
        if topic_name not in self._publishers:
            self._publishers[topic_name] = self.create_publisher(message_type, topic_name, 10)
            self.get_logger().info(f'Created publisher for topic: {topic_name}')
        
        self._publishers[topic_name].publish(msg)

    def get_latest(self, topic_name: str):
        with self._lock:
            return self._topic_cache.get(topic_name)

    def _handle_message(self, topic_name: str, msg):
        with self._lock:
            self._topic_cache[topic_name] = msg

    def call_service(self, srv_type, srv_name: str, request, timeout: float = 5.0):
        if srv_name not in self._service_clients:
            self._service_clients[srv_name] = self.create_client(srv_type, srv_name)

        client = self._service_clients[srv_name]
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"Сервис '{srv_name}' недоступен")

        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise TimeoutError(f"Таймаут вызова сервиса '{srv_name}'")
            time.sleep(0.01)

        return future.result()

    def send_action(self, action_type, action_name: str, goal, timeout: float = 30.0):
        if action_name not in self._action_clients:
            self._action_clients[action_name] = ActionClient(self, action_type, action_name)

        client = self._action_clients[action_name]
        if not client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(f"Action сервер '{action_name}' недоступен")

        goal_future = client.send_goal_async(goal)
        deadline = time.monotonic() + 5.0
        while not goal_future.done():
            if time.monotonic() > deadline:
                raise TimeoutError(f"Таймаут принятия goal '{action_name}'")
            time.sleep(0.01)

        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            raise RuntimeError(f"Goal отклонён сервером '{action_name}'")

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + timeout
        while not result_future.done():
            if time.monotonic() > deadline:
                raise TimeoutError(f"Таймаут выполнения action '{action_name}'")
            time.sleep(0.05)

        return result_future.result()



_bridge: CobotWebNode = None

def init_ros_node() -> None:
    global _bridge
    rclpy.init()
    _bridge = CobotWebNode()
    
    thread = threading.Thread(target=rclpy.spin, args=(_bridge,), daemon=True)
    thread.start()


def get_bridge() -> CobotWebNode:
    global _bridge
    if _bridge is None:
        raise RuntimeError("ROS node not initialized. Call init_ros_node() first.")
    return _bridge
