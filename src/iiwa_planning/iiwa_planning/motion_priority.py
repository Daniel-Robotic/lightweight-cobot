"""Wall-clock motion reservations and serialization, independent of ROS simulation time."""
import threading
import uuid


class LeaseError(RuntimeError):
    pass


class MotionLease:
    """Renew one owner until context exit; transport errors fail closed."""
    def __init__(self, exchange=None, *, on_lost=lambda: None, ttl=5.0,
                 renew_interval=1.0):
        self.owner = uuid.uuid4().hex
        self._exchange = exchange
        self._on_lost = on_lost
        self._ttl = ttl
        self._interval = renew_interval
        self._stop = threading.Event()
        self._lost = threading.Event()
        self._thread = None

    def __enter__(self):
        if self._exchange is not None:
            self._exchange(self.owner, True, self._ttl)
            self._thread = threading.Thread(target=self._renew, daemon=True)
            self._thread.start()
        return self

    def _renew(self):
        while not self._stop.wait(self._interval):
            try:
                self._exchange(self.owner, True, self._ttl)
            except Exception:
                self._lost.set()
                try:
                    self._on_lost()
                finally:
                    return

    @property
    def lost(self):
        return self._lost.is_set()

    def check(self):
        if self.lost:
            raise LeaseError('Motion priority reservation lost')

    def __exit__(self, exc_type, exc, tb):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            try:
                self._exchange(self.owner, False, self._ttl)
            except Exception:
                # Expiry bounds cleanup even if the service disappeared.
                pass
        if exc_type is None:
            self.check()
        return False


class MotionSerialiser:
    """One PlanningComponent owner; stop invalidates both planning and queued work."""
    def __init__(self):
        self.lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._generation = 0

    def generation(self):
        with self._state_lock:
            return self._generation

    def check(self, generation):
        if generation != self.generation():
            raise LeaseError('Motion interrupted by stop')

    def stop(self, callback):
        with self._state_lock:
            self._generation += 1
        callback()


def wait_response(future, timeout=2.0, *, available=lambda: True):
    """Wait only from worker threads; an independent executor services the future."""
    done = threading.Event()
    future.add_done_callback(lambda _: done.set())
    if timeout is None:
        # Acquisition is acknowledged only after a committed gizmo trajectory
        # finishes. Its duration can exceed the normal service RPC timeout.
        while not done.wait(.2):
            if not available():
                future.cancel()
                raise LeaseError('Motion priority service unavailable while waiting')
    elif not done.wait(timeout):
        future.cancel()
        raise LeaseError('Motion priority service response timed out')
    result = future.result()
    if result is None:
        raise LeaseError('Motion priority service returned no response')
    return result


class MotionPriorityClient:
    """Dedicated executor prevents lease starvation under concurrent motion requests."""
    def __init__(self, *, context=None):
        from rclpy.node import Node
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.executors import MultiThreadedExecutor
        from iiwa_msgs.srv import MotionPriority
        self._type = MotionPriority
        self._node = Node('motion_priority_' + uuid.uuid4().hex[:12], context=context,
                          start_parameter_services=False, use_global_arguments=False)
        self._client = self._node.create_client(
            MotionPriority, '/cobot/tcp_gizmo/priority',
            callback_group=ReentrantCallbackGroup())
        self._executor = MultiThreadedExecutor(num_threads=2, context=context)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        from rclpy.executors import ExternalShutdownException
        try:
            self._executor.spin()
        except ExternalShutdownException:
            pass

    def create_stop_service(self, callback):
        from std_srvs.srv import Trigger
        return self._node.create_service(Trigger, "/cobot/stop", callback)

    def exchange(self, owner, acquire, ttl):
        if not self._client.wait_for_service(timeout_sec=0.5):
            raise LeaseError('Motion priority service unavailable')
        request = self._type.Request(owner=owner, acquire=acquire, ttl=ttl)
        result = wait_response(
            self._client.call_async(request), timeout=None if acquire else 2.0,
            available=lambda: self._node.context.ok() and self._client.service_is_ready())
        if not result.success:
            raise LeaseError(result.message)

    def close(self):
        self._executor.shutdown(timeout_sec=2.0)
        self._thread.join(timeout=2.0)
        self._node.destroy_node()
