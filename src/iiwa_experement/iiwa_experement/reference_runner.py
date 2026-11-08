#!/usr/bin/env python3
"""Execute one reference experiment after the scene and MoveIt are ready."""
from __future__ import annotations

import sys
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .geometry import load_geometry
from .motion import MotionAdapter
from .operations import ReferenceOperations, SupervisorClient
from .planning_scene import PlanningSceneBridge
from .reference_scenarios import build_reference_sequence, execute_sequence
from .scenario import load_scenario
from .trace import RunResult, TraceWriter


SCENE_READY_SERVICE = '/iiwa_experement/scene_ready'
STATUS_TOPIC = '/iiwa_experement/reference_status'


class ReadinessError(RuntimeError):
    pass


class ReferenceRunner(Node):
    def __init__(self) -> None:
        super().__init__('reference_runner')
        package_share = Path(get_package_share_directory('iiwa_experement'))
        self.declare_parameter('experiment_id', 1)
        self.declare_parameter(
            'scene_config', str(package_share / 'config' / 'scenes.yaml'),
        )
        self.declare_parameter(
            'geometry_path', str(package_share / 'config' / 'geometry.yaml'),
        )
        self.declare_parameter('start_delay_sec', 3.0)
        self.declare_parameter('trace_dir', '~/.ros/iiwa_experement/runs')
        self.declare_parameter('readiness_timeout_sec', 180.0)

        self.experiment_id = int(self.get_parameter('experiment_id').value)
        self.scene_config = Path(str(self.get_parameter('scene_config').value))
        self.geometry_path = Path(str(self.get_parameter('geometry_path').value))
        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        self.trace_dir = str(self.get_parameter('trace_dir').value)
        self.readiness_timeout_sec = float(
            self.get_parameter('readiness_timeout_sec').value
        )

        self.spec = load_scenario(self.experiment_id, self.scene_config)
        self.geometry = load_geometry(self.geometry_path)
        self.trace = TraceWriter(
            self.experiment_id, self.trace_dir, sim_time=self.simulation_time,
        )
        self.status_publisher = self.create_publisher(String, STATUS_TOPIC, 10)
        self._scene_ready_client = self.create_client(Trigger, SCENE_READY_SERVICE)

        self.supervisor = SupervisorClient(self)
        self.motion = MotionAdapter(
            self,
            self.geometry,
            scene_check=lambda: (True, 'ok'),
        )
        self.bridge = PlanningSceneBridge(node=self)
        self.operations = ReferenceOperations(
            self,
            self.spec,
            self.geometry,
            self.supervisor,
            self.bridge,
            self.motion,
            event_sink=self.trace.event,
        )
        self.scene_ready_time: float | None = None

    # ------------------------------------------------------------------ time
    def simulation_time(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _spin(self, seconds: float = 0.1) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.05, end - time.monotonic()))

    def _publish_status(self, text: str) -> None:
        message = String()
        message.data = text
        self.status_publisher.publish(message)
        self.get_logger().info(text)

    # ------------------------------------------------------------------ wait
    def wait_scene_ready(self) -> float:
        if not self._scene_ready_client.wait_for_service(
            timeout_sec=self.readiness_timeout_sec,
        ):
            raise ReadinessError(f'{SCENE_READY_SERVICE} did not appear')
        deadline = time.monotonic() + self.readiness_timeout_sec
        while time.monotonic() < deadline:
            future = self._scene_ready_client.call_async(Trigger.Request())
            end = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            if future.done() and future.result().success:
                self.scene_ready_time = self.simulation_time()
                return self.scene_ready_time
            self._spin(0.5)
        raise ReadinessError('scene_ready did not become true')

    def wait_motion_ready(self) -> None:
        deadline = time.monotonic() + self.readiness_timeout_sec
        while time.monotonic() < deadline:
            if self.motion.action_client.wait_for_server(timeout_sec=1.0):
                if self.motion.named_client.wait_for_service(timeout_sec=1.0):
                    break
            self._spin(0.2)
        else:
            raise ReadinessError('motion action or named service is unavailable')

        deadline = time.monotonic() + self.readiness_timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.simulation_time() <= 0.0:
                continue
            fresh, _ = self.motion._fresh()
            tcp = self.motion.actual_tcp()
            if fresh and tcp is not None:
                return
        raise ReadinessError('joint_states/TF did not become fresh')

    def wait_delay(self) -> None:
        if self.scene_ready_time is None:
            raise ReadinessError('scene_ready time is unknown')
        start_at = self.scene_ready_time + self.start_delay_sec
        self._publish_status(
            f'experiment {self.experiment_id}: ready; waiting until '
            f'sim {start_at:.2f} s'
        )
        while self.simulation_time() < start_at:
            rclpy.spin_once(self, timeout_sec=0.05)
        self._publish_status(
            f'experiment {self.experiment_id}: starting at '
            f'sim {self.simulation_time():.2f} s'
        )

    # ------------------------------------------------------------------ run
    def execute(self) -> RunResult:
        try:
            self._publish_status(f'experiment {self.experiment_id}: waiting for scene')
            self.wait_scene_ready()
            self.supervisor.wait_ready(self.readiness_timeout_sec)
            self.wait_motion_ready()
            self.bridge.initialize(self.spec, self.geometry)
            self.wait_delay()
        except Exception as error:
            result = RunResult(
                self.experiment_id, 'failed', f'readiness: {error}',
                simulation_time=self.simulation_time(),
            )
            self.trace.finish(result)
            self._publish_status(f'experiment {self.experiment_id}: failed: {error}')
            return result

        sequence = build_reference_sequence(self.spec)
        self.trace.event({
            'event': 'route',
            'steps': [step.as_dict() for step in sequence],
        })
        self._publish_status(f'experiment {self.experiment_id}: executing route')
        result = execute_sequence(sequence, self)
        self.trace.finish(result)
        self._publish_status(
            f'experiment {self.experiment_id}: {result.status}: {result.message}'
        )
        return result

    @property
    def trace_path(self) -> Path:
        return self.trace.path


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = ReferenceRunner()
        result = node.execute()
        if result.status != 'completed':
            exit_code = 1
        print(f'[iiwa_experement] trace: {node.trace_path}')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print(f'[iiwa_experement] reference runner failed: {error}', file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            node.trace.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
