"""ROS facade that publishes fresh, transient-local scene snapshots."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from iiwa_experement.scenario import ScenarioSpec, load_scenario

from .catalog import SkillCatalog, DEFAULT_CATALOG
from .snapshot import build_envelope, build_scene


STATE_TOPIC = '/iiwa_experement/object_state'
READY_SERVICE = '/iiwa_experement/scene_ready'
SCENE_SERVICE = '/iiwa_scene_context/get_scene'
SCENE_TOPIC = '/iiwa_scene_context/scene_json'
DEFAULT_FRESHNESS_SEC = 2.0


def scene_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class SceneStore:
    """Pure state store: readiness, freshness and the scene revision rule.

    ``scene_revision`` advances only when the supervisor's discrete state
    version changes (grab, dock, place, delivery).  Pose updates of a held
    object keep the same version, so they do not create a new revision.
    """

    def __init__(
        self,
        spec: ScenarioSpec,
        catalog: SkillCatalog = DEFAULT_CATALOG,
        *,
        freshness_sec: float = DEFAULT_FRESHNESS_SEC,
    ) -> None:
        self.spec = spec
        self.catalog = catalog
        self.freshness_sec = float(freshness_sec)
        self.scene_ready = False
        self.ready_message = 'scene not ready'
        self.state: dict[str, Any] | None = None
        self.received_at: float | None = None
        self.scene_revision = 0
        self._last_version: int | None = None
        self._last_envelope: dict[str, Any] | None = None

    # ------------------------------------------------------------------ inputs
    def set_scene_ready(self, ready: bool, message: str = '') -> None:
        self.scene_ready = bool(ready)
        self.ready_message = message or ('scene ready' if ready else 'scene not ready')

    def update(self, state: Mapping[str, Any], received_at: float) -> dict[str, Any]:
        if not isinstance(state, Mapping):
            raise TypeError('supervisor state must be a mapping')
        data = dict(state)
        version = data.get('version')
        try:
            version_number = int(version)
        except (TypeError, ValueError) as error:
            raise ValueError('supervisor state has no integer version') from error
        if self._last_version is None:
            self.scene_revision = 1
        elif version_number != self._last_version:
            self.scene_revision += 1
        self._last_version = version_number
        self.state = data
        self.received_at = float(received_at)
        return data

    # ------------------------------------------------------------------ output
    def stale_reason(self, now: float) -> str | None:
        if not self.scene_ready:
            return f'scene is not ready ({self.ready_message})'
        if self.state is None or self.received_at is None:
            return 'no supervisor object state received yet'
        if float(now) - self.received_at > self.freshness_sec:
            return (
                'supervisor object state is stale '
                f'({float(now) - self.received_at:.2f} s old)'
            )
        return None

    def snapshot(self, now: float) -> tuple[dict[str, Any] | None, str | None]:
        reason = self.stale_reason(now)
        if reason is not None:
            return None, reason
        assert self.state is not None and self.received_at is not None
        assert self._last_version is not None
        envelope = build_envelope(
            build_scene(self.spec, self.state, self.catalog),
            scene_revision=self.scene_revision,
            state_version=self._last_version,
            stamp=float(self.received_at),
            experiment_id=self.spec.experiment_id,
        )
        self._last_envelope = envelope
        return envelope, None

    @property
    def last_envelope(self) -> dict[str, Any] | None:
        return self._last_envelope


class SceneContextNode(Node):
    def __init__(self) -> None:
        super().__init__('iiwa_scene_context')
        package_share = Path(get_package_share_directory('iiwa_experement'))
        self.declare_parameter('experiment_id', 1)
        self.declare_parameter(
            'scene_config', str(package_share / 'config' / 'scenes.yaml'),
        )
        self.declare_parameter('freshness_sec', DEFAULT_FRESHNESS_SEC)
        self.declare_parameter('publish_period_sec', 0.5)
        self.declare_parameter('ready_poll_period_sec', 1.0)

        experiment_id = int(self.get_parameter('experiment_id').value)
        scene_config = Path(str(self.get_parameter('scene_config').value))
        self.spec = load_scenario(experiment_id, scene_config)
        self.store = SceneStore(
            self.spec,
            DEFAULT_CATALOG,
            freshness_sec=float(self.get_parameter('freshness_sec').value),
        )
        self._last_published: tuple[int, int] | None = None
        self._ready_client = self.create_client(Trigger, READY_SERVICE)
        self._ready_request_pending = False
        self._service = self.create_service(
            Trigger, SCENE_SERVICE, self._handle_get_scene,
        )
        self._publisher = self.create_publisher(String, SCENE_TOPIC, scene_qos())
        self.create_subscription(
            String, STATE_TOPIC, self._on_state, 10,
        )
        self.create_timer(
            max(0.05, float(self.get_parameter('ready_poll_period_sec').value)),
            self._poll_ready,
        )
        self.create_timer(
            max(0.05, float(self.get_parameter('publish_period_sec').value)),
            self._publish_if_new,
        )
        self.get_logger().info(
            f'scene context started for experiment {experiment_id}'
        )

    # ------------------------------------------------------------------ inputs
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_state(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self.store.update(payload, self._now())
        except Exception as error:
            self.get_logger().error(f'ignoring invalid object state: {error}')
            return
        self._publish_if_new()

    def _poll_ready(self) -> None:
        if self._ready_request_pending:
            return
        if not self._ready_client.service_is_ready():
            self._ready_client.wait_for_service(timeout_sec=0.0)
            return
        self._ready_request_pending = True
        future = self._ready_client.call_async(Trigger.Request())
        future.add_done_callback(self._ready_result)

    def _ready_result(self, future) -> None:
        self._ready_request_pending = False
        try:
            response = future.result()
        except Exception as error:
            self.store.set_scene_ready(False, f'scene_ready call failed: {error}')
            return
        self.store.set_scene_ready(
            bool(response.success), str(response.message or ''),
        )

    # ------------------------------------------------------------------ output
    def _publish_if_new(self) -> None:
        envelope, _ = self.store.snapshot(self._now())
        if envelope is None:
            return
        key = (int(envelope['scene_revision']), int(envelope['state_version']))
        if key == self._last_published:
            return
        self._last_published = key
        message = String()
        message.data = json.dumps(envelope, ensure_ascii=False)
        self._publisher.publish(message)

    def _handle_get_scene(self, request, response):
        del request
        envelope, reason = self.store.snapshot(self._now())
        if envelope is None:
            response.success = False
            response.message = str(reason)
            return response
        response.success = True
        response.message = json.dumps(envelope, ensure_ascii=False)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = SceneContextNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print(f'[iiwa_scene_context] {error}')
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
