"""Deterministic scene spawner and readiness service."""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_srvs.srv import Trigger
from webots_ros2_msgs.srv import SpawnNodeFromString

from .scenario import ScenarioSpec, load_scenario
from .webots_nodes import build_nodes


SPAWN_SERVICE = '/Ros2Supervisor/spawn_node_from_string'
READY_SERVICE = '/iiwa_experement/scene_ready'
DEFAULT_SERVICE_TIMEOUT = 120.0
DEFAULT_RESPONSE_TIMEOUT = 30.0

_NAME_RE = re.compile(r'name "([^"]+)"')


class SpawnError(RuntimeError):
    """A spawn request failed or its outcome is unknown."""


@dataclass
class Readiness:
    ready: bool = False
    message: str = 'scene not ready'


@dataclass
class ScenePlan:
    experiment_id: int
    spec: ScenarioSpec
    nodes: list[str]
    readiness: Readiness = field(default_factory=Readiness)


def node_name(data: str) -> str:
    match = _NAME_RE.search(data)
    return match.group(1) if match else 'unnamed object'


def build_plan(
    experiment_id: int,
    scene_config: Path | str,
    asset_root: Path | str,
) -> ScenePlan:
    """Validate the experiment and prepare every node before spawning."""
    spec = load_scenario(experiment_id, scene_config)
    return ScenePlan(experiment_id, spec, build_nodes(spec, asset_root))


def spawn_nodes(
    node,
    client,
    nodes: list[str],
    *,
    service_timeout_sec: float = DEFAULT_SERVICE_TIMEOUT,
    response_timeout_sec: float = DEFAULT_RESPONSE_TIMEOUT,
    spin_until_future_complete=None,
) -> None:
    """Spawn nodes sequentially through the Webots supervisor.

    A timeout on the response is ambiguous: the first request may still have
    been executed. It is reported as an error and is never retried, because a
    retry could create a duplicate object.
    """
    spin = spin_until_future_complete or rclpy.spin_until_future_complete
    try:
        available = client.wait_for_service(timeout_sec=service_timeout_sec)
    except Exception as error:  # pragma: no cover - depends on rclpy internals
        raise SpawnError(f'Webots supervisor service failed: {error}') from error
    if not available:
        raise SpawnError(
            f'Webots supervisor service {SPAWN_SERVICE} unavailable after '
            f'{service_timeout_sec:g} s; no objects spawned'
        )

    logger = node.get_logger()
    total = len(nodes)
    for index, data in enumerate(nodes, start=1):
        name = node_name(data)
        future = client.call_async(SpawnNodeFromString.Request(data=data))
        spin(node, future, timeout_sec=response_timeout_sec)
        if not future.done():
            raise SpawnError(
                f'{name}: spawn request timed out after {response_timeout_sec:g} s; '
                'result unknown, no retry'
            )
        try:
            response = future.result()
        except Exception as error:
            raise SpawnError(f'{name}: spawn call failed: {error}') from error
        if response is None:
            raise SpawnError(f'{name}: spawn returned no response')
        if not response.success:
            raise SpawnError(
                f'{name}: Webots rejected the object ({index - 1}/{total} accepted)'
            )
        logger.info(f'Spawned {index}/{total}: {name}')


def execute_plan(
    plan: ScenePlan,
    client,
    node,
    *,
    service_timeout_sec: float = DEFAULT_SERVICE_TIMEOUT,
    response_timeout_sec: float = DEFAULT_RESPONSE_TIMEOUT,
    spin_until_future_complete=None,
) -> None:
    """Run a plan and publish readiness only after the final success."""
    try:
        spawn_nodes(
            node,
            client,
            plan.nodes,
            service_timeout_sec=service_timeout_sec,
            response_timeout_sec=response_timeout_sec,
            spin_until_future_complete=spin_until_future_complete,
        )
    except Exception as error:
        plan.readiness.ready = False
        plan.readiness.message = (
            f'experiment {plan.experiment_id}: {error}'
        )
        raise
    plan.readiness.ready = True
    plan.readiness.message = (
        f'experiment {plan.experiment_id}: {len(plan.nodes)} objects spawned'
    )


class SceneSpawnerNode(Node):
    def __init__(self) -> None:
        super().__init__('scene_spawner')
        default_config = str(
            Path(get_package_share_directory('iiwa_experement'))
            / 'config'
            / 'scenes.yaml'
        )
        self.declare_parameter('experiment_id', 0)
        self.declare_parameter('scene_config', default_config)
        self._readiness = Readiness()
        self.create_service(Trigger, READY_SERVICE, self._handle_ready)

    def _handle_ready(self, request, response):
        response.success = self._readiness.ready
        response.message = self._readiness.message
        return response

    def run(self) -> None:
        experiment_id = int(self.get_parameter('experiment_id').value)
        scene_config = str(self.get_parameter('scene_config').value)
        asset_root = (
            Path(get_package_share_directory('iiwa_description'))
            / 'objects'
            / 'grant'
        )

        self.get_logger().info(
            f'Preparing experiment {experiment_id} from {scene_config}'
        )
        plan = build_plan(experiment_id, scene_config, asset_root)
        self._readiness = plan.readiness
        client = self.create_client(SpawnNodeFromString, SPAWN_SERVICE)
        execute_plan(plan, client, self)
        self.get_logger().info(self._readiness.message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SceneSpawnerNode()
    exit_code = 0
    try:
        node.run()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        node.get_logger().error(f'Scene spawn failed: {error}')
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
