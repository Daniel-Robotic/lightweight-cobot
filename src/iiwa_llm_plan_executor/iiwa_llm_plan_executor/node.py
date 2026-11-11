"""ROS facade for the graph engine: submit, plan, status, cancel, logs."""
from __future__ import annotations

import json
import re
import threading
import time
import uuid
from pathlib import Path
from typing import Any, Mapping

import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
import webots_ros2_msgs.srv

from iiwa_experement.geometry import GeometrySpec, load_geometry
from iiwa_experement.planning_scene import (
    PlanningSceneBridge,
    attached_cap,
    attached_screw,
    attached_tool,
    cap_world_box,
    placed_cap_pose,
    screw_object_id,
)
from iiwa_experement.scenario import ScenarioSpec, load_scenario
from iiwa_msgs.srv import GetRunStatus
from iiwa_scene_context.catalog import DEFAULT_CATALOG
from iiwa_scene_context.node import SCENE_TOPIC

from .engine import GraphEngine
from .motion_policy import MotionPolicy, load_limits
from .moveit_backend import MoveItBackend
from .skill_context import SkillContext
from .skills import SkillRegistry


COMMAND_SERVICE = '/iiwa_experement/object_command'
STATE_TOPIC = '/iiwa_experement/object_state'
SCENE_SERVICE = '/iiwa_scene_context/get_scene'
SUBMIT_SERVICE = '/iiwa_llm_plan_executor/submit'
CANCEL_SERVICE = '/iiwa_llm_plan_executor/cancel'
STATUS_SERVICE = '/iiwa_llm_plan_executor/get_status'
PLAN_TOPIC = '/iiwa_llm_plan_executor/plan'
STATUS_TOPIC = '/iiwa_llm_plan_executor/status'
CONTROLLER_ACTION = '/iiwa_arm_controller/follow_joint_trajectory'
DEFAULT_TRACE_DIR = '~/.ros/iiwa_llm_plan_executor/runs'


def plan_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _safe_name(value: str) -> str:
    return re.sub(r'[^A-Za-z0-9_.-]+', '_', value)[:80] or 'run'


class RunLog:
    """Machine-readable JSONL log, one file per run id."""

    def __init__(self, directory: str) -> None:
        self.directory = Path(directory).expanduser()
        self._lock = threading.Lock()

    def write(self, payload: Mapping[str, Any]) -> None:
        try:
            self.directory.mkdir(parents=True, exist_ok=True)
            run_id = _safe_name(str(payload.get('run_id', 'run')))
            path = self.directory / f'{run_id}.jsonl'
            with self._lock, path.open('a', encoding='utf-8') as stream:
                stream.write(json.dumps(payload, ensure_ascii=False) + '\n')
        except Exception:
            pass


class SupervisorFacade:
    """Thread-safe supervisor client that never spins its own node."""

    def __init__(self, node: Node, callback_group) -> None:
        self.node = node
        self._lock = threading.Condition()
        self._latest: dict[str, Any] | None = None
        self._client = node.create_client(
            webots_ros2_msgs.srv.SetString,
            COMMAND_SERVICE,
            callback_group=callback_group,
        )
        node.create_subscription(
            String, STATE_TOPIC, self._on_state, 10,
            callback_group=callback_group,
        )

    def _on_state(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        with self._lock:
            self._latest = payload
            self._lock.notify_all()

    def state(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._latest or {})

    # Alias used by the skills.
    def latest_state(self) -> dict[str, Any]:
        return self.state()

    def latest(self) -> dict[str, Any]:
        return self.state()

    def command(
        self,
        op: str,
        object_id: str = '',
        *,
        target=None,
        target_name: str = '',
    ) -> dict[str, Any]:
        if not self._client.service_is_ready():
            if not self._client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f'{COMMAND_SERVICE} is unavailable')
        request_id = uuid.uuid4().hex
        payload: dict[str, Any] = {
            'request_id': request_id,
            'op': op,
            'object': object_id,
            'target_name': target_name,
        }
        if target is not None:
            payload['target'] = {
                'xyz': [float(value) for value in target.xyz],
                'rotation': [
                    [float(value) for value in row] for row in target.rotation
                ],
            }
        request = webots_ros2_msgs.srv.SetString.Request()
        request.value = json.dumps(payload)
        future = self._client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=15.0):
            raise RuntimeError(f'supervisor command {op} timed out')
        try:
            response = future.result()
        except Exception as error:
            raise RuntimeError(f'supervisor command {op} failed: {error}') from error
        accepted = bool(response.success)
        state = self._wait_for_result(request_id, timeout=3.0)
        last = (state or {}).get('last_command_result') or {}
        if last.get('request_id') != request_id:
            raise RuntimeError(
                f'supervisor did not publish the result of {op}'
            )
        if not accepted or not last.get('accepted', False):
            raise RuntimeError(
                last.get('error') or f'supervisor rejected {op}'
            )
        return state

    def _wait_for_result(self, request_id: str, timeout: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        with self._lock:
            while True:
                state = dict(self._latest or {})
                last = state.get('last_command_result') or {}
                if last.get('request_id') == request_id:
                    return state
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return state
                self._lock.wait(timeout=min(0.1, remaining))


class SceneAdapter:
    """Keep the MoveIt scene aligned with the supervisor's ownership state."""

    def __init__(
        self,
        bridge: PlanningSceneBridge,
        spec: ScenarioSpec,
        geometry: GeometrySpec,
    ) -> None:
        self.bridge = bridge
        self.spec = spec
        self.geometry = geometry

    def _sync(self) -> None:
        self.bridge.assert_synchronized(*self.bridge.expected_sets())

    def attach_tool(self, tool: str) -> None:
        self.bridge.attach(attached_tool(tool, self.geometry))
        self._sync()

    def dock_tool(self, tool: str) -> None:
        self.bridge.restore_world(f'grant_{tool}')
        self._sync()

    def attach_cap(self) -> None:
        self.bridge.attach(attached_cap(self.geometry))
        self._sync()

    def place_cap(self, seat) -> None:
        self.bridge.add_world(
            'grant_cap',
            cap_world_box(self.geometry),
            placed_cap_pose(seat, self.spec.body.yaw_deg),
        )
        self._sync()

    def attach_screw(self, screw_id: str) -> None:
        self.bridge.attach(attached_screw(screw_id, self.geometry))
        self._sync()

    def handoff_screw(self, screw_id: str) -> None:
        self.bridge.remove(screw_object_id(screw_id))
        self._sync()

    # Hooks used before entering a contact so the carried geometry cannot
    # collide with itself or with the target it is about to touch.
    def prepare_grab_tool(self, tool: str) -> None:
        self.bridge.remove(f'grant_{tool}')
        self._sync()

    def restore_tool(self, tool: str) -> None:
        self.bridge.restore_world(f'grant_{tool}')
        self._sync()

    def prepare_grab_cap(self, object_id: str) -> None:
        del object_id
        self.bridge.remove('grant_cap')
        self._sync()

    def prepare_grab_screw(self, screw_id: str) -> None:
        self.bridge.remove(screw_object_id(screw_id))
        self._sync()

    def restore_part(self, object_id: str) -> None:
        name = 'grant_cap' if object_id == 'cap' else screw_object_id(object_id)
        self.bridge.restore_world(name)
        self._sync()

    def prepare_dock_tool(self, tool: str) -> None:
        self.bridge.remove(f'grant_{tool}')
        self._sync()

    def prepare_place_cap(self) -> None:
        self.bridge.remove('grant_cap')
        self._sync()

    def prepare_handoff_screw(self, screw_id: str) -> None:
        self.bridge.remove(screw_object_id(screw_id))
        self._sync()


class PlannerExecutorNode(Node):
    def __init__(self) -> None:
        super().__init__('iiwa_llm_plan_executor')
        package_share = Path(get_package_share_directory('iiwa_experement'))
        executor_share = Path(
            get_package_share_directory('iiwa_llm_plan_executor')
        )
        self.declare_parameter('experiment_id', 20)
        self.declare_parameter(
            'scene_config', str(package_share / 'config' / 'scenes.yaml'),
        )
        self.declare_parameter(
            'geometry_path', str(package_share / 'config' / 'geometry.yaml'),
        )
        self.declare_parameter(
            'motion_limits_path',
            str(executor_share / 'config' / 'motion_limits.yaml'),
        )
        self.declare_parameter(
            'joint_limits_path',
            str(
                Path(get_package_share_directory('iiwa_config'))
                / 'config' / 'moveit' / 'joint_limits.yaml'
            ),
        )
        self.declare_parameter('start_delay_sec', 3.0)
        self.declare_parameter('scene_max_age_sec', 2.0)
        self.declare_parameter('max_steps', 512)
        self.declare_parameter('planning_group', 'iiwa_arm')
        self.declare_parameter('pose_link', 'tcp')
        self.declare_parameter('close_inspection_objects', ['cap'])
        self.declare_parameter('top_camera_topic', '/d455_top/image_raw/image_color')
        self.declare_parameter('hand_camera_topic', '/camera/color/image_color')
        self.declare_parameter('frame_timeout_sec', 2.0)
        self.declare_parameter('controller_action', CONTROLLER_ACTION)
        self.declare_parameter('trace_dir', DEFAULT_TRACE_DIR)

        self.experiment_id = int(self.get_parameter('experiment_id').value)
        self.spec = load_scenario(
            self.experiment_id, str(self.get_parameter('scene_config').value),
        )
        self.geometry = load_geometry(
            str(self.get_parameter('geometry_path').value),
        )
        self.limits = load_limits(
            str(self.get_parameter('motion_limits_path').value),
        )
        self.policy = MotionPolicy(self.limits)
        self.close_inspection = frozenset(
            str(value) for value in self.get_parameter(
                'close_inspection_objects',
            ).value
        )
        self.frame_timeout = float(
            self.get_parameter('frame_timeout_sec').value
        )
        self.scene_max_age = float(
            self.get_parameter('scene_max_age_sec').value
        )
        self.top_topic = str(self.get_parameter('top_camera_topic').value)
        self.hand_topic = str(self.get_parameter('hand_camera_topic').value)
        self.log = RunLog(str(self.get_parameter('trace_dir').value))

        self.callback_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        self._latest_scene: dict[str, Any] | None = None
        self._scene_ready_at: float | None = None
        self._last_joint: tuple[float, ...] | None = None
        self._last_joint_velocity: tuple[float, ...] | None = None
        self._last_joint_at: float | None = None
        self._moveit_ready = False
        self._bridge_ready = False
        self._motion_init_lock = threading.Lock()
        self._last_plan: dict[str, Any] | None = None
        self._last_status: dict[str, Any] | None = None
        self._frame_conditions = {
            self.top_topic: threading.Condition(),
            self.hand_topic: threading.Condition(),
        }
        self._frame_counts = {self.top_topic: 0, self.hand_topic: 0}
        self._frame_stamps: dict[str, float] = {}

        # ------------------------------------------------------------ interfaces
        self._plan_publisher = self.create_publisher(String, PLAN_TOPIC, plan_qos())
        self._status_publisher = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_service(
            webots_ros2_msgs.srv.SetString,
            SUBMIT_SERVICE,
            self._handle_submit,
            callback_group=self.callback_group,
        )
        self.create_service(
            webots_ros2_msgs.srv.SetString,
            CANCEL_SERVICE,
            self._handle_cancel,
            callback_group=self.callback_group,
        )
        self.create_service(
            GetRunStatus,
            STATUS_SERVICE,
            self._handle_get_status,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            SCENE_TOPIC,
            self._on_scene,
            plan_qos(),
            callback_group=self.callback_group,
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self._on_joints,
            10,
            callback_group=self.callback_group,
        )
        for topic in (self.top_topic, self.hand_topic):
            self.create_subscription(
                Image,
                topic,
                lambda message, name=topic: self._on_image(name, message),
                qos_profile_sensor_data,
                callback_group=self.callback_group,
            )

        self.supervisor = SupervisorFacade(self, self.callback_group)
        self._scene_client = self.create_client(
            Trigger, SCENE_SERVICE, callback_group=self.callback_group,
        )
        self._controller = ActionClient(
            self, FollowJointTrajectory,
            str(self.get_parameter('controller_action').value),
            callback_group=self.callback_group,
        )
        self.bridge = PlanningSceneBridge(
            node=self,
            use_sim_time=True,
            spin_fn=lambda timeout: time.sleep(max(0.0, float(timeout))),
        )
        self.adapter = SceneAdapter(self.bridge, self.spec, self.geometry)
        self.motion = MoveItBackend(
            self.geometry,
            self.policy,
            planner=self._make_planner(),
            freshness_guard=self._robot_fresh,
            scene_guard=self._scene_guard,
            current_joints=lambda: self._last_joint,
            before_plan=self._wait_stationary,
            on_event=self._on_motion_event,
        )
        self.skills = SkillRegistry(
            self.motion,
            self.supervisor,
            catalog=DEFAULT_CATALOG,
            scene_adapter=self.adapter,
        )
        self.engine = GraphEngine(
            self._scene_provider,
            self._context_factory,
            self.skills,
            catalog=DEFAULT_CATALOG,
            clock=self._now,
            on_plan=self._publish_plan,
            on_status=self._publish_status,
            on_replan=self._on_replan,
            on_stop=self._on_stop,
            robot_fresh=self._robot_fresh,
            threaded=True,
            min_start_delay_sec=float(
                self.get_parameter('start_delay_sec').value
            ),
            scene_max_age_sec=self.scene_max_age,
            max_steps=int(self.get_parameter('max_steps').value),
        )
        self.create_timer(
            0.5, self._readiness_tick, callback_group=self.callback_group,
        )
        self.get_logger().info(
            f'graph executor started for experiment {self.experiment_id}; '
            f'waiting for scene {SCENE_TOPIC}'
        )

    # ------------------------------------------------------------------ setup
    def _make_planner(self):
        import yaml

        from .moveit_backend import MoveItPyPlanner

        limits_path = Path(str(self.get_parameter('joint_limits_path').value))
        joint_limits = {}
        try:
            raw = yaml.safe_load(limits_path.read_text(encoding='utf-8')) or {}
            for name, entry in (raw.get('joint_limits') or {}).items():
                if isinstance(entry, dict):
                    low = max(
                        float(entry['min_position']),
                        float(entry.get('soft_min_position', entry['min_position'])),
                    )
                    high = min(
                        float(entry['max_position']),
                        float(entry.get('soft_max_position', entry['max_position'])),
                    )
                    joint_limits[name] = (low, high)
        except Exception as error:
            self.get_logger().warning(
                f'falling back to model joint bounds: {error}'
            )
        return MoveItPyPlanner(
            self.geometry,
            self.policy,
            planning_group=str(self.get_parameter('planning_group').value),
            pose_link=str(self.get_parameter('pose_link').value),
            joint_limits=joint_limits,
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_joints(self, message: JointState) -> None:
        if len(message.position) == 7:
            self._last_joint = tuple(float(value) for value in message.position)
            if len(message.velocity) == 7:
                self._last_joint_velocity = tuple(
                    float(value) for value in message.velocity
                )
            self._last_joint_at = self._now()

    def _on_image(self, topic: str, message: Image) -> None:
        condition = self._frame_conditions.get(topic)
        if condition is None:
            return
        stamp = (
            message.header.stamp.sec
            + message.header.stamp.nanosec * 1e-9
        )
        if stamp <= 0.0:
            stamp = self._now()
        with condition:
            self._frame_counts[topic] += 1
            self._frame_stamps[topic] = stamp
            condition.notify_all()

    def _on_scene(self, message: String) -> None:
        try:
            envelope = json.loads(message.data)
        except Exception:
            return
        if not isinstance(envelope, dict):
            return
        now = self._now()
        with self._lock:
            self._latest_scene = envelope
            stamp = envelope.get('timestamp')
            try:
                stamp_value = float(stamp)
            except (TypeError, ValueError):
                stamp_value = None
            # Ignore impossible future stamps from a stale/ghost publisher.
            candidate = (
                stamp_value
                if stamp_value is not None and stamp_value <= now + 1.0
                else now
            )
            if self._scene_ready_at is None or candidate < self._scene_ready_at:
                self._scene_ready_at = candidate

    def _scene_provider(self) -> dict[str, Any] | None:
        with self._lock:
            envelope = self._latest_scene
        if envelope is None:
            return None
        try:
            stamp = float(envelope.get('timestamp'))
        except (TypeError, ValueError):
            return None
        if self._now() - stamp > self.scene_max_age:
            return None
        return envelope

    def _robot_fresh(self) -> tuple[bool, str]:
        if self._last_joint is None or self._last_joint_at is None:
            return False, 'no /joint_states received yet'
        age = self._now() - self._last_joint_at
        if age > self.limits.joint_state_max_age_sec:
            return False, f'/joint_states is stale ({age:.2f} s old)'
        return True, 'ok'

    def _wait_stationary(
        self,
        timeout_sec: float = 4.0,
        tolerance: float = 0.01,
    ) -> None:
        start = time.monotonic()
        stable_since = None
        while time.monotonic() - start < timeout_sec:
            velocities = self._last_joint_velocity
            if velocities is not None and all(
                abs(value) <= tolerance for value in velocities
            ):
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= 0.2:
                    return
            else:
                stable_since = None
            time.sleep(0.05)
        raise RuntimeError(
            f'robot did not become stationary within {timeout_sec:.2f} s'
        )

    def _scene_guard(self) -> tuple[bool, str]:
        if not self._bridge_ready:
            return False, 'planning scene bridge is not initialized'
        try:
            self.bridge.assert_synchronized(
                *self.bridge.expected_sets(), timeout_sec=2.0,
            )
        except Exception as error:
            return False, f'planning scene is not synchronized: {error}'
        return True, 'ok'

    def _ensure_motion_ready(self) -> None:
        if self._moveit_ready and self._bridge_ready:
            return
        if not self._motion_init_lock.acquire(blocking=False):
            return
        try:
            if not self._moveit_ready:
                try:
                    self.motion.planner.ensure_ready()
                    self._moveit_ready = True
                except Exception as error:
                    self._moveit_ready = False
                    self.get_logger().warning(f'MoveIt is not ready yet: {error}')
            if self._moveit_ready and not self._bridge_ready:
                try:
                    self.bridge.initialize(self.spec, self.geometry)
                    self._bridge_ready = True
                except Exception as error:
                    self._bridge_ready = False
                    self.get_logger().warning(
                        f'planning scene bridge is not ready yet: {error}'
                    )
        finally:
            self._motion_init_lock.release()

    def _readiness_tick(self) -> None:
        self._ensure_motion_ready()
        controller_ready = self._controller.server_is_ready()
        fresh, _ = self._robot_fresh()
        scene = self._scene_provider()
        scene_ready = scene is not None
        motion_ready = (
            self._moveit_ready and self._bridge_ready and controller_ready
            and fresh
        )
        self.engine.update_readiness(
            scene_ready=scene_ready,
            ready_at=self._scene_ready_at,
            motion_ready=motion_ready,
            reason=(
                'ready' if scene_ready and motion_ready else
                f'scene={scene_ready} moveit={self._moveit_ready} '
                f'bridge={self._bridge_ready} controller={controller_ready} '
                f'joints={fresh}'
            ),
        )

    # ------------------------------------------------------------------ frames
    def _frame_provider(self, name: str) -> float | None:
        topic = self.top_topic if name == 'top' else self.hand_topic
        condition = self._frame_conditions.get(topic)
        if condition is None:
            return None
        deadline = time.monotonic() + self.frame_timeout
        with condition:
            baseline = self._frame_counts[topic]
            while self._frame_counts[topic] <= baseline:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                condition.wait(timeout=remaining)
            return self._frame_stamps.get(topic)

    # ------------------------------------------------------------------ engine
    def _context_factory(self, envelope: Mapping[str, Any], scene_json) -> SkillContext:
        del scene_json
        context = SkillContext(
            spec=self.spec,
            geometry=self.geometry,
            scene_revision=int(envelope.get('scene_revision', 0)),
        )
        context.frame_provider = self._frame_provider
        context.scene_clear_provider = lambda: True
        context.notifier = self._notify_operator
        context.scene_adapter = self.adapter
        context.close_inspection_objects = self.close_inspection
        return context

    def _publish_plan(self, payload: dict[str, Any]) -> None:
        self._last_plan = dict(payload)
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self._plan_publisher.publish(message)
        self.log.write({
            'run_id': payload.get('run_id'),
            'event': 'plan_accepted',
            'scene_revision': payload.get('scene_revision'),
            'response': payload.get('response'),
        })
        self.get_logger().info(
            f"accepted graph for run {payload.get('run_id')} "
            f"(scene_revision={payload.get('scene_revision')})"
        )

    def _publish_status(self, payload: dict[str, Any]) -> None:
        self._last_status = dict(payload)
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self._status_publisher.publish(message)
        self.log.write(payload)

    def _on_replan(self, envelope: dict[str, Any] | None) -> None:
        run_id = (self._last_status or {}).get('run_id')
        self.log.write({
            'run_id': run_id,
            'event': 'replan_required',
            'scene_revision': None if envelope is None else envelope.get('scene_revision'),
        })
        if envelope is not None:
            self._publish_status({
                'run_id': run_id,
                'state': 'replan_scene',
                'reason': 'fresh scene snapshot for the external planner',
                'scene_revision': envelope.get('scene_revision'),
                'scene': envelope,
                'timestamp': self._now(),
            })

    def _on_stop(self, reason: str) -> None:
        self.motion.stop(reason)

    def _on_motion_event(self, payload: dict[str, Any]) -> None:
        self.log.write({**payload, 'event': payload.get('event', 'motion')})

    def _notify_operator(self, reason: str, details: dict[str, Any]) -> bool:
        self.get_logger().warning(f'operator required: {reason}')
        self._publish_status({
            'run_id': (self._last_status or {}).get('run_id'),
            'state': 'operator_notification',
            'reason': reason,
            'details': details,
            'timestamp': self._now(),
        })
        return True

    # ------------------------------------------------------------------ services
    def _handle_submit(self, request, response):
        try:
            payload = json.loads(request.value)
            if not isinstance(payload, dict):
                raise ValueError('submit payload must be a JSON object')
            run_id = str(payload.get('run_id', '')).strip()
            scene_revision = int(payload.get('scene_revision'))
            graph = payload.get('response')
            if not isinstance(graph, (str, dict)):
                raise ValueError('response must be a JSON string or object')
        except Exception as error:
            response.success = False
            self.get_logger().error(f'invalid submit request: {error}')
            return response
        result = self.engine.submit(run_id, graph, scene_revision)
        response.success = bool(result.accepted)
        if result.accepted:
            self.get_logger().info(
                f'run {run_id}: {result.code} ({result.message})'
            )
        else:
            self.get_logger().warning(
                f'run {run_id} rejected: {result.code}: {result.message}'
            )
            self._publish_status({
                'run_id': run_id,
                'state': 'rejected',
                'reason': f'{result.code}: {result.message}',
                'terminal': result.code,
                'mounted_tool': None,
                'held_object': None,
                'scene_revision': scene_revision,
                'timestamp': self._now(),
            })
        return response

    def _handle_cancel(self, request, response):
        try:
            try:
                payload = json.loads(request.value)
                run_id = (
                    str(payload.get('run_id', ''))
                    if isinstance(payload, dict) else str(payload)
                )
            except json.JSONDecodeError:
                run_id = str(request.value)
            result = self.engine.cancel(run_id)
            response.success = bool(result.accepted)
        except Exception as error:
            self.get_logger().error(f'cancel failed: {error}')
            response.success = False
        return response

    def _handle_get_status(self, request, response):
        status = self.engine.get_status(request.run_id)
        response.found = bool(status.found)
        response.status_json = status.to_json()
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = PlannerExecutorNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print(f'[iiwa_llm_plan_executor] {error}')
        exit_code = 1
    finally:
        if node is not None:
            try:
                node.motion.stop('node shutdown')
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
