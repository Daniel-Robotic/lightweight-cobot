"""Mirror the Webots scene into MoveIt and keep attached objects in sync.

The pure builders in this module are used by the unit tests; the
``PlanningSceneBridge`` class is the thin ROS adapter used by the reference
runner.  A trajectory is allowed only after ``assert_synchronized`` has seen
the expected object ids in the monitored planning scene.
"""
from __future__ import annotations

import copy
import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Sequence

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
)
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive

from .geometry import Box, GeometrySpec, Transform, object_pose_transform
from .scenario import ScenarioSpec


SCENE_TOPIC = '/planning_scene'
MONITORED_TOPIC = '/monitored_planning_scene'
GET_SCENE_SERVICE = '/get_planning_scene'
SCENE_FRAME = 'base_link'
# The tool is rigidly bolted to the flange, so contact between its collision
# box and the wrist/flange links is expected and allowed.  Without 'link7'
# the wide vacuum box reads as a false collision against the wrist housing.
TOUCH_LINKS = (
    'tcp', 'patron', 'link7', 'camera_holder', 'camera_corner',
    'camera_hand', 'camera_optical_frame',
)
ROBOT_LINKS = (
    'base_link', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6',
    'link7', 'patron', 'camera_holder', 'camera_corner', 'camera_hand',
    'camera_optical_frame', 'tcp',
)
DEFAULT_SRDF = 'iiwa_config/config/moveit/iiwa7.srdf'


def load_srdf_disabled_pairs(path) -> list[tuple[str, str]]:
    root = ET.parse(str(path)).getroot()
    pairs = []
    for element in root.findall('disable_collisions'):
        first = element.get('link1')
        second = element.get('link2')
        if first and second:
            pairs.append((first, second))
    return pairs


def build_collision_matrix(
    object_ids: Iterable[str],
    *,
    disabled_pairs: Iterable[tuple[str, str]] = (),
    extra_allowed: Iterable[tuple[str, str]] = (),
) -> tuple[list[str], list[list[bool]]]:
    """Build a complete ACM: only explicitly disabled pairs are allowed.

    This mirrors the SRDF and keeps every other robot/object and object/object
    pair checked.  It replaces an empty ACM that would otherwise make the
    planning scene treat adjacent robot links as collisions.
    """
    names = list(ROBOT_LINKS)
    for name in object_ids:
        if name not in names:
            names.append(name)
    matrix = [[False] * len(names) for _ in names]
    for index in range(len(names)):
        matrix[index][index] = True
    for first, second in list(disabled_pairs) + list(extra_allowed):
        if first in names and second in names:
            row = names.index(first)
            column = names.index(second)
            matrix[row][column] = True
            matrix[column][row] = True
    return names, matrix


class SceneSynchronizationError(RuntimeError):
    """The MoveIt scene does not match the requested object set."""


def screw_object_id(screw_id: str) -> str:
    return f'grant_screw_{int(screw_id.split("_")[1]):02d}'


# Lift from an object node pose to the collision-box centre, matching
# build_world_objects.  A node origin sits on the lower face/axis of the OBJ
# and is rotated so the local Y axis becomes world Z; the collision box is a
# flat, yaw-only box centred above the support surface.
#
# The installed hatch is sandwiched between two hard constraints: its box
# must clear the conservative body box below AND the suction-cup box above at
# the place contact pose.  A zero-distance touch on either side reads as a
# scene contact and blocks every later plan from that state, so the world
# model of the hatch is 1 mm thinner than the real plate, leaving 0.5 mm of
# clearance on both sides.
CAP_BOX_LIFT = 0.004165
CAP_BOX_THICKNESS = 0.00267
SCREW_BOX_LIFT = 0.0055
TOOL_BOX_LIFT = 0.070


def cap_world_box(geometry: GeometrySpec) -> Box:
    """Collision box of the hatch as a world object (thinner than the plate)."""
    return Box(
        size=(
            geometry.cap_collision.size[0],
            geometry.cap_collision.size[1],
            CAP_BOX_THICKNESS,
        ),
        offset=geometry.cap_collision.offset,
    )


def placed_cap_pose(seat: Transform, body_yaw_deg: float) -> Transform:
    """Collision-box pose of the installed hatch: flat, centred on the plate."""
    return Transform.from_xyz_rpy(
        [seat.xyz[0], seat.xyz[1], seat.xyz[2] + CAP_BOX_LIFT],
        [0.0, 0.0, body_yaw_deg],
        parent='world',
        child='cap',
    )


@dataclass(frozen=True)
class SceneObject:
    object_id: str
    box: Box
    pose: Transform


@dataclass(frozen=True)
class AttachedSpec:
    object_id: str
    box: Box
    relative_pose: Transform
    touch_links: tuple[str, ...]
    frame_id: str | None = None


def _box_at(pose: Transform, box: Box) -> Box:
    del pose
    return box


def _pose_with_offset(pose: Transform, offset) -> Transform:
    translation = pose.position + pose.matrix @ np.asarray(offset, dtype=float)
    return Transform(
        xyz=tuple(float(value) for value in translation),
        rotation=pose.rotation,
        parent=pose.parent,
        child=pose.child,
    )


def _rotated_pose(x: float, y: float, z: float, yaw_deg: float) -> Transform:
    return Transform.from_xyz_rpy(
        [x, y, z], [0.0, 0.0, yaw_deg], parent='world', child='scene_object',
    )


def build_world_objects(spec: ScenarioSpec, geometry: GeometrySpec) -> list[SceneObject]:
    """Return the collision objects present in the initial Webots scene."""
    objects: list[SceneObject] = []
    table = geometry.scene_collisions['table']
    objects.append(SceneObject(
        'grant_table', table, _rotated_pose(0.0, 0.0, table.offset[2], 0.0),
    ))

    fixture = geometry.scene_collisions['fixture_base']
    objects.append(SceneObject(
        'grant_fixture',
        fixture,
        _rotated_pose(
            spec.fixture.x,
            spec.fixture.y,
            spec.fixture.z + fixture.offset[2],
            spec.fixture.yaw_deg,
        ),
    ))
    rail = geometry.scene_collisions['fixture_rail']
    sine = math.sin(math.radians(spec.fixture.yaw_deg))
    cosine = math.cos(math.radians(spec.fixture.yaw_deg))
    for sign, suffix in ((-1.0, 'a'), (1.0, 'b')):
        objects.append(SceneObject(
            f'grant_fixture_rail_{suffix}',
            rail,
            _rotated_pose(
                spec.fixture.x + sign * rail.offset[0] * cosine,
                spec.fixture.y + sign * rail.offset[0] * sine,
                spec.fixture.z + rail.offset[2],
                spec.fixture.yaw_deg,
            ),
        ))

    body = geometry.scene_collisions['body']
    objects.append(SceneObject(
        'grant_body',
        body,
        _rotated_pose(
            spec.body.x,
            spec.body.y,
            spec.body.z + body.offset[2],
            spec.body.yaw_deg,
        ),
    ))

    rack = geometry.scene_collisions['rack']
    objects.append(SceneObject(
        'grant_rack',
        rack,
        _rotated_pose(
            spec.rack.x,
            spec.rack.y,
            spec.rack.z + rack.offset[2],
            spec.rack.yaw_deg,
        ),
    ))

    if spec.tray_pose is not None:
        tray = spec.tray_pose
        base = geometry.scene_collisions['tray_base']
        objects.append(SceneObject(
            'grant_tray',
            base,
            _rotated_pose(
                tray.x, tray.y, tray.z + base.offset[2], tray.yaw_deg,
            ),
        ))
        wall_x = geometry.scene_collisions['tray_wall_x']
        for sign, suffix in ((-1.0, 'a'), (1.0, 'b')):
            dx, dy = math.cos(math.radians(tray.yaw_deg)) * sign * wall_x.offset[0], \
                math.sin(math.radians(tray.yaw_deg)) * sign * wall_x.offset[0]
            objects.append(SceneObject(
                f'grant_tray_wall_x_{suffix}',
                wall_x,
                _rotated_pose(
                    tray.x + dx,
                    tray.y + dy,
                    tray.z + wall_x.offset[2],
                    tray.yaw_deg,
                ),
            ))
        wall_y = geometry.scene_collisions['tray_wall_y']
        for sign, suffix in ((-1.0, 'a'), (1.0, 'b')):
            dx = -math.sin(math.radians(tray.yaw_deg)) * sign * wall_y.offset[1]
            dy = math.cos(math.radians(tray.yaw_deg)) * sign * wall_y.offset[1]
            objects.append(SceneObject(
                f'grant_tray_wall_y_{suffix}',
                wall_y,
                _rotated_pose(
                    tray.x + dx,
                    tray.y + dy,
                    tray.z + wall_y.offset[2],
                    tray.yaw_deg,
                ),
            ))
        for screw in spec.screws:
            objects.append(SceneObject(
                screw_object_id(screw.name),
                geometry.screw_collision,
                _rotated_pose(
                    screw.pose.x,
                    screw.pose.y,
                    screw.pose.z + SCREW_BOX_LIFT,
                    screw.pose.yaw_deg,
                ),
            ))

    for tool in spec.tools:
        collision = geometry.tool_collisions[tool.name]
        size = (collision.size[0], collision.size[1], 0.140)
        objects.append(SceneObject(
            f'grant_{tool.name}',
            Box(size=size, offset=(0.0, 0.0, 0.070)),
            _rotated_pose(
                tool.pose.x, tool.pose.y, tool.pose.z + TOOL_BOX_LIFT,
                tool.pose.yaw_deg + 180.0,
            ),
        ))

    if spec.cap_state != 'absent':
        cap_pose = spec.cap_pose
        assert cap_pose is not None
        objects.append(SceneObject(
            'grant_cap',
            cap_world_box(geometry),
            _rotated_pose(
                cap_pose.x, cap_pose.y, cap_pose.z + CAP_BOX_LIFT,
                cap_pose.yaw_deg,
            ),
        ))
    return objects


def attached_tool(name: str, geometry: GeometrySpec) -> AttachedSpec:
    collision = geometry.tool_collisions[name]
    return AttachedSpec(
        object_id=f'grant_{name}',
        box=Box(
            size=(collision.size[0], collision.size[1], 0.140),
            offset=collision.offset,
        ),
        relative_pose=Transform.from_xyz_rpy(
            [0.0, 0.0, collision.offset[2] + 0.0552],
            [0.0, 0.0, 0.0],
            parent='patron',
            child=f'grant_{name}',
        ),
        touch_links=TOUCH_LINKS,
    )


def attached_cap(geometry: GeometrySpec) -> AttachedSpec:
    # The plate box stops 1 mm short of the support surface (body/table at
    # the grasp pose): a zero-distance touch reads as a scene contact and
    # blocks the mandatory axial retreat right after the grasp.
    box = Box(
        size=(geometry.cap_collision.size[0], geometry.cap_collision.size[1],
              0.00267),
        offset=(0.0, 0.0, 0.141335),
    )
    return AttachedSpec(
        object_id='grant_cap',
        box=box,
        relative_pose=Transform.from_xyz_rpy(
            [0.0, 0.0, 0.141335 + 0.0552],
            [0.0, 0.0, 0.0],
            parent='patron',
            child='grant_cap',
        ),
        touch_links=TOUCH_LINKS,
    )


def attached_screw(screw_id: str, geometry: GeometrySpec) -> AttachedSpec:
    # Same 1 mm clearance at the tray floor as in attached_cap.
    box = Box(size=(0.0055, 0.0055, 0.008), offset=(0.0, 0.0, 0.144))
    object_id = screw_object_id(screw_id)
    return AttachedSpec(
        object_id=object_id,
        box=box,
        relative_pose=Transform.from_xyz_rpy(
            [0.0, 0.0, 0.144 + 0.0552],
            [0.0, 0.0, 0.0],
            parent='patron',
            child=object_id,
        ),
        touch_links=TOUCH_LINKS,
    )


def _pose_message(transform: Transform) -> Pose:
    from scipy.spatial.transform import Rotation

    quaternion = Rotation.from_matrix(transform.matrix).as_quat()
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (
        float(transform.xyz[0]),
        float(transform.xyz[1]),
        float(transform.xyz[2]),
    )
    pose.orientation.x = float(quaternion[0])
    pose.orientation.y = float(quaternion[1])
    pose.orientation.z = float(quaternion[2])
    pose.orientation.w = float(quaternion[3])
    return pose


def collision_object(
    object_id: str,
    box: Box,
    pose: Transform,
    *,
    operation: int = CollisionObject.ADD,
) -> CollisionObject:
    message = CollisionObject()
    message.header.frame_id = SCENE_FRAME
    message.id = object_id
    message.operation = operation
    message.pose = _pose_message(pose)
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(value) for value in box.size]
    message.primitives.append(primitive)
    identity = Pose()
    identity.orientation.w = 1.0
    message.primitive_poses.append(identity)
    return message


def attached_object_message(
    spec: AttachedSpec,
    *,
    link_name: str | None = None,
    operation: int = CollisionObject.ADD,
) -> AttachedCollisionObject:
    message = AttachedCollisionObject()
    message.link_name = link_name or spec.relative_pose.parent
    message.object = collision_object(
        spec.object_id, spec.box, spec.relative_pose, operation=operation,
    )
    # For attached objects the object pose is relative to the link.
    message.object.header.frame_id = (
        spec.frame_id if spec.frame_id is not None else message.link_name
    )
    message.touch_links = list(spec.touch_links)
    return message


def update_contact_entry(
    names: Sequence[str],
    values,
    first: str,
    second: str,
    allowed: bool,
) -> tuple[list[str], list[list[bool]]]:
    """Return a copy of an ACM with exactly one pair changed."""
    names = list(names)
    matrix = [list(row) for row in values]
    for name in (first, second):
        if name not in names:
            names.append(name)
            for row in matrix:
                row.append(allowed)
            matrix.append([allowed] * len(names))
    first_index = names.index(first)
    second_index = names.index(second)
    matrix[first_index][second_index] = allowed
    matrix[second_index][first_index] = allowed
    return names, matrix


def scene_ids(scene: PlanningScene) -> tuple[set[str], set[str]]:
    world = {obj.id for obj in scene.world.collision_objects}
    attached = {obj.object.id for obj in scene.robot_state.attached_collision_objects}
    return world, attached


class PlanningSceneBridge:
    """Thin ROS adapter around a ``moveit_msgs/PlanningScene`` diff stream."""

    def __init__(
        self,
        *,
        node: Node | None = None,
        use_sim_time: bool = True,
        srdf_path: str | None = None,
        spin_fn: Callable[[float], None] | None = None,
        on_timing: Callable[[dict], None] | None = None,
    ) -> None:
        self._owns_node = node is None
        self._on_timing = on_timing
        if node is None:
            node = Node('planning_scene_bridge')
            node.set_parameters([Parameter(
                'use_sim_time', value=use_sim_time,
            )])
        self.node = node
        self._spin_fn = spin_fn
        self.geometry: GeometrySpec | None = None
        self.spec: ScenarioSpec | None = None
        self.base_to_world: Transform | None = None
        self.last_scene: PlanningScene | None = None
        self._publisher = self.node.create_publisher(
            PlanningScene, SCENE_TOPIC, 10,
        )
        self._subscription = self.node.create_subscription(
            PlanningScene,
            MONITORED_TOPIC,
            self._on_scene,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self._get_scene = self.node.create_client(
            GetPlanningScene, GET_SCENE_SERVICE,
        )
        self._world_objects: dict[str, SceneObject] = {}
        self._initial_world: dict[str, SceneObject] = {}
        self._attached: dict[str, AttachedSpec] = {}
        self._seen_world: set[str] = set()
        self._seen_attached: set[str] = set()
        self._latest_joint_state = None
        self._last_full_scene: PlanningScene | None = None
        self._acm_names: list[str] = []
        self._acm_values: list[list[bool]] = []
        self._disabled_pairs = []
        srdf = srdf_path or str(
            Path(get_package_share_directory('iiwa_config'))
            / 'config' / 'moveit' / 'iiwa7.srdf'
        )
        try:
            self._disabled_pairs = load_srdf_disabled_pairs(srdf)
        except Exception:
            self._disabled_pairs = []
        self._joint_subscription = self.node.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10,
        )

    def _on_joint_state(self, message: JointState) -> None:
        self._latest_joint_state = message

    def _on_scene(self, message: PlanningScene) -> None:
        if not message.is_diff:
            self._last_full_scene = copy.deepcopy(message)
        self._seen_world.update(
            obj.id for obj in message.world.collision_objects
        )
        self._seen_attached.update(
            entry.object.id
            for entry in message.robot_state.attached_collision_objects
        )
        if not message.is_diff:
            self.last_scene = copy.deepcopy(message)
            return
        if self.last_scene is None:
            self.last_scene = PlanningScene()
            self.last_scene.is_diff = False
        scene = self.last_scene
        world = {obj.id: obj for obj in scene.world.collision_objects}
        for obj in message.world.collision_objects:
            if obj.operation == CollisionObject.REMOVE:
                world.pop(obj.id, None)
            else:
                world[obj.id] = copy.deepcopy(obj)
        scene.world.collision_objects = list(world.values())
        attached = {
            entry.object.id: entry
            for entry in scene.robot_state.attached_collision_objects
        }
        for entry in message.robot_state.attached_collision_objects:
            if entry.object.operation == CollisionObject.REMOVE:
                attached.pop(entry.object.id, None)
            else:
                attached[entry.object.id] = copy.deepcopy(entry)
        scene.robot_state.attached_collision_objects = list(attached.values())

    def _spin_once(self, timeout_sec: float) -> None:
        """Let the owning executor spin, or spin locally in the reference run."""
        if self._spin_fn is not None:
            self._spin_fn(timeout_sec)
            return
        rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def _spin(self, seconds: float = 0.05) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._spin_once(0.01)

    def _publish(self, scene: PlanningScene) -> None:
        self._seen_world.clear()
        self._seen_attached.clear()
        self._publisher.publish(scene)
        # A short non-blocking turn lets rclpy deliver the message to local
        # subscribers.  The authoritative synchronization check happens in
        # assert_synchronized() through /get_planning_scene.
        self._spin_once(0.0)

    def _fetch_acm(self) -> None:
        if getattr(self, '_get_scene', None) is None or self.node is None:
            return
        if not self._get_scene.service_is_ready():
            if not self._get_scene.wait_for_service(timeout_sec=1.0):
                return
        request = GetPlanningScene.Request()
        request.components.components = 128
        future = self._get_scene.call_async(request)
        end = time.monotonic() + 3.0
        while time.monotonic() < end and not future.done():
            self._spin_once(0.05)
        if future.done():
            try:
                matrix = future.result().scene.allowed_collision_matrix
                self._acm_names = list(matrix.entry_names)
                self._acm_values = [
                    list(row.enabled) for row in matrix.entry_values
                ]
            except Exception:
                pass

    def _rebuild_acm(self) -> None:
        extra = []
        for spec in self._attached.values():
            extra.extend((spec.object_id, link) for link in spec.touch_links)
        self._acm_names, self._acm_values = build_collision_matrix(
            list(self._world_objects) + list(self._attached),
            disabled_pairs=self._disabled_pairs,
            extra_allowed=extra,
        )

    def _full_scene(self) -> PlanningScene:
        scene = PlanningScene()
        scene.is_diff = False
        # Never overwrite the planner's current robot state from a planning
        # scene update: the monitor receives /joint_states directly and a
        # stale embedded sample makes the next trajectory start from an old
        # pose.  ``is_diff`` leaves the state untouched.
        scene.robot_state.is_diff = True
        for obj in self._world_objects.values():
            scene.world.collision_objects.append(collision_object(
                obj.object_id, obj.box, self._base_pose(obj.pose),
            ))
        for spec in self._attached.values():
            scene.robot_state.attached_collision_objects.append(
                attached_object_message(spec)
            )
        if self._acm_names:
            scene.allowed_collision_matrix.entry_names = list(self._acm_names)
            scene.allowed_collision_matrix.entry_values = [
                AllowedCollisionEntry(
                    enabled=[bool(value) for value in row],
                )
                for row in self._acm_values
            ]
        return scene

    def _refresh_joint_state(self) -> None:
        """Make sure a measured joint sample exists before a full scene update.

        The full scene is published with ``robot_state.is_diff=True`` so it
        never overwrites MoveIt's current state.  Waiting for the *next*
        sample on every scene update added a fixed delay between motions
        without improving that guarantee; the executor already checks joint
        state freshness immediately before every plan.
        """
        if self.node is None or self._latest_joint_state is not None:
            return
        end = time.monotonic() + 0.1
        while time.monotonic() < end:
            self._spin_once(0.01)
            if self._latest_joint_state is not None:
                return

    def _relay_full_scene(self) -> None:
        """Give MoveItPy the exact move_group scene, including its ACM.

        MoveItPy's monitor always listens on ``/planning_scene`` even when its
        configured topics differ.  Fetching the maintained scene from
        move_group and republishing it once keeps both planners identical
        without a feedback loop.
        """
        started = time.monotonic()
        relayed = False
        try:
            relayed = self._relay_full_scene_impl()
        finally:
            on_timing = getattr(self, '_on_timing', None)
            if on_timing is not None:
                try:
                    on_timing({
                        'stage': 'planning_scene_relay',
                        'started_at': started,
                        'finished_at': time.monotonic(),
                        'duration_sec': time.monotonic() - started,
                        'relayed': relayed,
                    })
                except Exception:
                    pass

    def _relay_full_scene_impl(self) -> bool:
        if getattr(self, '_get_scene', None) is None or self.node is None:
            return False
        if not self._get_scene.service_is_ready():
            if not self._get_scene.wait_for_service(timeout_sec=1.0):
                return False
        expected_world = set(self._world_objects)
        expected_attached = set(self._attached)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            request = GetPlanningScene.Request()
            request.components.components = 2 | 4 | 8 | 16 | 128
            future = self._get_scene.call_async(request)
            end = time.monotonic() + 1.0
            while time.monotonic() < end and not future.done():
                self._spin_once(0.05)
            if not future.done():
                continue
            try:
                scene = future.result().scene
            except Exception:
                continue
            world, attached = scene_ids(scene)
            if expected_world <= world and expected_attached <= attached:
                # Drop stale extras from move_group's maintained scene.  A
                # removed tool must not be reintroduced by the relay.
                scene.world.collision_objects = [
                    obj for obj in scene.world.collision_objects
                    if obj.id in expected_world
                ]
                scene.robot_state.attached_collision_objects = [
                    entry for entry in scene.robot_state.attached_collision_objects
                    if entry.object.id in expected_attached
                ]
                # Keep the SRDF-derived ACM.  move_group can report an
                # empty matrix early in startup; relaying that would make
                # MoveItPy treat adjacent robot links as collisions.
                if self._acm_names:
                    scene.allowed_collision_matrix.entry_names = list(self._acm_names)
                    scene.allowed_collision_matrix.entry_values = [
                        AllowedCollisionEntry(
                            enabled=[bool(value) for value in row],
                        )
                        for row in self._acm_values
                    ]
                scene.is_diff = False
                scene.robot_state.is_diff = True
                self._publisher.publish(scene)
                self._spin()
                return True
        # Never relay a stale scene: that would wipe the objects from the
        # MoveItPy monitor and make the next motion unverified.
        return False

    def _publish_full(self) -> None:
        self._refresh_joint_state()
        self._publish(self._full_scene())
        self._relay_full_scene()

    @staticmethod
    def _diff() -> PlanningScene:
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        return scene

    def update_world_to_base(self, world_to_base: Transform) -> None:
        self.base_to_world = world_to_base.inverse()

    def _base_pose(self, pose: Transform) -> Transform:
        if self.base_to_world is None:
            raise SceneSynchronizationError('base transform is not initialised')
        return self.base_to_world.compose_relaxed(pose)

    def initialize(
        self,
        spec: ScenarioSpec,
        geometry: GeometrySpec,
        *,
        world_to_base: Transform | None = None,
    ) -> None:
        self.spec = spec
        self.geometry = geometry
        self.base_to_world = (world_to_base or geometry.world_to_base).inverse()
        self._world_objects = {
            obj.object_id: obj for obj in build_world_objects(spec, geometry)
        }
        self._initial_world = dict(self._world_objects)
        self._attached.clear()
        extra = []
        for spec in self._attached.values():
            extra.extend((spec.object_id, link) for link in spec.touch_links)
        self._acm_names, self._acm_values = build_collision_matrix(
            list(self._world_objects) + list(self._attached),
            disabled_pairs=self._disabled_pairs,
            extra_allowed=extra,
        )
        self._publish_full()
        self.assert_synchronized(
            self._world_objects, (), seen_world=set(self._world_objects),
        )

    def _publish_attached_removal(self, object_id: str) -> None:
        """Send an explicit attached-object removal diff.

        Full scenes carry the attached list inside ``robot_state`` with
        ``is_diff=True``; a consumer that applies that list as a diff never
        treats an omission as a removal and keeps a ghost attachment.  The
        explicit REMOVE entry is unambiguous for every consumer.
        """
        publisher = getattr(self, '_publisher', None)
        if publisher is None or self.node is None:
            return
        scene = self._diff()
        removal = AttachedCollisionObject()
        removal.link_name = 'patron'
        removal.object.id = object_id
        removal.object.operation = CollisionObject.REMOVE
        scene.robot_state.attached_collision_objects.append(removal)
        publisher.publish(scene)
        self._spin()

    def attach(self, spec: AttachedSpec) -> None:
        self._world_objects.pop(spec.object_id, None)
        self._attached[spec.object_id] = spec
        self._rebuild_acm()
        self._publish_full()

    def detach(self, object_id: str, world_pose: Transform) -> None:
        if object_id not in self._attached:
            raise SceneSynchronizationError(
                f'cannot detach {object_id!r}: it is not attached'
            )
        spec = self._attached.pop(object_id)
        self._publish_attached_removal(object_id)
        object_spec = SceneObject(object_id, spec.box, world_pose)
        self._world_objects[object_id] = object_spec
        self._rebuild_acm()
        self._publish_full()

    def remove(self, object_id: str) -> None:
        if self._attached.pop(object_id, None) is not None:
            self._publish_attached_removal(object_id)
        self._world_objects.pop(object_id, None)
        self._rebuild_acm()
        self._publish_full()

    def add_world(self, object_id: str, box: Box, world_pose: Transform) -> None:
        if self._attached.pop(object_id, None) is not None:
            self._publish_attached_removal(object_id)
        self._world_objects[object_id] = SceneObject(object_id, box, world_pose)
        self._rebuild_acm()
        self._publish_full()

    def restore_world(self, object_id: str) -> None:
        """Return an object to the exact box pose it had at scene initialisation.

        Use this when an object physically returns to its calibrated support
        pose (a tool docked in its socket, a part restored after a simulated
        grasp failure): the initial pose already carries the correct
        box-centre lift and flat orientation.
        """
        original = self._initial_world.get(object_id)
        if original is None:
            raise SceneSynchronizationError(
                f'cannot restore {object_id!r}: it was not in the initial scene'
            )
        if self._attached.pop(object_id, None) is not None:
            self._publish_attached_removal(object_id)
        self._world_objects[object_id] = original
        self._rebuild_acm()
        self._publish_full()

    def allow_contact(self, first: str, second: str, allowed: bool = True) -> None:
        """Change exactly one ACM pair; all other checks stay active."""
        if self.last_scene is None or not self.last_scene.allowed_collision_matrix.entry_names:
            raise SceneSynchronizationError(
                'cannot update contact permissions without a monitored scene'
            )
        if not self._acm_names:
            raise SceneSynchronizationError(
                'cannot update contact permissions without an ACM'
            )
        self._acm_names, self._acm_values = update_contact_entry(
            self._acm_names, self._acm_values, first, second, allowed,
        )
        self._publish_full()

    def expected_sets(self) -> tuple[set[str], set[str]]:
        return set(self._world_objects), set(self._attached)

    def _query_scene(self, timeout_sec: float) -> tuple[set[str], set[str]] | None:
        if getattr(self, '_get_scene', None) is None or self.node is None:
            return
        if not self._get_scene.service_is_ready():
            if not self._get_scene.wait_for_service(timeout_sec=1.0):
                return None
        request = GetPlanningScene.Request()
        request.components.components = 8 | 16 | 4 | 128
        future = self._get_scene.call_async(request)
        end = time.monotonic() + timeout_sec
        while time.monotonic() < end and not future.done():
            self._spin_once(0.05)
        if not future.done():
            return None
        try:
            response = future.result()
        except Exception:
            return None
        return scene_ids(response.scene)

    def assert_synchronized(
        self,
        expected_world: Iterable[str] | None = None,
        expected_attached: Iterable[str] | None = None,
        *,
        seen_world: Iterable[str] | None = None,
        seen_attached: Iterable[str] | None = None,
        timeout_sec: float = 8.0,
    ) -> None:
        world_expected = set(expected_world) if expected_world is not None else set()
        attached_expected = (
            set(expected_attached) if expected_attached is not None else set()
        )
        world_seen_expected = set(seen_world) if seen_world is not None else set()
        attached_seen_expected = (
            set(seen_attached)
            if seen_attached is not None
            else set(attached_expected)
        )
        end = time.monotonic() + timeout_sec
        last_world: set[str] = set()
        last_attached: set[str] = set()
        while time.monotonic() < end:
            self._spin_once(0.05)
            # 1. Every expected id must have appeared in a monitored scene
            #    (published by the MoveItPy or move_group monitor).
            served = (
                world_seen_expected <= self._seen_world
                and attached_seen_expected <= self._seen_attached
            )
            # 2. The move_group service is the authoritative full snapshot.
            snapshot = self._query_scene(timeout_sec=1.0)
            if snapshot is not None:
                last_world, last_attached = snapshot
                if world_expected <= last_world and attached_expected <= last_attached:
                    if not served:
                        # move_group already has the scene, but MoveItPy's
                        # monitor also needs to have seen the same ids before
                        # the next plan is allowed.
                        continue
                    return
            elif served:
                if self.last_scene is not None:
                    last_world, last_attached = scene_ids(self.last_scene)
                return
        raise SceneSynchronizationError(
            'MoveIt scene is not synchronized: '
            f'missing world={sorted(world_expected - last_world)}, '
            f'missing attached={sorted(attached_expected - last_attached)}; '
            f'seen world={sorted(self._seen_world)}, '
            f'seen attached={sorted(self._seen_attached)}'
        )

    def destroy(self) -> None:
        if self._owns_node:
            self.node.destroy_node()
