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
from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AttachedCollisionObject,
    CollisionObject,
    PlanningScene,
)
from moveit_msgs.srv import GetPlanningScene
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive

from .geometry import Box, GeometrySpec, Transform, object_pose_transform
from .scenario import ScenarioSpec


SCENE_TOPIC = '/planning_scene'
MONITORED_TOPIC = '/monitored_planning_scene'
GET_SCENE_SERVICE = '/get_planning_scene'
SCENE_FRAME = 'base_link'
TOUCH_LINKS = ('tcp', 'patron', 'camera_holder')


class SceneSynchronizationError(RuntimeError):
    """The MoveIt scene does not match the requested object set."""


def screw_object_id(screw_id: str) -> str:
    return f'grant_screw_{int(screw_id.split("_")[1]):02d}'


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

    fixture = geometry.scene_collisions['fixture']
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
                    screw.pose.z + 0.0045,
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
                tool.pose.x, tool.pose.y, tool.pose.z + 0.070,
                tool.pose.yaw_deg + 180.0,
            ),
        ))

    if spec.cap_state != 'absent':
        cap_pose = spec.cap_pose
        assert cap_pose is not None
        objects.append(SceneObject(
            'grant_cap',
            geometry.cap_collision,
            _rotated_pose(
                cap_pose.x, cap_pose.y, cap_pose.z + 0.004165,
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
            collision.offset, [0.0, 0.0, 0.0], parent='tcp', child=f'grant_{name}',
        ),
        touch_links=TOUCH_LINKS,
    )


def attached_cap(geometry: GeometrySpec) -> AttachedSpec:
    return AttachedSpec(
        object_id='grant_cap',
        box=geometry.cap_collision,
        relative_pose=Transform.from_xyz_rpy(
            geometry.cap_collision.offset,
            [0.0, 0.0, 0.0],
            parent='tcp',
            child='grant_cap',
        ),
        touch_links=TOUCH_LINKS,
    )


def attached_screw(screw_id: str, geometry: GeometrySpec) -> AttachedSpec:
    object_id = screw_object_id(screw_id)
    return AttachedSpec(
        object_id=object_id,
        box=geometry.screw_collision,
        relative_pose=Transform.from_xyz_rpy(
            geometry.screw_collision.offset,
            [0.0, 0.0, 0.0],
            parent='tcp',
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
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [float(value) for value in box.size]
    message.primitives.append(primitive)
    message.primitive_poses.append(_pose_message(pose))
    return message


def attached_object_message(
    spec: AttachedSpec,
    *,
    link_name: str = 'tcp',
    operation: int = CollisionObject.ADD,
) -> AttachedCollisionObject:
    message = AttachedCollisionObject()
    message.link_name = link_name
    message.object = collision_object(
        spec.object_id, spec.box, spec.relative_pose, operation=operation,
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
    ) -> None:
        self._owns_node = node is None
        if node is None:
            node = Node('planning_scene_bridge')
            node.set_parameters([Parameter(
                'use_sim_time', value=use_sim_time,
            )])
        self.node = node
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
        self._attached: dict[str, AttachedSpec] = {}
        self._seen_world: set[str] = set()
        self._seen_attached: set[str] = set()

    def _on_scene(self, message: PlanningScene) -> None:
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

    def _spin(self, seconds: float = 0.05) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.01)

    def _publish(self, scene: PlanningScene) -> None:
        self._seen_world.clear()
        self._seen_attached.clear()
        self._publisher.publish(scene)
        self._spin()

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
        self._attached.clear()
        scene = self._diff()
        for obj in self._world_objects.values():
            scene.world.collision_objects.append(collision_object(
                obj.object_id, obj.box, self._base_pose(obj.pose),
            ))
        self._publish(scene)
        self.assert_synchronized(
            self._world_objects, (), seen_world=set(self._world_objects),
        )

    def attach(self, spec: AttachedSpec) -> None:
        if spec.object_id not in self._world_objects:
            raise SceneSynchronizationError(
                f'cannot attach {spec.object_id!r}: not a known world object'
            )
        scene = self._diff()
        scene.world.collision_objects.append(collision_object(
            spec.object_id, spec.box, Transform.from_xyz_rpy(
                [0, 0, 0], [0, 0, 0], parent='world', child='removed',
            ), operation=CollisionObject.REMOVE,
        ))
        scene.robot_state.attached_collision_objects.append(
            attached_object_message(spec)
        )
        self._world_objects.pop(spec.object_id, None)
        self._attached[spec.object_id] = spec
        self._publish(scene)

    def detach(self, object_id: str, world_pose: Transform) -> None:
        if object_id not in self._attached:
            raise SceneSynchronizationError(
                f'cannot detach {object_id!r}: it is not attached'
            )
        spec = self._attached.pop(object_id)
        object_spec = SceneObject(object_id, spec.box, world_pose)
        self._world_objects[object_id] = object_spec
        scene = self._diff()
        scene.robot_state.attached_collision_objects.append(
            attached_object_message(
                spec, operation=CollisionObject.REMOVE,
            )
        )
        scene.world.collision_objects.append(collision_object(
            object_id, spec.box, self._base_pose(world_pose),
        ))
        self._publish(scene)

    def remove(self, object_id: str) -> None:
        scene = self._diff()
        if object_id in self._attached:
            spec = self._attached.pop(object_id)
            scene.robot_state.attached_collision_objects.append(
                attached_object_message(
                    spec, operation=CollisionObject.REMOVE,
                )
            )
        if object_id in self._world_objects:
            self._world_objects.pop(object_id)
            scene.world.collision_objects.append(collision_object(
                object_id, Box(size=(0.001, 0.001, 0.001)), Transform.from_xyz_rpy(
                    [0, 0, 0], [0, 0, 0], parent='world', child='removed',
                ), operation=CollisionObject.REMOVE,
            ))
        self._publish(scene)

    def allow_contact(self, first: str, second: str, allowed: bool = True) -> None:
        """Change exactly one ACM pair; all other checks stay active."""
        if self.last_scene is None or not self.last_scene.allowed_collision_matrix.entry_names:
            raise SceneSynchronizationError(
                'cannot update contact permissions without a monitored scene'
            )
        scene = self._diff()
        names = list(self.last_scene.allowed_collision_matrix.entry_names)
        values = [
            list(row) for row in self.last_scene.allowed_collision_matrix.entry_values
        ]
        names, values = update_contact_entry(
            names, values, first, second, allowed,
        )
        scene.allowed_collision_matrix.entry_names = names
        scene.allowed_collision_matrix.entry_values = values
        self._publish(scene)

    def expected_sets(self) -> tuple[set[str], set[str]]:
        return set(self._world_objects), set(self._attached)

    def _query_scene(self, timeout_sec: float) -> tuple[set[str], set[str]] | None:
        if not self._get_scene.service_is_ready():
            if not self._get_scene.wait_for_service(timeout_sec=1.0):
                return None
        request = GetPlanningScene.Request()
        request.components.components = 8 | 16 | 4 | 128
        future = self._get_scene.call_async(request)
        end = time.monotonic() + timeout_sec
        while time.monotonic() < end and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.05)
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
            rclpy.spin_once(self.node, timeout_sec=0.05)
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
                        # The service proves move_group; still let the next
                        # monitored message confirm the second monitor.
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
