"""Calibrated transforms, working tips and scene collision models.

This module is pure Python (numpy/scipy only) so the same numbers are used by
the unit tests, the MoveIt bridge, the motion adapter and the Webots
supervisor.  Nothing here talks to ROS or Webots.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from .scenario import SCREW_HOLES, SEALANT_SIDES, ScenarioSpec, Tool


TOOL_GEOMETRY_KEYS = ('vacuum', 'magnetic_tool', 'sealant_tool', 'torque_tool')
_SEALANT_SIDES = SEALANT_SIDES


class GeometryError(ValueError):
    """Raised when the geometry file is missing or inconsistent."""


def _number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise GeometryError(f'{label} must be a finite number')
    result = float(value)
    if not np.isfinite(result):
        raise GeometryError(f'{label} must be a finite number')
    return result


def _vector(value: Any, label: str, length: int) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise GeometryError(f'{label} must be a list of {length} numbers')
    return tuple(_number(item, f'{label}[{index}]') for index, item in enumerate(value))


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise GeometryError(f'{label} must be a mapping')
    return value


@dataclass(frozen=True)
class Box:
    """Axis-aligned box in a named frame."""

    size: tuple[float, float, float]
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def half_extents(self) -> tuple[float, float, float]:
        return tuple(value / 2.0 for value in self.size)


@dataclass(frozen=True)
class Transform:
    """Rigid transform ``parent -> child``.

    ``rotation`` is a 3x3 row-major matrix.  Composition follows the usual
    convention: ``parent_to_child.compose(child_to_grandchild)``.
    """

    xyz: tuple[float, float, float]
    rotation: tuple[tuple[float, float, float], ...]
    parent: str
    child: str

    @classmethod
    def from_xyz_rpy(
        cls,
        xyz,
        rpy_deg,
        *,
        parent: str,
        child: str,
    ) -> 'Transform':
        values = _vector(xyz, f'{parent}->{child}.xyz', 3)
        angles = _vector(rpy_deg, f'{parent}->{child}.rpy_deg', 3)
        matrix = Rotation.from_euler('xyz', angles, degrees=True).as_matrix()
        return cls(
            xyz=values,
            rotation=tuple(tuple(float(v) for v in row) for row in matrix),
            parent=parent,
            child=child,
        )

    @property
    def matrix(self) -> np.ndarray:
        return np.asarray(self.rotation, dtype=float)

    @property
    def position(self) -> np.ndarray:
        return np.asarray(self.xyz, dtype=float)

    def compose_relaxed(self, other: 'Transform') -> 'Transform':
        """Compose two transforms whose frame labels are documentation only."""
        rotation = self.matrix @ other.matrix
        translation = self.position + self.matrix @ other.position
        return Transform(
            xyz=tuple(float(v) for v in translation),
            rotation=tuple(tuple(float(v) for v in row) for row in rotation),
            parent=self.parent,
            child=other.child,
        )

    def compose(self, other: 'Transform') -> 'Transform':
        if self.child != other.parent:
            raise GeometryError(
                f'cannot compose {self.parent}->{self.child} with '
                f'{other.parent}->{other.child}'
            )
        rotation = self.matrix @ other.matrix
        translation = self.position + self.matrix @ other.position
        return Transform(
            xyz=tuple(float(v) for v in translation),
            rotation=tuple(tuple(float(v) for v in row) for row in rotation),
            parent=self.parent,
            child=other.child,
        )

    def inverse(self) -> 'Transform':
        rotation = self.matrix.T
        translation = -rotation @ self.position
        return Transform(
            xyz=tuple(float(v) for v in translation),
            rotation=tuple(tuple(float(v) for v in row) for row in rotation),
            parent=self.child,
            child=self.parent,
        )

    def apply(self, point) -> tuple[float, float, float]:
        result = self.matrix @ np.asarray(point, dtype=float) + self.position
        return tuple(float(v) for v in result)

    def distance_to(self, other: 'Transform') -> float:
        if self.parent != other.parent:
            raise GeometryError('cannot compare transforms in different parents')
        return float(np.linalg.norm(self.position - other.position))

    def angle_to(self, other: 'Transform', *, parent: str | None = None) -> float:
        matrix = self.matrix
        other_matrix = other.matrix
        if parent is not None:
            matrix = _frame_matrix(self, parent) @ matrix
            other_matrix = _frame_matrix(other, parent) @ other_matrix
        cosine = (np.trace(matrix.T @ other_matrix) - 1.0) / 2.0
        return float(np.arccos(max(-1.0, min(1.0, cosine))))


def _frame_matrix(transform: Transform, frame: str) -> np.ndarray:
    # Kept small and explicit: only world/base_link are ever needed here.
    del transform
    if frame in ('base_link', 'world'):
        return np.eye(3)
    raise GeometryError(f'unsupported comparison frame {frame!r}')


@dataclass(frozen=True)
class HoleSpec:
    name: str
    dx: float
    dy: float


@dataclass(frozen=True)
class SegmentSpec:
    name: str
    start: tuple[float, float]
    end: tuple[float, float]


@dataclass(frozen=True)
class GeometrySpec:
    world_to_base: Transform
    patron_to_tcp: Transform
    tool_mounts: dict[str, Transform]
    tool_work_tips: dict[str, Transform]
    tool_tcp_to_nodes: dict[str, Transform]
    tool_collisions: dict[str, Box]
    cap_tcp_to_node: Transform
    cap_collision: Box
    body_to_installed_cap: Transform
    screw_tcp_to_node: Transform
    screw_collision: Box
    holes: dict[str, HoleSpec]
    hole_reference_yaw_deg: float
    hole_z_above_body_origin: float
    handoff_gap: float
    sealant_reference_yaw_deg: float
    sealant_sides: dict[str, SegmentSpec]
    sealant_z_above_body_top: float
    approach_gap: float
    contact_gap: float
    transfer_gap: float
    seat_gap: float
    position_tolerance: float
    orientation_tolerance_rad: float
    speeds: dict[str, float]
    scene_collisions: dict[str, Box]
    safe_poses: dict[str, Transform]
    named_joints: dict[str, tuple[float, ...]]

    @property
    def body_top_local(self) -> float:
        body = self.scene_collisions['body']
        return body.offset[2] + body.size[2] / 2.0


def _load_transform(raw: Any, label: str, parent: str, child: str) -> Transform:
    data = _mapping(raw, label)
    return Transform.from_xyz_rpy(
        data.get('xyz', [0.0, 0.0, 0.0]),
        data.get('rpy_deg', [0.0, 0.0, 0.0]),
        parent=parent,
        child=child,
    )


def _load_box(raw: Any, label: str) -> Box:
    data = _mapping(raw, label)
    size = _vector(data.get('size'), f'{label}.size', 3)
    offset = _vector(data.get('offset', [0.0, 0.0, 0.0]), f'{label}.offset', 3)
    if any(value <= 0.0 for value in size):
        raise GeometryError(f'{label}.size must be positive')
    return Box(size=size, offset=offset)


def load_geometry(path: Path | str) -> GeometrySpec:
    """Load and validate ``config/geometry.yaml``."""
    source = Path(path)
    if not source.is_file():
        raise FileNotFoundError(f'geometry file does not exist: {source}')
    try:
        raw = yaml.safe_load(source.read_text(encoding='utf-8'))
    except yaml.YAMLError as error:
        raise GeometryError(f'invalid YAML in {source}: {error}') from error
    data = _mapping(raw, 'geometry')
    if data.get('schema_version') != 1:
        raise GeometryError('geometry schema_version must be 1')

    world_to_base = _load_transform(
        data.get('world_to_base'), 'world_to_base', 'world', 'base_link',
    )
    patron_to_tcp = _load_transform(
        data.get('patron_to_tcp'), 'patron_to_tcp', 'patron', 'tcp',
    )

    tool_mounts_raw = _mapping(data.get('tool_mounts'), 'tool_mounts')
    tool_tips_raw = _mapping(data.get('tool_work_tips'), 'tool_work_tips')
    tool_nodes_raw = _mapping(data.get('tool_tcp_to_nodes'), 'tool_tcp_to_nodes')
    tool_collisions_raw = _mapping(data.get('tool_collisions'), 'tool_collisions')
    missing = [name for name in TOOL_GEOMETRY_KEYS if name not in tool_mounts_raw]
    if missing:
        raise GeometryError(f'tool_mounts is missing {missing}')
    tool_mounts = {
        name: _load_transform(
            tool_mounts_raw[name], f'tool_mounts.{name}', 'tcp', f'{name}_mount',
        )
        for name in TOOL_GEOMETRY_KEYS
    }
    tool_work_tips = {
        name: _load_transform(
            tool_tips_raw[name], f'tool_work_tips.{name}', 'tcp', f'{name}_tip',
        )
        for name in TOOL_GEOMETRY_KEYS
    }
    tool_tcp_to_nodes = {
        name: _load_transform(
            tool_nodes_raw[name], f'tool_tcp_to_nodes.{name}', 'tcp', f'tool_{name}',
        )
        for name in TOOL_GEOMETRY_KEYS
    }
    tool_collisions = {
        name: _load_box(tool_collisions_raw[name], f'tool_collisions.{name}')
        for name in TOOL_GEOMETRY_KEYS
    }

    holes_raw = _mapping(data.get('holes'), 'holes')
    holes = {}
    for name in SCREW_HOLES:
        if name not in holes_raw:
            raise GeometryError(f'holes is missing {name}')
        entry = _mapping(holes_raw[name], f'holes.{name}')
        holes[name] = HoleSpec(
            name=name,
            dx=_number(entry.get('dx'), f'holes.{name}.dx'),
            dy=_number(entry.get('dy'), f'holes.{name}.dy'),
        )

    sides_raw = _mapping(data.get('sealant_sides'), 'sealant_sides')
    sealant_sides = {}
    for name in _SEALANT_SIDES:
        entry = _mapping(sides_raw.get(name), f'sealant_sides.{name}')
        sealant_sides[name] = SegmentSpec(
            name=name,
            start=_vector(entry.get('start'), f'sealant_sides.{name}.start', 2),
            end=_vector(entry.get('end'), f'sealant_sides.{name}.end', 2),
        )

    speeds_raw = _mapping(data.get('speeds'), 'speeds')
    speeds = {
        key: _number(speeds_raw.get(key), f'speeds.{key}')
        for key in ('transfer', 'approach', 'contact', 'contour')
    }
    for key, value in speeds.items():
        if not 0.0 < value <= 1.0:
            raise GeometryError(f'speeds.{key} must be within (0, 1]')

    joints_raw = _mapping(data.get('named_joints', {}), 'named_joints')
    named_joints = {
        name: _vector(values, f'named_joints.{name}', 7)
        for name, values in joints_raw.items()
    }

    safe_raw = _mapping(data.get('safe_poses', {}), 'safe_poses')
    safe_poses = {
        name: _load_transform(
            entry, f'safe_poses.{name}', 'world', f'safe_{name}',
        )
        for name, entry in safe_raw.items()
    }

    scene_raw = _mapping(data.get('scene_collisions'), 'scene_collisions')
    scene_collisions = {}
    for name, entry in scene_raw.items():
        box_data = _mapping(entry, f'scene_collisions.{name}')
        size = _vector(box_data.get('size'), f'scene_collisions.{name}.size', 3)
        offset = (
            _number(box_data.get('offset_x', 0.0), f'scene_collisions.{name}.offset_x'),
            _number(box_data.get('offset_y', 0.0), f'scene_collisions.{name}.offset_y'),
            _number(box_data.get('offset_z', 0.0), f'scene_collisions.{name}.offset_z'),
        )
        scene_collisions[name] = Box(size=size, offset=offset)

    positive = {
        'approach_gap': data.get('approach_gap'),
        'contact_gap': data.get('contact_gap'),
        'transfer_gap': data.get('transfer_gap'),
        'seat_gap': data.get('seat_gap'),
        'position_tolerance': data.get('position_tolerance'),
    }
    values = {
        key: _number(value, key) for key, value in positive.items()
    }
    for key, value in values.items():
        if value <= 0.0:
            raise GeometryError(f'{key} must be positive')
    orientation_tolerance = _number(
        data.get('orientation_tolerance_rad'), 'orientation_tolerance_rad'
    )
    if orientation_tolerance <= 0.0:
        raise GeometryError('orientation_tolerance_rad must be positive')

    cap_tcp_to_node = _load_transform(
        data.get('cap_tcp_to_node'), 'cap_tcp_to_node', 'tcp', 'cap_node',
    )
    body_to_cap = _load_transform(
        data.get('body_to_installed_cap'), 'body_to_installed_cap', 'body', 'cap_seat',
    )
    screw_tcp_to_node = _load_transform(
        data.get('screw_tcp_to_node'), 'screw_tcp_to_node', 'tcp', 'screw_node',
    )

    return GeometrySpec(
        world_to_base=world_to_base,
        patron_to_tcp=patron_to_tcp,
        tool_mounts=tool_mounts,
        tool_work_tips=tool_work_tips,
        tool_tcp_to_nodes=tool_tcp_to_nodes,
        tool_collisions=tool_collisions,
        cap_tcp_to_node=cap_tcp_to_node,
        cap_collision=_load_box(data.get('cap_collision'), 'cap_collision'),
        body_to_installed_cap=body_to_cap,
        screw_tcp_to_node=screw_tcp_to_node,
        screw_collision=_load_box(data.get('screw_collision'), 'screw_collision'),
        holes=holes,
        hole_reference_yaw_deg=_number(
            data.get('hole_reference_yaw_deg'), 'hole_reference_yaw_deg'
        ),
        hole_z_above_body_origin=_number(
            data.get('hole_z_above_body_origin'), 'hole_z_above_body_origin'
        ),
        handoff_gap=_number(data.get('handoff_gap'), 'handoff_gap'),
        sealant_reference_yaw_deg=_number(
            data.get('sealant_reference_yaw_deg'), 'sealant_reference_yaw_deg'
        ),
        sealant_sides=sealant_sides,
        sealant_z_above_body_top=_number(
            data.get('sealant_z_above_body_top'), 'sealant_z_above_body_top'
        ),
        approach_gap=values['approach_gap'],
        contact_gap=values['contact_gap'],
        transfer_gap=values['transfer_gap'],
        seat_gap=values['seat_gap'],
        position_tolerance=values['position_tolerance'],
        orientation_tolerance_rad=orientation_tolerance,
        speeds=speeds,
        scene_collisions=scene_collisions,
        safe_poses=safe_poses,
        named_joints=named_joints,
    )


def object_pose_transform(
    x: float,
    y: float,
    z: float,
    yaw_deg: float,
    *,
    parent: str = 'world',
    child: str,
) -> Transform:
    """Webots pose of an OBJ whose local Y axis becomes world Z."""
    return Transform.from_xyz_rpy(
        [x, y, z], [90.0, 0.0, yaw_deg], parent=parent, child=child,
    )


def _tool_transform(tool: Tool, parent: str = 'world') -> Transform:
    return object_pose_transform(
        tool.pose.x,
        tool.pose.y,
        tool.pose.z,
        tool.pose.yaw_deg + 180.0,
        parent=parent,
        child=f'tool_{tool.name}',
    )


def _body_transform(spec: ScenarioSpec) -> Transform:
    return object_pose_transform(
        spec.body.x, spec.body.y, spec.body.z, spec.body.yaw_deg,
        parent='world', child='body',
    )


def _down_transform(x: float, y: float, z: float, yaw_deg: float, child: str) -> Transform:
    """Work-point pose whose local Z points down (tool working direction)."""
    return Transform.from_xyz_rpy(
        [x, y, z], [180.0, 0.0, yaw_deg], parent='world', child=child,
    )


def _rotate_xy(dx: float, dy: float, degrees: float) -> tuple[float, float]:
    angle = np.radians(degrees)
    cosine, sine = np.cos(angle), np.sin(angle)
    return dx * cosine - dy * sine, dx * sine + dy * cosine


def world_target(spec: ScenarioSpec, geometry: GeometrySpec, name: str) -> Transform:
    """Return the named world pose from an already validated scenario.

    Names:
      ``base_link``, ``tool:<name>``, ``cap``, ``cap_seat``,
      ``screw:<screw_i>``, ``hole:<hole_i>`` and ``sealant:<side_i>``.
    """
    if name == 'base_link':
        return geometry.world_to_base
    if name.startswith('tool:'):
        tool_name = name.split(':', 1)[1]
        tools = {tool.name: tool for tool in spec.tools}
        if tool_name not in tools:
            raise GeometryError(f'experiment {spec.experiment_id} has no tool {tool_name!r}')
        return _tool_transform(tools[tool_name])
    if name == 'cap':
        if spec.cap_pose is None:
            raise GeometryError(f'experiment {spec.experiment_id} has no hatch')
        pose = spec.cap_pose
        return object_pose_transform(
            pose.x, pose.y, pose.z, pose.yaw_deg, child='cap',
        )
    if name == 'cap_seat':
        return _body_transform(spec).compose(geometry.body_to_installed_cap)
    if name.startswith('screw:'):
        screw_name = name.split(':', 1)[1]
        screws = {screw.name: screw for screw in spec.screws}
        if screw_name not in screws:
            raise GeometryError(
                f'experiment {spec.experiment_id} has no screw {screw_name!r}'
            )
        screw = screws[screw_name]
        return object_pose_transform(
            screw.pose.x, screw.pose.y, screw.pose.z,
            screw.pose.yaw_deg, child=screw.name,
        )
    if name.startswith('hole:'):
        hole_name = name.split(':', 1)[1]
        if hole_name not in geometry.holes:
            raise GeometryError(f'unknown hole {hole_name!r}')
        hole = geometry.holes[hole_name]
        dx, dy = _rotate_xy(
            hole.dx, hole.dy, spec.body.yaw_deg - geometry.hole_reference_yaw_deg,
        )
        # The target is the screw node at the handoff point: upright, its
        # lower point just above the hole.  tcp_goal() turns it into a
        # downward-facing tcp with the magnet above the screw head.
        return object_pose_transform(
            spec.body.x + dx,
            spec.body.y + dy,
            spec.body.z + geometry.hole_z_above_body_origin + geometry.handoff_gap,
            spec.body.yaw_deg,
            child=hole_name,
        )
    if name.startswith('safe:'):
        pose_name = name.split(':', 1)[1]
        if pose_name not in geometry.safe_poses:
            raise GeometryError(f'unknown safe pose {pose_name!r}')
        return geometry.safe_poses[pose_name]
    if name.startswith('sealant:'):
        side_name = name.split(':', 1)[1]
        return sealant_point(geometry, spec, side_name, 0.0)
    raise GeometryError(f'unknown world target {name!r}')


def sealant_point(
    geometry: GeometrySpec,
    spec: ScenarioSpec,
    side_name: str,
    fraction: float,
) -> Transform:
    """Return a work-point transform along one named sealant segment."""
    if side_name not in geometry.sealant_sides:
        raise GeometryError(f'unknown sealant side {side_name!r}')
    if not 0.0 <= fraction <= 1.0:
        raise GeometryError('sealant fraction must be within 0..1')
    segment = geometry.sealant_sides[side_name]
    dx = segment.start[0] + (segment.end[0] - segment.start[0]) * fraction
    dy = segment.start[1] + (segment.end[1] - segment.start[1]) * fraction
    dx, dy = _rotate_xy(
        dx, dy, spec.body.yaw_deg - geometry.sealant_reference_yaw_deg,
    )
    z = spec.body.z + geometry.body_top_local + geometry.sealant_z_above_body_top
    return _down_transform(
        spec.body.x + dx, spec.body.y + dy, z, spec.body.yaw_deg,
        child=f'sealant_{side_name}',
    )


def tcp_goal(target: Transform, tcp_to_work_tip: Transform) -> Transform:
    """Convert a work-point target into the required tcp pose."""
    return target.compose_relaxed(tcp_to_work_tip.inverse())
