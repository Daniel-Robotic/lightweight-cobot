"""Deterministic object ownership for the grant reference executor.

The state machine is deliberately independent of ROS and Webots.  The
supervisor translates accepted commands into Webots field updates and the
reference runner uses the same transitions in its table tests.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from .geometry import GeometrySpec, Transform, object_pose_transform, world_target
from .scenario import ScenarioSpec


CAP_STATES = ('absent', 'free', 'installed', 'held')
SCREW_STATES = ('tray', 'held', 'delivered')
TOOL_STATES = ('docked', 'mounted')


class CommandError(RuntimeError):
    """A command was rejected; the state is unchanged."""


def transform_json(transform: Transform | None) -> dict[str, Any] | None:
    if transform is None:
        return None
    return {
        'xyz': [float(value) for value in transform.xyz],
        'rotation': [
            [float(value) for value in row] for row in transform.rotation
        ],
        'parent': transform.parent,
        'child': transform.child,
    }


def _position_distance(first: Transform, second: Transform) -> float:
    return float(np.linalg.norm(first.position - second.position))


def _angle_distance(first: Transform, second: Transform) -> float:
    cosine = (np.trace(first.matrix.T @ second.matrix) - 1.0) / 2.0
    return float(np.arccos(max(-1.0, min(1.0, cosine))))


class ObjectState:
    """Ownership and delivery state for one experiment."""

    def __init__(
        self,
        spec: ScenarioSpec,
        geometry: GeometrySpec,
        *,
        read_only: bool = False,
    ) -> None:
        self.spec = spec
        self.geometry = geometry
        self.read_only = read_only
        self.version = 0
        self.last_request_id: str | None = None
        self.last_error: str | None = None
        self.mounted_tool: str | None = None
        self.held_object: str | None = None
        self.cap_state = spec.cap_state
        self.cap_pose = (
            world_target(spec, geometry, 'cap')
            if spec.cap_state != 'absent'
            else None
        )
        self.cap_relative: Transform | None = None
        self.tools: dict[str, dict[str, Any]] = {}
        for tool in spec.tools:
            self.tools[tool.name] = {
                'state': 'docked',
                'pose': world_target(spec, geometry, f'tool:{tool.name}'),
            }
        self.screws: dict[str, dict[str, Any]] = {}
        for screw in spec.screws:
            self.screws[screw.name] = {
                'state': 'tray',
                'pose': world_target(spec, geometry, f'screw:{screw.name}'),
                'hole': None,
            }
        self.holes = {
            name: {'state': 'empty', 'screw': None}
            for name in ('hole_1', 'hole_2', 'hole_3', 'hole_4')
        }

    # ------------------------------------------------------------------ helpers
    def _reject(self, message: str) -> None:
        self.last_error = message
        raise CommandError(message)

    def _accept(self, request_id: str | None) -> None:
        self.version += 1
        self.last_request_id = request_id
        self.last_error = None

    def _require_not_read_only(self) -> None:
        if self.read_only:
            self._reject('supervisor is in read-only view mode')

    def _require_instrument(self, instrument: str, operation: str) -> None:
        if self.mounted_tool != instrument:
            self._reject(
                f'{operation} requires {instrument!r} in patron; '
                f'mounted tool is {self.mounted_tool!r}'
            )

    def _require_free_hand(self, operation: str) -> None:
        if self.held_object is not None:
            self._reject(
                f'{operation} is forbidden while {self.held_object!r} is held'
            )

    def _require_close(
        self,
        actual: Transform,
        expected: Transform,
        operation: str,
    ) -> None:
        if actual.parent != expected.parent:
            self._reject(f'{operation}: pose frames do not match')
        distance = _position_distance(actual, expected)
        angle = _angle_distance(actual, expected)
        if distance > self.geometry.position_tolerance:
            self._reject(
                f'{operation}: tcp is {distance:.4f} m from the contact pose '
                f'(limit {self.geometry.position_tolerance:.4f} m)'
            )
        if angle > self.geometry.orientation_tolerance_rad:
            self._reject(
                f'{operation}: tcp orientation error is {angle:.4f} rad '
                f'(limit {self.geometry.orientation_tolerance_rad:.4f} rad)'
            )

    def _expected_tcp(self, node_pose: Transform, tcp_to_node: Transform) -> Transform:
        return node_pose.compose_relaxed(tcp_to_node.inverse())

    # ------------------------------------------------------------------ tools
    def attach_tool(
        self,
        name: str,
        tcp: Transform,
        object_pose: Transform,
        *,
        request_id: str | None = None,
    ) -> None:
        self._require_not_read_only()
        if self.mounted_tool is not None:
            self._reject(f'another tool {self.mounted_tool!r} is already mounted')
        if name not in self.tools:
            self._reject(f'tool {name!r} is not present in this experiment')
        tool = self.tools[name]
        if tool['state'] != 'docked':
            self._reject(f'tool {name!r} is not in its socket')
        if _position_distance(object_pose, tool['pose']) > self.geometry.position_tolerance:
            self._reject(
                f'tool {name!r} moved away from its socket; refusing blind attach'
            )
        self._require_free_hand('attach_tool')
        self._require_close(
            tcp,
            self._expected_tcp(object_pose, self.geometry.tool_tcp_to_nodes[name]),
            f'attach_tool({name})',
        )
        tool['state'] = 'mounted'
        self.mounted_tool = name
        self._accept(request_id)

    def dock_tool(
        self,
        name: str,
        tcp: Transform,
        target: Transform,
        object_pose: Transform,
        *,
        request_id: str | None = None,
    ) -> None:
        self._require_not_read_only()
        if self.mounted_tool != name:
            self._reject(f'tool {name!r} is not the mounted tool')
        if name not in self.tools:
            self._reject(f'tool {name!r} is not present in this experiment')
        # The tool follows tcp, so the contact pose is only meaningful when
        # compared with the known socket target.
        self._require_close(
            tcp,
            self._expected_tcp(target, self.geometry.tool_tcp_to_nodes[name]),
            f'dock_tool({name})',
        )
        self._require_close(
            object_pose,
            tcp.compose_relaxed(self.geometry.tool_tcp_to_nodes[name]),
            f'dock_tool({name}) attached object',
        )
        tool = self.tools[name]
        tool['state'] = 'docked'
        tool['pose'] = target
        self.mounted_tool = None
        self._accept(request_id)

    # ------------------------------------------------------------------ hatch
    def grab_cap(
        self,
        tcp: Transform,
        object_pose: Transform,
        *,
        request_id: str | None = None,
    ) -> None:
        self._require_not_read_only()
        self._require_instrument('vacuum', 'grab_cap')
        self._require_free_hand('grab_cap')
        if self.cap_state not in ('free', 'installed'):
            self._reject(f'hatch is not available (state={self.cap_state!r})')
        # The suction cup is rotationally symmetric.  Require the correct
        # position and a downward tcp, but allow any grasp yaw; the measured
        # relative rotation is stored and reused for placement.
        expected_position = object_pose.position + np.array([0.0, 0.0, 0.146])
        distance = float(np.linalg.norm(tcp.position - expected_position))
        if distance > self.geometry.position_tolerance:
            self._reject(
                f'grab_cap: tcp is {distance:.4f} m from the hatch '
                f'(limit {self.geometry.position_tolerance:.4f} m)'
            )
        if tcp.matrix[2, 2] > -0.9:
            self._reject('grab_cap: tcp does not point down at the hatch')
        self.cap_relative = tcp.inverse().compose_relaxed(object_pose)
        self.cap_state = 'held'
        self.cap_pose = object_pose
        self.held_object = 'cap'
        self._accept(request_id)

    def place_cap(
        self,
        target: Transform,
        tcp: Transform,
        object_pose: Transform,
        *,
        request_id: str | None = None,
    ) -> None:
        self._require_not_read_only()
        self._require_instrument('vacuum', 'place_cap')
        if self.held_object != 'cap':
            self._reject('place_cap requires the hatch to be held')
        relative = self.cap_relative or self.geometry.cap_tcp_to_node
        self._require_close(
            tcp,
            self._expected_tcp(target, relative),
            'place_cap',
        )
        if _position_distance(object_pose, tcp.compose_relaxed(relative)) > (
            self.geometry.position_tolerance
        ):
            self._reject('place_cap: hatch does not follow the suction cup')
        self.cap_state = 'installed'
        self.cap_pose = target
        self.held_object = None
        self._accept(request_id)

    # ------------------------------------------------------------------ screws
    def grab_screw(
        self,
        screw_id: str,
        tcp: Transform,
        object_pose: Transform,
        *,
        request_id: str | None = None,
    ) -> None:
        self._require_not_read_only()
        self._require_instrument('magnetic_tool', 'grab_screw')
        self._require_free_hand('grab_screw')
        if screw_id not in self.screws:
            self._reject(f'screw {screw_id!r} is not present in this experiment')
        screw = self.screws[screw_id]
        if screw['state'] != 'tray':
            self._reject(f'screw {screw_id!r} is unavailable ({screw["state"]})')
        self._require_close(
            tcp,
            self._expected_tcp(object_pose, self.geometry.screw_tcp_to_node),
            f'grab_screw({screw_id})',
        )
        screw['state'] = 'held'
        screw['pose'] = object_pose
        self.held_object = f'screw:{screw_id}'
        self._accept(request_id)

    def handoff_screw(
        self,
        screw_id: str,
        hole_id: str,
        target: Transform,
        tcp: Transform,
        object_pose: Transform,
        *,
        request_id: str | None = None,
    ) -> None:
        self._require_not_read_only()
        self._require_instrument('magnetic_tool', 'handoff_screw')
        if self.held_object != f'screw:{screw_id}':
            self._reject(f'screw {screw_id!r} is not held')
        if screw_id not in self.screws:
            self._reject(f'screw {screw_id!r} is not present in this experiment')
        screw = self.screws[screw_id]
        if screw['state'] != 'held':
            self._reject(f'screw {screw_id!r} is not held ({screw["state"]})')
        if hole_id not in self.holes:
            self._reject(f'unknown hole {hole_id!r}')
        if self.holes[hole_id]['state'] != 'empty':
            self._reject(f'hole {hole_id!r} already has a delivered screw')
        if screw['hole'] == hole_id:
            self._reject(f'screw {screw_id!r} was already delivered to {hole_id!r}')
        self._require_close(
            tcp,
            self._expected_tcp(target, self.geometry.screw_tcp_to_node),
            f'handoff_screw({screw_id},{hole_id})',
        )
        expected_object = tcp.compose_relaxed(self.geometry.screw_tcp_to_node)
        if _position_distance(object_pose, expected_object) > self.geometry.position_tolerance:
            self._reject('handoff_screw: screw does not follow the magnet')
        screw['state'] = 'delivered'
        screw['hole'] = hole_id
        screw['pose'] = target
        self.holes[hole_id]['state'] = 'empty'
        self.holes[hole_id]['screw'] = screw_id
        self.held_object = None
        self._accept(request_id)

    # ------------------------------------------------------------------ inspect
    def snapshot(self) -> dict[str, Any]:
        return {
            'version': self.version,
            'experiment_id': self.spec.experiment_id,
            'mounted_tool': self.mounted_tool,
            'held_object': self.held_object,
            'read_only': self.read_only,
            'cap': {
                'state': self.cap_state,
                'pose': transform_json(self.cap_pose),
            },
            'tools': {
                name: {
                    'state': entry['state'],
                    'pose': transform_json(entry['pose']),
                }
                for name, entry in self.tools.items()
            },
            'screws': {
                name: {
                    'state': entry['state'],
                    'pose': transform_json(entry['pose']),
                    'hole': entry['hole'],
                }
                for name, entry in self.screws.items()
            },
            'holes': {
                name: {
                    'state': entry['state'],
                    'screw': entry['screw'],
                }
                for name, entry in self.holes.items()
            },
            'last_request_id': self.last_request_id,
            'last_error': self.last_error,
        }

    def delivered_pairs(self) -> list[tuple[str, str]]:
        return [
            (name, entry['hole'])
            for name, entry in self.screws.items()
            if entry['state'] == 'delivered'
        ]
