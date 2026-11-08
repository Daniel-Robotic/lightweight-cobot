#!/usr/bin/env python3
"""Webots supervisor for attachment and object ownership.

The controller is started next to Webots through the experiment launch file.
It exposes ``/iiwa_experement/object_command`` (webots_ros2_msgs/SetString)
and publishes ``/iiwa_experement/object_state`` as JSON.  It never commands
the robot; it only follows the actual ``tcp`` pose when an object is held.
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from webots_ros2_msgs.srv import SetString

from .geometry import GeometrySpec, Transform, load_geometry
from .object_state import CommandError, ObjectState, transform_json
from .scenario import ScenarioSpec, load_scenario


COMMAND_SERVICE = '/iiwa_experement/object_command'
STATE_TOPIC = '/iiwa_experement/object_state'


def screw_def_name(screw_id: str) -> str:
    parts = screw_id.split('_')
    if len(parts) != 2 or parts[0] != 'screw' or not parts[1].isdigit():
        raise ValueError(f'invalid screw id {screw_id!r}')
    return f'grant_screw_{int(parts[1]):02d}'


def _matrix_from_values(values) -> tuple[tuple[float, float, float], ...]:
    if not isinstance(values, (list, tuple)):
        raise ValueError('rotation must contain 9 row-major values')
    if len(values) == 3 and all(
        isinstance(row, (list, tuple)) and len(row) == 3 for row in values
    ):
        return tuple(
            tuple(float(value) for value in row) for row in values
        )
    if len(values) != 9:
        raise ValueError('rotation must contain 9 row-major values')
    return tuple(
        tuple(float(values[row * 3 + column]) for column in range(3))
        for row in range(3)
    )


def transform_from_json(data: dict[str, Any], child: str = 'target') -> Transform:
    xyz = data.get('xyz')
    if not isinstance(xyz, (list, tuple)) or len(xyz) != 3:
        raise ValueError('pose.xyz must contain three numbers')
    if 'rotation' in data:
        rotation = _matrix_from_values(data['rotation'])
    else:
        from scipy.spatial.transform import Rotation

        rotation = tuple(
            tuple(float(value) for value in row)
            for row in Rotation.from_euler(
                'xyz', data.get('rpy_deg', [0.0, 0.0, 0.0]), degrees=True
            ).as_matrix()
        )
    return Transform(
        xyz=tuple(float(value) for value in xyz),
        rotation=rotation,
        parent='world',
        child=child,
    )


def _axis_angle(rotation) -> list[float]:
    from scipy.spatial.transform import Rotation

    rotvec = Rotation.from_matrix(np.asarray(rotation, dtype=float)).as_rotvec()
    angle = float(np.linalg.norm(rotvec))
    if angle < 1e-12:
        return [0.0, 0.0, 1.0, 0.0]
    axis = (rotvec / angle).tolist()
    return [float(axis[0]), float(axis[1]), float(axis[2]), angle]


class GrantSupervisorNode(Node):
    """ROS facade over :class:`ObjectState` and the Webots scene tree."""

    def __init__(self) -> None:
        super().__init__('grant_attachment_supervisor')
        package_share = Path(get_package_share_directory('iiwa_experement'))
        self.declare_parameter('experiment_id', 1)
        self.declare_parameter(
            'scene_config', str(package_share / 'config' / 'scenes.yaml'),
        )
        self.declare_parameter(
            'geometry_path', str(package_share / 'config' / 'geometry.yaml'),
        )
        self.declare_parameter('read_only', False)
        experiment_id = int(self.get_parameter('experiment_id').value)
        scene_config = Path(str(self.get_parameter('scene_config').value))
        geometry_path = Path(str(self.get_parameter('geometry_path').value))
        read_only = bool(self.get_parameter('read_only').value)
        self.spec: ScenarioSpec = load_scenario(experiment_id, scene_config)
        self.geometry: GeometrySpec = load_geometry(geometry_path)
        self.state = ObjectState(self.spec, self.geometry, read_only=read_only)
        self.tool_held: str | None = None
        self.payload: tuple[str, str] | None = None
        self.last_command_result: dict[str, Any] | None = None
        self.rack_poses = {
            name: entry['pose'] for name, entry in self.state.tools.items()
        }
        self.nodes: dict[str, Any] = {}
        self.patron_node = None
        self.robot = None
        self.timestep = 32
        self._last_publish_time = -1.0
        self._publisher = self.create_publisher(String, STATE_TOPIC, 10)
        self._service = self.create_service(
            SetString, COMMAND_SERVICE, self.handle_command,
        )
        self.get_logger().info(
            f'grant supervisor started for experiment {experiment_id}; '
            f'read_only={read_only}'
        )

    # ------------------------------------------------------------------ Webots
    def _pose_node(self, node):
        """CadShape children can report an uninitialised matrix in Webots."""
        if node is None:
            return None
        try:
            parent = node.getParentNode()
        except Exception:
            parent = None
        if parent is not None:
            position = parent.getPosition()
            if all(np.isfinite(position)):
                return parent
        return node

    def connect(self) -> None:
        from controller import Supervisor

        self.robot = Supervisor()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.patron_node = self._pose_node(self._find_node('patron_visual'))
        required = self._required_defs()
        for name in required:
            node = self._find_node(name)
            if node is not None:
                self.nodes[name] = node
                self._freeze(node)
        self.get_logger().info(
            f'supervisor found {len(self.nodes)}/{len(required)} scene objects'
        )

    def _find_node(self, name: str):
        try:
            return self.robot.getFromDef(name)
        except Exception:
            return None

    @staticmethod
    def _freeze(node) -> None:
        try:
            field = node.getField('locked')
            if field is not None:
                field.setSFBool(True)
        except Exception:
            pass
        # All managed objects are teleported by the supervisor.  Leaving a
        # physics node attached lets Webots integrate huge velocities while
        # the pose is overwritten every step.
        try:
            physics = node.getField('physics')
            if physics is not None:
                physics.remove()
        except Exception:
            pass

    def _pose_of(self, node) -> Transform | None:
        if node is None:
            return None
        position = node.getPosition()
        orientation = node.getOrientation()
        return Transform(
            xyz=tuple(float(value) for value in position),
            rotation=_matrix_from_values(orientation),
            parent='world',
            child='object',
        )

    def _set_pose(self, node, transform: Transform) -> None:
        node.getField('translation').setSFVec3f(
            [float(value) for value in transform.xyz]
        )
        rotation = node.getField('rotation')
        if rotation is not None:
            rotation.setSFRotation(_axis_angle(transform.rotation))

    def tcp_pose(self) -> Transform | None:
        patron = self._pose_of(self.patron_node)
        if patron is None:
            return None
        return patron.compose_relaxed(self.geometry.patron_to_tcp)

    def _refresh_nodes(self) -> None:
        if self.robot is None:
            return
        if self.patron_node is None:
            self.patron_node = self._pose_node(self._find_node('patron_visual'))
        for name in self._required_defs():
            if name not in self.nodes:
                node = self._find_node(name)
                if node is not None:
                    self.nodes[name] = node
                    self._freeze(node)

    def _required_defs(self) -> list[str]:
        names = ['grant_fixture', 'grant_body', 'grant_rack']
        names.extend(f'grant_{tool}' for tool in self.spec.tool_names)
        if self.spec.tray_pose is not None:
            names.append('grant_tray')
            names.extend(f'grant_screw_{index:02d}' for index in range(1, 11))
        if self.spec.cap_state != 'absent':
            names.append('grant_cap')
        return names

    def _tool_node(self):
        if self.tool_held is None:
            return None
        return self.nodes.get(f'grant_{self.tool_held}')

    def _payload_node(self):
        if self.payload is None:
            return None
        kind, name = self.payload
        if kind == 'cap':
            return self.nodes.get('grant_cap')
        if kind == 'screw':
            return self.nodes.get(screw_def_name(name))
        return None

    def _payload_relative(self) -> Transform | None:
        if self.payload is None:
            return None
        kind, _ = self.payload
        if kind == 'cap':
            return self.geometry.cap_tcp_to_node
        if kind == 'screw':
            return self.geometry.screw_tcp_to_node
        return None

    def follow(self) -> None:
        tcp = self.tcp_pose()
        if tcp is None:
            return
        if self.tool_held is not None:
            node = self._tool_node()
            relative = self.geometry.tool_tcp_to_nodes[self.tool_held]
            if node is not None:
                pose = tcp.compose_relaxed(relative)
                self._set_pose(node, pose)
                self.state.tools[self.tool_held]['pose'] = pose
        if self.payload is not None:
            node = self._payload_node()
            relative = self._payload_relative()
            if node is not None and relative is not None:
                pose = tcp.compose_relaxed(relative)
                self._set_pose(node, pose)
                kind, name = self.payload
                if kind == 'cap':
                    self.state.cap_pose = pose
                elif kind == 'screw' and name in self.state.screws:
                    self.state.screws[name]['pose'] = pose

    # ------------------------------------------------------------------ state
    def _actual_pose(self, node) -> Transform | None:
        return self._pose_of(node)

    def _command_pose(self, command: dict[str, Any]) -> Transform:
        target = command.get('target')
        if not isinstance(target, dict):
            raise CommandError('command requires a target pose')
        return transform_from_json(target, child=str(command.get('object') or 'target'))

    def handle_command(self, request: SetString.Request, response: SetString.Response):
        try:
            command = json.loads(request.value)
            if not isinstance(command, dict):
                raise CommandError('command must be a JSON object')
            request_id = str(command.get('request_id', ''))
            operation = str(command.get('op', ''))
            accepted, error = self._apply(operation, command, request_id)
        except Exception as error:  # never leak an exception into Webots
            accepted = False
            error = str(error)
            self.state.last_error = error
        # The detailed outcome is published in the next object_state message;
        # the service response only carries the accepted flag.
        response.success = bool(accepted)
        self.last_command_result = {
            'request_id': request_id,
            'accepted': bool(accepted),
            'error': None if accepted else error,
            'version': self.state.version,
        }
        self.publish_state(force=True)
        return response

    def _apply(self, operation: str, command: dict[str, Any], request_id: str):
        try:
            return self._apply_impl(operation, command, request_id)
        except CommandError as error:
            self.state.last_error = str(error)
            return False, str(error)

    def _apply_impl(self, operation: str, command: dict[str, Any], request_id: str):
        if operation == 'inspect':
            return True, None
        # A command can arrive before the per-step follow update.  Apply the
        # current tcp to the held object first, otherwise the attachment
        # appears one control step behind (about 11 mm at transfer speed).
        self.follow()
        tcp = self.tcp_pose()
        if tcp is None:
            raise CommandError('actual tcp pose is unavailable')
        kind = str(command.get('object') or '')
        if operation == 'attach_tool':
            node = self.nodes.get(f'grant_{kind}')
            object_pose = self._actual_pose(node)
            if object_pose is None:
                raise CommandError(f'tool node grant_{kind} is unavailable')
            self.state.attach_tool(kind, tcp, object_pose, request_id=request_id)
            self.tool_held = kind
            return True, None
        if operation == 'dock_tool':
            node = self.nodes.get(f'grant_{kind}')
            object_pose = self._actual_pose(node)
            if object_pose is None:
                raise CommandError(f'tool node grant_{kind} is unavailable')
            rack = self.rack_poses[kind]
            self.state.dock_tool(
                kind, tcp, rack, object_pose, request_id=request_id,
            )
            self.tool_held = None
            self._set_pose(node, rack)
            self.state.tools[kind]['pose'] = rack
            return True, None
        if operation == 'grab_cap':
            node = self.nodes.get('grant_cap')
            object_pose = self._actual_pose(node)
            if object_pose is None:
                raise CommandError('hatch node is unavailable')
            self.state.grab_cap(tcp, object_pose, request_id=request_id)
            self.payload = ('cap', 'cap')
            return True, None
        if operation == 'place_cap':
            target = self._command_pose(command)
            node = self.nodes.get('grant_cap')
            object_pose = self._actual_pose(node)
            if object_pose is None:
                raise CommandError('hatch node is unavailable')
            self.state.place_cap(target, tcp, object_pose, request_id=request_id)
            self.payload = None
            self._set_pose(node, target)
            self._freeze(node)
            self.state.cap_pose = target
            return True, None
        if operation == 'grab_screw':
            node = self.nodes.get(screw_def_name(kind))
            object_pose = self._actual_pose(node)
            if object_pose is None:
                raise CommandError(f'screw node for {kind} is unavailable')
            self.state.grab_screw(kind, tcp, object_pose, request_id=request_id)
            self.payload = ('screw', kind)
            return True, None
        if operation == 'handoff_screw':
            hole = str(command.get('target_name') or '')
            target = self._command_pose(command)
            node = self.nodes.get(screw_def_name(kind))
            object_pose = self._actual_pose(node)
            if object_pose is None:
                raise CommandError(f'screw node for {kind} is unavailable')
            self.state.handoff_screw(
                kind, hole, target, tcp, object_pose, request_id=request_id,
            )
            self.payload = None
            if node is not None:
                try:
                    node.remove()
                except Exception:
                    pass
            self.nodes.pop(screw_def_name(kind), None)
            return True, None
        raise CommandError(f'unknown supervisor operation {operation!r}')

    def publish_state(self, *, force: bool = False) -> None:
        if self.robot is None:
            return
        now = float(self.robot.getTime())
        if not force and now - self._last_publish_time < 0.1:
            return
        self._last_publish_time = now
        payload = self.state.snapshot()
        payload['simulation_time'] = now
        payload['ready'] = self.patron_node is not None and all(
            name in self.nodes for name in self._required_defs()
        )
        payload['attached'] = {
            'tool': self.tool_held,
            'payload': None if self.payload is None else list(self.payload),
        }
        tcp = self.tcp_pose()
        payload['tcp'] = transform_json(tcp)
        payload['last_command_result'] = self.last_command_result
        payload['objects'] = {
            name: transform_json(self._pose_of(node))
            for name, node in self.nodes.items()
        }
        message = String()
        message.data = json.dumps(payload)
        self._publisher.publish(message)

    def run(self) -> None:
        self.connect()
        while self.robot.step(self.timestep) != -1:
            try:
                rclpy.spin_once(self, timeout_sec=0.0)
                self._refresh_nodes()
                self.follow()
                self.publish_state()
            except Exception as error:  # keep the controller alive for diagnosis
                self.get_logger().error(f'supervisor step failed: {error}')


def _publish_unavailable(node: Node, message: str) -> None:
    del node
    print(f'[iiwa_experement] grant supervisor: {message}', file=sys.stderr)


def main(args=None) -> None:
    rclpy.init(args=args)
    exit_code = 0
    node = None
    try:
        node = GrantSupervisorNode()
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print(
            f'[iiwa_experement] grant supervisor: {error}',
            file=sys.stderr,
        )
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
