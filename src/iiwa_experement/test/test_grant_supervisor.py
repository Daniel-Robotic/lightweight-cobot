"""Supervisor command plumbing with fake Webots nodes (no simulator run)."""
import math
from pathlib import Path

import numpy as np
import pytest

from iiwa_experement.geometry import load_geometry, tcp_goal, world_target
from iiwa_experement.grant_supervisor import (
    COMMAND_SERVICE,
    GrantSupervisorNode,
    _axis_angle,
    transform_from_json,
)
from iiwa_experement.object_state import ObjectState, transform_json
from iiwa_experement.scenario import load_scenario


PACKAGE = Path(__file__).resolve().parents[1]
SCENES = PACKAGE / 'config' / 'scenes.yaml'
GEOMETRY = PACKAGE / 'config' / 'geometry.yaml'


class FakeField:
    def __init__(self, value):
        self.value = value
        self.removed = False

    def setSFVec3f(self, value):
        self.value = list(value)

    def setSFRotation(self, value):
        self.value = list(value)

    def setSFBool(self, value):
        self.value = bool(value)

    def remove(self):
        self.removed = True


class FakeNode:
    def __init__(self, transform):
        self.translation = FakeField(list(transform.xyz))
        self.rotation = FakeField(_axis_angle(transform.rotation))
        self.locked = FakeField(True)
        self.removed = False
        self._matrix = transform.matrix

    def getPosition(self):
        return list(self.translation.value)

    def getOrientation(self):
        return [float(value) for value in self._matrix.reshape(-1)]

    def getField(self, name):
        if name == 'translation':
            return self.translation
        if name == 'rotation':
            return self.rotation
        if name == 'locked':
            return self.locked
        return None

    def remove(self):
        self.removed = True


class FakeRobot:
    def getTime(self):
        return 0.0


def make_node(experiment_id=1):
    geometry = load_geometry(GEOMETRY)
    spec = load_scenario(experiment_id, SCENES)
    node = object.__new__(GrantSupervisorNode)
    node.spec = spec
    node.geometry = geometry
    node.state = ObjectState(spec, geometry)
    node.tool_held = None
    node.payload = None
    node.rack_poses = {
        name: entry['pose'] for name, entry in node.state.tools.items()
    }
    node.robot = FakeRobot()
    node.patron_node = None
    node.nodes = {}
    for tool in spec.tools:
        pose = world_target(spec, geometry, f'tool:{tool.name}')
        node.nodes[f'grant_{tool.name}'] = FakeNode(pose)
    if spec.tray_pose is not None:
        for index in range(1, 11):
            name = f'screw_{index}'
            node.nodes[f'grant_screw_{index:02d}'] = FakeNode(
                world_target(spec, geometry, f'screw:{name}')
            )
    if spec.cap_state != 'absent':
        node.nodes['grant_cap'] = FakeNode(world_target(spec, geometry, 'cap'))
    return node


def set_tcp(node, tcp):
    patron = tcp.compose_relaxed(node.geometry.patron_to_tcp.inverse())
    node.patron_node = FakeNode(patron)


def test_command_service_name_is_stable():
    assert COMMAND_SERVICE == '/iiwa_experement/object_command'


def test_transform_json_roundtrip():
    from iiwa_experement.geometry import Transform

    transform = Transform.from_xyz_rpy(
        [0.1, -0.2, 0.3], [10.0, 20.0, 30.0], parent='world', child='x',
    )
    payload = transform_json(transform)
    restored = transform_from_json(payload)
    assert restored.xyz == pytest.approx(transform.xyz)
    assert restored.matrix == pytest.approx(transform.matrix)


def test_invalid_pose_payload_is_rejected():
    with pytest.raises(ValueError, match='xyz'):
        transform_from_json({'rotation': [1, 0, 0, 0, 1, 0, 0, 0, 1]})
    with pytest.raises(ValueError, match='rotation'):
        transform_from_json({'xyz': [0, 0, 0], 'rotation': [1, 0, 0]})


def test_axis_angle_identity_and_half_turn():
    assert _axis_angle(np.eye(3)) == [0.0, 0.0, 1.0, 0.0]
    from scipy.spatial.transform import Rotation

    axis = _axis_angle(Rotation.from_euler('z', 90, degrees=True).as_matrix())
    assert abs(axis[3]) == pytest.approx(math.pi / 2)
    assert abs(axis[2]) == pytest.approx(1.0)


def test_attach_requires_tcp_at_the_socket_and_rejects_far_pose():
    node = make_node(1)
    pose = node.state.tools['vacuum']['pose']
    good = tcp_goal(pose, node.geometry.tool_tcp_to_nodes['vacuum'])
    set_tcp(node, good)
    accepted, error = node._apply('attach_tool', {'object': 'vacuum'}, 'r1')
    assert accepted is True
    assert error is None
    assert node.tool_held == 'vacuum'

    other = make_node(1)
    set_tcp(other, good.compose_relaxed(
        type(good).from_xyz_rpy(
            [0.0, 0.0, 0.08], [0, 0, 0], parent='world', child='tcp',
        )
    ))
    accepted, error = other._apply('attach_tool', {'object': 'vacuum'}, 'r2')
    assert accepted is False
    assert 'tcp is' in error
    assert other.tool_held is None
    assert other.state.version == 0


def test_follow_moves_the_attached_tool_and_dock_returns_it_to_the_rack():
    node = make_node(1)
    rack_pose = node.state.tools['vacuum']['pose']
    tcp = tcp_goal(rack_pose, node.geometry.tool_tcp_to_nodes['vacuum'])
    set_tcp(node, tcp)
    assert node._apply('attach_tool', {'object': 'vacuum'}, 'attach')[0]
    node.follow()
    moved = node.nodes['grant_vacuum'].getPosition()
    assert moved == pytest.approx(rack_pose.position, abs=1e-9)
    accepted, _ = node._apply('dock_tool', {'object': 'vacuum'}, 'dock')
    assert accepted
    assert node.tool_held is None
    assert node.state.mounted_tool is None
    assert node.nodes['grant_vacuum'].getPosition() == pytest.approx(
        rack_pose.position, abs=1e-9,
    )


def test_grab_and_handoff_remove_only_the_selected_screw():
    node = make_node(11)
    magnetic = node.state.tools['magnetic_tool']['pose']
    tcp = tcp_goal(magnetic, node.geometry.tool_tcp_to_nodes['magnetic_tool'])
    set_tcp(node, tcp)
    assert node._apply('attach_tool', {'object': 'magnetic_tool'}, 'a')[0]

    screw = node.state.screws['screw_1']['pose']
    screw_node = node.nodes['grant_screw_01']
    tcp = tcp_goal(screw, node.geometry.screw_tcp_to_node)
    set_tcp(node, tcp)
    assert node._apply('grab_screw', {'object': 'screw_1'}, 'g')[0]
    node.follow()
    assert node.nodes['grant_screw_01'].getPosition() == pytest.approx(
        screw.position, abs=1e-9,
    )

    hole = world_target(node.spec, node.geometry, 'hole:hole_1')
    tcp = tcp_goal(hole, node.geometry.screw_tcp_to_node)
    set_tcp(node, tcp)
    node.follow()
    target = transform_json(hole)
    accepted, error = node._apply(
        'handoff_screw',
        {'object': 'screw_1', 'target': target, 'target_name': 'hole_1'},
        'h',
    )
    assert accepted, error
    assert screw_node.removed is True
    assert node.nodes['grant_screw_02'].removed is False
    assert node.state.holes['hole_1']['state'] == 'empty'


def test_follow_refuses_when_tcp_is_unknown():
    node = make_node(1)
    node.patron_node = None
    node.tool_held = 'vacuum'
    node.follow()
    assert node.nodes['grant_vacuum'].getPosition() == pytest.approx(
        node.state.tools['vacuum']['pose'].position, abs=1e-9,
    )


def test_mounted_tool_and_payload_follow_together():
    node = make_node(1)
    tool_pose = node.state.tools['vacuum']['pose']
    tcp = tcp_goal(tool_pose, node.geometry.tool_tcp_to_nodes['vacuum'])
    set_tcp(node, tcp)
    assert node._apply('attach_tool', {'object': 'vacuum'}, 't')[0]
    cap_pose = node.state.cap_pose
    cap_tcp = tcp_goal(cap_pose, node.geometry.cap_tcp_to_node)
    set_tcp(node, cap_tcp)
    assert node._apply('grab_cap', {'object': 'cap'}, 'c')[0]
    node.follow()
    expected_tool = cap_tcp.compose_relaxed(node.geometry.tool_tcp_to_nodes['vacuum'])
    expected_cap = cap_tcp.compose_relaxed(node.geometry.cap_tcp_to_node)
    assert node.nodes['grant_vacuum'].getPosition() == pytest.approx(
        expected_tool.position, abs=1e-9,
    )
    assert node.nodes['grant_cap'].getPosition() == pytest.approx(
        expected_cap.position, abs=1e-9,
    )
