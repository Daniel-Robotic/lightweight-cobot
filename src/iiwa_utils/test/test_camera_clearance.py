"""Visible camera geometry has a positive gap to the robot links."""
from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from urdf_parser_py.urdf import URDF

from ament_index_python.packages import get_package_share_directory
from iiwa_utils import converter, setting_loader


ROOT = Path(__file__).resolve().parents[2]
MIN_GAP_M = 0.005


def _transform(xyz, rpy):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    matrix[:3, 3] = xyz
    return matrix


def _axis_rotation(axis, angle):
    vector = np.asarray(axis, dtype=float)
    vector /= np.linalg.norm(vector)
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_rotvec(vector * angle).as_matrix()
    return matrix


def _robot_description():
    settings = setting_loader.build_settings(
        settings_path=str(
            Path(get_package_share_directory('iiwa_config'))
            / 'config' / 'setting.yaml'
        ),
        check_files=True,
    )
    return URDF.from_xml_string(converter.load_robot_description(
        settings.robot.description,
        settings.robot.name,
        {
            'simulate': 'true',
            'initial_positions_file': settings.controller.moveit.initial_positions,
            'joint_limits_file': settings.controller.moveit.joint_limits,
        },
    ))


def _fk(robot, target, joints):
    by_child = {joint.child: joint for joint in robot.joints}
    chain = []
    node = target
    while node != 'base_link':
        joint = by_child[node]
        chain.append(joint)
        node = joint.parent
    result = np.eye(4)
    for joint in reversed(chain):
        result = result @ _transform(joint.origin.xyz, joint.origin.rpy)
        if joint.type == 'revolute':
            result = result @ _axis_rotation(
                joint.axis, joints.get(joint.name, 0.0),
            )
    return result


def _stl_box(path):
    raw = Path(path).read_bytes()
    count = int(np.frombuffer(raw, dtype='<u4', count=1, offset=80)[0])
    dtype = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    vertices = np.frombuffer(
        raw, dtype=dtype, count=count, offset=84,
    )['v'].reshape(-1, 3)
    return vertices.min(axis=0), vertices.max(axis=0)


def _corners(centre, half, rotation):
    signs = np.array([
        [x, y, z]
        for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)
    ], dtype=float)
    return centre + (signs @ np.diag(half)) @ rotation.T


def _aabb_gap(first, second):
    delta = np.maximum(
        0.0,
        np.maximum(first.min(axis=0) - second.max(axis=0),
                   second.min(axis=0) - first.max(axis=0)),
    )
    return float(np.linalg.norm(delta))


CAMERA_BOXES = {
    'camera_holder': [
        ([-0.035, -0.035, 0.0], [0.035, 0.035, 0.03]),
        ([-0.015, 0.03, 0.0], [0.015, 0.13, 0.03]),
    ],
    'camera_corner': [
        ([-0.062, 0.1015, 0.0135], [0.062, 0.1305, 0.0305]),
    ],
    'camera_hand': [
        ([-0.06, 0.0, 0.0], [0.064, 0.031, 0.026]),
    ],
}


def _gaps(joints):
    robot = _robot_description()
    result = {}
    links = {}
    for name in ('link6', 'link7'):
        transform = _fk(robot, name, joints)
        minimum, maximum = _stl_box(
            ROOT / 'iiwa_description' / 'resource' / 'meshes'
            / 'iiwa' / 'collision' / f'{name}.STL'
        )
        centre = transform[:3, :3] @ ((minimum + maximum) / 2.0) + transform[:3, 3]
        corners = _corners(
            centre, (maximum - minimum) / 2.0, transform[:3, :3],
        )
        links[name] = corners
    for camera, boxes in CAMERA_BOXES.items():
        transform = _fk(robot, camera, joints)
        for link_name, link_corners in links.items():
            for minimum, maximum in boxes:
                centre_local = (np.array(minimum) + np.array(maximum)) / 2.0
                half = (np.array(maximum) - np.array(minimum)) / 2.0
                centre = transform[:3, :3] @ centre_local + transform[:3, 3]
                corners = _corners(centre, half, transform[:3, :3])
                result[(camera, link_name)] = min(
                    result.get((camera, link_name), float('inf')),
                    _aabb_gap(corners, link_corners),
                )
    return result


def test_camera_corner_and_hand_clear_the_robot_at_work_and_home():
    for joints in ({}, {'joint4': -1.57, 'joint6': 1.57}):
        gaps = _gaps(joints)
        for camera in ('camera_corner', 'camera_hand'):
            for link in ('link6', 'link7'):
                assert gaps[(camera, link)] >= MIN_GAP_M, (camera, link, gaps)
        # The holder is bolted to the flange; only link7 contact is
        # constructive and disabled in the SRDF.
        assert gaps[('camera_holder', 'link7')] == 0.0
        assert gaps[('camera_holder', 'link6')] >= MIN_GAP_M
