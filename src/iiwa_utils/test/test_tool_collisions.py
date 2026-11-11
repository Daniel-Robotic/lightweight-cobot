"""The camera/tool collision model only disables constructive contact pairs."""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

from iiwa_utils.tool_manager import _BASE_COLLISIONS, load_registry


ROOT = Path(__file__).resolve().parents[2]
TOOLS = ROOT / 'iiwa_config' / 'config' / 'tools.yaml'
SRDF = ROOT / 'iiwa_config' / 'config' / 'moveit' / 'iiwa7.srdf'
PATRON = ROOT / 'iiwa_description' / 'urdf' / 'tools' / 'patron.xacro'

FIXED_PAIRS = {
    tuple(sorted(pair)) for pair in (
        ('link7', 'camera_holder'),
        ('camera_holder', 'patron'),
        ('camera_holder', 'camera_corner'),
        ('camera_corner', 'camera_hand'),
        ('camera_holder', 'camera_hand'),
    )
}


def pairs(entries):
    return {
        tuple(sorted((str(first), str(second))))
        for first, second, *_ in entries
    }


def test_only_constructive_camera_pairs_are_disabled_in_the_source():
    registry = load_registry(TOOLS)
    collisions = registry['patron']['collisions']
    assert all(reason == 'Adjacent' for _, _, reason in collisions)
    assert pairs(collisions) == FIXED_PAIRS


def test_only_adjacent_robot_links_are_disabled():
    assert all(reason == 'Adjacent' for _, _, reason in _BASE_COLLISIONS)
    assert pairs(_BASE_COLLISIONS) == {
        ('base_link', 'link1'),
        ('link1', 'link2'),
        ('link2', 'link3'),
        ('link3', 'link4'),
        ('link4', 'link5'),
        ('link5', 'link6'),
        ('link6', 'link7'),
    }


def test_generated_srdf_keeps_camera_robot_collisions_enabled():
    root = ET.parse(SRDF).getroot()
    entries = [
        (
            element.get('link1'),
            element.get('link2'),
            element.get('reason'),
        )
        for element in root.findall('disable_collisions')
    ]
    assert all(reason == 'Adjacent' for _, _, reason in entries)
    disabled = pairs(entries)
    for camera in ('camera_holder', 'camera_corner', 'camera_hand'):
        for robot_link in (
            'base_link', 'link1', 'link2', 'link3', 'link4',
            'link5', 'link6', 'link7',
        ):
            pair = tuple(sorted((camera, robot_link)))
            if pair in FIXED_PAIRS:
                assert pair in disabled
            else:
                assert pair not in disabled, pair

def test_camera_hand_collision_box_matches_the_visible_d455_bounds():
    root = ET.parse(PATRON).getroot()
    link = root.find("link[@name='camera_hand']")
    collision = link.find('collision')
    origin = collision.find('origin')
    box = collision.find('geometry/box')
    values = [float(value) for value in box.get('size').split()]
    centre = [float(value) for value in origin.get('xyz').split()]
    assert values == [0.124, 0.031, 0.026]
    assert all(abs(a - b) < 1e-9 for a, b in zip(centre, [0.002, 0.0155, 0.013]))
