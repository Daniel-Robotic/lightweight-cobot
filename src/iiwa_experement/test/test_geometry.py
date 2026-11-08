"""Calibrated geometry: tool mounting, hatch, contour and holes."""
from pathlib import Path

import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from iiwa_experement.geometry import (
    GeometryError,
    Transform,
    load_geometry,
    object_pose_transform,
    sealant_point,
    tcp_goal,
    world_target,
)
from iiwa_experement.scenario import load_scenario


PACKAGE = Path(__file__).resolve().parents[1]
SCENES = PACKAGE / 'config' / 'scenes.yaml'
GEOMETRY = PACKAGE / 'config' / 'geometry.yaml'
ASSET_ROOT = (
    Path(get_package_share_directory('iiwa_description')) / 'objects' / 'grant'
)


def geometry():
    return load_geometry(GEOMETRY)


def scenario(experiment_id):
    return load_scenario(experiment_id, SCENES)


def obj_y_bounds(asset):
    low, high = np.inf, -np.inf
    for line in (ASSET_ROOT / asset).read_text(encoding='utf-8', errors='replace').splitlines():
        if line.startswith('v '):
            y = float(line.split()[2])
            low, high = min(low, y), max(high, y)
    return low, high


def test_tool_lengths_and_sealant_scale():
    for asset in ('vacum-tool.obj', 'magnit-tool.obj', 'wranch-tool.obj'):
        low, high = obj_y_bounds(asset)
        assert low == pytest.approx(0.0, abs=1e-12)
        assert high == pytest.approx(0.140, abs=1e-12)
    low, high = obj_y_bounds('germetic-tool.obj')
    assert low == pytest.approx(0.0, abs=1e-12)
    assert high == pytest.approx(0.014, abs=1e-12)
    assert high * 10.0 == pytest.approx(0.140)


def test_patron_tcp_offset_is_55_2_mm():
    spec = geometry()
    assert spec.patron_to_tcp.parent == 'patron'
    assert spec.patron_to_tcp.child == 'tcp'
    assert spec.patron_to_tcp.position == pytest.approx([0.0, 0.0, 0.0552])


def test_all_tools_mount_at_tcp_and_end_fourteen_centimetres_away():
    spec = geometry()
    for name in ('vacuum', 'magnetic_tool', 'sealant_tool'):
        mount = spec.tool_mounts[name]
        tip = spec.tool_work_tips[name]
        assert mount.parent == tip.parent == 'tcp'
        assert mount.position == pytest.approx([0.0, 0.0, 0.0])
        assert tip.position == pytest.approx([0.0, 0.0, 0.140])
        node = spec.tool_tcp_to_nodes[name]
        assert node.position == pytest.approx([0.0, 0.0, 0.140])
        # The node transform must reproduce an upright OBJ.
        world_tcp = Transform.from_xyz_rpy(
            [0.0, 0.0, 0.93], [180.0, 0.0, 45.0], parent='world', child='tcp',
        )
        tool_mount = world_tcp.compose(mount)
        assert tool_mount.parent == 'world'
        assert tool_mount.child == f'{name}_mount'
        node_world = world_tcp.compose(node)
        # Node origin is the working tip, 0.14 below the mount.
        assert node_world.position == pytest.approx([0.0, 0.0, 0.79], abs=1e-12)
        assert np.linalg.det(node_world.matrix) == pytest.approx(1.0)


def test_ten_screws_have_distinct_ids_and_poses():
    for experiment_id in range(11, 21):
        spec = scenario(experiment_id)
        assert len(spec.screws) == 10
        assert len({screw.name for screw in spec.screws}) == 10
        assert len({(screw.pose.x, screw.pose.y) for screw in spec.screws}) == 10


def test_experiment_three_moves_holes_with_the_body():
    standard = scenario(1)
    shifted = scenario(3)
    g = geometry()
    for hole in ('hole_1', 'hole_2', 'hole_3', 'hole_4'):
        ref = world_target(standard, g, f'hole:{hole}')
        moved = world_target(shifted, g, f'hole:{hole}')
        assert moved.position[:2] == pytest.approx(
            ref.position[:2] + np.array([-0.01, 0.02]), abs=1e-9,
        )
        assert moved.position[2] == pytest.approx(ref.position[2], abs=1e-12)


def test_standard_holes_match_the_documented_coordinates():
    g = geometry()
    spec = scenario(11)
    expected = {
        'hole_1': (0.2243, 0.1052),
        'hole_2': (0.2852, 0.0443),
        'hole_3': (0.3757, 0.1348),
        'hole_4': (0.3148, 0.1957),
    }
    for name, xy in expected.items():
        target = world_target(spec, g, f'hole:{name}')
        assert target.position[:2] == pytest.approx(xy, abs=1e-9)
        assert target.position[2] == pytest.approx(0.823 + 0.00894 + 0.002, abs=1e-12)


def test_experiment_nineteen_moves_screws_with_the_tray():
    g = geometry()
    standard = scenario(11)
    shifted = scenario(19)
    delta = np.array([-0.02, 0.02, 0.0])
    for index in range(1, 11):
        name = f'screw_{index}'
        ref = world_target(standard, g, f'screw:{name}')
        moved = world_target(shifted, g, f'screw:{name}')
        assert moved.position == pytest.approx(ref.position + delta, abs=1e-9)


def test_cap_seat_follows_the_body():
    g = geometry()
    for experiment_id in (1, 3, 11):
        spec = scenario(experiment_id)
        body = object_pose_transform(
            spec.body.x, spec.body.y, spec.body.z, spec.body.yaw_deg, child='body',
        )
        seat = world_target(spec, g, 'cap_seat')
        expected = body.compose(g.body_to_installed_cap)
        assert seat.position == pytest.approx(expected.position, abs=1e-12)
        assert seat.matrix == pytest.approx(expected.matrix, abs=1e-12)


def test_cap_collision_center_is_below_tcp():
    g = geometry()
    assert g.cap_collision.offset[2] == pytest.approx(0.146 - 0.004165, abs=1e-9)
    assert g.cap_collision.size[2] == pytest.approx(0.00367, abs=1e-12)


def test_sealant_contour_is_closed_and_ordered():
    g = geometry()
    spec = scenario(18)
    sides = ('side_1', 'side_2', 'side_3', 'side_4')
    starts = [world_target(spec, g, f'sealant:{side}') for side in sides]
    ends = [sealant_point(g, spec, side, 1.0) for side in sides]
    for index in range(4):
        assert starts[index].position[2] == pytest.approx(ends[index].position[2])
        assert starts[(index + 1) % 4].position[:2] == pytest.approx(
            ends[index].position[:2], abs=1e-9,
        )


def test_tcp_goal_places_tip_on_the_target_and_points_down():
    g = geometry()
    spec = scenario(18)
    target = sealant_point(g, spec, 'side_1', 0.5)
    tcp = tcp_goal(target, g.tool_work_tips['sealant_tool'])
    assert tcp.position == pytest.approx(
        target.position + np.array([0.0, 0.0, 0.140]), abs=1e-12,
    )
    assert tcp.matrix[2, 2] == pytest.approx(-1.0, abs=1e-12)


def test_tool_and_cap_node_targets_produce_downward_tcp():
    g = geometry()
    spec = scenario(1)
    for target, relative in (
        (world_target(spec, g, 'tool:vacuum'), g.tool_tcp_to_nodes['vacuum']),
        (world_target(spec, g, 'cap'), g.cap_tcp_to_node),
    ):
        tcp = tcp_goal(target, relative)
        assert tcp.matrix[2, 2] == pytest.approx(-1.0, abs=1e-9)


def test_missing_frames_are_rejected(tmp_path):
    data = yaml.safe_load(GEOMETRY.read_text(encoding='utf-8'))
    data['patron_to_tcp']['parent'] = 'missing'
    data['patron_to_tcp'].pop('parent')
    path = tmp_path / 'geometry.yaml'
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    # The loader supplies parent/child itself, so a missing key is not an
    # error; a missing transform block is.
    data.pop('patron_to_tcp')
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    with pytest.raises(GeometryError, match='patron_to_tcp'):
        load_geometry(path)


def test_nonnumeric_and_nonpositive_values_are_rejected(tmp_path):
    data = yaml.safe_load(GEOMETRY.read_text(encoding='utf-8'))
    data['approach_gap'] = 'far'
    path = tmp_path / 'geometry.yaml'
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    with pytest.raises(GeometryError, match='approach_gap'):
        load_geometry(path)

    data = yaml.safe_load(GEOMETRY.read_text(encoding='utf-8'))
    data['contact_gap'] = 0.0
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    with pytest.raises(GeometryError, match='contact_gap'):
        load_geometry(path)


def test_unknown_targets_are_rejected():
    g = geometry()
    with pytest.raises(GeometryError, match='unknown world target'):
        world_target(scenario(1), g, 'nothing')
    with pytest.raises(GeometryError, match='has no tool'):
        world_target(scenario(1), g, 'tool:magnetic_tool')
    with pytest.raises(GeometryError, match='has no screw'):
        world_target(scenario(1), g, 'screw:screw_1')
