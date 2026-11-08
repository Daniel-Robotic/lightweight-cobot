"""Deterministic Webots nodes derived from each scenario."""
import math
import re
from pathlib import Path

import pytest
from ament_index_python.packages import get_package_share_directory

from iiwa_experement.scenario import TOOL_NAMES, load_scenario
from iiwa_experement.webots_nodes import (
    ASSETS,
    TOOL_NODE_NAMES,
    build_nodes,
    rotation_axis_angle,
    rotation_matrix,
)


PACKAGE = Path(__file__).resolve().parents[1]
SCENES = PACKAGE / 'config' / 'scenes.yaml'
ASSET_ROOT = (
    Path(get_package_share_directory('iiwa_description')) / 'objects' / 'grant'
)

NAME_RE = re.compile(r'name "([^"]+)"')
TRANSLATION_RE = re.compile(r'translation ([^\n]+)')
ROTATION_RE = re.compile(r'rotation ([^\n]+)')
URL_RE = re.compile(r'url \[ "([^"]+)" \]')


def spec(experiment_id):
    return load_scenario(experiment_id, SCENES)


def nodes(experiment_id):
    return build_nodes(spec(experiment_id), ASSET_ROOT)


def by_name(node_list):
    result = {}
    for node in node_list:
        match = NAME_RE.search(node)
        assert match, node
        result[match.group(1)] = node
    return result


def translation(node):
    return tuple(map(float, TRANSLATION_RE.search(node).group(1).split()))


def rotation(node):
    return tuple(map(float, ROTATION_RE.search(node).group(1).split()))


def obj_y_bounds(asset):
    low, high = math.inf, -math.inf
    for line in (ASSET_ROOT / asset).read_text(encoding='utf-8', errors='replace').splitlines():
        if line.startswith('v '):
            y = float(line.split()[2])
            low, high = min(low, y), max(high, y)
    return low, high


def test_scenarios_one_six_eleven_sixteen_nineteen_are_complete():
    expected = {
        1: ['grant_fixture', 'grant_body', 'grant_rack', 'grant_vacuum', 'grant_cap'],
        6: ['grant_fixture', 'grant_body', 'grant_rack', 'grant_sealant_tool'],
        11: [
            'grant_fixture', 'grant_body', 'grant_rack', 'grant_magnetic_tool',
            'grant_tray', *[f'grant_screw_{i:02d}' for i in range(1, 11)],
            'grant_cap',
        ],
        16: [
            'grant_fixture', 'grant_body', 'grant_rack',
            'grant_vacuum', 'grant_magnetic_tool', 'grant_sealant_tool',
            'grant_tray',
            *[f'grant_screw_{i:02d}' for i in range(1, 11)], 'grant_cap',
        ],
        19: [
            'grant_fixture', 'grant_body', 'grant_rack',
            'grant_vacuum', 'grant_magnetic_tool', 'grant_sealant_tool',
            'grant_tray',
            *[f'grant_screw_{i:02d}' for i in range(1, 11)], 'grant_cap',
        ],
    }
    for experiment_id, names in expected.items():
        assert [NAME_RE.search(node).group(1) for node in nodes(experiment_id)] == names


def test_node_names_are_unique_and_never_duplicate_patron():
    for experiment_id in range(1, 21):
        node_list = nodes(experiment_id)
        names = [NAME_RE.search(node).group(1) for node in node_list]
        assert len(names) == len(set(names))
        assert 'patron' not in names
        assert all('patron' not in node for node in node_list)
        assert all('GrantToolRack' not in node for node in node_list)


def test_all_mesh_urls_are_existing_installed_assets():
    for experiment_id in range(1, 21):
        for node in nodes(experiment_id):
            urls = URL_RE.findall(node)
            assert len(urls) == 1, node
            path = Path(urls[0])
            assert path.is_file(), path
            assert path.parent == ASSET_ROOT
            assert 'output' not in path.parts


def test_every_required_obj_and_mtl_exists_for_all_twenty_scenarios():
    for experiment_id in range(1, 21):
        scenario = spec(experiment_id)
        required = {'fixture', 'body', 'rack'}
        required.update(tool.name for tool in scenario.tools)
        if scenario.tray_pose is not None:
            required.update({'tray', 'screw'})
        if scenario.cap_state != 'absent':
            required.add('cap')
        for name in required:
            asset = ASSET_ROOT / ASSETS[name]
            assert asset.is_file(), (experiment_id, asset)
            for line in asset.read_text(encoding='utf-8').splitlines():
                if line.startswith('mtllib'):
                    assert (ASSET_ROOT / line.split()[1]).is_file()


def test_shared_pose_translations_match_the_scenario():
    for experiment_id in (1, 2, 3, 10, 19):
        scenario = spec(experiment_id)
        parsed = by_name(nodes(experiment_id))
        assert translation(parsed['grant_fixture']) == pytest.approx(
            (scenario.fixture.x, scenario.fixture.y, scenario.fixture.z)
        )
        assert translation(parsed['grant_body']) == pytest.approx(
            (scenario.body.x, scenario.body.y, scenario.body.z)
        )
        assert translation(parsed['grant_rack']) == pytest.approx(
            (scenario.rack.x, scenario.rack.y, scenario.rack.z)
        )


def test_working_tools_are_in_their_own_sockets():
    scenario = spec(16)
    parsed = by_name(nodes(16))
    for tool in scenario.tools:
        assert translation(parsed[TOOL_NODE_NAMES[tool.name]]) == pytest.approx(
            (tool.pose.x, tool.pose.y, tool.pose.z)
        )
    positions = {
        translation(parsed[TOOL_NODE_NAMES[name]]) for name in scenario.tool_names
    }
    assert len(positions) == len(scenario.tool_names) == 3
    assert 'grant_torque_tool' not in parsed


def test_experiment_nineteen_shifts_the_tray_and_all_ten_screws():
    standard = by_name(nodes(11))
    shifted = by_name(nodes(19))
    tray = translation(shifted['grant_tray'])
    assert tray == pytest.approx((0.18, 0.48, 0.793))
    for index in range(1, 11):
        name = f'grant_screw_{index:02d}'
        old = translation(standard[name])
        new = translation(shifted[name])
        assert new[0] == pytest.approx(old[0] - 0.02)
        assert new[1] == pytest.approx(old[1] + 0.02)
    assert len({translation(shifted[f'grant_screw_{i:02d}']) for i in range(1, 11)}) == 10


def test_obj_vertical_axis_maps_to_webots_z():
    axis = (0.0, 1.0, 0.0)
    for yaw in (0.0, 45.0, 60.0, 135.0, 180.0):
        matrix = rotation_matrix(yaw)
        x = sum(matrix[0][i] * axis[i] for i in range(3))
        y = sum(matrix[1][i] * axis[i] for i in range(3))
        z = sum(matrix[2][i] * axis[i] for i in range(3))
        assert (x, y, z) == pytest.approx((0.0, 0.0, 1.0), abs=1e-12)
        trace = sum(matrix[i][i] for i in range(3))
        assert math.isclose(trace, math.cos(math.radians(yaw)), abs_tol=1e-12)


def test_cap_rotations_match_45_135_and_60_degrees():
    assert rotation(by_name(nodes(1))['grant_cap']) == pytest.approx(
        rotation_axis_angle(45), abs=1e-9
    )
    assert rotation(by_name(nodes(2))['grant_cap']) == pytest.approx(
        rotation_axis_angle(135), abs=1e-9
    )
    assert rotation(by_name(nodes(19))['grant_cap']) == pytest.approx(
        rotation_axis_angle(60), abs=1e-9
    )
    assert rotation(by_name(nodes(1))['grant_cap'])[3] == pytest.approx(
        1.717771517458, abs=1e-9
    )


def test_only_the_sealant_mesh_is_scaled_by_ten():
    for experiment_id in range(1, 21):
        scenario = spec(experiment_id)
        parsed = by_name(nodes(experiment_id))
        count = sum(node.count('scale 10 10 10') for node in parsed.values())
        assert count == (1 if 'sealant_tool' in scenario.tool_names else 0)
        if 'sealant_tool' in scenario.tool_names:
            assert 'scale 10 10 10' in parsed['grant_sealant_tool']


def test_loose_cap_touches_the_table_and_installed_cap_touches_the_body():
    cap_min, _ = obj_y_bounds(ASSETS['cap'])
    _, body_max = obj_y_bounds(ASSETS['body'])
    for experiment_id in range(1, 21):
        scenario = spec(experiment_id)
        if scenario.cap_state == 'absent':
            continue
        cap = by_name(nodes(experiment_id))['grant_cap']
        cap_z = translation(cap)[2]
        if scenario.cap_state == 'free':
            assert cap_z + cap_min == pytest.approx(0.793, abs=1e-9)
        else:
            body_z = translation(by_name(nodes(experiment_id))['grant_body'])[2]
            assert cap_z + cap_min == pytest.approx(body_z + body_max, abs=1e-9)


def test_free_cap_has_physics_and_installed_cap_is_locked():
    free_cap = by_name(nodes(1))['grant_cap']
    assert 'physics Physics {' in free_cap
    assert 'locked TRUE' not in free_cap
    installed_cap = by_name(nodes(11))['grant_cap']
    assert 'locked TRUE' in installed_cap
    assert 'physics Physics {' not in installed_cap


def test_ten_screws_are_distinct_and_inside_the_tray():
    for experiment_id in (11, 19):
        scenario = spec(experiment_id)
        parsed = by_name(nodes(experiment_id))
        positions = [
            translation(parsed[f'grant_screw_{index:02d}']) for index in range(1, 11)
        ]
        assert len(set(positions)) == 10
        tray = scenario.tray_pose
        assert tray is not None
        for x, y, z in positions:
            assert abs(x - tray.x) < 0.06
            assert abs(y - tray.y) < 0.035
            assert z == pytest.approx(0.7954)


def mirror_assets(tmp_path, missing):
    for source in ASSET_ROOT.iterdir():
        if source.name == missing:
            continue
        (tmp_path / source.name).symlink_to(source)


def test_missing_obj_is_a_hard_error(tmp_path):
    mirror_assets(tmp_path, ASSETS['screw'])
    with pytest.raises(FileNotFoundError, match='vint.obj'):
        build_nodes(spec(11), tmp_path)


def test_missing_mtl_is_a_hard_error(tmp_path):
    for source in ASSET_ROOT.iterdir():
        target = tmp_path / source.name
        if source.name in (ASSETS['screw'], 'vint.mtl'):
            continue
        target.symlink_to(source)
    (tmp_path / ASSETS['screw']).write_text(
        (ASSET_ROOT / ASSETS['screw']).read_text(encoding='utf-8'),
        encoding='utf-8',
    )
    with pytest.raises(FileNotFoundError, match='vint.mtl'):
        build_nodes(spec(11), tmp_path)
