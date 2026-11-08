"""The twenty experiments have exactly the composition from the specification."""
import math
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from iiwa_experement.scenario import (
    CAP_STATES,
    TOOL_NAMES,
    ScenarioError,
    load_scenario,
)


PACKAGE = Path(__file__).resolve().parents[1]
SCENES = PACKAGE / 'config' / 'scenes.yaml'

STANDARD_TOOLS = set(TOOL_NAMES)
STANDARD_SCREWS = {
    'screw_1': (0.160, 0.445),
    'screw_2': (0.240, 0.445),
    'screw_3': (0.160, 0.475),
    'screw_4': (0.240, 0.475),
    'screw_5': (0.180, 0.445),
    'screw_6': (0.200, 0.445),
    'screw_7': (0.220, 0.445),
    'screw_8': (0.180, 0.475),
    'screw_9': (0.200, 0.475),
    'screw_10': (0.220, 0.475),
}


def scenario(experiment_id):
    return load_scenario(experiment_id, SCENES)


def variant(tmp_path, mutate):
    data = yaml.safe_load(SCENES.read_text(encoding='utf-8'))
    mutate(data)
    path = tmp_path / 'scenes.yaml'
    path.write_text(yaml.safe_dump(data, sort_keys=False), encoding='utf-8')
    return path


def test_all_twenty_experiments_load():
    for experiment_id in range(1, 21):
        spec = scenario(experiment_id)
        assert spec.experiment_id == experiment_id
        assert spec.cap_state in CAP_STATES
        assert spec.tool_names
        if experiment_id != 3:
            assert (spec.fixture.x, spec.fixture.y) == (0.30, 0.12)
            assert (spec.body.x, spec.body.y) == (0.30, 0.12)
        assert (spec.rack.x, spec.rack.y, spec.rack.z) == (0.12, -0.42, 0.793)


def test_cap_only_experiments_one_to_five():
    for experiment_id in range(1, 6):
        spec = scenario(experiment_id)
        assert spec.cap_state == 'free'
        assert spec.tray_pose is None
        assert spec.screws == ()
        assert spec.tool_names == ('vacuum',)


def test_sealant_experiments_do_not_start_with_a_hatch_or_tray():
    for experiment_id in range(6, 10):
        spec = scenario(experiment_id)
        assert spec.cap_state == 'absent'
        assert spec.cap_pose is None
        assert spec.tray_pose is None
        assert spec.screws == ()
        assert spec.tool_names == ('sealant_tool',)


def test_experiment_ten_has_hatch_and_both_tools():
    spec = scenario(10)
    assert spec.cap_state == 'free'
    assert spec.tray_pose is None
    assert set(spec.tool_names) == {'sealant_tool', 'vacuum'}


def test_screw_experiments_have_installed_hatch_and_ten_screws():
    for experiment_id in range(11, 16):
        spec = scenario(experiment_id)
        assert spec.cap_state == 'installed'
        assert spec.tray_pose is not None
        assert len(spec.screws) == 10
        if experiment_id in (14, 15):
            assert set(spec.tool_names) == {'magnetic_tool', 'torque_tool'}
        else:
            assert spec.tool_names == ('magnetic_tool',)
        assert len(spec.screw_holes) == {
            11: 1, 12: 2, 13: 3, 14: 4, 15: 4,
        }[experiment_id]


def test_full_assembly_experiments_have_loose_hatch_tray_and_all_tools():
    for experiment_id in range(16, 21):
        spec = scenario(experiment_id)
        assert spec.cap_state == 'free'
        assert spec.tray_pose is not None
        assert len(spec.screws) == 10
        assert set(spec.tool_names) == {
            'vacuum', 'magnetic_tool', 'sealant_tool', 'torque_tool',
        }
        assert len(spec.screw_holes) == {16: 1, 17: 2, 18: 4, 19: 4, 20: 4}[experiment_id]


def test_sealant_coverage_matches_each_experiment():
    assert scenario(6).sealant_sides == ('side_1',)
    assert scenario(7).sealant_sides == ('side_1', 'side_2')
    for experiment_id in (8, 9, 10, 17, 18, 19, 20):
        assert scenario(experiment_id).sealant_sides == (
            'side_1', 'side_2', 'side_3', 'side_4',
        )
    assert scenario(16).sealant_sides == ('side_1',)
    for experiment_id in (1, 2, 3, 4, 5, 11, 12, 13, 14, 15):
        assert scenario(experiment_id).sealant_sides == ()


def test_screw_holes_are_unique_bounded_and_ordered():
    for experiment_id in range(11, 21):
        holes = scenario(experiment_id).screw_holes
        assert len(holes) == len(set(holes))
        assert set(holes) <= {'hole_1', 'hole_2', 'hole_3', 'hole_4'}
    assert scenario(14).screw_holes == ('hole_1', 'hole_3', 'hole_2', 'hole_4')
    assert scenario(15).screw_holes == ('hole_1', 'hole_3', 'hole_2', 'hole_4')


def test_reference_experiments_never_request_tightening():
    for experiment_id in range(1, 21):
        spec = scenario(experiment_id)
        assert all(
            failure.skill not in ('tighten', 'seat_screw')
            for failure in spec.failures
        )


def test_standard_screw_layout_matches_the_specification():
    spec = scenario(11)
    assert len({screw.name for screw in spec.screws}) == 10
    assert len({screw.pose.xy for screw in spec.screws}) == 10
    for screw in spec.screws:
        assert screw.pose.xy == pytest.approx(STANDARD_SCREWS[screw.name], abs=1e-9)
        assert screw.pose.z == pytest.approx(0.7954, abs=1e-12)


def test_experiment_two_hatch_is_rotated_135_degrees():
    spec = scenario(2)
    assert spec.cap_pose is not None
    assert spec.cap_pose.xy == (0.35, -0.22)
    assert spec.cap_pose.yaw_deg == 135


def test_experiment_three_moves_fixture_body_and_hatch_together():
    spec = scenario(3)
    assert (spec.fixture.x, spec.fixture.y, spec.fixture.z) == (0.29, 0.14, 0.803)
    assert (spec.body.x, spec.body.y, spec.body.z) == (0.29, 0.14, 0.823)
    assert spec.cap_pose is not None
    assert (spec.cap_pose.x, spec.cap_pose.y) == (0.33, -0.22)


def test_experiment_five_carries_the_misplaced_verification_metadata():
    spec = scenario(5)
    assert spec.cap_pose is not None
    assert spec.cap_pose.xy == (0.35, -0.22)
    assert len(spec.failures) == 1
    failure = spec.failures[0]
    assert (failure.skill, failure.outcome, failure.target, failure.attempt) == (
        'verify', 'not_ok', 'cap', 1,
    )
    assert dict(failure.details)['after_first_install_x'] == 0.303


def test_experiment_nineteen_moves_hatch_and_tray_with_all_screws():
    spec = scenario(19)
    assert spec.cap_pose is not None
    assert spec.cap_pose.xy == (0.37, -0.20)
    assert spec.cap_pose.yaw_deg == 60
    assert spec.tray_pose is not None
    assert (spec.tray_pose.x, spec.tray_pose.y, spec.tray_pose.z) == (0.18, 0.48, 0.793)
    for screw in spec.screws:
        expected = STANDARD_SCREWS[screw.name]
        assert screw.pose.xy == pytest.approx(
            (expected[0] - 0.02, expected[1] + 0.02), abs=1e-9
        )


def test_failure_metadata_for_experiments_four_nine_fifteen_twenty():
    assert [(f.skill, f.outcome) for f in scenario(4).failures] == [
        ('grab', 'grasp_failed'),
    ]
    assert [(f.skill, f.outcome, f.target) for f in scenario(9).failures] == [
        ('dispense', 'interrupted', 'side_3'),
    ]
    fifteen = [(f.skill, f.outcome, f.target) for f in scenario(15).failures]
    assert fifteen == [
        ('grab', 'grasp_failed', 'third_selected_screw'),
    ]
    twenty = [(f.skill, f.outcome, f.target) for f in scenario(20).failures]
    assert twenty == [
        ('dispense', 'interrupted', 'side_3'),
        ('grab', 'grasp_failed', 'first_selected_screw'),
        ('verify', 'not_ok', 'cap'),
    ]


def test_installed_cap_sits_on_the_body():
    for experiment_id in range(11, 16):
        spec = scenario(experiment_id)
        assert spec.cap_pose is not None
        assert spec.cap_pose.xy == spec.body.xy
        assert spec.cap_pose.yaw_deg == spec.body.yaw_deg


def test_scenarios_are_read_from_the_installed_manifest():
    installed = (
        Path(get_package_share_directory('iiwa_experement')) / 'config' / 'scenes.yaml'
    )
    for experiment_id in range(1, 21):
        assert load_scenario(experiment_id, installed).experiment_id == experiment_id


@pytest.mark.parametrize('experiment_id', [0, 21, '1', 1.0, True, None])
def test_invalid_experiment_ids_are_rejected(experiment_id):
    with pytest.raises(ScenarioError):
        load_scenario(experiment_id, SCENES)


def test_unknown_tool_is_rejected(tmp_path):
    def mutate(data):
        data['experiments']['1']['tools'] = ['vacuum_cleaner']

    with pytest.raises(ScenarioError, match='unknown tool'):
        load_scenario(1, variant(tmp_path, mutate))


@pytest.mark.parametrize('count', [9, 11])
def test_incorrect_screw_count_is_rejected(tmp_path, count):
    entries = [
        {'name': f'screw_{index}', 'dx': -0.04 + index * 0.005, 'dy': 0.0}
        for index in range(1, count + 1)
    ]

    def mutate(data):
        data['experiments']['1']['tray'] = True
        data['experiments']['1']['tools'] = ['magnetic_tool']
        data['experiments']['1']['screw_holes'] = ['hole_1']
        data['experiments']['1']['screws'] = entries

    with pytest.raises(ScenarioError, match='exactly 10 screws'):
        load_scenario(1, variant(tmp_path, mutate))


def test_duplicate_screw_name_is_rejected(tmp_path):
    entries = [
        {'name': 'screw_1', 'dx': -0.04 + index * 0.005, 'dy': 0.0}
        for index in range(10)
    ]

    def mutate(data):
        data['experiments']['1']['tray'] = True
        data['experiments']['1']['tools'] = ['magnetic_tool']
        data['experiments']['1']['screw_holes'] = ['hole_1']
        data['experiments']['1']['screws'] = entries

    with pytest.raises(ScenarioError, match='duplicate screw name'):
        load_scenario(1, variant(tmp_path, mutate))


def test_screw_outside_the_tray_is_rejected(tmp_path):
    entries = [
        {'name': f'screw_{index}', 'dx': -0.04 + index * 0.005, 'dy': 0.0}
        for index in range(10)
    ]
    entries[4]['dx'] = 0.2

    def mutate(data):
        data['experiments']['1']['tray'] = True
        data['experiments']['1']['tools'] = ['magnetic_tool']
        data['experiments']['1']['screw_holes'] = ['hole_1']
        data['experiments']['1']['screws'] = entries

    with pytest.raises(ScenarioError, match='outside the tray'):
        load_scenario(1, variant(tmp_path, mutate))


def test_fixture_and_body_must_share_xy(tmp_path):
    def mutate(data):
        data['experiments']['1']['body'] = {
            'x': 0.31, 'y': 0.12, 'z': 0.823, 'yaw_deg': 45,
        }

    with pytest.raises(ScenarioError, match='fixture and body'):
        load_scenario(1, variant(tmp_path, mutate))


def test_tray_without_screws_and_screws_without_tray_are_rejected(tmp_path):
    def no_screws(data):
        data['experiments']['1']['screws'] = [
            {'name': f'screw_{index}', 'dx': 0.0, 'dy': 0.0}
            for index in range(10)
        ]

    with pytest.raises(ScenarioError, match='no tray'):
        load_scenario(1, variant(tmp_path, no_screws))

    def no_tray(data):
        data['experiments']['1']['tray'] = False
        data['experiments']['1']['tray_pose'] = {
            'x': -0.08, 'y': 0.34, 'z': 0.793, 'yaw_deg': 0,
        }

    with pytest.raises(ScenarioError, match='tray_pose requires tray'):
        load_scenario(1, variant(tmp_path, no_tray))


def test_missing_manifest_is_reported(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_scenario(1, tmp_path / 'missing.yaml')


def test_invalid_sealant_side_is_rejected(tmp_path):
    def mutate(data):
        data['experiments']['6']['sealant'] = ['side_5']

    with pytest.raises(ScenarioError, match='sealant'):
        load_scenario(6, variant(tmp_path, mutate))


def test_invalid_screw_hole_is_rejected(tmp_path):
    def mutate(data):
        data['experiments']['11']['screw_holes'] = ['hole_5']

    with pytest.raises(ScenarioError, match='screw_holes'):
        load_scenario(11, variant(tmp_path, mutate))


def test_tray_without_screw_holes_is_rejected(tmp_path):
    def mutate(data):
        data['experiments']['11'].pop('screw_holes')

    with pytest.raises(ScenarioError, match='requires screw_holes'):
        load_scenario(11, variant(tmp_path, mutate))
