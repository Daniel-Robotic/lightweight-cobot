"""Scene snapshots expose only present ids, live poses and action rules."""
from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from iiwa_experement.geometry import load_geometry
from iiwa_experement.scenario import load_scenario
from iiwa_scene_context.catalog import DEFAULT_CATALOG
from iiwa_scene_context.snapshot import build_envelope, build_scene


PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parent
SCENES = ROOT / 'iiwa_experement' / 'config' / 'scenes.yaml'
GEOMETRY = load_geometry(ROOT / 'iiwa_experement' / 'config' / 'geometry.yaml')
FORBIDDEN_KEYS = {
    'simulation_time', 'last_command_result', 'last_error',
    'attached', 'scene_revision',
}


def spec_for(experiment_id: int):
    return load_scenario(experiment_id, SCENES)


def state_for(spec, *, version=3, mounted=None, held=None, cap=None,
              tool_states=None, screws=None, holes=None):
    cap_entry = cap if cap is not None else {'state': spec.cap_state}
    tools = {}
    for tool in spec.tools:
        tools[tool.name] = {
            'state': (tool_states or {}).get(tool.name, 'docked'),
        }
    screw_entries = {}
    for screw in spec.screws:
        screw_entries[screw.name] = {
            'state': (screws or {}).get(screw.name, 'tray'),
            'hole': None,
        }
    hole_entries = holes or {
        name: {'state': 'empty', 'screw': None}
        for name in ('hole_1', 'hole_2', 'hole_3', 'hole_4')
    }
    return {
        'version': version,
        'experiment_id': spec.experiment_id,
        'mounted_tool': mounted,
        'held_object': held,
        'ready': True,
        'cap': cap_entry,
        'tools': tools,
        'screws': screw_entries,
        'holes': hole_entries,
        'simulation_time': 12.5,
    }


def ids(entries):
    return [entry['id'] for entry in entries]


def walk_keys(value):
    if isinstance(value, dict):
        for key, nested in value.items():
            yield key
            yield from walk_keys(nested)
    elif isinstance(value, list):
        for item in value:
            yield from walk_keys(item)


def quaternion_norm(orientation):
    return math.sqrt(sum(
        float(orientation[axis]) ** 2 for axis in ('x', 'y', 'z', 'w')
    ))


@pytest.mark.parametrize('experiment_id', [1, 20])
def test_scene_lists_only_present_objects_and_goals(experiment_id):
    spec = spec_for(experiment_id)
    scene = build_scene(spec, state_for(spec), DEFAULT_CATALOG, GEOMETRY)
    components = ids(scene['components'])
    destinations = ids(scene['destinations'])
    assert 'fixture' in components and 'body' in components
    assert ('cap' in components) == (spec.cap_state != 'absent')
    assert ('tray' in components) == (spec.tray_pose is not None)
    assert all(f'screw_{index}' in components for index in range(1, 11)) == (
        spec.tray_pose is not None
    )
    assert 'work' in destinations
    assert 'hatch_seat' in destinations
    assert {'hole_1', 'hole_2', 'hole_3', 'hole_4'} <= set(destinations)
    for tool in spec.tools:
        assert f'{tool.name}_socket' in destinations
    assert set(ids(scene['tools'])) == set(spec.tool_names)
    for side in spec.sealant_sides:
        assert side in destinations
    for index in range(1, 5):
        if f'side_{index}' not in spec.sealant_sides:
            assert f'side_{index}' not in destinations
    if spec.tray_pose is None:
        assert not any(name.startswith('screw_') for name in components)
    else:
        assert len([name for name in components if name.startswith('screw_')]) >= 10


@pytest.mark.parametrize('experiment_id', range(1, 21))
def test_every_experiment_scene_matches_the_created_scene(experiment_id):
    spec = spec_for(experiment_id)
    scene = build_scene(spec, state_for(spec), DEFAULT_CATALOG, GEOMETRY)
    expected_components = {'fixture', 'body'}
    if spec.cap_state != 'absent':
        expected_components.add('cap')
    if spec.tray_pose is not None:
        expected_components.add('tray')
        expected_components.update(
            f'screw_{index}' for index in range(1, 11)
        )
    assert set(ids(scene['components'])) == expected_components
    assert set(ids(scene['tools'])) == set(spec.tool_names)
    assert set(ids(scene['destinations'])) == {
        'work', 'hatch_seat', 'hole_1', 'hole_2', 'hole_3', 'hole_4',
        *[f'{tool.name}_socket' for tool in spec.tools],
        *spec.sealant_sides,
    }
    assert set(scene['available_skills']) == set(scene['action_rules'])
    for rule in scene['action_rules'].values():
        assert set(rule['objects']) <= set(scene['service_objects']) | set(
            ids(scene['components'])
        ) | set(ids(scene['tools'])) | set(scene['cameras'])
        assert set(rule['targets']) <= set(ids(scene['destinations'])) | set(
            ids(scene['components'])
        ) | set(ids(scene['tools']))
    if 'dispense' in scene['available_skills']:
        assert scene['action_rules']['dispense']['targets'] == list(
            spec.sealant_sides
        )
    else:
        assert not spec.sealant_sides


def test_each_present_tool_has_a_named_socket_and_stable_ids():
    for experiment_id in (1, 20):
        spec = spec_for(experiment_id)
        first = build_scene(spec, state_for(spec), DEFAULT_CATALOG, GEOMETRY)
        second = build_scene(spec, state_for(spec), DEFAULT_CATALOG, GEOMETRY)
        assert first['components'] == second['components']
        assert first['destinations'] == second['destinations']
        tool_ids = [tool['id'] for tool in first['tools']]
        assert tool_ids == list(spec.tool_names)
        for tool in first['tools']:
            assert tool['socket'] == f"{tool['id']}_socket"
            assert tool['socket'] in ids(first['destinations'])


def test_scene_json_has_live_poses_and_no_supervisor_internals():
    spec = spec_for(20)
    state = state_for(spec, mounted='vacuum', held='cap', cap={'state': 'held'})
    scene = build_scene(spec, state, DEFAULT_CATALOG, GEOMETRY)
    keys = set(walk_keys(scene))
    assert not (keys & FORBIDDEN_KEYS)
    assert 'timestamp' not in keys
    assert scene['experiment_id'] == 20
    assert scene['contract_version'] == 2
    assert scene['catalog_version'] == 2
    assert scene['schema_version'] == 2
    assert scene['pose_conventions'] == {
        'frame_id': 'world',
        'length_unit': 'm',
        'orientation': 'quaternion_xyzw',
    }

    for entry in scene['components'] + scene['tools']:
        pose = entry['pose']
        assert pose['frame_id'] == 'world'
        assert pose['stamp'] == 12.5
        assert pose['source'] in ('webots_supervisor', 'scenario_manifest')
        assert abs(quaternion_norm(pose['orientation']) - 1.0) < 1e-9
        assert all(math.isfinite(value) for value in pose['position'].values())

    for entry in scene['destinations']:
        if entry['kind'] == 'contour':
            assert entry['start']['frame_id'] == 'world'
            assert entry['end']['frame_id'] == 'world'
            assert entry['start_tcp']['frame_id'] == 'world'
            assert entry['end_tcp']['frame_id'] == 'world'
        else:
            assert entry['pose']['frame_id'] == 'world'
            assert entry['tcp_pose']['frame_id'] == 'world'


def test_live_supervisor_pose_overrides_the_manifest_pose():
    spec = spec_for(20)
    state = state_for(spec)
    state['objects'] = {
        'grant_body': {
            'xyz': [0.31, 0.13, 0.82],
            'rotation': [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        },
    }
    scene = build_scene(spec, state, DEFAULT_CATALOG, GEOMETRY)
    body = next(entry for entry in scene['components'] if entry['id'] == 'body')
    assert body['pose']['position'] == {'x': 0.31, 'y': 0.13, 'z': 0.82}
    assert body['pose']['source'] == 'webots_supervisor'


def test_held_object_pose_is_updated_and_marked_live():
    spec = spec_for(20)
    state = state_for(spec, mounted='vacuum', held='cap', cap={
        'state': 'held',
        'pose': {
            'xyz': [0.4, -0.2, 0.9],
            'rotation': [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        },
    })
    scene = build_scene(spec, state, DEFAULT_CATALOG, GEOMETRY)
    cap = next(entry for entry in scene['components'] if entry['id'] == 'cap')
    assert cap['pose']['position'] == {'x': 0.4, 'y': -0.2, 'z': 0.9}
    assert cap['pose']['source'] == 'webots_supervisor'


def test_logical_state_reflects_the_live_supervisor_snapshot():
    spec = spec_for(20)
    scene = build_scene(
        spec,
        state_for(
            spec,
            mounted='vacuum',
            held='cap',
            cap={'state': 'held'},
            screws={'screw_1': 'delivered'},
            holes={'hole_1': {'state': 'empty', 'screw': 'screw_1'}},
        ),
        DEFAULT_CATALOG,
        GEOMETRY,
    )
    initial = scene['initial_state']
    assert initial['tool_mounted'] == 'vacuum'
    assert initial['held_object'] == 'cap'
    assert initial['tools']['vacuum'] == 'docked'
    assert initial['components']['cap'] == 'held'
    assert initial['components']['screw_1'] == 'delivered'
    assert initial['holes']['hole_1']['screw'] == 'screw_1'
    assert 'tool_mounted:vacuum' in initial['flags']
    assert 'holding:cap' in initial['flags']


def test_scene_json_carries_server_skill_contracts_and_scene_rules():
    spec = spec_for(20)
    scene = build_scene(spec, state_for(spec), DEFAULT_CATALOG, GEOMETRY)
    assert 'grab' in scene['available_skills']
    assert 'tighten' not in scene['available_skills']
    assert set(scene['skill_contracts']) == set(scene['available_skills'])
    assert set(scene['action_rules']) == set(scene['available_skills'])
    assert scene['skill_contracts']['grab']['outcomes']['grasp_failed']
    assert 'grasp_failed' in scene['skill_contracts']['pick']['outcomes']

    grab = scene['action_rules']['grab']
    assert grab['required_tool_by_kind']['hatch'] == 'vacuum'
    assert grab['required_tool_by_kind']['screw'] == 'magnetic_tool'
    assert set(grab['objects']) == {
        'cap',
        *[f'screw_{index}' for index in range(1, 11)],
        'vacuum', 'magnetic_tool', 'sealant_tool', 'torque_tool',
    }

    dispense = scene['action_rules']['dispense']
    assert dispense['required_tools'] == ['sealant_tool']
    assert dispense['targets'] == ['side_1', 'side_2', 'side_3', 'side_4']

    dock = scene['action_rules']['dock_tool']
    assert 'transfer(<tool>, <tool>_socket) precedes dock_tool' in (
        dock['preconditions']
    )
    assert 'handoff_screw' in scene['action_rules']
    assert 'no insertion or tightening' in (
        scene['action_rules']['handoff_screw']['result']
    )
    assert 'sealant_tool' not in scene['action_rules']['move_to_storage']['objects']
    assert 'hatch_seat' in scene['action_rules']['move_to_storage']['forbidden_targets']


def test_smaller_scene_does_not_advertise_unavailable_actions():
    spec = spec_for(1)
    scene = build_scene(spec, state_for(spec), DEFAULT_CATALOG, GEOMETRY)
    assert 'dispense' not in scene['available_skills']
    assert 'handoff_screw' not in scene['available_skills']
    assert 'dispense' not in scene['action_rules']
    assert 'side_1' not in ids(scene['destinations'])
    assert not any(
        entry['id'].startswith('screw_') for entry in scene['components']
    )


def test_experiment_20_example_is_the_generated_live_snapshot():
    example_path = PACKAGE / 'docs' / 'scene_json_experiment_20.json'
    example = json.loads(example_path.read_text(encoding='utf-8'))
    spec = spec_for(20)
    generated = build_scene(
        spec,
        state_for(spec),
        DEFAULT_CATALOG,
        GEOMETRY,
        stamp=42.0,
    )
    assert example == generated
    assert example['experiment_id'] == 20
    assert len(example['components']) == 14
    assert len(example['tools']) == 4
    assert len(example['destinations']) == 14
    assert [entry['id'] for entry in example['destinations']].count(
        'side_1'
    ) == 1


def test_envelope_keeps_transport_separate_from_scene():
    spec = spec_for(20)
    scene = build_scene(spec, state_for(spec), DEFAULT_CATALOG, GEOMETRY)
    envelope = build_envelope(
        scene, state_version=4, stamp=123.5,
    )
    assert envelope['experiment_id'] == 20
    assert envelope['state_version'] == 4
    assert 'scene_revision' not in envelope
    assert envelope['timestamp'] == 123.5
    assert envelope['scene_json'] == scene
    assert envelope['scene_json'] is not envelope
    assert envelope['scene_json']['experiment_id'] == 20
