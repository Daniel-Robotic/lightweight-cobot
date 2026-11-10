"""Scene snapshots expose only present ids and logical state."""
from __future__ import annotations

from pathlib import Path

import pytest

from iiwa_experement.scenario import load_scenario
from iiwa_scene_context.catalog import DEFAULT_CATALOG
from iiwa_scene_context.snapshot import build_envelope, build_scene


PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parent
SCENES = ROOT / 'iiwa_experement' / 'config' / 'scenes.yaml'
FORBIDDEN_KEYS = {
    'pose', 'poses', 'xyz', 'rotation', 'tcp', 'objects', 'simulation_time',
    'last_command_result', 'last_error', 'attached', 'version',
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


@pytest.mark.parametrize('experiment_id', [1, 20])
def test_scene_lists_only_present_objects_and_goals(experiment_id):
    spec = spec_for(experiment_id)
    scene = build_scene(spec, state_for(spec), DEFAULT_CATALOG)
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


def test_each_present_tool_has_a_named_socket_and_stable_ids():
    for experiment_id in (1, 20):
        spec = spec_for(experiment_id)
        first = build_scene(spec, state_for(spec), DEFAULT_CATALOG)
        second = build_scene(spec, state_for(spec), DEFAULT_CATALOG)
        assert first['components'] == second['components']
        assert first['destinations'] == second['destinations']
        tool_ids = [tool['id'] for tool in first['tools']]
        assert tool_ids == list(spec.tool_names)
        for tool in first['tools']:
            assert tool['socket'] == f"{tool['id']}_socket"
            assert tool['socket'] in ids(first['destinations'])


def test_scene_json_has_no_coordinates_or_supervisor_internals():
    spec = spec_for(20)
    state = state_for(spec, mounted='vacuum', held='cap', cap={'state': 'held'})
    state['tcp'] = {'xyz': [1, 2, 3]}
    state['objects'] = {'grant_cap': {'xyz': [0, 0, 0]}}
    state['simulation_time'] = 12.5
    scene = build_scene(spec, state, DEFAULT_CATALOG)
    keys = set(walk_keys(scene))
    assert not (keys & FORBIDDEN_KEYS)
    assert 'timestamp' not in keys
    assert scene['experiment_id'] == 20
    assert scene['contract_version'] == 1
    assert scene['catalog_version'] == 1


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


def test_scene_json_carries_server_skill_contracts_not_model_contracts():
    spec = spec_for(1)
    scene = build_scene(spec, state_for(spec), DEFAULT_CATALOG)
    assert 'grab' in scene['available_skills']
    assert 'tighten' not in scene['available_skills']
    assert set(scene['skill_contracts']) == set(scene['available_skills'])
    assert scene['skill_contracts']['grab']['outcomes']['grasp_failed']
    assert 'grasp_failed' in scene['skill_contracts']['pick']['outcomes']


def test_envelope_keeps_transport_separate_from_scene():
    spec = spec_for(20)
    scene = build_scene(spec, state_for(spec), DEFAULT_CATALOG)
    envelope = build_envelope(
        scene, scene_revision=7, state_version=4, stamp=123.5,
    )
    assert envelope['experiment_id'] == 20
    assert envelope['state_version'] == 4
    assert envelope['scene_revision'] == 7
    assert envelope['timestamp'] == 123.5
    assert envelope['scene_json'] == scene
    assert envelope['scene_json'] is not envelope
    assert envelope['scene_json']['experiment_id'] == 20
