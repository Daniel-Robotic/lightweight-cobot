"""External graph-v3 context: scene/catalog adaptation and version checks."""
from __future__ import annotations

from pathlib import Path

import pytest

from iiwa_experement.geometry import load_geometry
from iiwa_experement.scenario import load_scenario
from iiwa_scene_context.catalog import DEFAULT_CATALOG
from iiwa_scene_context.external import (
    ContextError,
    build_external_catalog,
    build_external_scene,
    build_planner_context,
)
from iiwa_scene_context.snapshot import build_envelope, build_scene


PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parent
SCENES = ROOT / 'iiwa_experement' / 'config' / 'scenes.yaml'
GEOMETRY = load_geometry(ROOT / 'iiwa_experement' / 'config' / 'geometry.yaml')


def make_envelope(experiment_id=20, version=7):
    spec = load_scenario(experiment_id, SCENES)
    tools = {tool.name: {'state': 'docked'} for tool in spec.tools}
    scene = build_scene(
        spec,
        {
            'version': version,
            'mounted_tool': None,
            'held_object': None,
            'ready': True,
            'cap': {'state': spec.cap_state},
            'tools': tools,
            'screws': {},
            'holes': {},
        },
        DEFAULT_CATALOG,
        GEOMETRY,
        stamp=12.0,
    )
    return build_envelope(
        scene,
        state_version=version,
        stamp=12.0,
        experiment_id=experiment_id,
    )


def test_context_bundle_is_one_consistent_graph_v3_snapshot():
    envelope = make_envelope()
    context = build_planner_context(envelope)
    assert context['schema_version'] == 3
    assert context['contract_version'] == 3
    assert context['catalog_version'] == 2
    assert context['state_version'] == 7
    assert context['scene_schema_version'] == 1
    assert context['scene']['schema_version'] == 1
    assert context['scene']['experiment_id'] == 20
    assert context['response_schema']['$defs']['ReadyResponse']
    assert context['scene']['objects']
    assert context['scene']['locations']
    assert context['scene']['tools']
    assert 'graph-v3' in context['system_prompt']
    assert 'installed' in context['system_prompt']
    assert context['catalog']['skills']['grab']['events'] == ['object_unavailable']


def test_external_scene_lists_only_present_ids_and_exact_types():
    scene = build_external_scene(make_envelope()['scene_json'])
    objects = {entry['id']: entry for entry in scene['objects']}
    locations = {entry['id']: entry for entry in scene['locations']}
    tools = {entry['id']: entry for entry in scene['tools']}
    assert objects['cap']['type'] == 'part'
    assert objects['fixture']['type'] == 'fixture'
    assert locations['hatch_seat']['type'] == 'seat'
    assert locations['hole_1']['type'] == 'hole'
    assert tools['vacuum']['type'] == 'suction'
    assert tools['magnetic_tool']['type'] == 'magnet'
    assert tools['vacuum']['socket'] == 'vacuum_socket'
    assert 'cap' in tools['vacuum']['compatible_objects']
    assert not tools['torque_tool']['compatible_objects']


def test_catalog_advertises_only_supported_predicates_and_events():
    catalog = build_external_catalog(make_envelope()['scene_json'])
    assert set(catalog['predicates']) >= {
        'installed', 'handoff', 'tool_docked', 'held', 'tool_mounted', 'at',
        'visible',
    }
    assert 'dispensed' not in catalog['predicates']
    assert set(catalog['observations']) >= {'installed', 'hole_empty'}
    assert 'inspect_state' not in catalog['skills']
    assert set(catalog['skills']['grab']['events']) == {'object_unavailable'}


def test_unsupported_scene_contract_is_rejected():
    envelope = make_envelope()
    envelope['contract_version'] = 1
    with pytest.raises(ContextError) as error:
        build_planner_context(envelope)
    assert error.value.code == 'unsupported_contract'

    envelope = make_envelope()
    envelope['scene_json']['schema_version'] = 1
    with pytest.raises(ContextError) as error:
        build_planner_context(envelope)
    assert error.value.code == 'unsupported_contract'
