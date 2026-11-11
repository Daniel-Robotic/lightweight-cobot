"""Graph admission: structure, safe branches, ownership and scene revision."""
from __future__ import annotations

from pathlib import Path

import pytest

from iiwa_experement.scenario import load_scenario
from iiwa_llm_plan_executor.schema import PlanSchemaError, parse_response
from iiwa_llm_plan_executor.validator import validate_graph
from iiwa_scene_context.catalog import DEFAULT_CATALOG
from iiwa_scene_context.snapshot import build_envelope, build_scene


PACKAGE = Path(__file__).resolve().parents[1]
SCENES = PACKAGE.parent / 'iiwa_experement' / 'config' / 'scenes.yaml'
SAFE_DEFAULTS = {
    'collision': 'operator_required',
    'grasp_failed': 'operator_required',
    'human_in_scene': 'operator_required',
    'not_found': 'replan_required',
    'failed': 'replan_required',
    'not_ok': 'operator_required',
    'timeout': 'operator_required',
    'unknown': 'operator_required',
}
PROGRESS = {'success', 'ok', 'found', 'clear', 'empty', 'holding', 'placed'}


def spec_for(experiment_id: int = 1):
    return load_scenario(experiment_id, SCENES)


def scene_for(experiment_id: int = 1, *, mounted=None, held=None,
              cap_state=None, revision=1):
    spec = spec_for(experiment_id)
    tools = {tool.name: {'state': 'docked'} for tool in spec.tools}
    if mounted:
        tools[mounted]['state'] = 'mounted'
    scene = build_scene(
        spec,
        {
            'version': 1,
            'mounted_tool': mounted,
            'held_object': held,
            'ready': True,
            'cap': {'state': cap_state or spec.cap_state},
            'tools': tools,
            'screws': {},
            'holes': {},
        },
        DEFAULT_CATALOG,
    )
    return build_envelope(
        scene,
        scene_revision=revision,
        state_version=1,
        stamp=10.0,
        experiment_id=experiment_id,
    )


def node(node_id, skill, obj='', target='', *, next_on=None, params=None,
         outcomes=None):
    events = outcomes or DEFAULT_CATALOG.outcomes(skill)
    next_on = next_on or {}
    transitions = []
    for event in events:
        if event in next_on:
            destination = next_on[event]
        elif event in PROGRESS:
            destination = 'completed'
        else:
            destination = SAFE_DEFAULTS.get(event, 'operator_required')
        transitions.append({'event': event, 'next': destination})
    entry = {
        'id': node_id,
        'skill': skill,
        'object': obj,
        'target': target,
        'transitions': transitions,
    }
    if params is not None:
        entry['params'] = params
    return entry


def graph(entry_id, *nodes):
    return parse_response({
        'status': 'ready',
        'entry': entry_id,
        'plan': list(nodes),
    })


def expect(graph_value, scene, *, expected_revision=None):
    return validate_graph(
        graph_value,
        scene,
        DEFAULT_CATALOG,
        expected_scene_revision=expected_revision,
    )


def test_blocked_status_never_reaches_execution():
    result = expect(
        parse_response({'status': 'blocked', 'entry': '', 'plan': []}),
        scene_for(),
    )
    assert result.ok is False
    assert result.code == 'blocked'


def test_cap_grab_without_vacuum_is_admitted_for_runtime_grasp_failed():
    value = graph(
        'grab_cap',
        node('grab_cap', 'grab', 'cap', next_on={
            'success': 'completed',
            'grasp_failed': 'operator_required',
        }),
    )
    result = expect(value, scene_for(1))
    assert result.ok is True

    # The success branch is kept in the graph but is not simulated as if the
    # hatch were already held without a vacuum tool.
    with_success_branch = graph(
        'grab_cap',
        node('grab_cap', 'grab', 'cap', next_on={
            'success': 'transfer',
            'grasp_failed': 'operator_required',
        }),
        node('transfer', 'transfer', 'cap', 'hatch_seat',
             next_on={'success': 'completed'}),
    )
    assert expect(with_success_branch, scene_for(1)).ok is True


def test_screw_grab_without_magnetic_tool_is_admitted():
    value = graph(
        'pick_screw',
        node('pick_screw', 'pick', 'screw_1', next_on={
            'success': 'completed',
            'grasp_failed': 'operator_required',
        }),
    )
    result = expect(value, scene_for(20))
    assert result.ok is True


def test_recovery_branch_that_assumes_a_held_hatch_has_its_own_reason():
    value = graph(
        'grab_cap',
        node('grab_cap', 'grab', 'cap', next_on={
            'success': 'completed',
            'grasp_failed': 'inspect',
        }),
        node('inspect', 'inspect_state', 'cap', next_on={
            'empty': 'transfer',
            'holding': 'transfer',
            'placed': 'completed',
            'unknown': 'operator_required',
        }),
        node('transfer', 'transfer', 'cap', 'hatch_seat',
             next_on={'success': 'completed'}),
    )
    result = expect(value, scene_for(1))
    assert result.ok is False
    assert result.code == 'replan_required'
    assert 'hatch is not held' in result.reason
    assert 'vacuum' not in result.reason


def test_take_vacuum_and_leave_it_in_patron_is_accepted():
    value = graph(
        'grab_vacuum',
        node('grab_vacuum', 'grab', 'vacuum',
             next_on={'success': 'transfer'}),
        node('transfer', 'transfer', 'vacuum', 'work',
             next_on={'success': 'completed'}),
    )
    result = expect(value, scene_for(1))
    assert result.ok is True


def test_release_from_work_without_a_socket_is_rejected():
    value = graph(
        'grab_vacuum',
        node('grab_vacuum', 'grab', 'vacuum',
             next_on={'success': 'transfer'}),
        node('transfer', 'transfer', 'vacuum', 'work',
             next_on={'success': 'release'}),
        node('release', 'release', 'vacuum', '',
             next_on={'success': 'completed'}),
    )
    result = expect(value, scene_for(1))
    assert result.ok is False
    assert result.code == 'replan_required'


def test_release_at_socket_requires_a_preceding_explicit_transfer():
    value = graph(
        'grab_vacuum',
        node('grab_vacuum', 'grab', 'vacuum',
             next_on={'success': 'transfer'}),
        node('transfer', 'transfer', 'vacuum', 'work',
             next_on={'success': 'release'}),
        node('release', 'release', 'vacuum', 'vacuum_socket',
             next_on={'success': 'completed'}),
    )
    result = expect(value, scene_for(1))
    assert result.code == 'replan_required'
    assert 'transfer' in result.reason


def test_explicit_transfer_then_dock_or_release_returns_the_tool():
    dock = graph(
        'grab_vacuum',
        node('grab_vacuum', 'grab', 'vacuum',
             next_on={'success': 'transfer'}),
        node('transfer', 'transfer', 'vacuum', 'vacuum_socket',
             next_on={'success': 'dock'}),
        node('dock', 'dock_tool', 'vacuum', 'vacuum_socket',
             next_on={'success': 'completed'}),
    )
    assert expect(dock, scene_for(1)).ok is True

    release = graph(
        'grab_vacuum',
        node('grab_vacuum', 'grab', 'vacuum',
             next_on={'success': 'transfer'}),
        node('transfer', 'transfer', 'vacuum', 'vacuum_socket',
             next_on={'success': 'release'}),
        node('release', 'release', 'vacuum', 'vacuum_socket',
             next_on={'success': 'completed'}),
    )
    assert expect(release, scene_for(1)).ok is True


def test_dock_tool_without_the_explicit_transfer_is_rejected():
    value = graph(
        'grab_vacuum',
        node('grab_vacuum', 'grab', 'vacuum',
             next_on={'success': 'dock'}),
        node('dock', 'dock_tool', 'vacuum', 'vacuum_socket',
             next_on={'success': 'completed'}),
    )
    assert expect(value, scene_for(1)).code == 'replan_required'


def test_collision_must_not_continue_the_operation():
    value = graph(
        'approach',
        node('approach', 'approach', 'cap',
             next_on={'success': 'completed', 'collision': 'completed'}),
    )
    result = expect(value, scene_for(1))
    assert result.ok is False
    assert 'collision' in result.reason


def test_grasp_failed_cannot_map_to_success():
    value = graph(
        'grab_vacuum',
        node('grab_vacuum', 'grab', 'vacuum',
             next_on={'success': 'completed', 'grasp_failed': 'completed'}),
    )
    result = expect(value, scene_for(1))
    assert result.ok is False
    assert 'grasp_failed' in result.reason


def test_structural_errors_are_rejected():
    scene = scene_for(1)
    unknown_skill = {
        'status': 'ready', 'entry': 'a',
        'plan': [{
            'id': 'a', 'skill': 'approval', 'object': 'cap', 'target': '',
            'transitions': [
                {'event': 'success', 'next': 'completed'},
                {'event': 'failed', 'next': 'replan_required'},
            ],
        }],
    }
    assert expect(parse_response(unknown_skill), scene).ok is False
    assert expect(
        graph('a', node('a', 'detect', 'ghost')), scene,
    ).ok is False
    assert expect(
        graph('a', node('a', 'detect', 'cap', 'ghost')), scene,
    ).ok is False
    assert expect(
        graph('missing', node('a', 'detect', 'cap')), scene,
    ).ok is False
    unreachable = graph(
        'a',
        node('a', 'detect', 'cap'),
        node('b', 'detect', 'cap'),
    )
    assert expect(unreachable, scene).ok is False
    no_events = {
        'status': 'ready', 'entry': 'a',
        'plan': [{
            'id': 'a', 'skill': 'detect', 'object': 'cap', 'target': '',
            'transitions': [],
        }],
    }
    assert expect(parse_response(no_events), scene).ok is False
    missing_event = {
        'status': 'ready', 'entry': 'a',
        'plan': [{
            'id': 'a', 'skill': 'detect', 'object': 'cap', 'target': '',
            'transitions': [{'event': 'found', 'next': 'completed'}],
        }],
    }
    assert expect(parse_response(missing_event), scene).ok is False
    bad_next = {
        'status': 'ready', 'entry': 'a',
        'plan': [node('a', 'detect', 'cap',
                      next_on={'found': 'ghost'})],
    }
    assert expect(parse_response(bad_next), scene).ok is False


def test_unknown_tighten_is_rejected_before_motion():
    payload = {
        'status': 'ready', 'entry': 'a',
        'plan': [{
            'id': 'a', 'skill': 'tighten', 'object': 'hole_1', 'target': '',
            'transitions': [
                {'event': 'success', 'next': 'completed'},
                {'event': 'failed', 'next': 'replan_required'},
            ],
        }],
    }
    result = expect(parse_response(payload), scene_for(1))
    assert result.ok is False
    assert 'unavailable skill' in result.reason


def test_cycles_are_bounded_and_must_keep_a_recovery_branch():
    dead_cycle = graph(
        'a',
        node('a', 'wait_scene_clear', 'workspace',
             next_on={'clear': 'b', 'timeout': 'b', 'failed': 'b'}),
        node('b', 'wait_scene_clear', 'workspace',
             next_on={'clear': 'a', 'timeout': 'a', 'failed': 'a'}),
    )
    assert expect(dead_cycle, scene_for(1)).ok is False

    recovering_cycle = graph(
        'a',
        node('a', 'wait_scene_clear', 'workspace',
             next_on={'clear': 'b'}),
        node('b', 'wait_scene_clear', 'workspace',
             next_on={'clear': 'a'}),
    )
    assert expect(recovering_cycle, scene_for(1)).ok is True


def test_wrong_scene_revision_is_rejected():
    value = graph('a', node('a', 'detect', 'cap'))
    result = expect(value, scene_for(1, revision=3), expected_revision=2)
    assert result.ok is False
    assert result.code == 'stale_scene'


def test_aliases_for_pick_move_to_storage_and_inspect_are_supported():
    value = graph(
        'grab_vacuum',
        node('grab_vacuum', 'grab', 'vacuum',
             next_on={'success': 'pick_cap'}),
        node('pick_cap', 'pick', 'cap', next_on={'success': 'transfer'}),
        node('transfer', 'transfer', 'cap', 'hatch_seat',
             next_on={'success': 'release'}),
        node('release', 'release', 'cap', 'hatch_seat',
             next_on={'success': 'inspect'}),
        node('inspect', 'inspect', 'cap', 'hatch_seat',
             next_on={'ok': 'completed'}),
    )
    assert expect(value, scene_for(1)).ok is True

    bad = graph('move', node(
        'move', 'move_to_storage', 'body', 'work',
        next_on={'success': 'completed'},
    ))
    assert expect(bad, scene_for(1)).ok is False


def test_screw_handoff_is_to_the_operator_and_leaves_the_hole_empty():
    value = graph(
        'grab_magnet',
        node('grab_magnet', 'grab', 'magnetic_tool',
             next_on={'success': 'pick_screw'}),
        node('pick_screw', 'pick', 'screw_1',
             next_on={'success': 'transfer'}),
        node('transfer', 'transfer', 'screw_1', 'hole_1',
             next_on={'success': 'handoff'}),
        node('handoff', 'handoff_screw', 'screw_1', 'hole_1',
             next_on={'success': 'completed'}),
    )
    result = expect(value, scene_for(20))
    assert result.ok is True


def test_dispense_requires_the_sealant_tool_at_runtime():
    without = graph(
        'dispense', node('dispense', 'dispense', 'side_1',
                         next_on={'success': 'completed'}),
    )
    assert expect(without, scene_for(20)).ok is True
    with_tool = graph(
        'grab_sealant',
        node('grab_sealant', 'grab', 'sealant_tool',
             next_on={'success': 'dispense'}),
        node('dispense', 'dispense', 'side_1',
             next_on={'success': 'completed'}),
    )
    assert expect(with_tool, scene_for(20)).ok is True


def test_params_limit_dispense_fractions():
    value = graph(
        'grab_sealant',
        node('grab_sealant', 'grab', 'sealant_tool',
             next_on={'success': 'dispense'}),
        node('dispense', 'dispense', 'side_1',
             next_on={'success': 'completed'},
             params={'start_fraction': 0.0, 'end_fraction': 1.0}),
    )
    assert expect(value, scene_for(20)).ok is True

    payload = {
        'status': 'ready', 'entry': 'dispense',
        'plan': [node('dispense', 'dispense', 'side_1',
                      next_on={'success': 'completed'},
                      params={'unknown': 1.0})],
    }
    result = expect(parse_response(payload), scene_for(20))
    assert result.ok is False
    assert 'params' in result.reason


def test_unconfirmed_grasp_requires_inspect_state():
    value = graph(
        'grab_vacuum',
        node('grab_vacuum', 'grab', 'vacuum',
             next_on={'success': 'transfer', 'grasp_failed': 'stop'}),
        node('stop', 'stop', 'robot',
             next_on={'success': 'transfer'}),
        node('transfer', 'transfer', 'vacuum', 'work',
             next_on={'success': 'completed'}),
    )
    result = expect(value, scene_for(1))
    assert result.ok is False
    assert 'inspect_state' in result.reason


def test_parse_errors_are_raised_not_smuggled_into_a_graph():
    with pytest.raises(PlanSchemaError):
        parse_response('[]')
