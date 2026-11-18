"""Graph-v3 admission: schema, goals, observations, branches and ownership."""
from __future__ import annotations

from pathlib import Path

import pytest

from iiwa_experement.geometry import load_geometry
from iiwa_experement.scenario import load_scenario
from iiwa_llm_plan_executor.schema import PlanSchemaError, parse_response
from iiwa_llm_plan_executor.validator import validate_graph
from iiwa_scene_context.catalog import DEFAULT_CATALOG
from iiwa_scene_context.snapshot import build_envelope, build_scene


PACKAGE = Path(__file__).resolve().parents[1]
SCENES = PACKAGE.parent / 'iiwa_experement' / 'config' / 'scenes.yaml'
GEOMETRY = load_geometry(PACKAGE.parent / 'iiwa_experement' / 'config' / 'geometry.yaml')


def spec_for(experiment_id: int = 1):
    return load_scenario(experiment_id, SCENES)


def scene_for(
    experiment_id: int = 1,
    *,
    mounted=None,
    held=None,
    cap_state=None,
    tool_states=None,
):
    spec = spec_for(experiment_id)
    tools = {tool.name: {'state': 'docked'} for tool in spec.tools}
    for name, status in (tool_states or {}).items():
        tools[name] = {'state': status}
    if mounted:
        tools[mounted] = {'state': 'mounted'}
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
        GEOMETRY,
        stamp=10.0,
    )
    return build_envelope(
        scene,
        state_version=1,
        stamp=10.0,
        experiment_id=experiment_id,
    )


def skill_node(
    node_id,
    skill,
    *,
    obj='',
    target='',
    tool=None,
    next_id='completed',
    on=None,
    params=None,
):
    payload = {
        'id': node_id,
        'kind': 'skill',
        'skill': skill,
        'object': obj,
        'target': target,
        'next': next_id,
        'on': on or {},
    }
    if tool is not None:
        payload['tool'] = tool
    if params is not None:
        payload['params'] = params
    return payload


def observe_node(node_id, predicate, args, *, true='completed',
                 false='replan_required', unknown='operator_required'):
    return {
        'id': node_id,
        'kind': 'observe',
        'predicate': {'name': predicate, 'args': args},
        'on': {'true': true, 'false': false, 'unknown': unknown},
    }


def graph(entry, nodes, goals=None):
    return parse_response({
        'schema_version': 3,
        'status': 'ready',
        'entry': entry,
        'goals': goals or [{
            'id': 'g1',
            'predicate': 'installed',
            'args': {'part': 'cap', 'destination': 'hatch_seat'},
        }],
        'plan': list(nodes),
    })


def valid_graph():
    return graph(
        'detect',
        [
            skill_node(
                'detect', 'detect', obj='cap', next_id='grab',
                on={'object_unavailable': 'replan_required'},
            ),
            skill_node('grab', 'grab', obj='cap', tool='vacuum', next_id='check'),
            observe_node(
                'check', 'installed',
                {'part': 'cap', 'destination': 'hatch_seat'},
            ),
        ],
    )


def expect(value, scene=None):
    return validate_graph(value, scene or scene_for(), DEFAULT_CATALOG)


def test_valid_graph_is_accepted():
    result = expect(valid_graph())
    assert result.ok is True, result.reason
    assert result.details['goals'] == [
        'installed(destination=hatch_seat,part=cap)',
    ]


def test_blocked_and_clarification_never_reach_execution():
    blocked = parse_response({
        'schema_version': 3,
        'status': 'blocked',
        'entry': '',
        'plan': [],
        'reason': 'vacuum is absent',
    })
    result = expect(blocked)
    assert result.ok is False
    assert result.code == 'not_executable'
    assert result.details['status'] == 'blocked'

    clarification = parse_response({
        'schema_version': 3,
        'status': 'clarification_required',
        'entry': '',
        'plan': [],
        'question': 'Сколько винтов?',
        'missing': ['count'],
    })
    result = expect(clarification)
    assert result.code == 'not_executable'
    assert result.details['missing'] == ['count']


def test_unknown_object_and_target_are_rejected():
    unknown_object = graph(
        'detect',
        [skill_node('detect', 'detect', obj='ghost')],
        goals=[{
            'id': 'g', 'predicate': 'visible', 'args': {'object': 'fixture'},
        }],
    )
    result = expect(unknown_object)
    assert result.ok is False
    assert result.code == 'unknown_id'
    assert result.details['node_id'] == 'detect'

    unknown_target = graph(
        'move',
        [skill_node('move', 'transfer', obj='cap', target='ghost')],
    )
    result = expect(unknown_target)
    assert result.code == 'unknown_id'


def test_declared_tool_must_match_the_grasp_hardware():
    value = graph(
        'grab',
        [skill_node('grab', 'grab', obj='cap', tool='magnetic_tool')],
    )
    result = expect(value, scene_for(20))
    assert result.ok is False
    assert 'vacuum' in result.reason


def test_grab_without_the_vacuum_tool_is_admitted_for_runtime_failure():
    value = graph(
        'grab',
        [
            skill_node(
                'grab', 'grab', obj='cap', tool='vacuum', next_id='completed',
                on={'object_unavailable': 'operator_required'},
            ),
        ],
    )
    result = expect(value, scene_for(1))
    assert result.ok is True


def test_unsupported_goal_is_rejected_before_motion():
    value = graph(
        'detect',
        [skill_node('detect', 'detect', obj='cap')],
        goals=[{
            'id': 'g', 'predicate': 'dispensed', 'args': {'side': 'side_1'},
        }],
    )
    result = expect(value, scene_for(20))
    assert result.ok is False
    assert result.code == 'unsupported_goal'


def test_unsupported_observation_is_rejected():
    value = graph(
        'detect',
        [
            skill_node('detect', 'detect', obj='cap', next_id='check'),
            observe_node('check', 'dispensed', {'side': 'side_1'}),
        ],
    )
    result = expect(value, scene_for(20))
    assert result.ok is False
    assert result.code == 'unsupported_predicate'


def test_unsupported_normalized_event_is_rejected_at_parse_time():
    with pytest.raises(PlanSchemaError) as error:
        graph(
            'detect',
            [skill_node(
                'detect', 'detect', obj='cap',
                on={'already_placed': 'completed'},
            )],
        )
    assert error.value.code == 'unsupported_contract'


def test_unsafe_unknown_observation_is_rejected_at_parse_time():
    with pytest.raises(PlanSchemaError) as error:
        graph(
            'detect',
            [
                skill_node('detect', 'detect', obj='cap', next_id='check'),
                observe_node(
                    'check', 'installed',
                    {'part': 'cap', 'destination': 'hatch_seat'},
                    unknown='detect',
                ),
            ],
        )
    assert error.value.code == 'unsafe_unknown'


def test_unknown_held_object_blocks_transfer_but_keeps_the_graph_admissible():
    value = graph(
        'transfer',
        [skill_node('transfer', 'transfer', obj='cap', target='hatch_seat')],
    )
    result = expect(value)
    assert result.ok is False
    assert result.code == 'replan_required'
