"""Strict parsing tests for the graph-v3 response contract."""
from __future__ import annotations

import json

import pytest

from iiwa_llm_plan_executor.schema import (
    MAX_NODES,
    PlanSchemaError,
    parse_response,
)


def skill_node(
    node_id: str,
    skill: str,
    *,
    obj: str = '',
    target: str = '',
    tool: str | None = None,
    next_id: str = 'completed',
    on: dict[str, str] | None = None,
    params: dict | None = None,
    max_retries: int | None = None,
) -> dict:
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
    if max_retries is not None:
        payload['max_retries'] = max_retries
    return payload


def sample_graph() -> dict:
    return {
        'schema_version': 3,
        'status': 'ready',
        'entry': 'p1_find',
        'goals': [
            {
                'id': 'g1',
                'predicate': 'installed',
                'args': {'part': 'cap', 'destination': 'hatch_seat'},
            },
        ],
        'plan': [
            skill_node(
                'p1_find', 'detect', obj='cap', next_id='p1_grab',
                on={'object_unavailable': 'replan_required'},
            ),
            skill_node(
                'p1_grab', 'grab', obj='cap', tool='vacuum',
                next_id='observe_install',
            ),
            {
                'id': 'observe_install',
                'kind': 'observe',
                'predicate': {
                    'name': 'installed',
                    'args': {'part': 'cap', 'destination': 'hatch_seat'},
                },
                'on': {
                    'true': 'completed',
                    'false': 'replan_required',
                    'unknown': 'operator_required',
                },
            },
        ],
    }


def test_ready_graph_v3_is_parsed_and_supported_events_are_mapped():
    graph = parse_response(json.dumps(sample_graph()))
    assert graph.status == 'ready'
    assert graph.entry == 'p1_find'
    assert graph.ids == ('p1_find', 'p1_grab', 'observe_install')
    assert graph.goals[0].canonical() == 'installed(destination=hatch_seat,part=cap)'
    detect = graph.node('p1_find')
    assert detect.transition('found') == 'p1_grab'
    assert detect.transition('not_found') == 'replan_required'
    assert detect.transition('failed') == 'replan_required'
    grab = graph.node('p1_grab')
    assert grab.tool == 'vacuum'
    assert grab.transition('grasp_failed') == 'operator_required'
    observe = graph.node('observe_install')
    assert observe.kind == 'observe'
    assert observe.transition('unknown') == 'operator_required'


def test_old_format_without_schema_version_is_unsupported_contract():
    payload = sample_graph()
    payload.pop('schema_version')
    with pytest.raises(PlanSchemaError) as error:
        parse_response(payload)
    assert error.value.code == 'unsupported_contract'

    payload['schema_version'] = 2
    with pytest.raises(PlanSchemaError) as error:
        parse_response(payload)
    assert error.value.code == 'unsupported_contract'


def test_blocked_and_clarification_are_parsed_without_a_plan():
    blocked = parse_response({
        'schema_version': 3,
        'status': 'blocked',
        'entry': '',
        'plan': [],
        'reason': 'vacuum tool is absent',
        'reason_code': 'missing_required_skill',
    })
    assert blocked.status == 'blocked'
    assert blocked.nodes == ()
    assert blocked.status_details['reason_code'] == 'missing_required_skill'

    clarification = parse_response({
        'schema_version': 3,
        'status': 'clarification_required',
        'entry': '',
        'plan': [],
        'question': 'Сколько винтов подать?',
        'missing': ['count'],
    })
    assert clarification.status == 'clarification_required'
    assert clarification.status_details['missing'] == ['count']


def test_not_executable_status_with_a_plan_is_rejected():
    payload = sample_graph()
    payload['status'] = 'blocked'
    payload['reason'] = 'blocked'
    with pytest.raises(PlanSchemaError) as error:
        parse_response(payload)
    assert error.value.code == 'invalid'


@pytest.mark.parametrize('mutation', [
    lambda payload: payload.pop('goals'),
    lambda payload: payload.update(entry=''),
    lambda payload: payload.update(plan=[]),
    lambda payload: payload.update(extra=1),
    lambda payload: payload.update(status='unknown'),
])
def test_invalid_top_level_shapes_are_rejected(mutation):
    payload = sample_graph()
    mutation(payload)
    with pytest.raises(PlanSchemaError):
        parse_response(payload)


def test_duplicate_ids_are_rejected():
    payload = sample_graph()
    payload['plan'][1]['id'] = 'p1_find'
    with pytest.raises(PlanSchemaError, match='duplicate node ids'):
        parse_response(payload)


def test_unknown_node_keys_and_events_are_rejected():
    payload = sample_graph()
    payload['plan'][0]['pose'] = {'x': 1}
    with pytest.raises(PlanSchemaError, match='unknown keys'):
        parse_response(payload)

    payload = sample_graph()
    payload['plan'][0]['on'] = {'found': 'p1_grab'}
    with pytest.raises(PlanSchemaError, match='unknown normalized event'):
        parse_response(payload)

    payload = sample_graph()
    payload['plan'][1]['on'] = {'already_held': 'completed'}
    with pytest.raises(PlanSchemaError) as error:
        parse_response(payload)
    assert error.value.code == 'unsupported_contract'


def test_reserved_terminal_ids_and_oversized_plans_are_rejected():
    payload = sample_graph()
    payload['plan'][0]['id'] = 'completed'
    with pytest.raises(PlanSchemaError, match='reserved terminal'):
        parse_response(payload)

    nodes = []
    for index in range(MAX_NODES + 1):
        nodes.append(skill_node(f'n{index}', 'stop', obj='robot'))
    with pytest.raises(PlanSchemaError, match='limit'):
        parse_response({
            'schema_version': 3,
            'status': 'ready',
            'entry': 'n0',
            'goals': [{'id': 'g', 'predicate': 'visible', 'args': {'object': 'fixture'}}],
            'plan': nodes,
        })


def test_observe_unknown_must_be_a_safe_terminal():
    payload = sample_graph()
    payload['plan'][2]['on']['unknown'] = 'p1_grab'
    with pytest.raises(PlanSchemaError) as error:
        parse_response(payload)
    assert error.value.code == 'unsafe_unknown'


def test_unbounded_cycles_are_rejected_and_bounded_cycles_are_accepted():
    payload = sample_graph()
    payload['plan'] = payload['plan'][:2]
    payload['plan'][1]['next'] = 'p1_find'
    with pytest.raises(PlanSchemaError, match='max_retries'):
        parse_response(payload)

    payload = sample_graph()
    payload['plan'] = payload['plan'][:2]
    payload['plan'][1]['next'] = 'p1_find'
    payload['plan'][1]['max_retries'] = 2
    graph = parse_response(payload)
    assert graph.node('p1_grab').max_retries == 2


def test_params_are_scalars_and_inspect_state_must_be_observe():
    payload = sample_graph()
    payload['plan'][0]['params'] = {'start_fraction': 0.2, 'end_fraction': 1.0}
    graph = parse_response(payload)
    assert graph.node('p1_find').params['start_fraction'] == 0.2

    payload['plan'][0]['params'] = {'bad': {'nested': 1}}
    with pytest.raises(PlanSchemaError):
        parse_response(payload)

    payload = sample_graph()
    payload['plan'][0] = skill_node('p1_find', 'inspect_state', obj='cap')
    with pytest.raises(PlanSchemaError) as error:
        parse_response(payload)
    assert error.value.code == 'unsupported_contract'


def test_invalid_json_is_reported():
    with pytest.raises(PlanSchemaError, match='not valid JSON'):
        parse_response('{not json')
