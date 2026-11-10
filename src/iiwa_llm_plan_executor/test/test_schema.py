"""Parsing tests for the externally supplied plan response."""
from __future__ import annotations

import json

import pytest

from iiwa_llm_plan_executor.schema import (
    MAX_NODES,
    PlanSchemaError,
    parse_response,
)


def sample_graph():
    return {
        'status': 'ready',
        'entry': 'p1_find',
        'plan': [
            {
                'id': 'p1_find',
                'skill': 'detect',
                'object': 'cap',
                'target': '',
                'transitions': [
                    {'event': 'found', 'next': 'p1_grab'},
                    {'event': 'not_found', 'next': 'replan_required'},
                    {'event': 'failed', 'next': 'replan_required'},
                ],
            },
            {
                'id': 'p1_grab',
                'skill': 'grab',
                'object': 'vacuum',
                'target': '',
                'transitions': [
                    {'event': 'success', 'next': 'completed'},
                    {'event': 'grasp_failed', 'next': 'operator_required'},
                    {'event': 'human_in_scene', 'next': 'operator_required'},
                    {'event': 'collision', 'next': 'operator_required'},
                    {'event': 'failed', 'next': 'replan_required'},
                ],
            },
        ],
    }


def test_shown_status_entry_plan_transitions_format_is_parsed():
    graph = parse_response(json.dumps(sample_graph()))
    assert graph.status == 'ready'
    assert graph.entry == 'p1_find'
    assert graph.ids == ('p1_find', 'p1_grab')
    assert graph.node('p1_find').transitions[0].event == 'found'
    assert graph.node('p1_grab').object == 'vacuum'


def test_top_level_transitions_are_merged_into_nodes():
    payload = sample_graph()
    payload['plan'][0].pop('transitions')
    payload['transitions'] = [
        {'from': 'p1_find', 'event': 'found', 'to': 'p1_grab'},
        {'from': 'p1_find', 'event': 'not_found', 'to': 'replan_required'},
        {'from': 'p1_find', 'event': 'failed', 'to': 'replan_required'},
    ]
    graph = parse_response(payload)
    events = {item.event for item in graph.node('p1_find').transitions}
    assert events == {'found', 'not_found', 'failed'}


def test_blocked_response_has_no_plan():
    graph = parse_response({'status': 'blocked', 'entry': '', 'plan': []})
    assert graph.status == 'blocked'
    assert graph.nodes == ()


def test_blocked_response_with_nodes_is_rejected():
    with pytest.raises(PlanSchemaError):
        parse_response({
            'status': 'blocked',
            'entry': 'a',
            'plan': sample_graph()['plan'],
        })


@pytest.mark.parametrize('payload', [
    {'status': 'ready', 'entry': 'a', 'plan': []},
    {'status': 'ready', 'entry': '', 'plan': sample_graph()['plan']},
    {'status': 'unknown', 'entry': 'a', 'plan': sample_graph()['plan']},
    {'status': 'ready', 'entry': 'a', 'plan': sample_graph()['plan'],
     'extra': 1},
])
def test_invalid_top_level_shapes_are_rejected(payload):
    with pytest.raises(PlanSchemaError):
        parse_response(payload)


def test_duplicate_ids_are_rejected():
    payload = sample_graph()
    payload['plan'][1]['id'] = 'p1_find'
    with pytest.raises(PlanSchemaError, match='duplicate node id'):
        parse_response(payload)


def test_unknown_node_keys_and_transition_events_are_rejected():
    payload = sample_graph()
    payload['plan'][0]['pose'] = {'x': 1}
    with pytest.raises(PlanSchemaError, match='unknown keys'):
        parse_response(payload)

    payload = sample_graph()
    payload['plan'][0]['transitions'].append(
        {'event': 'found', 'next': 'p1_grab'},
    )
    with pytest.raises(PlanSchemaError, match='duplicate event'):
        parse_response(payload)


def test_reserved_terminal_ids_and_oversized_plans_are_rejected():
    payload = sample_graph()
    payload['plan'][0]['id'] = 'completed'
    with pytest.raises(PlanSchemaError, match='reserved terminal'):
        parse_response(payload)

    nodes = []
    for index in range(MAX_NODES + 1):
        nodes.append({
            'id': f'n{index}',
            'skill': 'stop',
            'object': 'robot',
            'target': '',
            'transitions': [
                {'event': 'success', 'next': 'completed'},
                {'event': 'failed', 'next': 'replan_required'},
            ],
        })
    with pytest.raises(PlanSchemaError, match='limit'):
        parse_response({'status': 'ready', 'entry': 'n0', 'plan': nodes})


def test_params_are_strict_scalars():
    payload = sample_graph()
    payload['plan'][0]['params'] = {'start_fraction': 0.2, 'end_fraction': 1.0}
    graph = parse_response(payload)
    assert graph.node('p1_find').params['start_fraction'] == 0.2

    payload['plan'][0]['params'] = {'bad': {'nested': 1}}
    with pytest.raises(PlanSchemaError):
        parse_response(payload)


def test_invalid_json_is_reported():
    with pytest.raises(PlanSchemaError, match='not valid JSON'):
        parse_response('{not json')
