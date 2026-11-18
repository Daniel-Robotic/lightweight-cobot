"""Graph-v3 wire contract shared by the scene context and the executor.

The contract is deliberately stored in this ROS package instead of being read
from the training workspace.  ``GRAPH_V3_SCHEMA`` mirrors the published
graph-v3 schema (``schema_version: 3``) and is the single copy used by the
context endpoint, the executor parser and their tests.
"""
from __future__ import annotations

import copy
import re
from typing import Any


GRAPH_SCHEMA_VERSION = 3
GRAPH_CONTRACT_VERSION = 3
GRAPH_TERMINALS = ('completed', 'replan_required', 'operator_required')
NORMALIZED_EVENTS = (
    'object_unavailable',
    'target_unavailable',
    'already_held',
    'already_placed',
)
ID_PATTERN = re.compile(r'^[A-Za-z][A-Za-z0-9_\-]{0,63}$')
MAX_NODES = 256
MAX_RESPONSE_BYTES = 200_000


GRAPH_V3_SCHEMA: dict[str, Any] = {
    '$defs': {
        'BlockedResponse': {
            'additionalProperties': False,
            'properties': {
                'schema_version': {'const': 3, 'default': 3, 'type': 'integer'},
                'status': {'const': 'blocked', 'type': 'string'},
                'entry': {'const': '', 'default': '', 'type': 'string'},
                'goals': {'items': {'$ref': '#/$defs/Goal'}, 'type': 'array'},
                'plan': {
                    'items': {
                        'discriminator': {
                            'mapping': {
                                'observe': '#/$defs/ObserveNode',
                                'skill': '#/$defs/SkillNode',
                            },
                            'propertyName': 'kind',
                        },
                        'oneOf': [
                            {'$ref': '#/$defs/SkillNode'},
                            {'$ref': '#/$defs/ObserveNode'},
                        ],
                    },
                    'type': 'array',
                },
                'reason': {'minLength': 1, 'type': 'string'},
                'reason_code': {
                    'anyOf': [
                        {'maxLength': 64, 'type': 'string'},
                        {'type': 'null'},
                    ],
                    'default': None,
                },
                'entities': {'items': {'type': 'string'}, 'type': 'array'},
                'evidence': {'additionalProperties': True, 'type': 'object'},
            },
            'required': ['status', 'reason'],
            'type': 'object',
        },
        'ClarificationResponse': {
            'additionalProperties': False,
            'properties': {
                'schema_version': {'const': 3, 'default': 3, 'type': 'integer'},
                'status': {'const': 'clarification_required', 'type': 'string'},
                'entry': {'const': '', 'default': '', 'type': 'string'},
                'goals': {'items': {'$ref': '#/$defs/Goal'}, 'type': 'array'},
                'plan': {
                    'items': {
                        'discriminator': {
                            'mapping': {
                                'observe': '#/$defs/ObserveNode',
                                'skill': '#/$defs/SkillNode',
                            },
                            'propertyName': 'kind',
                        },
                        'oneOf': [
                            {'$ref': '#/$defs/SkillNode'},
                            {'$ref': '#/$defs/ObserveNode'},
                        ],
                    },
                    'type': 'array',
                },
                'question': {'minLength': 1, 'type': 'string'},
                'missing': {
                    'items': {'type': 'string'},
                    'minItems': 1,
                    'type': 'array',
                },
            },
            'required': ['status', 'question', 'missing'],
            'type': 'object',
        },
        'Goal': {
            'additionalProperties': False,
            'properties': {
                'id': {'maxLength': 64, 'minLength': 1, 'type': 'string'},
                'predicate': {
                    'maxLength': 64,
                    'minLength': 1,
                    'type': 'string',
                },
                'args': {
                    'additionalProperties': {'type': 'string'},
                    'type': 'object',
                },
            },
            'required': ['id', 'predicate'],
            'type': 'object',
        },
        'ObserveNode': {
            'additionalProperties': False,
            'properties': {
                'id': {'maxLength': 64, 'minLength': 1, 'type': 'string'},
                'kind': {'const': 'observe', 'type': 'string'},
                'predicate': {'$ref': '#/$defs/PredicateRef'},
                'on': {'$ref': '#/$defs/ObserveTransitions'},
                'max_retries': {
                    'anyOf': [
                        {'maximum': 20, 'minimum': 0, 'type': 'integer'},
                        {'type': 'null'},
                    ],
                    'default': None,
                },
            },
            'required': ['id', 'kind', 'predicate', 'on'],
            'type': 'object',
        },
        'ObserveTransitions': {
            'additionalProperties': False,
            'properties': {
                'true': {'type': 'string'},
                'false': {'type': 'string'},
                'unknown': {'type': 'string'},
            },
            'required': ['true', 'false', 'unknown'],
            'type': 'object',
        },
        'PredicateRef': {
            'additionalProperties': False,
            'properties': {
                'name': {'maxLength': 64, 'minLength': 1, 'type': 'string'},
                'args': {
                    'additionalProperties': {'type': 'string'},
                    'type': 'object',
                },
            },
            'required': ['name'],
            'type': 'object',
        },
        'ReadyResponse': {
            'additionalProperties': False,
            'properties': {
                'schema_version': {'const': 3, 'default': 3, 'type': 'integer'},
                'status': {'const': 'ready', 'type': 'string'},
                'entry': {'minLength': 1, 'type': 'string'},
                'goals': {
                    'items': {'$ref': '#/$defs/Goal'},
                    'minItems': 1,
                    'type': 'array',
                },
                'plan': {
                    'items': {
                        'discriminator': {
                            'mapping': {
                                'observe': '#/$defs/ObserveNode',
                                'skill': '#/$defs/SkillNode',
                            },
                            'propertyName': 'kind',
                        },
                        'oneOf': [
                            {'$ref': '#/$defs/SkillNode'},
                            {'$ref': '#/$defs/ObserveNode'},
                        ],
                    },
                    'minItems': 1,
                    'type': 'array',
                },
            },
            'required': ['status', 'entry', 'goals', 'plan'],
            'type': 'object',
        },
        'SkillNode': {
            'additionalProperties': False,
            'properties': {
                'id': {'maxLength': 64, 'minLength': 1, 'type': 'string'},
                'kind': {'const': 'skill', 'type': 'string'},
                'skill': {'maxLength': 64, 'minLength': 1, 'type': 'string'},
                'object': {'default': '', 'type': 'string'},
                'target': {'default': '', 'type': 'string'},
                'tool': {
                    'anyOf': [{'type': 'string'}, {'type': 'null'}],
                    'default': None,
                },
                'params': {'additionalProperties': True, 'type': 'object'},
                'next': {'minLength': 1, 'type': 'string'},
                'on': {
                    'additionalProperties': {'type': 'string'},
                    'type': 'object',
                },
                'max_retries': {
                    'anyOf': [
                        {'maximum': 20, 'minimum': 0, 'type': 'integer'},
                        {'type': 'null'},
                    ],
                    'default': None,
                },
            },
            'required': ['id', 'kind', 'skill', 'next'],
            'type': 'object',
        },
    },
    'discriminator': {
        'mapping': {
            'blocked': '#/$defs/BlockedResponse',
            'clarification_required': '#/$defs/ClarificationResponse',
            'ready': '#/$defs/ReadyResponse',
        },
        'propertyName': 'status',
    },
    'oneOf': [
        {'$ref': '#/$defs/ReadyResponse'},
        {'$ref': '#/$defs/ClarificationResponse'},
        {'$ref': '#/$defs/BlockedResponse'},
    ],
}


def response_schema() -> dict[str, Any]:
    """Return a fresh copy of the graph-v3 JSON Schema."""
    return copy.deepcopy(GRAPH_V3_SCHEMA)
