"""Launch validation and wiring for the planner scene."""
from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node

from iiwa_experement.runtime_config import EXPERIMENT_IDS, validate_experiment_id


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch' / 'planner.launch.py'
CONFIG = PACKAGE / 'config' / 'planner.yaml'


def load_launch_module():
    spec = importlib.util.spec_from_file_location('planner_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def flat_parameters(node: Node):
    def flatten(value):
        if hasattr(value, 'text'):
            return [value.text]
        if isinstance(value, (list, tuple)):
            result = []
            for item in value:
                result.extend(flatten(item))
            return result
        return [value]

    values = []
    for parameter_set in node._Node__parameters:
        for value in parameter_set.values():
            values.extend(flatten(value))
    return values


def test_valid_experiment_ids_and_planner_mode():
    module = load_launch_module()
    for experiment_id in EXPERIMENT_IDS:
        parsed, mode = module.validate_arguments(
            str(experiment_id), 'planner', 'true',
        )
        assert parsed == experiment_id
        assert mode == 'planner'
    assert validate_experiment_id(' 20 ') == 20


@pytest.mark.parametrize('raw', ['', '0', '21', 'abc', '1.0'])
def test_invalid_experiment_ids_fail_before_webots(raw):
    module = load_launch_module()
    with pytest.raises(ValueError):
        module.validate_arguments(raw, 'planner', 'true')


def test_unknown_mode_or_physical_robot_is_rejected_before_webots():
    module = load_launch_module()
    with pytest.raises(ValueError, match='mode'):
        module.validate_arguments('20', 'execute', 'true')
    with pytest.raises(ValueError, match='simulate'):
        module.validate_arguments('20', 'planner', 'false')


def test_runtime_actions_start_scene_context_and_executor_only():
    module = load_launch_module()
    config = module.load_config(CONFIG)
    actions = module.runtime_actions(
        experiment_id=20,
        scene_config='/tmp/scenes.yaml',
        base_setting='/tmp/setting.yaml',
        rviz='false',
        config=config,
    )
    includes = [action for action in actions if isinstance(action, IncludeLaunchDescription)]
    nodes = [action for action in actions if isinstance(action, Node)]
    assert len(includes) == 1
    assert len(nodes) == 2
    arguments = dict(includes[0].launch_arguments)
    assert arguments['experiment_id'] == '20'
    assert arguments['mode'] == 'planner'
    assert arguments['scene_config'] == '/tmp/scenes.yaml'
    assert arguments['base_setting'] == '/tmp/setting.yaml'

    packages = {node.node_package for node in nodes}
    executables = {node.node_executable for node in nodes}
    assert packages == {'iiwa_scene_context', 'iiwa_llm_plan_executor'}
    assert executables == {'scene_context', 'llm_plan_executor'}
    assert 'reference_runner' not in executables
    assert 'iiwa_web' not in packages

    scene_node = next(node for node in nodes if node.node_executable == 'scene_context')
    executor_node = next(
        node for node in nodes if node.node_executable == 'llm_plan_executor'
    )
    scene_values = flat_parameters(scene_node)
    executor_values = flat_parameters(executor_node)
    assert 20 in scene_values
    assert 20 in executor_values
    assert 3.0 in executor_values
    assert 512 in executor_values
    assert True in executor_values
    assert any('iiwa_arm' in str(value) for value in executor_values)


def test_config_contains_the_documented_defaults():
    module = load_launch_module()
    config = module.load_config(CONFIG)
    assert config['start_delay_sec'] == 3.0
    assert config['scene_max_age_sec'] == 2.0
    assert config['max_steps'] == 512
    assert config['close_inspection_objects'] == ['cap']
    assert config['trace_dir'].endswith('iiwa_llm_plan_executor/runs')


def test_launch_description_declares_all_arguments_and_validates_in_opaque():
    module = load_launch_module()
    description = module.generate_launch_description()
    arguments = {
        entity.name: entity.default_value
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert set(arguments) == {
        'experiment_id', 'mode', 'simulate', 'rviz', 'scene_config',
        'base_setting', 'config',
    }
    assert [item.text for item in arguments['experiment_id']] == ['']
    assert [item.text for item in arguments['mode']] == ['planner']
    assert [item.text for item in arguments['simulate']] == ['true']
    assert len([
        entity for entity in description.entities if isinstance(entity, OpaqueFunction)
    ]) == 1
