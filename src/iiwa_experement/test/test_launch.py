"""Launch validation and action wiring for view/execute modes."""
import importlib.util
from pathlib import Path

import pytest
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch_ros.actions import Node

from iiwa_experement.runtime_config import (
    DEFAULT_START_DELAY_SEC,
    DEFAULT_TRACE_DIR,
    validate_experiment_id,
    validate_mode,
    validate_start_delay,
    validate_trace_dir,
)


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch' / 'experiment.launch.py'


def load_launch_module():
    spec = importlib.util.spec_from_file_location('experiment_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('raw, expected', [('1', 1), ('20', 20), (' 11 ', 11)])
def test_valid_experiment_ids(raw, expected):
    assert validate_experiment_id(raw) == expected


@pytest.mark.parametrize('raw', ['', '0', '21', 'abc', '-1', '1.0', '01', '  '])
def test_invalid_experiment_ids_are_rejected(raw):
    with pytest.raises(ValueError):
        validate_experiment_id(raw)


@pytest.mark.parametrize('raw, expected', [
    ('view', 'view'),
    ('execute', 'execute'),
    ('planner', 'planner'),
    (' VIEW ', 'view'),
])
def test_valid_modes(raw, expected):
    assert validate_mode(raw) == expected


@pytest.mark.parametrize('raw', ['', 'run', 'VIEW '[:-1] + 'x', '1', 'plan'])
def test_invalid_modes_are_rejected(raw):
    with pytest.raises(ValueError):
        validate_mode(raw)


@pytest.mark.parametrize('raw, expected', [('0', 0.0), ('3', 3.0), ('2.5', 2.5)])
def test_valid_start_delays(raw, expected):
    assert validate_start_delay(raw) == expected


@pytest.mark.parametrize('raw', ['', '-1', 'abc', 'nan', 'inf'])
def test_invalid_start_delays_are_rejected(raw):
    with pytest.raises(ValueError):
        validate_start_delay(raw)


def test_trace_dir_must_not_be_empty():
    assert validate_trace_dir('~/.ros/run') == '~/.ros/run'
    with pytest.raises(ValueError):
        validate_trace_dir('  ')


def test_view_starts_bringup_spawner_and_read_only_supervisor():
    module = load_launch_module()
    actions = module.runtime_actions(
        experiment_id=19,
        scene_config='/tmp/custom scenes.yaml',
        setting_path='/tmp/run/setting.yaml',
        rviz='false',
        mode='view',
    )
    includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    nodes = [a for a in actions if isinstance(a, Node)]
    processes = [a for a in actions if type(a) is ExecuteProcess]
    assert len(includes) == 1
    assert len(nodes) == 1
    assert len(processes) == 1
    arguments = dict(includes[0].launch_arguments)
    assert arguments['simulate'] == 'true'
    assert arguments['rviz'] == 'false'
    assert arguments['setting'] == '/tmp/run/setting.yaml'
    spawner = nodes[0]
    assert spawner.node_package == 'iiwa_experement'
    assert spawner.node_executable == 'scene_spawner'
    assert module.spawner_parameters(19, '/tmp/custom scenes.yaml') == {
        'experiment_id': 19,
        'scene_config': '/tmp/custom scenes.yaml',
    }
    parameters = module.supervisor_parameters(
        19, '/tmp/custom scenes.yaml', '/tmp/geometry.yaml', True,
    )
    assert parameters['experiment_id'] == 19
    assert parameters['read_only'] is True
    # A view run has no motion client or trajectory targets.
    assert not any(
        isinstance(action, Node) and action.node_executable == 'reference_runner'
        for action in actions
    )


def test_execute_adds_the_reference_runner_after_the_supervisor():
    module = load_launch_module()
    actions = module.runtime_actions(
        experiment_id=1,
        scene_config='/tmp/scenes.yaml',
        setting_path='/tmp/setting.yaml',
        mode='execute',
        start_delay_sec=4.5,
    )
    runner = [
        action for action in actions
        if isinstance(action, Node) and action.node_executable == 'reference_runner'
    ]
    assert len(runner) == 1
    payload = module.runner_parameters(
        1, '/tmp/scenes.yaml', '/tmp/geometry.yaml', 4.5, DEFAULT_TRACE_DIR,
    )
    assert payload['experiment_id'] == 1
    assert payload['start_delay_sec'] == 4.5
    assert payload['trace_dir'] == DEFAULT_TRACE_DIR
    parameters = module.supervisor_parameters(
        1, '/tmp/scenes.yaml', '/tmp/geometry.yaml', False,
    )
    assert parameters['read_only'] is False
    assert any(type(action) is ExecuteProcess for action in actions)


def _process_arguments(process):
    return [item[0].text for item in process.cmd if item and item[0].text]


def test_planner_is_writable_and_has_no_reference_runner():
    module = load_launch_module()
    actions = module.runtime_actions(
        experiment_id=20,
        scene_config='/tmp/scenes.yaml',
        setting_path='/tmp/setting.yaml',
        mode='planner',
    )
    assert not any(
        isinstance(action, Node) and action.node_executable == 'reference_runner'
        for action in actions
    )
    processes = [action for action in actions if type(action) is ExecuteProcess]
    assert len(processes) == 1
    assert 'read_only:=false' in _process_arguments(processes[0])
    assert module.mode_read_only('planner') is False
    parameters = module.supervisor_parameters(
        20, '/tmp/scenes.yaml', '/tmp/geometry.yaml', False,
    )
    assert parameters['read_only'] is False


def test_view_remains_read_only_after_adding_the_planner_mode():
    module = load_launch_module()
    actions = module.runtime_actions(
        experiment_id=1,
        scene_config='/tmp/scenes.yaml',
        setting_path='/tmp/setting.yaml',
        mode='view',
    )
    processes = [action for action in actions if type(action) is ExecuteProcess]
    assert 'read_only:=true' in _process_arguments(processes[0])
    assert module.mode_read_only('view') is True
    assert module.mode_read_only('execute') is False


def test_runtime_actions_can_register_cleanup(tmp_path):
    module = load_launch_module()
    actions = module.runtime_actions(
        experiment_id=1,
        scene_config='/tmp/scenes.yaml',
        setting_path='/tmp/setting.yaml',
        cleanup_dir=str(tmp_path / 'run'),
    )
    assert isinstance(actions[-1], RegisterEventHandler)
    target = tmp_path / 'run'
    target.mkdir()
    (target / 'world.wbt').write_text('temporary', encoding='utf-8')
    module._remove_directory(None, directory=str(target))
    assert not target.exists()


def test_launch_description_declares_all_arguments_and_validates_in_opaque_function():
    module = load_launch_module()
    description = module.generate_launch_description()
    arguments = {
        entity.name: entity.default_value
        for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    }
    assert set(arguments) == {
        'experiment_id', 'mode', 'start_delay_sec', 'trace_dir',
        'rviz', 'scene_config', 'base_setting',
    }
    assert [substitution.text for substitution in arguments['experiment_id']] == ['']
    assert [substitution.text for substitution in arguments['mode']] == ['view']
    assert [substitution.text for substitution in arguments['start_delay_sec']] == [
        f'{DEFAULT_START_DELAY_SEC:g}'
    ]
    assert [substitution.text for substitution in arguments['trace_dir']] == [
        DEFAULT_TRACE_DIR
    ]
    opaque = [e for e in description.entities if isinstance(e, OpaqueFunction)]
    assert len(opaque) == 1
