"""Launch validation and action wiring."""
import importlib.util
from pathlib import Path

import pytest
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node

from iiwa_experement.runtime_config import validate_experiment_id


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


def test_runtime_actions_include_bringup_and_one_scene_spawner():
    module = load_launch_module()
    actions = module.runtime_actions(
        experiment_id=19,
        scene_config='/tmp/custom scenes.yaml',
        setting_path='/tmp/run/setting.yaml',
        rviz='false',
    )
    includes = [a for a in actions if isinstance(a, IncludeLaunchDescription)]
    nodes = [a for a in actions if isinstance(a, Node)]
    assert len(includes) == 1
    assert len(nodes) == 1
    assert 'iiwa_bringup' in repr(includes[0].launch_description_source.location)
    arguments = dict(includes[0].launch_arguments)
    assert arguments['simulate'] == 'true'
    assert arguments['rviz'] == 'false'
    assert arguments['setting'] == '/tmp/run/setting.yaml'
    node = nodes[0]
    assert node.node_package == 'iiwa_experement'
    assert node.node_executable == 'scene_spawner'
    assert vars(node)['_Node__node_name'] == 'scene_spawner'
    assert module.spawner_parameters(19, '/tmp/custom scenes.yaml') == {
        'experiment_id': 19,
        'scene_config': '/tmp/custom scenes.yaml',
    }


def test_runtime_actions_can_register_cleanup(tmp_path):
    from launch.actions import RegisterEventHandler

    module = load_launch_module()
    actions = module.runtime_actions(
        experiment_id=1,
        scene_config='/tmp/scenes.yaml',
        setting_path='/tmp/setting.yaml',
        cleanup_dir=str(tmp_path / 'run'),
    )
    assert len(actions) == 3
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
    assert set(arguments) == {'experiment_id', 'rviz', 'scene_config', 'base_setting'}
    assert [substitution.text for substitution in arguments['experiment_id']] == ['']
    assert [substitution.text for substitution in arguments['rviz']] == ['false']
    assert 'scenes.yaml' in repr(arguments['scene_config'])
    assert 'setting.yaml' in repr(arguments['base_setting'])
    opaque = [e for e in description.entities if isinstance(e, OpaqueFunction)]
    assert len(opaque) == 1
