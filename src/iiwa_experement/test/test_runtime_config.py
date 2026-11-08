"""Temporary runtime files point at installed assets and keep patron."""
from pathlib import Path

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

from iiwa_experement.runtime_config import (
    LOCAL_PROTOS,
    prepare_runtime_files,
    validate_mode,
    validate_start_delay,
    validate_trace_dir,
)


@pytest.mark.parametrize('raw, expected', [('view', 'view'), (' Execute ', 'execute')])
def test_mode_validation(raw, expected):
    assert validate_mode(raw) == expected


@pytest.mark.parametrize('raw', ['', 'motion', 'execute!'])
def test_mode_validation_rejects_unknown_modes(raw):
    with pytest.raises(ValueError):
        validate_mode(raw)


@pytest.mark.parametrize('raw, expected', [('0', 0.0), (' 3.0 ', 3.0)])
def test_start_delay_validation(raw, expected):
    assert validate_start_delay(raw) == expected


@pytest.mark.parametrize('raw', ['', '-0.1', 'no', 'nan', 'inf'])
def test_start_delay_validation_rejects_bad_values(raw):
    with pytest.raises(ValueError):
        validate_start_delay(raw)


def test_trace_dir_validation():
    assert validate_trace_dir('~/.ros/iiwa_experement/runs') == (
        '~/.ros/iiwa_experement/runs'
    )
    with pytest.raises(ValueError):
        validate_trace_dir('')


PACKAGE = Path(__file__).resolve().parents[1]
CONFIG_SHARE = Path(get_package_share_directory('iiwa_config'))
DESCRIPTION_SHARE = Path(get_package_share_directory('iiwa_description'))
PACKAGE_SHARE = Path(get_package_share_directory('iiwa_experement'))
BASE_SETTING = CONFIG_SHARE / 'config' / 'setting.yaml'
WORLD_TEMPLATE = PACKAGE_SHARE / 'worlds' / 'grant' / 'iiwa-grant.wbt'


def prepare(tmp_path, **overrides):
    arguments = {
        'base_setting': BASE_SETTING,
        'world_template': WORLD_TEMPLATE,
        'description_share': DESCRIPTION_SHARE,
        'output_dir': tmp_path / 'run',
    }
    arguments.update(overrides)
    return prepare_runtime_files(**arguments)


def test_world_copy_resolves_only_local_protos(tmp_path):
    world_path, _ = prepare(tmp_path)
    text = world_path.read_text(encoding='utf-8')
    for name in LOCAL_PROTOS:
        absolute = (DESCRIPTION_SHARE / 'resource' / 'protos' / name).resolve()
        assert absolute.is_file()
        assert f'EXTERNPROTO "{absolute}"' in text
    assert 'EXTERNPROTO "../../../iiwa_description' not in text
    assert text.count('https://raw.githubusercontent.com/cyberbotics/webots/R2025a/') == 3
    assert world_path == tmp_path / 'run' / 'iiwa-grant.wbt'


def test_setting_copy_disables_random_objects_and_keeps_patron_and_camera(tmp_path):
    world_path, setting_path = prepare(tmp_path)
    data = yaml.safe_load(setting_path.read_text(encoding='utf-8'))
    webots = data['digital_twin']['webots']
    assert webots['world'] == str(world_path.resolve())
    assert webots['objects']['enabled'] is False
    assert webots['objects']['count'] == 0
    assert data['tool']['active'] == 'patron'
    assert webots['transform'] == '-0.25 0 0.79'
    assert webots['rotation'] == '0 0 1 0'
    assert webots['controller_timer'] == '50'
    assert any(
        str(camera).endswith('config/cameras/d455_top.yaml')
        for camera in webots['cameras']
    )
    assert data['controller']['controller_path'].endswith('moveit/iiwa_controller.yaml')
    assert data['digital_twin']['rviz']['config'].endswith('rviz_moveit.rviz')


def test_prepared_setting_is_accepted_by_the_existing_loader(tmp_path):
    _, setting_path = prepare(tmp_path)
    from iiwa_utils.setting_loader import build_settings

    settings = build_settings(str(setting_path), check_files=True)
    assert settings.tool.active == 'patron'
    assert settings.digital_twin.webots.objects.enabled is False
    assert settings.digital_twin.webots.objects.count == 0
    assert settings.digital_twin.webots.cameras
    assert settings.digital_twin.webots.cameras[0].endswith('d455_top.yaml')


def test_tool_active_xacro_and_generated_urdf_include_patron(tmp_path):
    from iiwa_utils import converter
    from iiwa_utils.setting_loader import build_settings

    tool_active = DESCRIPTION_SHARE / 'urdf' / 'tools' / 'tool_active.xacro'
    assert 'patron.xacro' in tool_active.read_text(encoding='utf-8')
    _, setting_path = prepare(tmp_path)
    settings = build_settings(str(setting_path), check_files=True)
    urdf = converter.load_robot_description(
        settings.robot.description,
        settings.robot.name,
        {
            'simulate': 'true',
            'initial_positions_file': settings.controller.moveit.initial_positions,
            'joint_limits_file': settings.controller.moveit.joint_limits,
        },
    )
    assert 'name="patron"' in urdf
    assert 'link name="patron"' in urdf
    assert 'patron.dae' in urdf


def test_missing_world_template_is_reported(tmp_path):
    with pytest.raises(FileNotFoundError, match='world template'):
        prepare(tmp_path, world_template=tmp_path / 'missing.wbt')


def test_missing_proto_is_reported(tmp_path):
    empty_description = tmp_path / 'description'
    (empty_description / 'resource' / 'protos').mkdir(parents=True)
    with pytest.raises(FileNotFoundError, match='CollaborativeRobotTable.proto'):
        prepare(tmp_path, description_share=empty_description)


def test_missing_base_setting_is_reported(tmp_path):
    with pytest.raises(FileNotFoundError, match='base settings'):
        prepare(tmp_path, base_setting=tmp_path / 'missing.yaml')
