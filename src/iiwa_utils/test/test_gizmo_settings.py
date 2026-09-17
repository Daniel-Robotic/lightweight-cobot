import dataclasses

import pytest

from iiwa_utils import setting_loader as settings


def test_old_configuration_does_not_enable_new_input():
    assert settings.parse_tcp_gizmo(None).enabled is False


@pytest.mark.parametrize('value', ['false', 1, [], {'enabled': 'false'},
                                  {'enabled': True, 'publish_period': 0},
                                  {'max_linear_speed': float('nan')},
                                  {'state_timeout': -1}])
def test_bad_gizmo_settings_are_rejected(value):
    with pytest.raises(settings.SettingsError):
        settings.parse_tcp_gizmo(value)


def test_partial_config_preserves_defaults():
    cfg = settings.parse_tcp_gizmo({'enabled': True, 'max_linear_speed': 0.05})
    assert cfg.enabled
    assert cfg.max_linear_speed == 0.05
    assert dataclasses.asdict(cfg)['max_angular_speed'] > 0
