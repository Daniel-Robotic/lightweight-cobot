"""Camera cadence must not halve when 30 Hz is sampled on 32 ms steps."""
import pytest

from iiwa_utils.webots_camera import FrameSchedule


def test_30_hz_on_webots_32_ms_steps():
    schedule = FrameSchedule(30, 0.0)
    stamps = [tick * 0.032 for tick in range(1, 3126)
              if schedule.due(tick * 0.032)]
    assert len(stamps) == 3000
    assert all(b > a for a, b in zip(stamps, stamps[1:]))
    assert max(b - a for a, b in zip(stamps, stamps[1:])) <= 0.064001


def test_delayed_step_does_not_publish_catchup_burst():
    schedule = FrameSchedule(30, 0.0)
    assert schedule.due(1.0)
    assert not schedule.due(1.0)
    assert not schedule.due(1.001)
    assert schedule.due(1.034)


def test_time_reset_restarts_schedule():
    schedule = FrameSchedule(30, 10.0)
    assert not schedule.due(0.0)
    assert schedule.due(0.034)


@pytest.mark.parametrize('rate', [0, -1, float('nan'), float('inf')])
def test_invalid_rates_rejected(rate):
    with pytest.raises(ValueError):
        FrameSchedule(rate, 0.0)
