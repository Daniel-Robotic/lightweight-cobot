import time
from pathlib import Path
from types import SimpleNamespace
import pytest
from sensor_msgs.msg import JointState
from fastapi import HTTPException
from iiwa_web import trajectory as api


@pytest.fixture
def bridge(monkeypatch):
    msg = JointState()
    msg.name = [f'joint{i}' for i in range(7,0,-1)]
    msg.position = [0.] * 7
    msg.velocity = [0.] * 7
    msg.header.stamp.sec = 100
    params = dict(trajectory_state_timeout=.5, trajectory_start_tolerance=.001,
                  trajectory_stopped_velocity=.01,
                  joint_limits_path=str(Path(__file__).resolve().parents[2] / 'iiwa_config/config/moveit/joint_limits.yaml'))
    published = []
    node = SimpleNamespace(
        state=msg, age=0., published=published,
        get_parameter=lambda key: SimpleNamespace(value=params[key]),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=100_000_000_000)),
        publish=lambda topic, typ, payload: published.append(payload))
    node.get_latest = lambda topic: node.state
    node.get_latest_with_age = lambda topic: (node.state, node.age)
    monkeypatch.setattr(api, 'get_bridge', lambda: node)
    monkeypatch.setattr(api, '_direct_until', 0.)
    return node


def test_stale_and_moving_state_not_published(bridge):
    for age, speed in [(1.,0.),(0.,.2)]:
        bridge.age=age; bridge.state.velocity[0]=speed
        with pytest.raises(HTTPException): api._send_rows([([.1]*7,1.)])
    assert not bridge.published


def test_normal_preparation_and_duplicate_admission(bridge):
    msg, duration, scale = api._send_rows([([.1]*7,.1)])
    assert duration > .1 and scale > 1
    assert list(msg.points[0].positions) == [0.]*7
    assert list(msg.points[-1].positions) == [.1]*7
    assert len(msg.points[1].accelerations) == 7
    with pytest.raises(HTTPException): api._send_rows([([.2]*7,1.)])
    assert len(bridge.published) == 1


def test_state_rechecked_after_preparation(bridge, monkeypatch):
    original=api.prepare_trajectory
    def prepare(*args):
        result=original(*args)
        bridge.state.position[0]=.02
        return result
    monkeypatch.setattr(api,'prepare_trajectory',prepare)
    with pytest.raises(HTTPException): api._send_rows([([.1]*7,1.)])
    assert not bridge.published


def test_stop_cancels_inflight_preparation(bridge, monkeypatch):
    original=api.prepare_trajectory
    def prepare(*args):
        result=original(*args)
        api.send_stop_trajectory()
        return result
    monkeypatch.setattr(api,'prepare_trajectory',prepare)
    with pytest.raises(HTTPException): api._send_rows([([.1]*7,1.)])
    assert len(bridge.published)==1  # stop only


def test_stop_handles_missing_named_feedback(bridge):
    bridge.state.name=[]
    api.send_stop_trajectory()
    assert len(bridge.published)==1 and not bridge.published[0].points
