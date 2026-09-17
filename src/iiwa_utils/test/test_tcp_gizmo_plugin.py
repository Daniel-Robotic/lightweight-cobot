"""Exercise the real plugin step with Webots field/mouse adapters."""
import copy
from types import SimpleNamespace

import numpy as np
import pytest

from iiwa_msgs.msg import GizmoState
from iiwa_utils import tcp_gizmo as plugin
from iiwa_utils.tcp_gizmo_model import DragTarget


class Field:
    def __init__(self, value):
        self.value = value

    def getSFVec3f(self):
        return self.value

    getSFRotation = getSFVec3f

    def setSFVec3f(self, value):
        self.value = value

    setSFRotation = setSFVec3f


def make_plugin(monkeypatch):
    instance = plugin.WebotsTcpGizmo()
    instance._node = None
    instance._frame = 'base_link'
    instance._mount = np.eye(4)
    instance._inverse_mount = np.eye(4)
    instance._state_timeout = 0.5
    instance._drag = DragTarget()
    instance._last_state = 0
    instance._last_stamp = None
    instance._translation = Field([0, 0, 0])
    instance._rotation = Field([0, 0, 1, 0])
    mouse = SimpleNamespace(left=False)
    instance._mouse = SimpleNamespace(getState=lambda: mouse)
    marker = SimpleNamespace(getId=lambda: 42, getParentNode=lambda: None)
    selection = SimpleNamespace(node=marker)
    instance._marker_id = 42
    instance._robot = SimpleNamespace(getSelected=lambda: selection.node)
    commands = []
    instance._publisher = SimpleNamespace(publish=lambda msg: commands.append(copy.deepcopy(msg)))
    monkeypatch.setattr(plugin.rclpy, 'spin_once', lambda *args, **kwargs: None)
    return instance, mouse, selection, commands


def feedback(instance, epoch=1):
    state = GizmoState()
    state.actual.header.frame_id = 'base_link'
    state.actual.pose.position.x = 0.1
    state.actual.pose.orientation.w = 1.0
    state.enabled = True
    state.epoch = epoch
    instance._feedback(state)


def test_plugin_publishes_nothing_at_startup_even_with_delayed_fields(monkeypatch):
    instance, mouse, selection, commands = make_plugin(monkeypatch)
    for _ in range(100):
        feedback(instance)
        instance._translation.value = [0, 0, 0]
        instance.step()
    assert commands == []
    assert instance._translation.value == [0.1, 0.0, 0.0]


def test_plugin_mouse_drag_sends_command_and_rejection_stops_it(monkeypatch):
    instance, mouse, selection, commands = make_plugin(monkeypatch)
    feedback(instance)
    instance.step()
    mouse.left = True
    instance._translation.value = [0.2, 0, 0]
    instance.step()
    assert len(commands) == 0
    mouse.left = False
    instance.step()
    assert len(commands) == 1
    assert commands[0].target.pose.position.x == pytest.approx(0.2)
    feedback(instance, epoch=2)
    instance.step()
    assert len(commands) == 1
    assert instance._translation.value == [0.1, 0.0, 0.0]


def test_drag_still_commands_when_webots_has_no_selected_node(monkeypatch):
    """The 3-D handle may not be represented as a selected scene-tree node."""
    instance, mouse, selection, commands = make_plugin(monkeypatch)
    feedback(instance)
    instance.step()
    mouse.left = True
    selection.node = None
    instance._translation.value = [0.2, 0, 0]
    instance.step()
    assert len(commands) == 0
    mouse.left = False
    instance.step()
    assert len(commands) == 1


def test_clicking_other_object_does_not_start_tcp_motion(monkeypatch):
    instance, mouse, selection, commands = make_plugin(monkeypatch)
    feedback(instance)
    instance.step()
    mouse.left = True
    selection.node = None
    instance.step()
    assert commands == []
