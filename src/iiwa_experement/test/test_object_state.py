"""Object ownership rejects unsafe transitions and never mutates on failure."""
from pathlib import Path

import pytest

from iiwa_experement.geometry import load_geometry, tcp_goal, world_target
from iiwa_experement.object_state import CommandError, ObjectState
from iiwa_experement.scenario import load_scenario


PACKAGE = Path(__file__).resolve().parents[1]
SCENES = PACKAGE / 'config' / 'scenes.yaml'
GEOMETRY = PACKAGE / 'config' / 'geometry.yaml'


def make_state(experiment_id, *, read_only=False):
    return ObjectState(
        load_scenario(experiment_id, SCENES),
        load_geometry(GEOMETRY),
        read_only=read_only,
    )


def attach_tool(state, name):
    tool_pose = state.tools[name]['pose']
    tcp = tcp_goal(tool_pose, state.geometry.tool_tcp_to_nodes[name])
    state.attach_tool(name, tcp, tool_pose, request_id=f'attach-{name}')
    return tcp


def grab_cap(state):
    cap_pose = state.cap_pose
    tcp = tcp_goal(cap_pose, state.geometry.cap_tcp_to_node)
    state.grab_cap(tcp, cap_pose, request_id='grab-cap')
    return tcp


def place_cap(state):
    target = world_target(state.spec, state.geometry, 'cap_seat')
    tcp = tcp_goal(target, state.geometry.cap_tcp_to_node)
    object_pose = tcp.compose_relaxed(state.geometry.cap_tcp_to_node)
    state.place_cap(target, tcp, object_pose, request_id='place-cap')


def grab_screw(state, screw_id):
    screw_pose = state.screws[screw_id]['pose']
    tcp = tcp_goal(screw_pose, state.geometry.screw_tcp_to_node)
    state.grab_screw(screw_id, tcp, screw_pose, request_id=f'grab-{screw_id}')
    return tcp


def handoff(state, screw_id, hole_id):
    target = world_target(state.spec, state.geometry, f'hole:{hole_id}')
    tcp = tcp_goal(target, state.geometry.screw_tcp_to_node)
    object_pose = tcp.compose_relaxed(state.geometry.screw_tcp_to_node)
    state.handoff_screw(
        screw_id, hole_id, target, tcp, object_pose,
        request_id=f'handoff-{screw_id}-{hole_id}',
    )


def test_initial_state_for_all_cap_modes():
    free = make_state(1)
    assert free.mounted_tool is None
    assert free.cap_state == 'free'
    assert free.screws == {}
    installed = make_state(11)
    assert installed.cap_state == 'installed'
    assert len(installed.screws) == 10
    absent = make_state(6)
    assert absent.cap_state == 'absent'
    assert absent.cap_pose is None
    assert absent.screws == {}


def test_attach_and_dock_tool_changes_only_that_tool():
    state = make_state(1)
    attach_tool(state, 'vacuum')
    assert state.mounted_tool == 'vacuum'
    assert state.tools['vacuum']['state'] == 'mounted'
    assert state.version == 1
    rack_pose = state.tools['vacuum']['pose']
    attached_pose = rack_pose
    tcp = tcp_goal(rack_pose, state.geometry.tool_tcp_to_nodes['vacuum'])
    state.dock_tool('vacuum', tcp, rack_pose, attached_pose)
    assert state.mounted_tool is None
    assert state.tools['vacuum']['state'] == 'docked'
    assert state.version == 2


def test_tool_requires_contact_and_socket_state():
    state = make_state(1)
    tool_pose = state.tools['vacuum']['pose']
    good = tcp_goal(tool_pose, state.geometry.tool_tcp_to_nodes['vacuum'])
    far = good.compose_relaxed(
        type(good).from_xyz_rpy(
            [0.0, 0.0, 0.05], [0.0, 0.0, 0.0],
            parent=good.parent, child=good.child,
        )
    )
    with pytest.raises(CommandError, match='tcp is'):
        state.attach_tool('vacuum', far, tool_pose)
    assert state.version == 0
    assert state.mounted_tool is None


def test_second_tool_is_rejected_without_state_change():
    state = make_state(16)
    attach_tool(state, 'vacuum')
    version = state.version
    pose = state.tools['magnetic_tool']['pose']
    tcp = tcp_goal(pose, state.geometry.tool_tcp_to_nodes['magnetic_tool'])
    with pytest.raises(CommandError, match='another tool'):
        state.attach_tool('magnetic_tool', tcp, pose)
    assert state.version == version
    assert state.mounted_tool == 'vacuum'


def test_cap_needs_the_vacuum_tool_and_a_free_hand():
    state = make_state(1)
    with pytest.raises(CommandError, match='requires'):
        grab_cap(state)
    assert state.version == 0
    attach_tool(state, 'vacuum')
    grab_cap(state)
    assert state.cap_state == 'held'
    assert state.held_object == 'cap'
    with pytest.raises(CommandError, match='forbidden while'):
        state.grab_cap(state.cap_pose, state.cap_pose)
    assert state.version == 2


def test_place_cap_seats_only_with_the_hatch_held():
    state = make_state(1)
    attach_tool(state, 'vacuum')
    grab_cap(state)
    place_cap(state)
    assert state.cap_state == 'installed'
    assert state.held_object is None
    target = world_target(state.spec, state.geometry, 'cap_seat')
    assert state.cap_pose.position == pytest.approx(target.position, abs=1e-12)
    with pytest.raises(CommandError, match='requires the hatch'):
        place_cap(state)


def test_screws_require_the_magnetic_tool_and_are_consumed_on_handoff():
    state = make_state(11)
    with pytest.raises(CommandError, match='requires'):
        grab_screw(state, 'screw_1')
    attach_tool(state, 'magnetic_tool')
    grab_screw(state, 'screw_1')
    assert state.screws['screw_1']['state'] == 'held'
    handoff(state, 'screw_1', 'hole_1')
    assert state.screws['screw_1']['state'] == 'delivered'
    assert state.screws['screw_1']['hole'] == 'hole_1'
    assert state.holes['hole_1']['state'] == 'empty'
    assert state.holes['hole_1']['screw'] == 'screw_1'
    assert len(state.delivered_pairs()) == 1
    assert sum(
        1 for entry in state.screws.values() if entry['state'] == 'tray'
    ) == 9


def test_repeated_handoff_and_unknown_hole_are_rejected():
    state = make_state(11)
    attach_tool(state, 'magnetic_tool')
    grab_screw(state, 'screw_1')
    handoff(state, 'screw_1', 'hole_1')
    version = state.version
    with pytest.raises(CommandError, match='not held'):
        handoff(state, 'screw_1', 'hole_1')
    assert state.version == version
    grab_screw(state, 'screw_2')
    target = world_target(state.spec, state.geometry, 'hole:hole_2')
    tcp = tcp_goal(target, state.geometry.screw_tcp_to_node)
    object_pose = tcp.compose_relaxed(state.geometry.screw_tcp_to_node)
    with pytest.raises(CommandError, match='unknown hole'):
        state.handoff_screw(
            'screw_2', 'hole_9', target, tcp, object_pose,
        )


def test_a_screw_cannot_be_grabbed_twice():
    state = make_state(11)
    attach_tool(state, 'magnetic_tool')
    grab_screw(state, 'screw_1')
    with pytest.raises(CommandError, match='forbidden while'):
        grab_screw(state, 'screw_2')


def test_read_only_view_rejects_mutation_but_allows_inspection():
    state = make_state(1, read_only=True)
    before = state.snapshot()
    with pytest.raises(CommandError, match='read-only'):
        attach_tool(state, 'vacuum')
    after = state.snapshot()
    assert before['version'] == after['version'] == 0
    assert after['last_error']


def test_rejected_command_preserves_version_and_snapshot():
    state = make_state(1)
    attach_tool(state, 'vacuum')
    snapshot = state.snapshot()
    with pytest.raises(CommandError):
        state.attach_tool('vacuum', state.cap_pose, state.cap_pose)
    assert state.snapshot()['version'] == snapshot['version']
