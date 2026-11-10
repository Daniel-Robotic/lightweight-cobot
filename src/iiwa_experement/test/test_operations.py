"""Operation sequences are visible, ordered and fail closed."""
import math
from pathlib import Path

import pytest

from iiwa_experement.geometry import load_geometry, world_target
from iiwa_experement.motion import MotionResult
from iiwa_experement.operations import OperationError, ReferenceOperations
from iiwa_experement.scenario import load_scenario


PACKAGE = Path(__file__).resolve().parents[1]
SCENES = PACKAGE / 'config' / 'scenes.yaml'
GEOMETRY = PACKAGE / 'config' / 'geometry.yaml'


class FakeSupervisor:
    def __init__(self, spec, geometry):
        self.spec = spec
        self.geometry = geometry
        self.commands = []
        self.latest = {
            'cap': {'state': spec.cap_state},
            'screws': {},
        }
        self.fail_on = None

    def object_pose(self, name):
        mapping = {
            'grant_cap': 'cap',
            **{f'grant_{tool}': f'tool:{tool}' for tool in self.spec.tool_names},
            **{
                f'grant_screw_{index:02d}': f'screw:screw_{index}'
                for index in range(1, 11)
            },
        }
        target = mapping[name]
        return world_target(self.spec, self.geometry, target)

    def command(self, op, object_id='', **kwargs):
        if self.fail_on == op:
            raise OperationError(f'{op} rejected')
        self.commands.append((op, object_id, kwargs))
        return self.latest


class FakeBridge:
    def __init__(self):
        self.attached = []
        self.detached = []
        self.removed = []
        self.restored = []

    def attach(self, spec):
        self.attached.append(spec.object_id)

    def detach(self, object_id, pose):
        self.detached.append((object_id, pose))

    def remove(self, object_id):
        self.removed.append(object_id)
        if object_id in self.attached:
            self.attached.remove(object_id)

    def restore_world(self, object_id):
        if object_id in self.attached:
            self.attached.remove(object_id)
        self.restored.append(object_id)

    def add_world(self, object_id, box, pose):
        if object_id in self.attached:
            self.attached.remove(object_id)
        self.added_world = getattr(self, 'added_world', [])
        self.added_world.append((object_id, pose))

    def expected_sets(self):
        return set(), set()

    def assert_synchronized(self, *args, **kwargs):
        return None


class FakeMotion:
    def __init__(self):
        self.moves = []
        self.named = []
        self.fail_labels = set()
        self.joint_moves = []
        self.joint_values = (0.0,) * 7
        self.fail_positive_joint = False

    def move_pose(self, goal, planner='ptp', speed=0.2, label=''):
        self.moves.append((label, planner, speed, goal))
        if any(label.startswith(prefix) for prefix in self.fail_labels):
            return MotionResult(False, 'Планирование не удалось: injected failure', label)
        return MotionResult(True, 'ok', label, True, goal, goal)

    def move_named(self, name, speed=0.2):
        self.named.append((name, speed))
        return MotionResult(True, 'ok', f'move_named({name})')

    def joint_positions(self):
        return self.joint_values

    def move_joints(self, joints, speed=0.2, label=''):
        self.joint_moves.append((label, tuple(joints)))
        if self.fail_positive_joint and joints[6] > 0:
            return MotionResult(False, 'injected joint failure', label)
        self.joint_values = tuple(joints)
        return MotionResult(True, 'ok', label)

    def reset_stop(self):
        return None

    def wait_stationary(self, *args, **kwargs):
        return True


def make_operations(experiment_id):
    spec = load_scenario(experiment_id, SCENES)
    geometry = load_geometry(GEOMETRY)
    supervisor = FakeSupervisor(spec, geometry)
    bridge = FakeBridge()
    motion = FakeMotion()
    operations = ReferenceOperations(
        node=None,
        spec=spec,
        geometry=geometry,
        supervisor=supervisor,
        bridge=bridge,
        motion=motion,
    )
    return operations, supervisor, bridge, motion, geometry


def labels(motion):
    return [label for label, _, _, _ in motion.moves]


def test_take_tool_follows_the_visible_axial_route():
    operations, supervisor, bridge, motion, geometry = make_operations(1)
    tip = world_target(operations.spec, geometry, 'tool:vacuum')
    expected = tip.compose_relaxed(geometry.tool_tcp_to_nodes['vacuum'].inverse())
    result = operations.take_tool('vacuum')
    assert result.success
    assert labels(motion) == [
        'take_tool(vacuum):transfer',
        'take_tool(vacuum):approach',
        'take_tool(vacuum):contact',
        'take_tool(vacuum):lift',
        'take_tool(vacuum):exit',
    ]
    assert supervisor.commands[0][:2] == ('attach_tool', 'vacuum')
    assert bridge.attached == ['grant_vacuum']
    contact_goal = motion.moves[2][3]
    assert contact_goal.xyz == pytest.approx(expected.xyz, abs=1e-9)
    assert motion.moves[0][1] == 'ptp'
    assert motion.moves[1][1] == 'ptp'
    assert motion.moves[2][1] == 'lin'


def test_dock_tool_returns_the_tool_to_its_socket():
    operations, supervisor, bridge, motion, geometry = make_operations(1)
    result = operations.dock_tool('vacuum')
    assert result.success
    assert supervisor.commands[0][:2] == ('dock_tool', 'vacuum')
    assert bridge.restored == ['grant_vacuum']
    assert labels(motion)[-1] == 'dock_tool(vacuum):exit'


def test_cap_pick_and_place_use_the_hatch_node_transform():
    operations, supervisor, bridge, motion, geometry = make_operations(1)
    assert operations.pick_cap().success
    assert bridge.attached == ['grant_cap']
    assert supervisor.commands[0][:2] == ('grab_cap', 'cap')
    assert operations.place_cap().success
    assert bridge.added_world[0][0] == 'grant_cap'
    assert supervisor.commands[1][0] == 'place_cap'


def test_single_sealant_side_lowers_and_lifts():
    operations, supervisor, bridge, motion, geometry = make_operations(6)
    assert operations.dispense('side_1').success
    assert labels(motion) == [
        'dispense(side_1):transfer',
        'dispense(side_1):approach',
        'dispense(side_1):contact',
        'dispense(side_1):path',
        'dispense(side_1):retreat',
        'dispense(side_1):clear',
    ]
    assert operations.completed_sealant == {'side_1'}
    assert operations.inspect_sealant()['ok'] is True


def test_contour_does_not_lift_between_adjacent_sides():
    operations, supervisor, bridge, motion, geometry = make_operations(8)
    assert operations.dispense_contour(
        ['side_1', 'side_2', 'side_3', 'side_4']
    ).success
    names = labels(motion)
    assert names[:3] == [
        'dispense_contour:start:transfer',
        'dispense_contour:start:approach',
        'dispense_contour:start:contact',
    ]
    assert names[3:7] == [
        'dispense_contour:side_1',
        'dispense_contour:side_2',
        'dispense_contour:side_3',
        'dispense_contour:side_4',
    ]
    assert 'dispense_contour:side_1' in names
    assert names[-2:] == [
        'dispense_contour:retreat', 'dispense_contour:clear',
    ]
    assert operations.completed_sealant == {
        'side_1', 'side_2', 'side_3', 'side_4',
    }


def test_screw_pick_and_handoff_are_one_motion_and_one_removal():
    operations, supervisor, bridge, motion, geometry = make_operations(11)
    assert operations.pick_screw('screw_1').success
    assert bridge.attached == ['grant_screw_01']
    assert supervisor.commands[0][:2] == ('grab_screw', 'screw_1')
    assert operations.handoff_screw('screw_1', 'hole_1').success
    assert supervisor.commands[1][0] == 'handoff_screw'
    assert supervisor.commands[1][2]['target_name'] == 'hole_1'
    assert bridge.removed == ['grant_screw_01', 'grant_screw_01']
    assert bridge.attached == []


def test_tighten_hole_is_a_visible_revolution_without_a_screw():
    operations, supervisor, bridge, motion, geometry = make_operations(14)
    assert operations.tighten_hole('hole_1').success
    names = labels(motion)
    assert names[:3] == [
        'tighten_hole(hole_1):transfer',
        'tighten_hole(hole_1):approach',
        'tighten_hole(hole_1):contact',
    ]
    assert motion.joint_moves
    label, joints = motion.joint_moves[0]
    assert label == 'tighten_hole(hole_1):joint_twist'
    assert joints[:6] == (0.0,) * 6
    assert joints[6] == pytest.approx(math.radians(45.0))
    # The flange is untwisted before the axial retreat, so joint 7 does not
    # accumulate twist across holes.
    untwist_label, untwist_joints = motion.joint_moves[1]
    assert untwist_label == 'tighten_hole(hole_1):joint_untwist'
    assert untwist_joints == (0.0,) * 7
    assert names[-2:] == [
        'tighten_hole(hole_1):retreat', 'tighten_hole(hole_1):clear',
    ]
    # The operation is a simulation only: no supervisor command changes state.
    assert supervisor.commands == []
    assert bridge.attached == []


def test_screw_pick_reconfigures_through_work_when_a_cell_is_blocked():
    operations, supervisor, bridge, motion, geometry = make_operations(11)
    motion.fail_labels.add('pick_screw(screw_1):transfer')
    assert operations.pick_screw('screw_1').success
    assert motion.named[-1][0] == 'work'
    assert 'pick_screw(screw_1):recovery:transfer' in labels(motion)
    assert supervisor.commands[0][:2] == ('grab_screw', 'screw_1')


def test_screw_pick_stops_instead_of_dragging_when_the_lift_is_rejected():
    operations, supervisor, bridge, motion, geometry = make_operations(11)
    motion.fail_labels.add('pick_screw(screw_1):retreat')
    with pytest.raises(OperationError):
        operations.pick_screw('screw_1')
    assert operations.events[-1]['phase'] == 'failed'


def test_handoff_screw_lifts_axially_before_the_next_move():
    operations, supervisor, bridge, motion, geometry = make_operations(11)
    assert operations.pick_screw('screw_1').success
    motion.moves.clear()
    assert operations.handoff_screw('screw_1', 'hole_1').success
    names = labels(motion)
    assert names[-2:] == [
        'handoff_screw(screw_1,hole_1):retreat',
        'handoff_screw(screw_1,hole_1):clear',
    ]


def test_tighten_hole_reverses_when_the_flange_limit_blocks_one_direction():
    operations, supervisor, bridge, motion, geometry = make_operations(14)
    motion.fail_positive_joint = True
    assert operations.tighten_hole('hole_1').success
    assert len(motion.joint_moves) == 3
    assert motion.joint_moves[1][1][6] == pytest.approx(math.radians(-45.0))
    # The untwist returns the flange to the contact configuration.
    assert motion.joint_moves[2][0] == 'tighten_hole(hole_1):joint_untwist'
    assert motion.joint_moves[2][1][6] == pytest.approx(0.0)


def test_failed_motion_stops_the_operation_and_emits_failure():
    operations, supervisor, bridge, motion, geometry = make_operations(1)
    motion.fail_labels.update({
        'take_tool(vacuum):contact',
        'take_tool(vacuum):recovery:contact',
    })
    with pytest.raises(OperationError):
        operations.take_tool('vacuum')
    assert operations.events[-1]['phase'] == 'failed'
    assert bridge.attached == []


def test_failed_supervisor_command_is_not_ignored():
    operations, supervisor, bridge, motion, geometry = make_operations(1)
    supervisor.fail_on = 'attach_tool'
    with pytest.raises(OperationError):
        operations.take_tool('vacuum')
    assert bridge.attached == []
