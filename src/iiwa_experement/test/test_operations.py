"""Operation sequences are visible, ordered and fail closed."""
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

    def attach(self, spec):
        self.attached.append(spec.object_id)

    def detach(self, object_id, pose):
        self.detached.append((object_id, pose))

    def remove(self, object_id):
        self.removed.append(object_id)

    def add_world(self, object_id, box, pose):
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

    def move_pose(self, goal, planner='ptp', speed=0.2, label=''):
        self.moves.append((label, planner, speed, goal))
        if label in self.fail_labels:
            return MotionResult(False, 'injected failure', label)
        return MotionResult(True, 'ok', label, True, goal, goal)

    def move_named(self, name, speed=0.2):
        self.named.append((name, speed))
        return MotionResult(True, 'ok', f'move_named({name})')


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
        'take_tool(vacuum):retreat',
        'take_tool(vacuum):clear',
    ]
    assert supervisor.commands[0][:2] == ('attach_tool', 'vacuum')
    assert bridge.attached == []
    contact_goal = motion.moves[2][3]
    assert contact_goal.xyz == pytest.approx(expected.xyz, abs=1e-9)
    assert motion.moves[0][1] == 'ompl'
    assert motion.moves[1][1] == 'ompl'
    assert motion.moves[2][1] == 'lin'


def test_dock_tool_returns_the_tool_to_its_socket():
    operations, supervisor, bridge, motion, geometry = make_operations(1)
    result = operations.dock_tool('vacuum')
    assert result.success
    assert supervisor.commands[0][:2] == ('dock_tool', 'vacuum')
    assert bridge.added_world[0][0] == 'grant_vacuum'
    assert labels(motion)[-1] == 'dock_tool(vacuum):clear'


def test_cap_pick_and_place_use_the_hatch_node_transform():
    operations, supervisor, bridge, motion, geometry = make_operations(1)
    assert operations.pick_cap().success
    assert bridge.attached == []
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
    assert bridge.attached == []
    assert supervisor.commands[0][:2] == ('grab_screw', 'screw_1')
    assert operations.handoff_screw('screw_1', 'hole_1').success
    assert supervisor.commands[1][0] == 'handoff_screw'
    assert supervisor.commands[1][2]['target_name'] == 'hole_1'
    assert bridge.removed == ['grant_screw_01', 'grant_screw_01']
    assert bridge.attached == []


def test_failed_motion_stops_the_operation_and_emits_failure():
    operations, supervisor, bridge, motion, geometry = make_operations(1)
    motion.fail_labels.add('take_tool(vacuum):contact')
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
