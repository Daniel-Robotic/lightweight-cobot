"""Skill phases, tool ownership and object handling on a fake backend."""
from __future__ import annotations

import copy
from pathlib import Path

import pytest

from iiwa_experement.geometry import load_geometry, world_target
from iiwa_experement.object_state import transform_json
from iiwa_experement.scenario import load_scenario
from iiwa_llm_plan_executor.moveit_backend import MotionOutcome
from iiwa_llm_plan_executor.schema import PlanNode
from iiwa_llm_plan_executor.skill_context import SkillContext
from iiwa_llm_plan_executor.skills import SkillRegistry


PACKAGE = Path(__file__).resolve().parents[1]
ASSETS = PACKAGE.parent / 'iiwa_experement' / 'config'
SCENES = ASSETS / 'scenes.yaml'
GEOMETRY = load_geometry(ASSETS / 'geometry.yaml')


class FakeMotion:
    def __init__(self, *, success=True):
        self.calls = []
        self.stop_calls = []
        self.reset_calls = 0
        self.success = success

    def plan_and_execute(self, goal, phase, held_object):
        self.calls.append((goal, phase, held_object))
        return MotionOutcome(
            self.success, 'ok' if self.success else 'motion_limit',
            'ok' if self.success else 'rejected by policy',
        )

    def stop(self, reason='stop'):
        self.stop_calls.append(reason)

    def reset_stop(self):
        self.reset_calls += 1


class FakeSupervisor:
    def __init__(self, spec, geometry):
        self.spec = spec
        self.geometry = geometry
        self.commands = []
        self.state = {
            'version': 1,
            'mounted_tool': None,
            'held_object': None,
            'cap': {'state': spec.cap_state, 'pose': None},
            'tools': {},
            'screws': {},
            'holes': {},
            'objects': {},
            'tcp': None,
        }
        if spec.cap_state != 'absent':
            self.state['cap']['pose'] = transform_json(
                world_target(spec, geometry, 'cap')
            )
        for tool in spec.tools:
            self.state['tools'][tool.name] = {
                'state': 'docked',
                'pose': transform_json(world_target(spec, geometry, f'tool:{tool.name}')),
            }
            self.state['objects'][f'grant_{tool.name}'] = (
                self.state['tools'][tool.name]['pose']
            )
        for screw in spec.screws:
            self.state['screws'][screw.name] = {
                'state': 'tray',
                'pose': transform_json(
                    world_target(spec, geometry, f'screw:{screw.name}')
                ),
                'hole': None,
            }
            self.state['objects'][f'grant_screw_{int(screw.name.split("_")[1]):02d}'] = (
                self.state['screws'][screw.name]['pose']
            )
        self.state['objects']['grant_cap'] = self.state['cap']['pose']

    def latest(self):
        return copy.deepcopy(self.state)

    def _advance(self):
        self.state['version'] += 1
        return copy.deepcopy(self.state)

    def command(self, op, object_id='', target=None, target_name=''):
        self.commands.append((op, object_id, target_name))
        if op == 'attach_tool':
            if self.state['mounted_tool'] is not None:
                raise RuntimeError('tool already mounted')
            if object_id not in self.state['tools']:
                raise RuntimeError('unknown tool')
            if self.state['tools'][object_id]['state'] != 'docked':
                raise RuntimeError('tool is not docked')
            self.state['mounted_tool'] = object_id
            self.state['tools'][object_id]['state'] = 'mounted'
        elif op == 'dock_tool':
            if self.state['mounted_tool'] != object_id:
                raise RuntimeError('tool is not mounted')
            self.state['mounted_tool'] = None
            self.state['tools'][object_id]['state'] = 'docked'
            self.state['tools'][object_id]['pose'] = transform_json(
                world_target(self.spec, self.geometry, f'tool:{object_id}')
            )
        elif op == 'grab_cap':
            if self.state['mounted_tool'] != 'vacuum':
                raise RuntimeError('vacuum is not mounted')
            if self.state['held_object'] is not None:
                raise RuntimeError('hand is not free')
            self.state['held_object'] = 'cap'
            self.state['cap']['state'] = 'held'
        elif op == 'place_cap':
            if self.state['held_object'] != 'cap':
                raise RuntimeError('hatch is not held')
            self.state['held_object'] = None
            self.state['cap']['state'] = 'installed'
            self.state['cap']['pose'] = transform_json(target)
        elif op == 'grab_screw':
            if self.state['mounted_tool'] != 'magnetic_tool':
                raise RuntimeError('magnet is not mounted')
            if self.state['screws'][object_id]['state'] != 'tray':
                raise RuntimeError('screw is unavailable')
            self.state['held_object'] = object_id
            self.state['screws'][object_id]['state'] = 'held'
        elif op == 'handoff_screw':
            if self.state['held_object'] != object_id:
                raise RuntimeError('screw is not held')
            if self.state['mounted_tool'] != 'magnetic_tool':
                raise RuntimeError('magnet is not mounted')
            self.state['held_object'] = None
            self.state['screws'][object_id]['state'] = 'delivered'
            self.state['screws'][object_id]['hole'] = target_name
            self.state['screws'][object_id]['pose'] = transform_json(target)
            self.state['holes'].setdefault(target_name, {'state': 'empty', 'screw': None})
            self.state['holes'][target_name]['screw'] = object_id
        elif op == 'inspect':
            pass
        else:
            raise RuntimeError(f'unknown op {op}')
        return self._advance()


class FakeScene:
    def __init__(self):
        self.events = []

    def attach_tool(self, tool):
        self.events.append(('attach_tool', tool))

    def dock_tool(self, tool):
        self.events.append(('dock_tool', tool))

    def attach_cap(self):
        self.events.append(('attach_cap',))

    def attach_screw(self, screw):
        self.events.append(('attach_screw', screw))

    def place_cap(self, seat):
        self.events.append(('place_cap', seat))

    def handoff_screw(self, screw):
        self.events.append(('handoff_screw', screw))


def make_registry(experiment_id=20, *, motion=None, adapter=None):
    spec = load_scenario(experiment_id, SCENES)
    supervisor = FakeSupervisor(spec, GEOMETRY)
    motion = motion or FakeMotion()
    registry = SkillRegistry(motion, supervisor, scene_adapter=adapter or FakeScene())
    context = SkillContext(spec=spec, geometry=GEOMETRY, scene_revision=1)
    context.scene_adapter = registry.scene_adapter
    context.frame_provider = lambda name: 5.0
    return registry, context, motion, supervisor


def node(node_id, skill, obj='', target='', params=None):
    return PlanNode(
        id=node_id, skill=skill, object=obj, target=target,
        params=params or {},
    )


def phases(motion):
    return [phase for _, phase, _ in motion.calls]


def test_approach_goes_above_the_target_and_saves_the_height():
    registry, context, motion, _ = make_registry()
    result = registry.execute(node('a', 'approach', 'cap'), context)
    assert result.event == 'success'
    assert phases(motion) == ['approach']
    safe = motion.calls[0][0]
    assert context.approach_pose is not None
    assert context.phase == 'approach'
    assert safe.xyz[2] > context.contact_pose.xyz[2] if context.contact_pose else True

    repeated = registry.execute(node('a', 'approach', 'cap'), context)
    assert repeated.event == 'success'
    assert phases(motion) == ['approach']


def test_mounted_tool_is_reported_as_carried_during_motion():
    registry, context, motion, _ = make_registry()
    context.mounted_tool = 'vacuum'
    result = registry.execute(node('a', 'approach', 'cap'), context)
    assert result.event == 'success'
    assert motion.calls[0][2] == 'vacuum'


def test_approach_refuses_to_use_manifest_pose_when_supervisor_pose_is_missing():
    registry, context, motion, supervisor = make_registry()
    supervisor.state['cap']['pose'] = None
    result = registry.execute(node('a', 'approach', 'cap'), context)
    assert result.event == 'failed'
    assert motion.calls == []


def test_grab_keeps_target_in_collision_scene_during_approach():
    class RecordingScene(FakeScene):
        def prepare_grab_cap(self, object_id):
            self.events.append(('prepare_grab_cap', object_id))

    adapter = RecordingScene()
    registry, context, motion, supervisor = make_registry(
        motion=FakeMotion(success=False), adapter=adapter,
    )
    context.mounted_tool = 'vacuum'
    supervisor.state['mounted_tool'] = 'vacuum'
    result = registry.execute(node('g', 'grab', 'cap'), context)
    assert result.event == 'failed'
    assert phases(motion) == ['approach']
    assert adapter.events == []


def test_grab_restores_target_when_lowering_fails():
    class RecordingScene(FakeScene):
        def prepare_grab_cap(self, object_id):
            self.events.append(('prepare_grab_cap', object_id))

        def restore_part(self, object_id):
            self.events.append(('restore_part', object_id))

    class FailOnLower(FakeMotion):
        def plan_and_execute(self, goal, phase, held_object):
            self.calls.append((goal, phase, held_object))
            return MotionOutcome(
                phase != 'lower',
                'ok' if phase != 'lower' else 'motion_limit',
                'ok' if phase != 'lower' else 'rejected by policy',
            )

    adapter = RecordingScene()
    registry, context, motion, supervisor = make_registry(
        motion=FailOnLower(), adapter=adapter,
    )
    context.mounted_tool = 'vacuum'
    supervisor.state['mounted_tool'] = 'vacuum'
    result = registry.execute(node('g', 'grab', 'cap'), context)
    assert result.event == 'failed'
    assert phases(motion) == ['approach', 'lower']
    assert adapter.events == [
        ('prepare_grab_cap', 'cap'),
        ('restore_part', 'cap'),
    ]


def test_grab_cap_runs_missing_phases_and_lifts_to_saved_height():
    registry, context, motion, supervisor = make_registry()
    context.mounted_tool = 'vacuum'
    supervisor.state['mounted_tool'] = 'vacuum'
    supervisor.state['tools']['vacuum']['state'] = 'mounted'
    result = registry.execute(node('g', 'grab', 'cap'), context)
    assert result.event == 'success'
    assert phases(motion) == ['approach', 'lower', 'retreat']
    assert supervisor.commands == [('grab_cap', 'cap', '')]
    assert context.held_object == 'cap'
    assert context.phase == 'lifted'
    assert ('attach_cap',) in registry.scene_adapter.events


def test_explicit_phases_are_not_repeated_by_grab():
    registry, context, motion, supervisor = make_registry()
    context.mounted_tool = 'vacuum'
    supervisor.state['mounted_tool'] = 'vacuum'
    assert registry.execute(node('a', 'approach', 'cap'), context).event == 'success'
    assert registry.execute(node('l', 'lower', 'cap'), context).event == 'success'
    assert registry.execute(node('g', 'grab', 'cap'), context).event == 'success'
    assert phases(motion) == ['approach', 'lower', 'retreat']
    assert registry.execute(node('up', 'lift', 'cap'), context).event == 'success'
    assert phases(motion) == ['approach', 'lower', 'retreat']


def test_grab_cap_without_vacuum_reports_grasp_failed_without_motion_or_command():
    registry, context, motion, supervisor = make_registry()
    before = copy.deepcopy(supervisor.state)
    result = registry.execute(node('g', 'grab', 'cap'), context)
    assert result.event == 'grasp_failed'
    assert result.details['reason'] == 'required tool vacuum is not mounted'
    assert result.details['required_tool'] == 'vacuum'
    assert motion.calls == []
    assert supervisor.commands == []
    assert registry.scene_adapter.events == []
    assert supervisor.state['cap']['state'] == before['cap']['state']
    assert supervisor.state['held_object'] is None
    assert context.held_object is None
    assert context.mounted_tool is None


def test_supervisor_state_overrides_stale_context_for_tool_checks():
    registry, context, motion, supervisor = make_registry()
    context.mounted_tool = 'vacuum'  # stale memory; supervisor says no tool
    result = registry.execute(node('g', 'grab', 'cap'), context)
    assert result.event == 'grasp_failed'
    assert result.details['reason'] == 'required tool vacuum is not mounted'
    assert motion.calls == []
    assert supervisor.commands == []
    assert context.mounted_tool is None


def test_grab_screw_without_magnetic_tool_reports_grasp_failed_without_motion():
    registry, context, motion, supervisor = make_registry()
    before = copy.deepcopy(supervisor.state)
    result = registry.execute(node('g', 'grab', 'screw_1'), context)
    assert result.event == 'grasp_failed'
    assert result.details['reason'] == 'required tool magnetic_tool is not mounted'
    assert result.details['required_tool'] == 'magnetic_tool'
    assert motion.calls == []
    assert supervisor.commands == []
    assert registry.scene_adapter.events == []
    assert supervisor.state['screws']['screw_1']['state'] == before['screws']['screw_1']['state']
    assert supervisor.state['held_object'] is None
    assert context.held_object is None


def test_tool_grab_docks_at_the_socket_and_lifts_out():
    registry, context, motion, supervisor = make_registry()
    result = registry.execute(node('g', 'grab', 'vacuum'), context)
    assert result.event == 'success'
    assert phases(motion) == ['approach', 'lower', 'retreat']
    assert supervisor.commands == [('attach_tool', 'vacuum', '')]
    assert context.mounted_tool == 'vacuum'
    assert ('attach_tool', 'vacuum') in registry.scene_adapter.events


def test_bare_release_of_the_suction_cup_does_not_move_or_detach():
    registry, context, motion, supervisor = make_registry()
    context.mounted_tool = 'vacuum'
    supervisor.state['mounted_tool'] = 'vacuum'
    result = registry.execute(node('r', 'release', 'vacuum', ''), context)
    assert result.event == 'failed'
    assert motion.calls == []
    assert supervisor.commands == []
    assert context.mounted_tool == 'vacuum'


def test_release_at_socket_requires_the_explicit_transfer():
    registry, context, motion, supervisor = make_registry()
    context.mounted_tool = 'vacuum'
    supervisor.state['mounted_tool'] = 'vacuum'
    result = registry.execute(
        node('r', 'release', 'vacuum', 'vacuum_socket'), context,
    )
    assert result.event == 'failed'
    assert motion.calls == []
    assert context.mounted_tool == 'vacuum'


def test_explicit_transfer_then_dock_keeps_tool_in_patron_until_dock():
    registry, context, motion, supervisor = make_registry()
    assert registry.execute(node('g', 'grab', 'vacuum'), context).event == 'success'
    transfer = registry.execute(
        node('t', 'transfer', 'vacuum', 'work'), context,
    )
    assert transfer.event == 'success'
    assert context.mounted_tool == 'vacuum'
    assert phases(motion)[-1] == 'transfer'

    to_socket = registry.execute(
        node('t2', 'transfer', 'vacuum', 'vacuum_socket'), context,
    )
    assert to_socket.event == 'success'
    dock = registry.execute(
        node('d', 'dock_tool', 'vacuum', 'vacuum_socket'), context,
    )
    assert dock.event == 'success'
    assert context.mounted_tool is None
    assert supervisor.commands[-1][0] == 'dock_tool'
    assert ('dock_tool', 'vacuum') in registry.scene_adapter.events
    assert phases(motion)[-2:] == ['lower', 'retreat']


def test_cap_transfer_align_and_release_place_the_hatch():
    registry, context, motion, supervisor = make_registry()
    context.mounted_tool = 'vacuum'
    context.held_object = 'cap'
    supervisor.state['mounted_tool'] = 'vacuum'
    supervisor.state['held_object'] = 'cap'
    supervisor.state['cap']['state'] = 'held'
    assert registry.execute(
        node('t', 'transfer', 'cap', 'hatch_seat'), context,
    ).event == 'success'
    assert registry.execute(node('a', 'align', 'cap', 'hatch_seat'), context).event == 'success'
    before = len(motion.calls)
    result = registry.execute(node('r', 'release', 'cap', 'hatch_seat'), context)
    assert result.event == 'success'
    assert phases(motion)[before:] == ['lower', 'retreat']
    assert context.held_object is None
    assert supervisor.state['cap']['state'] == 'installed'
    assert ('place_cap',) in [tuple(event[:1]) for event in registry.scene_adapter.events]
    assert supervisor.commands[-1][0] == 'place_cap'


def test_release_lifts_axially_before_any_lateral_motion():
    registry, context, motion, _ = make_registry()
    context.mounted_tool = 'vacuum'
    context.held_object = 'cap'
    context.transfer_target = 'hatch_seat'
    seat = world_target(context.spec, context.geometry, 'cap_seat')
    from iiwa_experement.geometry import tcp_goal
    contact = tcp_goal(seat, context.geometry.cap_tcp_to_node)
    from iiwa_llm_plan_executor.skills import _above
    safe = _above(contact, context.geometry.approach_gap)
    context.mark_approach(safe, ('cap', 'hatch_seat'))
    context.mark_contact(contact)
    registry.supervisor.state['mounted_tool'] = 'vacuum'
    registry.supervisor.state['held_object'] = 'cap'
    registry.supervisor.state['cap']['state'] = 'held'
    result = registry.execute(node('r', 'release', 'cap', 'hatch_seat'), context)
    assert result.event == 'success'
    assert phases(motion)[-1] == 'retreat'
    assert motion.calls[-1][1] == 'retreat'
    assert motion.calls[-1][0].xyz[2] > contact.xyz[2]


def test_transfer_from_contact_lifts_axially_before_lateral_ptp():
    registry, context, motion, supervisor = make_registry()
    context.mounted_tool = 'vacuum'
    context.held_object = 'cap'
    supervisor.state['mounted_tool'] = 'vacuum'
    supervisor.state['held_object'] = 'cap'
    supervisor.state['cap']['state'] = 'held'
    seat = world_target(context.spec, context.geometry, 'cap_seat')
    from iiwa_experement.geometry import tcp_goal
    from iiwa_llm_plan_executor.skills import _above
    contact = tcp_goal(seat, context.geometry.cap_tcp_to_node)
    safe = _above(contact, context.geometry.approach_gap)
    context.mark_approach(safe, ('cap', 'hatch_seat'))
    context.mark_contact(contact)
    result = registry.execute(
        node('t', 'transfer', 'cap', 'hatch_seat'), context,
    )
    assert result.event == 'success'
    assert phases(motion) == ['retreat', 'transfer']
    assert motion.calls[0][1] == 'retreat'


def test_dispense_moves_along_the_segment_without_changing_ownership():
    registry, context, motion, supervisor = make_registry()
    context.mounted_tool = 'sealant_tool'
    supervisor.state['mounted_tool'] = 'sealant_tool'
    supervisor.state['tools']['sealant_tool']['state'] = 'mounted'
    result = registry.execute(
        node('d', 'dispense', 'side_1', params={
            'start_fraction': 0.0, 'end_fraction': 1.0,
        }),
        context,
    )
    assert result.event == 'success'
    assert phases(motion) == ['approach', 'lower', 'dispense', 'retreat']
    assert context.mounted_tool == 'sealant_tool'
    assert context.held_object is None


def test_screw_handoff_delivers_to_the_operator_and_keeps_the_hole_empty():
    registry, context, motion, supervisor = make_registry()
    context.mounted_tool = 'magnetic_tool'
    context.held_object = 'screw_1'
    supervisor.state['mounted_tool'] = 'magnetic_tool'
    supervisor.state['held_object'] = 'screw_1'
    supervisor.state['screws']['screw_1']['state'] = 'held'
    assert registry.execute(
        node('t', 'transfer', 'screw_1', 'hole_1'), context,
    ).event == 'success'
    before = len(motion.calls)
    result = registry.execute(
        node('h', 'handoff_screw', 'screw_1', 'hole_1'), context,
    )
    assert result.event == 'success'
    assert phases(motion)[before:] == ['lower', 'retreat']
    assert supervisor.state['screws']['screw_1']['state'] == 'delivered'
    assert supervisor.state['screws']['screw_1']['hole'] == 'hole_1'
    assert supervisor.state['holes']['hole_1']['state'] == 'empty'
    assert ('handoff_screw', 'screw_1') in registry.scene_adapter.events
    assert result.details['installed'] is False
    assert result.details['tightened'] is False


def test_detect_withdraws_axially_then_uses_fresh_frames():
    registry, context, motion, _ = make_registry()
    context.mark_approach(
        GEOMETRY.safe_poses['work'], ('cap', ''),
    )
    context.mark_contact(GEOMETRY.safe_poses['work'])
    result = registry.execute(node('d', 'detect', 'cap'), context)
    assert result.event == 'found'
    assert phases(motion)[0] == 'retreat'
    assert phases(motion)[1] == 'transfer'
    assert context.phase == 'free'

    context.frame_provider = lambda name: None
    assert registry.execute(node('d', 'detect', 'cap'), context).event == 'failed'

    context.frame_provider = lambda name: 5.0
    context.close_inspection_objects = frozenset({'cap'})
    context.frame_provider = lambda name: 5.0 if name == 'top' else None
    assert registry.execute(node('d', 'detect', 'cap'), context).event == 'failed'


def test_capture_requires_a_fresh_frame():
    registry, context, _, _ = make_registry()
    assert registry.execute(node('c', 'capture', 'top'), context).event == 'success'
    context.frame_provider = lambda name: None
    assert registry.execute(node('c', 'capture', 'top'), context).event == 'failed'


def test_inspect_state_distinguishes_empty_holding_placed_unknown():
    registry, context, _, supervisor = make_registry()

    supervisor.state['cap']['state'] = 'free'
    assert registry.execute(node('i', 'inspect_state', 'cap'), context).event == 'empty'

    supervisor.state['held_object'] = 'cap'
    supervisor.state['cap']['state'] = 'held'
    assert registry.execute(node('i', 'inspect_state', 'cap'), context).event == 'holding'

    supervisor.state['held_object'] = None
    supervisor.state['cap']['state'] = 'installed'
    assert registry.execute(node('i', 'inspect_state', 'cap'), context).event == 'placed'

    supervisor.state['cap']['state'] = 'bogus'
    assert registry.execute(node('i', 'inspect_state', 'cap'), context).event == 'unknown'


def test_inspect_state_recovers_ownership_after_a_failed_grasp():
    registry, context, _, supervisor = make_registry()
    supervisor.state['held_object'] = None
    supervisor.state['cap']['state'] = 'free'
    context.ownership_unknown = True
    result = registry.execute(node('i', 'inspect_state', 'cap'), context)
    assert result.event == 'empty'
    assert context.ownership_unknown is False


def test_verify_measures_the_placed_hatch():
    registry, context, _, supervisor = make_registry()
    supervisor.state['cap']['state'] = 'installed'
    supervisor.state['cap']['pose'] = transform_json(
        world_target(context.spec, context.geometry, 'cap_seat')
    )
    result = registry.execute(
        node('v', 'verify', 'cap', 'hatch_seat'), context,
    )
    assert result.event == 'ok'

    shifted = world_target(context.spec, context.geometry, 'cap_seat')
    shifted = type(shifted)(
        xyz=(shifted.xyz[0] + 0.02, shifted.xyz[1], shifted.xyz[2]),
        rotation=shifted.rotation,
        parent=shifted.parent,
        child=shifted.child,
    )
    supervisor.state['cap']['pose'] = transform_json(shifted)
    assert registry.execute(
        node('v', 'verify', 'cap', 'hatch_seat'), context,
    ).event == 'not_ok'


def test_stop_preserves_ownership_and_wait_scene_clear_reports_timeout():
    registry, context, motion, _ = make_registry()
    context.held_object = 'cap'
    assert registry.execute(node('s', 'stop', 'robot'), context).event == 'success'
    assert motion.stop_calls == ['graph stop node']
    assert motion.reset_calls == 1
    assert context.held_object == 'cap'
    context.scene_clear_provider = lambda: False
    assert registry.execute(
        node('w', 'wait_scene_clear', 'workspace'), context,
    ).event == 'timeout'
    context.scene_clear_provider = lambda: True
    assert registry.execute(
        node('w', 'wait_scene_clear', 'workspace'), context,
    ).event == 'clear'


def test_unknown_skill_and_backend_failure_are_not_successes():
    registry, context, motion, _ = make_registry(motion=FakeMotion(success=False))
    assert registry.execute(node('x', 'tighten', 'hole_1'), context).event == 'failed'
    assert registry.execute(node('a', 'approach', 'cap'), context).event == 'failed'
