"""All twenty routes differ by task and use only the agreed skills."""
from pathlib import Path

from iiwa_experement.reference_scenarios import (
    ReferenceStep,
    build_reference_sequence,
    execute_sequence,
)
from iiwa_experement.scenario import load_scenario
from iiwa_experement.trace import RunResult


PACKAGE = Path(__file__).resolve().parents[1]
SCENES = PACKAGE / 'config' / 'scenes.yaml'


def spec(experiment_id):
    return load_scenario(experiment_id, SCENES)


def route(experiment_id):
    return build_reference_sequence(spec(experiment_id))


def kinds(experiment_id):
    return [step.kind for step in route(experiment_id)]


def test_every_route_is_nonempty_and_returns_home():
    for experiment_id in range(1, 21):
        names = kinds(experiment_id)
        assert names
        assert 'tighten' not in names
        assert 'seat_screw' not in names
        assert 'move_named' in names
        assert any(
            step.kind == 'move_named' and step.args == ('home',)
            for step in route(experiment_id)
        )
        assert names[-2:] == ['dock_tool', 'move_named'] or names[-1] == 'move_named' or 'dock_tool' in names


def test_routes_start_at_work_and_only_end_at_home():
    for experiment_id in range(1, 21):
        steps = route(experiment_id)
        assert steps[0].kind == 'move_named'
        assert steps[0].args == ('work',)
        assert steps[-2].kind == 'move_named'
        assert steps[-2].args == ('work',)
        assert steps[-1].kind == 'move_named'
        assert steps[-1].args == ('home',)
        homes = [
            index for index, step in enumerate(steps)
            if step.kind == 'move_named' and step.args == ('home',)
        ]
        assert homes == [len(steps) - 1]
        # The 'transport' pose is forbidden in the reference routes.
        assert not any(
            step.kind == 'move_named' and step.args == ('transport',)
            for step in steps
        )


def test_cap_experiments_use_the_vacuum_tool():
    for experiment_id in range(1, 6):
        names = kinds(experiment_id)
        assert 'take_tool' in names
        take = next(step for step in route(experiment_id) if step.kind == 'take_tool')
        assert take.args == ('vacuum',)
        assert 'pick_cap' in names
        assert 'place_cap' in names
        assert 'inspect_cap' in names


def test_sealant_experiments_use_only_the_sealant_tool():
    for experiment_id in range(6, 10):
        names = kinds(experiment_id)
        takes = [step.args[0] for step in route(experiment_id) if step.kind == 'take_tool']
        assert takes == ['sealant_tool']
        assert any(name in names for name in ('dispense', 'dispense_contour'))
        assert 'inspect_sealant' in names


def test_full_assembly_tool_order_is_sealant_cap_screws():
    for experiment_id in range(16, 21):
        takes = [step.args[0] for step in route(experiment_id) if step.kind == 'take_tool']
        assert takes == ['sealant_tool', 'vacuum', 'magnetic_tool', 'torque_tool']


def test_tightening_simulation_matches_the_delivered_holes():
    assert not any(
        step.kind == 'tighten_hole' for step in route(11)
    )
    for experiment_id in range(14, 21):
        tighten = [
            step for step in route(experiment_id) if step.kind == 'tighten_hole'
        ]
        handoffs = [
            step for step in route(experiment_id) if step.kind == 'handoff_screw'
        ]
        assert [step.args[0] for step in tighten] == [
            step.args[1] for step in handoffs
        ]
    for experiment_id in (12, 13):
        assert not any(
            step.kind == 'tighten_hole' for step in route(experiment_id)
        )


def test_screw_counts_and_holes_match_the_contract():
    expected = {
        11: 1, 12: 2, 13: 3, 14: 4, 15: 4,
        16: 1, 17: 2, 18: 4, 19: 4, 20: 4,
    }
    for experiment_id, count in expected.items():
        handoffs = [
            step for step in route(experiment_id) if step.kind == 'handoff_screw'
        ]
        assert len(handoffs) == count
        screws = [step.args[0] for step in handoffs]
        holes = [step.args[1] for step in handoffs]
        assert len(set(screws)) == count
        assert len(set(holes)) == count
    assert [step.args[1] for step in route(14) if step.kind == 'handoff_screw'] == [
        'hole_1', 'hole_3', 'hole_2', 'hole_4',
    ]
    assert [step.args[1] for step in route(13) if step.kind == 'handoff_screw'] == [
        'hole_1', 'hole_2', 'hole_3',
    ]


def test_cap_grasp_failure_branch_is_one_retry():
    names = kinds(4)
    assert names.index('simulate_grasp_failure') < names.index('pick_cap')
    assert names.count('simulate_grasp_failure') == 1
    assert names.count('pick_cap') == 1


def test_cap_verification_failure_branch_uses_one_correction():
    names = kinds(5)
    assert names.count('place_cap_shifted') == 1
    assert names.count('place_cap') == 1
    assert names.count('pick_cap') == 2
    inspections = [
        step for step in route(5) if step.kind == 'inspect_cap'
    ]
    assert [step.kwargs.get('expect') for step in inspections] == ['not_ok', 'ok']


def test_interrupted_sealant_continues_from_the_break():
    for experiment_id in (9, 20):
        steps = [step for step in route(experiment_id) if step.kind == 'dispense']
        partial = [step for step in steps if step.args[0] == 'side_3']
        assert partial[0].args == ('side_3', 0.0, 0.5)
        assert partial[1].args == ('side_3', 0.5, 1.0)
        names = kinds(experiment_id)
        assert names.index('inspect_state') > names.index('dispense')


def test_screw_grasp_failure_targets_the_third_and_first_selected_screw():
    fifteen = [
        step for step in route(15) if step.kind == 'simulate_grasp_failure'
    ]
    assert len(fifteen) == 1
    assert fifteen[0].args == ('screw', 'screw_5')
    twenty = [
        step for step in route(20) if step.kind == 'simulate_grasp_failure'
    ]
    assert len(twenty) == 1
    assert twenty[0].args == ('screw', 'screw_1')


def test_experiment_nineteen_route_uses_the_shifted_scenario_objects():
    scenario = spec(19)
    assert scenario.tray_pose is not None
    names = kinds(19)
    assert names.count('handoff_screw') == 4
    # The route names no literal coordinates; actual poses come from the
    # supervisor state populated from the shifted scene.
    for step in route(19):
        for arg in step.args:
            assert not isinstance(arg, (int, float))


class FakeOperationResult:
    def __init__(self, success=True, message='ok'):
        self.success = success
        self.message = message


class FakeOperations:
    def __init__(self, spec):
        self.spec = spec
        self.events = []
        self.calls = []
        self.completed_sealant = set(spec.sealant_sides)

    def take_tool(self, name):
        self.calls.append(('take_tool', name))
        return FakeOperationResult()

    def dock_tool(self, name):
        self.calls.append(('dock_tool', name))
        return FakeOperationResult()

    def pick_cap(self):
        self.calls.append(('pick_cap',))
        return FakeOperationResult()

    def place_cap(self, target=None):
        self.calls.append(('place_cap',))
        return FakeOperationResult()

    def place_cap_shifted(self, dx=0.003):
        self.calls.append(('place_cap_shifted', dx))
        return FakeOperationResult()

    def pick_screw(self, screw):
        self.calls.append(('pick_screw', screw))
        return FakeOperationResult()

    def handoff_screw(self, screw, hole):
        self.calls.append(('handoff_screw', screw, hole))
        return FakeOperationResult()

    def simulate_grasp_failure(self, kind, name=''):
        self.calls.append(('simulate_grasp_failure', kind, name))
        return FakeOperationResult()

    def dispense(self, *args, **kwargs):
        self.calls.append(('dispense',) + args)
        return FakeOperationResult()

    def dispense_contour(self, sides):
        self.calls.append(('dispense_contour', tuple(sides)))
        return FakeOperationResult()

    def inspect_cap(self):
        return {'ok': True}

    def inspect_sealant(self):
        return {'ok': True}

    def inspect_state(self):
        return {'mounted_tool': None}


class FakeMotion:
    def __init__(self, fail=None):
        self.calls = []
        self.fail = fail

    def move_named(self, name, **kwargs):
        self.calls.append(name)
        if self.fail == name:
            return FakeOperationResult(False, 'named motion failed')
        return FakeOperationResult()

    def stop(self, reason):
        self.calls.append(('stop', reason))


class FakeSupervisor:
    latest = {
        'screws': {
            'screw_1': {'state': 'delivered', 'hole': 'hole_1'},
        }
    }


class FakeContext:
    def __init__(self, operations, motion):
        self.operations = operations
        self.motion = motion
        self.supervisor = FakeSupervisor()
        self.trace = self
        self.events = []

    def event(self, event):
        self.events.append(event)

    def simulation_time(self):
        return 12.5


def test_execute_sequence_runs_a_short_route_and_reports_delivery():
    scenario = spec(11)
    operations = FakeOperations(scenario)
    context = FakeContext(operations, FakeMotion())
    result = execute_sequence(route(11), context)
    assert result.status == 'completed'
    assert result.delivered == [('screw_1', 'hole_1')]
    assert result.home_reached is True
    assert result.returned_tools == ['magnetic_tool']


def test_execute_sequence_stops_on_failed_named_motion():
    scenario = spec(11)
    operations = FakeOperations(scenario)
    context = FakeContext(operations, FakeMotion(fail='work'))
    result = execute_sequence(route(11), context)
    assert result.status == 'stopped'
    assert 'named motion failed' in result.message
    assert any(event.get('event') == 'stopped' for event in context.events)
