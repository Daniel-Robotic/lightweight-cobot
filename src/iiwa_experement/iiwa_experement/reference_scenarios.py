"""The twenty reference routes and their controlled failure branches."""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

from .scenario import ScenarioSpec
from .trace import RunResult


class SequenceError(RuntimeError):
    """A reference step failed or produced an unexpected state."""


@dataclass(frozen=True)
class ReferenceStep:
    kind: str
    args: tuple[Any, ...] = ()
    kwargs: Mapping[str, Any] = field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {'kind': self.kind, 'args': list(self.args), 'kwargs': dict(self.kwargs)}


def _home(speed: float = 0.3) -> ReferenceStep:
    return ReferenceStep('move_named', ('home',), {'speed': speed})


def _work(speed: float = 0.3) -> ReferenceStep:
    return ReferenceStep('move_named', ('work',), {'speed': speed})


def _take(tool: str, *, first: bool = False) -> list[ReferenceStep]:
    steps: list[ReferenceStep] = []
    if not first:
        steps.append(_home())
    steps.extend([
        ReferenceStep('take_tool', (tool,)),
        _work(),
    ])
    return steps


def _cap_steps(
    spec: ScenarioSpec,
    *,
    grasp_failure: bool = False,
    verify_failure: bool = False,
    first_tool: bool = False,
) -> list[ReferenceStep]:
    steps = _take('vacuum', first=first_tool)
    if grasp_failure:
        steps.extend([
            ReferenceStep('simulate_grasp_failure', ('cap',)),
            ReferenceStep('inspect_state'),
        ])
    steps.extend([ReferenceStep('pick_cap'), _work()])
    if verify_failure:
        shift = 0.003
        for failure in spec.failures:
            if failure.skill == 'verify' and 'after_first_install_x' in dict(failure.details):
                expected = float(dict(failure.details)['after_first_install_x'])
                shift = expected - spec.body.x
        steps.extend([
            ReferenceStep('place_cap_shifted', (), {'dx': shift}),
            ReferenceStep('inspect_cap', (), {'expect': 'not_ok', 'tolerance': 0.002}),
            ReferenceStep('pick_cap'),
            _work(),
            ReferenceStep('place_cap'),
            ReferenceStep('inspect_cap', (), {'expect': 'ok'}),
        ])
    else:
        steps.extend([
            ReferenceStep('place_cap'),
            ReferenceStep('inspect_cap', (), {'expect': 'ok'}),
        ])
    steps.extend([
        _home(),
        ReferenceStep('dock_tool', ('vacuum',)),
        _home(),
    ])
    return steps


def _sealant_steps(
    spec: ScenarioSpec,
    *,
    interrupted: bool = False,
    first_tool: bool = True,
    finish_home: bool = True,
) -> list[ReferenceStep]:
    sides = list(spec.sealant_sides)
    steps = _take('sealant_tool', first=first_tool)
    if interrupted:
        steps.append(ReferenceStep('dispense', ('side_1',)))
        steps.append(ReferenceStep('dispense', ('side_2',)))
        steps.extend([
            ReferenceStep('dispense', ('side_3', 0.0, 0.5)),
            ReferenceStep('inspect_state'),
            ReferenceStep('dispense', ('side_3', 0.5, 1.0)),
        ])
        steps.append(ReferenceStep('dispense', ('side_4',)))
    elif len(sides) == 1:
        steps.append(ReferenceStep('dispense', (sides[0],)))
    else:
        steps.append(ReferenceStep('dispense_contour', (tuple(sides),)))
    steps.extend([
        ReferenceStep('inspect_sealant', (), {'expect': 'ok'}),
        _home(),
        ReferenceStep('dock_tool', ('sealant_tool',)),
    ])
    if finish_home:
        steps.append(_home())
    return steps


def _screw_steps(
    spec: ScenarioSpec,
    *,
    failure_index: int | None = None,
    first_tool: bool = False,
) -> list[ReferenceStep]:
    steps = _take('magnetic_tool', first=first_tool)
    # Prefer the closer tray cells.  The far +Y/+X corner can be outside the
    # comfortable IK envelope when the magnetic tool is mounted.
    available = ['screw_1', 'screw_2', 'screw_5', 'screw_6']
    selected = available[:len(spec.screw_holes)]
    for index, (screw, hole) in enumerate(zip(selected, spec.screw_holes)):
        if index > 0:
            steps.append(_home())
        if failure_index is not None and index == failure_index:
            steps.extend([
                ReferenceStep('simulate_grasp_failure', ('screw', screw)),
                ReferenceStep('inspect_state'),
            ])
        steps.extend([
            ReferenceStep('pick_screw', (screw,)),
            _work(),
            ReferenceStep('handoff_screw', (screw, hole)),
            _work(),
        ])
    steps.extend([
        _home(),
        ReferenceStep('dock_tool', ('magnetic_tool',)),
        _home(),
    ])
    return steps


def build_reference_sequence(spec: ScenarioSpec) -> tuple[ReferenceStep, ...]:
    """Return the deterministic route for one experiment."""
    experiment = spec.experiment_id
    steps: list[ReferenceStep] = []

    if experiment in (1, 2, 3, 4, 5):
        steps.extend(_cap_steps(
            spec,
            grasp_failure=(experiment == 4),
            verify_failure=(experiment == 5),
            first_tool=True,
        ))
    elif experiment in (6, 7, 8, 9):
        steps.extend(_sealant_steps(
            spec, interrupted=(experiment == 9), first_tool=True,
        ))
    elif experiment == 10:
        steps.extend(_sealant_steps(
            spec, interrupted=False, first_tool=True, finish_home=False,
        ))
        steps.extend(_cap_steps(spec, first_tool=False))
    elif 11 <= experiment <= 15:
        steps.extend(_screw_steps(
            spec,
            failure_index=2 if experiment == 15 else None,
            first_tool=True,
        ))
    else:
        interrupted = experiment == 20
        steps.extend(_sealant_steps(
            spec, interrupted=interrupted, first_tool=True, finish_home=False,
        ))
        steps.extend(_cap_steps(
            spec,
            verify_failure=(experiment == 20),
            first_tool=False,
        ))
        steps.extend(_screw_steps(
            spec,
            failure_index=0 if experiment == 20 else None,
            first_tool=False,
        ))
    return tuple(steps)


def execute_sequence(sequence: tuple[ReferenceStep, ...], context) -> RunResult:
    """Execute a route, stopping at the first unsafe or unexpected outcome."""
    operations = context.operations
    motion = context.motion
    trace = context.trace
    delivered: list[tuple[str, str]] = []
    returned: list[str] = []
    home_reached = False
    message = 'completed'
    status = 'completed'

    try:
        for step in sequence:
            trace.event({'event': 'step', 'step': step.as_dict(), 'phase': 'start'})
            if step.kind == 'move_named':
                result = motion.move_named(*step.args, **step.kwargs)
                if not result.success:
                    raise SequenceError(result.message)
                if step.args and step.args[0] == 'home':
                    home_reached = True
            elif step.kind == 'inspect_cap':
                kwargs = dict(step.kwargs)
                expect = kwargs.pop('expect', None)
                result = operations.inspect_cap(**kwargs)
                trace.event({'event': 'inspect_cap', 'result': result})
                if expect == 'ok' and not result.get('ok'):
                    raise SequenceError(f'hatch verification failed: {result}')
                if expect == 'not_ok' and result.get('ok'):
                    raise SequenceError('expected a failed hatch verification')
            elif step.kind == 'inspect_sealant':
                result = operations.inspect_sealant()
                trace.event({'event': 'inspect_sealant', 'result': result})
                if step.kwargs.get('expect') == 'ok' and not result.get('ok'):
                    raise SequenceError(f'sealant verification failed: {result}')
            elif step.kind == 'inspect_state':
                result = operations.inspect_state()
                trace.event({'event': 'inspect_state',
                             'mounted_tool': result.get('mounted_tool'),
                             'held_object': result.get('held_object')})
            elif step.kind == 'dock_tool':
                operations.dock_tool(*step.args, **step.kwargs)
                returned.extend(step.args)
            else:
                method = getattr(operations, step.kind, None)
                if method is None:
                    raise SequenceError(f'unknown reference step {step.kind!r}')
                result = method(*step.args, **step.kwargs)
                if hasattr(result, 'success') and not result.success:
                    raise SequenceError(result.message)
            trace.event({'event': 'step', 'step': step.as_dict(), 'phase': 'done'})
    except Exception as error:
        status = 'stopped'
        message = str(error)
        motion.stop(message)
        trace.event({'event': 'stopped', 'reason': message})

    state = context.supervisor.latest or {}
    for name, entry in (state.get('screws') or {}).items():
        if entry.get('state') == 'delivered' and entry.get('hole'):
            delivered.append((name, entry['hole']))
    cap = operations.inspect_cap() if operations.spec.cap_state != 'absent' else None
    sealant = operations.inspect_sealant() if operations.spec.sealant_sides else None
    return RunResult(
        experiment_id=operations.spec.experiment_id,
        status=status,
        message=message,
        events=list(operations.events),
        delivered=delivered,
        cap=cap,
        sealant=sealant,
        returned_tools=returned,
        home_reached=home_reached,
        simulation_time=context.simulation_time(),
    )
