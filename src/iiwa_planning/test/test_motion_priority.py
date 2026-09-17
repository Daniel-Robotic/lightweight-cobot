import threading
import pytest
from iiwa_planning.motion_priority import MotionLease, LeaseError, MotionSerialiser
from iiwa_planning.motion_priority import wait_response


def test_priority_acquisition_waits_for_full_gizmo_trajectory():
    from concurrent.futures import Future
    future = Future()
    timer = threading.Timer(2.1, lambda: future.set_result('granted'))
    timer.start()
    try:
        assert wait_response(future, timeout=None, available=lambda: True) == 'granted'
    finally:
        timer.join()


def test_waiting_priority_fails_if_server_disappears():
    from concurrent.futures import Future
    future = Future()
    with pytest.raises(LeaseError, match='unavailable'):
        wait_response(future, timeout=None, available=lambda: False)
    assert future.cancelled()


def test_overlapping_operations_have_independent_owners_and_release():
    owners = set()
    def exchange(owner, acquire, ttl):
        owners.add(owner) if acquire else owners.discard(owner)
    with MotionLease(exchange) as first:
        with MotionLease(exchange) as second:
            assert first.owner != second.owner
            assert len(owners) == 2
        assert owners == {first.owner}
    assert not owners


def test_acquisition_fails_closed():
    def unavailable(*args):
        raise LeaseError('unavailable')
    with pytest.raises(LeaseError), MotionLease(unavailable):
        pytest.fail('motion entered without reservation')


def test_renewal_failure_stops_operation():
    stopped = threading.Event()
    calls = []
    def exchange(owner, acquire, ttl):
        calls.append(acquire)
        if acquire and len(calls) > 1:
            raise LeaseError('lost')
    with pytest.raises(LeaseError):
        with MotionLease(exchange, on_lost=stopped.set, renew_interval=0.01) as lease:
            assert stopped.wait(1)
            lease.check()
    assert calls[-1] is False


def test_release_does_not_mask_original_error():
    def exchange(owner, acquire, ttl):
        if not acquire:
            raise LeaseError('release failed')
    with pytest.raises(ValueError, match='original'):
        with MotionLease(exchange):
            raise ValueError('original')


def test_stop_invalidates_plans_and_queued_requests():
    serial = MotionSerialiser()
    generation = serial.generation()
    serial.stop(lambda: None)
    with pytest.raises(LeaseError):
        serial.check(generation)
    serial.check(serial.generation())


def test_motion_lock_serializes_workers():
    serial = MotionSerialiser()
    entered = threading.Event()
    finished = threading.Event()
    with serial.lock:
        def worker():
            entered.set()
            with serial.lock:
                finished.set()
        thread = threading.Thread(target=worker)
        thread.start()
        assert entered.wait(1)
        assert not finished.is_set()
    thread.join(1)
    assert finished.is_set()


def _server_methods():
    """Exercise real server orchestration without requiring MoveIt or ROS libraries."""
    import ast
    from pathlib import Path
    path = Path(__file__).parents[1] / 'scripts' / 'move_to_pose_server.py'
    source = ast.parse(path.read_text())
    cls = next(n for n in source.body if isinstance(n, ast.ClassDef))
    names = {'_plan_and_execute', '_check_operation', '_guarded', '_lease'}
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    namespace = {'threading': threading, 'MotionLease': MotionLease,
                 'PlanRequestParameters': object}
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(path), 'exec'), namespace)
    return type('Server', (), {n: namespace[n] for n in names})


def _fake_server(plan, execute):
    from types import SimpleNamespace
    server = _server_methods()()
    server._motion_serial = MotionSerialiser()
    server._operation = threading.local()
    server._operation.generation = server._motion_serial.generation()
    server._operation.lease = MotionLease()
    server._arm = SimpleNamespace(plan=plan)
    server._moveit = SimpleNamespace(execute=execute)
    server.get_logger = lambda: SimpleNamespace(error=lambda _: None)
    return server


def test_stop_during_planning_prevents_execution():
    from types import SimpleNamespace
    executions = []
    def plan(**kwargs):
        server._motion_serial.stop(lambda: None)
        return SimpleNamespace(trajectory=object())
    server = _fake_server(plan, lambda *a, **kw: executions.append(a))
    with pytest.raises(LeaseError, match='stop'):
        server._plan_and_execute(None, SimpleNamespace(is_cancel_requested=False))
    assert not executions


def test_failed_execution_is_reported_as_failure():
    from types import SimpleNamespace
    server = _fake_server(lambda **kw: SimpleNamespace(trajectory=object()),
                          lambda *a, **kw: False)
    ok, _ = server._plan_and_execute(None, SimpleNamespace(is_cancel_requested=False))
    assert ok is False


def test_queued_motion_reserves_before_waiting_for_planning_lock():
    server = _server_methods()()
    from types import SimpleNamespace
    reserved = threading.Event()
    completed = threading.Event()
    server._motion_serial = MotionSerialiser()
    server._operation = threading.local()
    server._gizmo_enabled = True
    server._stop_execution = lambda: None
    server._priority = SimpleNamespace(exchange=lambda owner, acquire, ttl:
                                       reserved.set() if acquire else None)
    with server._motion_serial.lock:
        thread = threading.Thread(target=lambda: server._guarded(completed.set))
        thread.start()
        assert reserved.wait(1)
        assert not completed.is_set()
    thread.join(1)
    assert completed.is_set()


def test_disabled_lease_needs_no_ros_service():
    with MotionLease() as lease:
        lease.check()


def test_failed_sequence_reports_nonzero_exit_and_releases_resources():
    import ast
    from pathlib import Path
    from types import SimpleNamespace
    path = Path(__file__).parents[1] / 'scripts' / 'motion_sequence_runner.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'run')
    namespace = {'threading': threading, 'MotionLease': MotionLease, 'LeaseError': LeaseError}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
    stopped, closed, done = threading.Event(), threading.Event(), threading.Event()
    def unavailable():
        raise LeaseError('priority unavailable')
    runner = SimpleNamespace(_discover_priority=unavailable, _priority=None,
                             _abort_sequence=stopped.set, close_bag=closed.set,
                             get_logger=lambda: SimpleNamespace(error=lambda _: None))
    namespace['run'](runner, done)
    assert stopped.is_set() and closed.is_set() and done.is_set()
    assert runner.exit_code != 0
