"""Motion guards: stale state, unsynchronized scene, failures and stop."""
from pathlib import Path

import pytest

from iiwa_experement.geometry import Transform, load_geometry
from iiwa_experement.motion import MotionAdapter


PACKAGE = Path(__file__).resolve().parents[1]
GEOMETRY = PACKAGE / 'config' / 'geometry.yaml'


class FakeFuture:
    def __init__(self, value=None, done=True):
        self._value = value
        self._done = done

    def done(self):
        return self._done

    def result(self):
        return self._value


class FakeResult:
    def __init__(self, success=True, message='ok'):
        self.success = success
        self.message = message


class FakeWrapped:
    def __init__(self, result):
        self.result = result


class FakeGoalHandle:
    def __init__(self, accepted=True, result=None):
        self.accepted = accepted
        self._result = result or FakeResult()

    def get_result_async(self):
        return FakeFuture(FakeWrapped(self._result))


class FakeActionClient:
    def __init__(self, *, available=True, accepted=True, result=None, timeout=False):
        self.available = available
        self.accepted = accepted
        self.result = result
        self.timeout = timeout
        self.requests = []

    def wait_for_server(self, timeout_sec=0.0):
        return self.available

    def send_goal_async(self, request):
        self.requests.append(request)
        if self.timeout:
            return FakeFuture(None, done=False)
        return FakeFuture(FakeGoalHandle(self.accepted, self.result))


class FakeService:
    def __init__(self, *, available=True, success=True, message='ok', timeout=False):
        self.available = available
        self.success = success
        self.message = message
        self.timeout = timeout
        self.requests = []

    def wait_for_service(self, timeout_sec=0.0):
        return self.available

    def call_async(self, request):
        self.requests.append(request)
        if self.timeout:
            return FakeFuture(None, done=False)
        return FakeFuture(FakeResult(self.success, self.message))


class StopService(FakeService):
    def __init__(self):
        super().__init__(success=True, message='stopped')
        self.calls = 0

    def call_async(self, request):
        self.calls += 1
        return super().call_async(request)


def geometry():
    return load_geometry(GEOMETRY)


def goal():
    return Transform.from_xyz_rpy(
        [0.20, 0.10, 0.90], [180.0, 0.0, 45.0], parent='world', child='tcp',
    )


def make_adapter(
    *,
    action=None,
    named=None,
    stop=None,
    fresh=True,
    scene=True,
    actual=None,
    spin=None,
):
    g = geometry()
    action = action or FakeActionClient()
    named = named or FakeService()
    stop = stop or StopService()
    adapter = MotionAdapter(
        node=None,
        geometry=g,
        action_client=action,
        named_client=named,
        stop_client=stop,
        tf_buffer=object(),
        scene_check=lambda: (scene, 'scene is stale' if not scene else 'ok'),
        freshness_check=lambda: (fresh, 'joint state is stale' if not fresh else 'ok'),
        pose_provider=lambda: actual if actual is not None else adapter.to_base_goal(goal()),
        spin_until=spin or (lambda future, timeout: future.done()),
    )
    return adapter, action, named, stop


def test_successful_move_populates_abc_and_verifies_actual_pose():
    adapter, action, _, _ = make_adapter()
    result = adapter.move_pose(goal(), planner='ptp', speed=0.2, label='test')
    assert result.success, result.message
    assert len(action.requests) == 1
    request = action.requests[0]
    expected = adapter.to_base_goal(goal())
    assert (request.x, request.y, request.z) == pytest.approx(expected.xyz)
    assert request.speed == pytest.approx(0.2)
    assert request.planner == 'ptp'
    assert request.frame_id == 'base_link'
    assert result.goal.xyz == pytest.approx(expected.xyz)


def test_stale_joint_state_blocks_before_sending_and_stops():
    stop = StopService()
    adapter, action, _, _ = make_adapter(fresh=False, stop=stop)
    result = adapter.move_pose(goal(), label='stale')
    assert not result.success
    assert 'stale' in result.message
    assert action.requests == []
    assert adapter.stopped is True
    assert stop.calls == 1


def test_unsynchronized_scene_blocks_before_sending_and_stops():
    adapter, action, _, _ = make_adapter(scene=False)
    result = adapter.move_pose(goal(), label='scene')
    assert not result.success
    assert 'scene' in result.message
    assert action.requests == []
    assert adapter.stopped is True


def test_invalid_speed_and_planner_are_rejected_without_stopping():
    adapter, action, _, _ = make_adapter()
    assert not adapter.move_pose(goal(), speed=0.0).success
    assert not adapter.move_pose(goal(), speed=2.0).success
    assert not adapter.move_pose(goal(), planner='teleport').success
    assert action.requests == []
    assert adapter.stopped is False


def test_rejected_goal_stops_and_blocks_the_next_move():
    action = FakeActionClient(accepted=False)
    adapter, action, _, stop = make_adapter(action=action)
    result = adapter.move_pose(goal(), label='rejected')
    assert not result.success
    assert 'rejected' in result.message
    assert adapter.stopped is True
    assert stop.calls == 1
    count = len(action.requests)
    blocked = adapter.move_pose(goal(), label='again')
    assert not blocked.success
    assert 'blocked' in blocked.message
    assert len(action.requests) == count


def test_timeout_stops_the_robot():
    adapter, _, _, stop = make_adapter(action=FakeActionClient(timeout=True))
    result = adapter.move_pose(goal(), label='timeout')
    assert not result.success
    assert 'timed out' in result.message
    assert adapter.stopped is True
    assert stop.calls == 1


def test_actual_pose_out_of_tolerance_stops_the_robot():
    actual = goal().compose_relaxed(Transform.from_xyz_rpy(
        [0.0, 0.0, 0.05], [0, 0, 0], parent='world', child='tcp',
    ))
    adapter, _, _, stop = make_adapter(actual=actual)
    result = adapter.move_pose(goal(), label='mismatch')
    assert not result.success
    assert 'from the goal' in result.message
    assert adapter.stopped is True
    assert stop.calls == 1


def test_named_motion_uses_the_existing_service():
    adapter, _, named, _ = make_adapter()
    result = adapter.move_named('home', speed=0.2)
    assert result.success
    assert len(named.requests) == 1
    assert named.requests[0].name == 'home'
    assert named.requests[0].speed == pytest.approx(0.2)


def test_failed_named_motion_stops():
    named = FakeService(success=False, message='unknown state')
    adapter, _, _, stop = make_adapter(named=named)
    result = adapter.move_named('missing')
    assert not result.success
    assert 'unknown state' in result.message
    assert adapter.stopped is True
    assert stop.calls == 1


def test_unavailable_server_is_reported_without_motion():
    adapter, action, _, _ = make_adapter(action=FakeActionClient(available=False))
    result = adapter.move_pose(goal(), label='no server')
    assert not result.success
    assert 'unavailable' in result.message
    assert action.requests == []
    assert adapter.stopped is False
