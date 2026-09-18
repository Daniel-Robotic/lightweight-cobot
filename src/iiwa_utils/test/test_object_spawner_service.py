"""Exercise our service requests with the installed Jazzy message schema."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from iiwa_utils import object_spawner


@pytest.mark.parametrize('success,completed', [(True, True), (False, True), (True, False)])
def test_service_schema_and_failure_stop(monkeypatch, success, completed):
    node = Mock()
    node.declare_parameter.side_effect = [SimpleNamespace(value='{}'),
                                         SimpleNamespace(value='0 0 0'),
                                         SimpleNamespace(value='0 0 1 0')]
    monkeypatch.setattr(object_spawner, 'build_objects', lambda *args: ['Solid {}', 'Solid {}'])
    monkeypatch.setattr(object_spawner.rclpy, 'spin_until_future_complete', lambda *a, **kw: None)
    client = node.create_client.return_value
    client.wait_for_service.return_value = True
    future = client.call_async.return_value
    future.done.return_value = completed
    future.result.return_value = SimpleNamespace(success=success)
    if success and completed:
        object_spawner.spawn_objects(node)
        assert client.call_async.call_count == 2
        assert client.call_async.call_args.args[0].data == 'Solid {}'
    else:
        with pytest.raises(RuntimeError):
            object_spawner.spawn_objects(node)
        assert client.call_async.call_count == 1
