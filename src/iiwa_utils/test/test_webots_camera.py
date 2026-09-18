"""Check published pixels, units and calibration with a small device adapter."""
import array
import copy
import math
from types import SimpleNamespace

import pytest
import numpy as np

from iiwa_utils import webots_camera as camera


@pytest.mark.parametrize('depth', [False, True])
def test_messages_match_device_and_share_timestamp(monkeypatch, depth):
    messages = {}
    now = [0.0]
    device = SimpleNamespace(
        enable=lambda period: None,
        getWidth=lambda: 2, getHeight=lambda: 1,
        getFov=lambda: math.pi / 2,
        getImage=lambda: bytes([1, 2, 3, 255, 4, 5, 6, 255]),
        getRangeImage=lambda: [1.25, float('inf')],
    )
    robot = SimpleNamespace(getDevice=lambda name: device,
                            getBasicTimeStep=lambda: 32, getTime=lambda: now[0])

    def publisher(message_type, topic, qos):
        messages[topic] = []
        return SimpleNamespace(publish=lambda msg: messages[topic].append(copy.deepcopy(msg)),
                               get_subscription_count=lambda: 1)

    monkeypatch.setattr(camera.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(camera.rclpy, 'create_node',
                        lambda name: SimpleNamespace(create_publisher=publisher))
    plugin = camera.WebotsCamera()
    plugin.init(SimpleNamespace(robot=robot), dict(
        device='test_camera', depth=str(depth).lower(), updateRate='30',
        topicName='/test', frameName='test_optical_frame'))
    for tick in range(1, 101):
        now[0] = tick * 0.032
        plugin.step()
    images = messages['/test/image' if depth else '/test/image_color']
    infos = messages['/test/camera_info']
    assert len(images) == len(infos) == 96
    image, info = images[-1], infos[-1]
    assert image.header == info.header
    assert image.header.frame_id == 'test_optical_frame'
    assert (image.width, image.height, image.step) == (2, 1, 8)
    assert image.header.stamp.sec == 3
    assert image.header.stamp.nanosec == 200_000_000
    assert info.k[0] == pytest.approx(1.0)
    assert info.k[2] == 1.0
    assert info.k[5] == 0.5
    if depth:
        assert image.encoding == '32FC1'
        values = array.array('f', image.data.tobytes())
        assert values[0] == 1.25
        assert math.isinf(values[1])
        cloud = messages['/test/point_cloud'][-1]
        assert cloud.header == image.header
        xyz = np.frombuffer(cloud.data, dtype=np.float32).reshape(-1, 3)
        np.testing.assert_allclose(xyz[0], [-1.25, -0.625, 1.25])
        assert np.isnan(xyz[1]).all()
    else:
        assert image.encoding == 'bgra8'
        assert image.data.tobytes() == device.getImage()
