"""Publish Webots camera frames without accumulating timestep rounding drift."""
import array
import math
import sys

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class FrameSchedule:
    """Keep deadlines on the requested cadence, with at most one frame per step."""

    def __init__(self, rate, now):
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('Camera update rate must be finite and positive')
        self.period = 1.0 / rate
        self.last_time = now
        self.next_time = now + self.period

    def due(self, now):
        if now < self.last_time:
            self.next_time = now + self.period
        self.last_time = now
        if now + 1e-9 < self.next_time:
            return False
        elapsed = max(0, math.floor((now - self.next_time + 1e-9) / self.period))
        self.next_time += (elapsed + 1) * self.period
        return True


class WebotsCamera:
    """One RGB or depth stream; preserves the stock driver's topic suffixes."""

    def init(self, webots_node, properties):
        self._robot = webots_node.robot
        name = properties['device']
        self._device = self._robot.getDevice(name)
        if self._device is None:
            raise ValueError(f'Camera device not found: {name}')
        self._depth = properties.get('depth', 'false') == 'true'
        rate = float(properties.get('updateRate', '30'))
        self._schedule = FrameSchedule(rate, self._robot.getTime())
        timestep = int(self._robot.getBasicTimeStep())
        if timestep <= 0 or rate * timestep > 1000:
            raise ValueError('Camera rate exceeds the Webots simulation step rate')
        self._device.enable(timestep)
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node(f'{name}_publisher')
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        topic = properties['topicName']
        suffix = '/image' if self._depth else '/image_color'
        self._publisher = self._node.create_publisher(Image, topic + suffix, qos)
        self._info_publisher = self._node.create_publisher(CameraInfo, topic + '/camera_info', qos)
        self._info = CameraInfo()
        self._info.header.frame_id = properties['frameName']
        self._info.width = self._device.getWidth()
        self._info.height = self._device.getHeight()
        focal = self._info.width / (2 * math.tan(self._device.getFov() / 2))
        cx, cy = self._info.width / 2, self._info.height / 2
        self._info.distortion_model = 'plumb_bob'
        self._info.d = [0.0] * 5
        self._info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
        self._info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self._info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        if self._depth:
            self._cloud_publisher = self._node.create_publisher(
                PointCloud2, topic + '/point_cloud', qos)
            u, v = np.meshgrid(np.arange(self._info.width), np.arange(self._info.height))
            self._rays = np.stack(((u - cx) / focal, (v - cy) / focal,
                                   np.ones_like(u)), axis=-1).astype(np.float32)

    def step(self):
        now = self._robot.getTime()
        if not self._schedule.due(now):
            return
        if self._depth:
            data = array.array('f', self._device.getRangeImage()).tobytes()
        else:
            data = self._device.getImage()
        if data is None:
            return
        nanoseconds = round(now * 1_000_000_000)
        self._info.header.stamp = Time(
            sec=nanoseconds // 1_000_000_000, nanosec=nanoseconds % 1_000_000_000)
        message = Image()
        message.header = self._info.header
        message.width, message.height = self._info.width, self._info.height
        message.encoding = '32FC1' if self._depth else 'bgra8'
        message.is_bigendian = int(sys.byteorder == 'big') if self._depth else 0
        message.step = message.width * 4
        message.data = data
        self._publisher.publish(message)
        self._info_publisher.publish(self._info)
        if self._depth and self._cloud_publisher.get_subscription_count():
            depth = np.frombuffer(data, dtype=np.float32).reshape(message.height, message.width)
            # Optical coordinates: x right, y down, z forward, all in metres.
            valid = np.isfinite(depth) & (depth > 0)
            points = self._rays * np.where(valid, depth, 0.0)[..., None]
            points[~valid] = np.nan
            cloud = PointCloud2()
            cloud.header = message.header
            cloud.width, cloud.height = message.width, message.height
            cloud.fields = [PointField(name=name, offset=4 * i,
                                       datatype=PointField.FLOAT32, count=1)
                            for i, name in enumerate(('x', 'y', 'z'))]
            cloud.is_bigendian = bool(message.is_bigendian)
            cloud.point_step, cloud.row_step = 12, message.width * 12
            cloud.is_dense = bool(valid.all())
            cloud.data = points.tobytes()
            self._cloud_publisher.publish(cloud)
