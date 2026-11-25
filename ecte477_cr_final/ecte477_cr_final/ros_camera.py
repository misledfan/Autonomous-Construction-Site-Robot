"""
ros_camera.py - camera input shared by the vision nodes.

* Prefers /image_raw/compressed (JPEG, ~10-20x less Wi-Fi traffic). If nothing arrives
  within a few seconds (robot without the image_transport plugin), it falls back to
  the raw topic automatically.
* Callbacks only STORE the newest message; decoding happens on demand in the
  processing timer, so old frames are dropped instead of queued (no lag) and only the
  frames we actually analyse cost CPU.
* Freshness uses the time the frame ARRIVED on this PC (time.monotonic), not the
  header stamp, so a wrong clock on the Raspberry Pi cannot break it.
* No cv_bridge (avoids NumPy-version problems on Humble).
"""
import threading
import time

import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image


def image_msg_to_bgr(msg):
    enc = msg.encoding.lower()
    h, w, step = msg.height, msg.width, msg.step
    buf = np.frombuffer(bytes(msg.data), np.uint8)
    if enc in ('rgb8', 'bgr8'):
        img = buf.reshape(h, step)[:, :w * 3].reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == 'rgb8' else img.copy()
    if enc in ('rgba8', 'bgra8'):
        img = buf.reshape(h, step)[:, :w * 4].reshape(h, w, 4)
        return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR if enc == 'rgba8' else cv2.COLOR_BGRA2BGR)
    if enc in ('mono8', '8uc1'):
        return cv2.cvtColor(buf.reshape(h, step)[:, :w].copy(), cv2.COLOR_GRAY2BGR)
    if enc in ('yuv422_yuy2', 'yuyv'):
        return cv2.cvtColor(buf.reshape(h, step)[:, :w * 2].reshape(h, w, 2), cv2.COLOR_YUV2BGR_YUY2)
    if enc in ('yuv422', 'uyvy'):
        return cv2.cvtColor(buf.reshape(h, step)[:, :w * 2].reshape(h, w, 2), cv2.COLOR_YUV2BGR_UYVY)
    raise ValueError(f'unsupported image encoding {msg.encoding}')


class CameraInput:
    def __init__(self, node, topic='/image_raw', prefer_compressed=True,
                 callback_group=None, fallback_after_s=4.0):
        self.node = node
        self.topic = topic
        self.cg = callback_group
        self.lock = threading.Lock()
        self.msg = None
        self.msg_t = 0.0
        self.count = 0
        self.sub = None
        self.mode = None
        if prefer_compressed:
            self._subscribe('compressed')
            self.fallback_timer = node.create_timer(
                fallback_after_s, self._check_fallback, callback_group=callback_group)
        else:
            self._subscribe('raw')

    def _subscribe(self, mode):
        if self.sub is not None:
            self.node.destroy_subscription(self.sub)
        self.mode = mode
        if mode == 'compressed':
            self.sub = self.node.create_subscription(
                CompressedImage, self.topic + '/compressed', self._cb, qos_profile_sensor_data,
                callback_group=self.cg)
        else:
            self.sub = self.node.create_subscription(
                Image, self.topic, self._cb, qos_profile_sensor_data, callback_group=self.cg)
        self.node.get_logger().info(
            f'camera: subscribed to {self.topic}{"/compressed" if mode == "compressed" else ""}')

    def _check_fallback(self):
        self.fallback_timer.cancel()
        if self.count == 0 and self.mode == 'compressed':
            self.node.get_logger().warn(
                'camera: no compressed images received - falling back to raw images')
            self._subscribe('raw')

    def _cb(self, msg):
        with self.lock:
            self.msg = msg
            self.msg_t = time.monotonic()
            self.count += 1

    def latest(self, newer_than=0.0):
        """Return (bgr, arrival_time) of the newest frame if it arrived after newer_than,
        else (None, 0.0)."""
        with self.lock:
            msg, t = self.msg, self.msg_t
        if msg is None or t <= newer_than:
            return None, 0.0
        try:
            if isinstance(msg, CompressedImage):
                img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
            else:
                img = image_msg_to_bgr(msg)
        except Exception as e:  # noqa: BLE001
            self.node.get_logger().warn(f'camera: could not decode frame: {e}')
            return None, 0.0
        return img, t
