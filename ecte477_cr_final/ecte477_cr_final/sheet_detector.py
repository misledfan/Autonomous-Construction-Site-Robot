#!/usr/bin/env python3
"""
sheet_detector.py - watches the camera for a presented red / green A4 sheet.

Publishes  /sheet_signal  (std_msgs/String, latched): "RED", "GREEN" or "NONE",
           only when the debounced state CHANGES.
Service    /sheet_detector/enable  (std_srvs/SetBool): turn processing on/off.
           Enabling also resets the state to NONE (an old RED cannot stop a new trip).

Workload reduction (requirement 9):
  * only the newest frame is processed, on a timer at `process_hz` (default 4 Hz);
    older frames are dropped, never queued -> no lag;
  * the frame is shrunk to `resize_width` (default 160 px wide, 1/4 of the pixels of
    the 320x240 stream) before any processing - an A4 sheet held in front of the robot
    is large, so fine detail is not needed;
  * processing is disabled while the robot is not driving.
  A state change needs k of the last n frames (3 of 4): worst-case reaction ~1 s,
  i.e. ~22 cm at the Burger's 0.22 m/s top speed.
"""
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from ecte477_cr_final import vision_utils as vu
from ecte477_cr_final.common import LATCHED, share_path
from ecte477_cr_final.ros_camera import CameraInput


class SheetDetector(Node):
    def __init__(self):
        super().__init__('sheet_detector')
        self.declare_parameter('vision_file', share_path('config', 'vision.yaml'))
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('prefer_compressed', True)
        self.declare_parameter('start_enabled', True)

        cfg = vu.load_vision_config(self.get_parameter('vision_file').value)
        self.ranges = cfg['hsv_ranges']
        s = cfg.get('sheet', {})
        self.width = int(s.get('resize_width', 160))
        self.min_area = float(s.get('min_area_frac', 0.08))
        self.min_solidity = float(s.get('min_solidity', 0.80))
        hz = float(s.get('process_hz', 4.0))
        self.deb = vu.Debouncer(int(s.get('debounce_n', 4)), int(s.get('debounce_k', 3)))

        self.enabled = bool(self.get_parameter('start_enabled').value)
        self.pub = self.create_publisher(String, '/sheet_signal', LATCHED)
        self.create_service(SetBool, '/sheet_detector/enable', self.on_enable)
        self.cam = CameraInput(self, self.get_parameter('camera_topic').value,
                               bool(self.get_parameter('prefer_compressed').value))
        self.last_t = 0.0
        self.stats_n, self.stats_ms, self.stats_t0 = 0, 0.0, time.monotonic()
        self.last_info = {}
        self.create_timer(1.0 / hz, self.process)
        self.create_timer(10.0, self.report_stats)
        self.publish('NONE')
        self.get_logger().info(
            f'sheet detector ready: {hz:.1f} Hz, {self.width}px wide, '
            f'min area {self.min_area:.2f}, debounce {self.deb.k}/{self.deb.n}, '
            f'enabled={self.enabled}')

    def publish(self, state):
        self.pub.publish(String(data=state))

    def on_enable(self, req, res):
        self.enabled = bool(req.data)
        self.deb = vu.Debouncer(self.deb.n, self.deb.k)
        self.publish('NONE')
        res.success = True
        res.message = f'sheet detector {"enabled" if self.enabled else "disabled"}'
        self.get_logger().info(res.message)
        return res

    def process(self):
        if not self.enabled:
            return
        frame, t = self.cam.latest(newer_than=self.last_t)
        if frame is None:
            return
        self.last_t = t
        t0 = time.perf_counter()
        small = vu.resize_to_width(frame, self.width)
        label, info = vu.detect_sheet(small, self.ranges, self.min_area, self.min_solidity)
        self.stats_ms += (time.perf_counter() - t0) * 1000.0
        self.stats_n += 1
        self.last_info = info
        if self.deb.update(label):
            self.get_logger().warn(f'SHEET STATE -> {self.deb.state}   {info.get(self.deb.state)}')
            self.publish(self.deb.state)

    def report_stats(self):
        dt = time.monotonic() - self.stats_t0
        if self.stats_n:
            self.get_logger().info(
                f'processed {self.stats_n / dt:.1f} frames/s, {self.stats_ms / self.stats_n:.1f} '
                f'ms/frame, camera={self.cam.mode}, state={self.deb.state}, last={self.last_info}')
        elif self.enabled:
            self.get_logger().warn('no camera frames processed in the last 10 s')
        self.stats_n, self.stats_ms, self.stats_t0 = 0, 0.0, time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = SheetDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
