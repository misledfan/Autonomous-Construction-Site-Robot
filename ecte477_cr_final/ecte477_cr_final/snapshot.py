#!/usr/bin/env python3
"""
snapshot.py - save camera frames to PNG for tuning colours and growing the CNN dataset.

  ros2 run ecte477_cr_final snapshot                 # press Enter to save a frame
  ros2 run ecte477_cr_final snapshot --ros-args -p interval_s:=1.0   # auto every 1 s
Files go to ~/cr_snapshots/ (parameter save_dir).
"""
import os
import threading
import time

import cv2
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from ecte477_cr_final.ros_camera import CameraInput


class Snapshot(Node):
    def __init__(self):
        super().__init__('snapshot')
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('prefer_compressed', True)
        self.declare_parameter('save_dir', os.path.expanduser('~/cr_snapshots'))
        self.declare_parameter('interval_s', 0.0)
        self.dir = os.path.expanduser(self.get_parameter('save_dir').value)
        os.makedirs(self.dir, exist_ok=True)
        self.cam = CameraInput(self, self.get_parameter('camera_topic').value,
                               bool(self.get_parameter('prefer_compressed').value))
        iv = float(self.get_parameter('interval_s').value)
        if iv > 0:
            self.create_timer(iv, self.save)

    def save(self):
        img, _ = self.cam.latest()
        if img is None:
            self.get_logger().warn('no frame yet')
            return
        path = os.path.join(self.dir, time.strftime('snap_%H%M%S_') + f'{int(time.time() * 1000) % 1000:03d}.png')
        cv2.imwrite(path, img)
        self.get_logger().info(f'saved {path} ({img.shape[1]}x{img.shape[0]})')


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Snapshot()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    print(f'Saving to {node.dir}. Press Enter to save a frame, q + Enter to quit.')
    try:
        while True:
            if input().strip().lower() == 'q':
                break
            node.save()
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        ex.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
