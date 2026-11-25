#!/usr/bin/env python3
"""
zone_verifier.py - identifies the construction zone in front of the robot and counts
its markers / objects (requirements 11-15).

Service  /verify_zone   (std_srvs/Trigger)
         success = detected zone matches the requested zone
         message = human-readable report (zone, counts, how confident and how measured)
Topic in /requested_zone (std_msgs/String, latched) - zone key set by the selector.
Topic out /zone_report   (std_msgs/String, latched) - same report, for logging/bags.
Also saves an annotated image of the analysed view to ~/cr_reports/ (report evidence).

Method (all OpenCV except the object CNN):
  1. Wait for FRESH frames: only frames that arrive after the request (the selector
     already waited `settle_s` after Nav2 reported arrival).
  2. Per frame: HSV masks for the three zone colours -> the colour with the largest
     area is the zone (requirement 12: OpenCV, not CNN).
  3. Shapes are counted ONLY in the winning colour's mask (requirement 13), e.g. the
     pink squares on the blue sheet are ignored. approxPolyDP vertex count +
     circularity name each contour; contours cut by the image border are skipped.
  4. Objects printed on the zone colour (light bulbs on the yellow disc) are holes in
     that colour's mask -> each hole is cropped and classified by the custom CNN
     (models/object_cnn.onnx, run with cv2.dnn).
  0. ALIGN (optional, on by default): Nav2 stops within its goal tolerance and AMCL
     is never perfect, so the marker can sit at the edge of the view. Before
     analysing, the robot turns (P-controller on the marker's horizontal offset in
     the image) until the zone colour is centred, then creeps forward while the
     marker is small, stopping early if the LiDAR sees a wall within `stop_dist_m`
     or after `max_advance_m`. This is visual servoing using the camera.
  5. N frames are combined by majority vote; "k/N frames agree" is reported.
  Confidence wording (requirement 15): colour dominance is a relative pixel share and
  frame agreement is a vote count - neither is presented as a probability.
"""
import math
import os
import time

import cv2
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ecte477_cr_final import vision_utils as vu
from ecte477_cr_final.common import LATCHED, load_zones, share_path
from ecte477_cr_final.ros_camera import CameraInput


class ZoneVerifier(Node):
    def __init__(self):
        super().__init__('zone_verifier')
        self.declare_parameter('vision_file', share_path('config', 'vision.yaml'))
        self.declare_parameter('zones_file', share_path('config', 'zones.yaml'))
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('prefer_compressed', True)
        self.declare_parameter('save_dir', os.path.expanduser('~/cr_reports'))

        cfg = vu.load_vision_config(self.get_parameter('vision_file').value)
        self.ranges = cfg['hsv_ranges']
        self.zone_colours = list(cfg.get('zone_colours', ['blue', 'pink', 'yellow']))
        z = cfg.get('zone', {})
        self.width = int(z.get('resize_width', 320))
        self.n_frames = int(z.get('frames', 7))
        self.spacing = float(z.get('frame_spacing_s', 0.2))
        self.timeout = float(z.get('timeout_s', 8.0))
        self.min_zone_frac = float(z.get('min_zone_frac', 0.01))
        self.min_shape_area = int(z.get('min_shape_area_px', 150))
        self.min_obj_score = float(z.get('cnn_min_score', 0.7))
        a = cfg.get('align', {})
        self.align_on = bool(a.get('enabled', True))
        self.al_timeout = float(a.get('timeout_s', 8.0))
        self.al_tol = float(a.get('centre_tolerance', 0.12))
        self.al_min_w = float(a.get('min_turn_rad_s', 0.25))
        self.al_max_w = float(a.get('max_turn_rad_s', 0.6))
        self.al_k = float(a.get('k_turn', 0.8))
        self.al_search = math.radians(float(a.get('search_deg', 70.0)))
        self.al_back_speed = float(a.get('backup_speed_m_s', 0.06))
        self.al_max_back = float(a.get('max_backup_m', 0.10))
        self.al_rear = float(a.get('rear_clear_m', 0.45))

        self.zones = load_zones(self.get_parameter('zones_file').value)
        self.colour_to_zone = {v['colour']: k for k, v in self.zones.items()}

        self.classifier = None
        model = share_path('models', z.get('cnn_model', 'object_cnn.onnx'))
        labels_file = os.path.splitext(model)[0] + '_labels.txt'
        if os.path.exists(model) and os.path.exists(labels_file):
            with open(labels_file) as f:
                labels = [line.strip() for line in f if line.strip()]
            try:
                self.classifier = vu.OnnxClassifier(model, labels, int(z.get('cnn_input', 40)))
                self.get_logger().info(f'CNN loaded: {model} classes={labels}')
            except Exception as e:  # noqa: BLE001
                self.get_logger().error(f'CNN could not be loaded ({e}) - objects not counted')
        else:
            self.get_logger().warn(f'CNN model not found ({model}) - objects will not be counted')

        self.requested = None
        self.align_colour = None
        cam_group = ReentrantCallbackGroup()
        self.create_subscription(String, '/requested_zone', self.on_requested, LATCHED,
                                 callback_group=cam_group)
        self.cam = CameraInput(self, self.get_parameter('camera_topic').value,
                               bool(self.get_parameter('prefer_compressed').value),
                               callback_group=cam_group)
        self.report_pub = self.create_publisher(String, '/zone_report', LATCHED)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.front_dist = 0.0      # unknown until the first scan -> no forward motion
        self.rear_dist = 0.0
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data,
                                 callback_group=cam_group)
        self.create_service(Trigger, '/verify_zone', self.on_verify,
                            callback_group=MutuallyExclusiveCallbackGroup())
        self.save_dir = os.path.expanduser(self.get_parameter('save_dir').value)
        self.get_logger().info(
            f'zone verifier ready: {self.n_frames} frames, zones={self.colour_to_zone}')

    def on_scan(self, msg):
        """Closest LiDAR return within +/-20 deg of straight ahead (safety for approach)."""
        front, rear = float('inf'), float('inf')
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            ang = math.atan2(math.sin(msg.angle_min + i * msg.angle_increment),
                             math.cos(msg.angle_min + i * msg.angle_increment))
            if abs(ang) <= math.radians(20):
                front = min(front, r)
            elif abs(ang) >= math.radians(160):
                rear = min(rear, r)
        self.front_dist, self.rear_dist = front, rear

    def drive(self, v, w):
        t = Twist()
        t.linear.x = float(v)
        t.angular.z = float(w)
        self.cmd_pub.publish(t)

    def next_frame(self, last, timeout=1.0):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            img, t = self.cam.latest(newer_than=last)
            if img is not None:
                return img, t
            time.sleep(0.02)
        return None, last

    def look(self, last):
        """Newest frame -> (colour, x_offset, area, cut, frame_time)."""
        img, last = self.next_frame(last)
        if img is None:
            return None, 0.0, 0.0, False, last
        col, ex, area, cut = vu.zone_target(vu.resize_to_width(img, 160), self.ranges,
                                            self.zone_colours, only=self.align_colour)
        return col, ex, area, cut, last

    def search(self, last):
        """Marker not in view: sweep left then right (up to search_deg each side) at a
        speed that really turns the robot; stop as soon as a zone colour appears."""
        w = self.al_max_w
        for direction, span in ((1.0, self.al_search), (-1.0, 2 * self.al_search)):
            turned, t_prev = 0.0, time.monotonic()
            while turned < span:
                col, _, _, _, last = self.look(last)
                if col is not None:
                    self.drive(0.0, 0.0)
                    return True, last
                self.drive(0.0, direction * w)
                now = time.monotonic()
                turned += w * (now - t_prev)
                t_prev = now
        self.drive(0.0, 0.0)
        return False, last

    def align(self):
        # Turn towards the REQUESTED zone's colour (we know what we are looking for);
        # the verification afterwards still checks all colours independently.
        z = self.zones.get(self.requested or '', {})
        self.align_colour = z.get('colour')
        return self._align()

    def _align(self):
        """Rotate-first visual alignment (no forward driving, so the robot never ends up
        boxed in against a wall):
          1. marker not visible -> sweep left/right to find it;
          2. turn (in place) until the marker is centred - every turn command is at least
             `min_turn_rad_s`, so small corrections really move the robot;
          3. only when centred: if the marker is cut off by the image edge, reverse
             (max `max_backup_m`, only with `rear_clear_m` free behind) to fit it all in."""
        t_end = time.monotonic() + self.al_timeout
        last, backed, searched, stable, note = 0.0, 0.0, False, 0, 'timeout'
        t_prev = time.monotonic()
        col = None
        while time.monotonic() < t_end:
            col, ex, area, cut, last = self.look(last)
            now = time.monotonic()
            dt, t_prev = now - t_prev, now
            if col is None:
                if searched:
                    note = 'no zone colour visible after searching'
                    break
                searched = True
                self.get_logger().info('alignment: marker not in view - searching')
                found, last = self.search(last)
                t_prev = time.monotonic()
                if not found:
                    note = 'no zone colour visible after searching'
                    break
                continue
            if abs(ex) > self.al_tol:                       # 2. turn towards the marker
                stable = 0
                w = min(self.al_max_w, max(self.al_min_w, self.al_k * abs(ex)))
                self.drive(0.0, -w if ex > 0 else w)
                continue
            if cut and backed < self.al_max_back and self.rear_dist > self.al_rear:
                stable = 0                                  # 3. centred but cut off
                self.drive(-self.al_back_speed, 0.0)
                backed += self.al_back_speed * dt
                continue
            self.drive(0.0, 0.0)                            # centred and settled
            stable += 1
            if stable >= 2:
                note = (f'centred (offset {ex:+.2f}), marker {area:.0%} of view'
                        f'{", still cut off by the image edge" if cut else ", fully in view"}'
                        f', reversed {backed:.2f} m')
                break
        for _ in range(4):
            self.drive(0.0, 0.0)
            time.sleep(0.03)
        time.sleep(0.4)          # let the image settle
        self.get_logger().info(f'alignment: {note}')
        return note

    def on_requested(self, msg):
        self.requested = msg.data

    def label(self, zone_key):
        z = self.zones.get(zone_key)
        return f'{z.get("label", zone_key)} ({z["colour"]})' if z else 'UNKNOWN'

    def on_verify(self, req, res):
        align_note = self.align() if self.align_on else 'off'
        t_req = time.monotonic()
        results, last_used, frame = [], t_req, None
        while len(results) < self.n_frames and time.monotonic() - t_req < self.timeout:
            img, t = self.cam.latest(newer_than=last_used)
            if img is None or t - last_used < self.spacing and results:
                time.sleep(0.02)
                continue
            last_used = t
            frame = vu.resize_to_width(img, self.width)
            results.append(vu.analyse_zone_frame(
                frame, self.ranges, self.zone_colours, self.min_zone_frac,
                self.min_shape_area, self.classifier, self.min_obj_score))
        if not results:
            res.success = False
            res.message = 'No fresh camera frames received - is the camera running?'
            self.get_logger().error(res.message)
            return res

        summary, agree = vu.combine_frames(results)
        detected = self.colour_to_zone.get(summary['colour'], 'unknown')
        requested = self.requested or 'unknown'
        match = detected == requested
        shapes = ', '.join(f'{k}={v}' for k, v in sorted(summary['shapes'].items())) or 'none'
        objects = ', '.join(f'{k}={v}' for k, v in sorted(summary['objects'].items())) or 'none'
        lines = [
            '================ ZONE VERIFICATION ================',
            f'Requested zone : {self.label(requested)}',
            f'Detected zone  : {self.label(detected)}',
            f'Colour dominance: {summary["dominance"]:.2f}  (share of zone-coloured pixels;'
            ' a relative measure, NOT a probability)',
            f'Frame agreement: {agree}/{len(results)} frames gave this result',
            f'Alignment      : {align_note}',
            f'Shapes ({summary["colour"]} only): {shapes}',
            f'Objects (CNN)  : {objects}' + ('' if self.classifier else '  [CNN not loaded]'),
            f'RESULT         : {"MATCH" if match else "MISMATCH"}',
            '===================================================',
        ]
        text = '\n'.join(lines)
        if frame is not None:
            try:
                os.makedirs(self.save_dir, exist_ok=True)
                img = vu.annotate(frame, self.ranges, summary['colour'], self.classifier,
                                  self.min_shape_area, self.min_obj_score)
                path = os.path.join(self.save_dir, time.strftime('verify_%H%M%S.png'))
                cv2.imwrite(path, img)
                text += f'\n(annotated image saved: {path})'
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f'could not save annotated image: {e}')
        self.get_logger().info('\n' + text)
        self.report_pub.publish(String(data=text))
        res.success = match
        res.message = text
        return res


def main(args=None):
    rclpy.init(args=args)
    node = ZoneVerifier()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
