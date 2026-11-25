#!/usr/bin/env python3
"""
destination_selector.py  -  ECTE477 Construction-Site Robot (v2)

The user-facing node and the navigation controller:
  1. Loads the zone goal poses from config/zones.yaml (nothing hard-coded); the robot
     stays stationary until the user picks a zone from the terminal menu.
  2. Sends the pose to Nav2 (/navigate_to_pose action). Nav2 plans the route on the
     map and avoids obstacles with the LiDAR costmaps - no fixed movement sequence.
  3. While driving it listens to /sheet_signal from sheet_detector:
        RED   -> cancel the Nav2 goal + publish zero velocity  => robot stops and stays
        GREEN -> re-send the SAME goal; Nav2 re-plans from the current pose
     RED is latched: losing sight of the red sheet does NOT resume; only GREEN does.
  4. On arrival: disables the sheet detector, waits `settle_s` for the view to settle,
     publishes the requested zone and calls /verify_zone (zone_verifier), then prints
     the requested zone, detected zone, counts and MATCH/MISMATCH.
  Ctrl+C while driving cancels the Nav2 goal so the robot really stops.

Communication used: action client (Nav2), service clients (SetBool enable, Trigger
verify), subscriber (/sheet_signal), publishers (/requested_zone, /cmd_vel).
"""
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from ecte477_cr_final.common import LATCHED, load_zones, share_path

STATUS_TEXT = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED (Nav2 could not reach the goal)',
}


def yaw_to_quaternion(yaw_rad):
    """Planar robot: rotation about Z only -> quaternion (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)


def wait_future(future, timeout_s=None, abort_event=None):
    """Wait for a future completed by the background executor thread (never spin here)."""
    start = time.monotonic()
    while not future.done():
        if abort_event is not None and abort_event.is_set():
            return False
        if timeout_s is not None and time.monotonic() - start > timeout_s:
            return False
        time.sleep(0.05)
    return True


class DestinationSelector(Node):
    def __init__(self):
        super().__init__('destination_selector')
        self.declare_parameter('zones_file', share_path('config', 'zones.yaml'))
        self.declare_parameter('settle_s', 1.0)
        self.declare_parameter('verify_timeout_s', 60.0)
        self.declare_parameter('use_sheet_stop', True)
        self.declare_parameter('feedback_period_s', 1.0)

        self.zones = load_zones(self.get_parameter('zones_file').value)
        self.start = self.load_start(self.get_parameter('zones_file').value)
        self.settle_s = float(self.get_parameter('settle_s').value)
        self.verify_timeout = float(self.get_parameter('verify_timeout_s').value)
        self.use_sheet = bool(self.get_parameter('use_sheet_stop').value)
        self.fb_period = float(self.get_parameter('feedback_period_s').value)

        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.enable_cli = self.create_client(SetBool, '/sheet_detector/enable')
        self.verify_cli = self.create_client(Trigger, '/verify_zone')
        self.clear_clis = [
            self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap'),
            self.create_client(ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')]
        self.req_pub = self.create_publisher(String, '/requested_zone', LATCHED)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(String, '/sheet_signal', self.on_sheet, LATCHED)

        self.red = threading.Event()
        self.green = threading.Event()
        self.paused = False
        self._last_fb = 0.0
        self.get_logger().info(f'ecte477_cr_final (FINAL v8) - loaded {len(self.zones)} zones: {list(self.zones)}')

    # ------------------------------------------------------------ callbacks
    def on_sheet(self, msg):
        if msg.data == 'RED':
            self.green.clear()
            self.red.set()
        elif msg.data == 'GREEN':
            self.green.set()

    def on_feedback(self, msg):
        now = time.monotonic()
        if self.paused or now - self._last_fb < self.fb_period:
            return
        self._last_fb = now
        fb = msg.feedback
        self.get_logger().info(f'NAVIGATING  distance remaining: {fb.distance_remaining:.2f} m'
                               f' | recoveries: {fb.number_of_recoveries}')

    # ------------------------------------------------------- start / localise
    @staticmethod
    def load_start(path):
        import yaml
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        st = data.get('start', {'x': 0.0, 'y': 0.0, 'yaw_deg': 0.0})
        return {k: float(st.get(k, 0.0)) for k in ('x', 'y', 'yaw_deg')}

    def set_initial_pose(self):
        """Tell AMCL the robot is on the taped START spot (exact, no mouse dragging)."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.start['x']
        msg.pose.pose.position.y = self.start['y']
        qx, qy, qz, qw = yaw_to_quaternion(math.radians(self.start['yaw_deg']))
        o = msg.pose.pose.orientation
        o.x, o.y, o.z, o.w = qx, qy, qz, qw
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.02           # ~14 cm standard deviation in x and y
        cov[35] = 0.03                   # ~10 deg standard deviation in yaw
        msg.pose.covariance = cov
        for _ in range(3):
            self.init_pub.publish(msg)
            time.sleep(0.2)
        self.get_logger().info(
            f'Initial pose set to START x={self.start["x"]:.2f} y={self.start["y"]:.2f} '
            f'yaw={self.start["yaw_deg"]:.0f} deg. Check in RViz that the red scan dots sit on '
            'the walls; if not, fix it with 2D Pose Estimate.')

    # -------------------------------------------------------------- helpers
    def call(self, client, request, timeout_s):
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        fut = client.call_async(request)
        return fut.result() if wait_future(fut, timeout_s) else None

    def set_detector(self, on):
        if not self.use_sheet:
            return
        r = self.call(self.enable_cli, SetBool.Request(data=on), 3.0)
        if r is None:
            self.get_logger().warn('sheet_detector not reachable - red/green stop unavailable')

    def clear_costmaps(self):
        """Forget temporary obstacles (people, held sheets) before a new trip so old
        marks cannot block the path. Best effort - skipped if the service is missing."""
        for cli in self.clear_clis:
            if cli.wait_for_service(timeout_sec=0.5):
                wait_future(cli.call_async(ClearEntireCostmap.Request()), 1.0)

    def stop_robot(self):
        for _ in range(5):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)

    def make_goal(self, zone):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(zone['x'])
        pose.pose.position.y = float(zone['y'])
        qx, qy, qz, qw = yaw_to_quaternion(math.radians(float(zone['yaw_deg'])))
        pose.pose.orientation.x, pose.pose.orientation.y = qx, qy
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        goal = NavigateToPose.Goal()
        goal.pose = pose
        return goal

    def send_goal(self, zone):
        fut = self.nav.send_goal_async(self.make_goal(zone), feedback_callback=self.on_feedback)
        if not wait_future(fut, 10.0):
            return None
        handle = fut.result()
        return handle if handle is not None and handle.accepted else None

    def cancel(self, handle, result_future):
        cf = handle.cancel_goal_async()
        wait_future(cf, 3.0)
        wait_future(result_future, 5.0)

    # ----------------------------------------------------------- navigation
    def navigate(self, key):
        """Drive to a zone with red/green pause support. Returns final GoalStatus."""
        zone = self.zones[key]
        name = f'{zone.get("label", key)} ({zone["colour"]})'
        if not self.nav.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 not available - is navigation running and ACTIVE?')
            return None
        self.set_detector(True)          # resets detector state to NONE
        self.red.clear()
        self.green.clear()
        self.get_logger().info(f'Going to {name}: x={zone["x"]:.2f} y={zone["y"]:.2f} '
                               f'yaw={zone["yaw_deg"]:.0f} deg')
        handle = None
        try:
            while True:
                self.paused = False
                self.clear_costmaps()
                handle = self.send_goal(zone)
                if handle is None:
                    self.get_logger().error('Goal rejected by Nav2. Did you do 2D Pose Estimate '
                                            'and is Navigation "active" in RViz?')
                    return None
                result = handle.get_result_async()
                stopped_by_red = not wait_future(result, abort_event=self.red)
                if not stopped_by_red:
                    break
                # ---- RED: stop and hold until GREEN ----
                self.paused = True
                self.cancel(handle, result)
                self.stop_robot()
                self.get_logger().warn('RED sheet detected -> STOPPED. Show the GREEN sheet '
                                       'to resume.')
                while not self.green.is_set():
                    time.sleep(0.05)
                self.red.clear()
                self.green.clear()
                self.get_logger().info('GREEN sheet detected -> RESUMING to the same zone.')
            status = result.result().status
        except KeyboardInterrupt:
            self.get_logger().warn('Ctrl+C - cancelling the Nav2 goal so the robot stops.')
            if handle is not None:
                self.cancel(handle, handle.get_result_async())
            self.stop_robot()
            self.set_detector(False)
            return GoalStatus.STATUS_CANCELED
        finally:
            self.paused = False
        self.set_detector(False)
        text = STATUS_TEXT.get(status, f'status {status}')
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'ARRIVED at {name}: {text}')
        else:
            self.get_logger().warn(f'Navigation to {name} ended: {text}')
        return status

    def verify(self, key):
        self.req_pub.publish(String(data=key))
        self.get_logger().info(f'Letting the view settle for {self.settle_s:.1f} s ...')
        time.sleep(self.settle_s)
        self.get_logger().info('Verifying zone with fresh camera frames ...')
        r = self.call(self.verify_cli, Trigger.Request(), self.verify_timeout)
        if r is None:
            self.get_logger().error('zone_verifier did not answer (is it running?)')
            return
        print('\n' + r.message + '\n', flush=True)

    # ---------------------------------------------------------------- menu
    def menu(self):
        keys = list(self.zones.keys())
        while rclpy.ok():
            print('\n=========== SELECT DESTINATION ===========')
            for i, k in enumerate(keys, 1):
                z = self.zones[k]
                print(f'  {i}. {z.get("label", k)} ({z["colour"]})')
            print('  s. Robot is on the taped START spot -> set initial pose')
            print('  q. Quit')
            try:
                choice = input('Choice: ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                print()
                return
            if choice == 'q':
                return
            if choice == 's':
                self.set_initial_pose()
                continue
            if choice.isdigit() and 1 <= int(choice) <= len(keys):
                key = keys[int(choice) - 1]
                if self.navigate(key) == GoalStatus.STATUS_SUCCEEDED:
                    self.verify(key)
            else:
                print('Invalid choice, try again.')


def main(args=None):
    # rclpy's own Ctrl+C handler is off so that Ctrl+C reaches our code while ROS is still
    # alive -> we can still send the cancel to Nav2.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = DestinationSelector()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        node.get_logger().info('Robot is stationary until you pick a zone. Localise it first '
                               '(2D Pose Estimate) and wait for Navigation: active.')
        node.menu()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
