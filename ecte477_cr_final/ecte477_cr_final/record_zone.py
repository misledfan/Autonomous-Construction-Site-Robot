#!/usr/bin/env python3
"""
record_zone.py - save the robot's CURRENT map pose as a zone goal in config/zones.yaml.

Drive the robot (teleop) to where its camera sees the zone sheet centred, ~0.5 m away,
make sure it is localised (scan on the walls in RViz), close teleop, then:

    ros2 run ecte477_cr_final record_zone electrical     # or mechanical / material_storage
    ros2 run ecte477_cr_final record_zone start          # the taped start spot
    ros2 run ecte477_cr_final record_zone --list         # show what is stored

Reads the map->base_link transform (same as `tf2_echo map base_link`), averages 10
readings, and writes x, y, yaw_deg into the zones.yaml in the SOURCE package (the
installed copy is a symlink to it), keeping label and colour.
"""
import math
import os
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from ecte477_cr_final.common import share_path

DEFAULTS = {
    'electrical': {'label': 'Electrical Zone', 'colour': 'blue'},
    'mechanical': {'label': 'Mechanical Zone', 'colour': 'pink'},
    'material_storage': {'label': 'Material Storage Zone', 'colour': 'yellow'},
}


def zones_path():
    return os.path.realpath(share_path('config', 'zones.yaml'))


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--ros-args')]
    path = zones_path()
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    zones = data.setdefault('zones', {})
    if not args or args[0] in ('-h', '--help'):
        sys.exit('usage: ros2 run ecte477_cr_final record_zone <zone_key> | --list\n'
                 f'zone keys: {", ".join(sorted(set(zones) | set(DEFAULTS)))}')
    if args[0] == '--list':
        print(path)
        st = data.get('start')
        if st:
            print(f'  {"start":17s} {"":7s} x={st["x"]:.3f} y={st["y"]:.3f} yaw={st["yaw_deg"]:.1f}')
        for k, z in zones.items():
            print(f'  {k:17s} {z.get("colour", "?"):7s} x={z["x"]:.3f} y={z["y"]:.3f} '
                  f'yaw={z["yaw_deg"]:.1f}')
        return
    key = args[0]

    rclpy.init()
    node = Node('record_zone')
    buf = Buffer()
    TransformListener(buf, node)
    xs, ys, ss, cs = [], [], [], []
    t0 = time.monotonic()
    while len(xs) < 10 and time.monotonic() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if buf.can_transform('map', 'base_link', Time()):
            tr = buf.lookup_transform('map', 'base_link', Time()).transform
            q = tr.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            xs.append(tr.translation.x)
            ys.append(tr.translation.y)
            ss.append(math.sin(yaw))
            cs.append(math.cos(yaw))
            time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()
    if not xs:
        sys.exit('No map->base_link transform. Is navigation running and the robot localised '
                 '(2D Pose Estimate)?')
    x, y = sum(xs) / len(xs), sum(ys) / len(ys)
    yaw_deg = math.degrees(math.atan2(sum(ss), sum(cs)))
    pose = {'x': round(x, 3), 'y': round(y, 3), 'yaw_deg': round(yaw_deg, 1)}
    if key == 'start':
        data['start'] = pose
    else:
        entry = zones.get(key, dict(DEFAULTS.get(key, {'label': key, 'colour': 'unknown'})))
        entry.update(pose)
        zones[key] = entry
    with open(path, 'w') as f:
        f.write('# Zone goal poses in the map frame (recorded with record_zone / tf map->base_link)\n')
        yaml.safe_dump(data, f, sort_keys=False)
    print(f'saved {key}: x={x:.3f} y={y:.3f} yaw={yaw_deg:.1f} deg  ({len(xs)} readings) -> {path}')


if __name__ == '__main__':
    main()
