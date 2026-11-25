"""Small helpers shared by the nodes (package paths, latched QoS)."""
import os

from ament_index_python.packages import get_package_share_directory
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

PKG = 'ecte477_cr_final'

# "Latched" topic: a node that subscribes late still receives the last message.
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def share_path(*parts):
    return os.path.join(get_package_share_directory(PKG), *parts)


def load_zones(path):
    import yaml
    with open(path, 'r') as f:
        zones = yaml.safe_load(f)['zones']
    for key, z in zones.items():
        for field in ('x', 'y', 'yaw_deg', 'colour'):
            if field not in z:
                raise ValueError(f'zone "{key}" is missing "{field}" in {path}')
    return zones
