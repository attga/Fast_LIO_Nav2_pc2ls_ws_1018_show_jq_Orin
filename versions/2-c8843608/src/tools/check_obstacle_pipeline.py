#!/usr/bin/env python3
"""Report whether near scan returns reach the local Nav2 costmap."""

import math
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    rclpy.init()
    node = rclpy.create_node('check_obstacle_pipeline')
    latest = {}

    def scan_callback(msg):
        finite = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r < msg.range_max]
        latest['scan'] = (len(finite), min(finite) if finite else math.inf,
                          sum(r < 2.0 for r in finite))

    def costmap_callback(msg):
        data = msg.data
        latest['costmap'] = (sum(0 < c < 100 for c in data),
                             sum(c >= 100 for c in data))

    node.create_subscription(LaserScan, '/scan', scan_callback, qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                             costmap_callback, QoSProfile(depth=1))
    deadline = time.monotonic() + 12.0
    try:
        while time.monotonic() < deadline and len(latest) < 2:
            rclpy.spin_once(node, timeout_sec=0.2)
        if len(latest) < 2:
            print('ERROR: missing /scan or /local_costmap/costmap', file=sys.stderr)
            return 1
        valid, nearest, close = latest['scan']
        inflated, occupied = latest['costmap']
        print(f'/scan: valid={valid}, nearest={nearest:.2f}m, rays<2m={close}')
        print(f'/local_costmap/costmap: inflated={inflated}, occupied={occupied}')
        if close > 0 and inflated + occupied == 0:
            print('WARNING: near scan returns but no local obstacle cells; inspect TF and height limits')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
