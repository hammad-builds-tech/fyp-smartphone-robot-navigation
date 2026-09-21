#!/usr/bin/env python3
"""Sensor-QoS topic probe for run_system.sh readiness gates.

`ros2 topic hz` subscribes with RELIABLE QoS and therefore never receives
messages on best-effort sensor topics (/scan, gz-bridged /odom, ...), which
made the bringup readiness checks false-negative. This probe subscribes with
qos_profile_sensor_data and exits 0 once messages arrive.

Usage: topic_probe.py <topic>
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

TOPIC_TYPES = {
    "/scan": ("sensor_msgs.msg", "LaserScan"),
    "/odom": ("nav_msgs.msg", "Odometry"),
    "/camera/depth/points": ("sensor_msgs.msg", "PointCloud2"),
    "/depth_occupancy_grid": ("nav_msgs.msg", "OccupancyGrid"),
    "/smartphone_map": ("nav_msgs.msg", "OccupancyGrid"),
}


def main():
    topic = sys.argv[1]
    mod_name, cls_name = TOPIC_TYPES.get(
        topic, ("sensor_msgs.msg", "LaserScan")
    )
    import importlib

    msg_cls = getattr(importlib.import_module(mod_name), cls_name)

    rclpy.init()
    node = rclpy.node.Node("fyp_topic_probe")
    count = [0]
    node.create_subscription(
        msg_cls, topic, lambda m: count.__setitem__(0, count[0] + 1),
        qos_profile_sensor_data,
    )
    t0 = time.time()
    while time.time() - t0 < 10 and count[0] < 2:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if count[0] >= 2 else 1)


if __name__ == "__main__":
    main()
