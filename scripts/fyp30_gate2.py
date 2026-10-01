#!/usr/bin/env python3
"""Sim-time traction gate (wall-clock gave false alarms under RTF<1).

Drives 0.3 m/s on /cmd_vel until the SIM clock has advanced 3.0 s, then
requires >= 0.7 m of physical displacement (odom==world via OdometryPublisher,
verified separately). Exit 0 = PASS. Wall budget 120 s.

Run: /usr/bin/python3 fyp30_gate2.py   (ROS sourced, ROS_DOMAIN_ID=0)
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

rclpy.init()
node = rclpy.create_node('fyp30_gate2')
state = {'sec': None, 'x': None, 'y': None}

node.create_subscription(Clock, '/clock',
                         lambda m: state.__setitem__('sec', m.clock.sec + m.clock.nanosec / 1e9), 10)
node.create_subscription(Odometry, '/odom',
                         lambda m: (state.__setitem__('x', m.pose.pose.position.x),
                                    state.__setitem__('y', m.pose.pose.position.y)), 10)
pub = node.create_publisher(Twist, '/cmd_vel', 10)

t0 = time.time()
while state['sec'] is None and time.time() - t0 < 15:
    rclpy.spin_once(node, timeout_sec=0.2)
while state['x'] is None and time.time() - t0 < 15:
    rclpy.spin_once(node, timeout_sec=0.2)
if state['sec'] is None or state['x'] is None:
    print("GATE FAIL: no /clock or /odom flow")
    sys.exit(2)

t_start = state['sec']
x0, y0 = state['x'], state['y']
cmd = Twist()
cmd.linear.x = 0.3
while state['sec'] - t_start < 3.0 and time.time() - t0 < 120:
    pub.publish(cmd)
    rclpy.spin_once(node, timeout_sec=0.05)
for _ in range(10):
    pub.publish(Twist())
    rclpy.spin_once(node, timeout_sec=0.02)
time.sleep(0.5)
rclpy.spin_once(node, timeout_sec=0.3)

dist = math.hypot(state['x'] - x0, state['y'] - y0)
sim_dt = state['sec'] - t_start
print(f"GATE: sim_dt={sim_dt:.2f}s dist={dist:.3f} m "
      f"start=({x0:.2f},{y0:.2f}) end=({state['x']:.2f},{state['y']:.2f})")
ok = dist >= 0.7 and sim_dt >= 2.9
print("GATE PASS" if ok else "GATE FAIL")
sys.exit(0 if ok else 1)
