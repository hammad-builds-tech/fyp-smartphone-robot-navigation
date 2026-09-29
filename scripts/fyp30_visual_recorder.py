#!/usr/bin/env python3
"""Session evidence recorder (viewing instrument only, not part of the pipeline).

Subscribes the in-sim evidence camera (bridged from gz), /cmd_vel and /odom.
Saves a Gazebo frame every ~1.5 s into <out>/frames/ and logs velocity+pose
at 5 Hz to <out>/visual_log.csv. Exits after --seconds or on Ctrl-C.
"""
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp/visual_evidence"
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 600.0
TOPIC = sys.argv[3] if len(sys.argv) > 3 else '/evidence_cam/image'
os.makedirs(f"{OUT}/frames", exist_ok=True)


class Rec(Node):
    def __init__(self):
        super().__init__('fyp30_visual_recorder')
        self.bridge = CvBridge()
        self.vel = (0.0, 0.0)
        self.pose = None
        self.frame_count = 0
        self.last_frame_t = 0.0
        qos = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, TOPIC, self.on_img, qos)
        self.create_subscription(Twist, '/cmd_vel', self.on_vel, qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos)
        self.log = open(f"{OUT}/visual_log.csv", "w", buffering=1)
        self.log.write("t,robot_x_odom,robot_y_odom,pose_ok,cmd_vx,cmd_wz,frames\n")
        self.t0 = time.time()

    def on_img(self, m):
        now = time.time()
        if now - self.last_frame_t < 1.5:
            return
        self.last_frame_t = now
        try:
            img = self.bridge.imgmsg_to_cv2(m, 'rgb8')
        except Exception:
            return
        self.frame_count += 1
        cv2.imwrite(f"{OUT}/frames/frame_{self.frame_count:03d}.png",
                    cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

    def on_vel(self, m):
        self.vel = (m.linear.x, m.angular.z)

    def on_odom(self, m):
        p = m.pose.pose.position
        self.pose = (p.x, p.y)

    def run(self):
        while time.time() - self.t0 < SECONDS:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.pose is not None:
                self.log.write("%.1f,%.3f,%.3f,1,%.3f,%.3f,%d\n" % (
                    time.time() - self.t0, self.pose[0], self.pose[1],
                    self.vel[0], self.vel[1], self.frame_count))
            else:
                self.log.write("%.1f,0,0,0,%.3f,%.3f,%d\n" % (
                    time.time() - self.t0, self.vel[0], self.vel[1],
                    self.frame_count))


rclpy.init()
n = Rec()
try:
    n.run()
finally:
    print(f"recorder done: {n.frame_count} gz frames, log at {OUT}/visual_log.csv")
    n.destroy_node()
    rclpy.shutdown()
