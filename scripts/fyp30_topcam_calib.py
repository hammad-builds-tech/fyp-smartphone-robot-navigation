#!/usr/bin/env python3
"""Data-driven calibration of the top evidence camera (viewing instrument).

Teleports fyp_robot to a grid of known world poses via the gz set_pose
service, saves one rendered frame per pose, auto-detects the robot blob
(the only newly-dark moved object vs a background frame), and fits
pixel = H @ [x, y, 1] with a least-squares homography.

Output: /tmp/top_calib.json  { H (3x3), pts, err_px }
"""
import json
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

TOPIC = '/top_cam/image'
WORLD = 'capture_fresh'
# grid east of comp-65 (wall bbox x 2.21-4.31, y 12.53-16.33) - avoid it
CENTER = (6.0, 15.6)
OFFS = [(0, 0), (1.8, 0), (0, 1.5), (0, -1.5),
        (1.4, 1.0), (-1.4, 1.0), (1.4, -1.0), (-1.4, -1.0)]
BRIDGE = CvBridge()
OUT = '/tmp/top_calib.json'

rclpy.init()
node = rclpy.create_node('top_calib')
latest = {}
node.create_subscription(Image, TOPIC, lambda m: latest.__setitem__('m', m), 10)


def grab(timeout=8.0):
    latest.pop('m', None)
    t0 = time.time()
    while 'm' not in latest and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    if 'm' not in latest:
        return None
    return BRIDGE.imgmsg_to_cv2(latest['m'], 'bgr8')


def teleport(x, y):
    req = ("service: /world/capture_fresh/set_pose\n"
           "request:\n"
           "  name: fyp_robot\n"
           "  position: { x: %.3f, y: %.3f, z: 0.1 }\n"
           "  orientation: { w: 1.0 }") % (x, y)
    subprocess.run(['gz', 'service', '-s', '/world/capture_fresh/set_pose',
                    '--reqtype', 'gz.msgs.SetPose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '3000', '--req', req],
                   capture_output=True, text=True, timeout=20)


pts = []
prev = None
for k, (dx, dy) in enumerate(OFFS):
    wx, wy = CENTER[0] + dx, CENTER[1] + dy
    teleport(wx, wy)
    time.sleep(1.0)
    best = None
    for _ in range(4):
        im = grab()
        if im is None:
            continue
        if prev is None:
            prev = im.copy()
            continue
        g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        gp = cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY)
        d = cv2.absdiff(g, gp)
        d = cv2.threshold(d, 15, 255, cv2.THRESH_BINARY)[1]
        d = cv2.morphologyEx(d, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        n, lb, st, ce = cv2.connectedComponentsWithStats(d, 8)
        prev = im.copy()
        if n > 1:
            j = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
            if st[j, cv2.CC_STAT_AREA] > 500:
                best = ce[j]
    if best is None:
        print(f"pose {k} ({wx:.2f},{wy:.2f}): blob not found")
        continue
    pts.append((wx, wy, best[0], best[1]))
    print(f"pose {k}: world ({wx:.2f},{wy:.2f}) -> pixel ({best[0]:.1f},{best[1]:.1f})")

if len(pts) < 4:
    sys.exit("not enough calibration points")

wp = np.array([(p[0], p[1], 1.0) for p in pts])
pp = np.array([(p[2], p[3], 1.0) for p in pts])
Hm, _ = cv2.findHomography(wp.reshape(-1, 1, 2), pp.reshape(-1, 1, 2), 0)
proj = np.array([Hm @ w for w in wp])
proj = proj[:, :2] / proj[:, 2:3]
err = np.hypot(proj[:, 0] - pp[:, 0], proj[:, 1] - pp[:, 1])
print(f"homography reprojection error: {err.mean():.2f} px mean")
json.dump(dict(H=Hm.tolist(), pts=pts, err_px=float(err.mean())),
          open(OUT, 'w'), indent=2)
print("wrote", OUT)
