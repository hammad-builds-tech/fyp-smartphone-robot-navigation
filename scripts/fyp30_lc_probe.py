#!/usr/bin/env python3
"""Decode the live /local_costmap/costmap around a world point.

Usage: /usr/bin/python3 fyp30_lc_probe.py <world_x> <world_y>
Prints an ASCII cost map (row = y descending, col = x ascending) of the
5x5 m window centred on the given world pose. Symbols:
  . free (0)   : low (1-99)   # lethal (100)  ? unknown (-1)
"""
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid

WX, WY = float(sys.argv[1]), float(sys.argv[2])

rclpy.init()
n = rclpy.create_node('lc_probe')
box = {}

n.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                      lambda m: box.__setitem__('msg', m), 10)

t0 = time.time()
while 'msg' not in box and time.time() - t0 < 45:
    rclpy.spin_once(n, timeout_sec=0.2)

if 'msg' not in box:
    print('NO COSTMAP MSG in 45 s')
    sys.exit(1)

m = box['msg']
res = m.info.resolution
ox = m.info.origin.position.x
oy = m.info.origin.position.y
W, H = m.info.width, m.info.height

cw = int((WX - ox) / res)
ch = int((WY - oy) / res)
half = 50  # 2.5 m half-window at 0.05 res

print(f'grid {W}x{H} res {res:.2f} origin ({ox:.2f},{oy:.2f}) '
      f'frame {m.header.frame_id}; probe at ({WX},{WY})')
# print rows from top (north) to bottom
for r in range(max(0, ch - half), min(H, ch + half), 5):
    row = ''
    for c in range(max(0, cw - half), min(W, cw + half), 2):
        v = m.data[r * W + c]
        row += '.' if v == 0 else ('?' if v < 0 else
                                   ('#' if v >= 99 else
                                    (':' if v >= 50 else ':')))
    print(f'y={oy + r * res:6.2f} {row}')

# stats in the 2x2 m box around the probe point
from collections import Counter
cnt = Counter()
for r in range(max(0, ch - 20), min(H, ch + 20)):
    for c in range(max(0, cw - 20), min(W, cw + 20)):
        v = m.data[r * W + c]
        cnt['lethal' if v >= 99 else ('unknown' if v < 0 else
                                      ('inflated' if v > 0 else 'free'))] += 1
print('4x4 m around probe:', dict(cnt))
