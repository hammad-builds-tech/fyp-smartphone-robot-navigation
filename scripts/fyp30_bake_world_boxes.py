#!/usr/bin/env python3
"""Bake the world SDF's real obstacle boxes into the map pgm.

The map pgm is a sparse sketch of the reconstruction (2,453 occupied px; the
comp-65 wall band has gaps at y>=16.6), while the world SDF built from the
same reconstruction has solid boxes there (x 2.33-3.28, y 15.65-17.21). The
robot planned through a map gap and physically wedged on the real boxes.

This script rasterizes EVERY obstacle_* and wall_* box from the world SDF
onto the pgm grid (cell center inside grown box -> lethal), preserving the
existing SLAM lethal, and writes capture_fresh_baked.pgm/.yaml. Pure
reconstruction data - no synthetic geometry is added.

pgm row convention VERIFIED live: row 0 = TOP = max y (matches the running
map_server's costmap output pixel-for-pixel).
"""
import re
import math

import numpy as np
import cv2

BASE = "/home/hammad/FYP/realroom/capture_fresh"
WORLD = f"{BASE}/realroom_capture_fresh_world.sdf"
MAP_YAML = f"{BASE}/realroom_capture_fresh.yaml"
OUT_PGM = f"{BASE}/capture_fresh_baked.pgm"
OUT_YAML = f"{BASE}/capture_fresh_baked.yaml"

GROW = 0.25  # half robot footprint: fringe-hugging can no longer clip corners

ytxt = open(MAP_YAML).read()
res = float(re.search(r"resolution:\s*([\d.]+)", ytxt).group(1))
ox, oy = [float(v) for v in
          re.search(r"origin:\s*\[([^\]]+)\]", ytxt).group(1).split(",")[:2]]
img = cv2.imread(f"{BASE}/realroom_capture_fresh.pgm", cv2.IMREAD_GRAYSCALE)
H, W = img.shape
grid = img.copy()

sdf = open(WORLD).read()
blocks = re.findall(
    r'<model name="((?:obstacle|wall)_\w+)">.*?<pose>([^<]+)</pose>.*?'
    r'<size>([^<]+)</size>.*?</model>', sdf, re.S)
boxes = []
for name, pose, size in blocks:
    p = [float(v) for v in pose.split()]
    s = [float(v) for v in size.split()]
    if len(p) < 6 or len(s) < 3:
        continue
    boxes.append((name, p[0], p[1], p[5], s[0], s[1]))
print(f"map {W}x{H} res {res} origin ({ox},{oy}); parsed {len(boxes)} boxes")

def to_row(wy):
    """world y -> image row, row0=TOP convention."""
    return H - 1 - int(round((wy - oy) / res))

lethal = 0
for name, cx, cy, yaw, sx, sy in boxes:
    hs, hl = sx / 2.0 + GROW, sy / 2.0 + GROW
    if sx < sy:
        hs, hl = hl, hs
        yaw += math.pi / 2.0
    c, sn = math.cos(yaw), math.sin(yaw)
    rx = hs * abs(c) + hl * abs(sn)
    ry = hs * abs(sn) + hl * abs(c)
    gx0 = max(0, int((cx - rx - ox) / res))
    gx1 = min(W - 1, int((cx + rx - ox) / res) + 1)
    r0 = max(0, to_row(cy + ry))
    r1 = min(H - 1, to_row(cy - ry))
    for r in range(r0, r1 + 1):
        wy = oy + (H - 1 - r) * res
        dy = wy - cy
        for gx in range(gx0, gx1 + 1):
            wx = ox + gx * res
            dx = wx - cx
            u = c * dx + sn * dy
            v = -sn * dx + c * dy
            if abs(u) <= hs and abs(v) <= hl and grid[r, gx] > 64:
                grid[r, gx] = 0
                lethal += 1
print(f"baked {lethal} new lethal cells (grow {GROW} m, {len(boxes)} boxes)")

cv2.imwrite(OUT_PGM, grid)
open(OUT_YAML, "w").write(
    f"image: {OUT_PGM}\nresolution: {res}\norigin: [{ox}, {oy}, 0.0]\n"
    f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n")
print(f"wrote {OUT_PGM} + {OUT_YAML}")

# Verification: same ASCII window that exposed the mismatch + key cells free
def ascii_window(grid_, label):
    print(f"--- {label}: x[2.3,3.6] y[15.0,17.6] ---")
    for wy in np.arange(17.5, 14.9, -0.25):
        row = ""
        for wx in np.arange(2.3, 3.65, 0.05):
            gx = int(round((wx - ox) / res))
            r = to_row(wy)
            v = grid_[r, gx]
            row += "#" if v <= 64 else ("." if v >= 191 else ":")
        print(f"y={wy:5.2f} {row}")

ascii_window(img, "ORIGINAL")
ascii_window(grid, "BAKED")
for label, wx, wy in [("spawn", 8.01, 13.73), ("leg1 goal", 0.11, 17.98)]:
    gx = int(round((wx - ox) / res))
    v = grid[to_row(wy), gx]
    print(f"{label} ({wx},{wy}): value {v} -> {'FREE' if v > 64 else 'LETHAL!'}")
