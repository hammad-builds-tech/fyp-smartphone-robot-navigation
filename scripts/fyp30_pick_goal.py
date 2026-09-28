#!/usr/bin/env python3
"""Pick spawn/goal whose straight line passes THROUGH real captured geometry.

Searches random pairs of walkable cells (free space carved by capture rays,
within 4 m of the camera path, >= 0.45 m clearance) 4-9 m apart, and keeps
the pair whose connecting segment crosses the most captured-geometry cells
(wall segments and furniture both count — both are reconstructed obstacles).
"""
import math
import sys

import cv2
import numpy as np
import yaml

MAP = sys.argv[1] if len(sys.argv) > 1 else "realroom_fresh.yaml"
POSES = sys.argv[2] if len(sys.argv) > 2 else "colmap/sparse/txt/images.txt"

meta = yaml.safe_load(open(MAP))
res = float(meta["resolution"]); ox, oy = map(float, meta["origin"][:2])
with open(MAP.replace(".yaml", ".pgm"), "rb") as f:
    assert f.readline().strip() == b"P5"
    line = f.readline()
    while line.startswith(b"#"):
        line = f.readline()
    W, H = map(int, line.split()); f.readline()
    pgm = np.frombuffer(f.read(), dtype=np.uint8).reshape(H, W)
occ = (pgm < 100).astype(np.uint8); free = (pgm == 205)

# same geometry filter as the world builder: components >= 4 cells
n_lbl, lbl, stats, _ = cv2.connectedComponentsWithStats(occ, connectivity=8)
occ_real = np.zeros_like(occ)
for i in range(1, n_lbl):
    if stats[i, cv2.CC_STAT_AREA] >= 4:
        occ_real[lbl == i] = 1

cams = []
for line in open(POSES):
    line = line.strip()
    if not line or line.startswith("#"):
        continue
    p = line.split()
    if len(p) >= 10 and p[0].isdigit():
        tx, ty, tz = map(float, p[5:8]); cams.append([tx, ty, tz])
cam_mask = np.zeros((H, W), np.uint8)
for tx, ty, tz in cams:
    c = int((ty - ox) / res); r = int((tz - oy) / res)
    if 0 <= r < H and 0 <= c < W:
        cam_mask[r, c] = 1
dt_cam = cv2.distanceTransform(1 - cam_mask, cv2.DIST_L2, 5)
clear = cv2.distanceTransform((~occ_real.astype(bool)).astype(np.uint8), cv2.DIST_L2, 5)
# free cells within 4 m of the camera path: all carved by real capture rays
walkable = (free == 1) & (dt_cam <= 80) & (clear >= 9)
n2, lbl2 = cv2.connectedComponents(walkable.astype(np.uint8), connectivity=8)
sizes = sorted([(int((lbl2 == i).sum()), i) for i in range(1, n2)], reverse=True)
comp = (lbl2 == sizes[0][1])
print("largest walkable component:", sizes[0][0], "cells")

cand = np.argwhere(comp)
rng = np.random.default_rng(11)
idx = rng.choice(len(cand), size=min(len(cand), 12000), replace=False)
P = cand[idx]

best = None
tried = 0
for a in range(len(P)):
    ra, ca = P[a]
    rest = P[a + 1:]
    dd = np.hypot(rest[:, 1] - ca, rest[:, 0] - ra) * res
    sel = np.where((dd >= 4.0) & (dd <= 9.0))[0]
    if len(sel) > 60:
        sel = rng.choice(sel, size=60, replace=False)
    for b in sel:
        rb, cb = rest[b]
        d = dd[b]
        tried += 1
        npts = int(max(abs(rb - ra), abs(cb - ca))) + 1
        rr = np.linspace(ra, rb, npts).astype(int)
        cc = np.linspace(ca, cb, npts).astype(int)
        cross = int(occ_real[rr, cc].sum())
        if cross < 2:
            continue
        # the blocking geometry must be a substantial obstacle, not a speck
        ids = lbl[rr, cc]
        ids = ids[ids > 0]
        areas = [int(stats[i, cv2.CC_STAT_AREA]) for i in np.unique(ids)]
        big_area = max(areas)
        if big_area < 25:
            continue
        score = (min(big_area, 400) / 400.0 * 2.0
                 + min(cross, 20) * 0.3
                 + min(float(clear[ra, ca]), float(clear[rb, cb])) * 0.2
                 - 0.15 * abs(d - 6.5))
        if best is None or score > best[0]:
            best = (score, (ra, ca), (rb, cb), d, cross, big_area)
print("pairs tried:", tried)
if not best:
    sys.exit("NO crossing pair found")

_, (ra, ca), (rb, cb), d, cross, big_area = best

def to_world(r, c):
    return ox + (c + 0.5) * res, oy + (H - 1 - r + 0.5) * res

sx, sy = to_world(ra, ca)
gx, gy = to_world(rb, cb)
print("SPAWN world:", round(sx, 2), round(sy, 2), "clear:", round(float(clear[ra, ca]) * res, 2))
print("GOAL world:", round(gx, 2), round(gy, 2), "clear:", round(float(clear[rb, cb]) * res, 2))
print("straight line crosses", cross, "cells of captured geometry; biggest blocked component area:", big_area, "dist:", round(d, 2))
print("SPAWN_X=%.2f SPAWN_Y=%.2f GOAL_X=%.2f GOAL_Y=%.2f" % (sx, sy, gx, gy))
