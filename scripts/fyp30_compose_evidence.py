#!/usr/bin/env python3
"""Compose VISUAL evidence for the capture_fresh demo (viewing instrument only).

Inputs (all generated during the demo run):
  <DS>/realroom_capture_fresh.pgm/.yaml   occupancy map -> obstacle components
  /tmp/ve_final/gz_pose.csv               Gazebo ground-truth fyp_robot pose (2 Hz)
  /tmp/ve_final/visual_log.csv            /odom pose + /cmd_vel (5 Hz)
  /tmp/ve_final/frames/*.png              /evidence_cam Gazebo renders (1.5 s)

Outputs into <DS>/evidence/:
  calib.json            auto data->pixel fit of camera pose (x0,y0,scale)
  traj_on_frames.png    trajectory + robot markers drawn on 4 camera frames
  map_view.png          map with comps 61/65 + gz ground-truth trajectory
  detour_panel.png      2x2: map view + three frames = one contact sheet
"""
import glob
import json
import math
import os
import sys

import cv2
import numpy as np
import yaml

DS = "/home/hammad/FYP/realroom/capture_fresh"
VE = sys.argv[1] if len(sys.argv) > 1 else "/tmp/ve_run4"
EV = f"{DS}/evidence"
os.makedirs(EV, exist_ok=True)

# ---------------- map + components -------------------------------------------
meta = yaml.safe_load(open(f"{DS}/realroom_capture_fresh.yaml"))
res = float(meta["resolution"]); ox, oy = map(float, meta["origin"][:2])
pgm = cv2.imread(f"{DS}/realroom_capture_fresh.pgm", cv2.IMREAD_GRAYSCALE)
H, W = pgm.shape
occ = (pgm < 100).astype(np.uint8)
n_lbl, lbl, stats, _ = cv2.connectedComponentsWithStats(occ, connectivity=8)

def cell_of(x, y):
    c = int((x - ox) / res); r = H - 1 - int((y - oy) / res)
    return r, c

# ---------------- data --------------------------------------------------------
gz = []
for line in open(f"{VE}/gz_pose.csv").read().splitlines()[1:]:
    p = line.split(",")
    if len(p) >= 3 and p[1] and p[2]:
        gz.append((float(p[1]), float(p[2])))
gz = np.array(gz)
rows = list(open(f"{VE}/visual_log.csv").read().splitlines()[1:])
od = []
for line in rows:
    p = line.split(",")
    if len(p) >= 6:
        try:
            od.append((float(p[1]), float(p[2]), float(p[4])))  # odom is world-frame now
        except ValueError:
            pass
od = np.array(od)

print(f"gz samples: {len(gz)}, odom samples: {len(od)}")

# ---------------- auto calibration data->pixels ------------------------------
# evidence camera: pose (0.1, 9.0, 14.0), pitch -1.15, fov 1.3, 1280x720
frames = sorted(glob.glob(f"{VE}/frames/*.png"))
if not frames:
    sys.exit("no frames found")
img0 = cv2.imread(frames[0])
FH, FW = img0.shape[:2]

def project(wx, wy, x0, y0, s):
    u = FW / 2 + (wx - x0) * s
    v = FH / 2 + (wy - y0) * s * 0.62   # foreshortening from the steep pitch
    return float(u), float(v)

xs, ys = gz[:, 0], gz[:, 1]
cx, cy = xs.mean(), ys.mean()
d0 = np.hypot(xs - cx, ys - cy).max()
best = None
for s in np.arange(80, 340, 5):
    pts = np.array([project(x, y, cx, cy, s) for x, y in gz])
    u, v = pts[:, 0], pts[:, 1]
    inb = ((u >= 20) & (u <= FW - 20) & (v >= 20) & (v <= FH - 20)).mean()
    spread = (u.max() - u.min()) * (v.max() - v.min())
    score = inb - 1e-7 * spread
    if best is None or score > best[0]:
        best = (score, s, inb)
scale = best[1]
print(f"calibration: scale={scale:.0f} px/m, in-bounds frac={best[2]:.2f}")
json.dump(dict(x0=float(cx), y0=float(cy), scale=float(scale), pitch_corr=0.62),
          open(f"{EV}/calib.json", "w"), indent=2)

# ---------------- draw trajectory on frames ----------------------------------
def draw_world(cvimg, wx, wy, color, thick=3, dot=9):
    for x, y in zip(wx, wy):
        u, v = project(x, y, cx, cy, scale)
        if 0 <= u < FW and 0 <= v < FH:
            cv2.circle(cvimg, (int(u), int(v)), dot, color, -1)
    for a, b in zip(zip(wx, wy), list(zip(wx, wy))[1:]):
        ua, va = project(*a, cx, cy, scale)
        ub, vb = project(*b, cx, cy, scale)
        if 0 <= ua < FW and 0 <= va < FH and 0 <= ub < FW and 0 <= vb < FH:
            cv2.line(cvimg, (int(ua), int(va)), (int(ub), int(vb)), color, thick)

pick = [frames[int(k * (len(frames) - 1) / 3)] for k in range(4)]
panels = []
for i, fp in enumerate(pick):
    im = cv2.imread(fp)
    draw_world(im, gz[:, 0], gz[:, 1], (0, 90, 255), 2, 4)      # path
    if len(gz):
        u, v = project(gz[-1, 0], gz[-1, 1], cx, cy, scale)
        cv2.circle(im, (int(u), int(v)), 12, (0, 0, 255), 3)     # final position
        cv2.circle(im, (int(u), int(v)), 5, (0, 0, 255), -1)
    stamp = f"t={i * (len(frames) - 1) * 1.5:.0f}s  gz pose=({gz[-1,0]:.2f},{gz[-1,1]:.2f})" if len(gz) else ""
    cv2.rectangle(im, (0, 0), (FW, 46), (30, 30, 30), -1)
    cv2.putText(im, stamp, (14, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.85,
                (255, 255, 255), 2, cv2.LINE_AA)
    panels.append(im)

traj_img = np.vstack(panels)
cv2.imwrite(f"{EV}/traj_on_frames.png", traj_img)

# ---------------- map view with comps + gz trajectory ------------------------
mv = cv2.cvtColor(pgm, cv2.COLOR_GRAY2BGR)
for i in range(1, n_lbl):
    if stats[i, cv2.CC_STAT_AREA] >= 25:
        m = (lbl == i)
        if i == 65:
            mv[m] = (0, 0, 255)
        elif i == 61:
            mv[m] = (0, 140, 255)
        else:
            mv[m] = (160, 160, 160)
if len(gz):
    for x, y in gz:
        r, c = cell_of(x, y)
        if 0 <= r < H and 0 <= c < W:
            cv2.circle(mv, (c, r), 1, (0, 255, 0), -1)
    r, c = cell_of(gz[0, 0], gz[0, 1])
    cv2.circle(mv, (c, r), 7, (255, 0, 0), -1)
    r, c = cell_of(gz[-1, 0], gz[-1, 1])
    cv2.circle(mv, (c, r), 7, (0, 0, 255), -1)
mv = cv2.resize(mv, (W * 2, H * 2), interpolation=cv2.INTER_NEAREST)
cv2.putText(mv, "map: comp65=red comp61=orange gz ground-truth path=green",
            (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
cv2.imwrite(f"{EV}/map_view.png", mv)

# ---------------- contact sheet ----------------------------------------------
p2 = [cv2.resize(p, (854, 480)) for p in panels[1:3]]
right = np.vstack(p2)
h_target = right.shape[0]
mv_r = cv2.resize(mv, (int(mv.shape[1] * h_target / mv.shape[0]), h_target))
w_left = mv_r.shape[1]
sheet = np.full((h_target, w_left + right.shape[1] + 8, 3), 20, np.uint8)
sheet[:h_target, :w_left] = mv_r
sheet[:right.shape[0], w_left + 8:] = right
cv2.imwrite(f"{EV}/detour_panel.png", sheet)
print("wrote traj_on_frames.png, map_view.png, detour_panel.png, calib.json")
