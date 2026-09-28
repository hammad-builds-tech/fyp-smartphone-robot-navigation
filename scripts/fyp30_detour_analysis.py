#!/usr/bin/env python3
"""Detour + provenance analysis for the 30-second realroom test.

1. DETOUR: executed trajectory (odom + spawn offset) vs straight spawn->goal
   line — length ratio, max perpendicular deviation, and which reconstructed
   component the executed path steered around (per-component straight-line
   threat vs executed clearance).
2. PROVENANCE: the blocking component's cells are matched back to the fused
   COLMAP+MiDaS point cloud and onto the recorded RGB frames.

FRAME NOTE: realroom_cloud_to_map.py writes the PGM with image row = (y-y0)/res
(row 0 = min-y), while map consumers (nav2 map_server, world builder, goal
picker) treat row 0 as TOP: world_y = oy + (H-1-row+0.5)*res. Hence the
consumed room is y-mirrored w.r.t. the cloud: cloud_y = 2*oy + H*res - world_y.
Robot/map/world are internally consistent (robot drove the map free space and
reached the goal); only the cloud<->map provenance link needs the mirror.

Run with the fyp conda env (open3d):  ~/miniconda3/envs/fyp/bin/python
"""
import json
import math
import os
import sys

import cv2
import numpy as np
import yaml

try:
    import open3d as o3d
    from scipy.spatial import cKDTree
except ImportError:
    sys.exit("run with the fyp conda env (needs open3d + scipy)")

BASE = sys.argv[1] if len(sys.argv) > 1 else "/home/hammad/FYP/realroom/capture_fresh"
MAP = f"{BASE}/realroom_{__import__('os').path.basename(BASE)}.yaml"
POSES = f"{BASE}/colmap/sparse/txt/images.txt"
CLOUD = f"{BASE}/fused_cloud.ply"
TRAJ = (f"{BASE}/trajectory_odom.log"
        if __import__('os').path.exists(f"{BASE}/trajectory_odom.log")
        else "/tmp/robot_traj_final_full.log")
PLAN = "/tmp/fyp30_plan.log"
OUT = f"{BASE}/evidence"

# spawn/goal come from the dataset's own nav_targets.env (auto-selected)
SPAWN = GOAL = None
for line in open(f"{BASE}/nav_targets.env"):
    if line.startswith("SPAWN_X="):
        SPAWN = (float(line.split("=")[1]), None)
    elif line.startswith("SPAWN_Y="):
        SPAWN = (SPAWN[0], float(line.split("=")[1]))
    elif line.startswith("GOAL_X="):
        GOAL = (float(line.split("=")[1]), None)
    elif line.startswith("GOAL_Y="):
        GOAL = (GOAL[0], float(line.split("=")[1]))
assert SPAWN and GOAL and SPAWN[1] is not None and GOAL[1] is not None, "nav_targets.env incomplete"

# ---------------- map + geometry -------------------------------------------
meta = yaml.safe_load(open(MAP))
res = float(meta["resolution"]); ox, oy = map(float, meta["origin"][:2])
with open(MAP.replace(".yaml", ".pgm"), "rb") as f:
    assert f.readline().strip() == b"P5"
    line = f.readline()
    while line.startswith(b"#"):
        line = f.readline()
    W, H = map(int, line.split()); f.readline()
    pgm = np.frombuffer(f.read(), dtype=np.uint8).reshape(H, W)
occ = (pgm < 100).astype(np.uint8)

n_lbl, lbl, stats, _ = cv2.connectedComponentsWithStats(occ, connectivity=8)
real_ids = [i for i in range(1, n_lbl) if stats[i, cv2.CC_STAT_AREA] >= 4]

def cell_of(x, y):
    c = int((x - ox) / res)
    r = H - 1 - int((y - oy) / res)
    return max(0, min(H - 1, r)), max(0, min(W - 1, c))

def world_of(r, c):
    return ox + (c + 0.5) * res, oy + (H - 1 - r + 0.5) * res

def world_to_cloud(p):
    """consumed map frame -> fused-cloud (rotated COLMAP) frame (y-mirror)."""
    return np.array([p[0], 2.0 * oy + H * res - p[1]])

comp_cells, comp_world = {}, {}
for i in real_ids:
    cells = np.argwhere(lbl == i)
    comp_cells[i] = cells
    comp_world[i] = np.array([world_of(r, c) for r, c in cells])

# ---------------- trajectory ------------------------------------------------
pts, segs = [], []
for line in open(TRAJ):
    line = line.strip()
    if line.startswith("# --- run start"):
        if pts:
            segs.append(pts)
        pts = []
        continue
    if not line:
        continue
    p = line.split()
    if len(p) >= 3:
        pts.append([float(p[1]) + SPAWN[0], float(p[2]) + SPAWN[1]])
if pts:
    segs.append(pts)
assert segs, "trajectory log has no run segments"
pts = segs[-1]   # analyse the MOST RECENT run only (driver appends per run)
traj = np.array(pts)
keep = [traj[0]]
for p in traj[1:]:
    if np.hypot(*(p - keep[-1])) >= 0.05:
        keep.append(p)
traj_ds = np.array(keep)
exec_len = float(np.hypot(*np.diff(traj_ds, axis=0).T).sum())
straight = float(np.hypot(GOAL[0] - SPAWN[0], GOAL[1] - SPAWN[1]))
print(f"executed length: {exec_len:.2f} m | straight: {straight:.2f} m "
      f"| ratio: {exec_len / straight:.3f}")

d_vec = np.array([GOAL[0] - SPAWN[0], GOAL[1] - SPAWN[1]])
L2 = d_vec @ d_vec
rel = traj_ds - np.array(SPAWN)
t_along = np.clip((rel @ d_vec) / L2, 0.0, 1.0)
proj = np.array(SPAWN) + t_along[:, None] * d_vec
dev = np.hypot(*(traj_ds - proj).T)
i_max = int(np.argmax(dev))
bend = traj_ds[i_max]
print(f"max perpendicular deviation: {dev[i_max]:.2f} m at t={t_along[i_max]:.2f} "
      f"({t_along[i_max] * straight:.2f} m along the line), point ({bend[0]:.2f},{bend[1]:.2f})")

# straight line sampled densely (for threat tests)
n = int(straight / 0.02) + 1
ss = np.linspace(0.0, 1.0, n)
line_pts = np.stack([SPAWN[0] + ss * d_vec[0], SPAWN[1] + ss * d_vec[1]], axis=1)
tree_line = cKDTree(line_pts)

# ---------------- which real component does the path detour around? --------
print("\nper-real-component: dist(straight line) vs dist(executed path):")
cand = []
for i in real_ids:
    area = int(stats[i, cv2.CC_STAT_AREA])
    d_line = float(tree_line.query(comp_world[i])[0].min())
    d_path = float(cKDTree(traj_ds).query(comp_world[i])[0].min())
    if d_line <= 0.25 and area >= 25:      # line threatened substantial geometry
        cand.append((d_path - d_line, i, area, d_line, d_path))
cand.sort(reverse=True)
for gain, i, area, dl, dp in cand[:6]:
    print(f"  comp {i}: area={area} d_line={dl:.2f} d_path={dp:.2f} gain={gain:.2f}")
assert cand, "no substantial real geometry near the straight line"
gain, block_comp, block_area, dline_b, dpath_b = cand[0]
r0b, c0b = int(stats[block_comp, 1]), int(stats[block_comp, 0])
hb, wb = int(stats[block_comp, 3]), int(stats[block_comp, 2])
bx0, by0 = world_of(r0b, c0b); bx1, by1 = world_of(r0b + hb - 1, c0b + wb - 1)
print(f"BLOCKING comp id={block_comp} area={block_area} cells "
      f"bbox=({bx0:.2f},{by0:.2f})-({bx1:.2f},{by1:.2f}) m | "
      f"line passes {dline_b:.2f} m from it, path keeps {dpath_b:.2f} m")

# where along the line does it come closest to the blocking comp?
# (per LINE point: distance to nearest blocker cell -> t of the closest approach)
d_line_pts = cKDTree(comp_world[block_comp]).query(line_pts)[0]
t_hit = float(ss[int(np.argmin(d_line_pts))])
t_span = (float(ss[d_line_pts < 0.15].min()), float(ss[d_line_pts < 0.15].max())) \
    if (d_line_pts < 0.15).any() else (t_hit, t_hit)
dev_at_blk = float(np.interp(t_hit * straight, t_along * straight, dev))
dev_min_in_span = float(dev[(t_along >= t_span[0]) & (t_along <= t_span[1])].min()) \
    if (t_along >= t_span[0]).any() else dev_at_blk
print(f"line grazes blocker over t={t_span[0]:.2f}-{t_span[1]:.2f} "
      f"({t_span[0] * straight:.2f}-{t_span[1] * straight:.2f} m from spawn); "
      f"robot lateral offset there: min {dev_min_in_span:.2f} m, at closest approach {dev_at_blk:.2f} m")

# nearest REAL component to the bend point
brr, bcc = cell_of(*bend)
best = None
for i in real_ids:
    cells = comp_cells[i]
    dd = np.hypot(cells[:, 0] - brr, cells[:, 1] - bcc).min() * res
    if best is None or dd < best[0]:
        best = (dd, i)
bend_d, bend_comp = best
print(f"bend point hugs comp {bend_comp} (area {stats[bend_comp, cv2.CC_STAT_AREA]}, "
      f"{bend_d:.2f} m away)")

clear_blk = cKDTree(comp_world[block_comp]).query(traj_ds)[0]
dt_occ = cv2.distanceTransform(1 - occ, cv2.DIST_L2, 5) * res
clear_any = np.array([dt_occ[cell_of(x, y)] for x, y in traj_ds])
print(f"min clearance kept from blocking comp: {clear_blk.min():.2f} m "
      f"(mean {clear_blk.mean():.2f}); min from ANY occupied cell: {clear_any.min():.2f} m")

# ---------------- provenance: blocking comp <-> fused cloud ----------------
pcd = o3d.io.read_point_cloud(CLOUD)
pts3 = np.asarray(pcd.points)
ext = pts3.max(axis=0) - pts3.min(axis=0)
up = "xyz"[int(np.argmin(ext))]
order = {"x": [1, 2, 0], "y": [2, 0, 1], "z": [0, 1, 2]}[up]
pts3 = pts3[:, order]
centers = []
for line in open(POSES):
    line = line.strip()
    if not line or line.startswith("#"):
        continue
    p = line.split()
    if len(p) >= 10 and p[0].isdigit():
        centers.append([float(p[5]), float(p[6]), float(p[7])])
centers = np.array(centers)[:, order]   # rotated-cloud frame: x,y ground, z up

pcd2 = o3d.geometry.PointCloud()
pcd2.points = o3d.utility.Vector3dVector(pts3.astype(np.float64))
pcd2 = pcd2.voxel_down_sample(0.05)
pcd2, _ = pcd2.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
sor = np.asarray(pcd2.points)
work = o3d.geometry.PointCloud()
work.points = o3d.utility.Vector3dVector(sor)
for _ in range(2):
    if len(work.points) < 500:
        break
    model, inliers = work.segment_plane(0.02, 3, 200)
    a, b, c, d = model
    nn = math.sqrt(a * a + b * b + c * c)
    if abs(c) / nn > 0.85 and len(inliers) > 0.02 * len(sor):
        work = work.select_by_index(inliers, invert=True)
    else:
        break
band = np.asarray(work.points)
band = band[(band[:, 2] >= 0.15) & (band[:, 2] <= 1.20)]
print(f"\nband points replicated from fused cloud: {len(band)}")

# blocking comp cells -> cloud frame via the mirror
cell_pts = np.array([world_to_cloud(p) for p in comp_world[block_comp]])
tree_band = cKDTree(band[:, :2])
dist, _ = tree_band.query(cell_pts)
print(f"PROVENANCE blocking comp cells <-> fused cloud: "
      f"median={np.median(dist):.3f} m  p95={np.percentile(dist, 95):.3f} m  "
      f"max={dist.max():.3f} m  (cells={len(cell_pts)})")
# how many cloud points fall inside the comp's cells (direct cell match)?
crows = ((band[:, 1] - oy) / res).astype(int)          # writer convention rows
ccols = ((band[:, 0] - ox) / res).astype(int)
ok = (crows >= 0) & (crows < H) & (ccols >= 0) & (ccols < W)
cell_ids = np.zeros(H * W, np.int32)
cell_ids[comp_cells[block_comp][:, 0] * W + comp_cells[block_comp][:, 1]] = 1
in_cells = cell_ids[crows[ok] * W + ccols[ok]].sum()
print(f"cloud band points landing INSIDE blocking comp cells: {int(in_cells)} "
      f"of {len(band)} (cell area {block_area} x {res*res:.3f} m2)")

# ---------------- overlay on recorded RGB frames ----------------------------
os.makedirs(OUT, exist_ok=True)
cams_txt = f"{BASE}/colmap/sparse/txt/cameras.txt"
cam_line = None
for line in open(cams_txt):
    if not line.startswith("#"):
        cam_line = line.split(); break
fx, fy, cx, cy = map(float, cam_line[4:8])
img_metas = []
for line in open(POSES):
    line = line.strip()
    if not line or line.startswith("#"):
        continue
    p = line.split()
    if len(p) >= 10 and p[0].isdigit():
        img_metas.append((int(p[0]), list(map(float, p[1:8]))))

def q2r(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])

YMIR = 2.0 * oy + H * res

def map_to_colmap(p_map):
    """consumed map (x,y,zup) -> rotated-cloud (x,y ground, z up) -> raw COLMAP."""
    mx, my, mz = p_map
    rc = np.array([mx, YMIR - my, mz])       # cloud frame (mirror back)
    return np.array([rc[2], rc[0], rc[1]])   # invert order rotation

def cloud_to_map_xy(c):
    return np.array([c[0], YMIR - c[1]])

def project(img_meta, p_map, img_w, img_h):
    q = img_meta[0:4]; t = np.array(img_meta[4:7])
    X = q2r(q) @ map_to_colmap(p_map) + t
    if X[2] <= 0.05:
        return None
    u = fx * X[0] / X[2] + cx
    v = fy * X[1] / X[2] + cy
    if not (-img_w * 0.2 <= u <= img_w * 1.2 and -img_h * 0.2 <= v <= img_h * 1.2):
        return None
    return u, v

# cameras near the blocking region (consumer frame)
blk_xy = comp_world[block_comp].mean(axis=0)
cam_map = np.array([cloud_to_map_xy(c) for c in centers])
cam_d = np.hypot(cam_map[:, 0] - blk_xy[0], cam_map[:, 1] - blk_xy[1])
picks = sorted(range(len(cam_d)), key=lambda i: cam_d[i])[:2]

plan = None
try:
    pl = [l.split() for l in open(PLAN) if not l.startswith("#")]
    plan = np.array([[float(a), float(b)] for a, b in pl])
except Exception:
    pass

for k, i_img in enumerate(picks):
    img_id, meta8 = img_metas[i_img]
    fpath = f"{BASE}/img/frame_{img_id:04d}.jpg"
    img = cv2.imread(fpath)
    if img is None:
        print("missing frame", fpath); continue
    ih, iw = img.shape[:2]
    for p in traj_ds[::4]:
        uv = project(meta8, [p[0], p[1], 0.0], iw, ih)
        if uv:
            cv2.circle(img, (int(uv[0]), int(uv[1])), 3, (0, 255, 0), -1)
    if plan is not None:
        for p in plan[::6]:
            uv = project(meta8, [p[0], p[1], 0.0], iw, ih)
            if uv:
                cv2.circle(img, (int(uv[0]), int(uv[1])), 2, (0, 255, 255), -1)
    for p in comp_world[block_comp][::5]:
        uv = project(meta8, [p[0], p[1], 0.4], iw, ih)
        if uv:
            cv2.circle(img, (int(uv[0]), int(uv[1])), 2, (0, 0, 255), -1)
    for p in comp_world[bend_comp][::5]:
        uv = project(meta8, [p[0], p[1], 0.4], iw, ih)
        if uv:
            cv2.circle(img, (int(uv[0]), int(uv[1])), 2, (255, 0, 255), -1)
    uv_s = project(meta8, [SPAWN[0], SPAWN[1], 0.0], iw, ih)
    uv_g = project(meta8, [GOAL[0], GOAL[1], 0.0], iw, ih)
    if uv_s:
        cv2.drawMarker(img, (int(uv_s[0]), int(uv_s[1])), (255, 0, 255), cv2.MARKER_TILTED_CROSS, 18, 3)
    if uv_g:
        cv2.drawMarker(img, (int(uv_g[0]), int(uv_g[1])), (255, 0, 255), cv2.MARKER_STAR, 18, 3)
    for tt in np.linspace(0, 1, 200):
        uv = project(meta8, [SPAWN[0] + tt * d_vec[0], SPAWN[1] + tt * d_vec[1], 0.0], iw, ih)
        if uv:
            cv2.circle(img, (int(uv[0]), int(uv[1])), 1, (160, 160, 160), -1)
    cv2.imwrite(f"{OUT}/frame_overlay_{img_id:04d}.png", img)

    pad_px = 60
    rs = H - 1 - ((traj[:, 1] - oy) / res).astype(int)
    cs = ((traj[:, 0] - ox) / res).astype(int)
    r_lo, r_hi = max(0, rs.min() - pad_px), min(H, rs.max() + pad_px)
    c_lo, c_hi = max(0, cs.min() - pad_px), min(W, cs.max() + pad_px)
    g = pgm[r_lo:r_hi, c_lo:c_hi].copy()
    panel = np.zeros((*g.shape, 3), np.uint8)
    panel[g < 100] = (0, 0, 255)
    panel[g == 205] = (90, 90, 90)
    panel[g > 205] = (255, 255, 255)
    for x, y in traj_ds:
        c = int((x - ox) / res) - c_lo; r = H - 1 - int((y - oy) / res) - r_lo
        cv2.circle(panel, (c, r), 1, (0, 255, 0), -1)
    for x, y in comp_world[block_comp]:
        c = int((x - ox) / res) - c_lo; r = H - 1 - int((y - oy) / res) - r_lo
        panel[max(0,r-1):r+2, max(0,c-1):c+2] = (0, 0, 255)
    for x, y in comp_world[bend_comp]:
        c = int((x - ox) / res) - c_lo; r = H - 1 - int((y - oy) / res) - r_lo
        panel[max(0,r-1):r+2, max(0,c-1):c+2] = (255, 0, 255)
    for (X, Y) in (SPAWN, GOAL):
        c = int((X - ox) / res) - c_lo; r = H - 1 - int((Y - oy) / res) - r_lo
        cv2.drawMarker(panel, (c, r), (255, 0, 255), cv2.MARKER_CROSS, 14, 2)
    hgt = 480
    panel = cv2.resize(panel, (int(panel.shape[1] * hgt / panel.shape[0]), hgt))
    imgr = cv2.resize(img, (int(iw * hgt / ih), hgt))
    both = np.hstack([imgr, panel])
    cv2.putText(both, f"frame_{img_id:04d}.jpg (recorded) | map: green=executed red=blocker magenta=bend-obj",
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)
    cv2.imwrite(f"{OUT}/detour_composite_{img_id:04d}.png", both)

print("overlays written to", OUT)

# ---------------- verdict ---------------------------------------------------
gates = {
    "reached_goal": bool(np.hypot(*(traj[-1] - np.array(GOAL))) < 0.35),
    "length_ratio>=1.05": exec_len / straight >= 1.05,
    "max_dev>=0.5m": float(dev[i_max]) >= 0.5,
    "blocker_substantial(>=25 cells)": block_area >= 25,
    "path_avoids_blocker(d_path-d_line>=0.3)": (dpath_b - dline_b) >= 0.3,
    "offset_at_blocker>=0.5m": dev_min_in_span >= 0.5,
    "provenance_median<=0.15m": float(np.median(dist)) <= 0.15,
    "provenance_p95<=0.30m": float(np.percentile(dist, 95)) <= 0.30,
    "min_clearance_blocker>=0.20m": float(clear_blk.min()) >= 0.20,
}
print("\n=== VERDICT ===")
for k, v in gates.items():
    print(("PASS " if v else "FAIL ") + k)
ok = all(gates.values())
print("OVERALL:", "PASS" if ok else "FAIL")
json.dump({
    "exec_len": exec_len, "straight": straight, "ratio": exec_len / straight,
    "max_dev": float(dev[i_max]), "bend": bend.tolist(),
    "dev_at_blocker_m": dev_at_blk, "dev_min_in_blocker_span_m": dev_min_in_span,
    "blocker_t_span": list(t_span),
    "blocking_comp": int(block_comp), "blocking_area_cells": int(block_area),
    "blocking_bbox_m": [bx0, by0, bx1, by1],
    "d_line": dline_b, "d_path": dpath_b,
    "bend_comp": int(bend_comp), "bend_comp_area": int(stats[bend_comp, cv2.CC_STAT_AREA]),
    "bend_to_comp_m": float(bend_d),
    "provenance_median_m": float(np.median(dist)),
    "provenance_p95_m": float(np.percentile(dist, 95)),
    "provenance_max_m": float(dist.max()),
    "cloud_points_in_blocker_cells": int(in_cells),
    "min_clearance_blocker_m": float(clear_blk.min()),
    "min_clearance_any_m": float(clear_any.min()),
    "gates": gates, "verdict": "PASS" if ok else "FAIL",
}, open(f"{OUT}/detour_metrics.json", "w"), indent=2)
sys.exit(0 if ok else 1)
