#!/usr/bin/env python3
"""Multi-goal continuous navigation demo (single recorded-video reconstruction).

Uses the SAME walkable-space oracle conventions as fyp30_pick_goal.py
(free cells near the capture camera path, >= 0.45 m clearance, components
>= 4 cells are real geometry). Chains SPAWN -> G1 -> G2 -> G3 where every leg
is 4-9 m and its straight line crosses real captured geometry (area >= 40
cells preferred); blocker components are deduplicated across legs when the
data allows. Each leg is navigated by the proven fyp30_nav_driver.py and
verified against the executed trajectory (per-leg detour check).

Run: /usr/bin/python3 -u fyp30_multi_goal_demo.py <dataset_dir> <n_legs>
Requires: source /opt/ros/lyrical/setup.bash + ws, ROS_DOMAIN_ID=0.
"""
import json
import math
import os
import subprocess
import sys
import time

import cv2
import numpy as np
import yaml

import rclpy
from nav_msgs.msg import Odometry

# --- pre-flight: /odom must be FLOWING before each leg -----------------------
# A freshly spawned driver node can lag on DDS discovery; without this check
# its trajectory evidence may starve even while navigation itself succeeds.
rclpy.init()
_pf = rclpy.create_node('odom_preflight')
_pf_count = [0]
_pf.create_subscription(Odometry, '/odom', lambda m: _pf_count.__setitem__(0, _pf_count[0] + 1), 10)

def wait_odom_flow(min_msgs=5, budget=60.0):
    _pf_count[0] = 0
    t0 = time.time()
    while _pf_count[0] < min_msgs and time.time() - t0 < budget:
        rclpy.spin_once(_pf, timeout_sec=0.5)
    return _pf_count[0]

DS = sys.argv[1] if len(sys.argv) > 1 else ""
N_LEGS = int(sys.argv[2]) if len(sys.argv) > 2 else 3
if not DS or not os.path.isdir(DS):
    sys.exit("usage: fyp30_multi_goal_demo.py <dataset_dir> [n_legs]")
DS = os.path.realpath(DS)
DRIVER = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "fyp30_nav_driver.py")

# --- targets from the dataset's own auto-selection ---------------------------
SPAWN = GOAL0 = None
for line in open(f"{DS}/nav_targets.env"):
    if line.startswith("SPAWN_X="):
        SPAWN = [float(line.split("=")[1]), None]
    elif line.startswith("SPAWN_Y="):
        SPAWN[1] = float(line.split("=")[1])
    elif line.startswith("GOAL_X="):
        GOAL0 = [float(line.split("=")[1]), None]
    elif line.startswith("GOAL_Y="):
        GOAL0[1] = float(line.split("=")[1])
assert SPAWN[1] is not None and GOAL0[1] is not None

# the pipeline records the baked (world-consistent) map in nav_targets.env
MAPY = None
_nt = f"{DS}/nav_targets.env"
if os.path.exists(_nt):
    for line in open(_nt):
        if line.startswith("FYP_MAP="):
            MAPY = line.split("=", 1)[1].strip()
            if MAPY and not os.path.isabs(MAPY):
                MAPY = os.path.join(DS, MAPY)
            break
if not MAPY or not os.path.exists(MAPY):
    MAPY = f"{DS}/realroom_{os.path.basename(DS)}.yaml"
assert os.path.exists(MAPY), "dataset map yaml not found (run fyp30_run_pipeline.sh first)"
print(f"demo oracle map: {MAPY}")

meta = yaml.safe_load(open(MAPY))
res = float(meta["resolution"]); ox, oy = map(float, meta["origin"][:2])
with open(MAPY.replace(".yaml", ".pgm"), "rb") as f:
    assert f.readline().strip() == b"P5"
    line = f.readline()
    while line.startswith(b"#"):
        line = f.readline()
    W, H = map(int, line.split()); f.readline()
    pgm = np.frombuffer(f.read(), dtype=np.uint8).reshape(H, W)
occ = (pgm < 100).astype(np.uint8); free = (pgm == 205)

# --- exact pick_goal conventions (writer row orientation) --------------------
n_lbl, lbl, stats, _ = cv2.connectedComponentsWithStats(occ, connectivity=8)
occ_real = np.zeros_like(occ)
for i in range(1, n_lbl):
    if stats[i, cv2.CC_STAT_AREA] >= 4:
        occ_real[lbl == i] = 1

cams = []
for line in open(f"{DS}/colmap/sparse/txt/images.txt"):
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
clear = cv2.distanceTransform((~occ_real.astype(bool)).astype(np.uint8),
                              cv2.DIST_L2, 5)
# Goal must hold the robot's real half-width (0.25 m chassis half-width plus
# 0.20 m margin) from real captured geometry — matches the nav2 footprint
# (inscribed 0.25 m) + 0.45 m inflation planning model.
walkable = (free == 1) & (dt_cam <= 80) & (clear >= 9)
n2, lbl2 = cv2.connectedComponents(walkable.astype(np.uint8), connectivity=8)
sizes = sorted([(int((lbl2 == i).sum()), i) for i in range(1, n2)], reverse=True)
big_comp = (lbl2 == sizes[0][1])
cand = np.argwhere(big_comp)
print(f"walkable candidates: {len(cand)} cells in comp of {sizes[0][0]}")

def to_world(r, c):
    return ox + (c + 0.5) * res, oy + (H - 1 - r + 0.5) * res

def to_cell(x, y):
    # inverse of to_world (writer orientation for line sampling)
    c = int((x - ox) / res)
    r = H - 1 - int((y - oy) / res)
    return max(0, min(H - 1, r)), max(0, min(W - 1, c))

def leg_blocker_hits(p_from, p_to):
    """Sample the straight leg; return {component_id: cells_crossed}."""
    (xa, ya), (xb, yb) = p_from, p_to
    n = int(max(8, math.hypot(xb - xa, yb - ya) / 0.05))
    hits = {}
    for t in np.linspace(0, 1, n):
        r, c = to_cell(xa + t * (xb - xa), ya + t * (yb - ya))
        lid = int(lbl[r, c])
        if lid > 0 and stats[lid, cv2.CC_STAT_AREA] >= 4:
            hits[lid] = hits.get(lid, 0) + 1
    return hits

MIN_AREA = 30   # smallest real component counted as a leg blocker

def pick_next(p_from, used_blockers, prev_goals, rng):
    """Pick a leg endpoint 4-9 m away whose straight line crosses real geometry.

    Prefers legs crossing a component not yet used as a blocker; falls back to
    re-using one (a real detour from a different approach is still a real
    detour). Goal-ring constraint relaxes in stages so the chain never dies
    while honest candidates exist.
    """
    pf = np.array(p_from)
    for ring in (2.5, 1.0, 0.0):
        idx = rng.choice(len(cand), size=min(len(cand), 12000), replace=False)
        best_fresh = best_any = None
        for j in idx:
            r, c = cand[j]
            pw = np.array(to_world(r, c))
            d = float(np.hypot(*(pw - pf)))
            if not (4.0 <= d <= 9.0):
                continue
            if ring and any(np.hypot(*(pw - np.array(g))) < ring
                            for g in prev_goals + [SPAWN]):
                continue
            hits = leg_blocker_hits(p_from, tuple(pw))
            fresh = {k: v for k, v in hits.items()
                     if int(stats[k, cv2.CC_STAT_AREA]) >= MIN_AREA
                     and k not in used_blockers}
            pool = fresh or {k: v for k, v in hits.items()
                             if int(stats[k, cv2.CC_STAT_AREA]) >= MIN_AREA}
            if not pool:
                continue
            lid = max(pool, key=lambda k: (int(stats[k, cv2.CC_STAT_AREA]), pool[k]))
            area = int(stats[lid, cv2.CC_STAT_AREA])
            score = min(area, 800) / 400.0 + min(pool[lid], 20) * 0.15 \
                + 0.25 * float(clear[r, c]) - 0.10 * abs(d - 6.5)
            cand_t = (score, tuple(pw), lid, area, d)
            if best_any is None or score > best_any[0]:
                best_any = cand_t
            if fresh and (best_fresh is None or score > best_fresh[0]):
                best_fresh = cand_t
        pick = best_fresh or best_any
        if pick is not None:
            return pick
    return None

rng = np.random.default_rng(7)
used_blockers, goals, results = [], [], []
t_all = time.time()

# --- return the robot to the spawn pose first (legs are designed from SPAWN;
# a robot left at a previous goal would silently invalidate leg-1 analysis) ---
cur = None
_home_env = dict(os.environ)
_home_env["FYP_TRAJ_LOG"] = f"{DS}/traj_home.log"
open(f"{DS}/traj_home.log", "w").close()
r = subprocess.run(
    ["/usr/bin/python3", "-u", DRIVER,
     "navigate", f"{SPAWN[0]:.2f}", f"{SPAWN[1]:.2f}"],        env=_home_env, capture_output=True, text=True, timeout=1600)
print("return-to-spawn:", "REACHED" if r.returncode == 0 else
      ("/FAILED or already there" if "REACHED" not in r.stdout else "?"))
if "REACHED" in r.stdout or "already" in r.stdout:
    cur = tuple(SPAWN)
else:
    # fall back: read where the robot actually is from the home trajectory
    _rows = [l.split() for l in open(f"{DS}/traj_home.log")
             if not l.startswith("#") and len(l.split()) >= 3]
    if _rows:
        _last = _rows[-1]
        cur = (float(_last[1]), float(_last[2]))   # odom is world-frame now
        print(f"  start pose (from odom): ({cur[0]:.2f},{cur[1]:.2f})")
    else:
        cur = tuple(SPAWN)
if math.hypot(cur[0] - SPAWN[0], cur[1] - SPAWN[1]) > 0.5:
    print(f"WARNING: starting legs from ({cur[0]:.2f},{cur[1]:.2f}), not spawn")
for leg in range(1, N_LEGS + 1):
    pick = pick_next(cur, used_blockers, goals, rng)
    if pick is None:
        print(f"LEG {leg}: NO valid goal found — stopping chain honestly")
        break
    score, goal, blocker, barea, dist = pick
    used_blockers.append(blocker)
    goals.append(goal)
    print(f"\n=== LEG {leg}: ({cur[0]:.2f},{cur[1]:.2f}) -> "
          f"({goal[0]:.2f},{goal[1]:.2f}) [{dist:.2f} m] | blocker comp {blocker} "
          f"({barea} cells) ===")
    got = wait_odom_flow()
    print(f"  pre-flight: /odom flowing ({got} msgs)")
    traj_leg = f"{DS}/traj_leg{leg}.log"          # fresh per-leg evidence file
    open(traj_leg, "w").close()
    env = dict(os.environ)
    env["FYP_TRAJ_LOG"] = traj_leg
    r = subprocess.run(
        ["/usr/bin/python3", "-u", DRIVER,
         "navigate", f"{goal[0]:.2f}", f"{goal[1]:.2f}"],
        env=env, capture_output=True, text=True, timeout=1600)
    tail = [l for l in r.stdout.splitlines() if "NAV_RESULT" in l or "REACHED" in l
            or "OVERALL" in l or "FAILED" in l or "aborted" in l.lower()]
    print("  driver:", " | ".join(tail[-3:]) if tail else r.stdout[-200:])
    if r.returncode != 0:
        print(f"LEG {leg} FAILED (rc={r.returncode})"); break
    cur = goal

    # --- per-leg verification: EVERY substantial component the line threatens
    segs, pts = [], []
    for line in open(traj_leg):
        if line.startswith("# --- run start"):
            if pts:
                segs.append(pts)
            pts = []
            continue
        p = line.split()
        if len(p) >= 3 and not line.startswith("#"):
            # odom is world-frame since the OdometryPublisher swap:
            # map->odom is identity, so no SPAWN offset is added here
            pts.append([float(p[1]), float(p[2])])
    if pts:
        segs.append(pts)
    leg_pts = np.array(segs[-1])
    keep = [leg_pts[0]]
    for p in leg_pts[1:]:
        if np.hypot(*(p - keep[-1])) >= 0.05:
            keep.append(p)
    # trim stationary lead-in/out so 'executed' measures actual driving
    i0, i1 = 0, len(keep) - 1
    while i0 < i1 and np.hypot(*(keep[i0 + 1] - keep[i0])) < 0.02:
        i0 += 1
    while i1 > i0 and np.hypot(*(keep[i1] - keep[i1 - 1])) < 0.02:
        i1 -= 1
    lp = np.array(keep[i0:i1 + 1])
    a = np.array([SPAWN[0] if leg == 1 else goals[-2][0],
                  SPAWN[1] if leg == 1 else goals[-2][1]])
    b = np.array(goal)
    straight = float(np.hypot(*(b - a)))
    exec_len = float(np.hypot(*np.diff(lp, axis=0).T).sum())
    line_pts = a + np.linspace(0, 1, 300)[:, None] * (b - a)
    threatened = []
    for i in range(1, n_lbl):
        area_i = int(stats[i, cv2.CC_STAT_AREA])
        if area_i < 25:
            continue
        wi = np.array([to_world(r_, c_) for r_, c_ in np.argwhere(lbl == i)])
        dl = float(np.hypot(
            line_pts[:, 0][:, None] - wi[None, :, 0],
            line_pts[:, 1][:, None] - wi[None, :, 1]).min())
        if dl <= 0.35:
            dp = float(np.hypot(
                lp[:, 0][:, None] - wi[None, :, 0],
                lp[:, 1][:, None] - wi[None, :, 1]).min())
            threatened.append((i, area_i, dl, dp))
    ok = exec_len / straight >= 1.03 and len(threatened) >= 1 \
        and all(dp >= 0.45 for (_, _, _, dp) in threatened)
    blk_str = ", ".join(f"comp {i}({ar}c): line {dl:.2f}->path {dp:.2f} m"
                        for i, ar, dl, dp in threatened)
    results.append(dict(leg=leg, From=list(a), to=list(b), straight=round(straight, 2),
                        executed=round(exec_len, 2), ratio=round(exec_len / straight, 3),
                        threatened=[dict(comp=int(i), area=int(ar), d_line=round(dl, 2),
                                         d_path=round(dp, 2))
                                    for i, ar, dl, dp in threatened], ok=bool(ok)))
    print(f"  executed {exec_len:.2f} m vs straight {straight:.2f} m "
          f"(ratio {exec_len / straight:.2f}); obstacles on the line: "
          f"{blk_str if threatened else 'none'} -> "
          f"{'DETOUR OK' if ok else 'WEAK'}")

print(f"\ntotal demo time: {time.time() - t_all:.0f} s")
# verdict + evidence file must be produced BEFORE the forced exit
all_ok = len(results) == N_LEGS and all(r["ok"] for r in results)
print("MULTI-GOAL DEMO:", "PASS" if all_ok else "PARTIAL/FAIL")
os.makedirs(f"{DS}/evidence", exist_ok=True)
json.dump(results, open(f"{DS}/evidence/multi_goal_results.json", "w"), indent=2)
sys.stdout.flush()
# force exit: rclpy teardown can hang on live DDS participants and would
# leave this toolling process dangling after the verdict is already printed
os._exit(0 if all_ok else 1)
