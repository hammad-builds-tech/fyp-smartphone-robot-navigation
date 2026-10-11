#!/usr/bin/env python3
"""READ-ONLY analysis: which reconstructed obstacle components can honestly be
approached/crossed by straight navigation legs between walkable cells?

Reports, per component (area >= 30 cells):
  - area, world bbox
  - min distance to walkable space (approachability)
  - number of sampled 3-9 m straight lines between walkable cells that CROSS
    it with >= min(10, area) cells of penetration (deep crossing)
No map/pipeline changes.
"""
import sys
import math
import yaml
import numpy as np
import cv2
import glob

# GENERIC: default = the most recent reconstruction run (arg 1 overrides).
def _latest_run():
    runs = sorted(glob.glob("/home/hammad/FYP/realroom/runs/*/"), reverse=True)
    return runs[0].rstrip("/") if runs else "."

DS = sys.argv[1] if len(sys.argv) > 1 else _latest_run()
# pick this run's baked map (its name matches the dataset dir prefix)
_baked = sorted(glob.glob(f"{DS}/*_baked.yaml"))
MAPY = _baked[0] if _baked else ""

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

n_lbl, lbl, stats, _ = cv2.connectedComponentsWithStats(occ, connectivity=8)
occ_real = np.zeros_like(occ)
for i in range(1, n_lbl):
    if stats[i, cv2.CC_STAT_AREA] >= 4:
        occ_real[lbl == i] = 1

# walkable = carved free cells with >= 0.45 m clearance (same as demo oracle)
clear = cv2.distanceTransform((~occ_real.astype(bool)).astype(np.uint8),
                              cv2.DIST_L2, 5)
walkable = (free == 1) & (clear >= 9)
n2, lbl2 = cv2.connectedComponents(walkable.astype(np.uint8), connectivity=8)
sizes = sorted([(int((lbl2 == i).sum()), i) for i in range(1, n2)], reverse=True)
big = (lbl2 == sizes[0][1])
cand = np.argwhere(big)
print(f"walkable: {len(cand)} cells (largest comp {sizes[0][0]})")

def to_world(r, c):
    return ox + (c + 0.5) * res, oy + (H - 1 - r + 0.5) * res

def to_cell(x, y):
    c = int((x - ox) / res)
    r = H - 1 - int((y - oy) / res)
    return max(0, min(H - 1, r)), max(0, min(W - 1, c))

rng = np.random.default_rng(21)
cross_count = {i: 0 for i in range(1, n_lbl) if stats[i, cv2.CC_STAT_AREA] >= 30}
deep_pen = {i: 0 for i in cross_count}
idx = rng.choice(len(cand), size=min(len(cand), 20000), replace=False)
pts = cand[idx]
N_LINES = 60000
for k in range(N_LINES):
    a, b = pts[rng.integers(len(pts))], pts[rng.integers(len(pts))]
    d = np.hypot(*(a - b))
    if not (3.0 <= d <= 9.0):
        continue
    n = int(max(8, d / 0.05))
    ts = np.linspace(0, 1, n)
    xs = a[1] + ts * (b[1] - a[1])
    ys = a[0] + ts * (b[0] - a[0])
    lids = lbl[ys.astype(int), xs.astype(int)]
    for lid in np.unique(lids):
        lid = int(lid)
        if lid in cross_count:
            c = int((lids == lid).sum())
            cross_count[lid] += 1
            deep_pen[lid] = max(deep_pen[lid], c)

print(f"\n{'comp':>5} {'area':>7} {'crossable':>9} {'max_pen':>8}  "
      f"min-dist-to-walk  world-bbox")
rows = []
for i in sorted(cross_count, key=lambda k: -stats[k, cv2.CC_STAT_AREA]):
    area = int(stats[i, cv2.CC_STAT_AREA])
    m = lbl == i
    dt = cv2.distanceTransform((~m).astype(np.uint8), cv2.DIST_L2, 5)
    mind = float(dt[big > 0].min()) if cross_count[i] or True else -1
    x, y, w, h = stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP], \
        stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT]
    x0, y0 = to_world(y + h, x); x1, y1 = to_world(y, y and x + w)
    rows.append((i, area, cross_count[i], deep_pen[i], mind))
    print(f"{i:>5} {area:>7} {cross_count[i]:>9} {deep_pen[i]:>8}  "
          f"{mind * res:>7.2f} m     ({x0:.1f},{y0:.1f})-({x1:.1f},{y1:.1f})")
multi = [r for r in rows if r[2] >= 50]
print(f"\ncomponents with >=50 crossable lines: {len(multi)}")
print("substantial+crossable obstacles:", [(r[0], r[1]) for r in multi])
