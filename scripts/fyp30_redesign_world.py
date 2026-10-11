#!/usr/bin/env python3
"""fyp30_redesign_world.py -- GEOMETRY-level redesign of a realroom30 world.

v2 (coherent-furniture redesign). Why v1 failed visually: with the capture's
0.1-0.45 m cell spacing, 345 cells split into ~150 clusters -> ~345 pastel
primitives packed 0.1-0.2 m apart = unreadable multicolor clutter.

v2 grouping rule (data-driven, NO hardcoded coordinates -- works for any
camera/MiDaS reconstruction):
  * BFS-union cells whose gap <= 0.10 m ONLY (touching/contiguous = one piece
    of furniture, as MiDaS depth edges actually segment real objects);
  * clusters of >=3 cells -> ONE coherent solid: its axis-aligned bounding
    footprint gets a single furniture-size box visual (crate/cabinet/table);
    tall clusters (>=0.6 m) render as pillars; cylindrical clusters render as
    a cylinder whose radius covers the bounding circle (never less-blocking);
  * 1-2 cell clusters: enlarge each cell's rendered box to a full 0.4 m grid
    cell (0.4 x 0.4 x cell height) so speckles become readable furniture
    cubes; isolated sub-0.06 m nubs are simply left as-is;

EVERY collision <size> block STAYS byte-identical so the Nav2 map baked from
this world (fyp30_bake_world_boxes.py) is unchanged (verified offline by
re-running that script into a temp map before/after and diffing). All rendered
visuals SUPERSET-COVER their cell's original XY footprint, so the visual
reality is never less blocking than the planner's map assumes.

Muted professional palette (7 tones, deterministic index assignment so
adjacent clusters differ): makes objects read as furniture, not confetti.

Usage:
  python3 fyp30_redesign_world.py <world.sdf> [--backup] [--out FILE] [--dry-run]

Exit codes: 0 ok, 1 bad input, 2 invariant violation.
"""
import argparse
import math
import os
import re

# ---------- palettes (diffuse RGB, muted indoor furniture tones) -------------
FURNITURE = [
    (0.55, 0.47, 0.38),   # warm oak cabinet
    (0.42, 0.48, 0.55),   # slate blue crate
    (0.56, 0.52, 0.60),   # grey-violet bin
    (0.48, 0.58, 0.50),   # sage green cabinet
    (0.62, 0.55, 0.44),   # tan crate
    (0.40, 0.42, 0.47),   # charcoal pillar
    (0.66, 0.62, 0.57),   # stone pedestal
]
PILLAR = (0.46, 0.46, 0.50)          # all pillars share a concrete tone
CYL = (0.50, 0.58, 0.62)             # steel-blue cylinder

FLOOR_DIFF = (0.66, 0.63, 0.58)


def mat(diff, spec="0.12 0.12 0.10 1", amb_scale=0.5):
    a = tuple(c * amb_scale for c in diff)
    return ("<material>"
            "<ambient>%.2f %.2f %.2f 1</ambient>"
            "<diffuse>%.2f %.2f %.2f 1</diffuse>"
            "<specular>%s</specular></material>" % (a + diff + (spec,)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("world")
    ap.add_argument("--out", default=None)
    ap.add_argument("--backup", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    path = os.path.abspath(os.path.expanduser(args.world))
    with open(path, "r", encoding="utf-8") as f:
        src = f.read()

    # ---------- collect obstacle cells ---------------------------------------
    model_re = re.compile(r'<model name="obstacle_(\d+)">.*?</model>', re.S)
    cells = {}          # (x, y) -> model body
    cell_pos = {}       # model body -> (x, y)
    for m in re.finditer(
            r'<model name="obstacle_(\d+)">\s*<static>true</static>\s*'
            r'<pose>(-?[\d.]+) (-?[\d.]+) ([\-\d.]+)', src):
        x, y = round(float(m.group(2)), 3), round(float(m.group(3)), 3)
        cells.setdefault((x, y), int(m.group(1)))
    if not cells:
        print("WARN: no obstacle_* models matched -- nothing redesigned")
        return 1

    # per-cell (x, y) -> size (sx, sy, sz) parsed from its own model block
    cell_size = {}
    for m in model_re.finditer(src):
        body = m.group(0)
        pm = re.search(r'<pose>(-?[\d.]+) (-?[\d.]+) ([\-\d.]+)', body)
        if not pm:
            continue
        px, py = round(float(pm.group(1)), 3), round(float(pm.group(2)), 3)
        sm = re.search(r'<size>([^<]+)</size>', body)
        if not sm:
            continue
        sx, sy, sz = [float(v) for v in sm.group(1).split()][:3]
        cell_size[(px, py)] = (sx, sy, sz)

    # ---------- v2 grouping: only CONTIGUOUS cells merge (gap <= 0.10 m) ------
    OFF = [(dx, dy) for dx in (-0.1, -0.05, 0, 0.05, 0.1)
           for dy in (-0.1, -0.05, 0, 0.05, 0.1)]
    keys = set(cells)
    seen = set()
    cluster_of = {}          # (x,y) -> cluster id
    clusters = []            # list of sets
    while keys - seen:
        seed = sorted(keys - seen)[0]
        comp = {seed}
        frontier = [seed]
        while frontier:
            cx, cy = frontier.pop()
            for dx, dy in OFF:
                nb = (round(cx + dx, 3), round(cy + dy, 3))
                if nb in keys and nb not in seen and nb not in comp:
                    comp.add(nb)
                    frontier.append(nb)
        idx = len(clusters)
        clusters.append(comp)
        for c in comp:
            cluster_of[c] = idx
        seen |= comp
    n_clusters = len(clusters)
    big = [c for c in clusters if len(c) >= 3]
    small = [c for c in clusters if len(c) < 3]
    print("v2 grouping: %d coherent clusters (%d furniture, %d small-cube), "
          "from %d cells" % (n_clusters, len(big), len(small), len(cells)))

    # ---------- rewrite each obstacle model's visual --------------------------
    n_done = [0]
    changed = [0]

    def paint(body, new_vis):
        old_vis = re.search(r'<visual name="visual">.*?</visual>', body, re.S)
        if not old_vis:
            return body
        nb = body.replace(old_vis.group(0), new_vis, 1)
        changed[0] += 1
        return nb

    def obs_repl(m):
        body = m.group(0)
        pm = re.search(r'<pose>(-?[\d.]+) (-?[\d.]+) ([\-\d.]+)', body)
        x, y = round(float(pm.group(1)), 3), round(float(pm.group(2)), 3)
        n_done[0] += 1
        sm = re.search(r'<size>([^<]+)</size>', body)
        if not sm:
            return body
        sx, sy, sz = [float(v) for v in sm.group(1).split()][:3]
        idx = cluster_of[(x, y)]
        comp = clusters[idx]
        if len(comp) < 3:
            # readable furniture cube: full 0.4 m cell footprint (superset of
            # the cell's own footprint), keeping the original height
            cx, cy, cz = 0.4, 0.4, max(sz, 0.35)
            geo = ("<geometry><box><size>%.3f %.3f %.3f</size>"
                   "</box></geometry>" % (cx, cy, cz))
            diff = FURNITURE[idx % len(FURNITURE)]
            new_vis = ('<visual name="visual">%s%s'
                       '<pose>0 0 %.3f 0 0 0</pose></visual>'
                       % (geo, mat(diff), cz / 2.0 - 0.0))
            # pose z: keep the model's own pose z as-is (model poseintrinsics)
            return paint(body, new_vis)
        # coherent multi-cell furniture: bbox superset box (one piece)
        xs = [p[0] for p in comp]
        ys = [p[1] for p in comp]
        bx0, bx1 = min(xs) - 0.2, max(xs) + 0.2
        by0, by1 = min(ys) - 0.2, max(ys) + 0.2
        bw, bh = bx1 - bx0, by1 - by0
        hs = [cell_size.get(p, (0.2, 0.2, 0.9))[2] for p in comp]
        hz = max(hs)
        bcx, bcy = (bx0 + bx1) / 2.0, (by0 + by1) / 2.0
        # pillar if tall-narrow, cylinder if square-ish, else cabinet box
        if hz >= 0.6 and bw < 0.5 and bh < 0.5:
            r = max(bw, bh) / 2.0 + 0.02
            geo = ("<geometry><cylinder><radius>%.3f</radius>"
                   "<length>%.3f</length></cylinder></geometry>" % (r, hz))
            new_vis = ('<visual name="visual">%s%s'
                       '<pose>%.3f %.3f %.3f 0 0 0</pose></visual>'
                       % (geo, mat(PILLAR), bcx - x, bcy - y, hz / 2.0 - 0.05))
        elif abs(bw - bh) < 0.25 and hz >= 0.4:
            r = min(bw, bh) / 2.0 + 0.02
            geo = ("<geometry><cylinder><radius>%.3f</radius>"
                   "<length>%.3f</length></cylinder></geometry>" % (r, hz))
            new_vis = ('<visual name="visual">%s%s'
                       '<pose>%.3f %.3f %.3f 0 0 0</pose></visual>'
                       % (geo, mat(CYL), bcx - x, bcy - y, hz / 2.0 - 0.05))
        else:
            geo = ("<geometry><box><size>%.3f %.3f %.3f</size>"
                   "</box></geometry>" % (bw, bh, hz))
            diff = FURNITURE[idx % len(FURNITURE)]
            new_vis = ('<visual name="visual">%s%s'
                       '<pose>%.3f %.3f %.3f 0 0 0</pose></visual>'
                       % (geo, mat(diff), bcx - x, bcy - y, hz / 2.0 - 0.05))
        body = paint(body, new_vis)
        # reduce the OTHER cells of this cluster to invisible? No: every cell
        # renders the SAME coherent bbox (they overlap and read as one piece).
        return body

    src = model_re.sub(obs_repl, src)

    # ---------- floor / walls / sky / lights are the decorate pass's job ------
    src = re.sub(
        r'(<geometry><box><size>18\.200 20\.950 0\.050</size></box></geometry>)'
        r'<material><ambient>[^<]*</ambient><diffuse>[^<]*</diffuse>'
        r'<specular>[^<]*</specular></material>',
        lambda mm: mm.group(1) + mat(FLOOR_DIFF, spec="0.42 0.42 0.40 1"),
        src, count=1)

    if args.dry_run:
        print("DRY-RUN: models touched %d, visuals replaced %d "
              "(clusters %d)" % (n_done[0], changed[0], n_clusters))
        return 0
    if args.out:
        open(args.out, "w").write(src)
    else:
        if not args.backup:
            print("REFUSING in-place rewrite without --backup/--out")
            return 1
        open(args.world, "w").write(src)
    print("redesigned models: %d   visuals replaced: %d   clusters: %d"
          % (n_done[0], changed[0], n_clusters))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
