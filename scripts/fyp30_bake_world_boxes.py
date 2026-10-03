#!/usr/bin/env python3
"""Bake the world SDF's real obstacle boxes into the occupancy map pgm.

The raw occupancy map is a sparse sketch of the reconstruction (lethal cells
exist only where capture rays stopped), while the Gazebo world built from the
same reconstruction renders solid obstacle boxes. Nav2 can therefore plan
through map gaps that the physical world blocks. This script rasterizes EVERY
obstacle_* and wall_* box from the world SDF onto the map grid (a cell is
lethal if its center lies inside the box grown by --grow), preserving all
existing lethal cells, and writes <out-base>.pgm/.yaml. Pure reconstruction
data - no synthetic geometry is added.

pgm row convention (verified live against the running map_server): row 0 =
TOP = max y.

Usage:
  python3 fyp30_bake_world_boxes.py --world WORLD.sdf --map-yaml MAP.yaml \
      [--out-base PREFIX] [--grow 0.25]

--out-base defaults to <map-yaml-dir>/<map-yaml-basename>_baked.
Exit codes: 0 ok, 1 bad inputs (missing/empty world boxes, unreadable map).
"""
import argparse
import math
import os
import re
import sys

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    ap.add_argument("--world", required=True,
                    help="Gazebo world SDF generated from the reconstruction")
    ap.add_argument("--map-yaml", required=True,
                    help="occupancy map .yaml (matching .pgm must sit beside it)")
    ap.add_argument("--out-base",
                    help="output base path; default <map-yaml>_baked")
    ap.add_argument("--grow", type=float, default=0.25,
                    help="inflation around each box in metres "
                         "(default 0.25 = half robot footprint)")
    args = ap.parse_args()

    out_base = args.out_base or f"{args.map_yaml[:-5]}_baked"
    if not os.path.isfile(args.world):
        sys.exit(f"FAIL: world SDF not found: {args.world}")
    if not os.path.isfile(args.map_yaml):
        sys.exit(f"FAIL: map yaml not found: {args.map_yaml}")

    ytxt = open(args.map_yaml).read()
    m = re.search(r"resolution:\s*([\d.]+)", ytxt)
    o = re.search(r"origin:\s*\[([^\]]+)\]", ytxt)
    if not m or not o:
        sys.exit("FAIL: map yaml missing resolution/origin")
    res = float(m.group(1))
    ox, oy = [float(v) for v in o.group(1).split(",")[:2]]

    pgm_path = args.map_yaml.replace(".yaml", ".pgm")
    img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        sys.exit(f"FAIL: cannot read map pgm: {pgm_path}")
    H, W = img.shape
    grid = img.copy()

    sdf = open(args.world).read()
    blocks = re.findall(
        r'<model name="((?:obstacle|wall)_\w+)">.*?<pose>([^<]+)</pose>.*?'
        r'<size>([^<]+)</size>.*?</model>', sdf, re.S)
    if not blocks:
        sys.exit("FAIL: no obstacle_*/wall_* boxes parsed from the world SDF")

    boxes = []
    for name, pose, size in blocks:
        p = [float(v) for v in pose.split()]
        s = [float(v) for v in size.split()]
        if len(p) < 6 or len(s) < 3:
            continue
        boxes.append((name, p[0], p[1], p[5], s[0], s[1]))
    if not boxes:
        sys.exit("FAIL: world SDF boxes had malformed pose/size")
    print(f"map {W}x{H} res {res} origin ({ox},{oy}); parsed {len(boxes)} boxes")

    def to_row(wy):
        """world y -> image row, row0=TOP convention."""
        return H - 1 - int(round((wy - oy) / res))

    lethal = 0
    for name, cx, cy, yaw, sx, sy in boxes:
        hs, hl = sx / 2.0 + args.grow, sy / 2.0 + args.grow
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
    print(f"baked {lethal} new lethal cells (grow {args.grow} m, {len(boxes)} boxes)")

    out_pgm = f"{out_base}.pgm"
    out_yaml = f"{out_base}.yaml"
    cv2.imwrite(out_pgm, grid)
    open(out_yaml, "w").write(
        f"image: {out_pgm}\nresolution: {res}\norigin: [{ox}, {oy}, 0.0]\n"
        f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n")
    print(f"wrote {out_pgm} + {out_yaml}")


if __name__ == "__main__":
    main()
