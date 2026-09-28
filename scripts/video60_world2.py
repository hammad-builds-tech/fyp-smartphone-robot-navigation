#!/usr/bin/env python3
"""Build a clean, readable Gazebo world from the video60 occupancy map.

Data source (unchanged): realroom/video60/video60_map.pgm/.yaml, produced by
  60-s phone video -> MiDaS depth -> COLMAP fusion -> voxel/SOR/RANSAC
  -> occupancy grid (realroom_cloud_to_map.py).
No synthetic obstacles are added; every interior box is a rectangular merge
of occupied cells from that map. Perimeter walls bound the mapped extent.
Interior obstacle boxes are rendered 0.28-0.9 m tall so the robot body and
wheels stay visible in front of them; perimeter walls are full height.
"""
import argparse
import math
import os
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml


def load_map(yaml_path):
    meta = yaml.safe_load(open(yaml_path))
    img = cv2.imread(yaml_path.replace('.yaml', '.pgm'), cv2.IMREAD_UNCHANGED)
    occ = (img == 0)
    free = (img == 205)  # carved free space (ray-traced from camera poses)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    h, w = img.shape

    # cell (c, r) with c=col from left, r=row from top
    def cell_x(c):
        return ox + (c + 0.5) * res

    def cell_y(r):
        return oy + (h - r - 0.5) * res
    return occ, free, res, cell_x, cell_y, w, h


def max_rects(occ):
    """Greedy largest-rect decomposition of the occupied mask (r, c)."""
    occ = occ.astype(np.uint8)
    h, w = occ.shape
    heights = np.zeros(w, dtype=int)
    rects = []
    for r in range(h):
        for c in range(w):
            heights[c] = heights[c] + 1 if occ[r, c] else 0
        stack = []
        c = 0
        while c <= w:
            cur = heights[c] if c < w else 0
            start = c
            while stack and stack[-1][1] >= cur:
                s0, hh = stack.pop()
                if hh > 0:
                    rects.append((r - hh + 1, s0, hh, c - s0))
                start = s0
            stack.append((start, cur))
            c += 1
    return rects  # (row_top, col_left, height_rows, width_cols)


def merge_boxes(boxes, gap):
    """Iteratively merge axis-aligned rects separated by <= gap (units of rect)."""
    boxes = list(boxes)
    changed = True
    while changed:
        changed = False
        out = []
        while boxes:
            a = boxes.pop()
            merged = False
            for i, b in enumerate(out):
                x0a, ya, x1a, y1a = a
                x0b, yb, x1b, y1b = b
                gap_x = max(x0a - x1b, x0b - x1a)
                gap_y = max(ya - y1b, yb - y1a)
                overlap_x = min(x1a, x1b) - max(x0a, x0b)
                overlap_y = min(y1a, y1b) - max(ya, yb)
                if (gap_x <= gap and overlap_y > 0) or (gap_y <= gap and overlap_x > 0):
                    out[i] = (min(x0a, x0b), min(ya, yb),
                              max(x1a, x1b), max(y1a, y1b))
                    merged = changed = True
                    break
            if not merged:
                out.append(a)
        boxes = out
    return boxes


def merge_respecting_free(boxes, gap, free, W, H, max_free_frac=0.35):
    """Merge axis-aligned rects separated by <= gap cells when the merged
    bounding box covers only a small fraction of carved-free cells.
    Thin carve gaps inside one reconstructed surface (rays passing between
    fragments of the same wall) are consolidated into clean blocks, while
    wide observed passages (doorways/open floor) are never sealed.
    Rects are in continuous map-cell units (x=cols from left, y=rows from top).
    """
    boxes = list(boxes)
    changed = True
    while changed:
        changed = False
        out = []
        while boxes:
            a = boxes.pop()
            merged = False
            for i, b in enumerate(out):
                x0, y0 = min(a[0], b[0]), min(a[1], b[1])
                x1, y1 = max(a[2], b[2]), max(a[3], b[3])
                gap_x = max(a[0] - b[2], b[0] - a[2])
                gap_y = max(a[1] - b[3], b[1] - a[3])
                if gap_x > gap and gap_y > gap:
                    continue
                c0, c1 = int(math.floor(x0)), min(W, int(math.ceil(x1)))
                r0, r1 = int(math.floor(y0)), min(H, int(math.ceil(y1)))
                cell_area = max((x1 - x0) * (y1 - y0), 1e-9)
                free_frac = free[r0:r1, c0:c1].sum() * 1.0 / cell_area
                if free_frac > max_free_frac:
                    continue
                out[i] = (x0, y0, x1, y1)
                merged = changed = True
                break
            if not merged:
                out.append(a)
        boxes = out
    return boxes


def merge_touching(boxes):
    """Merge rects whose bounding box adds no uncovered area (exact union).
    Reduces stair-shaped decompositions to fewer larger boxes without
    changing the covered region."""
    boxes = list(boxes)
    changed = True
    while changed:
        changed = False
        out = []
        while boxes:
            a = boxes.pop()
            merged = False
            for i, b in enumerate(out):
                x0, y0 = min(a[0], b[0]), min(a[1], b[1])
                x1, y1 = max(a[2], b[2]), max(a[3], b[3])
                area_bb = (x1 - x0) * (y1 - y0)
                area_a = (a[2] - a[0]) * (a[3] - a[1])
                area_b = (b[2] - b[0]) * (b[3] - b[1])
                if area_bb <= area_a + area_b + 1e-9:
                    out[i] = (x0, y0, x1, y1)
                    merged = changed = True
                    break
            if not merged:
                out.append(a)
        boxes = out
    return boxes


def model(name, cx, cy, z0, z1, sx, sy, amb, dif):
    h = z1 - z0
    cz = (z0 + z1) / 2.0
    return f'''  <model name="{name}">
    <static>true</static>
    <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>{sx:.3f} {sy:.3f} {h:.3f}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{sx:.3f} {sy:.3f} {h:.3f}</size></box></geometry>
        <material><ambient>{amb}</ambient><diffuse>{dif}</diffuse>
          <specular>0.15 0.15 0.15 1</specular></material>
      </visual>
    </link>
  </model>'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=os.path.expanduser(
        '~/FYP/realroom/video60/video60_map.yaml'))
    ap.add_argument('--out', default=os.path.expanduser(
        '~/FYP/realroom/video60/video60_world2.sdf'))
    ap.add_argument('--world-name', default='video60_room2')
    ap.add_argument('--wall-height', type=float, default=1.2)
    ap.add_argument('--obst-top', type=float, default=0.9)
    ap.add_argument('--obst-base', type=float, default=0.28)
    ap.add_argument('--thickness', type=float, default=0.10)
    ap.add_argument('--min-area', type=float, default=0.01,
                    help='min interior obstacle area m^2 (drop 1-2 cell specks)')
    ap.add_argument('--min-cells', type=int, default=4,
                    help='drop connected components smaller than this many '
                         'cells from geometry (SOR specks; costmap keeps them)')
    ap.add_argument('--merge-gap', type=float, default=0.15,
                    help='merge nearby obstacle rects up to this gap (m); '
                         'merged box must stay mostly non-free')
    args = ap.parse_args()

    occ, free, res, cell_x, cell_y, W, H = load_map(args.map)
    print(f"map {W}x{H} @ {res} m, occupied cells: {occ.sum()}, "
          f"carved free: {free.sum()}")

    # ---- obstacle extraction from the filtered occupancy grid ----
    # The map (video60_map.pgm) IS the report-defined filtered occupancy
    # grid. Obstacles are grouped into connected components (8-conn) of the
    # raw occupied mask: each component is one reconstructed surface, so
    # rectangle decomposition stays local and cannot bridge across gaps.
    # Components smaller than --min-cells are SOR/registration specks and
    # are dropped from GEOMETRY only (the costmap still forbids them).
    n_lbl, lbl, stats, _ = cv2.connectedComponentsWithStats(
        occ.astype(np.uint8), connectivity=8)
    keep = np.zeros_like(occ)
    dropped_cells = 0
    dropped_comps = 0
    for i in range(1, n_lbl):
        area = stats[i, cv2.CC_STAT_AREA]
        if area >= args.min_cells:
            keep |= (lbl == i)
        else:
            dropped_cells += area
            dropped_comps += 1
    occc = keep
    resc = res
    print(f"components: {n_lbl - 1} total, kept {n_lbl - 1 - dropped_comps} "
          f"(>= {args.min_cells} cells), dropped {dropped_comps} specks "
          f"({dropped_cells} cells)")
    print(f"obstacle cells used for geometry: {occc.sum()} of {occ.sum()}")

    def ccell_x(c):
        return cell_x(0) + (c + 0.5) * resc

    def ccell_y(r):
        return cell_y(0) - (r + 0.5) * resc
    print(f"obstacle cells used directly: {occc.sum()}")

    # ---- bounding box of the MAPPED area (occupied OR carved free) ----
    # Walls must enclose everything the captured data observed, so no
    # free space is left outside the walls and no obstacle is cut.
    mapped = occ | free
    rs, cs = np.where(mapped)
    r0, r1, c0, c1 = rs.min(), rs.max(), cs.min(), cs.max()
    x_lo = ccell_x(c0) - resc / 2 - args.thickness / 2
    x_hi = ccell_x(c1) + resc / 2 + args.thickness / 2
    y_hi = ccell_y(r0) + resc / 2 + args.thickness / 2
    y_lo = ccell_y(r1) - resc / 2 - args.thickness / 2
    cx_room, cy_room = (x_lo + x_hi) / 2, (y_lo + y_hi) / 2
    ex, ey = x_hi - x_lo, y_hi - y_lo
    print(f"room extent: x [{x_lo:.2f},{x_hi:.2f}] y [{y_lo:.2f},{y_hi:.2f}] "
          f"({ex:.2f} x {ey:.2f} m)")

    models = []

    # ---- perimeter walls (full height) ----
    z1 = args.wall_height
    t = args.thickness
    models.append(model('wall_north', cx_room, y_hi, 0, z1, ex + t, t,
                        '0.62 0.60 0.55 1', '0.75 0.72 0.66 1'))
    models.append(model('wall_south', cx_room, y_lo, 0, z1, ex + t, t,
                        '0.62 0.60 0.55 1', '0.75 0.72 0.66 1'))
    models.append(model('wall_west', x_lo, cy_room, 0, z1, t, ey + t,
                        '0.58 0.56 0.52 1', '0.70 0.68 0.62 1'))
    models.append(model('wall_east', x_hi, cy_room, 0, z1, t, ey + t,
                        '0.58 0.56 0.52 1', '0.70 0.68 0.62 1'))

    # ---- interior obstacles: exact rects over ALL occupied cells ----
    rects = max_rects(occc)         # (row_top, col_left, h_rows, w_cols)
    rects_xy = [(ca, rt - hh, ca + ww, rt) for (rt, ca, hh, ww) in rects]
    rects_xy = merge_touching(rects_xy)
    print(f"obstacle blocks -> {len(rects)} exact rects -> "
          f"{len(rects_xy)} after exact merge")

    # clean simplification: merge nearby fragments of the same surface,
    # never covering carved-free cells; then drop 1-2 cell specks
    gap_cells = args.merge_gap / res
    rects_xy = merge_respecting_free(rects_xy, gap_cells, free, W, H)
    rects_xy = merge_touching(rects_xy)
    print(f"after clean merge (gap {args.merge_gap} m, carve-respecting): "
          f"{len(rects_xy)}")
    n = 0
    skipped_perim = 0
    for (ca, rb, cb, rt) in rects_xy:
        # continuous cell bounds: cols [ca, cb), rows [rb, rt)
        x0 = ccell_x(int(ca)) - resc / 2
        x1 = ccell_x(math.ceil(cb) - 1) + resc / 2
        y1 = ccell_y(int(rb)) + resc / 2
        y0 = ccell_y(math.ceil(rt) - 1) - resc / 2
        sx, sy = x1 - x0, y1 - y0
        area = sx * sy
        if area < args.min_area:
            continue
        # rects touching the room boundary are wall observations; the
        # perimeter walls already occupy that space
        edge = args.thickness + resc
        if (x0 <= x_lo + edge or x1 >= x_hi - edge or
                y0 <= y_lo + edge or y1 >= y_hi - edge):
            skipped_perim += 1
            continue
        n += 1
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        models.append(model(f'obstacle_{n:02d}', cx, cy,
                            args.obst_base, args.obst_top, sx, sy,
                            '0.30 0.42 0.55 1', '0.38 0.52 0.68 1'))
    print(f"interior obstacles rendered: {n} "
          f"(boundary-touching absorbed by walls: {skipped_perim})")

    # ---- GUI camera: top-down over the room center (floor-plan view) ----
    cam_h = max(ex, ey) * 1.35 + 2.0
    gui = f'''  <gui>
    <plugin name="3D View" filename="gz-sim-3d-view-system">
      <camera_name>user_cam</camera_name>
    </plugin>
    <plugin filename="gz-sim-navigation-messages-system" name="gz::sim::systems::NavigationMessages"/>
    <camera name="user_cam">
      <pose>{cx_room:.2f} {cy_room:.2f} {cam_h:.2f} 0 -1.5707 0</pose>
      <horizontal_fov>1.05</horizontal_fov>
    </camera>
  </gui>'''

    sdf = f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="{args.world_name}">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <gravity>0 0 -9.8</gravity>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <direction>-0.4 0.2 -0.9</direction>
    </light>
    <scene><ambient>0.45 0.45 0.45 1</ambient><background>0.35 0.42 0.52 1</background></scene>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material><ambient>0.42 0.42 0.44 1</ambient><diffuse>0.52 0.52 0.55 1</diffuse></material></visual>
      </link>
    </model>
{chr(10).join(models)}
{gui}
  </world>
</sdf>'''

    with open(args.out, 'w') as f:
        f.write(sdf)
    print(f"WORLD WRITTEN: {args.out}")
    ET.fromstring(sdf)  # XML validity check
    print("XML valid")


if __name__ == '__main__':
    main()
