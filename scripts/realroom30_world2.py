#!/usr/bin/env python3
"""Build a clean, readable Gazebo world from the realroom30 occupancy map.

Data source (unchanged): realroom/capture30/realroom30_map.pgm/.yaml, produced by
  30-s phone capture -> MiDaS depth -> COLMAP fusion -> voxel/SOR/RANSAC
  -> occupancy grid (realroom_cloud_to_map.py).
No synthetic obstacles are added; every interior box is an exact rectangle
decomposition of connected components of the captured occupied cells.
Perimeter walls bound the full mapped extent.
"""
import argparse
import math
import os
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', default=os.path.expanduser(
        '~/FYP/realroom/capture30/realroom30_map.yaml'))
    ap.add_argument('--out', default=os.path.expanduser(
        '~/FYP/realroom/capture30/realroom30_world2.sdf'))
    ap.add_argument('--world-name', default='realroom30_v2')
    ap.add_argument('--wall-height', type=float, default=1.2)
    ap.add_argument('--obst-top', type=float, default=0.9)
    ap.add_argument('--obst-base', type=float, default=0.28)
    ap.add_argument('--thickness', type=float, default=0.10)
    ap.add_argument('--min-cells', type=int, default=4,
                    help='drop connected components smaller than this many '
                         'cells from geometry (SOR specks; costmap keeps them)')
    ap.add_argument('--min-area', type=float, default=0.01,
                    help='min interior obstacle area m^2')
    args = ap.parse_args()

    meta = yaml.safe_load(open(args.map))
    img = cv2.imread(args.map.replace('.yaml', '.pgm'), cv2.IMREAD_GRAYSCALE)
    occ = (img < 100)
    free = (img == 205)
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    H, W = img.shape
    print(f"map {W}x{H} @ {res} m, occupied {occ.sum()}, carved free {free.sum()}")

    def cell_x(c):
        return ox + (c + 0.5) * res

    def cell_y(r):
        return oy + (H - r - 0.5) * res

    # connected components of the raw occupied mask (one per reconstructed
    # surface); tiny components are dropped from geometry only
    n_lbl, lbl, stats, _ = cv2.connectedComponentsWithStats(
        occ.astype(np.uint8), connectivity=8)
    keep = np.zeros_like(occ)
    dropped_cells = dropped_comps = 0
    for i in range(1, n_lbl):
        area = stats[i, cv2.CC_STAT_AREA]
        if area >= args.min_cells:
            keep |= (lbl == i)
        else:
            dropped_cells += area
            dropped_comps += 1
    print(f"components: {n_lbl - 1} total, kept {n_lbl - 1 - dropped_comps}, "
          f"dropped {dropped_comps} specks ({dropped_cells} cells)")

    # exact largest-rect decomposition of the kept mask (rows from top, cols)
    heights = np.zeros(W, dtype=int)
    rects = []
    for r in range(H):
        for c in range(W):
            heights[c] = heights[c] + 1 if keep[r, c] else 0
        stack = []
        c = 0
        while c <= W:
            cur = heights[c] if c < W else 0
            start = c
            while stack and stack[-1][1] >= cur:
                s0, hh = stack.pop()
                if hh > 0:
                    rects.append((r - hh + 1, s0, hh, c - s0))
                start = s0
            stack.append((start, cur))
            c += 1
    # (row_top, col_left, h_rows, w_cols) -> (x0c, y0r, x1c, y1r) in cell units
    boxes = [(ca, rt - hh, ca + ww, rt) for (rt, ca, hh, ww) in rects]
    # exact-touching merge: union bounding box adds no uncovered area
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
                if area_bb <= (a[2] - a[0]) * (a[3] - a[1]) + \
                        (b[2] - b[0]) * (b[3] - b[1]) + 1e-9:
                    out[i] = (x0, y0, x1, y1)
                    merged = changed = True
                    break
            if not merged:
                out.append(a)
        boxes = out
    print(f"obstacle rects: {len(rects)} exact -> {len(boxes)} after merge")

    # room extent over everything the captured data observed
    mapped = occ | free
    rs, cs = np.where(mapped)
    r0, r1, c0, c1 = rs.min(), rs.max(), cs.min(), cs.max()
    x_lo = cell_x(c0) - res / 2 - args.thickness / 2
    x_hi = cell_x(c1) + res / 2 + args.thickness / 2
    y_hi = cell_y(r0) + res / 2 + args.thickness / 2
    y_lo = cell_y(r1) - res / 2 - args.thickness / 2
    cx_room, cy_room = (x_lo + x_hi) / 2, (y_lo + y_hi) / 2
    ex, ey = x_hi - x_lo, y_hi - y_lo
    print(f"room extent: x [{x_lo:.2f},{x_hi:.2f}] y [{y_lo:.2f},{y_hi:.2f}] "
          f"({ex:.2f} x {ey:.2f} m)")

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

    models = []
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

    n = 0
    for (ca, rb, cb, rt) in boxes:
        x0 = cell_x(int(ca)) - res / 2
        x1 = cell_x(math.ceil(cb) - 1) + res / 2
        y1 = cell_y(int(rb)) + res / 2
        y0 = cell_y(math.ceil(rt) - 1) - res / 2
        sx, sy = x1 - x0, y1 - y0
        if sx * sy < args.min_area:
            continue
        if sx >= ex * 0.95 or sy >= ey * 0.95:
            continue  # spans the room -> part of the perimeter
        n += 1
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        models.append(model(f'obstacle_{n:02d}', cx, cy,
                            args.obst_base, args.obst_top, sx, sy,
                            '0.30 0.42 0.55 1', '0.38 0.52 0.68 1'))
    print(f"interior obstacles rendered: {n}")

    # top-down GUI camera over the room center
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
    ET.fromstring(sdf)
    print(f"WORLD WRITTEN: {args.out} (XML valid)")


if __name__ == '__main__':
    main()
