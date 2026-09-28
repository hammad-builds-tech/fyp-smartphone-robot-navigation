#!/usr/bin/env python
"""Convert the real-room occupancy map into a Gazebo world replica.

Takes the Nav2 map generated from the real-room reconstruction
(scripts/realroom_cloud_to_map.py output) and extrudes occupied cells into
Gazebo boxes, producing a simulated environment that geometrically matches
the scanned real room. This lets the report-scoped Gazebo navigation
demonstration run inside the reconstructed real environment:

    REAL ROOM -> smartphone -> reconstruction -> occupancy map -> Gazebo world
    -> Nav2 navigation among the real room's obstacles

Usage:
    ~/miniconda3/envs/fyp/bin/python scripts/realroom_map_to_world.py \
        --map /tmp/realroom_map.yaml \
        --out ~/FYP/ros2_ws/src/indoor_nav_gazebo/worlds/realroom_world.sdf

The map YAML follows the ROS map_server format (image, resolution, origin).
A simple pass merges adjacent occupied cells into larger boxes so the world
stays lightweight.
"""
import argparse
import numpy as np
import yaml
from pathlib import Path


def load_map(yaml_path):
    meta = yaml.safe_load(open(yaml_path))
    img_path = Path(yaml_path).parent / meta["image"]
    import cv2
    img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise SystemExit(f"cannot read map image: {img_path}")
    img = 255 - img  # PGM: 255=free, 0=occupied -> occupied mask = high
    res = float(meta["resolution"])
    ox, oy, _ = meta["origin"]
    h, w = img.shape
    # realroom_cloud_to_map.py writes band point y directly into file rows
    # (row 0 = min y). map_server mirrors the image into the ROS map frame
    # (row 0 = max y), so flip the rows here to place boxes where Nav2/AMCL
    # will actually see them:
    img = img[::-1]
    xs = ox + (np.arange(w) + 0.5) * res
    ys = oy + (np.arange(h) + 0.5) * res
    return img, xs, ys, res


def extract_boxes(occ, xs, ys, res, min_cells=2):
    """Greedy merge of occupied cells into axis-aligned rectangles."""
    boxes = []
    free = occ.copy()
    for r in range(occ.shape[0]):
        c = 0
        while c < occ.shape[1]:
            if free[r, c]:
                c += 1
                continue
            width = 1
            while c + width < occ.shape[1] and not free[r, c + width]:
                width += 1
            height = 1
            while (r + height < occ.shape[0]
                   and not free[r + height, c:c + width].any()):
                height += 1
            if width * height >= min_cells:
                boxes.append((c, r, width, height))
                free[r:r + height, c:c + width] = True
            c += width
    return boxes


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", required=True, help="map YAML from realroom_cloud_to_map.py")
    ap.add_argument("--out", required=True, help="output SDF world path")
    ap.add_argument("--wall-height", type=float, default=1.2)
    ap.add_argument("--cell-size", type=float, default=0.2,
                    help="world grid cell size in m (decimates the map to keep "
                         "the Gazebo model count low; the Nav2 map is untouched)")
    ap.add_argument("--name", default="realroom_replica")
    args = ap.parse_args()

    occ, xs, ys, res = load_map(args.map)
    occ = occ > 127  # occupied cells in the PGM-derived grid
    print(f"map {occ.shape[1]}x{occ.shape[0]}, occupied cells: {occ.sum()}")

    # Decimate to the requested world cell size (larger cells -> fewer Gazebo
    # models -> real-time simulation; the occupancy MAP keeps its resolution).
    k = max(1, int(round(args.cell_size / res)))
    if k > 1:
        H0, W0 = occ.shape
        occ = occ[: H0 // k * k, : W0 // k * k]
        occ = occ.reshape(H0 // k, k, W0 // k, k).any(axis=(1, 3))
        res = args.cell_size
        xs = xs[::k]
        ys = ys[::k]
        print(f"decimated to {res} m cells for the world: {occ.sum()} blocks")

    boxes = extract_boxes(occ.astype(np.uint8), xs, ys, res)
    models = []
    for i, (c, r, w, h) in enumerate(boxes):
        x0 = xs[c] - res / 2
        x1 = xs[c + w - 1] + res / 2
        y0 = ys[r] - res / 2
        y1 = ys[r + h - 1] + res / 2
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        sx, sy = x1 - x0, y1 - y0
        models.append(
            f'''    <model name="obstacle_{i}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {args.wall_height / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.3f} {sy:.3f} {args.wall_height:.3f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.3f} {sy:.3f} {args.wall_height:.3f}</size></box></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>'''
        )
    print(f"extracted {len(models)} merged obstacle boxes")

    sdf = f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="{args.name}">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <gravity>0 0 -9.8</gravity>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material></visual>
      </link>
    </model>
{chr(10).join(models)}
  </world>
</sdf>'''
    Path(args.out).write_text(sdf)
    print(f"WORLD WRITTEN: {args.out}")


if __name__ == "__main__":
    main()
