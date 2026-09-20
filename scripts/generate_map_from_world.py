#!/usr/bin/env python3
"""Generate the 2D occupancy map for Nav2 directly from the Gazebo world.

The saved indoor_map.pgm must match indoor_world.sdf exactly, otherwise the
static costmap shows walls where Gazebo has free space (or vice versa) and
AMCL can never localize. This script rasterizes every static model in the
world that intersects the robot's body-height band into a PGM.

Usage (with the fyp conda env):
    ~/miniconda3/envs/fyp/bin/python scripts/generate_map_from_world.py
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

FYP_DIR = Path.home() / "FYP"
WORLD_SDF = (
    FYP_DIR / "ros2_ws" / "src" / "indoor_nav_gazebo" / "worlds" / "indoor_world.sdf"
)
MAP_DIR = FYP_DIR / "ros2_ws" / "src" / "indoor_nav_costmap" / "maps"
MAP_PGM = MAP_DIR / "indoor_map.pgm"
MAP_YAML = MAP_DIR / "indoor_map.yaml"

RESOLUTION = 0.05          # m per pixel, must match nav2_params costmaps
ROBOT_Z_MIN = 0.05         # body-height band: obstacles overlapping these
ROBOT_Z_MAX = 1.50         # heights can collide with the robot


def parse_pose(element, default=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)):
    if element is None:
        return list(default)
    values = [float(v) for v in (element.text or "").split()]
    return values + list(default[len(values):])


def world_aabb(model):
    """Return (xmin, xmax, ymin, ymax, zmin, zmax) of a static model."""
    model_pose = parse_pose(model.find("pose"))
    mx, my, mz = model_pose[0], model_pose[1], model_pose[2]

    xmin = ymin = zmin = float("inf")
    xmax = ymax = zmax = float("-inf")

    for link in model.findall("link"):
        link_pose = parse_pose(link.find("pose"))
        lx, ly, lz = mx + link_pose[0], my + link_pose[1], mz + link_pose[2]

        for collision in link.findall("collision"):
            size_el = collision.find("geometry/box/size")
            if size_el is None:
                continue
            sx, sy, sz = (float(v) for v in size_el.text.split())
            xmin = min(xmin, lx - sx / 2)
            xmax = max(xmax, lx + sx / 2)
            ymin = min(ymin, ly - sy / 2)
            ymax = max(ymax, ly + sy / 2)
            zmin = min(zmin, lz - sz / 2)
            zmax = max(zmax, lz + sz / 2)

    if xmin is float("inf"):
        return None
    return xmin, xmax, ymin, ymax, zmin, zmax


def main():
    if not WORLD_SDF.exists():
        print(f"ERROR: world file not found: {WORLD_SDF}")
        return 1

    tree = ET.parse(WORLD_SDF)
    world = tree.getroot().find("world")

    # Determine map extent from the floor (first static model with a box).
    floor_size = None
    for model in world.findall("model"):
        if model.get("name") == "floor":
            size_el = model.find("link/collision/geometry/box/size")
            if size_el is not None:
                floor_size = [float(v) for v in size_el.text.split()]
            break
    if floor_size is None:
        print("ERROR: could not determine floor size from world")
        return 1

    world_w = floor_size[0]          # x extent, m
    world_h = floor_size[1]          # y extent, m
    origin_x = -world_w / 2          # map frame origin (lower-left corner)
    origin_y = -world_h / 2

    width = int(round(world_w / RESOLUTION))
    height = int(round(world_h / RESOLUTION))

    # 255 = free, 0 = occupied (Nav2 map_server: black == occupied)
    grid = [[255] * width for _ in range(height)]

    marked = 0
    for model in world.findall("model"):
        if model.get("name") == "floor":
            continue  # the floor is walkable, never an obstacle
        aabb = world_aabb(model)
        if aabb is None:
            continue
        xmin, xmax, ymin, ymax, zmin, zmax = aabb
        # Skip models the robot's body can pass over/under.
        if zmax <= ROBOT_Z_MIN or zmin >= ROBOT_Z_MAX:
            print(f"  skipped (outside body band): {model.get('name')}")
            continue

        col0 = int((xmin - origin_x) / RESOLUTION)
        col1 = int(round((xmax - origin_x) / RESOLUTION))
        row0 = int((origin_y + world_h - ymax) / RESOLUTION)   # image y is flipped
        row1 = int(round((origin_y + world_h - ymin) / RESOLUTION))
        col0, col1 = max(0, col0), min(width, col1)
        row0, row1 = max(0, row0), min(height, row1)

        cells = 0
        for r in range(row0, row1):
            for c in range(col0, col1):
                grid[r][c] = 0
                cells += 1
        marked += cells
        print(
            f"  {model.get('name'):12s} cols[{col0}:{col1}) rows[{row0}:{row1}) "
            f"= {cells} px"
        )
        if cells == 0:
            print(f"  WARNING: {model.get('name')} fell outside the raster")

    MAP_DIR.mkdir(parents=True, exist_ok=True)

    # Write binary PGM (P5)
    payload = bytearray()
    payload += f"P5\n{width} {height}\n255\n".encode("ascii")
    for r in range(height):
        payload += bytes(grid[r])

    MAP_PGM.write_bytes(payload)
    print(f"wrote {MAP_PGM} ({width}x{height}, {marked} occupied px)")

    MAP_YAML.write_text(
        f"image: {MAP_PGM.name}\n"
        f"resolution: {RESOLUTION}\n"
        f"origin: [{origin_x}, {origin_y}, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n"
        "mode: trinary\n"
    )
    print(f"wrote {MAP_YAML}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
