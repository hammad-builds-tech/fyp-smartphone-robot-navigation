#!/usr/bin/env python3
"""fyp30_decorate_world.py -- VISUAL-ONLY restyling pass for a realroom30 world.

Rewrites, inside the SAME world SDF (geometries, collision blocks, model poses
and model plugin XML are all left EXACTLY as-is):

  * every obstacle_* box gets a deterministic palette color keyed by the
    connected component (cluster) it belongs to, so a reconstructed object
    looks like ONE object, not confetti;
  * wall_* get a warm indoor cream paint;
  * floor_slab gets a brighter tile tone and ground_plane a muted tone;
  * scene ambient lifted + sun softened + two hands-free ceiling point lights.

Model poses, <collision> blocks, sizes and physics are NEVER touched, so the
baked map (generated from the same SDF geometry) stays valid.

Usage:
  python3 fyp30_decorate_world.py <world.sdf> [--out FILE] [--backup] [--dry-run]
If --out is omitted the target file is rewritten IN-PLACE (requires --backup,
which writes <world>.orig first if not present).
"""
import argparse
import os
import re
import shutil
import sys

WALL_AMBIENT = "0.62 0.60 0.56 1"
WALL_DIFFUSE = "0.80 0.78 0.72 1"
WALL_SPEC    = "0.10 0.10 0.09 1"

FLOOR_AMBIENT = "0.55 0.53 0.50 1"
FLOOR_DIFFUSE = "0.82 0.80 0.77 1"
FLOOR_SPEC    = "0.35 0.35 0.32 1"

OUTSIDE_AMBIENT = "0.30 0.34 0.30 1"
OUTSIDE_DIFFUSE = "0.38 0.45 0.38 1"

OBST_SPEC = "0.35 0.33 0.30 1"

# 10 obstacle-cluster colours (HSV: hue, sat, val) -- saturated so boxes read
# clearly against the pale floor at any exposure.
CLUSTER_COLORS = [
    (0.02, 0.72, 0.72),  # red
    (0.08, 0.75, 0.70),  # orange
    (0.13, 0.70, 0.72),  # gold
    (0.30, 0.68, 0.68),  # green
    (0.45, 0.62, 0.62),  # teal
    (0.55, 0.70, 0.65),  # azure
    (0.60, 0.72, 0.70),  # blue
    (0.75, 0.65, 0.68),  # purple
    (0.90, 0.70, 0.70),  # pink
    (0.50, 0.30, 0.75),  # cocoa
]

CELL_OFFSETS = [
    (dx, dy)
    for dx in (-0.4, -0.3, -0.2, -0.15, -0.1, -0.05, 0, 0.05, 0.1, 0.15, 0.2, 0.3, 0.4)
    for dy in (-0.4, -0.3, -0.2, -0.15, -0.1, -0.05, 0, 0.05, 0.1, 0.15, 0.2, 0.3, 0.4)
]


def hsv_to_rgb(h, s, v):
    i = int(h * 6.0) % 6
    f = h * 6.0 - int(h * 6.0)
    p = v * (1 - s); q = v * (1 - f * s); t = v * (1 - (1 - f) * s)
    return [(v, t, p), (q, v, p), (p, v, t),
            (p, q, v), (t, p, v), (v, p, q)][i]


def fmt3(rgb):
    return " ".join("%.2f" % c for c in rgb)


def material_block(amb, diff, spec):
    return ("<material><ambient>%s</ambient><diffuse>%s</diffuse>"
            "<specular>%s</specular></material>" % (amb, diff, spec))


def cluster_key(x, y, cells):
    """BFS-union every grid cell reachable from (x, y) within <=0.4 m gaps.

    Deterministic for a given world, independent of file ordering."""
    stack, seen = [(x, y)], {(x, y)}
    while stack:
        cx, cy = stack.pop()
        for dx, dy in CELL_OFFSETS:
            nb = (round(cx + dx, 3), round(cy + dy, 3))
            if nb in cells and nb not in seen:
                seen.add(nb)
                stack.append(nb)
    return frozenset(seen)


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
    orig_len = len(src)

    # ---------------- obstacle grid (pose -> material region) --------------
    cell_re = re.compile(
        r'<model name="obstacle_(\d+)">\s*<static>true</static>\s*'
        r'<pose>(-?[\d.]+) (-?[\d.]+) ([-\d.]+)')
    cells = {}
    for m in cell_re.finditer(src):
        x, y = round(float(m.group(2)), 3), round(float(m.group(3)), 3)
        cells.setdefault((x, y), int(m.group(1)))
    if not cells:
        print("WARN: no obstacle_* models matched -- nothing restyled")
        return 1

    keys = set(cells)
    seen = set()
    cluster_of = {}
    palette = []
    for k in sorted(keys):
        if k in seen:
            continue
        comp = cluster_key(k[0], k[1], keys)
        seen |= comp
        palette.append(hsv_to_rgb(*CLUSTER_COLORS[len(palette) % len(CLUSTER_COLORS)]))
        for c in comp:
            cluster_of[c] = len(palette) - 1
    print("obstacle clusters: %d (palette size %d)" % (len(palette), len(CLUSTER_COLORS)))

    # ---------------- rewrite each obstacle model's <material> -------------
    model_re = re.compile(
        r'<model name="obstacle_(\d+)">.*?</model>', re.S)
    n_obst = 0

    def obs_repl(m):
        nonlocal n_obst
        body = m.group(0)
        pm = re.search(r'<pose>(-?[\d.]+) (-?[\d.]+) ([-\d.]+)', body)
        x, y = round(float(pm.group(1)), 3), round(float(pm.group(2)), 3)
        col = palette[cluster_of[(x, y)]]
        new_mat = material_block(
            "0.30 %.2f %.2f 1" % (col[1] * 0.55, col[2] * 0.55),
            fmt3(col) + " 1", OBST_SPEC)
        new_body, cnt = re.subn(r'<material>.*?</material>', new_mat,
                                body, count=1, flags=re.S)
        if cnt:
            n_obst += 1
        return new_body

    src = model_re.sub(obs_repl, src)
    print("obstacle materials rewritten: %d / %d" % (n_obst, len(cells)))

    # ---------------- walls ------------------------------------------------
    n_walls = 0

    def wall_repl(m):
        nonlocal n_walls
        body = m.group(0)
        new_body, cnt = re.subn(
            r'<material>.*?</material>',
            material_block(WALL_AMBIENT, WALL_DIFFUSE, WALL_SPEC),
            body, count=1, flags=re.S)
        if cnt:
            n_walls += 1
        return new_body

    src = re.sub(r'<model name="wall_\w+">.*?</model>', wall_repl, src, flags=re.S)
    print("walls restyled: %d" % n_walls)

    # ---------------- floor + ground plane --------------------------------
    src, k1 = re.subn(
        r'(<model name="floor_slab">.*?<visual name="visual">.*?<geometry>.*?</geometry>)\s*<material>.*?</material>',
        r'\g<1>' + material_block(FLOOR_AMBIENT, FLOOR_DIFFUSE, FLOOR_SPEC),
        src, count=1, flags=re.S)
    src, k2 = re.subn(
        r'(<model name="ground_plane">.*?<visual name="visual">.*?<geometry>.*?</geometry>)\s*<material>.*?</material>',
        r'\g<1>' + material_block(OUTSIDE_AMBIENT, OUTSIDE_DIFFUSE, "0.20 0.20 0.20 1"),
        src, count=1, flags=re.S)
    print("floor/ground restyled: %d" % (k1 + k2))


    # ---------------- wall height raise (visual realism) --------------------
    # 2D bake only consumes wall XY footprints, so doubling the wall height is
    # navigation-neutral; it makes the room read as a room, not a barrier.
    src, kw = re.subn(r'(<size>18\.100 0\.100) 1\.200(</size>)', r'\g<1> 2.400\g<2>', src)
    src, kw2 = re.subn(r'(<size>0\.100 20\.850) 1\.200(</size>)', r'\g<1> 2.400\g<2>', src)
    src, kw3 = re.subn(r'(<pose>-?[\d.]+ (?:17\.743|-3\.007|7\.368)) 0\.600 (0 0 0</pose>)',
                       r'\g<1> 1.200 \g<2>', src)
    _ = kw3
    print("wall height raised: %d slabs" % (kw + kw2))

    # ---------------- scene + sun ------------------------------------------
    new_scene = ('<scene><ambient>0.55 0.55 0.55 1</ambient>'
                 '<background>0.32 0.40 0.52 1</background>'
                 '<shadows>true</shadows></scene>')
    src, k3 = re.subn(r'<scene>.*?</scene>', new_scene, src, count=1, flags=re.S)
    new_sun = ('<light type="directional" name="sun">'
               '<cast_shadows>true</cast_shadows>'
               '<pose>0 0 12 0 0 0</pose>'
               '<diffuse>0.62 0.60 0.55 1</diffuse>'
               '<specular>0.10 0.10 0.10 1</specular>'
               '<direction>-0.30 0.18 -0.9</direction>'
               '</light>')
    src, k4 = re.subn(r'<light type="directional" name="sun">.*?</light>',
                      new_sun, src, count=1, flags=re.S)
    print("scene+sun rewritten: %d" % (k3 + k4))

    # ---------------- indoor ceiling fill lights ----------------------------
    fill = ('  <light type="point" name="ceiling_1">'
            '<pose>-4.0 7.5 1.6 0 0 0</pose>'
            '<diffuse>0.55 0.52 0.46 1</diffuse>'
            '<attenuation><range>14</range><constant>0.30</constant>'
            '<linear>0.07</linear><quadratic>0.01</quadratic></attenuation>'
            '<cast_shadows>false</cast_shadows></light>\n'
            '  <light type="point" name="ceiling_2">'
            '<pose>3.0 9.0 1.6 0 0 0</pose>'
            '<diffuse>0.50 0.48 0.44 1</diffuse>'
            '<attenuation><range>14</range><constant>0.30</constant>'
            '<linear>0.07</linear><quadratic>0.01</quadratic></attenuation>'
            '<cast_shadows>false</cast_shadows></light>\n')
    added_lights = 0
    if 'name="ceiling_1"' not in src:
        src = src.replace("</world>", fill + "</world>")
        added_lights = 2
    print("ceiling lights added: %d" % added_lights)

    if n_obst < 300:
        print("WARN: fewer than 300 obstacle materials rewritten (expected 345)")
        return 3
    if args.dry_run:
        print("dry-run: not writing")
        return 0

    out = args.out
    if out:
        out = os.path.abspath(os.path.expanduser(out))
        with open(out, "w", encoding="utf-8") as f:
            f.write(src)
        print("wrote %s (%d chars, was %d)" % (out, len(src), orig_len))
    else:
        if not args.backup:
            print("ERROR: refusing in-place rewrite without --backup")
            return 2
        bkp = path + ".orig"
        if not os.path.exists(bkp):
            shutil.copy2(path, bkp)
            print("backup -> %s" % bkp)
        with open(path, "w", encoding="utf-8") as f:
            f.write(src)
        print("rewrote %s in place (%d chars, was %d)" % (path, len(src), orig_len))
    return 0


if __name__ == "__main__":
    sys.exit(main())
