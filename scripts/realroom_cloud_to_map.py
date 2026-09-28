#!/usr/bin/env python
"""Convert the real-room reconstructed point cloud into a Nav2 2D occupancy map.

Takes the fused real-room PLY (scripts/realroom_fuse.py output), slices it at
robot-body height (obstacle band), projects the points into a 2D grid, marks
occupied cells, and carves FREE SPACE by ray-tracing from the COLMAP camera
positions (the real places the phone stood) to each obstacle point. This is
the missing link:

    REAL ROOM -> smartphone -> MiDaS+COLMAP reconstruction -> obstacle map -> Nav2

Usage:
    ~/miniconda3/envs/fyp/bin/python scripts/realroom_cloud_to_map.py \
        --cloud  ~/FYP/realroom/realroom_cloud.ply \
        --poses  ~/FYP/realroom/colmap/sparse/txt/images.txt \
        --out    ~/FYP/realroom/realroom_map
"""
import argparse
import math
from pathlib import Path

import numpy as np

try:
    import open3d as o3d
    HAVE_O3D = True
except ImportError:
    HAVE_O3D = False


def read_camera_centers(path):
    """COLMAP images.txt -> world-frame camera centers (-R^T t)."""
    def q2r(q):
        qw, qx, qy, qz = q
        return np.array([
            [1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
            [2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw],
            [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy]])
    centers, names = [], []
    for line in open(path):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        p = line.split()
        if len(p) >= 10 and p[0].isdigit():
            R = q2r(np.array(list(map(float, p[1:5]))))
            t = np.array(list(map(float, p[5:8])))
            centers.append(-R.T @ t)
            names.append(p[9])
    return np.array(centers), names


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cloud", required=True)
    ap.add_argument("--poses", required=True, help="COLMAP images.txt (camera centers for ray carving)")
    ap.add_argument("--out", required=True, help="output path base -> <out>.pgm + <out>.yaml")
    ap.add_argument("--zmin", type=float, default=0.15, help="obstacle band bottom (robot body height)")
    ap.add_argument("--zmax", type=float, default=1.20, help="obstacle band top")
    ap.add_argument("--res", type=float, default=0.05, help="grid resolution m/cell")
    ap.add_argument("--min-points", type=int, default=3, help="points required to mark a cell occupied")
    ap.add_argument("--max-ray", type=float, default=6.0, help="max carving ray length (m)")
    ap.add_argument("--pad", type=float, default=0.5, help="border padding (m)")
    ap.add_argument("--voxel", type=float, default=0.05, help="Stage-4 voxel downsampling size (m)")
    ap.add_argument("--sor-k", type=int, default=20, help="Stage-4 statistical outlier removal neighbours")
    ap.add_argument("--sor-std", type=float, default=2.0, help="Stage-4 SOR std ratio")
    ap.add_argument("--ransac-dist", type=float, default=0.02, help="Stage-4 RANSAC plane distance threshold (m)")
    ap.add_argument("--planes", type=int, default=2, help="max horizontal planes to remove (floor, ceiling)")
    ap.add_argument("--up-axis", choices=["auto", "x", "y", "z"], default="auto",
                    help="which cloud axis is room height (auto = smallest extent)")
    args = ap.parse_args()

    # ---- load cloud ----
    cpath = Path(args.cloud).expanduser()
    if HAVE_O3D:
        pcd = o3d.io.read_point_cloud(str(cpath))
        pts = np.asarray(pcd.points)
    else:
        pts = []
        in_data = False
        for line in open(cpath):
            if line.startswith("end_header"):
                in_data = True
                continue
            if in_data:
                v = line.split()
                pts.append([float(v[0]), float(v[1]), float(v[2])])
        pts = np.array(pts)
    print("cloud points:", len(pts))
    if len(pts) < 100:
        raise SystemExit("Point cloud too small — run realroom_fuse.py first.")

    centers, _ = read_camera_centers(Path(args.poses).expanduser())
    print("camera centers:", len(centers))

    # ---- gravity alignment (COLMAP world frame is arbitrary) ----
    if args.up_axis == "auto":
        ext = pts.max(axis=0) - pts.min(axis=0)
        up = "xyz"[int(np.argmin(ext))]
        print(f"auto up-axis: {up} (extents: {np.round(ext, 2)})")
    else:
        up = args.up_axis
    if up != "z":
        # rotate so the chosen axis becomes +z
        order = {"x": [1, 2, 0], "y": [2, 0, 1]}[up]
        pts = pts[:, order]
        centers = centers[:, order]
        print(f"rotated cloud: up-axis {up} -> z")

    # ---- report Stage 4: point-cloud filtering ----
    # 1) voxel downsampling  2) statistical outlier removal  3) RANSAC floor
    # removal. (4) pass-through filtering is the obstacle-band slice below,
    # applied in the gravity-aligned frame.
    if HAVE_O3D:
        n0 = len(pts)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
        pcd = pcd.voxel_down_sample(args.voxel)
        pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=args.sor_k, std_ratio=args.sor_std)
        sor_pts = np.asarray(pcd.points)
        removed = 0
        work = o3d.geometry.PointCloud()
        work.points = o3d.utility.Vector3dVector(sor_pts)
        for _ in range(args.planes):
            if len(work.points) < 500:
                break
            model, inliers = work.segment_plane(args.ransac_dist, 3, 200)
            a, b, c, d = model
            n = math.sqrt(a*a + b*b + c*c)
            # floor/ceiling-like = horizontal plane (normal close to vertical);
            # walls have horizontal normals and are kept.
            if abs(c) / n > 0.85 and len(inliers) > 0.02 * len(sor_pts):
                work = work.select_by_index(inliers, invert=True)
                removed += len(inliers)
            else:
                break
        pts = np.asarray(work.points)
        print(f"Stage-4 filtering: voxel+SOR {n0} -> {len(sor_pts)}; RANSAC floor removed {removed} -> {len(pts)}")

    # ---- obstacle band ----
    band = pts[(pts[:, 2] >= args.zmin) & (pts[:, 2] <= args.zmax)]
    print(f"points in obstacle band z=[{args.zmin},{args.zmax}]: {len(band)}")
    if len(band) < 50:
        raise SystemExit("Too few points in obstacle band — check cloud height/scale.")

    # ---- grid frame (2D: cloud x,y) ----
    # COLMAP world is often z-up from video captures; keep x,y as ground plane.
    res = args.res
    pad = args.pad
    x0, y0 = band[:, 0].min() - pad, band[:, 1].min() - pad
    x1, y1 = band[:, 0].max() + pad, band[:, 1].max() + pad
    W = int(math.ceil((x1 - x0) / res))
    H = int(math.ceil((y1 - y0) / res))
    print(f"map: {W}x{H} cells @ {res} m  origin=({x0:.2f},{y0:.2f})")

    def cell(p):
        return int((p[1] - y0) / res), int((p[0] - x0) / res)  # (row, col)

    # occupied votes
    occ = np.zeros((H, W), dtype=np.int32)
    rows = ((band[:, 1] - y0) / res).astype(int)
    cols = ((band[:, 0] - x0) / res).astype(int)
    ok = (rows >= 0) & (rows < H) & (cols >= 0) & (cols < W)
    np.add.at(occ, (rows[ok], cols[ok]), 1)

    # free-space carving from camera centers to band points
    free = np.zeros((H, W), dtype=bool)
    ncarve = 0
    for c in centers:
        c2 = (c[0], c[1])
        d = np.hypot(band[:, 0] - c2[0], band[:, 1] - c2[1])
        ends = band[d <= args.max_ray]
        for e in ends:
            r0, cc0 = cell((c2[0], c2[1]))
            r1, cc1 = cell(e)
            n = max(abs(r1 - r0), abs(cc1 - cc0))
            if n == 0:
                continue
            for i in range(1, n):  # stop before the endpoint (it's an obstacle)
                t = i / n
                rr = int(round(r0 + (r1 - r0) * t))
                ccc = int(round(cc0 + (cc1 - cc0) * t))
                if 0 <= rr < H and 0 <= ccc < W:
                    free[rr, ccc] = True
            ncarve += 1
    print("carved rays:", ncarve)

    # ---- build PGM ----
    grid = np.full((H, W), 205, dtype=np.uint8)  # unknown
    grid[free] = 254                             # free
    grid[occ >= args.min_points] = 0             # occupied
    out = Path(args.out).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    try:
        from PIL import Image
        Image.fromarray(grid).save(str(out) + ".pgm")
    except ImportError:
        raise SystemExit("PIL required")

    with open(str(out) + ".yaml", "w") as f:
        f.write(f"image: {out.name}.pgm\n")
        f.write(f"resolution: {res}\n")
        f.write(f"origin: [{x0:.3f}, {y0:.3f}, 0.0]\n")
        f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n")

    occ_cells = int((grid == 0).sum())
    free_cells = int((grid == 254).sum())
    print(f"MAP WRITTEN: {out}.pgm/.yaml  occupied={occ_cells} free={free_cells} unknown={int((grid==205).sum())}")
    if occ_cells < 20 or free_cells < 200:
        print("WARNING: map looks degenerate — inspect the point cloud before navigating.", flush=True)


if __name__ == "__main__":
    main()
