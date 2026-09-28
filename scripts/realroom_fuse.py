#!/usr/bin/env python
"""Fuse MiDaS per-frame depth with COLMAP camera poses into a real-room
point cloud (parameterized version of the project's fuse_midas_colmap.py).

Same math as the original offline pipeline that produced dense/midas_fused_fixed.ply:
  - MiDaS inverse depth -> robust normalize -> z = 1/(d + 0.05)
  - pinhole unprojection at a pixel grid
  - COLMAP world transform X_world = R^T (X_cam - t)
  - global radial outlier trim

Usage:
    ~/miniconda3/envs/fyp/bin/python scripts/realroom_fuse.py \
        --img-dir  ~/FYP/realroom/capture/img \
        --depth-dir ~/FYP/realroom/capture/depth_midas \
        --poses    ~/FYP/realroom/colmap/sparse/txt/images.txt \
        --cameras  ~/FYP/realroom/colmap/sparse/txt/cameras.txt \
        --out      ~/FYP/realroom/realroom_cloud.ply
"""
import argparse
import os
import sys
from pathlib import Path

import numpy as np
from PIL import Image


def read_pfm(path):
    with open(path, "rb") as f:
        header = f.readline().decode().strip()
        color = header == "PF"
        dims = f.readline().decode().strip()
        while dims.startswith("#"):
            dims = f.readline().decode().strip()
        w, h = map(int, dims.split())
        scale = float(f.readline().decode().strip())
        data = np.fromfile(f, dtype="<f4" if scale < 0 else ">f4")
        shape = (h, w, 3) if color else (h, w)
        return np.flipud(np.reshape(data, shape))


def qvec2rotmat(q):
    qw, qx, qy, qz = q
    return np.array([
        [1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
        [2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
        [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy],
    ])


def read_cameras(path):
    for line in open(path):
        if not line.strip() or line.startswith("#"):
            continue
        p = line.split()
        model = p[1]
        if model == "PINHOLE":
            return dict(fx=float(p[4]), fy=float(p[5]), cx=float(p[6]), cy=float(p[7]), w=int(p[2]), h=int(p[3]))
        if model in ("SIMPLE_PINHOLE", "SIMPLE_RADIAL", "RADIAL"):
            return dict(fx=float(p[4]), fy=float(p[4]), cx=float(p[5]), cy=float(p[6]), w=int(p[2]), h=int(p[3]))
        if model == "OPENCV":
            # fx fy cx cy + distortion (k1 k2 p1 p2 ...). Distortion is
            # negligible at the phone camera's narrow FOV; the pinhole
            # projection equations from the report are applied directly.
            return dict(fx=float(p[4]), fy=float(p[5]), cx=float(p[6]), cy=float(p[7]), w=int(p[2]), h=int(p[3]))
        raise ValueError(f"Unsupported camera model: {model}")
    raise RuntimeError("No camera in file")


def read_poses(path):
    poses = {}
    for line in open(path):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        p = line.split()
        if len(p) >= 10 and p[0].isdigit():
            q = np.array(list(map(float, p[1:5])))
            t = np.array(list(map(float, p[5:8])))
            poses[p[9]] = (qvec2rotmat(q), t)
    return poses


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--img-dir", required=True)
    ap.add_argument("--depth-dir", required=True, help="MiDaS output dir with <name>-dpt_hybrid_384.pfm")
    ap.add_argument("--poses", required=True, help="COLMAP images.txt")
    ap.add_argument("--cameras", required=True, help="COLMAP cameras.txt")
    ap.add_argument("--out", required=True)
    ap.add_argument("--step", type=int, default=4)
    ap.add_argument("--max-frames", type=int, default=0)
    args = ap.parse_args()

    cam = read_cameras(args.cameras)
    poses = read_poses(args.poses)
    print("Registered images:", len(poses))
    if not poses:
        sys.exit("COLMAP registered no images — re-capture with more overlap / texture.")

    img_dir = Path(args.img_dir).expanduser()
    depth_dir = Path(args.depth_dir).expanduser()
    ref_img = next(img_dir.glob("*.jpg"))
    W0, H0 = Image.open(ref_img).size  # intrinsics in cameras.txt refer to this size

    all_points = []
    used = 0
    for name, (R, t) in poses.items():
        base = os.path.splitext(name)[0]
        pfm = depth_dir / f"{base}-dpt_hybrid_384.pfm"
        image_file = img_dir / name
        if not pfm.exists() or not image_file.exists():
            print("Missing:", pfm if not pfm.exists() else image_file)
            continue

        depth = read_pfm(pfm).astype(np.float32)
        image = np.asarray(Image.open(image_file).convert("RGB"))
        h, w = depth.shape

        valid = np.isfinite(depth)
        if not np.any(valid):
            continue
        d = depth[valid]
        lo, hi = np.percentile(d, 2), np.percentile(d, 98)
        depth = np.clip(depth, lo, hi)
        depth = (depth - lo) / max(hi - lo, 1e-6)
        z = 1.0 / np.maximum(depth + 0.05, 0.05)

        rgb = np.asarray(Image.fromarray(image).resize((w, h)))
        sx, sy = w / W0, h / H0
        fxx, fyy = cam["fx"] * sx, cam["fy"] * sy
        cxx, cyy = cam["cx"] * sx, cam["cy"] * sy

        step = args.step
        ys, xs = np.mgrid[0:h:step, 0:w:step]
        zz = z[::step, ::step]
        xx = (xs - cxx) * zz / fxx
        yy = (ys - cyy) * zz / fyy
        pts_cam = np.stack([xx, yy, zz], axis=-1).reshape(-1, 3)
        cols = rgb[::step, ::step].reshape(-1, 3)

        pts_world = (R.T @ (pts_cam.T - t[:, None])).T
        ok = np.isfinite(pts_world).all(axis=1)
        all_points.append(np.column_stack([pts_world[ok], cols[ok]]))
        used += 1
        print(f"{name} -> {int(ok.sum())} points")

    if not all_points or used == 0:
        sys.exit("No points generated.")
    points = np.vstack(all_points)

    xyz = points[:, :3]
    center = np.median(xyz, axis=0)
    dist = np.linalg.norm(xyz - center, axis=1)
    points = points[dist <= np.percentile(dist, 97)]

    out = Path(args.out).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        np.savetxt(f, points, fmt="%.4f %.4f %.4f %d %d %d")

    print(f"FUSED {used} frames -> {out} ({len(points)} points)")


if __name__ == "__main__":
    main()
