#!/usr/bin/env python
"""Offline MiDaS batch depth for the fresh 30-second capture.

Runs AFTER the capture window is closed (offline enforcement active):
reads paired RGB frames and writes one PFM per frame named
<frame>-dpt_hybrid_384.pfm — exactly the naming/format realroom_fuse.py
expects (Pf, float32, little-endian => scale written as -1.0).

Usage:
    ~/miniconda3/envs/fyp/bin/python scripts/fyp30_midas_batch.py \
        --img-dir  ~/FYP/realroom/capture_fresh/img \
        --out-dir  ~/FYP/realroom/capture_fresh/depth_midas
"""
import argparse
import sys
from pathlib import Path

FYP_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(FYP_ROOT))

import cv2
import numpy as np

from backend.depth.midas_processor import MiDaSProcessor


def write_pfm(path, image, scale=1.0):
    """Standard PFM writer (MiDaS-repo compatible)."""
    image = np.ascontiguousarray(image.astype(np.float32))
    color = len(image.shape) == 3 and image.shape[2] == 3
    with open(path, "wb") as f:
        f.write(b"PF\n" if color else b"Pf\n")
        f.write(b"%d %d\n" % (image.shape[1], image.shape[0]))
        endian = image.dtype.byteorder
        if endian == "<" or (endian == "=" and np.little_endian):
            scale = -scale
        f.write(b"%f\n" % scale)
        image.tofile(f)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--img-dir", required=True)
    ap.add_argument("--out-dir", required=True)
    args = ap.parse_args()

    img_dir = Path(args.img_dir).expanduser()
    out_dir = Path(args.out_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)

    files = sorted(list(img_dir.glob("*.jpg")) + list(img_dir.glob("*.png")))
    if not files:
        sys.exit(f"No RGB frames in {img_dir}")

    proc = MiDaSProcessor()
    for f in files:
        img = cv2.imread(str(f))
        if img is None:
            print(f"SKIP unreadable: {f.name}")
            continue
        depth = proc.predict(img)  # (H, W) inverse depth at image resolution
        out = out_dir / (f.stem + "-dpt_hybrid_384.pfm")
        write_pfm(out, depth)
        print(f"{f.name} -> {out.name} shape={depth.shape} "
              f"range=[{depth.min():.3f}, {depth.max():.3f}]")

    print(f"DONE: {len(files)} PFM files in {out_dir}")


if __name__ == "__main__":
    main()
