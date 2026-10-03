#!/usr/bin/env python
"""Useful-frame selection for the generic pipeline (speed stage).

A raw extraction at N fps produces many frames that add nothing to the
reconstruction: motion-blurred frames (no COLMAP features) and near-
duplicates of the previous frame (wasted MiDaS + matching work). This stage
keeps only frames that are (a) sharp enough and (b) sufficiently different
from the previously kept frame, then caps the kept set at --max-frames with
order-preserving uniform sampling (sequential COLMAP matching still sees a
temporal chain).

Rejected frames are MOVED to <img-dir>_rejected so the dataset only carries
frames that will actually be processed. Idempotent: reruns converge (the
kept set is stable).

Usage:
  python fyp30_select_frames.py --img-dir <dataset>/img \
      [--min-sharpness 25] [--dup-threshold 2.0] [--max-frames 140]
"""
import argparse
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np


def sharpness(gray_small):
    """Variance of the Laplacian - classic blur metric (higher = sharper)."""
    return float(cv2.Laplacian(gray_small, cv2.CV_64F).var())


def frame_diff(a, b):
    """Mean absolute pixel difference (0 = identical)."""
    return float(np.mean(cv2.absdiff(a, b)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--img-dir", required=True)
    ap.add_argument("--min-sharpness", type=float, default=25.0,
                    help="min variance-of-Laplacian to keep a frame")
    ap.add_argument("--dup-threshold", type=float, default=2.0,
                    help="mean abs diff below which a frame is a near-duplicate")
    ap.add_argument("--max-frames", type=int, default=140,
                    help="cap on kept frames (uniform, order-preserving)")
    args = ap.parse_args()

    img_dir = Path(args.img_dir).expanduser()
    files = sorted(list(img_dir.glob("*.jpg")) + list(img_dir.glob("*.jpeg"))
                   + list(img_dir.glob("*.png")))
    if not files:
        sys.exit(f"FAIL: no frames in {img_dir}")
    if len(files) <= args.max_frames:
        # still worth dedup/blur filtering, but nothing to cap
        pass

    rejected = img_dir.parent / (img_dir.name + "_rejected")
    kept, prev = [], None
    for f in files:
        img = cv2.imread(str(f), cv2.IMREAD_GRAYSCALE)
        if img is None:
            print(f"DROP unreadable: {f.name}")
            (rejected or img_dir.parent).mkdir(parents=True, exist_ok=True)
            shutil.move(str(f), rejected / f.name)
            continue
        h, w = img.shape
        scale = 480.0 / max(h, w)
        small = cv2.resize(img, (int(w * scale), int(h * scale)),
                           interpolation=cv2.INTER_AREA) if scale < 1 else img
        s = sharpness(small)
        if s < args.min_sharpness:
            print(f"DROP blurred ({s:.1f} < {args.min_sharpness}): {f.name}")
            rejected.mkdir(parents=True, exist_ok=True)
            shutil.move(str(f), rejected / f.name)
            continue
        if prev is not None and frame_diff(prev, small) < args.dup_threshold:
            print(f"DROP duplicate: {f.name}")
            rejected.mkdir(parents=True, exist_ok=True)
            shutil.move(str(f), rejected / f.name)
            continue
        kept.append(f)
        prev = small

    over = len(kept) - args.max_frames
    if over > 0:
        keep_idx = set(np.unique(
            np.linspace(0, len(kept) - 1, args.max_frames).round().astype(int)))
        rejected.mkdir(parents=True, exist_ok=True)
        for i, f in enumerate(kept):
            if i not in keep_idx:
                shutil.move(str(f), rejected / f.name)
        kept = [f for i, f in enumerate(kept) if i in keep_idx]

    print(f"frames kept: {len(kept)}  rejected: {len(files) - len(kept)} "
          f"(-> {rejected})")
    if len(kept) < 5:
        sys.exit(f"FAIL: only {len(kept)} usable frames - capture is too "
                 f"blurred/repetitive for reconstruction")


if __name__ == "__main__":
    main()
