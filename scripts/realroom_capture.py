#!/usr/bin/env python
"""Capture a real-room scan from the LIVE Android phone stream.

Snapshots paired RGB frames (/latest-frame) and MiDaS depth maps
(/latest-depth-image) from the running FastAPI backend while the phone is
physically swept around the room. The output layout mirrors the project's
existing offline reconstruction inputs:

  realroom/capture/img/frame_XXXX.jpg        (COLMAP input images)
  realroom/capture/depth/frame_XXXX.png      (live MiDaS depth, cross-checked)
  realroom/capture/capture_log.csv           (frame index, depth sequence)

Usage:
    ~/miniconda3/envs/fyp/bin/python scripts/realroom_capture.py \
        [--frames 25] [--interval 3.0] [--out ~/FYP/realroom/capture]
"""
import argparse
import csv
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import requests

BACKEND = "http://127.0.0.1:8000"


def fetch_png(client, url):
    r = client.get(url, timeout=10)
    if r.status_code != 200 or not r.content:
        return None, None
    seq = r.headers.get("X-FYP-Depth-Sequence", "")
    buf = np.frombuffer(r.content, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
    return img, seq


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames", type=int, default=25)
    ap.add_argument("--interval", type=float, default=3.0)
    ap.add_argument("--out", default=str(Path.home() / "FYP/realroom/capture"))
    args = ap.parse_args()

    out = Path(args.out).expanduser()
    img_dir = out / "img"
    dep_dir = out / "depth"
    img_dir.mkdir(parents=True, exist_ok=True)
    dep_dir.mkdir(parents=True, exist_ok=True)

    client = requests.Session()
    h = client.get(f"{BACKEND}/health", timeout=5).json()
    if h.get("depth_available") is not True:
        print("Backend has no live depth. Start the Android stream first.", file=sys.stderr)
        sys.exit(1)
    print(f"Backend OK: frames_received={h['frames_received']} shape={h['depth_shape']}")
    print(f"== SWEEP THE PHONE SLOWLY AROUND THE ROOM NOW ({args.frames} snapshots x {args.interval}s) ==")

    log_rows = []
    for i in range(1, args.frames + 1):
        # RGB frame
        fr = client.get(f"{BACKEND}/latest-frame", timeout=10)
        rgb = None
        if fr.status_code == 200:
            rgb = cv2.imdecode(np.frombuffer(fr.content, np.uint8), cv2.IMREAD_COLOR)
        if rgb is None:
            j = fr.json() if fr.headers.get("content-type", "").startswith("application/json") else {}
            lp = j.get("file")
            if lp and Path(lp).exists():
                rgb = cv2.imread(lp)
        # Depth image
        dep, seq = fetch_png(client, f"{BACKEND}/latest-depth-image")

        if rgb is None or dep is None:
            print(f"[{i:02d}] MISS (rgb={rgb is not None}, depth={dep is not None}) — retrying")
            time.sleep(1.5)
            continue

        name = f"frame_{i:04d}"
        cv2.imwrite(str(img_dir / f"{name}.jpg"), rgb)
        cv2.imwrite(str(dep_dir / f"{name}.png"), dep)
        log_rows.append([name, seq, h["depth_shape"][0], h["depth_shape"][1]])
        print(f"[{i:02d}] {name}.jpg {rgb.shape[1]}x{rgb.shape[0]}  depth_seq={seq}")

        if i < args.frames:
            time.sleep(args.interval)

    with open(out / "capture_log.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "depth_seq", "depth_h", "depth_w"])
        w.writerows(log_rows)

    print(f"Captured {len(log_rows)} paired frames -> {out}")
    if len(log_rows) < 8:
        print("Too few frames for reconstruction — re-run with the phone sweeping.", file=sys.stderr)
        sys.exit(2)


if __name__ == "__main__":
    main()
