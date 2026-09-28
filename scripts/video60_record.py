#!/usr/bin/env python3
"""Record ~60s of live phone-camera frames from the FastAPI backend,
save an actual video file (MP4), then force-stop the phone app (disconnect).

Recorded-video test per user directive:
  phone video -> SAVED video -> phone disconnected -> offline processing only.

Frame acquisition follows the proven capture60 pattern:
  GET /latest-frame  ->  backend persists ~/FYP/backend/uploads/latest.jpg
  -> recorder reads that file (fresh every poll).
"""
import hashlib
import json
import os
import subprocess
import sys
import time
import urllib.request

import cv2
import numpy as np

BACKEND = "http://127.0.0.1:8000"
OUT_DIR = os.path.expanduser("~/FYP/realroom/video60")
VIDEO_PATH = os.path.join(OUT_DIR, "video60.mp4")
FRAME_DIR = os.path.join(OUT_DIR, "raw_frames")
LATEST_JPG = os.path.expanduser("~/FYP/backend/uploads/latest.jpg")
DURATION = 62.0  # ~1 minute of recording
APP_PKG = "com.fyp.reconstruction"


def health():
    try:
        with urllib.request.urlopen(BACKEND + "/health", timeout=8) as r:
            return json.loads(r.read().decode())
    except Exception:
        return None


def poll_latest_frame():
    """Trigger the backend to persist its newest phone frame, then read it."""
    try:
        with urllib.request.urlopen(BACKEND + "/latest-frame", timeout=8) as r:
            meta = json.loads(r.read().decode())
        buf = np.fromfile(LATEST_JPG, dtype=np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return meta, img
    except Exception:
        return None, None


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(FRAME_DIR, exist_ok=True)

    h = health()
    print("health:", h, flush=True)
    if not h:
        print("FAIL: backend unreachable", flush=True)
        sys.exit(1)

    start_recv = h.get("frames_received")
    print(f"frames_received at start: {start_recv}", flush=True)

    # Grab first frame to learn dimensions
    meta, frame = None, None
    t0 = time.time()
    while frame is None and time.time() - t0 < 20:
        meta, frame = poll_latest_frame()
        if frame is None:
            time.sleep(0.25)
    if frame is None:
        print("FAIL: no frame via /latest-frame", flush=True)
        sys.exit(1)

    hgt, wid = frame.shape[:2]
    print(f"frame size: {wid}x{hgt}", flush=True)

    fps = 8.0  # poll rate; playback fps for the saved video
    writer = cv2.VideoWriter(
        VIDEO_PATH,
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (wid, hgt),
    )
    if not writer.isOpened():
        print("FAIL: VideoWriter not opened", flush=True)
        sys.exit(1)

    n_saved = 0
    n_unique = 0
    last_hash = None
    last_frame_id = meta.get("frame") if meta else -1
    max_frame_id = last_frame_id
    rec_start = time.time()
    while True:
        now = time.time()
        if now - rec_start >= DURATION:
            break
        meta, f = poll_latest_frame()
        if f is None:
            time.sleep(0.05)
            continue
        if meta:
            fid = meta.get("frame", -1)
            last_frame_id = fid
            max_frame_id = max(max_frame_id, fid)
        if f.shape[:2] != (hgt, wid):
            f = cv2.resize(f, (wid, hgt))
        hsh = hashlib.md5(f.tobytes()).hexdigest()
        writer.write(f)
        n_saved += 1
        if hsh != last_hash:
            n_unique += 1
            last_hash = hsh
            cv2.imwrite(os.path.join(FRAME_DIR, f"raw_{n_saved:05d}.jpg"), f)
        time.sleep(1.0 / fps)

    writer.release()
    dur = time.time() - rec_start

    # Force-stop the phone app => disconnect phone from backend
    print("force-stopping phone app ...", flush=True)
    subprocess.run(["adb", "shell", "am", "force-stop", APP_PKG],
                   capture_output=True, timeout=20)
    time.sleep(4)

    h2 = health()
    end_recv = h2.get("frames_received") if h2 else -1
    print(f"frames_received at end: {end_recv}", flush=True)
    print(f"backend frame ids seen during recording: "
          f"{start_recv} -> {max_frame_id}", flush=True)
    print(f"RECORD DONE: {n_saved} frames written, {n_unique} unique, "
          f"{dur:.1f}s, video={VIDEO_PATH}", flush=True)
    print(f"video size: {os.path.getsize(VIDEO_PATH)/1e6:.1f} MB", flush=True)

    # 10 s grace: confirm no new frames arrive => truly disconnected
    time.sleep(10)
    h3 = health()
    after = h3.get("frames_received") if h3 else -1
    print(f"post-disconnect frames_received (after 10s wait): {after}",
          flush=True)
    ok_dc = (h3 is not None and after == end_recv)
    print("DISCONNECTED" if ok_dc else "WARNING: frames still arriving!",
          flush=True)

    # Motion self-check on the saved video (must have real motion)
    cap = cv2.VideoCapture(VIDEO_PATH)
    prev = None
    moved = 0
    tot = 0
    while True:
        ok, f = cap.read()
        if not ok:
            break
        g = cv2.resize(f, (80, 60)).astype(int)
        if prev is not None:
            tot += 1
            if np.abs(g - prev).mean() > 0.5:
                moved += 1
        prev = g
    cap.release()
    print(f"VIDEO MOTION CHECK: {moved}/{tot} frames with motion", flush=True)
    if moved < 10:
        print("WARNING: video appears mostly STATIC", flush=True)
    sys.exit(0 if (ok_dc and moved >= 10) else 2)


if __name__ == "__main__":
    main()
