#!/usr/bin/env python3
"""Record a live phone-camera clip to a (default 30 s) recording.

Phone video -> SAVED MP4 -> phone force-stopped (disconnected) -> offline
processing only. Data is never deleted: the writer is released BEFORE the
disconnect step, so even an abort keeps everything recorded up to that point.

Sequence:
  1. wait for the live stream (frames_received advancing, max 300 s)
  2. 15 s get-ready countdown (user sweeps the phone around the room)
  3. record exactly 30 s wall-clock -> MP4 + raw unique frames
  4. release writer, force-stop com.fyp.reconstruction via adb
  5. verify frames_received frozen (phone truly disconnected)
  6. motion self-check on the saved video (rejects a static/failed take)
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
STAMP = time.strftime("%Y%m%d_%H%M%S")
OUT_DIR = os.path.expanduser("~/FYP/realroom/fresh30_capture")
VIDEO_PATH = os.path.join(OUT_DIR, f"fresh30_{STAMP}.mp4")
# recording length: arg 1 (seconds) or FYP_RECORD_SECONDS env, default 30 
DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else float(os.environ.get("FYP_RECORD_SECONDS", 30.0))
FRAME_DIR = os.path.join(OUT_DIR, "raw_frames")
LATEST_JPG = os.path.expanduser("~/FYP/backend/uploads/latest.jpg")
PREROLL = 15
WAIT_MAX = 300
APP_PKG = "com.fyp.reconstruction"
POLL_FPS = 8.0


def health():
    try:
        with urllib.request.urlopen(BACKEND + "/health", timeout=8) as r:
            return json.loads(r.read().decode())
    except Exception:
        return None


def poll_latest_frame():
    """Make the backend persist its newest phone frame, then read the file."""
    try:
        with urllib.request.urlopen(BACKEND + "/latest-frame", timeout=8) as r:
            meta = json.loads(r.read().decode())
        buf = np.fromfile(LATEST_JPG, dtype=np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        return meta, img
    except Exception:
        return None, None


def force_stop_app():
    try:
        subprocess.run(["adb", "shell", "am", "force-stop", APP_PKG],
                       capture_output=True, timeout=20)
        return True
    except Exception as e:
        print(f"adb force-stop failed: {e}", flush=True)
        return False


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    os.makedirs(FRAME_DIR, exist_ok=True)

    # ---- 1. wait for the live phone stream -------------------------------
    print("== waiting for the phone to stream (open the app now; "
          f"max {WAIT_MAX}s) ==", flush=True)
    h0, t0 = None, time.time()
    while time.time() - t0 < WAIT_MAX:
        h0 = health()
        if h0 and h0.get("frames_received", 0) > 0 and h0.get("depth_available"):
            break
        time.sleep(3)
    if not (h0 and h0.get("frames_received", 0) > 0):
        print("FAIL: phone never streamed", flush=True)
        sys.exit(1)
    start_recv = h0["frames_received"]
    print(f"STREAM LIVE: frames_received={start_recv} "
          f"depth={h0.get('depth_shape')}", flush=True)

    # ---- 2. get-ready countdown ------------------------------------------
    print(f"== GET READY: SWEEP THE PHONE SLOWLY AROUND THE ROOM NOW =="
          f"  (recording starts in {PREROLL}s)", flush=True)
    time.sleep(PREROLL)

    # ---- 3. record exactly DURATION seconds -------------------------------
    meta, frame = None, None
    t0 = time.time()
    while frame is None and time.time() - t0 < 20:
        meta, frame = poll_latest_frame()
        if frame is None:
            time.sleep(0.2)
    if frame is None:
        print("FAIL: no frame readable via /latest-frame", flush=True)
        sys.exit(1)
    hgt, wid = frame.shape[:2]
    print(f"frame size: {wid}x{hgt}", flush=True)

    writer = cv2.VideoWriter(VIDEO_PATH, cv2.VideoWriter_fourcc(*"mp4v"),
                             POLL_FPS, (wid, hgt))
    if not writer.isOpened():
        print("FAIL: VideoWriter not opened", flush=True)
        sys.exit(1)

    print(f"== RECORDING {DURATION:.0f} s -> {VIDEO_PATH} ==", flush=True)
    n_saved = n_unique = 0
    last_hash = None
    max_frame_id = start_recv
    rec_start = time.time()
    try:
        while True:
            now = time.time()
            remain = DURATION - (now - rec_start)
            if remain <= 0:
                break
            meta, f = poll_latest_frame()
            if f is None:
                time.sleep(0.03)
                continue
            if meta:
                max_frame_id = max(max_frame_id, meta.get("frame", -1))
            if f.shape[:2] != (hgt, wid):
                f = cv2.resize(f, (wid, hgt))
            hsh = hashlib.md5(f.tobytes()).hexdigest()
            writer.write(f)
            n_saved += 1
            if hsh != last_hash:
                n_unique += 1
                last_hash = hsh
                cv2.imwrite(os.path.join(FRAME_DIR, f"raw_{n_saved:05d}.jpg"), f)
            time.sleep(max(0.0, 1.0 / POLL_FPS - 0.0))
            if int(remain) in (20, 10, 5, 3, 1):
                print(f"   ... {int(remain)} s left", flush=True)
                time.sleep(0.5)  # avoid printing the same countdown twice
    finally:
        # writer released FIRST: everything recorded so far is safe on disk
        writer.release()
        dur = time.time() - rec_start
        size_mb = os.path.getsize(VIDEO_PATH) / 1e6 if os.path.exists(VIDEO_PATH) else 0
        print(f"RECORDER STOPPED at {dur:.1f}s: {n_saved} frames written, "
              f"{n_unique} unique, {size_mb:.1f} MB", flush=True)

    # ---- 4. auto-disconnect the phone -------------------------------------
    print("== 30 s reached: disconnecting phone (force-stop app) ==", flush=True)
    ok_stop = force_stop_app()
    time.sleep(4)

    # ---- 5. verify disconnect ---------------------------------------------
    h2 = health()
    end_recv = h2.get("frames_received") if h2 else -1
    print(f"backend frame ids: start={start_recv} last_seen={max_frame_id} "
          f"at_stop={end_recv}", flush=True)
    time.sleep(8)
    h3 = health()
    after = h3.get("frames_received") if h3 else -1
    disconnected = (h3 is not None and after == end_recv)
    print(f"post-disconnect frames_received (after 8s wait): {after} -> "
          + ("PHONE DISCONNECTED" if disconnected else "WARNING: still streaming!"),
          flush=True)

    # ---- 6. motion self-check on the saved video --------------------------
    cap = cv2.VideoCapture(VIDEO_PATH)
    prev, moved, tot = None, 0, 0
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

    print(f"\nDATASET VIDEO: {VIDEO_PATH}", flush=True)
    print("NEXT: run the frozen pipeline on this file.", flush=True)
    good = disconnected and n_saved >= 60 and moved >= 20
    sys.exit(0 if good else 2)


if __name__ == "__main__":
    main()
