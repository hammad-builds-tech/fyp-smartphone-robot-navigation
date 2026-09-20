"""Test client: acts like the Android phone.

Connects to the backend WebSocket, streams real JPEG frames, then verifies
that MiDaS depth becomes available via the HTTP endpoints (exactly what the
ROS depth bridge consumes).
"""

import asyncio
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import requests
import websockets

args = [a for a in sys.argv[1:] if not a.startswith("--")]
LOOP_SECONDS = 0
for a in sys.argv[1:]:
    if a.startswith("--loop"):
        _, _, val = a.partition("=")
        LOOP_SECONDS = float(val) if val else 3600

BACKEND = args[0] if args else "ws://127.0.0.1:8000/ws/video"
HTTP_BASE = BACKEND.replace("ws://", "http://").rsplit("/ws/", 1)[0]
FRAME_DIR = Path.home() / "FYP" / "video_frames"
N_FRAMES = 3


def pick_frames():
    frames = []
    for path in sorted(FRAME_DIR.glob("*.jpg"))[:N_FRAMES]:
        img = cv2.imread(str(path))
        if img is None:
            continue
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            frames.append(buf.tobytes())
    return frames


async def main():
    frames = pick_frames()
    if not frames:
        print("FAIL: no test frames found in", FRAME_DIR)
        return 1

async def stream_once(frames, loop_seconds):
    async with websockets.connect(BACKEND, max_size=10 * 1024 * 1024) as ws:
        deadline = time.time() + loop_seconds
        i = 0
        while True:
            jpeg = frames[i % len(frames)]
            i += 1
            await ws.send(jpeg)
            reply = await asyncio.wait_for(ws.recv(), timeout=15)
            if i <= N_FRAMES or i % 50 == 0:
                print(f"frame {i}: {len(jpeg)} B sent -> {reply[:110]}")
            if loop_seconds and time.time() >= deadline:
                break
            if not loop_seconds:
                break


async def main():
    frames = pick_frames()
    if not frames:
        print("FAIL: no test frames found in", FRAME_DIR)
        return 1

    print(f"Connecting to {BACKEND} ...")
    # Reconnect forever on transient failures, exactly like the Android app.
    while True:
        try:
            await stream_once(frames, LOOP_SECONDS)
            break  # one-shot mode finished normally
        except (OSError, websockets.WebSocketException) as exc:
            print(f"connection lost ({type(exc).__name__}); retrying in 3s")
            await asyncio.sleep(3)

    # Give the background MiDaS worker time to finish (CPU: ~1-2 s/frame).
    wait = 60 if not LOOP_SECONDS else 5
    deadline = time.time() + wait
    depth = None
    while time.time() < deadline:
        r = requests.get(HTTP_BASE + "/latest-depth", timeout=5)
        if r.status_code == 200:
            depth = r.json()
            if depth.get("depth_available", True):
                break
        time.sleep(2)

    if depth is None:
        print("FAIL: /latest-depth never returned 200")
        return 1
    print("/latest-depth:", depth)

    # Check the PNG the ROS bridge polls (must include the sequence header).
    r = requests.get(HTTP_BASE + "/latest-depth-image", timeout=5)
    seq = r.headers.get("X-FYP-Depth-Sequence")
    arr = np.frombuffer(r.content, dtype=np.uint8)
    png = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
    print(
        f"/latest-depth-image: HTTP {r.status_code}, {len(r.content)} B, "
        f"sequence={seq}, png_shape={None if png is None else png.shape}"
    )
    if r.status_code != 200 or png is None or seq is None:
        print("FAIL: depth image endpoint unusable for the ROS bridge")
        return 1

    print("BACKEND PIPELINE TEST PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
