from fastapi import FastAPI, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
import cv2
import numpy as np
from pathlib import Path
import asyncio
import os
import socket
import threading
from typing import Optional

from backend.depth.midas_processor import MiDaSProcessor


app = FastAPI(
    title="FYP 3D Indoor Mapping Backend",
    version="1.0.0"
)

UPLOAD_DIR = Path.home() / "FYP" / "backend" / "uploads"
DEPTH_DIR = Path.home() / "FYP" / "backend" / "depth" / "output"

UPLOAD_DIR.mkdir(parents=True, exist_ok=True)
DEPTH_DIR.mkdir(parents=True, exist_ok=True)

API_PORT = int(os.environ.get("FYP_BACKEND_PORT", "8000"))
DISCOVERY_PORT = int(os.environ.get("FYP_DISCOVERY_PORT", "51315"))

latest_frame: Optional[np.ndarray] = None
latest_depth: Optional[np.ndarray] = None
latest_depth_png: Optional[bytes] = None  # encoded PNG served by /latest-depth-image
frame_count = 0
pending_frame: Optional[np.ndarray] = None
processing_lock = threading.Lock()
inference_active = False

# Hard offline switch: when True, ALL new video websocket connections are
# rejected with policy code 1008. Used to enforce the 30-second capture
# window so all post-processing runs purely OFFLINE on saved data.
video_stream_locked = os.environ.get("FYP_VIDEO_STREAM_LOCK", "0") == "1"


# =========================
# LAN AUTO-DISCOVERY
#
# The Android app can find this backend without any hard-coded IP.
# It broadcasts "FYP_DISCOVER_REQ" on the LAN; we answer with our
# address so the phone can connect on any Wi-Fi / hotspot network.
# =========================

def _lan_ip() -> str:
    """Best-effort local LAN IP (no packets are actually sent)."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        sock.close()


def discovery_responder():
    """Answer UDP discovery probes so the phone app needs no manual IP."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind(("", DISCOVERY_PORT))
    except OSError as exc:
        print(f"UDP discovery disabled (port {DISCOVERY_PORT} busy): {exc}")
        return

    while True:
        try:
            data, addr = sock.recvfrom(1024)
        except OSError:
            continue
        if data.strip() == b"FYP_DISCOVER_REQ":
            reply = f"FYP_BACKEND|{_lan_ip()}|{API_PORT}"
            try:
                sock.sendto(reply.encode("utf-8"), addr)
                print(f"Discovery probe answered for {addr[0]}")
            except OSError:
                pass


threading.Thread(target=discovery_responder, daemon=True).start()


# =========================
# LOAD MiDaS
# =========================

print("Initializing MiDaS...")

midas = MiDaSProcessor()

print("MiDaS ready.")


# =========================
# Background Inference Worker
# =========================

def inference_worker():
    """Background thread that processes frames from the buffer."""
    global latest_depth, pending_frame, inference_active
    
    while True:
        frame_to_process = None
        
        with processing_lock:
            if pending_frame is not None:
                frame_to_process = pending_frame
                pending_frame = None
                inference_active = True
        
        if frame_to_process is not None:
            # Run MiDaS inference (blocks this background thread only)
            depth = midas.predict(frame_to_process)
            
            # Save latest depth
            with processing_lock:
                latest_depth = depth
                inference_active = False

            # Persist normalized PNG for /latest-depth-image consumers
            # (Android viewer + the ROS depth bridge).
            _write_depth_png(depth)
        else:
            # No frame to process, sleep briefly
            threading.Event().wait(0.05)


def _write_depth_png(depth: np.ndarray) -> None:
    """Persist the latest depth as a normalized PNG for /latest-depth-image.

    The file is encoded in memory and moved into place atomically (os.replace)
    so a concurrent FileResponse reader can never observe an empty or
    half-written PNG while the worker rewrites it.
    """
    depth_normalized = cv2.normalize(
        depth,
        None,
        0,
        255,
        cv2.NORM_MINMAX,
    ).astype(np.uint8)
    ok, buf = cv2.imencode(".png", depth_normalized)
    if not ok:
        return
    data = buf.tobytes()

    # Keep the encoded PNG in memory: /latest-depth-image serves these exact
    # bytes, so consumers never see a half-written file. The on-disk copy is
    # only a convenience for human inspection / the Android viewer.
    global latest_depth_png
    latest_depth_png = data
    final_path = DEPTH_DIR / "latest_depth.png"
    tmp_path = DEPTH_DIR / ".latest_depth.png.tmp"
    tmp_path.write_bytes(data)
    os.replace(str(tmp_path), str(final_path))


# Start background inference thread
inference_thread = threading.Thread(target=inference_worker, daemon=True)
inference_thread.start()


# =========================
# ROOT
# =========================

@app.get("/")
def root():
    return {
        "project": "Smartphone-Based Real-Time 3D Indoor Mapping",
        "status": "running",
        "midas": "loaded"
    }


# =========================
# HEALTH
# =========================

@app.get("/health")
def health():
    with processing_lock:
        active = inference_active
        depth = latest_depth

    return {
        "status": "ok",
        "frames_received": frame_count,
        "midas": "loaded",
        "device": "cuda" if midas.device.type == "cuda" else "cpu",
        "depth_available": depth is not None,
        "depth_shape": list(depth.shape) if depth is not None else None,
        "inference_active": active,
        "discovery_port": DISCOVERY_PORT,
        "video_stream_locked": video_stream_locked,
    }


# =========================
# LIVE VIDEO
# =========================

@app.websocket("/ws/video")
async def video_websocket(websocket: WebSocket):

    global latest_frame
    global frame_count
    global pending_frame

    if video_stream_locked:
        await websocket.accept()
        await websocket.close(code=1008, reason="stream locked: capture window closed")
        print("PHONE REJECTED (stream locked - offline enforcement)")
        return

    await websocket.accept()

    print("PHONE CONNECTED")

    try:

        while True:

            data = await websocket.receive_bytes()

            # JPEG bytes → OpenCV image
            image_array = np.frombuffer(
                data,
                dtype=np.uint8
            )

            frame = cv2.imdecode(
                image_array,
                cv2.IMREAD_COLOR
            )

            if frame is None:

                await websocket.send_json({
                    "status": "error",
                    "message": "Invalid JPEG frame"
                })

                continue

            frame_count += 1

            latest_frame = frame

            # Submit frame for background processing (non-blocking)
            # Only keep the latest frame - drop old ones
            with processing_lock:
                pending_frame = frame.copy()

            # Save every 10th RGB frame
            if frame_count % 10 == 0:

                rgb_file = (
                    UPLOAD_DIR /
                    f"frame_{frame_count:06d}.jpg"
                )

                cv2.imwrite(
                    str(rgb_file),
                    frame
                )

            # Get current depth status
            with processing_lock:
                depth_available = latest_depth is not None
                active = inference_active

            print(
                f"Frame {frame_count} | "
                f"RGB {frame.shape[1]}x{frame.shape[0]} | "
                f"Depth {'available' if depth_available else 'processing...'} | "
                f"Inference {'active' if active else 'idle'}"
            )

            await websocket.send_json({

                "status": "ok",

                "frame": frame_count,

                "width": frame.shape[1],

                "height": frame.shape[0],

                "depth_available": depth_available,

                "inference_active": active

            })

    except WebSocketDisconnect:

        print("PHONE DISCONNECTED")


# =========================
# LATEST FRAME
# =========================

@app.get("/latest-frame")
def get_latest_frame():

    if latest_frame is None:

        return JSONResponse(
            status_code=404,
            content={
                "message": "No frame received yet"
            }
        )

    filename = UPLOAD_DIR / "latest.jpg"

    cv2.imwrite(
        str(filename),
        latest_frame
    )

    return {
        "status": "ok",
        "file": str(filename),
        "frame": frame_count
    }


# =========================
# LATEST DEPTH
# =========================

@app.get("/latest-depth")
def get_latest_depth():

    with processing_lock:
        depth = latest_depth

    if depth is None:

        return JSONResponse(
            status_code=404,
            content={
                "message": "No depth generated yet"
            }
        )

    return {

        "status": "ok",

        "shape": list(depth.shape),

        "min": float(depth.min()),

        "max": float(depth.max()),

        "file": str(
            DEPTH_DIR / "latest_depth.png"
        )

    }


@app.get("/latest-depth-image")
def get_latest_depth_image():

    global latest_depth_png

    with processing_lock:
        depth = latest_depth
        png_bytes = latest_depth_png

    # Server restart recovery: the last phone-derived MiDaS depth PNG is
    # persisted on disk by the inference worker, so a backend restart (e.g.
    # uvicorn --reload) must not 404 and drop the /scan compatibility layer.
    if png_bytes is None:
        persisted = DEPTH_DIR / "latest_depth.png"
        if persisted.exists() and persisted.stat().st_size > 0:
            png_bytes = persisted.read_bytes()
            latest_depth_png = png_bytes

    # png_bytes alone is sufficient: after a restart the last phone-derived
    # depth is restored from disk even though no inference has run yet.
    if png_bytes is None:
        return JSONResponse(
            status_code=404,
            content={
                "message": "No depth image available yet"
            }
        )

    # Serve the exact encoded bytes produced by the worker (immutable
    # snapshot) so the response length always matches the body — a reader
    # can never hit an IncompleteRead from a concurrently rewritten file.
    return Response(
        content=png_bytes,
        media_type="image/png",
        headers={
            "Cache-Control": "no-store",
            "X-FYP-Depth-Sequence": str(frame_count),
        },
    )
