from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
import cv2
import numpy as np
from pathlib import Path
import asyncio
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


latest_frame: Optional[np.ndarray] = None
latest_depth: Optional[np.ndarray] = None
frame_count = 0
pending_frame: Optional[np.ndarray] = None
processing_lock = threading.Lock()
inference_active = False


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
            
            # Save latest depth as normalized PNG
            depth_normalized = cv2.normalize(
                depth,
                None,
                0,
                255,
                cv2.NORM_MINMAX
            ).astype(np.uint8)

            cv2.imwrite(
                str(DEPTH_DIR / "latest_depth.png"),
                depth_normalized
            )
        else:
            # No frame to process, sleep briefly
            threading.Event().wait(0.05)


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
    
    return {
        "status": "ok",
        "frames_received": frame_count,
        "midas": "loaded",
        "inference_active": active
    }


# =========================
# LIVE VIDEO
# =========================

@app.websocket("/ws/video")
async def video_websocket(websocket: WebSocket):

    global latest_frame
    global frame_count
    global pending_frame

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

    if latest_depth is None:

        return JSONResponse(
            status_code=404,
            content={
                "message": "No depth generated yet"
            }
        )

    return {

        "status": "ok",

        "shape": list(latest_depth.shape),

        "min": float(latest_depth.min()),

        "max": float(latest_depth.max()),

        "file": str(
            DEPTH_DIR / "latest_depth.png"
        )

    }


@app.get("/latest-depth-image")
def get_latest_depth_image():

    depth_file = DEPTH_DIR / "latest_depth.png"

    if latest_depth is None or not depth_file.exists():

        return JSONResponse(
            status_code=404,
            content={
                "message": "No depth image available yet"
            }
        )

    from fastapi.responses import FileResponse

    return FileResponse(
        str(depth_file),
        media_type="image/png",
        headers={
            "Cache-Control": "no-store",
            "X-FYP-Depth-Sequence": str(frame_count),
        },
    )
