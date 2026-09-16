from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pathlib import Path

app = FastAPI(
    title="FYP 3D Reconstruction API",
    version="1.0.0"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

BASE_DIR = Path(__file__).resolve().parent.parent
DENSE_DIR = BASE_DIR / "dense"

@app.get("/")
def root():
    return {
        "status": "ok",
        "message": "FYP 3D Reconstruction Backend is running"
    }

@app.get("/models")
def models():
    return {
        "models": [
            p.name for p in DENSE_DIR.glob("*.ply")
        ]
    }

@app.get("/health")
def health():
    return {"status": "healthy"}
