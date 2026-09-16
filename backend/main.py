from fastapi import FastAPI, UploadFile, File
from fastapi.responses import FileResponse
from pathlib import Path
import shutil
import uuid

app = FastAPI(title="3D Reconstruction API")

BASE = Path("/home/hammad/FYP")
UPLOADS = BASE / "backend" / "uploads"
MODEL = BASE / "android_model" / "model.glb"
UPLOADS.mkdir(parents=True, exist_ok=True)

@app.get("/")
def root():
    return {"status": "running", "service": "3D Reconstruction API"}

@app.get("/model")
def get_model():
    return FileResponse(MODEL, media_type="model/gltf-binary", filename="model.glb")

@app.post("/upload")
async def upload_video(file: UploadFile = File(...)):
    job_id = str(uuid.uuid4())
    path = UPLOADS / f"{job_id}_{file.filename}"
    with path.open("wb") as buffer:
        shutil.copyfileobj(file.file, buffer)
    return {
        "status": "uploaded",
        "job_id": job_id,
        "filename": file.filename
    }
