# Backend Environment Fix

**Date**: September 16, 2026  
**Issue**: Backend fails with `ModuleNotFoundError: No module named 'timm'`  
**Root Cause**: run_backend.sh used system Python instead of FYP conda environment

---

## Problem

The `run_backend.sh` script was using whatever `uvicorn` was found in PATH, which defaulted to the base Miniconda Python 3.14 environment. This environment did not have the required dependencies (torch, timm, opencv) for MiDaS.

### Error
```
ModuleNotFoundError: No module named 'timm'
```

### Root Cause
```bash
# OLD run_backend.sh
uvicorn backend.api.main:app ...  # Uses whatever is in PATH
```

This would find the base environment uvicorn, which runs with Python 3.14, not the FYP environment with Python 3.10.21 and all required packages.

---

## Solution

### Fixed run_backend.sh

Updated script to explicitly use the FYP conda environment:

```bash
FYP_PYTHON="/home/hammad/miniconda3/envs/fyp/bin/python"
FYP_UVICORN="/home/hammad/miniconda3/envs/fyp/bin/uvicorn"

PYTHONPATH=/home/hammad/FYP:$PYTHONPATH \
"$FYP_UVICORN" backend.api.main:app \
    --host 0.0.0.0 \
    --port 8000 \
    --reload
```

### FYP Environment Specifications

**Path**: `~/miniconda3/envs/fyp`

**Python**: 3.10.21

**Key Packages**:
- PyTorch: 2.14.0+cu130
- CUDA available: False (CPU mode)
- fastapi: ✓ Installed
- uvicorn: ✓ Installed
- opencv-python: ✓ Installed (cv2 5.0.0)
- numpy: ✓ Installed (2.2.6)
- timm: ✓ Installed

---

## Testing Performed

### 1. Environment Verification ✅
```bash
/home/hammad/miniconda3/envs/fyp/bin/python --version
# Python 3.10.21
```

### 2. Import Test ✅
```python
from backend.api.main import app, midas
# ✓ MiDaS loaded successfully
# ✓ Device: cpu
```

### 3. Endpoint Tests ✅
```python
from fastapi.testclient import TestClient
client = TestClient(app)

client.get("/")                    # 200 ✓
client.get("/health")              # 200 ✓
client.get("/latest-depth-image")  # 404 ✓ (no frames yet)
```

**Result**: All tests passed with FYP environment

---

## Files Changed

### run_backend.sh ✅ MODIFIED

**Changes**:
1. Added explicit FYP environment paths
2. Added Python version check
3. Updated uvicorn invocation to use FYP environment
4. Added environment validation

**Before**:
```bash
uvicorn backend.api.main:app ...
```

**After**:
```bash
FYP_UVICORN="/home/hammad/miniconda3/envs/fyp/bin/uvicorn"
"$FYP_UVICORN" backend.api.main:app ...
```

---

## Usage

### Start Backend (Correct)
```bash
cd ~/FYP
./run_backend.sh
```

This will now:
1. Verify FYP environment exists
2. Show Python version (3.10.21)
3. Load MiDaS with all dependencies
4. Start FastAPI on port 8000

### Verify Backend
```bash
curl http://localhost:8000/health
# {"status":"ok","frames_received":0,"midas":"loaded","inference_active":false}
```

---

## Architecture Preserved ✅

No changes to:
- Backend code (backend/api/main.py unchanged)
- MiDaS processor (backend/depth/midas_processor.py unchanged)
- ROS bridge (smartphone_depth_bridge unchanged)
- Nav2 integration (unchanged)
- Android WebSocket protocol (unchanged)

Only changed: **Environment selection in run_backend.sh**

---

## Next Steps

With backend now working in FYP environment:

1. ✅ Backend loads successfully
2. ✅ MiDaS initialized on CPU
3. ✅ All endpoints responsive
4. ⏭️ Ready for live Android testing
5. ⏭️ Ready for end-to-end pipeline test

### Live Test Procedure

**Terminal 1**: Start Backend
```bash
cd ~/FYP
./run_backend.sh
```

**Terminal 2**: Start ROS + Gazebo + Nav2
```bash
cd ~/FYP
./run_system.sh
```

**Terminal 3**: Connect Android
- Set WebSocket: `ws://<server-ip>:8000/ws/video`
- Start streaming

**Terminal 4**: Send Navigation Goal
```bash
cd ~/FYP
./send_test_goal.sh
```

Expected: Robot navigates with smartphone depth-based obstacle avoidance

---

## Conclusion

**Issue**: ✅ RESOLVED  
**Backend**: ✅ Working with FYP environment  
**Testing**: ✅ All endpoints verified  
**Ready For**: Live end-to-end demonstration

The backend now correctly uses the FYP conda environment with all required dependencies. MiDaS loads successfully and the complete smartphone → FastAPI → MiDaS → ROS → Nav2 pipeline is ready for testing.
