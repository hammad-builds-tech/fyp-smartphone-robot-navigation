#!/bin/bash

# FYP Backend Startup Script

set -e

cd /home/hammad/FYP

echo "======================================="
echo "FYP Backend Server"
echo "======================================="
echo

# Check if uvicorn is available
if ! command -v uvicorn &> /dev/null; then
    echo "ERROR: uvicorn not found"
    echo "Install with: pip install uvicorn"
    exit 1
fi

# Check if MiDaS directory exists
if [ ! -d "MiDaS" ]; then
    echo "ERROR: MiDaS directory not found"
    exit 1
fi

# Check if weights exist
if [ ! -f "MiDaS/weights/dpt_hybrid_384.pt" ]; then
    echo "ERROR: MiDaS weights not found at MiDaS/weights/dpt_hybrid_384.pt"
    exit 1
fi

echo "Starting FastAPI backend on http://0.0.0.0:8000"
echo "MiDaS will load on first request..."
echo

# Start the backend
PYTHONPATH=/home/hammad/FYP:$PYTHONPATH \
uvicorn backend.api.main:app \
    --host 0.0.0.0 \
    --port 8000 \
    --reload
