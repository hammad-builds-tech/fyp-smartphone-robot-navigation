#!/bin/bash

# FYP Backend Startup Script

set -e

cd /home/hammad/FYP

echo "======================================="
echo "FYP Backend Server"
echo "======================================="
echo

# Use FYP conda environment
FYP_PYTHON="/home/hammad/miniconda3/envs/fyp/bin/python"
FYP_UVICORN="/home/hammad/miniconda3/envs/fyp/bin/uvicorn"

# Check if FYP environment exists
if [ ! -f "$FYP_PYTHON" ]; then
    echo "ERROR: FYP Python environment not found at $FYP_PYTHON"
    echo "Expected conda environment: ~/miniconda3/envs/fyp"
    exit 1
fi

# Verify Python version
PYTHON_VERSION=$("$FYP_PYTHON" --version 2>&1)
echo "Using: $PYTHON_VERSION"
echo "Environment: fyp"
echo

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
echo "MiDaS will load on startup..."
echo

# Start the backend with FYP environment
PYTHONPATH=/home/hammad/FYP:$PYTHONPATH \
"$FYP_UVICORN" backend.api.main:app \
    --host 0.0.0.0 \
    --port 8000 \
    --reload
