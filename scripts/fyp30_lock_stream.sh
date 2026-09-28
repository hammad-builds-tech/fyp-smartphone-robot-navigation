#!/bin/bash
# Lock the FYP backend video stream (offline enforcement) and restart backend.
# Pattern lives in this file so pkill cannot self-match the invoking shell.
set -u
pkill -f 'uvicorn backend[.]api[.]main' || true
sleep 3
FYP_VIDEO_STREAM_LOCK=1 setsid nohup bash /home/hammad/FYP/run_backend.sh >> /tmp/fyp_backend.log 2>&1 < /dev/null &
disown
for i in $(seq 1 18); do
  H=$(curl -s -m 2 http://127.0.0.1:8000/health)
  if [ -n "$H" ]; then echo "LOCKED: $H"; exit 0; fi
  sleep 5
done
echo "ERROR: backend did not come up locked"; tail -5 /tmp/fyp_backend.log; exit 1
