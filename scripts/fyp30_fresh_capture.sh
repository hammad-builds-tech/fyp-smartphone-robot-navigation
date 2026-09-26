#!/bin/bash
# Fresh 30-second recorded-video capture: phone -> backend -> paired RGB/depth.
# - Waits for the live phone stream (health: depth_available=true)
# - Captures paired frames for EXACTLY 30 seconds (timeout-enforced)
# - Verifies the stream stopped (depth sequence frozen => phone disconnected)
# - Prints dataset summary. From here on, everything downstream is OFFLINE.
set -u
FYP="$HOME/FYP"
OUT="${1:-$FYP/realroom/capture_fresh}"
PY="$HOME/miniconda3/envs/fyp/bin/python"

echo "== waiting for phone stream (connect the phone app now; max 300 s) =="
ok=""
for i in $(seq 1 60); do
  H=$(curl -s -m 3 http://127.0.0.1:8000/health)
  case "$H" in
    *'"depth_available":true'*) ok=1; echo "stream live: $H"; break ;;
  esac
  sleep 5
done
if [ -z "${ok:-}" ]; then echo "TIMEOUT: phone never streamed"; exit 1; fi

echo "== GET READY: START SWEEPING THE PHONE SLOWLY AROUND THE ROOM NOW =="
echo "   (30-second recording window begins in 15 seconds)"
for c in 15 10 5; do echo "   ... $c s"; sleep 5; done

echo "== capturing for EXACTLY 30 s -> $OUT =="
mkdir -p "$OUT"
# realroom_capture.py loops until Ctrl-C equivalent; enforce hard 30 s cutoff
timeout --signal=INT 30 "$PY" "$FYP/scripts/realroom_capture.py" \
    --frames 60 --interval 2.5 --out "$OUT"
rc=$?
echo "capture loop ended (rc=$rc) — 30 s window enforced"

sleep 2
echo "== disconnect check =="
S1=$(curl -s -m 3 -D - -o /dev/null http://127.0.0.1:8000/latest-depth-image | grep -i x-fyp-depth-sequence | tr -d '\r')
sleep 4
S2=$(curl -s -m 3 -D - -o /dev/null http://127.0.0.1:8000/latest-depth-image | grep -i x-fyp-depth-sequence | tr -d '\r')
echo "depth seq before=$S1"
echo "depth seq after =$S2"
[ -n "$S1" ] && [ "$S1" = "$S2" ] && echo "PHONE DISCONNECTED (sequence frozen)" || echo "WARNING: sequence still advancing"

echo "== dataset summary =="
ls "$OUT/img" 2>/dev/null | wc -l
ls "$OUT/depth" 2>/dev/null | wc -l
head -20 "$OUT/capture_log.csv" 2>/dev/null
