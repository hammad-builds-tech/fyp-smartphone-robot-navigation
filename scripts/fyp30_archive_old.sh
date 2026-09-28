#!/bin/bash
# Archive ALL old test data from previous runs (does NOT touch source code).
set -u
FYP="$HOME/FYP"
ARC="$FYP/realroom/archive_old_20260926"
mkdir -p "$ARC"
cd "$FYP"

moved=0
mv_item () {
  if [ -e "$1" ]; then
    mv "$1" "$ARC/" && echo "archived: $1" && moved=$((moved+1))
  fi
}

# per-capture reconstruction datasets + COLMAP workspaces
for d in realroom/capture realroom/capture30 realroom/capture60 realroom/video60 \
         realroom/colmap realroom/colmap30 realroom/colmap60 realroom/colmap60b; do
  mv_item "$d"
done
# stray maps/worlds in realroom root
for f in realroom/realroom_map.pgm realroom/realroom_map.yaml; do mv_item "$f"; done
# old captures' worlds (kept inside capture dirs above) + any root-level worlds
mv_item video60_world.sdf 2>/dev/null || true

# historical video test data
mv_item video_frames
mv_item colmap_video
mv_item sparse
mv_item sparse_txt
mv_item dense
mv_item database
mv_item images
mv_item data
mv_item MiDaS/input/WhatsApp\ Video\ 2026-09-03\ at\ 4.10.54\ PM.mp4
mv_item MiDaS/input/test.png

# old backend depth output (persisted latest frame from previous sessions)
mv_item backend/depth/output
mkdir -p backend/depth/output

echo "items archived: $moved"
echo "archive at: $ARC"
du -sh "$ARC"
ls "$ARC"
