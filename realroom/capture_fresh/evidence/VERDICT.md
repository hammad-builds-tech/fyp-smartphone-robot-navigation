# 30-Second Recorded-Video Test — FINAL VERDICT: PASS (9/9 gates)

## Chain of provenance (all from the single 30-s capture, 2026-09-27)
1. CAPTURE: phone -> backend, exactly 30 s (timeout-enforced), 12 RGB frames
   (640x480) + 12 MiDaS depth PNGs; depth_seq 181->463 (continuous stream).
   Backend then LOCKED (health: video_stream_locked:true) — offline thereafter.
2. RECONSTRUCTION: MiDaS PFMs -> COLMAP 4.3 (SIFT CPU, sequential+exhaustive):
   12/12 images registered, 1050 pts, reprojection err 0.399 px.
3. FUSION: realroom_fuse.py -> fused_cloud.ply (223,488 pts).
4. MAP: realroom_cloud_to_map.py (voxel+SOR+RANSAC+band 0.15-1.20 m + ray carve)
   -> realroom_fresh.pgm/.yaml (547x521 @ 0.05 m).
5. WORLD: realroom30_world2.py -> realroom_fresh_world.sdf (real components only,
   >=4 cells; 66 kept). No generic floor, no invented boxes.

## Test execution
- Spawn (8.01, 13.73) — captured free space, clearance 0.64 m (goal-picker oracle).
- Goal (0.81, 17.58) — straight line crosses the real captured component 65
  (765 cells). Nav2 plan: 363 poses, 9.11 m vs 8.16 m straight (ratio 1.12).
- Executed at 0.11-0.15 m/s, ~90 s, status 4 SUCCEEDED, final (0.94, 17.52).

## Detour metrics (scripts/fyp30_detour_analysis.py -> detour_metrics.json)
- executed 8.96 m vs straight 8.16 m (ratio 1.098)
- max perpendicular deviation 1.39 m (t=0.19)
- BLOCKER: component 65 — 765 cells (1.9 m^2), bbox (2.21,16.33)-(4.31,12.53) m
  straight line grazes it at 0.03 m (t=0.53-0.69); robot kept >= 0.80 m from it
  (lateral offset 0.83-0.86 m in the graze span); min clearance to ANY obstacle 0.45 m
- PROVENANCE: blocker cells vs replicated fused-cloud band:
  median 0.015 m, p95 0.026 m, max 0.032 m; 3302/28061 band points inside its cells
- RGB PROOF: frame_overlay_0001/0004.png + detour_composite_0001/0004.png
  (recorded photos with executed path (green), plan (yellow), blocker (red))
- Logs: logs/ (backend lock, sim, plan, trajectory, health), this dir.

## Gate results
PASS reached_goal | PASS length_ratio>=1.05 | PASS max_dev>=0.5m
PASS blocker_substantial(>=25 cells) | PASS path_avoids_blocker | PASS offset_at_blocker>=0.5m
PASS provenance_median<=0.15m | PASS provenance_p95<=0.30m | PASS min_clearance_blocker>=0.20m

GUIs during run (Xwayland :0, xwininfo): "Gazebo Sim" 1250x1087, RViz 1450x987.
(Desktop is GNOME Wayland: background screenshot capture is portal-gated; take a
manual PrtScr if a picture of the windows is required in the archive.)
