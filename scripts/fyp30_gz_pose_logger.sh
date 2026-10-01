#!/bin/bash
# Gazebo ground-truth pose logger: appends fyp_robot world x,y at ~2 Hz to
# /tmp/ve_final/gz_pose.csv. Header "t,x,y" with empty t (compose-evidence
# convention). World name fixed: capture_fresh.
export GZ_IP=127.0.0.1
OUT=/tmp/ve_final
mkdir -p "$OUT"
echo "t,x,y" > "$OUT/gz_pose.csv"
while true; do
  SNAP=$(timeout 5 gz topic -e -t /world/capture_fresh/dynamic_pose/info --num 1 2>/dev/null)
  if [ -n "$SNAP" ]; then
    X=$(echo "$SNAP" | awk '/name:/{f=($0 ~ /"fyp_robot"/)} f && /^ *x:/{print $2; exit}')
    Y=$(echo "$SNAP" | awk '/name:/{f=($0 ~ /"fyp_robot"/)} f && /^ *x:/{gx=1} f && gx && /^ *y:/{print $2; exit}')
    if [ -n "$X" ] && [ -n "$Y" ]; then
      echo ",$X,$Y" >> "$OUT/gz_pose.csv"
    fi
  fi
  sleep 0.5
done
