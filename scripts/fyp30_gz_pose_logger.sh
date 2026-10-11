#!/bin/bash
# Gazebo ground-truth pose logger: appends fyp_robot world x,y at ~2 Hz to
# /tmp/ve_final/gz_pose.csv. Header "t,x,y" with empty t (compose-evidence
# convention). World name is DISCOVERED from the running sim (not hardcoded
# so the logger works for any reconstruction world).
export GZ_IP=127.0.0.1
export PATH="/opt/ros/lyrical/opt/gz_tools_vendor/bin:$PATH"
OUT=/tmp/ve_final
mkdir -p "$OUT"
echo "t,x,y" > "$OUT/gz_pose.csv"
# discover the world name from the service list (first /world/*/dynamic_pose)
WORLD=""
for i in $(seq 1 60); do
  WORLD=$(timeout 5 gz service -l 2>/dev/null | grep -m1 -oP '(?<=/world/)[^/]+(?=/dynamic_pose/info)')
  [ -n "$WORLD" ] && break
  sleep 2
done
if [ -z "$WORLD" ]; then
  echo "pose logger: no dynamic_pose service found; exiting" >&2
  exit 1
fi
echo "pose logger: world=$WORLD" >&2
while true; do
  SNAP=$(timeout 5 gz topic -e -t "/world/$WORLD/dynamic_pose/info" --num 1 2>/dev/null)
  if [ -n "$SNAP" ]; then
    X=$(echo "$SNAP" | awk '/name:/{f=($0 ~ /"fyp_robot"/)} f && /^ *x:/{print $2; exit}')
    Y=$(echo "$SNAP" | awk '/name:/{f=($0 ~ /"fyp_robot"/)} f && /^ *x:/{gx=1} f && gx && /^ *y:/{print $2; exit}')
    if [ -n "$X" ] && [ -n "$Y" ]; then
      echo ",$X,$Y" >> "$OUT/gz_pose.csv"
    fi
  fi
  sleep 0.5
done
