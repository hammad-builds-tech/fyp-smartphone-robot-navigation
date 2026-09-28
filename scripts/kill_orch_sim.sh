#!/bin/bash
# Kill only the orchestrator and recorded test-streamer processes (keep Gazebo/ROS).
for pat in "run_system.sh" "test_backend_pipeline.py"; do
  for pid in $(pgrep -f "$pat"); do
    [ "$pid" != "$$" ] && kill "$pid" 2>/dev/null
  done
done
sleep 2
echo "orchestrators left: $(pgrep -cf 'run_system.sh')"
echo "streamers left: $(pgrep -cf 'test_backend_pipeline.py')"
echo "gz left: $(pgrep -cf 'gz-sim-main')"
