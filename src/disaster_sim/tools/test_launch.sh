#!/bin/bash
# Step 7: run the ONE launch file end to end, headless.
source /usr/share/gazebo/setup.sh
source /opt/ros/humble/setup.bash
source ~/disaster_sim_ws/install/setup.bash
export GAZEBO_MASTER_URI=http://localhost:${1:-11470}

pkill -9 -f 'gzserver --verbose .*disaster_city' 2>/dev/null; sleep 1

timeout 300 ros2 launch disaster_sim mission.launch.py \
    gui:=false rviz:=false world_wait:=38 > /tmp/launch_run.log 2>&1
echo "ros2 launch exit: $?"

echo "=== phase / alert lines ==="
grep -E 'phase:|lifting|fanning|breaking|scan pattern|orange contour|VICTIM|ALERT|MISSION|shutting the demo|spawned entity' /tmp/launch_run.log
echo "=== alert topic content in log ==="
grep -E 'VICTIM DETECTED' /tmp/launch_run.log
echo "=== shutdown ==="
grep -iE 'shutting|all .* processes have finished|\[INFO\].*mission_node.*finished' /tmp/launch_run.log | tail
echo "=== errors ==="
grep -iE 'error|traceback|failed to|could not load|no such file' /tmp/launch_run.log \
  | grep -viE 'RTShader|end-of-life|migration|gazebo_ros_init.cpp' | head -15

pkill -9 -f 'gzserver --verbose .*disaster_city' 2>/dev/null
pkill -9 -f 'nodes/swarm_controller.py' 2>/dev/null
