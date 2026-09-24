#!/bin/bash
# Step-3 milestone test: one drone spawns, camera publishes, kinematic move works.
source ~/disaster_sim_ws/src/disaster_sim/setup_env.sh
export GAZEBO_MASTER_URI=http://localhost:${1:-11370}
WS=~/disaster_sim_ws/src/disaster_sim
cd "$WS"
rm -f /tmp/wcam/t3_*.jpg

# gen drone_0 SDF at base-station spawn point
python3 - <<'PY'
t=open("description/drone.sdf.template").read()
s=(t.replace("__NS__","drone_0").replace("__X__","16").replace("__Y__","-119")
    .replace("__Z__","1.5").replace("__YAW__","1.5708")
    .replace("__R__","0.95").replace("__G__","0.15").replace("__B__","0.1"))
open("/tmp/dgen/drone_0.sdf","w").write(s)
PY

pkill -9 -f "gzserver --verbose $WS/worlds/disaster_city" 2>/dev/null
sleep 1
timeout 150 gzserver --verbose "$WS/worlds/disaster_city.world" \
  -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > /tmp/t3_gz.log 2>&1 &
GZ=$!
echo "waiting for gazebo + state service ..."
for i in $(seq 1 60); do
  ros2 service list 2>/dev/null | grep -q /gazebo/set_entity_state && break
  sleep 2
done
ros2 service list 2>/dev/null | grep -E "set_entity_state|spawn_entity" || echo "!! services missing"

echo "--- spawn drone_0 ---"
ros2 run gazebo_ros spawn_entity.py -entity drone_0 -file /tmp/dgen/drone_0.sdf 2>&1 | tail -1
sleep 3

echo "--- start controller ---"
python3 nodes/swarm_controller.py --ros-args \
  -p "drones:=['drone_0']" -p "init_x:=[16.0]" -p "init_y:=[-119.0]" \
  -p "init_z:=[1.5]" -p "init_yaw:=[1.5708]" -p max_speed:=6.0 > /tmp/t3_ctrl.log 2>&1 &
CTRL=$!
sleep 4

echo "--- camera hz ---"
timeout 6 ros2 topic hz /drone_0/camera/image_raw 2>&1 | grep average | head -1

echo "--- fly to building (0,-100,18) ---"
timeout 4 ros2 topic pub --once /drone_0/cmd_pose geometry_msgs/Pose \
  "{position: {x: 0.0, y: -100.0, z: 18.0}}" 2>&1 | tail -1
for s in 3 6 9 13; do
  sleep 3
  timeout 5 python3 tools/grab_image.py /drone_0/camera/image_raw 1 t3_at${s}s 2>&1 | tail -1
done

echo "--- final pose / at_goal ---"
timeout 5 ros2 topic echo --once /drone_0/pose 2>/dev/null | grep -A3 position
timeout 5 ros2 topic echo --once /drone_0/at_goal 2>/dev/null
echo "--- tf frames ---"
timeout 8 ros2 run tf2_ros tf2_echo world drone_0/camera_link 2>&1 | grep -A2 Translation | head -4

kill $CTRL $GZ 2>/dev/null; wait 2>/dev/null
echo "--- controller errors ---"; grep -iE "error|traceback|exception" /tmp/t3_ctrl.log | head
echo "--- gz errors ---"; grep -iE "error|exception" /tmp/t3_gz.log | grep -viE "RTShader|multicast" | head
ls -la /tmp/wcam/t3_*.jpg 2>/dev/null
