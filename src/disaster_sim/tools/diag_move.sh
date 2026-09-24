#!/bin/bash
source ~/disaster_sim_ws/src/disaster_sim/setup_env.sh
export GAZEBO_MASTER_URI=http://localhost:${1:-11375}
WS=~/disaster_sim_ws/src/disaster_sim
cd "$WS"
rm -f /tmp/wcam/dm_*.jpg
python3 - <<'PY'
t=open("description/drone.sdf.template").read()
s=(t.replace("__NS__","drone_0").replace("__X__","16").replace("__Y__","-119")
   .replace("__Z__","2.0").replace("__YAW__","1.5708")
   .replace("__R__","0.95").replace("__G__","0.15").replace("__B__","0.1"))
open("/tmp/dgen/drone_0.sdf","w").write(s)
PY
pkill -9 -f "gzserver --verbose $WS/worlds/disaster_city" 2>/dev/null; sleep 1
timeout 160 gzserver --verbose "$WS/worlds/disaster_city.world" \
  -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > /tmp/dm_gz.log 2>&1 &
GZ=$!
for i in $(seq 1 60); do ros2 service list 2>/dev/null | grep -q set_entity_state && break; sleep 2; done
ros2 run gazebo_ros spawn_entity.py -entity drone_0 -file /tmp/dgen/drone_0.sdf 2>&1 | tail -1
sleep 3
# persistent camera saver (continuous)
python3 tools/grab_image.py /drone_0/camera/image_raw 40 dm 2>/tmp/dm_grab.log &
GRAB=$!
python3 nodes/swarm_controller.py --ros-args \
  -p "drones:=['drone_0']" -p "init_x:=[16.0]" -p "init_y:=[-119.0]" \
  -p "init_z:=[2.0]" -p "init_yaw:=[1.5708]" -p max_speed:=6.0 > /tmp/dm_ctrl.log 2>&1 &
CTRL=$!
sleep 4
ros2 topic pub --once /drone_0/cmd_pose geometry_msgs/Pose "{position: {x: 0.0, y: -108.0, z: 11.0}}" >/dev/null 2>&1
# sample gazebo truth (odom) vs controller (pose) every 2s for 20s
for k in $(seq 1 10); do
  sleep 2
  O=$(timeout 3 ros2 topic echo --once /drone_0/odom 2>/dev/null | grep -A3 "position:" | grep -E "x:|y:|z:" | tr -d ' \n')
  P=$(timeout 3 ros2 topic echo --once /drone_0/pose 2>/dev/null | grep -A3 "position:" | grep -E "x:|y:|z:" | tr -d ' \n')
  C=$(timeout 3 ros2 topic echo --once /clock 2>/dev/null | grep sec | tr -d ' \n')
  echo "t=${k}x2s  GAZEBO_odom[$O]  CTRL_pose[$P]  clock[$C]"
done
kill $GRAB $CTRL $GZ 2>/dev/null; wait 2>/dev/null
echo "--- grabbed frames (uniq sizes tell us if camera froze) ---"
ls -la /tmp/wcam/dm_*.jpg | awk '{print $5, $NF}' | sort | uniq -c -w6 | tail
echo "--- gz err ---"; grep -iE "error|excep|segfault" /tmp/dm_gz.log|grep -viE "RTShader|multicast"|head
