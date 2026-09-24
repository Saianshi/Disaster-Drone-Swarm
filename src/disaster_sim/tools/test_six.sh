#!/bin/bash
# Step 3 full: spawn all six drones, one controller, verify every camera +
# every drone moves. Measure RTF with six cameras live.
source ~/disaster_sim_ws/src/disaster_sim/setup_env.sh
export GAZEBO_MASTER_URI=http://localhost:${1:-11385}
WS=~/disaster_sim_ws/src/disaster_sim
cd "$WS"
GEN=/tmp/disaster_sim_gen; mkdir -p "$GEN"; rm -f "$GEN"/*.sdf /tmp/wcam/six_*.jpg

python3 - <<'PY'
tmpl=open("description/drone.sdf.template").read()
D={
 "drone_0":(12.5,-122.5,1.2,1.57,0.95,0.15,0.10),
 "drone_1":(15.0,-122.5,1.2,1.57,0.98,0.55,0.05),
 "drone_2":(17.5,-122.5,1.2,1.57,0.95,0.90,0.12),
 "drone_3":(12.5,-125.0,1.2,1.57,0.15,0.85,0.22),
 "drone_4":(15.0,-125.0,1.2,1.57,0.12,0.78,0.92),
 "drone_5":(17.5,-125.0,1.2,1.57,0.90,0.18,0.80),
}
for ns,(x,y,z,yaw,r,g,b) in D.items():
    s=(tmpl.replace("__NS__",ns).replace("__X__",str(x)).replace("__Y__",str(y))
        .replace("__Z__",str(z)).replace("__YAW__",str(yaw))
        .replace("__R__",str(r)).replace("__G__",str(g)).replace("__B__",str(b)))
    open(f"/tmp/disaster_sim_gen/{ns}.sdf","w").write(s)
print("generated 6 sdf")
PY

pkill -9 -f "gzserver --verbose $WS" 2>/dev/null; sleep 1
timeout 220 gzserver --verbose "$WS/worlds/disaster_city.world" \
  -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > /tmp/six_gz.log 2>&1 &
GZ=$!
for i in $(seq 1 60); do ros2 service list 2>/dev/null | grep -q set_entity_state && break; sleep 2; done

for ns in drone_0 drone_1 drone_2 drone_3 drone_4 drone_5; do
  ros2 run gazebo_ros spawn_entity.py -entity $ns -file $GEN/$ns.sdf 2>&1 | tail -1
  sleep 2
done
sleep 3

python3 nodes/swarm_controller.py --ros-args \
 -p "drones:=['drone_0','drone_1','drone_2','drone_3','drone_4','drone_5']" \
 -p "init_x:=[12.5,15.0,17.5,12.5,15.0,17.5]" \
 -p "init_y:=[-122.5,-122.5,-122.5,-125.0,-125.0,-125.0]" \
 -p "init_z:=[1.2,1.2,1.2,1.2,1.2,1.2]" \
 -p "init_yaw:=[1.57,1.57,1.57,1.57,1.57,1.57]" \
 -p max_speed:=6.0 > /tmp/six_ctrl.log 2>&1 &
CTRL=$!
sleep 5

echo "=== all camera topics ==="
ros2 topic list 2>/dev/null | grep -c "camera/image_raw$"
echo "=== per-camera hz (2s each) ==="
for n in 0 1 2 3 4 5; do
  r=$(timeout 4 ros2 topic hz /drone_$n/camera/image_raw 2>/dev/null | grep -m1 average)
  echo "  drone_$n: $r"
done

echo "=== fan out to a line in front of the building ==="
tx=(-16 -10 -4 4 10 16)
for n in 0 1 2 3 4 5; do
  ros2 topic pub --once /drone_$n/cmd_pose geometry_msgs/Pose \
    "{position: {x: ${tx[$n]}.0, y: -112.0, z: 12.0}}" >/dev/null 2>&1
done
sleep 12

echo "=== gazebo truth vs command ==="
for n in 0 1 2 3 4 5; do
  o=$(timeout 3 ros2 topic echo --once /drone_$n/odom 2>/dev/null | grep -A3 "position:" | grep -E "x:|y:|z:" | tr -d ' \n')
  echo "  drone_$n odom[$o]"
done

echo "=== grab all six camera frames ==="
for n in 0 1 2 3 4 5; do
  timeout 5 python3 tools/grab_image.py /drone_$n/camera/image_raw 1 six_d$n 2>/dev/null | tail -1
done

echo "=== RTF ==="
timeout 4 ros2 topic echo --once /clock 2>/dev/null | grep sec
grep -iE "real.time factor|RTF" /tmp/six_gz.log | tail -2

kill $CTRL $GZ 2>/dev/null; wait 2>/dev/null
grep -iE "error|traceback|exception" /tmp/six_ctrl.log | grep -v Externa | head
ls /tmp/wcam/six_d*_00.jpg 2>/dev/null
