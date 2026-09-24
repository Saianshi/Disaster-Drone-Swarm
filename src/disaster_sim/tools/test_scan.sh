#!/bin/bash
# One drone flies to an east-facade scan position; grab drone-cam + external hero.
source ~/disaster_sim_ws/src/disaster_sim/setup_env.sh
export GAZEBO_MASTER_URI=http://localhost:${1:-11380}
WS=~/disaster_sim_ws/src/disaster_sim
cd "$WS"
rm -f /tmp/wcam/sc_*.jpg /tmp/wcam/scan_*.jpg

python3 - <<'PY'
t=open("description/drone.sdf.template").read()
s=(t.replace("__NS__","drone_0").replace("__X__","16").replace("__Y__","-119")
   .replace("__Z__","2.0").replace("__YAW__","1.5708")
   .replace("__R__","0.95").replace("__G__","0.15").replace("__B__","0.1"))
open("/tmp/dgen/drone_0.sdf","w").write(s)
# external hero cam spliced into the world
base=open("worlds/disaster_city.world").read()
import math
cams=""
for nm,(x,y,z,tx,ty,tz) in {
  "scan_hero":(40,-120,26, 0,-95,9),
  "scan_side":(28,-70,14, 8,-95,8),
}.items():
    dx,dy,dz=tx-x,ty-y,tz-z
    yaw=math.atan2(dy,dx); pit=-math.atan2(dz,math.hypot(dx,dy))
    cams+=f'<model name="{nm}"><static>true</static><pose>{x} {y} {z} 0 {pit:.4f} {yaw:.4f}</pose><link name="l"><sensor name="c" type="camera"><camera><horizontal_fov>1.2</horizontal_fov><image><width>1100</width><height>720</height></image><clip><near>0.3</near><far>600</far></clip><save enabled="true"><path>/tmp/wcam</path></save></camera><always_on>1</always_on><update_rate>2</update_rate></sensor></link></model>\n'
open("/tmp/scan.world","w").write(base.replace("  </world>", cams+"  </world>"))
PY

pkill -9 -f "gzserver --verbose $WS" 2>/dev/null; sleep 1
timeout 170 gzserver --verbose /tmp/scan.world \
  -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > /tmp/sc_gz.log 2>&1 &
GZ=$!
for i in $(seq 1 60); do ros2 service list 2>/dev/null | grep -q set_entity_state && break; sleep 2; done
ros2 run gazebo_ros spawn_entity.py -entity drone_0 -file /tmp/dgen/drone_0.sdf 2>&1 | tail -1
sleep 3
python3 tools/grab_image.py /drone_0/camera/image_raw 300 sc_live 2>/dev/null &
GRAB=$!
python3 nodes/swarm_controller.py --ros-args \
  -p "drones:=['drone_0']" -p "init_x:=[16.0]" -p "init_y:=[-119.0]" \
  -p "init_z:=[2.0]" -p "init_yaw:=[1.5708]" -p max_speed:=6.0 > /tmp/sc_ctrl.log 2>&1 &
CTRL=$!
sleep 4
echo "launch -> fan out -> east scan position"
ros2 topic pub --once /drone_0/cmd_pose geometry_msgs/Pose "{position: {x: 16.0, y: -110.0, z: 8.0}}" >/dev/null 2>&1
sleep 6
ros2 topic pub --once /drone_0/cmd_pose geometry_msgs/Pose "{position: {x: 24.0, y: -95.0, z: 9.0}, orientation: {z: 1.0, w: 0.0}}" >/dev/null 2>&1
sleep 12
timeout 4 ros2 topic echo --once /drone_0/odom 2>/dev/null | grep -A3 position
cp "$(ls -t /tmp/wcam/sc_live_*.jpg | head -1)" /tmp/wcam/scan_dronecam.jpg
sleep 3
for c in scan_hero scan_side; do f=$(ls -t /tmp/wcam/*_${c}_l_* 2>/dev/null | head -1); [ -n "$f" ] && cp "$f" /tmp/wcam/${c}.jpg; done
kill $GRAB $CTRL $GZ 2>/dev/null; wait 2>/dev/null
echo "=== frame check ==="
ls -la /tmp/wcam/scan_dronecam.jpg /tmp/wcam/scan_hero.jpg /tmp/wcam/scan_side.jpg 2>&1
grep -iE "error|traceback" /tmp/sc_ctrl.log | head -3
