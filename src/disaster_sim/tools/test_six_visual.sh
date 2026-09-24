#!/bin/bash
# Final step-3 visual: 6 drones, persistent camera savers, fan out, then
# capture every drone cam + an external hero of the merged world.
source ~/disaster_sim_ws/src/disaster_sim/setup_env.sh
export GAZEBO_MASTER_URI=http://localhost:${1:-11390}
WS=~/disaster_sim_ws/src/disaster_sim
cd "$WS"
GEN=/tmp/disaster_sim_gen; mkdir -p "$GEN"
rm -f /tmp/wcam/fv_*.jpg /tmp/wcam/fhero*.jpg

python3 - <<'PY'
tmpl=open("description/drone.sdf.template").read()
D={"drone_0":(12.5,-122.5,0.95,0.15,0.10),"drone_1":(15.0,-122.5,0.98,0.55,0.05),
   "drone_2":(17.5,-122.5,0.95,0.90,0.12),"drone_3":(12.5,-125.0,0.15,0.85,0.22),
   "drone_4":(15.0,-125.0,0.12,0.78,0.92),"drone_5":(17.5,-125.0,0.90,0.18,0.80)}
for ns,(x,y,r,g,b) in D.items():
    s=(tmpl.replace("__NS__",ns).replace("__X__",str(x)).replace("__Y__",str(y))
        .replace("__Z__","1.2").replace("__YAW__","1.57")
        .replace("__R__",str(r)).replace("__G__",str(g)).replace("__B__",str(b)))
    open(f"/tmp/disaster_sim_gen/{ns}.sdf","w").write(s)
base=open("worlds/disaster_city.world").read()
import math
x,y,z,tx,ty,tz=44,-135,30, 0,-100,10
dx,dy,dz=tx-x,ty-y,tz-z
cam=f'<model name="fhero"><static>true</static><pose>{x} {y} {z} 0 {-math.atan2(dz,math.hypot(dx,dy)):.4f} {math.atan2(dy,dx):.4f}</pose><link name="l"><sensor name="c" type="camera"><camera><horizontal_fov>1.25</horizontal_fov><image><width>1280</width><height>720</height></image><clip><near>0.3</near><far>700</far></clip><save enabled="true"><path>/tmp/wcam</path></save></camera><always_on>1</always_on><update_rate>2</update_rate></sensor></link></model>'
open("/tmp/fv.world","w").write(base.replace("  </world>", cam+"\n  </world>"))
PY

pkill -9 -f "gzserver --verbose $WS" 2>/dev/null; sleep 1
timeout 200 gzserver --verbose /tmp/fv.world \
  -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > /tmp/fv_gz.log 2>&1 &
GZ=$!
for i in $(seq 1 60); do ros2 service list 2>/dev/null | grep -q set_entity_state && break; sleep 2; done
for ns in drone_0 drone_1 drone_2 drone_3 drone_4 drone_5; do
  ros2 run gazebo_ros spawn_entity.py -entity $ns -file $GEN/$ns.sdf >/dev/null 2>&1
  sleep 2
done
sleep 2
# persistent savers, one per drone
for n in 0 1 2 3 4 5; do
  python3 tools/grab_image.py /drone_$n/camera/image_raw 500 fv_d$n 2>/dev/null &
done
python3 nodes/swarm_controller.py --ros-args \
 -p "drones:=['drone_0','drone_1','drone_2','drone_3','drone_4','drone_5']" \
 -p "init_x:=[12.5,15.0,17.5,12.5,15.0,17.5]" \
 -p "init_y:=[-122.5,-122.5,-122.5,-125.0,-125.0,-125.0]" \
 -p "init_z:=[1.2,1.2,1.2,1.2,1.2,1.2]" -p "init_yaw:=[1.57,1.57,1.57,1.57,1.57,1.57]" \
 -p max_speed:=6.0 > /tmp/fv_ctrl.log 2>&1 &
CTRL=$!
sleep 4
# swarm arc facing the building's south/east
declare -A P=( [0]="-14 -114 9" [1]="-8 -110 11" [2]="0 -108 13" [3]="8 -109 12" [4]="16 -113 10" [5]="22 -100 9" )
for n in 0 1 2 3 4 5; do
  set -- ${P[$n]}
  ros2 topic pub --once /drone_$n/cmd_pose geometry_msgs/Pose "{position: {x: $1.0, y: $2.0, z: $3.0}}" >/dev/null 2>&1
done
sleep 16
for n in 0 1 2 3 4 5; do
  o=$(timeout 3 ros2 topic echo --once /drone_$n/odom 2>/dev/null | grep -A3 "position:" | grep -E "x:|y:|z:" | tr -d ' \n')
  echo "drone_$n -> $o"
done
sleep 2
kill $CTRL $GZ 2>/dev/null; pkill -f grab_image.py 2>/dev/null; wait 2>/dev/null
# keep newest frame per drone
for n in 0 1 2 3 4 5; do
  f=$(ls -t /tmp/wcam/fv_d${n}_*.jpg 2>/dev/null | head -1); [ -n "$f" ] && cp "$f" /tmp/wcam/fv_final_d$n.jpg
done
f=$(ls -t /tmp/wcam/*fhero* 2>/dev/null | head -1); [ -n "$f" ] && cp "$f" /tmp/wcam/fhero_final.jpg
echo "frames:"; ls -la /tmp/wcam/fv_final_d*.jpg /tmp/wcam/fhero_final.jpg 2>&1
