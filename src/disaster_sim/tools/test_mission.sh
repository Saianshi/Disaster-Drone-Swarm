#!/bin/bash
# Step 5 end-to-end (no RViz): world + 6 drones + controller + mission node.
# Watch the phase log, capture the alert, grab the scanner cam at detection.
source ~/disaster_sim_ws/src/disaster_sim/setup_env.sh
export GAZEBO_MASTER_URI=http://localhost:${1:-11430}
WS=~/disaster_sim_ws/src/disaster_sim
cd "$WS"
GEN=/tmp/disaster_sim_gen; mkdir -p "$GEN"
rm -f /tmp/wcam/mis_*.jpg /tmp/mis_alert.txt

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
PY

pkill -9 -f "gzserver --verbose $WS/worlds/disaster_city" 2>/dev/null; sleep 1
timeout 360 gzserver --verbose "$WS/worlds/disaster_city.world" \
  -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > /tmp/mis_gz.log 2>&1 &
GZ=$!
for i in $(seq 1 60); do ros2 service list 2>/dev/null | grep -q set_entity_state && break; sleep 2; done
for ns in drone_0 drone_1 drone_2 drone_3 drone_4 drone_5; do
  ros2 run gazebo_ros spawn_entity.py -entity $ns -file $GEN/$ns.sdf >/dev/null 2>&1; sleep 2
done
sleep 3
python3 nodes/swarm_controller.py --ros-args \
 -p "drones:=['drone_0','drone_1','drone_2','drone_3','drone_4','drone_5']" \
 -p "init_x:=[12.5,15.0,17.5,12.5,15.0,17.5]" \
 -p "init_y:=[-122.5,-122.5,-122.5,-125.0,-125.0,-125.0]" \
 -p "init_z:=[1.2,1.2,1.2,1.2,1.2,1.2]" -p "init_yaw:=[1.57,1.57,1.57,1.57,1.57,1.57]" \
 -p max_speed:=7.0 -p max_climb:=3.5 > /tmp/mis_ctrl.log 2>&1 &
CTRL=$!
sleep 4

# capture the alert topic and a rolling scanner-cam frame
( ros2 topic echo /base_station/alerts std_msgs/String 2>/dev/null | tee /tmp/mis_alert.txt ) &
ECHOPID=$!
( python3 tools/grab_image.py /drone_2/camera/image_raw 3000 mis_scan 2>/dev/null ) &
GRABPID=$!

python3 nodes/mission_node.py > /tmp/mis_node.log 2>&1
echo "mission_node exited: $?"

sleep 2
kill $ECHOPID $GRABPID $CTRL 2>/dev/null
kill $GZ 2>/dev/null; sleep 3; kill -9 $GZ 2>/dev/null
pkill -9 -f "gzserver --verbose $WS/worlds/disaster_city" 2>/dev/null

echo "=== phase log ==="
grep -E 'phase:|lifting|fanning|breaking|scan pattern|ALERT|MISSION|timed out' /tmp/mis_node.log
echo "=== /base_station/alerts ==="
cat /tmp/mis_alert.txt
echo "=== terminal banner ==="
grep -A6 'VICTIM LOCATED' /tmp/mis_node.log
echo "=== ctrl errors ==="
grep -iE 'error|traceback' /tmp/mis_ctrl.log | grep -v Extern | head
# keep last scanner frame
f=$(ls -t /tmp/wcam/mis_scan_*.jpg 2>/dev/null | head -1); [ -n "$f" ] && cp "$f" /tmp/wcam/mis_scan_last.jpg && echo "scan frame: $f"
