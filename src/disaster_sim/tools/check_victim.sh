#!/bin/bash
source ~/disaster_sim_ws/src/disaster_sim/setup_env.sh
export GAZEBO_MASTER_URI=http://localhost:${1:-11395}
WS=~/disaster_sim_ws/src/disaster_sim
cd "$WS"
rm -f /tmp/wcam/vic_*.jpg

python3 - <<'PY'
import math
base=open("worlds/disaster_city.world").read()
V=(5.72,-95.0,7.9)               # centre east window
cams={
 "E_close":   ((11,-95,7.7), (5.6,-95,7.6)),  # right at the opening
 "E_window":  ((18,-95,8),   V),   # due east, window height -> SHOULD see
 "E_far":     ((30,-95,9),   (5.6,-95,7.6)),  # approach distance -> SHOULD see
 "NE_30":     ((16,-88,8),   V),   # 30 deg off -> marginal / NOT
 "S_facade":  ((5.72,-108,9),V),   # from south -> NOT
 "top_orbit": ((14,-95,22),  V),   # high -> NOT
}
blk=""
for nm,((x,y,z),(tx,ty,tz)) in cams.items():
    dx,dy,dz=tx-x,ty-y,tz-z
    yaw=math.atan2(dy,dx); pit=-math.atan2(dz,math.hypot(dx,dy))
    blk+=(f'<model name="{nm}"><static>true</static><pose>{x} {y} {z} 0 {pit:.4f} {yaw:.4f}</pose>'
          f'<link name="l"><sensor name="c" type="camera"><camera><horizontal_fov>1.05</horizontal_fov>'
          f'<image><width>900</width><height>680</height></image><clip><near>0.2</near><far>400</far></clip>'
          f'<save enabled="true"><path>/tmp/wcam</path></save></camera>'
          f'<always_on>1</always_on><update_rate>2</update_rate></sensor></link></model>\n')
open("/tmp/vic.world","w").write(base.replace("  </world>", blk+"  </world>"))
PY

pkill -9 -f "gzserver --verbose /tmp/vic.world" 2>/dev/null; sleep 1
timeout 120 gzserver --verbose /tmp/vic.world > /tmp/vic_gz.log 2>&1 &
GZ=$!
# wait until camera frames appear (world fully loaded) or 90s
for i in $(seq 1 45); do
  ls /tmp/wcam/*_E_close_l_*.jpg >/dev/null 2>&1 && { sleep 3; break; }
  sleep 2
done
kill $GZ 2>/dev/null; sleep 3; kill -9 $GZ 2>/dev/null
pkill -9 -f "gzserver --verbose /tmp/vic.world" 2>/dev/null
sed 's/\x1b\[[0-9;]*m//g' /tmp/vic_gz.log | grep -iE 'error|excep|victim|not find|spawn' | head
for c in E_close E_window E_far NE_30 S_facade top_orbit; do
  f=$(ls -t /tmp/wcam/*_${c}_l_*.jpg 2>/dev/null | head -1); [ -n "$f" ] && cp "$f" /tmp/wcam/vic_${c}.jpg
done
ls -la /tmp/wcam/vic_*.jpg 2>&1
