#!/bin/bash
# Headless layout check: launch gui:=false rviz:=false, print every model +
# drone world pose, assert the datum / adjacency / distance / ground-level
# rules, grab one overhead screenshot.
export DISPLAY=:1
source /usr/share/gazebo/setup.sh
source /opt/ros/humble/setup.bash
source ~/disaster_sim_ws/install/setup.bash
_SH=~/disaster_sim_ws/install/disaster_sim/share/disaster_sim
export GAZEBO_MODEL_PATH=$_SH/models:${GAZEBO_MODEL_PATH:-}
export GAZEBO_RESOURCE_PATH=$_SH:${GAZEBO_RESOURCE_PATH:-}
export GAZEBO_MODEL_DATABASE_URI=""      # never touch the online DB
export GAZEBO_MASTER_URI=http://localhost:${1:-11480}
rm -f /tmp/wcam/layout_*.jpg

# splice a wide overhead camera into the world for the screenshot
python3 - <<'PY'
import math
base = open("/home/raj/disaster_sim_ws/install/disaster_sim/share/disaster_sim/worlds/disaster_city.world").read()
# look down at the whole scene centre (~x40, y-4) from high up, slight tilt
def cam(name, c, t, fov=1.3):
    dx, dy, dz = t[0]-c[0], t[1]-c[1], t[2]-c[2]
    yaw = math.atan2(dy, dx); pit = -math.atan2(dz, math.hypot(dx, dy))
    return (f'<model name="{name}"><static>true</static>'
            f'<pose>{c[0]} {c[1]} {c[2]} 0 {pit:.4f} {yaw:.4f}</pose><link name="l">'
            f'<sensor name="c" type="camera"><camera><horizontal_fov>{fov}</horizontal_fov>'
            f'<image><width>1500</width><height>950</height></image>'
            f'<clip><near>1</near><far>700</far></clip>'
            f'<save enabled="true"><path>/tmp/wcam</path></save></camera>'
            f'<always_on>1</always_on><update_rate>2</update_rate></sensor></link></model>')
cams = (cam("overhead", (60, -20, 240), (55, 0, 4), 1.45)          # whole scene
        + cam("bldedge", (85, -35, 45), (46, 0, 6))                # city east edge + building
        + cam("los", (128, -12, 8), (50, 0, 8), 1.1))              # base-station POV to window
open("/tmp/verify.world", "w").write(base.replace("  </world>", cams + "\n  </world>"))
PY

pkill -9 -f 'gzserver --verbose /tmp/verify.world' 2>/dev/null; sleep 1
timeout 150 gzserver --verbose /tmp/verify.world \
    -s libgazebo_ros_init.so -s libgazebo_ros_factory.so > /tmp/verify_gz.log 2>&1 &
GZ=$!
for i in $(seq 1 40); do ros2 service list 2>/dev/null | grep -q set_entity_state && break; sleep 1; done

GEN=/tmp/disaster_sim_gen
python3 - <<'PY'
import os
tmpl = open("/home/raj/disaster_sim_ws/install/disaster_sim/share/disaster_sim/description/drone.sdf.template").read()
D = {"drone_0": (121.0, -13.0), "drone_1": (124.0, -13.0), "drone_2": (127.0, -13.0),
     "drone_3": (121.0, -16.0), "drone_4": (124.0, -16.0), "drone_5": (127.0, -16.0)}
os.makedirs("/tmp/disaster_sim_gen", exist_ok=True)
for ns, (x, y) in D.items():
    s = (tmpl.replace("__NS__", ns).replace("__X__", str(x)).replace("__Y__", str(y))
             .replace("__Z__", "0.15").replace("__YAW__", "3.0")
             .replace("__R__", "0.9").replace("__G__", "0.5").replace("__B__", "0.1"))
    open(f"/tmp/disaster_sim_gen/{ns}.sdf", "w").write(s)
PY
for ns in drone_0 drone_1 drone_2 drone_3 drone_4 drone_5; do
    ros2 run gazebo_ros spawn_entity.py -entity $ns -file $GEN/$ns.sdf \
        -x $(grep -o '__X__' /dev/null || true) >/dev/null 2>&1 &
done
# spawn with the pose baked into the sdf
for ns in drone_0 drone_1 drone_2 drone_3 drone_4 drone_5; do
    ros2 run gazebo_ros spawn_entity.py -entity $ns -file $GEN/$ns.sdf >/dev/null 2>&1
done
sleep 4

python3 - <<'PY'
import subprocess, math, sys, time
import rclpy
from gazebo_msgs.msg import ModelStates

rclpy.init()
n = rclpy.create_node("verify")
got = {}
def cb(m):
    for name, p in zip(m.name, m.pose):
        got[name] = (p.position.x, p.position.y, p.position.z)
n.create_subscription(ModelStates, "/gazebo/model_states", cb, 10)
t0 = time.time()
while time.time() - t0 < 8 and len(got) < 10:
    rclpy.spin_once(n, timeout_sec=0.3)

# measured mesh min-Z (lowest geometry relative to model origin)
MESH_MINZ = {
    "disaster_city": -1.47, "damaged_building": -0.33,
    "fiveg_base_station": -0.60, "terrain": 0.0,
    "victim_nook": None, "victim": 0.0,
}
DATUM = 0.0
CITY = dict(xmin=-60.14, xmax=60.25, ymin=-56.23, ymax=55.00)
BLD = dict(sx=23.71, sy=11.18, minz=-0.33)   # building mesh, yaw 0

print("\n================ WORLD POSES ================")
for name in sorted(got):
    x, y, z = got[name]
    lo = ""
    if name in MESH_MINZ and MESH_MINZ[name] is not None:
        lo = f"   lowest geom Z = {z + MESH_MINZ[name]:+.3f}"
    print(f"  {name:20s} ({x:8.2f}, {y:8.2f}, {z:7.2f}){lo}")

fails = []

# 1. nothing that rests on the ground is below the datum
for m, mz in [("damaged_building", -0.33), ("fiveg_base_station", -0.60)]:
    if m in got:
        low = got[m][2] + mz
        if low < DATUM - 0.03:
            fails.append(f"{m} lowest geom Z {low:.3f} is below datum {DATUM}")
        elif low > DATUM + 0.05:
            fails.append(f"{m} floats: lowest geom Z {low:.3f} above datum {DATUM}")

# 2. damaged_building adjacent to / within the city footprint
if "damaged_building" in got:
    bx, by, _ = got["damaged_building"]
    b_xmin, b_xmax = bx - 11.24, bx + 12.47
    b_ymin, b_ymax = by - 5.90, by + 5.28
    # gap between building bbox and city bbox on X (city extends to xmax 60.25)
    gap_x = max(0.0, b_xmin - CITY["xmax"], CITY["xmin"] - b_xmax)
    gap_y = max(0.0, b_ymin - CITY["ymax"], CITY["ymin"] - b_ymax)
    gap = math.hypot(gap_x, gap_y)
    within_y = CITY["ymin"] <= by <= CITY["ymax"]
    print(f"\n  damaged_building bbox X[{b_xmin:.1f},{b_xmax:.1f}] Y[{b_ymin:.1f},{b_ymax:.1f}]")
    print(f"  gap to city footprint: {gap:.2f} m  (within city Y span: {within_y})")
    if gap > 6.0:
        fails.append(f"damaged_building not adjacent to city: {gap:.1f} m gap")

# 3. base station 60-100 m from the building
if "damaged_building" in got and "fiveg_base_station" in got:
    bx, by, _ = got["damaged_building"]
    sx, sy, _ = got["fiveg_base_station"]
    d = math.hypot(sx - bx, sy - by)
    print(f"\n  base station to building origin: {d:.1f} m")
    if not (55 <= d <= 105):
        fails.append(f"base station {d:.1f} m from building (want ~60-100)")

# 4. drones at ground level, 3 m spacing
dz = [got[f"drone_{i}"][2] for i in range(6) if f"drone_{i}" in got]
print(f"\n  drone spawn Z: {['%.2f' % z for z in dz]}")
if dz and (max(dz) > 0.6 or min(dz) < -0.05):
    fails.append(f"drones not at ground level: Z {min(dz):.2f}..{max(dz):.2f}")
if "drone_0" in got and "drone_1" in got:
    sp = math.hypot(got['drone_1'][0]-got['drone_0'][0], got['drone_1'][1]-got['drone_0'][1])
    print(f"  drone_0->drone_1 spacing: {sp:.2f} m")

# 5. scan geometry keeps the window in frame
VICTIM = (51.7, 0.0, 7.4)
CAM_PITCH = 0.72
for wp in [(60.0, 0.0, 13.5)]:
    cam = (wp[0] - 0.8, wp[1], wp[2] + 0.05)   # camera is boomed 0.8 m forward (west), yaw pi
    dx, dz2 = VICTIM[0] - cam[0], VICTIM[2] - cam[2]
    horiz = abs(dx)
    depress = math.atan2(-dz2, horiz)
    off = abs(depress - CAM_PITCH)
    vfov_half = math.atan(math.tan(1.40 / 2) * 480 / 640)
    print(f"\n  scan wp {wp}: standoff {horiz:.1f} m, window depression {math.degrees(depress):.1f} deg,"
          f" cam pitch {math.degrees(CAM_PITCH):.1f} deg, off-axis {math.degrees(off):.1f} deg"
          f" (VFOV half {math.degrees(vfov_half):.1f} deg)")
    if math.degrees(off) > math.degrees(vfov_half) - 8:
        fails.append(f"scan wp {wp}: window {math.degrees(off):.1f} deg off-axis, near/out of frame")

print("\n================ RESULT ================")
if fails:
    for f in fails:
        print("  FAIL:", f)
    sys.exit(1)
print("  ALL CHECKS PASSED")
PY
RC=$?

sleep 3
for c in overhead bldedge los; do
    f=$(ls -t /tmp/wcam/*_${c}_l_*.jpg 2>/dev/null | head -1)
    [ -n "$f" ] && cp "$f" /tmp/wcam/layout_${c}.jpg && echo "screenshot: /tmp/wcam/layout_${c}.jpg"
done
kill -9 $GZ 2>/dev/null
pkill -9 -f 'gzserver --verbose /tmp/verify.world' 2>/dev/null
exit $RC
