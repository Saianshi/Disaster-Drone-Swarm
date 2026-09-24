#!/usr/bin/env python3
"""Splice inspection camera models into a world file and print the path.
Usage: render_views.py WORLD OUT.world  (cameras defined below)
"""
import sys, math

WORLD, OUT = sys.argv[1], sys.argv[2]
OUTDIR = "/tmp/wcam"

# (name, x, y, z, pitch, yaw)  -- explicit list, edit as needed
CAMS = []


def look_at(name, cx, cy, cz, tx, ty, tz, fov=1.2, w=1100, h=720):
    dx, dy, dz = tx - cx, ty - cy, tz - cz
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.hypot(dx, dy))
    CAMS.append((name, cx, cy, cz, pitch, yaw, fov, w, h))


BX, BY = 0, -95
look_at("hero", 48, -140, 30, BX, BY - 4, 8, fov=1.15, w=1280, h=720)
look_at("basepov", 24, -126, 6, BX, BY, 8, fov=1.15, w=1280, h=720)
look_at("facade_S", 0, -118, 7, BX, BY, 7, fov=1.0, w=1100, h=720)
look_at("facade_E", 26, -95, 7, BX, BY, 7, fov=1.0, w=1100, h=720)

base = open(WORLD).read()
blocks = ""
for (nm, x, y, z, p, yw, fov, w, h) in CAMS:
    blocks += f"""
    <model name="{nm}"><static>true</static><pose>{x:.3f} {y:.3f} {z:.3f} 0 {p:.4f} {yw:.4f}</pose>
      <link name="l"><sensor name="c" type="camera"><camera>
        <horizontal_fov>{fov}</horizontal_fov>
        <image><width>{w}</width><height>{h}</height></image>
        <clip><near>0.3</near><far>600</far></clip>
        <save enabled="true"><path>{OUTDIR}</path></save>
      </camera><always_on>1</always_on><update_rate>2</update_rate></sensor></link></model>"""

open(OUT, "w").write(base.replace("  </world>", blocks + "\n  </world>"))
print(OUT, "with", len(CAMS), "cameras")
