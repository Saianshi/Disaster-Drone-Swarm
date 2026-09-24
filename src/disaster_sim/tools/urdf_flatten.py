#!/usr/bin/env python3
"""Flatten an all-primitive URDF into a single static SDF model, preserving
named-material colours (which Gazebo's own `gz sdf -p` silently drops).

Usage: urdf_flatten.py in.urdf out.sdf MODEL_NAME [--no-collision]
"""
import sys, math
import numpy as np
import xml.etree.ElementTree as ET


def rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def R_to_rpy(R):
    sy = -R[2, 0]
    sy = max(-1.0, min(1.0, sy))
    p = math.asin(sy)
    if abs(sy) < 0.999999:
        r = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(R[1, 0], R[0, 0])
    else:
        r = math.atan2(-R[1, 2], R[1, 1])
        y = 0.0
    return r, p, y


def T(xyz, rpy):
    M = np.eye(4)
    M[:3, :3] = rpy_to_R(*rpy)
    M[:3, 3] = xyz
    return M


def origin_of(el):
    o = el.find('origin')
    if o is None:
        return T([0, 0, 0], [0, 0, 0])
    xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
    rpy = [float(v) for v in o.get('rpy', '0 0 0').split()]
    return T(xyz, rpy)


def pose_str(M):
    x, y, z = M[:3, 3]
    r, p, yw = R_to_rpy(M[:3, :3])
    return f"{x:.5f} {y:.5f} {z:.5f} {r:.5f} {p:.5f} {yw:.5f}"


def geom_xml(g):
    b, c, s = g.find('box'), g.find('cylinder'), g.find('sphere')
    if b is not None:
        return f"<box><size>{b.get('size')}</size></box>"
    if c is not None:
        return f"<cylinder><radius>{c.get('radius')}</radius><length>{c.get('length')}</length></cylinder>"
    if s is not None:
        return f"<sphere><radius>{s.get('radius')}</radius></sphere>"
    return None


def main():
    src, dst, name = sys.argv[1], sys.argv[2], sys.argv[3]
    no_col = '--no-collision' in sys.argv[4:]

    root = ET.parse(src).getroot()

    # robot-level named materials -> rgba
    mats = {}
    for m in root.findall('material'):
        col = m.find('color')
        if col is not None:
            mats[m.get('name')] = [float(v) for v in col.get('rgba').split()]

    links = {l.get('name'): l for l in root.findall('link')}
    parent_of, jointT = {}, {}
    for j in root.findall('joint'):
        ch = j.find('child').get('link')
        parent_of[ch] = j.find('parent').get('link')
        jointT[ch] = origin_of(j)

    worldT = {}

    def wT(n):
        if n in worldT:
            return worldT[n]
        worldT[n] = (wT(parent_of[n]) @ jointT[n]) if n in parent_of else np.eye(4)
        return worldT[n]

    vis_out, col_out = [], []
    nvis = ncol = 0
    for lname, link in links.items():
        WT = wT(lname)
        for vis in link.findall('visual'):
            g = vis.find('geometry')
            gx = geom_xml(g)
            if gx is None:
                continue
            M = WT @ origin_of(vis)
            # resolve material
            rgba = None
            mel = vis.find('material')
            if mel is not None:
                if mel.find('color') is not None:
                    rgba = [float(v) for v in mel.find('color').get('rgba').split()]
                elif mel.get('name') in mats:
                    rgba = mats[mel.get('name')]
            matxml = ""
            if rgba:
                r, g_, b, a = rgba
                matxml = (f"<material>"
                          f"<ambient>{r*0.4:.4f} {g_*0.4:.4f} {b*0.4:.4f} {a}</ambient>"
                          f"<diffuse>{r:.4f} {g_:.4f} {b:.4f} {a}</diffuse>"
                          f"<specular>0.1 0.1 0.1 1</specular>"
                          f"</material>")
            vis_out.append(f'      <visual name="v{nvis}">'
                           f'<pose>{pose_str(M)}</pose>'
                           f'<geometry>{gx}</geometry>{matxml}</visual>')
            nvis += 1
        if no_col:
            continue
        for col in link.findall('collision'):
            gx = geom_xml(col.find('geometry'))
            if gx is None:
                continue
            M = WT @ origin_of(col)
            col_out.append(f'      <collision name="c{ncol}">'
                           f'<pose>{pose_str(M)}</pose>'
                           f'<geometry>{gx}</geometry></collision>')
            ncol += 1

    with open(dst, 'w') as f:
        f.write('<?xml version="1.0"?>\n<sdf version="1.7">\n')
        f.write(f'  <model name="{name}">\n    <static>true</static>\n')
        f.write('    <link name="body">\n')
        f.write("\n".join(col_out))
        if col_out:
            f.write("\n")
        f.write("\n".join(vis_out))
        f.write('\n    </link>\n  </model>\n</sdf>\n')

    print(f"{name}: {nvis} visuals, {ncol} collisions -> {dst}")


if __name__ == '__main__':
    main()
