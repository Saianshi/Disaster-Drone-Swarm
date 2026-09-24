#!/usr/bin/env python3
"""Flatten an all-primitive URDF into ONE Collada mesh with a single geometry
per material colour. Turns thousands of draw calls into a handful - essential
for running six offscreen drone cameras over the city in real time.

Writes <out.dae> and a matching one-visual static <out.sdf>.
Usage: urdf_to_merged_dae.py in.urdf out_dir model_name [--min SIZE]
"""
import os
import sys
import math
import xml.etree.ElementTree as ET

import numpy as np

SRC, OUTDIR, NAME = sys.argv[1], sys.argv[2], sys.argv[3]
MIN = 0.0
if "--min" in sys.argv:
    MIN = float(sys.argv[sys.argv.index("--min") + 1])

# --carve x0,x1,y0,y1,z0,z1 : punch this axis-aligned hole through any
# axis-aligned box that overlaps it (used to open a window in the facade)
CARVE = None
if "--carve" in sys.argv:
    CARVE = [float(v) for v in sys.argv[sys.argv.index("--carve") + 1].split(",")]


def split_box(cx, cy, cz, sx, sy, sz, cv):
    """box (centre,size) minus axis-aligned region cv -> list of (centre,size)."""
    lo = [cx - sx / 2, cy - sy / 2, cz - sz / 2]
    hi = [cx + sx / 2, cy + sy / 2, cz + sz / 2]
    clo = [max(lo[i], cv[2 * i]) for i in range(3)]
    chi = [min(hi[i], cv[2 * i + 1]) for i in range(3)]
    if any(clo[i] >= chi[i] for i in range(3)):
        return [((cx, cy, cz), (sx, sy, sz))]          # no overlap
    out = []
    # slabs along each axis outside the cut, keeping it watertight
    segs = []
    for i in range(3):
        a, b = lo[i], hi[i]
        parts = []
        if clo[i] > a:
            parts.append((a, clo[i]))
        if chi[i] < b:
            parts.append((chi[i], b))
        segs.append(parts)
    # -X / +X full-height slabs
    for (a, b) in segs[0]:
        out.append((( (a + b) / 2, cy, cz), (b - a, sy, sz)))
    # middle X band, split in Y
    mxa, mxb = clo[0], chi[0]
    for (a, b) in segs[1]:
        out.append(((( mxa + mxb) / 2, (a + b) / 2, cz), (mxb - mxa, b - a, sz)))
    # middle X,Y band, split in Z
    mya, myb = clo[1], chi[1]
    for (a, b) in segs[2]:
        out.append(((( mxa + mxb) / 2, (mya + myb) / 2, (a + b) / 2),
                    (mxb - mxa, myb - mya, b - a)))
    return out


def rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
            @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def T(xyz, rpy):
    M = np.eye(4)
    M[:3, :3] = rpy_to_R(*rpy)
    M[:3, 3] = xyz
    return M


def origin_of(el):
    o = el.find("origin")
    if o is None:
        return T([0, 0, 0], [0, 0, 0])
    return T([float(v) for v in o.get("xyz", "0 0 0").split()],
             [float(v) for v in o.get("rpy", "0 0 0").split()])


def box_mesh(sx, sy, sz):
    h = np.array([sx, sy, sz]) / 2.0
    v = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * h
    f = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
         (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return v, f


def cyl_mesh(r, L, n=14):
    v = []
    for s in (-1, 1):
        for i in range(n):
            a = 2 * math.pi * i / n
            v.append([r * math.cos(a), r * math.sin(a), s * L / 2])
    v.append([0, 0, -L / 2])
    v.append([0, 0, L / 2])
    v = np.array(v)
    bot, top = 2 * n, 2 * n + 1
    f = []
    for i in range(n):
        j = (i + 1) % n
        f += [(i, j, n + j), (i, n + j, n + i)]
        f += [(bot, j, i), (top, n + i, n + j)]
    return v, f


def sph_mesh(r, n=8):
    v, idx = [], {}
    for a in range(n + 1):
        th = math.pi * a / n
        for b in range(2 * n):
            ph = 2 * math.pi * b / (2 * n)
            idx[(a, b)] = len(v)
            v.append([r * math.sin(th) * math.cos(ph),
                      r * math.sin(th) * math.sin(ph), r * math.cos(th)])
    v = np.array(v)
    f = []
    for a in range(n):
        for b in range(2 * n):
            b2 = (b + 1) % (2 * n)
            f.append((idx[(a, b)], idx[(a + 1, b)], idx[(a + 1, b2)]))
            f.append((idx[(a, b)], idx[(a + 1, b2)], idx[(a, b2)]))
    return v, f


root = ET.parse(SRC).getroot()
mats = {}
for m in root.findall("material"):
    c = m.find("color")
    if c is not None:
        mats[m.get("name")] = tuple(float(v) for v in c.get("rgba").split())

links = {l.get("name"): l for l in root.findall("link")}
parent, jT = {}, {}
for j in root.findall("joint"):
    ch = j.find("child").get("link")
    parent[ch] = j.find("parent").get("link")
    jT[ch] = origin_of(j)

wcache = {}


def wT(n):
    if n not in wcache:
        wcache[n] = (wT(parent[n]) @ jT[n]) if n in parent else np.eye(4)
    return wcache[n]


groups = {}   # rgba -> [verts(Nx3), faces list with running offset]
counts = {"box": 0, "cylinder": 0, "sphere": 0, "skip": 0}

for lname, link in links.items():
    W = wT(lname)
    for vis in link.findall("visual"):
        g = vis.find("geometry")
        b, c, s = g.find("box"), g.find("cylinder"), g.find("sphere")
        if b is not None:
            sz = [float(x) for x in b.get("size").split()]
            if max(sz) < MIN:
                counts["skip"] += 1
                continue
            v, f = box_mesh(*sz)
            counts["box"] += 1
        elif c is not None:
            r, L = float(c.get("radius")), float(c.get("length"))
            if max(2 * r, L) < MIN:
                counts["skip"] += 1
                continue
            v, f = cyl_mesh(r, L)
            counts["cylinder"] += 1
        elif s is not None:
            r = float(s.get("radius"))
            if 2 * r < MIN:
                counts["skip"] += 1
                continue
            v, f = sph_mesh(r)
            counts["sphere"] += 1
        else:
            continue

        M = W @ origin_of(vis)

        rgba = (0.6, 0.6, 0.6, 1.0)
        mel = vis.find("material")
        if mel is not None:
            if mel.find("color") is not None:
                rgba = tuple(float(x) for x in mel.find("color").get("rgba").split())
            elif mel.get("name") in mats:
                rgba = mats[mel.get("name")]
        key = tuple(round(x, 4) for x in rgba)

        pieces = []
        do_carve = False
        if b is not None and CARVE is not None:
            # carve region -> this box's local frame (handles a leaning wall)
            Minv = np.linalg.inv(M)
            corners = np.array([[CARVE[0], CARVE[2], CARVE[4], 1],
                                [CARVE[1], CARVE[2], CARVE[4], 1],
                                [CARVE[0], CARVE[3], CARVE[4], 1],
                                [CARVE[1], CARVE[3], CARVE[4], 1],
                                [CARVE[0], CARVE[2], CARVE[5], 1],
                                [CARVE[1], CARVE[2], CARVE[5], 1],
                                [CARVE[0], CARVE[3], CARVE[5], 1],
                                [CARVE[1], CARVE[3], CARVE[5], 1]])
            lc = (Minv @ corners.T).T[:, :3]
            lcv = [lc[:, 0].min(), lc[:, 0].max(),
                   lc[:, 1].min(), lc[:, 1].max(),
                   lc[:, 2].min(), lc[:, 2].max()]
            half = [s / 2 for s in sz]
            overlap = all(lcv[2 * i] < half[i] and lcv[2 * i + 1] > -half[i]
                          for i in range(3))
            if overlap:
                do_carve = True
                for (pc, ps) in split_box(0, 0, 0, sz[0], sz[1], sz[2], lcv):
                    pv, pf = box_mesh(*ps)
                    pvh = np.hstack([pv + np.array(pc), np.ones((len(pv), 1))])
                    pieces.append(((M @ pvh.T).T[:, :3], pf))
        if not do_carve:
            vh = np.hstack([v, np.ones((len(v), 1))])
            pieces.append(((M @ vh.T).T[:, :3], f))

        gv, gf = groups.setdefault(key, [[], []])
        for (pv, pf) in pieces:
            off = sum(len(a) for a in gv)
            gv.append(pv)
            gf.extend([(a + off, bb + off, cc + off) for (a, bb, cc) in pf])

print("primitives:", counts, "colour groups:", len(groups))

os.makedirs(OUTDIR, exist_ok=True)
dae_path = os.path.join(OUTDIR, "mesh.dae")


def farr(a):
    return " ".join(f"{x:.5g}" for x in np.asarray(a).reshape(-1))


out = ['<?xml version="1.0" encoding="utf-8"?>',
       '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">',
       '<asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>',
       '<library_effects>']
gid = {}
for i, key in enumerate(groups):
    r, g_, b, a = key
    gid[key] = f"c{i}"
    out.append(f'<effect id="c{i}-fx"><profile_COMMON><technique sid="t"><lambert>'
               f'<diffuse><color>{r} {g_} {b} {a}</color></diffuse>'
               f'<ambient><color>{r*0.5:.4f} {g_*0.5:.4f} {b*0.5:.4f} {a}</color></ambient>'
               f'</lambert></technique></profile_COMMON></effect>')
out.append('</library_effects><library_materials>')
for key in groups:
    out.append(f'<material id="{gid[key]}-mat" name="{gid[key]}">'
               f'<instance_effect url="#{gid[key]}-fx"/></material>')
out.append('</library_materials><library_geometries>')
for key, (gv, gf) in groups.items():
    V = np.vstack(gv)
    F = np.array(gf, dtype=int)
    # per-vertex normals
    N = np.zeros_like(V)
    fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
    for k in range(3):
        np.add.at(N, F[:, k], fn)
    ln = np.linalg.norm(N, axis=1, keepdims=True)
    ln[ln == 0] = 1
    N /= ln
    cid = gid[key]
    out.append(f'<geometry id="{cid}-mesh"><mesh>'
               f'<source id="{cid}-p"><float_array id="{cid}-pa" count="{V.size}">{farr(V)}</float_array>'
               f'<technique_common><accessor source="#{cid}-pa" count="{len(V)}" stride="3">'
               '<param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>'
               '</accessor></technique_common></source>'
               f'<source id="{cid}-n"><float_array id="{cid}-na" count="{N.size}">{farr(N)}</float_array>'
               f'<technique_common><accessor source="#{cid}-na" count="{len(N)}" stride="3">'
               '<param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>'
               '</accessor></technique_common></source>'
               f'<vertices id="{cid}-v"><input semantic="POSITION" source="#{cid}-p"/>'
               f'<input semantic="NORMAL" source="#{cid}-n"/></vertices>'
               f'<triangles count="{len(F)}" material="{cid}">'
               f'<input semantic="VERTEX" source="#{cid}-v" offset="0"/>'
               f'<p>{" ".join(map(str, F.reshape(-1)))}</p></triangles>'
               '</mesh></geometry>')
out.append('</library_geometries><library_visual_scenes><visual_scene id="s" name="s">')
for key in groups:
    cid = gid[key]
    out.append(f'<node id="{cid}-n" name="{cid}"><instance_geometry url="#{cid}-mesh">'
               f'<bind_material><technique_common>'
               f'<instance_material symbol="{cid}" target="#{cid}-mat"/>'
               f'</technique_common></bind_material></instance_geometry></node>')
out.append('</visual_scene></library_visual_scenes><scene><instance_visual_scene url="#s"/></scene></COLLADA>')
open(dae_path, "w").write("\n".join(out))

sdf = f'''<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{NAME}">
    <static>true</static>
    <link name="body">
      <visual name="v">
        <geometry><mesh><uri>model://{NAME}/meshes/mesh.dae</uri></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>
'''
open(os.path.join(OUTDIR, "model.sdf"), "w").write(sdf)
print("wrote", dae_path, os.path.getsize(dae_path), "bytes")
