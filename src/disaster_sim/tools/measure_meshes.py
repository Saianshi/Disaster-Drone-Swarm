#!/usr/bin/env python3
"""Measure the bounding box of every mesh used by the world models.

Parses Collada 1.4.1 directly (no pycollada). Applies visual-scene node
matrices. Prints, per asset:
  min/max/centre/size on X,Y,Z, where the origin sits, Z-up vs Y-up guess.
"""
import sys
import xml.etree.ElementTree as ET
import numpy as np

NS = "{http://www.collada.org/2005/11/COLLADASchema}"


def _f(el):
    return el.tag.split("}")[-1]


def dae_points(path):
    root = ET.parse(path).getroot()
    up = "?"
    a = root.find(f"{NS}asset")
    if a is not None:
        u = a.find(f"{NS}up_axis")
        if u is not None:
            up = u.text.strip()

    # geometry_id -> Nx3 array of local positions
    geos = {}
    lib_geo = root.find(f"{NS}library_geometries")
    for g in lib_geo.findall(f"{NS}geometry"):
        gid = g.get("id")
        mesh = g.find(f"{NS}mesh")
        if mesh is None:
            continue
        verts = mesh.find(f"{NS}vertices")
        pos_src_id = None
        if verts is not None:
            for inp in verts.findall(f"{NS}input"):
                if inp.get("semantic") == "POSITION":
                    pos_src_id = inp.get("source").lstrip("#")
        if pos_src_id is None:
            # fall back: a source whose id contains 'pos'
            for s in mesh.findall(f"{NS}source"):
                if "pos" in s.get("id", "").lower():
                    pos_src_id = s.get("id")
                    break
        src = None
        for s in mesh.findall(f"{NS}source"):
            if s.get("id") == pos_src_id:
                src = s
        if src is None:
            continue
        fa = src.find(f"{NS}float_array")
        acc = src.find(f"{NS}technique_common/{NS}accessor")
        stride = int(acc.get("stride", "3")) if acc is not None else 3
        vals = np.fromstring(fa.text, sep=" ")
        vals = vals.reshape(-1, stride)[:, :3]
        geos[gid] = vals

    # apply visual-scene node matrices
    allpts = []
    vs = root.find(f"{NS}library_visual_scenes")
    used = set()
    if vs is not None:
        for scene in vs.findall(f"{NS}visual_scene"):
            for node in scene.iter(f"{NS}node"):
                m = node.find(f"{NS}matrix")
                M = np.eye(4)
                if m is not None:
                    M = np.fromstring(m.text, sep=" ").reshape(4, 4)
                for ig in node.findall(f"{NS}instance_geometry"):
                    gid = ig.get("url").lstrip("#")
                    if gid in geos:
                        used.add(gid)
                        p = geos[gid]
                        ph = np.hstack([p, np.ones((len(p), 1))])
                        allpts.append((M @ ph.T).T[:, :3])
    for gid, p in geos.items():
        if gid not in used:
            allpts.append(p)

    P = np.vstack(allpts)
    return P, up


def report(name, P, up, origin_note=""):
    lo = P.min(axis=0)
    hi = P.max(axis=0)
    c = (lo + hi) / 2
    s = hi - lo
    # origin (0,0,0) relative to geometry
    ox = "centre" if abs(c[0]) < s[0] * 0.05 else f"{-c[0]:+.2f} from centre"
    oy = "centre" if abs(c[1]) < s[1] * 0.05 else f"{-c[1]:+.2f} from centre"
    up_axis = "Z-up" if s[2] < max(s[0], s[1]) * 3 and up != "Y_UP" else up
    print(f"\n=== {name} ===")
    print(f"  up_axis in file : {up}")
    print(f"  X  min {lo[0]:8.2f}   max {hi[0]:8.2f}   size {s[0]:7.2f}   centre {c[0]:7.2f}")
    print(f"  Y  min {lo[1]:8.2f}   max {hi[1]:8.2f}   size {s[1]:7.2f}   centre {c[1]:7.2f}")
    print(f"  Z  min {lo[2]:8.2f}   max {hi[2]:8.2f}   size {s[2]:7.2f}   centre {c[2]:7.2f}")
    base_off = lo[2]              # how far the lowest point is below origin
    if abs(lo[2]) < 0.15:
        onote = "origin ~AT BASE (lowest point ~= z0)"
    elif abs(c[2]) < s[2] * 0.1:
        onote = "origin ~AT GEOMETRIC CENTRE (vertically)"
    else:
        onote = f"origin is {-lo[2]:+.2f} m above the lowest point"
    print(f"  origin vs geom  : X {ox} | Y {oy} | Z: {onote}")
    if origin_note:
        print(f"  note            : {origin_note}")
    return dict(lo=lo, hi=hi, size=s, centre=c)


def boxes_from_sdf(path):
    """AABB corners of every <box> visual in an SDF <model> (poses are local)."""
    r = ET.parse(path).getroot()
    pts = []
    for vis in r.iter("visual"):
        po = vis.find("pose")
        b = vis.find("geometry/box")
        if po is None or b is None:
            continue
        c = np.array([float(x) for x in po.text.split()][:3])
        s = np.array([float(x) for x in b.find("size").text.split()])
        pts.append(c - s / 2)
        pts.append(c + s / 2)
    return np.array(pts)


def boxes_from_world_model(world_path, model_name):
    r = ET.parse(world_path).getroot()
    for m in r.iter("model"):
        if m.get("name") != model_name:
            continue
        base = m.find("pose")
        bp = np.array([float(x) for x in base.text.split()][:3]) if base is not None else np.zeros(3)
        pts = []
        for vis in m.iter("visual"):
            po = vis.find("pose")
            b = vis.find("geometry/box")
            if po is None or b is None:
                continue
            c = np.array([float(x) for x in po.text.split()][:3])
            s = np.array([float(x) for x in b.find("size").text.split()])
            pts.append(c - s / 2 - bp)      # local to the model origin
            pts.append(c + s / 2 - bp)
        return np.array(pts)
    return None


if __name__ == "__main__":
    M = "/home/raj/disaster_sim_ws/src/disaster_sim/models"
    W = "/home/raj/disaster_sim_ws/src/disaster_sim/worlds/disaster_city.world"
    for n, p in [
        ("disaster_city", f"{M}/disaster_city/meshes/mesh.dae"),
        ("damaged_building", f"{M}/damaged_building/meshes/mesh.dae"),
        ("fiveg_base_station", f"{M}/fiveg_base_station/meshes/mesh.dae"),
        ("drone (drone.dae; model.sdf scales x0.45, +0.143 visual z)",
         f"{M}/sar_drone/meshes/drone.dae"),
    ]:
        P, up = dae_points(p)
        report(n, P, up)
    report("victim (model.sdf boxes, local)", boxes_from_sdf(f"{M}/victim/model.sdf"),
           "Z_UP", "articulated box figure; origin at the feet")
    report("victim_nook (world boxes, local to its model origin)",
           boxes_from_world_model(W, "victim_nook"), "Z_UP",
           "deck + baffles; positioned relative to the damaged_building")
