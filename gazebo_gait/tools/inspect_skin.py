#!/usr/bin/env python3
"""Inspect a COLLADA skinned mesh: geometry sources + skin controller (bind-shape
matrix, joints, inverse-bind matrices, vertex weights layout). Plans limb extraction.
Usage: python inspect_skin.py /path/walk.dae
"""
import sys
import xml.etree.ElementTree as ET

NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
def tag(e): return e.tag.split("}")[-1]

root = ET.parse(sys.argv[1]).getroot()

print("=== geometries ===")
for g in root.iter():
    if tag(g) == "geometry":
        mesh = g.find("c:mesh", NS)
        if mesh is None:
            continue
        prim = [tag(c) for c in mesh if tag(c) in ("triangles", "polylist", "polygons")]
        nverts = "?"
        for s in mesh.findall("c:source", NS):
            fa = s.find("c:float_array", NS)
            if fa is not None and "position" in (s.get("id") or "").lower():
                nverts = int(fa.get("count")) // 3
        print(f"  geometry id={g.get('id')} prims={prim} positions~{nverts}")

print("\n=== controllers (skin) ===")
for ctl in root.iter():
    if tag(ctl) == "controller":
        skin = ctl.find("c:skin", NS)
        if skin is None:
            continue
        print(f"  controller id={ctl.get('id')} skin_source={skin.get('source')}")
        bsm = skin.find("c:bind_shape_matrix", NS)
        if bsm is not None:
            print("   bind_shape_matrix:", " ".join(bsm.text.split()[:4]), "...")
        # joints Name_array
        for s in skin.findall("c:source", NS):
            na = s.find("c:Name_array", NS)
            if na is not None:
                names = na.text.split()
                print(f"   JOINTS ({na.get('count')}): {names[:6]} ...")
            fa = s.find("c:float_array", NS)
            if fa is not None and "bind" in (s.get("id") or "").lower():
                print(f"   inv_bind_matrices source id={s.get('id')} floats={fa.get('count')}")
        vw = skin.find("c:vertex_weights", NS)
        if vw is not None:
            print(f"   vertex_weights count={vw.get('count')}")
            for inp in vw.findall("c:input", NS):
                print(f"     input sem={inp.get('semantic')} src={inp.get('source')} offset={inp.get('offset')}")
            vc = vw.find("c:vcount", NS)
            v = vw.find("c:v", NS)
            if vc is not None:
                print("     vcount[:12]:", vc.text.split()[:12])
            if v is not None:
                print("     v[:16]:", v.text.split()[:16])
