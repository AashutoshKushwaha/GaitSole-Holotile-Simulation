#!/usr/bin/env python3
"""Dump rest matrices of key leg/pelvis bones + the animation output-array layout
for one bone, so we can author custom bone matrices correctly. Usage:
  python dae_detail.py /path/walk.dae
"""
import sys
import xml.etree.ElementTree as ET

NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
def tag(e): return e.tag.split("}")[-1]

path = sys.argv[1]
root = ET.parse(path).getroot()

WANT = ["Hips", "LeftUpLeg", "LeftLeg", "LeftFoot", "RightUpLeg", "RightLeg", "RightFoot"]

print("=== rest local matrices (row-major 4x4) ===")
for node in root.iter():
    if tag(node) == "node" and node.get("type") == "JOINT" and node.get("name") in WANT:
        m = node.find("c:matrix", NS)
        if m is not None:
            vals = [float(x) for x in m.text.split()]
            print(f"\n{node.get('name')}  (sid={node.get('sid')}):")
            for r in range(4):
                print("   " + "  ".join(f"{vals[r*4+c]:+.4f}" for c in range(4)))

# Animation layout for LeftLeg
print("\n=== animation block for LeftLeg/transform ===")
# find channel
sampler_id = None
for ch in root.iter():
    if tag(ch) == "channel" and ch.get("target") == "LeftLeg/transform":
        sampler_id = ch.get("source").lstrip("#")
print("sampler id:", sampler_id)
# find sampler -> inputs
for s in root.iter():
    if tag(s) == "sampler" and s.get("id") == sampler_id:
        for inp in s:
            print(f"  input semantic={inp.get('semantic')} source={inp.get('source')}")
# find OUTPUT source of any LeftLeg animation source
for src in root.iter():
    if tag(src) == "source" and "LeftLeg" in (src.get("id") or "") and "output" in (src.get("id") or "").lower():
        fa = src.find("c:float_array", NS)
        acc = src.find("c:technique_common/c:accessor", NS)
        print(f"\noutput source id={src.get('id')}")
        print(f"  float_array count={fa.get('count')}")
        print(f"  accessor stride={acc.get('stride') if acc is not None else '?'} "
              f"count={acc.get('count') if acc is not None else '?'}")
        vals = [float(x) for x in fa.text.split()]
        print("  first 16 floats (keyframe-0 matrix):")
        for r in range(4):
            print("   " + "  ".join(f"{vals[r*4+c]:+.4f}" for c in range(4)))
        break
