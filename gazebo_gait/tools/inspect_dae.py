#!/usr/bin/env python3
"""Inspect a COLLADA (.dae) actor: skeleton bone hierarchy (rest transforms) and
animation channel structure. Used to plan a custom Camargo-driven animation.

Usage: python inspect_dae.py /path/to/walk.dae
"""
import sys
import xml.etree.ElementTree as ET

NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


def tag(e):
    return e.tag.split("}")[-1]


def main():
    path = sys.argv[1]
    tree = ET.parse(path)
    root = tree.getroot()

    # --- skeleton: walk JOINT-node hierarchy under visual_scenes ---
    print("=== SKELETON (bone : #children : has_rest_matrix) ===")
    joints = []

    def walk(node, depth):
        if node.get("type") == "JOINT":
            name = node.get("name") or node.get("sid") or node.get("id")
            kids = [c for c in node if tag(c) == "node" and c.get("type") == "JOINT"]
            mat = node.find("c:matrix", NS)
            joints.append(name)
            print(f"{'  '*depth}{name}  (children={len(kids)}, "
                  f"rest_matrix={'yes' if mat is not None else 'no'})")
        for c in node:
            if tag(c) == "node":
                walk(c, depth + 1 if node.get("type") == "JOINT" else depth)

    for vs in root.iter():
        if tag(vs) == "visual_scene":
            for n in vs:
                if tag(n) == "node":
                    walk(n, 0)
    print(f"\ntotal JOINT bones: {len(joints)}")

    # --- animation channels ---
    print("\n=== ANIMATION CHANNELS (target -> sampler) ===")
    chans = []
    for anim in root.iter():
        if tag(anim) == "channel":
            chans.append(anim.get("target"))
    for t in chans[:12]:
        print("  ", t)
    print(f"total channels: {len(chans)}")

    # --- count keyframes on first animation's input ---
    for src in root.iter():
        if tag(src) == "source" and "input" in (src.get("id") or "").lower():
            fa = src.find("c:float_array", NS)
            if fa is not None:
                vals = fa.get("count")
                print(f"\nfirst time-input source '{src.get('id')}': {vals} keyframes")
                arr = [float(x) for x in fa.text.split()][:6]
                print("  first times:", arr)
                break


if __name__ == "__main__":
    main()
