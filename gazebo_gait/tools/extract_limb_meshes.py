#!/usr/bin/env python3
"""
Split the photoreal actor skin (walk.dae) into body-part meshes aligned to the
capsule rig's links, so the correctly-moving rig wears the real human geometry.

The skin is rigidly bound (1 bone/vertex), so each triangle belongs to the bone
of its majority vertex. We group triangles into rig links, then map each group
from the actor's BIND pose into the matching capsule-link local frame by a
similarity transform that sends the proximal joint -> link origin and the distal
joint -> the link's bone axis (THIGH/SHANK/FOOT down/forward), with roll fixed by
the hip lateral axis. Colours are kept per bone group (shirt/pants/skin/shoe).

Outputs OBJ files to meshes/ and prints the per-mesh (link, colour) table that
build_human_sdf.py consumes. Run: python extract_limb_meshes.py
"""
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
from build_human_sdf import THIGH, SHANK, FOOT, TRUNK, SHIRT, PANTS, SKIN, SHOE

DAE = "/root/.gz/fuel/fuel.gazebosim.org/mingfei/models/actor/1/meshes/walk.dae"
OUTDIR = os.path.join(ROOT, "meshes")
NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
def tag(e): return e.tag.split("}")[-1]


def src_floats(root, sid):
    for s in root.iter():
        if tag(s) == "source" and s.get("id") == sid:
            fa = s.find("c:float_array", NS)
            return np.array([float(x) for x in fa.text.split()])
    raise KeyError(sid)


def load(root):
    # positions
    mesh = next(g.find("c:mesh", NS) for g in root.iter()
                if tag(g) == "geometry" and g.find("c:mesh", NS) is not None)
    verts_el = mesh.find("c:vertices", NS)
    pos_src = next(i.get("source")[1:] for i in verts_el.findall("c:input", NS)
                   if i.get("semantic") == "POSITION")
    P = src_floats(root, pos_src).reshape(-1, 3)

    # triangles from all polylists (fan-triangulated), as position-index triples
    tris = []
    for pl in mesh.findall("c:polylist", NS):
        inputs = pl.findall("c:input", NS)
        stride = max(int(i.get("offset")) for i in inputs) + 1
        voff = int(next(i.get("offset") for i in inputs if i.get("semantic") == "VERTEX"))
        vcount = [int(x) for x in pl.find("c:vcount", NS).text.split()]
        p = [int(x) for x in pl.find("c:p", NS).text.split()]
        k = 0
        for n in vcount:
            idx = [p[(k + j) * stride + voff] for j in range(n)]
            for j in range(1, n - 1):
                tris.append((idx[0], idx[j], idx[j + 1]))
            k += n
    tris = np.array(tris)

    # skin: bind shape matrix, joints, inv-bind, per-vertex joint
    skin = next(c.find("c:skin", NS) for c in root.iter()
                if tag(c) == "controller" and c.find("c:skin", NS) is not None)
    BSM = np.array([float(x) for x in skin.find("c:bind_shape_matrix", NS).text.split()]).reshape(4, 4)
    joints, invbind = None, None
    for s in skin.findall("c:source", NS):
        na = s.find("c:Name_array", NS)
        if na is not None:
            joints = na.text.split()
        fa = s.find("c:float_array", NS)
        if fa is not None and "bind" in (s.get("id") or "").lower():
            invbind = np.array([float(x) for x in fa.text.split()]).reshape(-1, 4, 4)
    vw = skin.find("c:vertex_weights", NS)
    vcount = [int(x) for x in vw.find("c:vcount", NS).text.split()]
    vv = [int(x) for x in vw.find("c:v", NS).text.split()]
    wsrc = next(i.get("source")[1:] for i in vw.findall("c:input", NS)
                if i.get("semantic") == "WEIGHT")
    W = src_floats(root, wsrc)
    # dominant joint per vertex (offsets: JOINT=0, WEIGHT=1, stride 2)
    vjoint = np.empty(len(vcount), int)
    k = 0
    for i, n in enumerate(vcount):
        best_j, best_w = 0, -1.0
        for _ in range(n):
            j, wi = vv[2 * k], vv[2 * k + 1]
            if W[wi] > best_w:
                best_w, best_j = W[wi], j
            k += 1
        vjoint[i] = best_j
    return P, tris, BSM, joints, invbind, vjoint


def main():
    os.makedirs(OUTDIR, exist_ok=True)
    root = ET.parse(DAE).getroot()
    P, tris, BSM, joints, invbind, vjoint = load(root)
    jidx = {n: i for i, n in enumerate(joints)}

    # bind-pose world vertices and joint positions
    Pw = (BSM @ np.c_[P, np.ones(len(P))].T).T[:, :3]
    def jpos(name):
        return np.linalg.inv(invbind[jidx[name]])[:3, 3]

    # bone -> rig link
    L = {"LeftUpLeg": "thigh_l", "LHipJoint": "thigh_l", "LeftLeg": "shank_l",
         "LeftFoot": "foot_l", "LeftToeBase": "foot_l",
         "RightUpLeg": "thigh_r", "RHipJoint": "thigh_r", "RightLeg": "shank_r",
         "RightFoot": "foot_r", "RightToeBase": "foot_r"}
    SKIN_BONES = {"Neck", "Neck1", "Head", "LeftForeArm", "RightForeArm", "LeftHand",
                  "RightHand", "LeftFingerBase", "RightFingerBase", "LeftHandIndex1",
                  "RightHandIndex1", "LThumb", "RThumb"}
    def link_of(bone):
        if bone in L:
            return L[bone]
        return "skin" if bone in SKIN_BONES else "torso"   # rest of upper body

    # lateral axis (actor left) to fix roll
    lat = jpos("LeftUpLeg") - jpos("RightUpLeg")

    # pose the bind-pose T-pose arms down to the sides (cosmetic; arms aren't
    # animated). Rotate each arm's vertices about its shoulder around the forward
    # axis so they hang alongside the torso.
    spine_up0 = jpos("Spine1") - jpos("Hips")
    fwd = np.cross(spine_up0, lat); fwd /= np.linalg.norm(fwd)
    def rodrigues(axis, ang):
        a = axis / np.linalg.norm(axis)
        c, s = np.cos(ang), np.sin(ang)
        K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        return np.eye(3) + s * K + (1 - c) * K @ K
    ARM = {"l": ["LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand",
                 "LeftFingerBase", "LeftHandIndex1", "LThumb"],
           "r": ["RightShoulder", "RightArm", "RightForeArm", "RightHand",
                 "RightFingerBase", "RightHandIndex1", "RThumb"]}
    vbone = np.array([joints[vjoint[i]] for i in range(len(Pw))])
    for side, sgn, pj in (("l", 1.0, "LeftArm"), ("r", -1.0, "RightArm")):
        Rd = rodrigues(fwd, sgn * np.deg2rad(78))
        piv = jpos(pj)
        mask = np.isin(vbone, ARM[side])
        Pw[mask] = piv + (Rd @ (Pw[mask] - piv).T).T

    # per-link alignment: (proximal, distal, target_axis, target_len)
    hipc = 0.5 * (jpos("LeftUpLeg") + jpos("RightUpLeg"))
    spine_up = jpos("Spine1") - jpos("Hips")
    seg = {
        "thigh_l": ("LeftUpLeg", "LeftLeg", np.array([0, 0, -1.]), THIGH),
        "shank_l": ("LeftLeg", "LeftFoot", np.array([0, 0, -1.]), SHANK),
        "foot_l":  ("LeftFoot", "LeftToeBase", np.array([1., 0, 0]), FOOT),
        "thigh_r": ("RightUpLeg", "RightLeg", np.array([0, 0, -1.]), THIGH),
        "shank_r": ("RightLeg", "RightFoot", np.array([0, 0, -1.]), SHANK),
        "foot_r":  ("RightFoot", "RightToeBase", np.array([1., 0, 0]), FOOT),
    }

    def basis(xaxis, yref):
        x = xaxis / np.linalg.norm(xaxis)
        y = yref - x * (yref @ x)
        y /= np.linalg.norm(y)
        z = np.cross(x, y)
        return np.array([x, y, z]).T          # columns = frame axes

    def transform_for(link):
        if link in seg:
            prox, dist, a_t, length = seg[link]
            Pp, Dp = jpos(prox), jpos(dist)
            u = Dp - Pp
            s = length / np.linalg.norm(u)
            Ra = basis(u, lat)                 # actor frame
            Rt = basis(a_t, np.array([0, 1., 0]))   # target: +y is left
            R = Rt @ Ra.T
            return Pp, s, R
        else:                                  # torso / skin: pelvis-link frame
            # use the LEG scale (actor is already human-proportioned) -- scaling by
            # TRUNK/|Hips->Spine1| blew the torso up ~2.3x (Spine1 is only mid-chest).
            s = THIGH / np.linalg.norm(jpos("LeftLeg") - jpos("LeftUpLeg"))
            Ra = basis(spine_up, lat)          # actor up + lateral
            Rt = basis(np.array([0, 0, 1.]), np.array([0, 1., 0]))  # up=+z, left=+y
            R = Rt @ Ra.T
            return hipc, s, R

    # group triangles by link (majority vertex bone)
    groups = {}
    for t in tris:
        links = [link_of(joints[vjoint[v]]) for v in t]
        lk = max(set(links), key=links.count)
        groups.setdefault(lk, []).append(t)

    COLOR = {"thigh_l": PANTS, "thigh_r": PANTS, "shank_l": PANTS, "shank_r": PANTS,
             "foot_l": SHOE, "foot_r": SHOE, "torso": SHIRT, "skin": SKIN}
    ATTACH = {"torso": "pelvis", "skin": "pelvis"}   # else attaches to its own link

    table = []
    for lk, tlist in groups.items():
        origin, s, R = transform_for(lk)
        used = sorted({v for t in tlist for v in t})
        remap = {v: i for i, v in enumerate(used)}
        V = np.array([s * (R @ (Pw[v] - origin)) for v in used])
        F = np.array([[remap[t[0]], remap[t[1]], remap[t[2]]] for t in tlist])

        # smooth vertex normals (averaged face normals) so lighting shades the mesh
        N = np.zeros_like(V)
        fn = np.cross(V[F[:, 1]] - V[F[:, 0]], V[F[:, 2]] - V[F[:, 0]])
        for i, f3 in enumerate(F):
            N[f3] += fn[i]
        ln = np.linalg.norm(N, axis=1, keepdims=True)
        N = N / np.where(ln > 1e-9, ln, 1.0)

        rgb = COLOR[lk].split()[:3]
        with open(os.path.join(OUTDIR, f"{lk}.mtl"), "w") as f:
            f.write(f"newmtl {lk}\nKd {rgb[0]} {rgb[1]} {rgb[2]}\n"
                    f"Ka {rgb[0]} {rgb[1]} {rgb[2]}\nKs 0.05 0.05 0.05\nNs 8\n")
        path = os.path.join(OUTDIR, f"{lk}.obj")
        with open(path, "w") as f:
            f.write(f"mtllib {lk}.mtl\nusemtl {lk}\n")
            for p in V:
                f.write(f"v {p[0]:.5f} {p[1]:.5f} {p[2]:.5f}\n")
            for n in N:
                f.write(f"vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n")
            for a, b, c in F + 1:
                f.write(f"f {a}//{a} {b}//{b} {c}//{c}\n")
        attach = ATTACH.get(lk, lk)
        table.append((lk, attach, COLOR[lk], len(V), len(tlist)))
        print(f"  {lk:8s} -> link {attach:8s} {len(V):5d} verts {len(tlist):5d} tris  {path}")
    print("MESHES_DONE")


if __name__ == "__main__":
    main()
