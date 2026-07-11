#!/usr/bin/env python3
"""
Edit a COLLADA actor's per-bone animation matrices in place.

A gz-sim actor animation stores, per bone, a sequence of 4x4 local-transform
matrices (row-major, stride 16) sampled at keyframe times. We keep the skin +
skeleton untouched and only overwrite the OUTPUT matrix arrays of chosen bones,
so the result is guaranteed skin-compatible.

Each animated bone's matrix at frame i is built as

    M_i = M_rest @ R(theta_i, axis)

i.e. the bone's rest local transform (offset + rest orientation) post-multiplied
by a pure rotation about a local axis -> the joint angle. The translation column
is preserved from M_rest (bone length to parent stays fixed).

DaeAnim wraps load / read-rest / set-bone-frames / save.
"""
import numpy as np
import xml.etree.ElementTree as ET

NS_URI = "http://www.collada.org/2005/11/COLLADASchema"
NS = {"c": NS_URI}


def _tag(e):
    return e.tag.split("}")[-1]


def Rx(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]], float)


def Ry(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]], float)


def Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], float)


_AXIS = {"x": Rx, "y": Ry, "z": Rz}


class DaeAnim:
    def __init__(self, path):
        ET.register_namespace("", NS_URI)
        self.path = path
        self.tree = ET.parse(path)
        self.root = self.tree.getroot()
        self.rest = {}        # bone -> 4x4 rest matrix
        self.out_src = {}     # bone -> output <source> float_array element
        self.parent = {}      # bone -> parent bone name (None for root)
        self.order = []       # bones in preorder (parents before children)
        self.n_keys = None
        self._index()
        self._hierarchy()

    def _hierarchy(self):
        def rec(node, par):
            if node.get("type") == "JOINT":
                nm = node.get("name")
                self.parent[nm] = par
                self.order.append(nm)
                par = nm
            for c in node:
                if _tag(c) == "node":
                    rec(c, par)
        for vs in self.root.iter():
            if _tag(vs) == "visual_scene":
                for n in vs:
                    if _tag(n) == "node":
                        rec(n, None)

    def fk_world(self, local_override=None):
        """World (model-frame) 4x4 per bone. local_override: dict bone->4x4 to
        use instead of the rest local for that bone."""
        ov = local_override or {}
        W = {}
        for b in self.order:
            L = ov.get(b, self.rest[b])
            p = self.parent.get(b)
            W[b] = W[p] @ L if p else L
        return W

    def _index(self):
        for node in self.root.iter():
            if _tag(node) == "node" and node.get("type") == "JOINT":
                m = node.find("c:matrix", NS)
                if m is not None:
                    self.rest[node.get("name")] = np.array(
                        [float(x) for x in m.text.split()], float).reshape(4, 4)
        # map each bone to its OUTPUT float_array via channel -> sampler -> source
        chan_target = {}
        for ch in self.root.iter():
            if _tag(ch) == "channel":
                tgt = ch.get("target", "")
                if tgt.endswith("/transform"):
                    chan_target[ch.get("source").lstrip("#")] = tgt.split("/")[0]
        samp_out = {}
        for s in self.root.iter():
            if _tag(s) == "sampler" and s.get("id") in chan_target:
                for inp in s:
                    if inp.get("semantic") == "OUTPUT":
                        samp_out[inp.get("source").lstrip("#")] = chan_target[s.get("id")]
        for src in self.root.iter():
            if _tag(src) == "source" and src.get("id") in samp_out:
                fa = src.find("c:float_array", NS)
                self.out_src[samp_out[src.get("id")]] = fa
                n = int(fa.get("count")) // 16
                self.n_keys = n if self.n_keys is None else self.n_keys

    def bones(self):
        return sorted(self.out_src)

    def set_bone_frames(self, bone, matrices):
        """matrices: array [F,4,4] row-major local transforms for this bone."""
        fa = self.out_src[bone]
        flat = np.asarray(matrices, float).reshape(-1)
        fa.text = " ".join(f"{v:.6f}" for v in flat)
        fa.set("count", str(flat.size))

    def rest_drive(self, bone, thetas, axis):
        """Return [F,4,4] = M_rest @ R(theta) for each theta (radians)."""
        Mr = self.rest[bone]
        R = _AXIS[axis]
        return np.stack([Mr @ R(t) for t in thetas], axis=0)

    def get_bone_frames(self, bone):
        """Original [F,4,4] local matrices for a bone (row-major)."""
        fa = self.out_src[bone]
        return np.array([float(x) for x in fa.text.split()]).reshape(-1, 4, 4)

    def rescale_time(self, target_duration):
        """Scale every animation INPUT (time) source so the loop lasts
        target_duration seconds (sets natural cadence). Returns old duration."""
        old = None
        for src in self.root.iter():
            if _tag(src) == "source" and (src.get("id") or "").endswith("-input"):
                fa = src.find("c:float_array", NS)
                if fa is None:
                    continue
                t = np.array([float(x) for x in fa.text.split()], float)
                if old is None:
                    old = float(t[-1])
                t = t * (target_duration / t[-1])
                fa.text = " ".join(f"{v:.6f}" for v in t)
        return old

    def save(self, out_path):
        self.tree.write(out_path, xml_declaration=True, encoding="utf-8")
