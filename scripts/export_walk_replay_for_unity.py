"""
Export OpenSim 2D walk for Unity (flattened JSON for JsonUtility).

Run:
  E:\\conda\\envs\\opensim_env\\python.exe scripts/export_walk_replay_for_unity.py
"""
import json
import os
import re

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MOTION = os.path.join(ROOT, "output", "walk_motion.sto")
GRF = os.path.join(ROOT, "output", "walk_GRF.sto")
OUT_DIR = os.path.join(ROOT, "holotile_unity", "Assets", "StreamingAssets", "OpenSim")
OUT = os.path.join(OUT_DIR, "walk_replay.json")

CONTACT_PREFIX = {
    "contactHeel": "Heel",
    "contactMidfoot": "Midfoot",
    "contactForefoot": "Forefoot",
    "contactToe": "Toe",
}
SIDES = ("r", "l")


def read_sto(path):
    with open(path, "r", encoding="utf-8") as fh:
        lines = fh.readlines()
    header_end = next(i for i, l in enumerate(lines) if l.strip() == "endheader")
    labels = lines[header_end + 1].strip().split()
    rows = []
    for line in lines[header_end + 2 :]:
        line = line.strip()
        if not line:
            continue
        rows.append([float(x) for x in line.split()])
    return labels, np.array(rows, dtype=np.float64)


def short_coord_label(label):
    if label == "time":
        return label
    m = re.search(r"/([^/]+)/value$", label)
    return m.group(1) if m else label


def main():
    m_labels, m_data = read_sto(MOTION)
    g_labels, g_data = read_sto(GRF)

    m_short = [short_coord_label(l) for l in m_labels]
    m_idx = {m_short[i]: i for i in range(len(m_short))}

    n = len(m_data)
    out = {
        "source": "2D_gait_4regions walk_motion + walk_GRF",
        "frameCount": n,
        "duration": float(m_data[-1, m_idx["time"]] - m_data[0, m_idx["time"]]),
        "height_m": 1.70,
        "time": m_data[:, m_idx["time"]].tolist(),
    }

    coord_keys = [k for k in m_idx if k != "time"]
    for k in coord_keys:
        out[k] = m_data[:, m_idx[k]].tolist()

    g_idx = {g_labels[i]: i for i in range(len(g_labels))}
    for side in SIDES:
        for _, rname in CONTACT_PREFIX.items():
            for comp in ("Fx", "Fy", "Fz"):
                col = f"contact{rname}_{side}.{comp}"
                key = f"grf_{rname.lower()}_{side}_{comp.lower()}"
                if col in g_idx:
                    out[key] = g_data[:, g_idx[col]].tolist()

    os.makedirs(OUT_DIR, exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as fh:
        json.dump(out, fh)
    print(f"Wrote {OUT}  ({n} frames, {out['duration']:.3f}s)")


if __name__ == "__main__":
    main()
