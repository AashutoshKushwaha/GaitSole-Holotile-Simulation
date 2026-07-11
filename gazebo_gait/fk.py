"""
Forward kinematics of the capsule humanoid's leg chain, matching build_human_sdf.py
EXACTLY (same segment lengths + same joint axes/order), so we can compute the
pelvis height that keeps the lower foot on the floor each frame (grounding + bob).

Chain per leg (commanded gz angles): F1 = Ry(hip_flexion) @ Rx(hip_adduction);
knee = F1·(0,0,-THIGH); F2 = F1·Ry(knee); ankle = knee + F2·(0,0,-SHANK);
F3 = F2·Ry(ankle); toe = ankle + F3·(FOOT,0,0). Hip sits at pelvis z, so the z of
each joint relative to the pelvis is just its computed z.
"""
import numpy as np

from build_human_sdf import THIGH, SHANK, FOOT, R_FOOT

# Anthropometry for keypoint FK (matches holotile_sim/skeleton3d.py).
HIP_HALF = 0.09
TRUNK = 0.34 * 1.70
# Ordered 3D keypoints the perception/solver pipeline uses.
KEYPOINT_NAMES = ["pelvis", "hip_r", "hip_l", "knee_r", "knee_l",
                  "ankle_r", "ankle_l", "toe_r", "toe_l", "chest", "head"]


def _Ry(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _Rx(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _Rz(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _leg_pts(pose, side, hip, Rp):
    hf, ha = pose[f"hip_flexion_{side}"], pose[f"hip_adduction_{side}"]
    kn, an = pose[f"knee_angle_{side}"], pose[f"ankle_angle_{side}"]
    add = _Rx(ha if side == "l" else -ha)
    th, sh, fa = hf, hf + kn, hf + kn + an
    thigh = Rp @ add @ np.array([np.sin(th), 0, -np.cos(th)])
    shank = Rp @ add @ np.array([np.sin(sh), 0, -np.cos(sh)])
    foot = Rp @ add @ np.array([np.cos(fa), 0, np.sin(fa)])
    knee = hip + THIGH * thigh
    ankle = knee + SHANK * shank
    toe = ankle + FOOT * foot
    return knee, ankle, toe


def keypoints(pose):
    """3D keypoints (forward +x, left +y, up +z) from the 12 pose angles (rad),
    pelvis placed at pelvis_ty height. Returns [K,3] in KEYPOINT_NAMES order.
    Inverse is solve_angles() (exact round-trip)."""
    Rp = _Rz(pose["pelvis_rotation"]) @ _Ry(pose["pelvis_tilt"]) @ _Rx(pose["pelvis_list"])
    pelvis = np.array([0.0, 0.0, pose["pelvis_ty"]])
    hipL = pelvis + Rp @ np.array([0, HIP_HALF, 0]); hipR = pelvis + Rp @ np.array([0, -HIP_HALF, 0])
    kL, aL, tL = _leg_pts(pose, "l", hipL, Rp)
    kR, aR, tR = _leg_pts(pose, "r", hipR, Rp)
    chest = pelvis + Rp @ np.array([0, 0, TRUNK]); head = chest + Rp @ np.array([0, 0, 0.12])
    return np.array([pelvis, hipR, hipL, kR, kL, aR, aL, tR, tL, chest, head])


def solve_angles(kp):
    """Inverse of keypoints(): recover the 12 pose angles (dict) from [K,3] keypoints.
    This is the perception->angle 'IK' the predictor consumes."""
    d = dict(zip(KEYPOINT_NAMES, kp))
    pelvis, chest = d["pelvis"], d["chest"]

    # pelvis frame: z=up (pelvis->chest), y=left (hipR->hipL), x=fwd
    z = chest - pelvis; z /= np.linalg.norm(z)
    y = d["hip_l"] - d["hip_r"]; y -= z * (y @ z); y /= np.linalg.norm(y)
    x = np.cross(y, z)
    Rp = np.column_stack([x, y, z])
    out = {
        "pelvis_rotation": np.arctan2(Rp[1, 0], Rp[0, 0]),
        "pelvis_tilt": np.arctan2(-Rp[2, 0], np.hypot(Rp[2, 1], Rp[2, 2])),
        "pelvis_list": np.arctan2(Rp[2, 1], Rp[2, 2]),
        "pelvis_ty": float(pelvis[2]),
    }
    for side, s in (("r", -1.0), ("l", 1.0)):
        hip, knee = d[f"hip_{side}"], d[f"knee_{side}"]
        ankle, toe = d[f"ankle_{side}"], d[f"toe_{side}"]
        thigh = Rp.T @ (knee - hip)                      # local: Rx(s*ha)@[sin hf,0,-cos hf]
        alpha = np.arctan2(thigh[1], -thigh[2])          # = s*ha
        de = _Rx(-alpha)                                 # de-rotate adduction -> sagittal
        ts, ss, fs = de @ thigh, de @ (Rp.T @ (ankle - knee)), de @ (Rp.T @ (toe - ankle))
        hf = np.arctan2(ts[0], -ts[2])
        sh = np.arctan2(ss[0], -ss[2])
        fa = np.arctan2(fs[2], fs[0])
        out[f"hip_flexion_{side}"] = hf
        out[f"hip_adduction_{side}"] = alpha * s
        out[f"knee_angle_{side}"] = sh - hf
        out[f"ankle_angle_{side}"] = fa - sh
    return out


def _leg_min_z(hf, ha, kn, an):
    F1 = _Ry(hf) @ _Rx(ha)
    knee = F1 @ np.array([0, 0, -THIGH])
    F2 = F1 @ _Ry(kn)
    ankle = knee + F2 @ np.array([0, 0, -SHANK])
    F3 = F2 @ _Ry(an)
    toe = ankle + F3 @ np.array([FOOT, 0, 0])
    return min(knee[2], ankle[2], toe[2])


def pelvis_height(cmd, clearance=None):
    """cmd: dict of commanded gz joint angles (post-sign). Returns pelvis z so the
    lowest foot point rests on z=0."""
    clr = R_FOOT if clearance is None else clearance
    low = min(_leg_min_z(cmd[f"hip_flexion_{s}"], cmd[f"hip_adduction_{s}"],
                         cmd[f"knee_angle_{s}"], cmd[f"ankle_angle_{s}"])
              for s in ("r", "l"))
    return clr - low
