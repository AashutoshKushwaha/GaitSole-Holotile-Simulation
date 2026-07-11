"""
3D leg skeleton, posed from the predictor's joint angles, drawn as scene geoms.

Extends motion_predictor/animate_skeleton.fk (2D sagittal) to 3D: forward=+x,
left=+y, up=+z. Per leg the thigh/shank/foot directions come from hip_flexion,
knee_angle, ankle_angle (cumulative, as in the 2D version), with hip_adduction
tilting the leg in the frontal plane and pelvis tilt/list/rotation orienting the
whole pelvis. Segment lengths scale with height (Winter anthropometry, matching
animate_skeleton). Rendered as capsules + joint spheres on any mjvScene.
"""

import numpy as np
import mujoco

HEIGHT = 1.70
THIGH = 0.245 * HEIGHT
SHANK = 0.246 * HEIGHT
FOOT = 0.152 * HEIGHT
TRUNK = 0.34 * HEIGHT
HIP_HALF = 0.09          # half pelvis width

_BONE = np.array([0.85, 0.86, 0.90, 1.0])
_JOINT = np.array([0.95, 0.45, 0.20, 1.0])
_PELVIS = np.array([0.30, 0.55, 0.85, 1.0])


def _Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _leg(pose, side, hip, Rp):
    """Return (knee, ankle, toe) world points for one leg from `hip`."""
    hf = pose[f"hip_flexion_{side}"]
    ha = pose[f"hip_adduction_{side}"]
    kn = pose[f"knee_angle_{side}"]
    an = pose[f"ankle_angle_{side}"]
    add = _Rx(ha if side == "l" else -ha)   # adduction pulls leg toward midline
    th = hf
    sh = hf + kn
    fa = sh + an
    thigh_dir = Rp @ add @ np.array([np.sin(th), 0.0, -np.cos(th)])
    shank_dir = Rp @ add @ np.array([np.sin(sh), 0.0, -np.cos(sh)])
    foot_dir = Rp @ add @ np.array([np.cos(fa), 0.0, np.sin(fa)])
    knee = hip + THIGH * thigh_dir
    ankle = knee + SHANK * shank_dir
    toe = ankle + FOOT * foot_dir
    return knee, ankle, toe


def solve(pose, center_xy, floor_z):
    """Full skeleton joints in world frame; pelvis auto-raised so the lower foot
    sits on `floor_z`. Returns a dict of named 3D points."""
    Rp = _Rz(pose["pelvis_rotation"]) @ _Ry(pose["pelvis_tilt"]) @ _Rx(pose["pelvis_list"])
    # First solve with pelvis at origin to find how far down the feet reach.
    origin = np.zeros(3)
    hipL = origin + Rp @ np.array([0.0, HIP_HALF, 0.0])
    hipR = origin + Rp @ np.array([0.0, -HIP_HALF, 0.0])
    kL, aL, tL = _leg(pose, "l", hipL, Rp)
    kR, aR, tR = _leg(pose, "r", hipR, Rp)
    low = min(aL[2], aR[2], tL[2], tR[2])
    pelvis = np.array([center_xy[0], center_xy[1], floor_z - low])

    hipL = pelvis + Rp @ np.array([0.0, HIP_HALF, 0.0])
    hipR = pelvis + Rp @ np.array([0.0, -HIP_HALF, 0.0])
    kL, aL, tL = _leg(pose, "l", hipL, Rp)
    kR, aR, tR = _leg(pose, "r", hipR, Rp)
    chest = pelvis + Rp @ np.array([0.0, 0.0, TRUNK])
    head = chest + Rp @ np.array([0.0, 0.0, 0.12])
    return {"pelvis": pelvis, "hipL": hipL, "hipR": hipR,
            "kneeL": kL, "ankleL": aL, "toeL": tL,
            "kneeR": kR, "ankleR": aR, "toeR": tR,
            "chest": chest, "head": head,
            "footL": aL, "footR": aR}


def _bone(scene, a, b, width, rgba):
    if scene.ngeom >= scene.maxgeom:
        return
    g = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE,
                        np.zeros(3), np.zeros(3), np.zeros(9), rgba)
    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, width,
                         np.asarray(a, float), np.asarray(b, float))
    scene.ngeom += 1


def _ball(scene, c, r, rgba):
    if scene.ngeom >= scene.maxgeom:
        return
    mujoco.mjv_initGeom(scene.geoms[scene.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([r, 0, 0.0]), np.asarray(c, float),
                        np.eye(3).flatten(), rgba)
    scene.ngeom += 1


def draw(scene, joints):
    """Append the skeleton geoms to the scene."""
    j = joints
    _bone(scene, j["hipL"], j["hipR"], 0.035, _PELVIS)             # pelvis bar
    _bone(scene, j["pelvis"], j["chest"], 0.040, _PELVIS)          # trunk
    for s in ("L", "R"):
        _bone(scene, j[f"hip{s}"], j[f"knee{s}"], 0.030, _BONE)    # thigh
        _bone(scene, j[f"knee{s}"], j[f"ankle{s}"], 0.026, _BONE)  # shank
        _bone(scene, j[f"ankle{s}"], j[f"toe{s}"], 0.022, _BONE)   # foot
        _ball(scene, j[f"knee{s}"], 0.032, _JOINT)
        _ball(scene, j[f"ankle{s}"], 0.030, _JOINT)
    _ball(scene, j["head"], 0.09, _BONE)
    _ball(scene, j["pelvis"], 0.045, _JOINT)
