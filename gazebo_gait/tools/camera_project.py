#!/usr/bin/env python3
"""
Pinhole projection for the gait_world.sdf camera, so 3D world points can be drawn
onto the camera image (the tracking overlay).

Camera (from gait_world.sdf): pose 0 -3.2 0.95, yaw 1.5708 (looks along world +Y),
horizontal_fov 1.1, image 640x480. gz optical convention: camera +X = forward,
+Y = left, +Z = up; image u increases right (-Y), v increases down (-Z).
"""
import numpy as np

CAM_POS = np.array([0.0, -3.2, 0.95])
CAM_YAW = 1.5708
W, H = 640, 480
HFOV = 1.1
FX = (W / 2) / np.tan(HFOV / 2)
FY = FX
CX, CY = W / 2, H / 2

_c, _s = np.cos(CAM_YAW), np.sin(CAM_YAW)
# camera-to-world rotation (yaw about Z); world = R @ cam_local
_R = np.array([[_c, -_s, 0], [_s, _c, 0], [0, 0, 1]])
_RT = _R.T


def project(P):
    """World point [3] -> (u, v, visible). visible=False if behind the camera."""
    pc = _RT @ (np.asarray(P, float) - CAM_POS)     # camera frame: x fwd, y left, z up
    if pc[0] <= 1e-3:
        return 0.0, 0.0, False
    u = CX - FX * (pc[1] / pc[0])
    v = CY - FY * (pc[2] / pc[0])
    return float(u), float(v), True


def project_many(pts):
    """[N,3] -> ([N,2] uv, [N] visible)."""
    uv = np.zeros((len(pts), 2)); vis = np.zeros(len(pts), bool)
    for i, p in enumerate(pts):
        u, v, ok = project(p)
        uv[i] = (u, v); vis[i] = ok
    return uv, vis


# skeleton bone connectivity (indices into fk.KEYPOINT_NAMES)
# pelvis hip_r hip_l knee_r knee_l ankle_r ankle_l toe_r toe_l chest head
BONES = [(0, 1), (0, 2), (1, 3), (3, 5), (5, 7),
         (2, 4), (4, 6), (6, 8), (0, 9), (9, 10)]
