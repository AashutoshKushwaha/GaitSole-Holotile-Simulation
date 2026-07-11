#!/usr/bin/env python3
"""
Retarget one real Camargo gait cycle onto the SKINNED actor (smooth deformation,
no rigid seams) by reusing each leg bone's NATURAL hinge axis.

For each driven bone we read its original walk animation, extract the dominant
rotation axis it uses (in its rest-local frame) at peak flexion, then rebuild the
animation as  local = rest_local @ R(axis, SIGN*camargo_angle).  Because the axis
is the bone's own, the knee/hip/ankle bend in their correct planes (no distortion)
and the skin still deforms smoothly. Arms/spine/Hips keep their original motion.

Output: custom_gait.dae (skin+animation) + worlds/gait_world.sdf (actor). The
ground-truth pipeline is unchanged (gait_player streams the full Camargo trial).

Run: ~/venvs/gait/bin/python tools/make_gait_retarget.py
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from dae_anim import DaeAnim

sys.path.insert(0, ROOT)
import gait_config as GC
sys.path.insert(0, GC.MOTION_PREDICTOR_DIR)
import config as MC
import camargo

MESH_DIR = "/root/.gz/fuel/fuel.gazebosim.org/mingfei/models/actor/1/meshes"
SRC = os.path.join(MESH_DIR, "walk.dae")
OUT_DAE = os.path.join(MESH_DIR, "custom_gait.dae")
WORLD = os.path.join(ROOT, "worlds", "gait_world.sdf")
CAMARGO_CSV = os.path.join(GC.MOTION_PREDICTOR_DIR, "data", "camargo_csv")

# actor bone -> (camargo column, sign).  Sign maps the Camargo angle onto the
# bone's extracted natural axis in the anatomical direction (calibrated visually).
BONE = {
    "LeftUpLeg":  ("hip_flexion_l", +1.0),
    "LeftLeg":    ("knee_angle_l",  -1.0),
    "LeftFoot":   ("ankle_angle_l", +1.0),
    "RightUpLeg": ("hip_flexion_r", +1.0),
    "RightLeg":   ("knee_angle_r",  -1.0),
    "RightFoot":  ("ankle_angle_r", +1.0),
}


def axis_angle(R):
    """3x3 rotation -> (unit axis, angle>=0)."""
    ang = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
    if ang < 1e-6:
        return np.array([0, 0, 1.0]), 0.0
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    n = np.linalg.norm(ax)
    return (ax / n if n > 1e-9 else np.array([0, 0, 1.0])), ang


def rodrigues(axis, ang):
    a = axis / np.linalg.norm(axis)
    c, s = np.cos(ang), np.sin(ang)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + s * K + (1 - c) * K @ K


def dominant_axis(rest_R, frames):
    """Hinge axis a bone uses, from its original animation (rest-local frame)."""
    invrest = rest_R.T
    best_ax, best_ang = np.array([1., 0, 0]), 0.0
    for M in frames:
        Rrel = invrest @ M[:3, :3]
        ax, ang = axis_angle(Rrel)
        if ang > best_ang:                 # axis at peak rotation = the hinge
            best_ang, best_ax = ang, ax
    return best_ax


def detect_cycle(tr):
    g = tr["grf_y_r"]; contact = g > 0.05
    rising = np.where((~contact[:-1]) & contact[1:])[0] + 1
    return (int(rising[1]), int(rising[2])) if len(rising) >= 3 else (0, int(GC.FPS * 1.1))


def main():
    tr = camargo.load_camargo(CAMARGO_CSV)[0]
    i0, i1 = detect_cycle(tr); dur = (i1 - i0) / GC.FPS
    print(f"gait cycle frames {i0}..{i1} ({dur:.2f}s)")

    d = DaeAnim(SRC)
    N = d.n_keys
    src_idx = i0 + np.linspace(0, 1, N) * (i1 - i0)
    cyc = np.arange(i0, i1 + 1)

    driven = {}
    for bone, (col, sign) in BONE.items():
        rest_R = d.rest[bone][:3, :3]
        rest_t = d.rest[bone][:3, 3]
        axis = dominant_axis(rest_R, d.get_bone_frames(bone))
        theta = sign * np.interp(src_idx, cyc, tr[col][i0:i1 + 1])
        frames = np.zeros((N, 4, 4))
        for k in range(N):
            frames[k, :3, :3] = rest_R @ rodrigues(axis, theta[k])
            frames[k, :3, 3] = rest_t
            frames[k, 3, 3] = 1.0
        driven[bone] = frames
        d.set_bone_frames(bone, frames)
        print(f"  {bone:11s} axis={np.round(axis,2)}  {col} x{sign:+.0f}")

    # --- ground the feet via the TRAJECTORY height (gz actors position the body
    # from the trajectory, not the root bone). Foot height comes from FK of the
    # actor's own retargeted skeleton; the per-frame variation is tiny (~2 cm), so
    # a constant trajectory z that clears the deepest foot keeps it grounded.
    FOOT = ["LeftFoot", "LeftToeBase", "RightFoot", "RightToeBase"]
    CLEAR = 0.04                                   # sole offset below the joints
    lows = []
    for k in range(N):
        W = d.fk_world({b: driven[b][k] for b in driven})   # Hips at rest origin
        lows.append(min(W[b][2, 3] for b in FOOT))
    TRAJ_Z = -min(lows) + CLEAR                    # lift so deepest foot ~ floor
    print(f"grounding: foot_z {min(lows):.3f}..{max(lows):.3f} -> trajectory z={TRAJ_Z:.3f}")

    old = d.rescale_time(dur)
    print(f"rescaled loop {old:.2f}s -> {dur:.2f}s")
    d.save(OUT_DAE)
    print("wrote", OUT_DAE)

    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="gait">
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular><direction>-0.5 0.3 -0.9</direction></light>
    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry>
        <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual>
      </link></model>
    <model name="cam"><static>true</static><pose>0 -3.2 0.95 0 0 1.5708</pose>
      <link name="link"><sensor name="camera" type="camera">
        <camera><horizontal_fov>1.1</horizontal_fov>
          <image><width>640</width><height>480</height></image>
          <clip><near>0.1</near><far>50</far></clip></camera>
        <always_on>1</always_on><update_rate>30</update_rate><topic>camera</topic></sensor>
      </link></model>
    <actor name="walker">
      <skin><filename>{OUT_DAE}</filename><scale>1.0</scale></skin>
      <animation name="custom"><filename>{OUT_DAE}</filename><interpolate_x>true</interpolate_x></animation>
      <script><loop>true</loop><auto_start>true</auto_start>
        <trajectory id="0" type="custom">
          <waypoint><time>0</time><pose>0 0 {TRAJ_Z:.3f} 0 0 0</pose></waypoint>
          <waypoint><time>10</time><pose>0 0 {TRAJ_Z:.3f} 0 0 0</pose></waypoint>
        </trajectory></script>
    </actor>
  </world>
</sdf>
"""
    with open(WORLD, "w") as f:
        f.write(sdf)
    print("wrote", WORLD)


if __name__ == "__main__":
    main()
