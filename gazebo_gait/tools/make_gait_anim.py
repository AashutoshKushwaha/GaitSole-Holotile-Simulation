#!/usr/bin/env python3
"""
Drive the skinned actor's leg bones from one real Camargo gait cycle.

Keeps the actor's 139-keyframe timeline (and original arm/spine swing) intact and
only overwrites the leg-bone OUTPUT matrices: one detected Camargo gait cycle
(right heel-strike to right heel-strike) is resampled onto the 139 keyframes, and
each leg bone gets M_i = M_rest @ prod(R(angle, axis)) per the CALIB table.

Ground truth still comes from the full Camargo trial via gait_player (this is just
the visual), so the actor and the pipeline use the same data.

Run: ~/venvs/gait/bin/python tools/make_gait_anim.py
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from dae_anim import DaeAnim, Rx, Ry, Rz

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

_AX = {"x": Rx, "y": Ry, "z": Rz}

# bone -> list of (camargo_col, axis, sign). Applied right-multiplied in order.
# Knee confirmed: local x, and Camargo knee is negative-for-flexion so sign=-1
# turns it into the +fold that the static test proved correct. Hip/ankle signs
# are first guesses, calibrated by rendering.
CALIB = {
    "LeftUpLeg":  [("hip_flexion_l", "x", +1.0), ("hip_adduction_l", "z", +1.0)],
    "LeftLeg":    [("knee_angle_l", "x", -1.0)],
    "LeftFoot":   [("ankle_angle_l", "x", +1.0)],
    "RightUpLeg": [("hip_flexion_r", "x", +1.0), ("hip_adduction_r", "z", -1.0)],
    "RightLeg":   [("knee_angle_r", "x", -1.0)],
    "RightFoot":  [("ankle_angle_r", "x", +1.0)],
}


def detect_cycle(tr):
    """Return (i0, i1) frame indices of one right-foot gait cycle (HS to HS)."""
    g = tr["grf_y_r"]
    contact = g > 0.05
    rising = np.where((~contact[:-1]) & contact[1:])[0] + 1
    if len(rising) >= 3:
        return int(rising[1]), int(rising[2])      # a clean mid-trial stride
    return 0, min(len(g) - 1, int(GC.FPS * 1.1))    # fallback ~1.1 s


def main():
    trials = camargo.load_camargo(CAMARGO_CSV)
    tr = trials[0]
    i0, i1 = detect_cycle(tr)
    dur = (i1 - i0) / GC.FPS
    print(f"gait cycle: frames {i0}..{i1}  ({dur:.2f} s)")

    d = DaeAnim(SRC)
    N = d.n_keys
    frac = np.linspace(0, 1, N)
    src_idx = i0 + frac * (i1 - i0)                 # fractional camargo frames

    # resample each needed angle column onto the keyframes
    cyc = np.arange(i0, i1 + 1)
    ang = {}
    for bone, rots in CALIB.items():
        for col, _, _ in rots:
            if col not in ang:
                ang[col] = np.interp(src_idx, cyc, tr[col][i0:i1 + 1])

    for bone, rots in CALIB.items():
        Mr = d.rest[bone]
        frames = np.empty((N, 4, 4))
        for k in range(N):
            M = Mr.copy()
            for col, axis, sign in rots:
                M = M @ _AX[axis](sign * ang[col][k])
            frames[k] = M
        d.set_bone_frames(bone, frames)
        cols = ", ".join(c for c, _, _ in rots)
        print(f"  drove {bone:11s} from [{cols}]")

    old = d.rescale_time(dur)
    print(f"rescaled animation loop {old:.2f}s -> {dur:.2f}s (natural cadence)")
    d.save(OUT_DAE)
    print(f"wrote {OUT_DAE}")

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
          <waypoint><time>0</time><pose>0 0 0.97 0 0 0</pose></waypoint>
          <waypoint><time>10</time><pose>0 0 0.97 0 0 0</pose></waypoint>
        </trajectory></script>
    </actor>
  </world>
</sdf>
"""
    with open(WORLD, "w") as f:
        f.write(sdf)
    print(f"wrote {WORLD}")


if __name__ == "__main__":
    main()
