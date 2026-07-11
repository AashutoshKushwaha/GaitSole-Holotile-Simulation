#!/usr/bin/env python3
"""
Proof-of-concept: edit the actor's knee bones to a STATIC bent pose across all
keyframes, save a custom .dae next to walk.dae (so its textures still resolve),
and emit a world that renders it. Confirms the edit->render loop and lets us
calibrate the knee axis/sign before driving real Camargo angles.

Run: ~/venvs/gait/bin/python tools/make_static_pose.py [knee_rad]
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dae_anim import DaeAnim

MESH_DIR = "/root/.gz/fuel/fuel.gazebosim.org/mingfei/models/actor/1/meshes"
SRC = os.path.join(MESH_DIR, "walk.dae")
OUT_DAE = os.path.join(MESH_DIR, "custom_static.dae")
WORLD = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "worlds", "static_pose_world.sdf")


def main():
    knee = float(sys.argv[1]) if len(sys.argv) > 1 else 1.4
    d = DaeAnim(SRC)
    n = d.n_keys
    print(f"loaded {SRC}: {len(d.bones())} animated bones, {n} keyframes")
    for bone in ("LeftLeg", "RightLeg"):
        thetas = np.full(n, knee)
        d.set_bone_frames(bone, d.rest_drive(bone, thetas, "x"))
    d.save(OUT_DAE)
    print(f"wrote {OUT_DAE}  (knees bent {np.rad2deg(knee):.0f} deg about local x)")

    sdf = f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="static_pose">
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
    <model name="cam"><static>true</static><pose>2.5 -2.5 1.3 0 0.12 2.356</pose>
      <link name="link"><sensor name="camera" type="camera">
        <camera><horizontal_fov>1.2</horizontal_fov>
          <image><width>640</width><height>480</height></image>
          <clip><near>0.1</near><far>50</far></clip></camera>
        <always_on>1</always_on><update_rate>30</update_rate><topic>camera</topic></sensor>
      </link></model>
    <actor name="walker">
      <skin><filename>{OUT_DAE}</filename><scale>1.0</scale></skin>
      <animation name="custom"><filename>{OUT_DAE}</filename><interpolate_x>true</interpolate_x></animation>
      <script><loop>true</loop><auto_start>true</auto_start>
        <trajectory id="0" type="custom">
          <waypoint><time>0</time><pose>2 0 1.0 0 0 1.57</pose></waypoint>
          <waypoint><time>10</time><pose>2 0 1.0 0 0 1.57</pose></waypoint>
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
