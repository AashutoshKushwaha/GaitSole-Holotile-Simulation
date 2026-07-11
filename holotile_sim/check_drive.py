"""
Verify the foot is driven by the disks' ACTUAL rotation (causal physics):
  * spin the real disks to push +x  -> foot accelerates +x at ~surface speed,
  * stop the disks                  -> foot decelerates (no drive),
  * the foot's speed tracks the disks' measured surface speed.
Also reports wall-clock cost of a real-disk floor.
"""
import time
import math
import numpy as np

import holotile_config as C
from mjcf_builder import build_model_xml
from sim_world import SimWorld

# push +x : v_surf dir = (sin a, -cos a) = (1,0) when a = 90 deg
A_PUSH = math.pi / 2
SPIN = 24.0
SUB = max(1, round((1.0 / C.CONTROL_HZ) / C.TIMESTEP))

xml, meta = build_model_xml(4, 4)          # real spinning disks, frictionless support
w = SimWorld(xml, meta)
w.set_all_tiles(0.0, 0.0)
w.step(500)
print(f"real-disk floor 4x4 = {4*4*C.DISKS_PER_TILE**2} disks; "
      f"surface speed @spin{SPIN:.0f} = {C.DRAG_PER_OMEGA*SPIN:.3f} m/s")

# --- drive +x ---
w.set_all_tiles(A_PUSH, SPIN)
p0 = w.puck_pos("puck_0").copy()
t0 = time.perf_counter()
for k in range(10):
    w.step_driven(SUB * 10)   # 0.1 s
    v = w.puck_vel("puck_0")
    vs = w.surface_velocity(w.tile_of_xy(*w.puck_pos("puck_0")[:2]))
    print(f" drive  t={w.time:.2f}  foot v=({v[0]:+.3f},{v[1]:+.3f})  "
          f"disk surface=({vs[0]:+.3f},{vs[1]:+.3f})")
wall = time.perf_counter() - t0
disp = w.puck_pos("puck_0") - p0
print(f" -> displacement ({disp[0]:+.3f},{disp[1]:+.3f}) m  (expect +x)")

# --- stop disks: foot should coast down, not keep driving ---
w.set_all_tiles(A_PUSH, 0.0)
for k in range(6):
    w.step_driven(SUB * 10)
    v = w.puck_vel("puck_0")
    print(f" stop   t={w.time:.2f}  foot v=({v[0]:+.3f},{v[1]:+.3f})")

sim_seconds = 1.0
print(f"\nperf: 1.0 s of 4x4 real-disk sim took {wall:.2f} s wall "
      f"({sim_seconds/wall:.1f}x realtime)")
