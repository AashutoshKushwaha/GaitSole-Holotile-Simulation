"""
M1 physics proof: does a tile of tilted, spinning disks drag a puck in a
controllable direction?

Builds a single tile + one puck, lets the puck settle, then for several
(azimuth, spin) commands measures the puck's travel heading and speed. The make-
or-break checks:
  * the puck actually moves (drag works at all),
  * reversing spin reverses the heading (~180 deg flip),
  * stepping azimuth by 90 deg steps the heading by ~90 deg (controllable).

Run (Python312/torch env):
  python check_m1.py            # headless numeric proof
  python check_m1.py --view     # interactive viewer (watch one push)
"""

import argparse
import math

import numpy as np

import holotile_config as C
from mjcf_builder import build_model_xml
from sim_world import SimWorld


def settle(world, steps=600):
    world.set_all_tiles(0.0, 0.0)
    world.step(steps)


def run_push(azimuth, spin, hold_s=0.5):
    """Fresh world; settle; apply (azimuth, spin); return (heading_deg, speed).

    Uses a 3x3-tile floor so the puck has room to translate. The disks are first
    RE-ORIENTED to `azimuth` with spin off (then allowed to settle), so the steady
    drag is measured without the transient of whipping the azimuth around under
    spin. In the real controller this is what azimuth slew-limiting achieves.
    """
    xml, meta = build_model_xml(3, 3)
    world = SimWorld(xml, meta)
    settle(world)
    world.set_all_tiles(azimuth, 0.0)      # reorient first, no drive
    world.step_driven(400)                  # let azimuth + puck settle
    p0 = world.puck_pos("puck_0")
    world.set_all_tiles(azimuth, spin)      # now drive -> measure steady drag
    world.step_driven(int(hold_s / C.TIMESTEP))
    p1 = world.puck_pos("puck_0")
    d = p1 - p0
    disp = d[:2]
    dist = float(np.linalg.norm(disp))
    heading = math.degrees(math.atan2(disp[1], disp[0])) if dist > 1e-4 else float("nan")
    return heading, dist / hold_s, p1[2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", action="store_true")
    args = ap.parse_args()

    print(f"geometry: tile {C.TILE_SIZE} m, {C.DISKS_PER_TILE}x{C.DISKS_PER_TILE} "
          f"disks r={C.DISK_RADIUS*1000:.0f} mm, tilt {C.TILT_DEG} deg, "
          f"support_z={C.SUPPORT_Z*1000:.0f} mm")
    print(f"drag_per_omega={C.DRAG_PER_OMEGA*1000:.2f} mm/(rad/s)  "
          f"(spin {C.SPIN_MAX} rad/s -> surface ~{C.SPIN_MAX*C.DRAG_PER_OMEGA:.2f} m/s)\n")

    if args.view:
        import mujoco.viewer
        xml, meta = build_model_xml(3, 3)
        world = SimWorld(xml, meta)
        settle(world)
        world.set_all_tiles(0.0, 24.0)
        with mujoco.viewer.launch_passive(world.model, world.data) as v:
            while v.is_running():
                world.step_driven(10)
                v.sync()
        return

    spin = 24.0
    print(f"{'azimuth_cmd':>11} {'spin':>6} {'heading':>9} {'speed(m/s)':>11} {'puck_z(mm)':>11}")
    headings = {}
    for azi_deg in (0, 90, 180, 270):
        azi = math.radians(azi_deg)
        h, s, z = run_push(azi, spin)
        headings[azi_deg] = h
        print(f"{azi_deg:>9}   {spin:>6.0f} {h:>9.1f} {s:>11.3f} {z*1000:>11.1f}")

    # reverse spin at azimuth 0
    h_rev, s_rev, _ = run_push(0.0, -spin)
    print(f"{0:>9}   {-spin:>6.0f} {h_rev:>9.1f} {s_rev:>11.3f}")

    # --- verdict ---
    print("\n--- M1 verdict ---")
    moved = all(not math.isnan(headings[a]) for a in headings)
    print(f"puck moves under all azimuths:           {moved}")
    if not math.isnan(headings[0]) and not math.isnan(h_rev):
        flip = abs(((h_rev - headings[0] + 180) % 360) - 180)
        print(f"spin reversal flips heading ~180 deg:     {flip:.0f} deg")
    if moved:
        # push direction should equal the commanded azimuth (belt model).
        errs = [abs(((headings[a] - a + 180) % 360) - 180) for a in (0, 90, 180, 270)]
        print(f"heading error vs commanded azimuth:       "
              f"{[round(e) for e in errs]} deg (expect all ~0)")


if __name__ == "__main__":
    main()
