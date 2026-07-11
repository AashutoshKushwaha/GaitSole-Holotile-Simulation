"""
Rigid-spin reference demo (the "literal physics" illustration).

Unlike the production floor (which uses the moving-surface friction model because
fast rigid-spin contact is numerically unstable), this builds REAL tilted spinning
disk bodies with real friction and lets the genuine spin-drag carry the puck --
NO belt model. It only works at LOW spin (the contact pumps energy and launches
the foot at realistic speeds), so it is a labelled low-speed reference, not the
working sim. Close-up camera so the tilt + spin are clearly visible.

Run (Python312/torch env):
  python rigid_spin_demo.py --mp4 output/rigid_spin_demo.mp4 --seconds 6
  python rigid_spin_demo.py --live
  python rigid_spin_demo.py --frame          # one PNG to output/
"""

import argparse
import os

import numpy as np
import mujoco
import mujoco.viewer

import holotile_config as C
from mjcf_builder import build_model_xml
from sim_world import SimWorld

GRIP = (1.2, 0.01, 0.0001)     # grippy disk/foot friction for real drag
SPIN = 2.5                     # very low spin (stable regime); push azimuth = 0
SUBSTEPS = max(1, round((1.0 / C.CONTROL_HZ) / C.TIMESTEP))


def make_world():
    xml, meta = build_model_xml(2, 2, disk_friction=GRIP, puck_friction=GRIP, condim=3)
    w = SimWorld(xml, meta)
    w.set_all_tiles(0.0, 0.0)
    w.step(500)                 # settle puck onto the spinning-disk bed
    w.set_all_tiles(0.0, SPIN)  # real spin -> real friction drag (plain step)
    return w


def close_camera():
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.0, 0.0, C.SUPPORT_Z]
    cam.distance = 0.55
    cam.azimuth = 110.0
    cam.elevation = -22.0
    return cam


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mp4", default=None)
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--frame", action="store_true")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--fps", type=int, default=60)
    args = ap.parse_args()

    w = make_world()
    cam = close_camera()

    if args.frame or args.mp4:
        import imageio
        os.makedirs(C.OUTPUT_DIR, exist_ok=True)
        r = mujoco.Renderer(w.model, height=720, width=1280)
        if args.mp4:
            writer = imageio.get_writer(args.mp4, fps=args.fps, macro_block_size=None)
            frame_dt, nxt = 1.0 / args.fps, 0.0
            for _ in range(int(args.seconds / (1.0 / C.CONTROL_HZ))):
                w.step(SUBSTEPS)
                if w.time >= nxt:
                    r.update_scene(w.data, cam)
                    writer.append_data(r.render())
                    nxt += frame_dt
            writer.close()
            print(f"wrote {args.mp4}; final puck {w.puck_pos('puck_0')}")
        if args.frame:
            for _ in range(250):
                w.step(SUBSTEPS)
            r.update_scene(w.data, cam)
            imageio.imwrite(os.path.join(C.OUTPUT_DIR, "rigid_spin_frame.png"), r.render())
            print(f"wrote rigid_spin_frame.png; puck {w.puck_pos('puck_0')}")
        r.close()

    if args.live:
        with mujoco.viewer.launch_passive(w.model, w.data) as v:
            while v.is_running() and w.time < args.seconds:
                w.step(SUBSTEPS)
                v.sync()


if __name__ == "__main__":
    main()
