"""
M3 demo: the trained predictor drives a 3D walking skeleton on the HoloTile floor.

Streams a gait through PredictorBridge, poses the 3D skeleton from the predicted
joint angles each frame, and renders it standing on a real spinning-disk floor.
(The floor->foot dynamic coupling and centering control come in M4; here the disks
spin to show the live floor and the skeleton walks from the model's predictions.)

Run (Python312/torch env):
  python run_walk.py --frame
  python run_walk.py --mp4 output/holotile_walk.mp4 --seconds 8
  python run_walk.py --live
"""

import argparse
import math
import os

import numpy as np
import mujoco
import mujoco.viewer

import holotile_config as C
from mjcf_builder import build_model_xml
from sim_world import SimWorld
from predictor_bridge import PredictorBridge
import data as MD          # motion_predictor data (sys.path set by predictor_bridge)
import skeleton3d

SUB = max(1, round((1.0 / C.CONTROL_HZ) / C.TIMESTEP))


def make_world(tiles):
    xml, meta = build_model_xml(tiles, tiles, pucks=[])   # real disks, no foot pucks (M3)
    w = SimWorld(xml, meta)
    w.step(200)
    return w


def make_camera(tiles):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.0, 0.0, 0.85]
    cam.distance = max(3.4, tiles * (C.TILE_SIZE + C.TILE_GAP) * 1.6)
    cam.azimuth = 130.0
    cam.elevation = -10.0
    return cam


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mp4", default=None)
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--frame", action="store_true")
    ap.add_argument("--tiles", type=int, default=3)
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--fps", type=int, default=60)
    args = ap.parse_args()

    w = make_world(args.tiles)
    br = PredictorBridge()
    br.load_trial(MD.make_synthetic_trial(rng=np.random.default_rng(3)))
    cam = make_camera(args.tiles)

    def control_tick():
        # keep the floor visibly alive: disks spin, push azimuth drifts slowly
        azi = 0.4 * math.sin(0.5 * w.time)
        w.set_all_tiles(azi, 14.0)
        res = control_tick.last = br.step()
        w.step_driven(SUB)
        return res

    def draw(scene):
        pose = br.pose_dict(control_tick.last["pose_now"])
        j = skeleton3d.solve(pose, (0.0, 0.0), C.SUPPORT_Z)
        skeleton3d.draw(scene, j)

    control_tick.last = br.step()

    if args.frame or args.mp4:
        import imageio
        os.makedirs(C.OUTPUT_DIR, exist_ok=True)
        r = mujoco.Renderer(w.model, height=720, width=1280)
        if args.frame:
            for _ in range(120):
                control_tick()
            r.update_scene(w.data, cam)
            draw(r.scene)
            imageio.imwrite(os.path.join(C.OUTPUT_DIR, "walk_frame.png"), r.render())
            print("wrote walk_frame.png")
        if args.mp4:
            writer = imageio.get_writer(args.mp4, fps=args.fps, macro_block_size=None)
            frame_dt, nxt = 1.0 / args.fps, w.time
            for _ in range(int(args.seconds / (1.0 / C.CONTROL_HZ))):
                control_tick()
                if w.time >= nxt:
                    r.update_scene(w.data, cam)
                    draw(r.scene)
                    writer.append_data(r.render())
                    nxt += frame_dt
            writer.close()
            print(f"wrote {args.mp4}")
        r.close()

    if args.live:
        with mujoco.viewer.launch_passive(w.model, w.data) as v:
            while v.is_running() and w.time < args.seconds:
                control_tick()
                v.user_scn.ngeom = 0
                draw(v.user_scn)
                v.sync()


if __name__ == "__main__":
    main()
