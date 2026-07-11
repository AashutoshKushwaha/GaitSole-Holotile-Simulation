"""
HoloTile demo driver.

M2 scope: a presentable production floor (frictionless tile pads + visual disk
overlay) with the moving-surface drive. Until the real controller lands (M4),
the tiles follow a scripted command -- the push azimuth slowly rotates while the
disks spin -- so you can watch the disks tilt, re-orient and spin while the floor
carries the foot in a curving path. Renders an MP4 and/or a live viewer.

Run (Python312/torch env):
  python run_demo.py --mp4 output/holotile_m2.mp4 --seconds 10
  python run_demo.py --live --seconds 30
"""

import argparse
import math
import os

import numpy as np

import holotile_config as C
from mjcf_builder import build_floor_xml
from sim_world import SimWorld
from disk_overlay import DiskOverlay

CONTROL_DT = 1.0 / C.CONTROL_HZ
SUBSTEPS = max(1, round(CONTROL_DT / C.TIMESTEP))


def scripted_command(t):
    """Demo command: push direction rotates (0.1 Hz), constant spin."""
    azimuth = 2.0 * math.pi * 0.1 * t
    spin = 30.0
    return azimuth, spin


def make_camera(tiles):
    cam = __import__("mujoco").MjvCamera()
    cam.type = __import__("mujoco").mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.0, 0.0, C.SUPPORT_Z]
    cam.distance = (tiles * (C.TILE_SIZE + C.TILE_GAP)) * 1.6
    cam.azimuth = 120.0
    cam.elevation = -28.0
    return cam


def run_mp4(world, overlay, seconds, path, fps, width, height):
    import mujoco
    import imageio

    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    renderer = mujoco.Renderer(world.model, height=height, width=width)
    cam = make_camera(world.grid["nx"])
    writer = imageio.get_writer(path, fps=fps, macro_block_size=None)

    frame_dt = 1.0 / fps
    next_frame = 0.0
    n_ticks = int(seconds / CONTROL_DT)
    for _ in range(n_ticks):
        azi, spin = scripted_command(world.time)
        world.set_all_tiles(azi, spin)
        world.step_driven(SUBSTEPS)
        if world.time >= next_frame:
            renderer.update_scene(world.data, cam)
            overlay.update(renderer.scene, world.tile_cmd, world.time)
            writer.append_data(renderer.render())
            next_frame += frame_dt
    writer.close()
    renderer.close()
    print(f"wrote {path}  ({seconds:.0f}s @ {fps} fps, {width}x{height})")


def run_live(world, overlay, seconds):
    import mujoco
    import mujoco.viewer

    with mujoco.viewer.launch_passive(world.model, world.data) as v:
        while v.is_running() and world.time < seconds:
            azi, spin = scripted_command(world.time)
            world.set_all_tiles(azi, spin)
            world.step_driven(SUBSTEPS)
            v.user_scn.ngeom = 0
            overlay.update(v.user_scn, world.tile_cmd, world.time)
            v.sync()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mp4", default=None, help="output MP4 path")
    ap.add_argument("--live", action="store_true", help="open interactive viewer")
    ap.add_argument("--tiles", type=int, default=C.FLOOR_TILES_X)
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--fps", type=int, default=60)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    args = ap.parse_args()

    xml, meta = build_floor_xml(args.tiles, args.tiles)
    world = SimWorld(xml, meta)
    world.step(300)  # settle feet onto the pads
    overlay = DiskOverlay(world.grid)
    print(f"floor {args.tiles}x{args.tiles} tiles, "
          f"{len(overlay.tiles)*overlay.per_tile**2} visual disks, "
          f"{C.TIMESTEP*1000:.0f} ms step, {SUBSTEPS} substeps/tick")

    if args.mp4:
        run_mp4(world, overlay, args.seconds, args.mp4, args.fps, args.width, args.height)
    if args.live:
        run_live(world, overlay, args.seconds)
    if not args.mp4 and not args.live:
        print("nothing to do: pass --mp4 PATH and/or --live")


if __name__ == "__main__":
    main()
