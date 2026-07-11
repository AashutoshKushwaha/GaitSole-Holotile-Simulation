"""
M4: the floor keeps a walking person centered, via genuine disk->foot physics.

Two-foot stance/swing model (user's choice):
  * A footstep planner makes the person "walk" at the commanded intended velocity
    v_cmd(t): the body vaults forward over the planted (stance) foot while the
    swing foot lifts and re-plants ahead.
  * The STANCE foot is a physical puck the disks actually drive (the verified
    causal belt drive). The floor controller commands the disks so the stance
    foot tracks  center + foot_rel_stance, i.e. it is carried backward at exactly
    the body's forward progress -> the pelvis stays centered.
  * The SWING foot is placed kinematically along its step arc.
  * The pelvis is derived from the real stance-foot position, so if the floor
    can't keep up (saturation) the person genuinely drifts -- nothing is faked.

With the controller ON the person stays centered; OFF they walk off the floor.

Run (Python312/torch env):
  python run_control.py --compare              # headless ON-vs-OFF drift + plot
  python run_control.py --mp4 output/holotile_control.mp4 --seconds 10
  python run_control.py --live
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
import data as MD                       # motion_predictor (sys.path set by bridge)
from intended_velocity import IntendedVelocityEstimator
from floor_controller import FloorController
from sensors import LiveSensor
from fusion import VelocityFusion
import skeleton3d as SK

DT = 1.0 / C.CONTROL_HZ
SUB = max(1, round(DT / C.TIMESTEP))
HIP = SK.HIP_HALF
STEP_DUR = 0.55
LIFT = 0.07
PELVIS_Z = C.SUPPORT_Z + 0.78
FOOT = {"l": "puck_0", "r": "puck_1"}


def two_link_knee(hip, foot, L1, L2):
    """Knee position for a 2-link leg, bending forward (+x)."""
    v = foot - hip
    d = float(np.linalg.norm(v))
    d = min(d, (L1 + L2) * 0.999)
    if d < 1e-6:
        return (hip + foot) / 2.0
    dirv = v / np.linalg.norm(v)
    a = (L1 * L1 - L2 * L2 + d * d) / (2 * d)
    h = math.sqrt(max(0.0, L1 * L1 - a * a))
    fwd = np.array([1.0, 0.0, 0.0])
    ref = fwd - dirv * np.dot(fwd, dirv)
    if np.linalg.norm(ref) < 1e-6:
        ref = np.array([0, 0, 1.0]) - dirv * dirv[2]
    ref = ref / np.linalg.norm(ref)
    return hip + a * dirv + h * ref


def lateral(side):
    return np.array([0.0, HIP if side == "l" else -HIP])


def skeleton_joints(pelvis_xy, footL, footR):
    pelvis = np.array([pelvis_xy[0], pelvis_xy[1], PELVIS_Z])
    hipL = pelvis + np.array([0, HIP, 0])
    hipR = pelvis + np.array([0, -HIP, 0])
    kneeL = two_link_knee(hipL, footL, SK.THIGH, SK.SHANK)
    kneeR = two_link_knee(hipR, footR, SK.THIGH, SK.SHANK)
    toeL = footL + np.array([0.12, 0, 0.0])
    toeR = footR + np.array([0.12, 0, 0.0])
    chest = pelvis + np.array([0, 0, SK.TRUNK])
    head = chest + np.array([0, 0, 0.12])
    return {"pelvis": pelvis, "hipL": hipL, "hipR": hipR,
            "kneeL": kneeL, "kneeR": kneeR, "ankleL": footL, "ankleR": footR,
            "toeL": toeL, "toeR": toeR, "chest": chest, "head": head}


class Walker:
    """Footstep planner + floor-coupled state for one run."""

    def __init__(self, controller_on=True, est_mode="true",
                 w_live=0.5, w_model=0.5, model_lead=0.12, model_noise=0.04,
                 sensor_latency=12, sensor_noise=0.02, seed=0, tiles=7, k_p=8.0):
        self.tiles = tiles
        xml, meta = build_model_xml(tiles, tiles, pucks=[(0.0, HIP), (0.0, -HIP)])
        self.w = SimWorld(xml, meta)
        self.w.step(300)
        self.br = PredictorBridge()
        self.br.load_trial(MD.make_synthetic_trial(rng=np.random.default_rng(5)))
        self.est = IntendedVelocityEstimator(source="commanded")
        self.ctrl = FloorController(DT, k_p=k_p)
        self.on = controller_on
        # velocity estimation: how the controller learns the intended velocity
        self.est_mode = est_mode          # true | live | model | fused
        self.sensor = LiveSensor(sensor_latency, sensor_noise, seed)
        self.fusion = VelocityFusion(w_live, w_model)
        self.model_lead = model_lead
        self.model_rng = np.random.default_rng(seed + 1)
        self.model_noise = model_noise
        self.stance, self.swing = "r", "l"
        self.t_step = 0.0
        self.foot_rel = {"l": lateral("l").copy(), "r": lateral("r").copy()}
        self.swing_from = self.foot_rel["l"].copy()
        self.pelvis_xy = np.zeros(2)
        self.t = 0.0
        self.log = []

    def _v_estimate(self, vcmd):
        """The controller's estimate of intended velocity, per est_mode."""
        if self.est_mode == "true":
            return vcmd
        v_live = self.sensor.measure(vcmd)                       # reactive, lags
        v_model = self.est.commanded(self.t + self.model_lead) \
            + self.model_rng.normal(0.0, self.model_noise, 2)    # anticipatory
        if self.est_mode == "live":
            return v_live
        if self.est_mode == "model":
            return v_model
        return self.fusion.fuse(v_live, v_model)                 # fused

    def tick(self):
        vcmd = self.est.commanded(self.t)
        v_est = self._v_estimate(vcmd)
        self.br.step()                                  # predictor in the loop

        # body vaults forward over the stance foot at vcmd
        self.foot_rel[self.stance] = self.foot_rel[self.stance] - vcmd * DT
        # swing foot arcs from lift-off toward the next foothold (ahead)
        phase = min(1.0, self.t_step / STEP_DUR)
        plant = lateral(self.swing) + 0.5 * vcmd * STEP_DUR
        self.foot_rel[self.swing] = (1 - phase) * self.swing_from + phase * plant
        swing_z = C.SUPPORT_Z + LIFT * math.sin(math.pi * phase)

        # drive the stance foot so the pelvis stays centered. The feedforward
        # uses the ESTIMATED intended velocity (live/model/fused), so estimation
        # error -> tracking error -> drift; the gait itself still moves at vcmd.
        if self.on:
            self.ctrl.command_stance(self.w, FOOT[self.stance],
                                     self.foot_rel[self.stance], -v_est)
        else:
            self.ctrl.idle(self.w)

        self.w.step_driven(SUB)

        # pelvis derived from the REAL stance-foot position (physics, not faked)
        stance_world = self.w.puck_pos(FOOT[self.stance])[:2]
        self.pelvis_xy = stance_world - self.foot_rel[self.stance]
        sw = self.pelvis_xy + self.foot_rel[self.swing]
        self.w.set_puck_kinematic(FOOT[self.swing], sw[0], sw[1], swing_z)

        self.t += DT
        self.t_step += DT
        if self.t_step >= STEP_DUR:
            self.stance, self.swing = self.swing, self.stance
            self.t_step = 0.0
            self.swing_from = self.foot_rel[self.swing].copy()

        self.log.append((self.t, self.pelvis_xy[0], self.pelvis_xy[1],
                         vcmd[0], vcmd[1], self.ctrl.prev_spin))

    def joints(self):
        footL = self.w.puck_pos("puck_0")
        footR = self.w.puck_pos("puck_1")
        return skeleton_joints(self.pelvis_xy, footL, footR)


def make_camera(tiles):
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.0, 0.0, 0.85]
    cam.distance = max(3.6, tiles * (C.TILE_SIZE + C.TILE_GAP) * 1.3)
    cam.azimuth = 130.0
    cam.elevation = -12.0
    return cam


def compare(seconds):
    res = {}
    for on in (True, False):
        wk = Walker(controller_on=on)
        for _ in range(int(seconds / DT)):
            wk.tick()
        log = np.array(wk.log)
        drift = np.max(np.hypot(log[:, 1], log[:, 2]))
        res[on] = (log, drift)
        print(f" controller {'ON ' if on else 'OFF'}: max pelvis drift from "
              f"center = {drift:.3f} m  (final ({log[-1,1]:+.2f},{log[-1,2]:+.2f}))")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        os.makedirs(C.OUTPUT_DIR, exist_ok=True)
        fig, ax = plt.subplots(figsize=(6, 6))
        for on, c in ((True, "tab:green"), (False, "tab:red")):
            lg = res[on][0]
            ax.plot(lg[:, 1], lg[:, 2], c, label=f"controller {'ON' if on else 'OFF'}")
        ax.scatter([0], [0], c="k", marker="+", s=120, label="center")
        ax.set_aspect("equal"); ax.legend(); ax.grid(alpha=0.3)
        ax.set_xlabel("pelvis x (m)"); ax.set_ylabel("pelvis y (m)")
        ax.set_title("HoloTile centering: walking-person pelvis trajectory")
        fig.savefig(os.path.join(C.OUTPUT_DIR, "control_trajectory.png"), dpi=110)
        print(f" wrote {os.path.join(C.OUTPUT_DIR, 'control_trajectory.png')}")
    except Exception as e:
        print(" (plot skipped:", e, ")")


def fusion_sweep(seconds, tiles=5):
    """Sweep the model weight and show fused beats live-only / model-only.

    live (w_model=0) lags at the turn; model-only (w_model=1) carries model noise;
    an intermediate blend minimizes centering drift -> the weighting matters.
    """
    weights = [0.0, 0.25, 0.5, 0.75, 1.0]
    turn_t0, turn_t1 = 2.8, 5.5
    rows = []
    for wm in weights:
        wk = Walker(est_mode="fused", w_live=1 - wm, w_model=wm, tiles=tiles, k_p=2.5)
        for _ in range(int(seconds / DT)):
            wk.tick()
        log = np.array(wk.log)
        settled = log[:, 0] >= 1.5            # exclude startup transient
        log = log[settled]
        drift = np.hypot(log[:, 1], log[:, 2])
        turn = (log[:, 0] >= turn_t0) & (log[:, 0] <= turn_t1)
        rows.append((wm, float(np.max(drift)), float(np.sqrt(np.mean(drift**2))),
                     float(np.max(drift[turn]))))
        label = "live-only" if wm == 0 else ("model-only" if wm == 1 else f"w_model={wm}")
        print(f" {label:12s}: max {rows[-1][1]:.3f} m  rms {rows[-1][2]:.3f}  "
              f"turn-max {rows[-1][3]:.3f}")
    best = min(rows, key=lambda r: r[3])
    print(f" -> lowest turn drift at w_model={best[0]} ({best[3]:.3f} m)")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        os.makedirs(C.OUTPUT_DIR, exist_ok=True)
        a = np.array(rows)
        fig, ax = plt.subplots(figsize=(6.5, 4.2))
        ax.plot(a[:, 0], a[:, 3], "o-", color="tab:purple", label="max drift at turn")
        ax.plot(a[:, 0], a[:, 2], "s--", color="tab:gray", label="rms drift")
        ax.axvline(best[0], color="tab:green", ls=":", label=f"best w_model={best[0]}")
        ax.set_xlabel("model weight  (0 = live-only, 1 = model-only)")
        ax.set_ylabel("pelvis drift from center (m)")
        ax.set_title("M5 fusion: tuning live-vs-model weight")
        ax.legend(); ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(C.OUTPUT_DIR, "fusion_sweep.png"), dpi=110)
        print(f" wrote {os.path.join(C.OUTPUT_DIR, 'fusion_sweep.png')}")
    except Exception as e:
        print(" (plot skipped:", e, ")")


def render(seconds, mp4=None, frame=False, live=False):
    wk = Walker(controller_on=True)
    cam = make_camera(7)
    if frame or mp4:
        import imageio
        os.makedirs(C.OUTPUT_DIR, exist_ok=True)
        r = mujoco.Renderer(wk.w.model, height=720, width=1280)
        if frame:
            for _ in range(220):
                wk.tick()
            r.update_scene(wk.w.data, cam)
            SK.draw(r.scene, wk.joints())
            imageio.imwrite(os.path.join(C.OUTPUT_DIR, "control_frame.png"), r.render())
            print("wrote control_frame.png")
        if mp4:
            writer = imageio.get_writer(mp4, fps=60, macro_block_size=None)
            nxt = 0.0
            for _ in range(int(seconds / DT)):
                wk.tick()
                if wk.t >= nxt:
                    r.update_scene(wk.w.data, cam)
                    SK.draw(r.scene, wk.joints())
                    writer.append_data(r.render())
                    nxt += 1.0 / 60.0
            writer.close()
            print(f"wrote {mp4}")
        r.close()
    if live:
        with mujoco.viewer.launch_passive(wk.w.model, wk.w.data) as v:
            while v.is_running() and wk.t < seconds:
                wk.tick()
                v.user_scn.ngeom = 0
                SK.draw(v.user_scn, wk.joints())
                v.sync()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--compare", action="store_true")
    ap.add_argument("--sweep", action="store_true", help="M5 fusion weight sweep")
    ap.add_argument("--mp4", default=None)
    ap.add_argument("--frame", action="store_true")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()
    if args.compare:
        compare(args.seconds)
    if args.sweep:
        fusion_sweep(args.seconds)
    if args.mp4 or args.frame or args.live:
        render(args.seconds, mp4=args.mp4, frame=args.frame, live=args.live)
    if not any((args.compare, args.sweep, args.mp4, args.frame, args.live)):
        compare(args.seconds)


if __name__ == "__main__":
    main()
