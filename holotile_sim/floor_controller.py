"""
Floor controller -- the patent's keep-centered logic.

Goal: keep the person centered. With the two-foot stance/swing model, the pelvis
rides on the planted (stance) foot: pelvis = stance_foot - foot_rel_stance. To
hold the pelvis at the floor centre C, the stance foot must track the moving
target  C + foot_rel_stance. This controller drives the stance foot there by
commanding the disks' surface velocity (azimuth + spin), with a velocity
feedforward + position P-term, slew-limited for visibly smooth re-orientation.

Surface velocity <-> command: a sphere spinning at `spin` about an axis tilted by
TILT toward azimuth `a` gives surface velocity  spin*DRAG_PER_OMEGA*(sin a,-cos a).
Invert: spin = |V|/DRAG_PER_OMEGA, a = atan2(Vx, -Vy).
"""

import math
import numpy as np

import holotile_config as C


def surface_to_command(V, prev_azi):
    mag = float(np.linalg.norm(V))
    if mag < 1e-4:
        return prev_azi, 0.0
    spin = min(mag / C.DRAG_PER_OMEGA, C.SPIN_MAX)
    azi = math.atan2(V[0], -V[1])
    return azi, spin


def slew_angle(prev, target, max_step):
    d = (target - prev + math.pi) % (2 * math.pi) - math.pi
    return prev + max(-max_step, min(max_step, d))


class FloorController:
    def __init__(self, dt, k_p=8.0, center=(0.0, 0.0)):
        self.dt = dt
        self.k_p = k_p
        self.center = np.asarray(center, float)
        self.prev_azi = 0.0
        self.prev_spin = 0.0

    def command_stance(self, world, stance_foot, foot_rel_stance, foot_rel_vel):
        """Drive the stance foot so the pelvis stays at centre.

        target = center + foot_rel_stance ; target_vel = d(foot_rel_stance)/dt.
        Returns (azimuth, spin, V_desired, surface_speed_capped).
        """
        foot_xy = world.puck_pos(stance_foot)[:2]
        target = self.center + np.asarray(foot_rel_stance)
        target_vel = np.asarray(foot_rel_vel)
        V = target_vel + self.k_p * (target - foot_xy)

        azi, spin = surface_to_command(V, self.prev_azi)
        azi = slew_angle(self.prev_azi, azi, C.AZI_SLEW * self.dt)
        spin = float(np.clip(spin, self.prev_spin - C.SPIN_SLEW * self.dt,
                             self.prev_spin + C.SPIN_SLEW * self.dt))
        world.set_all_tiles(azi, spin)
        self.prev_azi, self.prev_spin = azi, spin
        return azi, spin, V, spin * C.DRAG_PER_OMEGA

    def idle(self, world):
        world.set_all_tiles(self.prev_azi, 0.0)
        self.prev_spin = 0.0
