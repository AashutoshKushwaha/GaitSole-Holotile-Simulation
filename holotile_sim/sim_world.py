"""
Physics facade around MuJoCo MjModel/MjData for the HoloTile floor.

Hides name<->index bookkeeping: callers work with tile coordinates, actuator
commands (azimuth, spin) and puck states -- never raw MuJoCo arrays. The
controller writes per-tile (azimuth, spin); SimWorld broadcasts them to every
disk actuator in that tile (patent: all disks in a tile driven identically).

Contact model: disk<->foot contact is frictionless (vertical support only). The
omnidirectional drag is a moving-surface (conveyor) friction force applied each
step in step_driven(): for each foot puck, the tile beneath it commands a surface
velocity (direction = azimuth, speed = DRAG_PER_OMEGA * spin); a friction force
drives the foot toward that velocity, saturated at BELT_MU * N where N is the
measured normal contact force. Real mass, real slip, real friction limit.
"""

import numpy as np
import mujoco

import holotile_config as C


class SimWorld:
    def __init__(self, xml, meta):
        self.model = mujoco.MjModel.from_xml_string(xml)
        self.data = mujoco.MjData(self.model)
        self.meta = meta
        self.grid = meta["grid"]

        # Resolve actuator names -> ctrl indices once.
        self._aid = {}
        for i in range(self.model.nu):
            self._aid[self.model.actuator(i).name] = i

        # Per-tile actuator index arrays + commanded (azimuth, spin) state.
        self._tile_azi, self._tile_spin = {}, {}
        self.tile_cmd = {}
        for tile, info in meta["tiles"].items():
            self._tile_azi[tile] = np.array([self._aid[n] for n in info["azi"]], dtype=int)
            self._tile_spin[tile] = np.array([self._aid[n] for n in info["spin"]], dtype=int)
            self.tile_cmd[tile] = (0.0, 0.0)

        # Per-tile representative disk joint addresses, so the drive can read the
        # disks' ACTUAL azimuth + spin (the real cause of the foot's motion)
        # rather than the commanded value. Empty for flat-pad (overlay) tiles.
        self._tile_has_disks, self._tile_azi_q, self._tile_spin_dof = {}, {}, {}
        for tile, info in meta["tiles"].items():
            if info["disks"]:
                suffix = info["disks"][0][2:]   # strip leading 'd_'
                ja = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "azi_" + suffix)
                js = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "spin_" + suffix)
                self._tile_azi_q[tile] = self.model.jnt_qposadr[ja]
                self._tile_spin_dof[tile] = self.model.jnt_dofadr[js]
                self._tile_has_disks[tile] = True
            else:
                self._tile_has_disks[tile] = False

        # Puck free-joint qpos/qvel addresses, body ids, geom names.
        self._puck_qpos, self._puck_qvel, self._puck_bid = {}, {}, {}
        for name in meta["pucks"]:
            k = name.split("_")[1]
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "puckj_" + k)
            self._puck_qpos[name] = self.model.jnt_qposadr[jid]
            self._puck_qvel[name] = self.model.jnt_dofadr[jid]
            self._puck_bid[name] = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        # Optional external horizontal force per puck (e.g. the person's walking
        # self-propulsion the floor must counter). Added on top of disk friction.
        self._puck_ext = {name: np.zeros(2) for name in meta["pucks"]}

        mujoco.mj_forward(self.model, self.data)

    def set_external_force(self, name, fxy):
        self._puck_ext[name][:] = fxy

    def set_puck_kinematic(self, name, x, y, z):
        """Place a puck and freeze its velocity (used for the airborne swing foot)."""
        a, v = self._puck_qpos[name], self._puck_qvel[name]
        self.data.qpos[a:a + 3] = (x, y, z)
        self.data.qpos[a + 3:a + 7] = (1.0, 0.0, 0.0, 0.0)
        self.data.qvel[v:v + 6] = 0.0

    # --- commands -----------------------------------------------------------
    def set_tile_command(self, tile, azimuth, spin):
        """Aim all disks in `tile` to `azimuth` (rad) and spin them at `spin`."""
        spin = float(np.clip(spin, -C.SPIN_MAX, C.SPIN_MAX))
        self.tile_cmd[tile] = (float(azimuth), spin)
        self.data.ctrl[self._tile_azi[tile]] = azimuth
        self.data.ctrl[self._tile_spin[tile]] = spin

    def set_all_tiles(self, azimuth, spin):
        for tile in self.meta["tiles"]:
            self.set_tile_command(tile, azimuth, spin)

    def tile_of_xy(self, x, y):
        g = self.grid
        tx = int(np.clip(round((x - g["x0"]) / g["pitch"]), 0, g["nx"] - 1))
        ty = int(np.clip(round((y - g["y0"]) / g["pitch"]), 0, g["ny"] - 1))
        return (tx, ty)

    # --- drive + stepping ---------------------------------------------------
    def surface_velocity(self, tile):
        """The horizontal surface velocity the tile's disks impart at the contact.

        Uses the disks' ACTUAL azimuth `a` and spin rate `w` (read from the live
        sim state) so the foot is driven by the real disk rotation. The exact
        top-of-sphere surface velocity of a sphere spinning at `w` about an axis
        tilted by TILT toward azimuth `a` is  w*R*sin(tilt) * (sin a, -cos a)
        =  w * DRAG_PER_OMEGA * (sin a, -cos a). Flat-pad tiles fall back to the
        commanded (azimuth, spin).
        """
        if self._tile_has_disks[tile]:
            a = float(self.data.qpos[self._tile_azi_q[tile]])
            w = float(self.data.qvel[self._tile_spin_dof[tile]])
        else:
            a, w = self.tile_cmd[tile]
        speed = C.DRAG_PER_OMEGA * w
        return np.array([speed * np.sin(a), -speed * np.cos(a)])

    def _apply_belt_drive(self):
        """Apply the conveyor friction force to every puck (call before mj_step).

        The force drives the foot toward the disks' actual surface velocity,
        saturated at BELT_MU * N (N = measured normal contact force). Real slip,
        real friction limit -- and zero force when the disks are not spinning.
        """
        self.data.xfrc_applied[:] = 0.0
        for name, bid in self._puck_bid.items():
            p = self.puck_pos(name)
            v_surf = self.surface_velocity(self.tile_of_xy(p[0], p[1]))
            slip = self.puck_vel(name)[:2] - v_surf
            n = abs(self.contact_force_on("puckg_" + name.split("_")[1])[2])
            f = -C.BELT_K * slip
            fmag = float(np.linalg.norm(f))
            fmax = C.BELT_MU * n
            if fmag > fmax and fmag > 1e-9:
                f *= fmax / fmag
            self.data.xfrc_applied[bid, 0:2] = f + self._puck_ext[name]

    def step(self, n=1):
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)

    def step_driven(self, n=1):
        """Step with the moving-surface drive active (the real sim loop)."""
        for _ in range(n):
            self._apply_belt_drive()
            mujoco.mj_step(self.model, self.data)

    @property
    def time(self):
        return self.data.time

    # --- puck state ---------------------------------------------------------
    def puck_pos(self, name):
        a = self._puck_qpos[name]
        return self.data.qpos[a:a + 3].copy()

    def puck_vel(self, name):
        """World-frame linear velocity of the puck (free-joint dofs 0:3)."""
        a = self._puck_qvel[name]
        return self.data.qvel[a:a + 3].copy()

    # --- contact ------------------------------------------------------------
    def contact_force_on(self, geom_name):
        """Net contact force (world N, 3-vector) acting on a named geom."""
        gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
        total = np.zeros(3)
        buf = np.zeros(6)
        for c in range(self.data.ncon):
            con = self.data.contact[c]
            if con.geom1 == gid or con.geom2 == gid:
                mujoco.mj_contactForce(self.model, self.data, c, buf)
                f = buf[:3].copy()
                if con.geom2 == gid:
                    f = -f
                frame = con.frame.reshape(3, 3)
                total += frame.T @ f
        return total
