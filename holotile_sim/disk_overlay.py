"""
Visual disk overlay for the production floor.

The production floor's physics is just feet on frictionless tile pads; the disks
are drawn here as scene geoms so they cost rendering only, not simulation. Each
visual disk is a tilted "coin" plus a small orbiting rim marker; the coin's tilt
direction shows the tile AZIMUTH (push direction) and the marker orbits at the
tile SPIN rate, so re-orientation and spin are both visible. Thousands scale fine.

Works on any mjvScene: pass viewer.user_scn (reset ngeom=0 first) or a
mujoco.Renderer's .scene (appends after the model geoms).
"""

import numpy as np
import mujoco

import holotile_config as C

_COIN_RGBA = np.array([0.30, 0.62, 0.90, 1.0])
_MARK_RGBA = np.array([0.98, 0.82, 0.20, 1.0])


def _tilt_axis(azi):
    s, c = np.sin(C.TILT_RAD), np.cos(C.TILT_RAD)
    return np.array([s * np.cos(azi), s * np.sin(azi), c])


def _plane_basis(n):
    """Two orthonormal vectors spanning the disk plane (perpendicular to n)."""
    ref = np.array([0.0, 0.0, 1.0])
    if abs(n[2]) > 0.95:
        ref = np.array([1.0, 0.0, 0.0])
    u = np.cross(ref, n)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    return u, v


class DiskOverlay:
    def __init__(self, grid, per_tile=None, radius=None):
        self.per_tile = per_tile or C.VIS_DISKS_PER_TILE
        self.radius = radius or C.VIS_DISK_RADIUS
        self.coin_hz = C.DISK_HALF_THICKNESS
        self.z = C.SUPPORT_Z - self.coin_hz
        # Precompute disk centres grouped by tile.
        g = grid
        dpitch = C.TILE_SIZE / self.per_tile
        d0 = -(self.per_tile - 1) / 2.0 * dpitch
        self.tiles = {}
        for tx in range(g["nx"]):
            for ty in range(g["ny"]):
                cx, cy = g["x0"] + tx * g["pitch"], g["y0"] + ty * g["pitch"]
                pts = [(cx + d0 + i * dpitch, cy + d0 + j * dpitch)
                       for i in range(self.per_tile) for j in range(self.per_tile)]
                self.tiles[(tx, ty)] = np.array(pts)
        self._coin_size = np.array([self.radius, self.coin_hz, 0.0])
        self._mark_size = np.array([self.radius * 0.16, 0.0, 0.0])

    def update(self, scene, tile_cmd, t):
        """Append coin+marker geoms for every visual disk to `scene`."""
        for tile, pts in self.tiles.items():
            azi, spin = tile_cmd.get(tile, (0.0, 0.0))
            n = _tilt_axis(azi)
            u, v = _plane_basis(n)
            mat = np.column_stack([u, v, n]).flatten()
            phase = spin * t
            mvec = self.radius * 0.78 * (np.cos(phase) * u + np.sin(phase) * v)
            for (dx, dy) in pts:
                if scene.ngeom + 2 > scene.maxgeom:
                    return
                center = np.array([dx, dy, self.z])
                gi = scene.ngeom
                mujoco.mjv_initGeom(scene.geoms[gi], mujoco.mjtGeom.mjGEOM_CYLINDER,
                                    self._coin_size, center, mat, _COIN_RGBA)
                mujoco.mjv_initGeom(scene.geoms[gi + 1], mujoco.mjtGeom.mjGEOM_SPHERE,
                                    self._mark_size, center + mvec,
                                    np.eye(3).flatten(), _MARK_RGBA)
                scene.ngeom += 2
