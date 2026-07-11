"""
Procedurally generate the HoloTile floor as a MuJoCo MJCF (XML) string.

Two builders, same meta contract:

* build_floor_xml(...) -- the PRODUCTION floor: feet rest on thin frictionless
  tile PADS (the visible tiles); the disks are a visual overlay (disk_overlay.py)
  driven by the tile commands, so physics stays tiny and scales to a full floor.
  The omnidirectional drag is the moving-surface friction model in SimWorld.

* build_model_xml(...) -- the PHYSICAL-DISK floor: each tile is a real array of
  tilted, spinning disk bodies (azimuth + spin hinge joints + sphere geom +
  actuators). Used by the M1 proof and the rigid-spin reference demo.

Both return (xml, meta) where meta = {
  'tiles': {(tx,ty): {'azi':[act names], 'spin':[act names], 'disks':[body names]}},
  'pucks': ['puck_0', ...],
  'grid':  {'x0','y0','pitch','nx','ny'},
}. For the production floor the per-tile azi/spin lists are empty (no actuators).
"""

import holotile_config as C


# ---------------------------------------------------------------------------
# Shared scene scaffolding
# ---------------------------------------------------------------------------
def _scene_assets_and_options():
    gfr = "%g %g %g" % C.GROUND_FRICTION
    return f"""  <option timestep="{C.TIMESTEP}" gravity="0 0 {C.GRAVITY}" integrator="implicitfast"
          cone="{C.CONTACT_CONE}" impratio="{C.CONTACT_IMPRATIO}"/>
  <visual>
    <headlight diffuse="0.55 0.55 0.55" ambient="0.35 0.35 0.35" specular="0.2 0.2 0.2"/>
    <rgba haze="0.15 0.18 0.22 1"/>
    <map shadowscale="0.4"/>
    <quality shadowsize="4096"/>
    <global azimuth="120" elevation="-25" offwidth="1920" offheight="1080"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.4 0.55" rgb2="0.05 0.07 0.10"
             width="512" height="512"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.16 0.18 0.22"
             rgb2="0.09 0.11 0.14" width="300" height="300"/>
    <material name="gridmat" texture="grid" texrepeat="10 10" reflectance="0.1"/>
    <material name="tilemat" rgba="0.20 0.30 0.42 1" reflectance="0.25"/>
    <material name="footmat" rgba="0.85 0.30 0.25 1" reflectance="0.1"/>
  </asset>
  <worldbody>
    <light pos="2 -2 4" dir="-0.4 0.4 -1" diffuse="0.5 0.5 0.5" castshadow="true"/>
    <geom name="ground" type="plane" size="6 6 0.1" material="gridmat"
          friction="{gfr}" condim="3"/>"""


def _puck_body(k, x, y, z, friction=None, condim=None):
    sr = "%g %g" % C.SOLREF
    si = "%g %g %g" % C.SOLIMP
    fr = "%g %g %g" % (friction or C.PUCK_FRICTION)
    cd = condim or C.CONTACT_CONDIM
    hx, hy, hz = C.PUCK_HALF
    density = C.PUCK_MASS / (8.0 * hx * hy * hz)
    return f"""
    <body name="puck_{k}" pos="{x:.5f} {y:.5f} {z:.5f}">
      <freejoint name="puckj_{k}"/>
      <geom name="puckg_{k}" type="box" size="{hx:.5f} {hy:.5f} {hz:.5f}"
            density="{density:.2f}" material="footmat"
            friction="{fr}" condim="{cd}" solref="{sr}" solimp="{si}"/>
    </body>"""


def _grid_geometry(tiles_x, tiles_y):
    pitch = C.TILE_SIZE + C.TILE_GAP
    x0 = -(tiles_x - 1) / 2.0 * pitch
    y0 = -(tiles_y - 1) / 2.0 * pitch
    grid = {"x0": x0, "y0": y0, "pitch": pitch, "nx": tiles_x, "ny": tiles_y}
    return pitch, x0, y0, grid


def _default_pucks(pucks):
    pz = C.SUPPORT_Z + C.PUCK_HALF[2] + C.PUCK_START_CLEAR
    if pucks is None:
        pucks = [(0.0, 0.0)]
    return pucks, pz


# ---------------------------------------------------------------------------
# Production floor: flat frictionless tile pads + feet; disks are overlay.
# ---------------------------------------------------------------------------
def build_floor_xml(tiles_x=8, tiles_y=8, pucks=None):
    pitch, x0, y0, grid = _grid_geometry(tiles_x, tiles_y)
    half = C.TILE_SIZE / 2.0
    pad_hz = C.SUPPORT_PAD_HALF_THICKNESS
    pad_z = C.SUPPORT_Z - pad_hz
    fr = "%g %g %g" % C.PUCK_FRICTION
    sr = "%g %g" % C.SOLREF
    si = "%g %g %g" % C.SOLIMP

    pads, meta = [], {"tiles": {}, "pucks": [], "grid": grid}
    for tx in range(tiles_x):
        for ty in range(tiles_y):
            cx, cy = x0 + tx * pitch, y0 + ty * pitch
            pads.append(
                f'    <geom name="pad_{tx}_{ty}" type="box" '
                f'size="{half:.5f} {half:.5f} {pad_hz:.5f}" '
                f'pos="{cx:.5f} {cy:.5f} {pad_z:.5f}" material="tilemat" '
                f'friction="{fr}" condim="{C.CONTACT_CONDIM}" '
                f'solref="{sr}" solimp="{si}"/>')
            meta["tiles"][(tx, ty)] = {"azi": [], "spin": [], "disks": []}

    pucks, pz = _default_pucks(pucks)
    puck_xml = []
    for k, (px, py) in enumerate(pucks):
        puck_xml.append(_puck_body(k, px, py, pz))
        meta["pucks"].append(f"puck_{k}")

    xml = f"""<mujoco model="holotile_floor">
{_scene_assets_and_options()}
{chr(10).join(pads)}
{''.join(puck_xml)}
  </worldbody>
</mujoco>"""
    return xml, meta


# ---------------------------------------------------------------------------
# Physical-disk floor: real tilted spinning disk bodies (M1 / rigid demo).
# ---------------------------------------------------------------------------
def _disk_body(name, x, y, ax, az, friction=None, condim=None):
    fr = "%g %g %g" % (friction or C.DISK_FRICTION)
    cd = condim or C.CONTACT_CONDIM
    sr = "%g %g" % C.SOLREF
    si = "%g %g %g" % C.SOLIMP
    axis = "%g 0 %g" % (ax, az)
    return f"""
      <body name="d_{name}" pos="{x:.5f} {y:.5f} {C.DISK_CENTER_Z:.5f}">
        <joint name="azi_{name}" type="hinge" axis="0 0 1" damping="0.02"/>
        <joint name="spin_{name}" type="hinge" axis="{axis}" damping="0.002"/>
        <geom name="g_{name}" type="sphere" size="{C.DISK_RADIUS:.5f}"
              rgba="0.30 0.55 0.85 1"
              friction="{fr}" condim="{cd}" solref="{sr}" solimp="{si}"/>
        <geom type="cylinder" size="{C.DISK_RADIUS*0.95:.5f} {C.DISK_HALF_THICKNESS:.5f}"
              zaxis="{axis}" pos="0 0 {C.DISK_RADIUS*0.6:.5f}"
              rgba="0.95 0.80 0.20 1" contype="0" conaffinity="0"/>
      </body>"""


def build_model_xml(tiles_x=1, tiles_y=1, pucks=None,
                    disk_friction=None, puck_friction=None, condim=None):
    pitch, x0, y0, grid = _grid_geometry(tiles_x, tiles_y)
    dpitch = C.TILE_SIZE / C.DISKS_PER_TILE
    d0 = -(C.DISKS_PER_TILE - 1) / 2.0 * dpitch
    ax, _, az = C.TILT_AXIS

    bodies, actuators, meta = [], [], {"tiles": {}, "pucks": [], "grid": grid}
    for tx in range(tiles_x):
        for ty in range(tiles_y):
            cx, cy = x0 + tx * pitch, y0 + ty * pitch
            tinfo = {"azi": [], "spin": [], "disks": []}
            for i in range(C.DISKS_PER_TILE):
                for j in range(C.DISKS_PER_TILE):
                    name = f"{tx}_{ty}_{i}_{j}"
                    dx, dy = cx + d0 + i * dpitch, cy + d0 + j * dpitch
                    bodies.append(_disk_body(name, dx, dy, ax, az,
                                             friction=disk_friction, condim=condim))
                    actuators.append(
                        f'    <position name="aazi_{name}" joint="azi_{name}" '
                        f'kp="{C.AZI_KP}" kv="{C.AZI_KV}"/>')
                    actuators.append(
                        f'    <velocity name="aspin_{name}" joint="spin_{name}" '
                        f'kv="{C.SPIN_KV}"/>')
                    tinfo["azi"].append(f"aazi_{name}")
                    tinfo["spin"].append(f"aspin_{name}")
                    tinfo["disks"].append(f"d_{name}")
            meta["tiles"][(tx, ty)] = tinfo

    pucks, pz = _default_pucks(pucks)
    puck_xml = []
    for k, (px, py) in enumerate(pucks):
        puck_xml.append(_puck_body(k, px, py, pz, friction=puck_friction, condim=condim))
        meta["pucks"].append(f"puck_{k}")

    xml = f"""<mujoco model="holotile_disks">
{_scene_assets_and_options()}
    {''.join(bodies)}
    {''.join(puck_xml)}
  </worldbody>
  <actuator>
{chr(10).join(actuators)}
  </actuator>
</mujoco>"""
    return xml, meta
