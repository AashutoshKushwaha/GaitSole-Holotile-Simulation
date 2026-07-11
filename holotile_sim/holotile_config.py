"""
Single source of truth for the HoloTile simulator.

Everything (MJCF builder, physics world, controller, logging, plotting) imports
constants from here so the model and the code that drives it never disagree --
the same discipline used by motion_predictor/config.py.

Geometry follows the patent (US20180217662A1): a modular floor of square
"active tiles"; each tile is an array of "disk assemblies"; each disk sits at a
fixed TILT angle so only a raised rim arc contacts a shoe, can be re-oriented
about vertical (swashplate AZIMUTH -> push direction), and spins about its tilted
axis (SPIN rate -> push speed). All disks in a tile are driven identically.
"""

import math
import os

# ---------------------------------------------------------------------------
# Physics
# ---------------------------------------------------------------------------
TIMESTEP = 0.001            # s  (1 kHz physics; spin up to ~30 rad/s stays stable)
GRAVITY = -9.81             # m/s^2 (world -z)
CONTROL_HZ = 100.0          # control/predictor tick rate (matches predictor FPS)

# ---------------------------------------------------------------------------
# Tile / disk geometry  (metres, degrees)
# ---------------------------------------------------------------------------
TILE_SIZE = 0.30            # square tile edge (~1 foot, per patent FIG.3)
TILE_GAP = 0.004            # small spacing between tile edges (patent: <=0.25 in)
DISKS_PER_TILE = 5          # disks along each tile axis -> DISKS_PER_TILE^2 per tile
                            # (fewer/larger disks -> faster sim + more drag per spin)
TILT_DEG = 35.0            # tilt of the spin axis (patent hemisphere variant 15-60)
DISK_HALF_THICKNESS = 0.006  # thin coin -- VISUAL disk only (M2); contact is a sphere
DISK_CENTER_Z = 0.030      # height of disk body centres above the tile base

# Contact element = a SPHERE spinning about a tilted axis (patent FIG.19-22
# hemisphere / Holobelt variant). A sphere's top stays at constant height while
# it spins (no vertical pumping -> no catapult) and the top-point surface moves
# horizontally at omega*R*sin(tilt) -> smooth, controllable friction drag, with a
# vertical contact normal (no spurious tilt bias).
_DISK_PITCH = TILE_SIZE / DISKS_PER_TILE
DISK_RADIUS = 0.46 * _DISK_PITCH
TILT_RAD = math.radians(TILT_DEG)

# Tilted spin axis in the (post-azimuth) body frame, tilted toward +x.
TILT_AXIS = (math.sin(TILT_RAD), 0.0, math.cos(TILT_RAD))

# Sphere top sits at the support plane the foot rides on.
SUPPORT_Z = DISK_CENTER_Z + DISK_RADIUS

# Horizontal surface speed at the sphere top per unit spin rate (m/s per rad/s):
# |omega_axis x (0,0,R)| = omega * R * sin(tilt). Used to convert desired surface
# speed <-> spin rate in the controller.
DRAG_PER_OMEGA = DISK_RADIUS * math.sin(TILT_RAD)

# ---------------------------------------------------------------------------
# Floor grid (full demo). M1 overrides to 1x1.
# ---------------------------------------------------------------------------
FLOOR_TILES_X = 8
FLOOR_TILES_Y = 8

# Production floor: feet rest on thin frictionless tile pads (the visible tiles);
# the disks are a VISUAL OVERLAY (no physics cost) driven by the tile commands.
SUPPORT_PAD_HALF_THICKNESS = 0.02
VIS_DISKS_PER_TILE = 5          # visual disks per tile axis (overlay only)
VIS_DISK_RADIUS = 0.46 * (TILE_SIZE / VIS_DISKS_PER_TILE)

# ---------------------------------------------------------------------------
# Contact / friction
# ---------------------------------------------------------------------------
# Disk-foot contact is FRICTIONLESS (condim=1): the disks only provide stable
# vertical support. The omnidirectional drag is supplied by the moving-surface
# (conveyor) friction model below -- the physically-correct, numerically-stable
# way to model "a spinning surface drags the foot" (literal fast rigid-spin
# contact pumps energy and launches the foot at realistic speeds). The disks
# still physically tilt/spin as visible DOFs; their spin rate sets the commanded
# surface speed via DRAG_PER_OMEGA.
DISK_FRICTION = (0.0, 0.0, 0.0)
PUCK_FRICTION = (0.0, 0.0, 0.0)
GROUND_FRICTION = (0.3, 0.005, 0.0001)
CONTACT_CONDIM = 1                     # normal-only support; tangential via belt model
SOLREF = (0.01, 1.0)                   # firm-but-stable contact (timeconst, damping)
SOLIMP = (0.95, 0.99, 0.001)
CONTACT_CONE = "pyramidal"
CONTACT_IMPRATIO = 1.0

# Moving-surface (conveyor) friction model: tangential force on the foot drives
# its velocity toward the commanded surface velocity, saturated at BELT_MU * N
# (N = measured normal contact force). Real slip, real friction limit.
BELT_MU = 1.0                          # effective foot<->surface friction coefficient
BELT_K = 3000.0                        # N per (m/s) slip before saturation (stiff -> Coulomb)

# ---------------------------------------------------------------------------
# Actuators
# ---------------------------------------------------------------------------
AZI_KP = 8.0               # azimuth position-servo stiffness
AZI_KV = 0.3               # azimuth damping
SPIN_KV = 0.6              # spin velocity-servo gain (must hold rate under foot load)
SPIN_MAX = 150.0           # |spin| clamp (rad/s); max surface ~ SPIN_MAX*DRAG_PER_OMEGA
                           # ~2.4 m/s -> enough to keep up with walking foot speeds
AZI_SLEW = math.radians(720.0)   # max azimuth slew (rad/s) -> visibly smooth re-orient
SPIN_SLEW = 800.0                # max spin-rate change per second (responsive tracking)

# ---------------------------------------------------------------------------
# Foot / shoe puck
# ---------------------------------------------------------------------------
PUCK_HALF = (0.060, 0.040, 0.015)     # box half-extents (a shoe-ish footprint)
PUCK_MASS = 2.0                       # kg (a loaded foot; refined later from GRF)
PUCK_START_CLEAR = 0.02               # start this far above the support plane

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
PKG_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(PKG_DIR)               # E:\OpenSim
MOTION_PREDICTOR_DIR = os.path.join(PROJECT_ROOT, "motion_predictor")
OUTPUT_DIR = os.path.join(PKG_DIR, "output")
