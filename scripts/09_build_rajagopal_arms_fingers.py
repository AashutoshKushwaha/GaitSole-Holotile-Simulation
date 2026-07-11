"""
Step 9: Build the 3D upper-limb-augmented working model.

Starts from the canonical Rajagopal2016.osim (full-body, has arms with
shoulder/elbow/forearm/wrist already wired up with CoordinateActuators on
every DOF). Adds two things on top:

  1. Four-region foot contacts per side, in the same heel/midfoot/forefoot/toe
     scheme used by 2D_gait_4regions.osim so the GRF pipeline (scripts 04,
     06, 07, 08) keeps working downstream. The toe sphere now lives on the
     real `toes_*` body (Rajagopal has a separate MTP joint), which is more
     correct than the 2D-gait variant where everything was on calcn.

  2. Articulated fingers per hand:
        - Index / middle / ring / little: 3 phalanges (PP, MP, DP) connected
          by single-DOF flex hinges (MCP, PIP, DIP).
        - Thumb: 2 phalanges (proximal, distal) connected by MCP + IP.
     14 phalanges + 14 single-DOF joints per hand. Anatomy is simplified to
     pure flex/extend about a common axis (the MCP abduction/adduction DOF
     and the trapeziometacarpal saddle joint are NOT modeled). Phalanx
     lengths use Buchholz 1992 hand-length proportions; masses redistribute
     ~25% of the existing hand mass into the phalanges so total upper-limb
     mass is conserved.

Why fingers at all? Standard running-biomechanics models (Hamner & Delp
2010, Rajagopal 2016, Lai/Uhlrich 2023) deliberately omit finger DOFs
because the angular-momentum contribution is <1%. Including them was an
explicit user choice; the implementation here is intentionally conservative
(small masses, single DOF, weak reserve actuators) so that adding fingers
neither destabilises forward sims nor blows up Moco solve cost
catastrophically.

Output: E:/OpenSim/models/Rajagopal2016_4regions_fingers.osim
"""

import math
import opensim as osim

SRC_MODEL = "E:/OpenSim/4.5/Models/Rajagopal/Rajagopal2016.osim"
DST_MODEL = "E:/OpenSim/models/Rajagopal2016_4regions_fingers.osim"

# -------------------------------------------------------------------------
# Foot-contact configuration  (matches script 03 physics)
# -------------------------------------------------------------------------
CONTACT_PHYS = dict(
    stiffness=3067776.0,
    dissipation=2.0,
    static_friction=0.8,
    dynamic_friction=0.8,
    viscous_friction=0.5,
    transition_velocity=0.2,
    constant_contact_force=1e-5,
    hertz_smoothing=300.0,
    hunt_crossley_smoothing=50.0,
)

# Sphere positions in body-local coordinates (x forward, y up, z medial).
# Adult-foot tuned; refine for your subject in the GUI if needed.
SPHERES_CALCN = {
    "heel":     dict(loc=(0.010, -0.010, 0.000), radius=0.025),
    "midfoot":  dict(loc=(0.090, -0.005, 0.000), radius=0.025),
    "forefoot": dict(loc=(0.160, -0.012, 0.000), radius=0.020),
}
SPHERES_TOES = {
    "toe":      dict(loc=(0.030, -0.005, 0.000), radius=0.018),
}

# -------------------------------------------------------------------------
# Finger configuration
# -------------------------------------------------------------------------
# Phalanx length as a fraction of hand length (Buchholz et al. 1992).
HAND_LENGTH = 0.19  # m, generic adult; rescale with subject scaling later.

LONG_FINGERS = {
    # ratio of (proximal phalanx, middle phalanx, distal phalanx) to hand length
    "index":  (0.265, 0.143, 0.097),
    "middle": (0.277, 0.160, 0.108),
    "ring":   (0.259, 0.155, 0.107),
    "little": (0.206, 0.117, 0.093),
}
THUMB = (0.196, 0.158)  # proximal, distal

# Lateral offsets of each finger root from the hand centerline, in the +z
# (radial) direction for the RIGHT hand. Mirrored for the left.
# x_root is the distal offset (toward fingertips) from the wrist origin.
FINGER_ROOTS = {
    "thumb":  dict(x=0.040, z=+0.040),
    "index":  dict(x=0.080, z=+0.020),
    "middle": dict(x=0.085, z=+0.000),
    "ring":   dict(x=0.080, z=-0.020),
    "little": dict(x=0.075, z=-0.035),
}

# Phalanx masses (kg). Sum per hand ~= 0.115 kg, which we subtract from
# the existing hand body mass so total stays conserved.
PHALANX_MASS = dict(
    pp=0.012, mp=0.007, dp=0.004,      # long fingers
    thumb_prox=0.015, thumb_dist=0.008,
)

# Range of motion (radians) for finger flexion.
ROM_DEG = dict(mcp=(0, 90), pip=(0, 110), dip=(0, 80),
               thumb_mcp=(0, 55), thumb_ip=(0, 80))

# Reserve actuator strength on each finger DOF. Weak so the fingers don't
# dominate predictive solves but strong enough to overcome gravity.
FINGER_ACTUATOR_OPT_FORCE = 2.0  # Nm


# =========================================================================
# Helpers
# =========================================================================
def deg2rad(d):
    return d * math.pi / 180.0


def _phalanx_inertia(mass, length, radius=0.005):
    """Slim cylinder about its COM. Returns Inertia(Ixx, Iyy, Izz, 0,0,0)."""
    Ixx = mass * (3.0 * radius * radius + length * length) / 12.0
    Iyy = Ixx
    Izz = 0.5 * mass * radius * radius
    return osim.Inertia(Ixx, Iyy, Izz, 0.0, 0.0, 0.0)


def add_floor_and_contacts(model):
    """Add a ground halfspace + 4 contact spheres per foot, each driven by a
    SmoothSphereHalfSpaceForce. SmoothSphereHalfSpaceForce is the
    gradient-friendly variant used by Moco, matching the convention in
    2D_gait_4regions.osim so the downstream GRF scripts (04/06/07/08)
    continue to work without changes."""
    ground = model.getGround()

    floor = osim.ContactHalfSpace(
        osim.Vec3(0, 0, 0),
        osim.Vec3(0, 0, -math.pi / 2.0),   # rotate so normal = +y (world up)
        ground,
        "floor",
    )
    model.addContactGeometry(floor)

    def _set_phys(force):
        force.set_stiffness(CONTACT_PHYS["stiffness"])
        force.set_dissipation(CONTACT_PHYS["dissipation"])
        force.set_static_friction(CONTACT_PHYS["static_friction"])
        force.set_dynamic_friction(CONTACT_PHYS["dynamic_friction"])
        force.set_viscous_friction(CONTACT_PHYS["viscous_friction"])
        force.set_transition_velocity(CONTACT_PHYS["transition_velocity"])
        force.set_constant_contact_force(CONTACT_PHYS["constant_contact_force"])
        force.set_hertz_smoothing(CONTACT_PHYS["hertz_smoothing"])
        force.set_hunt_crossley_smoothing(CONTACT_PHYS["hunt_crossley_smoothing"])

    def _add_sphere_and_force(name, body, loc, radius):
        sphere = osim.ContactSphere(radius,
                                    osim.Vec3(*loc),
                                    body,
                                    name)
        model.addContactGeometry(sphere)

        force = osim.SmoothSphereHalfSpaceForce(
            f"contact_{name}", sphere, floor,
        )
        _set_phys(force)
        model.addForce(force)

    for side in ("r", "l"):
        calcn = model.getBodySet().get(f"calcn_{side}")
        toes = model.getBodySet().get(f"toes_{side}")
        for region, params in SPHERES_CALCN.items():
            loc = list(params["loc"])
            if side == "l":
                loc[2] = -loc[2]
            _add_sphere_and_force(f"{region}_{side}", calcn, tuple(loc), params["radius"])
        for region, params in SPHERES_TOES.items():
            loc = list(params["loc"])
            if side == "l":
                loc[2] = -loc[2]
            _add_sphere_and_force(f"{region}_{side}", toes, tuple(loc), params["radius"])


def _make_phalanx_body(name, mass, length):
    """Construct a Body for a phalanx. Attaches a small Brick centered at
    the body origin so the GUI shows something; visual will sit at the
    proximal joint rather than along the phalanx axis (cosmetic only)."""
    body = osim.Body(name, mass,
                     osim.Vec3(length * 0.5, 0, 0),     # COM at midpoint
                     _phalanx_inertia(mass, length))
    box = osim.Brick(osim.Vec3(length * 0.5, 0.006, 0.006))
    box.setColor(osim.Vec3(0.85, 0.75, 0.65))
    body.attachGeometry(box)
    return body


def _add_pin_joint(model, name, parent_frame, child_body,
                   parent_loc, child_loc, coord_name, rom_rad, default=0.0):
    """Add a 1-DOF PinJoint with rotation axis along z of the joint frame
    (flex/extend in the x-y plane of the finger chain)."""
    joint = osim.PinJoint(
        name,
        parent_frame, osim.Vec3(*parent_loc), osim.Vec3(0, 0, 0),
        child_body,  osim.Vec3(*child_loc),  osim.Vec3(0, 0, 0),
    )
    coord = joint.upd_coordinates(0)
    coord.setName(coord_name)
    coord.setDefaultValue(default)
    coord.setRangeMin(rom_rad[0])
    coord.setRangeMax(rom_rad[1])
    coord.setDefaultClamped(True)
    coord.setDefaultLocked(False)
    model.addJoint(joint)
    return joint


def _add_finger_actuator(model, coord_name):
    act = osim.CoordinateActuator(coord_name)
    act.setName(f"act_{coord_name}")
    act.setOptimalForce(FINGER_ACTUATOR_OPT_FORCE)
    act.setMinControl(-float("inf"))
    act.setMaxControl(float("inf"))
    model.addForce(act)


def add_long_finger(model, side, finger_name, ratios, root_offset):
    """Add MCP+PIP+DIP for one long finger on one hand."""
    pp_len, mp_len, dp_len = (r * HAND_LENGTH for r in ratios)
    sign = +1 if side == "r" else -1   # mirror z for the left hand

    hand = model.getBodySet().get(f"hand_{side}")

    pp = _make_phalanx_body(f"{finger_name}_pp_{side}", PHALANX_MASS["pp"], pp_len)
    mp = _make_phalanx_body(f"{finger_name}_mp_{side}", PHALANX_MASS["mp"], mp_len)
    dp = _make_phalanx_body(f"{finger_name}_dp_{side}", PHALANX_MASS["dp"], dp_len)
    for b in (pp, mp, dp):
        model.addBody(b)

    # MCP: hand -> pp, located at the finger root.
    _add_pin_joint(
        model,
        name=f"{finger_name}_mcp_{side}",
        parent_frame=hand,
        child_body=pp,
        parent_loc=(root_offset["x"], 0.0, sign * root_offset["z"]),
        child_loc=(0.0, 0.0, 0.0),
        coord_name=f"{finger_name}_mcp_{side}",
        rom_rad=(deg2rad(ROM_DEG["mcp"][0]), deg2rad(ROM_DEG["mcp"][1])),
    )
    # PIP: pp -> mp, at the distal end of pp.
    _add_pin_joint(
        model,
        name=f"{finger_name}_pip_{side}",
        parent_frame=pp,
        child_body=mp,
        parent_loc=(pp_len, 0.0, 0.0),
        child_loc=(0.0, 0.0, 0.0),
        coord_name=f"{finger_name}_pip_{side}",
        rom_rad=(deg2rad(ROM_DEG["pip"][0]), deg2rad(ROM_DEG["pip"][1])),
    )
    # DIP: mp -> dp, at the distal end of mp.
    _add_pin_joint(
        model,
        name=f"{finger_name}_dip_{side}",
        parent_frame=mp,
        child_body=dp,
        parent_loc=(mp_len, 0.0, 0.0),
        child_loc=(0.0, 0.0, 0.0),
        coord_name=f"{finger_name}_dip_{side}",
        rom_rad=(deg2rad(ROM_DEG["dip"][0]), deg2rad(ROM_DEG["dip"][1])),
    )

    for joint_kind in ("mcp", "pip", "dip"):
        _add_finger_actuator(model, f"{finger_name}_{joint_kind}_{side}")


def add_thumb(model, side):
    """Add MCP+IP (2 phalanges) for the thumb on one hand. Anatomy
    simplification: pure flex/extend about the same axis as the long
    fingers, ignoring trapeziometacarpal opposition."""
    prox_len, dist_len = (r * HAND_LENGTH for r in THUMB)
    sign = +1 if side == "r" else -1

    hand = model.getBodySet().get(f"hand_{side}")
    root = FINGER_ROOTS["thumb"]

    prox = _make_phalanx_body(f"thumb_prox_{side}", PHALANX_MASS["thumb_prox"], prox_len)
    dist = _make_phalanx_body(f"thumb_dist_{side}", PHALANX_MASS["thumb_dist"], dist_len)
    for b in (prox, dist):
        model.addBody(b)

    _add_pin_joint(
        model,
        name=f"thumb_mcp_{side}",
        parent_frame=hand,
        child_body=prox,
        parent_loc=(root["x"], 0.0, sign * root["z"]),
        child_loc=(0.0, 0.0, 0.0),
        coord_name=f"thumb_mcp_{side}",
        rom_rad=(deg2rad(ROM_DEG["thumb_mcp"][0]), deg2rad(ROM_DEG["thumb_mcp"][1])),
    )
    _add_pin_joint(
        model,
        name=f"thumb_ip_{side}",
        parent_frame=prox,
        child_body=dist,
        parent_loc=(prox_len, 0.0, 0.0),
        child_loc=(0.0, 0.0, 0.0),
        coord_name=f"thumb_ip_{side}",
        rom_rad=(deg2rad(ROM_DEG["thumb_ip"][0]), deg2rad(ROM_DEG["thumb_ip"][1])),
    )

    for joint_kind in ("mcp", "ip"):
        _add_finger_actuator(model, f"thumb_{joint_kind}_{side}")


def redistribute_hand_mass(model, total_phalanx_mass_per_hand):
    """Subtract the new phalanx mass from each hand body so total upper-limb
    mass is conserved. Inertia scales with the mass ratio to a first
    approximation (we don't try to re-derive the palmar inertia tensor)."""
    for side in ("r", "l"):
        hand = model.getBodySet().get(f"hand_{side}")
        old_mass = hand.getMass()
        new_mass = max(old_mass - total_phalanx_mass_per_hand, 0.05)
        ratio = new_mass / old_mass
        hand.setMass(new_mass)
        inertia = hand.getInertia()
        moments = inertia.getMoments()
        products = inertia.getProducts()
        hand.setInertia(osim.Inertia(
            moments.get(0) * ratio, moments.get(1) * ratio, moments.get(2) * ratio,
            products.get(0) * ratio, products.get(1) * ratio, products.get(2) * ratio,
        ))
        print(f"  hand_{side}: mass {old_mass:.4f} -> {new_mass:.4f} kg")


# =========================================================================
# Main
# =========================================================================
def main():
    print(f"Loading {SRC_MODEL}")
    model = osim.Model(SRC_MODEL)
    model.setName("Rajagopal2016_4regions_fingers")

    print("Adding floor + 4-region foot contacts ...")
    add_floor_and_contacts(model)

    print("Adding articulated fingers (both hands) ...")
    for side in ("r", "l"):
        for finger_name, ratios in LONG_FINGERS.items():
            add_long_finger(model, side, finger_name, ratios, FINGER_ROOTS[finger_name])
        add_thumb(model, side)

    phalanx_total = (
        sum(PHALANX_MASS[k] for k in ("pp", "mp", "dp")) * 4
        + PHALANX_MASS["thumb_prox"] + PHALANX_MASS["thumb_dist"]
    )
    print(f"Per-hand phalanx mass total = {phalanx_total:.4f} kg")
    print("Redistributing hand mass ...")
    redistribute_hand_mass(model, phalanx_total)

    print("Finalizing connections ...")
    model.finalizeConnections()

    print(f"Writing {DST_MODEL}")
    model.printToXML(DST_MODEL)

    # Final tally
    n_bodies = model.getBodySet().getSize()
    n_joints = model.getJointSet().getSize()
    n_coords = model.getCoordinateSet().getSize()
    n_forces = model.getForceSet().getSize()
    print()
    print("Model summary:")
    print(f"  bodies      : {n_bodies}")
    print(f"  joints      : {n_joints}")
    print(f"  coordinates : {n_coords}")
    print(f"  forces      : {n_forces}")
    print()
    print("Next: run 10_inspect_and_test_rajagopal.py to verify articulation.")


if __name__ == "__main__":
    main()
