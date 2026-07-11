"""
Step 10: Verify the upper-limb-augmented model from script 09 by:

  1. Loading Rajagopal2016_4regions_fingers.osim and printing a structured
     inventory: leg coordinates, lumbar, arms, fingers, contacts, actuators.
     This is the equivalent of script 01's inspection step, adapted to the
     new model.

  2. Generating a 2-second prescribed kinematic preview that shows:
       - hips and arms swinging in opposite-leg phase (right arm forward
         when right hip flexes back, etc.) -- the rotational-momentum
         counter-balance pattern from Hamner & Delp 2013;
       - elbows held near 75 deg flex (running posture, Mann 1981) with
         a small in-phase modulation;
       - fingers gently curled (~30 deg flex) into a relaxed running fist.
     Output is a .sto motion file that you load in the OpenSim GUI on top
     of the .osim model to watch articulation, with no integrator step
     so this preview cannot diverge or fall over.

  3. (Optional, controlled by RUN_FORWARD) Running a true ForwardTool
     simulation for 0.2 s to confirm the model is dynamically valid; this
     catches things like singular inertia, locked DOFs, or bad contact
     definitions.

Output files in E:/OpenSim/output/:
   - rajagopal_arms_preview.sto    (kinematic motion you can replay in GUI)
   - rajagopal_fwd_states.sto      (only if RUN_FORWARD = True)
"""

import math
import os
import opensim as osim

MODEL_PATH = "E:/OpenSim/models/Rajagopal2016_4regions_fingers.osim"
OUT_DIR = "E:/OpenSim/output"
PREVIEW_STO = os.path.join(OUT_DIR, "rajagopal_arms_preview.sto")
FWD_STATES_STO = os.path.join(OUT_DIR, "rajagopal_fwd_states.sto")

RUN_FORWARD = False        # set True to also run a real ForwardTool sim
PREVIEW_DURATION = 2.0     # seconds
PREVIEW_RATE_HZ = 100.0    # samples per second
STRIDE_HZ = 1.4            # running cadence ~1.4 Hz (180 spm halved)

ARM_FLEX_AMPLITUDE_DEG = 30.0   # peak shoulder flex during running, Mann 1981
ELBOW_MEAN_DEG = 75.0           # held flexed for running
ELBOW_MOD_DEG = 12.0            # small oscillation about mean
HIP_FLEX_AMPLITUDE_DEG = 30.0   # +/- in stride
KNEE_FLEX_AMPLITUDE_DEG = 35.0  # peak knee flex in swing
FINGER_REST_DEG = 30.0          # relaxed running fist


# =========================================================================
# Inventory
# =========================================================================
def _bucket_coords(model):
    cs = model.getCoordinateSet()
    buckets = {
        "pelvis (root)":     [],
        "lumbar / torso":    [],
        "hips":              [],
        "knees":             [],
        "ankles / subtalar": [],
        "MTP toes":          [],
        "shoulders":         [],
        "elbows / forearm":  [],
        "wrists":            [],
        "fingers":           [],
        "other":             [],
    }
    for i in range(cs.getSize()):
        n = cs.get(i).getName()
        if n.startswith("pelvis_"):
            buckets["pelvis (root)"].append(n)
        elif n.startswith("lumbar_"):
            buckets["lumbar / torso"].append(n)
        elif n.startswith("hip_"):
            buckets["hips"].append(n)
        elif n.startswith("knee_"):
            buckets["knees"].append(n)
        elif n.startswith("ankle_") or n.startswith("subtalar_"):
            buckets["ankles / subtalar"].append(n)
        elif n.startswith("mtp_"):
            buckets["MTP toes"].append(n)
        elif n.startswith("arm_"):
            buckets["shoulders"].append(n)
        elif n.startswith("elbow_") or n.startswith("pro_sup_"):
            buckets["elbows / forearm"].append(n)
        elif n.startswith("wrist_"):
            buckets["wrists"].append(n)
        elif any(n.startswith(p) for p in
                 ("thumb_", "index_", "middle_", "ring_", "little_")):
            buckets["fingers"].append(n)
        else:
            buckets["other"].append(n)
    return buckets


def _bucket_contacts(model):
    cg = model.getContactGeometrySet()
    out = []
    for i in range(cg.getSize()):
        c = cg.get(i)
        out.append((c.getName(), c.getConcreteClassName()))
    return out


def _bucket_actuators(model):
    fs = model.getForceSet()
    muscles, cact, tact, contacts, other = [], [], [], [], []
    for i in range(fs.getSize()):
        f = fs.get(i)
        cls = f.getConcreteClassName()
        nm = f.getName()
        if "Muscle" in cls:
            muscles.append(nm)
        elif cls == "CoordinateActuator":
            cact.append(nm)
        elif cls == "TorqueActuator":
            tact.append(nm)
        elif "Smooth" in cls or "Contact" in cls or "HuntCrossley" in cls:
            contacts.append(nm)
        else:
            other.append((nm, cls))
    return muscles, cact, tact, contacts, other


def print_inventory(model):
    print("=" * 70)
    print(f"MODEL: {model.getName()}")
    print(f"  bodies      : {model.getBodySet().getSize()}")
    print(f"  joints      : {model.getJointSet().getSize()}")
    print(f"  coordinates : {model.getCoordinateSet().getSize()}")
    print(f"  forces      : {model.getForceSet().getSize()}")
    print()
    print("COORDINATES BY REGION")
    print("-" * 70)
    for region, names in _bucket_coords(model).items():
        if not names:
            continue
        print(f"  {region}  ({len(names)})")
        for n in sorted(names):
            print(f"      {n}")
    print()
    print("CONTACT GEOMETRY")
    print("-" * 70)
    for nm, cls in _bucket_contacts(model):
        print(f"  {nm:18s}  {cls}")
    muscles, cact, tact, contacts, other = _bucket_actuators(model)
    print()
    print("ACTUATORS / FORCES")
    print("-" * 70)
    print(f"  muscles               : {len(muscles)}")
    print(f"  CoordinateActuators   : {len(cact)}")
    print(f"  TorqueActuators       : {len(tact)}")
    print(f"  Contact forces        : {len(contacts)}")
    print(f"  Other                 : {len(other)}")
    if other:
        for nm, cls in other[:10]:
            print(f"      {nm:24s}  {cls}")
    print()


# =========================================================================
# Kinematic preview .sto
# =========================================================================
def _deg(d):
    return d * math.pi / 180.0


def _build_preview_motion(model):
    """Return (time_array, dict of coord_name -> values array). Sets pelvis
    height so the feet sit close to the floor, drives arms / hips / knees
    with opposite-leg sinusoids, holds elbows near 75 deg flex, and curls
    fingers into a relaxed grip."""
    n = int(PREVIEW_DURATION * PREVIEW_RATE_HZ) + 1
    ts = [i / PREVIEW_RATE_HZ for i in range(n)]
    omega = 2.0 * math.pi * STRIDE_HZ

    coord_values = {n: [0.0] * len(ts) for n in
                    [c.getName() for c in model.getCoordinateSet()]}

    # Stand the pelvis at roughly natural height. The default standing
    # pose for Rajagopal2016 puts pelvis_ty near 0.94 m; nudging down a
    # hair so contact spheres engage if you later run a forward sim.
    coord_values["pelvis_ty"] = [0.93] * len(ts)

    # Lock the pelvis horizontal so the preview is a "running in place".
    coord_values["pelvis_tx"] = [0.0] * len(ts)
    coord_values["pelvis_tz"] = [0.0] * len(ts)

    for k, t in enumerate(ts):
        s = math.sin(omega * t)
        c = math.cos(omega * t)

        # Hips: right leg forward when sin > 0
        coord_values["hip_flexion_r"][k] = _deg(HIP_FLEX_AMPLITUDE_DEG) * s
        coord_values["hip_flexion_l"][k] = -_deg(HIP_FLEX_AMPLITUDE_DEG) * s

        # Knees flex during swing (when ipsilateral hip is flexing forward).
        # Use a rectified-ish profile: positive flex always.
        coord_values["knee_angle_r"][k] = (
            _deg(KNEE_FLEX_AMPLITUDE_DEG) * 0.5 * (1.0 - c)
        )
        coord_values["knee_angle_l"][k] = (
            _deg(KNEE_FLEX_AMPLITUDE_DEG) * 0.5 * (1.0 + c)
        )

        # Arms: shoulder flexion opposite to ipsilateral hip flexion.
        coord_values["arm_flex_r"][k] = -_deg(ARM_FLEX_AMPLITUDE_DEG) * s
        coord_values["arm_flex_l"][k] = +_deg(ARM_FLEX_AMPLITUDE_DEG) * s

        # Elbows held at mean flex with small in-phase modulation
        coord_values["elbow_flex_r"][k] = _deg(ELBOW_MEAN_DEG + ELBOW_MOD_DEG * abs(s))
        coord_values["elbow_flex_l"][k] = _deg(ELBOW_MEAN_DEG + ELBOW_MOD_DEG * abs(s))

        # Fingers: relaxed light grip, all in slight flex
        for side in ("r", "l"):
            for finger in ("index", "middle", "ring", "little"):
                for joint in ("mcp", "pip", "dip"):
                    coord_values[f"{finger}_{joint}_{side}"][k] = _deg(FINGER_REST_DEG)
            coord_values[f"thumb_mcp_{side}"][k] = _deg(FINGER_REST_DEG * 0.6)
            coord_values[f"thumb_ip_{side}"][k] = _deg(FINGER_REST_DEG * 0.8)

    return ts, coord_values


def _write_sto(path, times, coord_values):
    """Write a .sto motion file directly as text. Columns use the
    `/jointset/<joint>/<coord>/value` state-name format so the GUI and
    Moco both recognise the values as coordinate states."""
    # Map coordinate name -> joint name for column labels.
    model = osim.Model(MODEL_PATH)
    model.initSystem()
    js = model.getJointSet()
    coord_to_joint = {}
    for ji in range(js.getSize()):
        j = js.get(ji)
        for ci in range(j.numCoordinates()):
            coord_to_joint[j.get_coordinates(ci).getName()] = j.getName()

    coord_order = list(coord_values.keys())
    labels = ["time"] + [
        f"/jointset/{coord_to_joint.get(n, 'unknown')}/{n}/value"
        for n in coord_order
    ]

    with open(path, "w", encoding="utf-8") as fh:
        fh.write(f"{os.path.basename(path)}\n")
        fh.write("version=1\n")
        fh.write(f"nRows={len(times)}\n")
        fh.write(f"nColumns={len(labels)}\n")
        fh.write("inDegrees=no\n")
        fh.write("endheader\n")
        fh.write("\t".join(labels) + "\n")
        for k, t in enumerate(times):
            row = [f"{t:.6f}"] + [f"{coord_values[n][k]:.6f}" for n in coord_order]
            fh.write("\t".join(row) + "\n")


def write_preview(model):
    os.makedirs(OUT_DIR, exist_ok=True)
    times, coord_values = _build_preview_motion(model)
    _write_sto(PREVIEW_STO, times, coord_values)
    print(f"Wrote kinematic preview: {PREVIEW_STO}")
    print("  Load Rajagopal2016_4regions_fingers.osim in the GUI, then")
    print(f"  File -> Load Motion... -> {PREVIEW_STO}")
    print("  to watch arms counter-swing the legs and fingers curl.")


# =========================================================================
# Optional forward-dynamics smoke test
# =========================================================================
def run_short_forward(model):
    print()
    print("Running 0.2 s forward sim to check dynamic validity ...")
    # Drop pelvis at a slightly elevated height so contacts engage and we
    # exercise the foot spheres.
    state = model.initSystem()
    cs = model.getCoordinateSet()
    cs.get("pelvis_ty").setValue(state, 0.96)
    manager = osim.Manager(model)
    manager.setIntegratorAccuracy(1e-3)
    manager.initialize(state)
    final_state = manager.integrate(0.2)
    print(f"  integrated to t = {final_state.getTime():.3f} s, OK")
    osim.STOFileAdapter.write(manager.getStatesTable(), FWD_STATES_STO)
    print(f"  wrote {FWD_STATES_STO}")


# =========================================================================
def main():
    print(f"Loading {MODEL_PATH}")
    model = osim.Model(MODEL_PATH)
    model.initSystem()

    print_inventory(model)
    write_preview(model)
    if RUN_FORWARD:
        run_short_forward(model)


if __name__ == "__main__":
    main()
