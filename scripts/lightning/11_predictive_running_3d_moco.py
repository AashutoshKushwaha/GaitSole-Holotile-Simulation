"""
Step 11 (PORTABLE / Lightning.ai build): 3D predictive running for
Rajagopal2016_4regions_fingers.osim.

Identical optimal-control problem to scripts/11_predictive_running_3d_moco.py,
but with platform-independent paths so it runs unchanged on a Linux cloud box
(e.g. a Lightning.ai Studio) AND on the Windows laptop. The ONLY differences vs
the original are this header, the path-resolution block, and a thread knob;
the model/weld/muscle/periodicity/guess logic below is byte-for-byte the same.

IMPORTANT: Moco is CPU-ONLY. IPOPT (the optimizer) and Simbody (the physics)
have no GPU/CUDA path -- a GPU machine would sit idle. Pick a HIGH-CORE-COUNT
CPU instance on Lightning, not a GPU one. More/faster CPU cores = faster solve.

-------------------------------------------------------------------------------
FILE LAYOUT EXPECTED (mirrors the laptop project)
  <PROJECT_ROOT>/
    Rajagopal2016_4regions_fingers.osim      <- upload this here
    scripts_lightning/
      11_predictive_running_3d_moco.py        <- this file
    output/                                    <- created automatically

  PROJECT_ROOT is auto-detected as the parent of this script's folder, i.e.
  on Lightning that is /teamspace/studios/this_studio/ when this file lives at
  /teamspace/studios/this_studio/scripts_lightning/. Override with the env var
  OPENSIM_PROJECT_ROOT if your layout differs.

RUN (after `pip install opensim` or conda-install into the default env):
  python -u scripts_lightning/11_predictive_running_3d_moco.py

Then download <PROJECT_ROOT>/output/run3d_solution.sto back to the laptop and
load it in the OpenSim GUI on top of the .osim model (File -> Load Motion).
-------------------------------------------------------------------------------

WHY this fixes the script-10 preview complaints (unchanged from the original):
  * "moving in one spot" -> pelvis_tx is FREE + MocoAverageSpeedGoal drives it.
  * "elbows don't bend"  -> elbow_flex_{r,l} FREE + actuator-driven + periodic.
  * "unnatural"          -> motion from physics + effort cost, not a sinusoid.

DESIGN: half-stride anti-symmetric predictive (Falisse 2019 / Moco
example3DWalking). Welds fingers/wrist/forearm-pronation/subtalar/MTP for
tractability; keeps the model 3D but tightly bounds out-of-plane DOFs; frees
the sagittal DOFs plus shoulder flexion + elbow. See the TUNING block in main().
"""

import math
import os

# --- Path resolution (platform independent) ----------------------------------
HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.environ.get("OPENSIM_PROJECT_ROOT") or os.path.dirname(HERE)

_MODEL_NAME = "Rajagopal2016_4regions_fingers.osim"
_MODEL_CANDIDATES = [
    os.path.join(PROJECT_ROOT, _MODEL_NAME),
    os.path.join(HERE, _MODEL_NAME),
    os.path.join(PROJECT_ROOT, "scripts", _MODEL_NAME),
    os.path.join(PROJECT_ROOT, "..", _MODEL_NAME),
]


def _resolve_model():
    """Find the .osim: try the obvious spots, then fall back to a one-pass walk
    of PROJECT_ROOT so it's found wherever it was uploaded in the Studio."""
    for p in _MODEL_CANDIDATES:
        if os.path.isfile(p):
            return p
    for root, _dirs, files in os.walk(PROJECT_ROOT):
        if _MODEL_NAME in files:
            return os.path.join(root, _MODEL_NAME)
    return _MODEL_CANDIDATES[0]  # not found; build_model_processor reports it


MODEL_PATH = _resolve_model()
OUT_DIR = os.path.join(PROJECT_ROOT, "output")
TEMP_MODEL = os.path.join(OUT_DIR, "_rajagopal_for_moco.osim")
OUT_SOLUTION = os.path.join(OUT_DIR, "run3d_solution.sto")

# --- Windows-only: make CasADi's IPOPT plugin loadable -----------------------
# On Windows, launching the conda env python.exe directly leaves Library/bin off
# the DLL search path, so solve() fails with "Plugin 'ipopt' is not found". On
# Linux/Mac (Lightning) conda/pip set the library path correctly, so this block
# is simply skipped (the directory doesn't exist). Must run BEFORE import opensim.
if os.name == "nt":
    _CONDA_BIN = r"E:/conda/envs/opensim_env/Library/bin"
    if os.path.isdir(_CONDA_BIN):
        os.environ["PATH"] = _CONDA_BIN + os.pathsep + os.environ.get("PATH", "")
        os.environ["CASADIPATH"] = _CONDA_BIN
        try:
            os.add_dll_directory(_CONDA_BIN)
        except (AttributeError, OSError):
            pass

import opensim as osim

# Silence the harmless "Couldn't find file '*.vtp'" geometry-mesh warnings.
# Those .vtp files are GUI display meshes only -- irrelevant to the solve. Moco
# reloads the model once per CPU worker thread, so on a many-core box this would
# otherwise flood the console with the same ~80 warnings per core. IPOPT's solve
# progress prints separately (stdout) and is NOT affected by this.
osim.Logger.setLevelString("Error")

# --- Gait targets --------------------------------------------------------
RUN_SPEED = 2.5            # m/s average forward speed. Lowered from 3.0 to ease
                           # the FIRST convergence; raise toward 3.0-3.5 on a
                           # warm-started re-run once a solution exists.
HALF_STRIDE_MIN = 0.22     # s   (right contact -> left contact)
HALF_STRIDE_MAX = 0.45     # s   (widened a touch to give the optimizer room)

# --- Solver settings -----------------------------------------------------
MESH_INTERVALS = 25
CONVERGENCE_TOL = 1e-2
# Constraint tol RELAXED 1e-4 -> 1e-3. A contact-rich predictive gait problem
# usually cannot drive the dynamics-defect violation below 1e-4, so 1e-4 makes
# IPOPT grind forever; 1e-3 is the standard, perfectly-valid gait tolerance.
CONSTRAINT_TOL = 1e-3
# Cap lower so that even a non-converged run still hits the auto-save (unseal +
# write) and gives you a warm-start trajectory in reasonable time.
MAX_ITERATIONS = 1000

# --- CPU parallelism -----------------------------------------------------
# Moco parallelizes per-mesh-point evaluations across threads. 0 => use ALL
# available cores (the right choice on a big cloud CPU box). Set a positive
# integer to cap it (e.g. to keep a laptop usable). Env override: OPENSIM_NUM_THREADS.
NUM_THREADS = int(os.environ.get("OPENSIM_NUM_THREADS", "0"))

# --- Strength scaling (Rajagopal muscles need a boost to run, as in 05) --
STRENGTH_SCALE = 3.0

# Joints to WELD (remove their DOFs entirely). Whole joints only -- every DOF
# of these joints is one we want frozen for a tractable sagittal-dominant run.
FINGER_PREFIXES = ("thumb_", "index_", "middle_", "ring_", "little_")
WELD_JOINTS_EXACT = (
    "radioulnar_r", "radioulnar_l",      # forearm pro/sup
    "radius_hand_r", "radius_hand_l",    # wrist
    "subtalar_r", "subtalar_l",
    "mtp_r", "mtp_l",
)


# =========================================================================
# Model preparation
# =========================================================================
def _weld_joint_paths_and_coords(model):
    """Return (list of joint names to weld, set of coordinate names that those
    joints own). Coordinate actuators on those coords must be removed before
    welding or finalizeConnections will fail on dangling sockets."""
    js = model.getJointSet()
    weld_paths = []
    weld_coords = set()
    for ji in range(js.getSize()):
        j = js.get(ji)
        name = j.getName()
        is_finger = any(name.startswith(p) for p in FINGER_PREFIXES)
        if is_finger or name in WELD_JOINTS_EXACT:
            # ModOpReplaceJointsWithWelds matches by joint NAME, not path.
            weld_paths.append(name)
            for ci in range(j.numCoordinates()):
                weld_coords.add(j.get_coordinates(ci).getName())
    return weld_paths, weld_coords


def _strip_actuators_on_coords(model, coord_names):
    """Remove CoordinateActuators whose coordinate is about to be welded away.
    Removal shifts indices, so collect first then remove by descending index."""
    fs = model.updForceSet()
    to_remove = []
    for i in range(fs.getSize()):
        f = fs.get(i)
        if f.getConcreteClassName() != "CoordinateActuator":
            continue
        act = osim.CoordinateActuator.safeDownCast(f)
        if act is None:
            continue
        if act.get_coordinate() in coord_names:
            to_remove.append(i)
    for i in reversed(to_remove):
        fs.remove(i)
    return len(to_remove)


def build_model_processor():
    """Load the base model, strip the actuators on soon-to-be-welded coords,
    save a temp .osim, then build a ModelProcessor that welds those joints and
    converts the muscles to the Moco-friendly DeGrooteFregly form."""
    if not os.path.isfile(MODEL_PATH):
        raise FileNotFoundError(
            f"Could not find {_MODEL_NAME}. Looked in:\n  "
            + "\n  ".join(os.path.abspath(p) for p in _MODEL_CANDIDATES)
            + f"\nUpload the model to {os.path.abspath(PROJECT_ROOT)} or set "
              "OPENSIM_PROJECT_ROOT to its parent folder.")

    base = osim.Model(MODEL_PATH)
    base.initSystem()

    weld_paths, weld_coords = _weld_joint_paths_and_coords(base)
    n_removed = _strip_actuators_on_coords(base, weld_coords)
    print(f"  removed {n_removed} CoordinateActuators on welded coords")
    print(f"  welding {len(weld_paths)} joints "
          f"({len(weld_coords)} DOFs: fingers/wrist/forearm/subtalar/mtp)")

    base.finalizeConnections()
    os.makedirs(OUT_DIR, exist_ok=True)
    base.printToXML(TEMP_MODEL)

    mp = osim.ModelProcessor(TEMP_MODEL)
    # Weld the unwanted joints (removes the DOFs from the optimal-control problem).
    weld = osim.StdVectorString()
    for p in weld_paths:
        weld.append(p)
    mp.append(osim.ModOpReplaceJointsWithWelds(weld))
    # Muscles: Millard -> DeGrooteFregly, rigid tendon, drop passive force, widen
    # active curve. This is the standard Moco muscle pipeline -- it makes the
    # problem smooth and removes per-muscle fiber-length states (huge speedup).
    mp.append(osim.ModOpReplaceMusclesWithDeGrooteFregly2016())
    mp.append(osim.ModOpIgnoreTendonCompliance())
    mp.append(osim.ModOpIgnorePassiveFiberForcesDGF())
    mp.append(osim.ModOpScaleActiveFiberForceCurveWidthDGF(1.5))
    mp.append(osim.ModOpScaleMaxIsometricForce(STRENGTH_SCALE))
    return mp


# =========================================================================
# Problem definition
# =========================================================================
def jp(joint, coord, kind):
    return f"/jointset/{joint}/{coord}/{kind}"


def D(deg):
    return deg * math.pi / 180.0


# Free sagittal DOFs: (jointset path, coord, value-bounds-rad). Speeds left free.
FREE_STATE_BOUNDS = [
    ("ground_pelvis", "pelvis_tilt", (D(-15), D(15))),
    ("ground_pelvis", "pelvis_ty",   (0.80, 1.10)),
    ("hip_r", "hip_flexion_r", (D(-30), D(70))),
    ("hip_l", "hip_flexion_l", (D(-30), D(70))),
    ("walker_knee_r", "knee_angle_r", (D(0), D(110))),
    ("walker_knee_l", "knee_angle_l", (D(0), D(110))),
    ("ankle_r", "ankle_angle_r", (D(-40), D(30))),
    ("ankle_l", "ankle_angle_l", (D(-40), D(30))),
    ("back", "lumbar_extension", (D(-30), D(10))),
    ("acromial_r", "arm_flex_r", (D(-60), D(60))),
    ("acromial_l", "arm_flex_l", (D(-60), D(60))),
    ("elbow_r", "elbow_flex_r", (D(10), D(130))),
    ("elbow_l", "elbow_flex_l", (D(10), D(130))),
]

# Out-of-plane DOFs: kept 3D but bounded. RELAXED vs. the first attempt -- the
# original ranges (pelvis sway +/-0.10 m, list +/-6 deg, etc.) were so tight
# that balancing a 3D forward run inside them was near-infeasible, which made
# IPOPT cycle on constraint violation instead of converging. These give the
# optimizer real room to balance while staying sagittal-dominant.
TIGHT_STATE_BOUNDS = [
    ("ground_pelvis", "pelvis_list",     (D(-15), D(15))),
    ("ground_pelvis", "pelvis_rotation", (D(-15), D(15))),
    ("ground_pelvis", "pelvis_tz",       (-0.25, 0.25)),
    ("hip_r", "hip_adduction_r", (D(-20), D(20))),
    ("hip_l", "hip_adduction_l", (D(-20), D(20))),
    ("hip_r", "hip_rotation_r",  (D(-25), D(25))),
    ("hip_l", "hip_rotation_l",  (D(-25), D(25))),
    ("back", "lumbar_bending",   (D(-20), D(20))),
    ("back", "lumbar_rotation",  (D(-20), D(20))),
    ("acromial_r", "arm_add_r",  (D(-40), D(40))),
    ("acromial_l", "arm_add_l",  (D(-40), D(40))),
    ("acromial_r", "arm_rot_r",  (D(-45), D(45))),
    ("acromial_l", "arm_rot_l",  (D(-45), D(45))),
]


def set_state_bounds(problem):
    # pelvis_tx is special: starts at 0, advances forward.
    problem.setStateInfo(jp("ground_pelvis", "pelvis_tx", "value"), [0.0, 3.0], 0.0)
    for joint, coord, (lo, hi) in FREE_STATE_BOUNDS + TIGHT_STATE_BOUNDS:
        problem.setStateInfo(jp(joint, coord, "value"), [lo, hi])


# --- Anti-symmetric periodicity: end of half-stride = mirror of the start ---
# Coords whose left/right values+speeds swap (the sagittal + bounded ones).
SWAP_COORDS = [
    ("hip_flexion_r", "hip_flexion_l", "hip_r", "hip_l"),
    ("hip_adduction_r", "hip_adduction_l", "hip_r", "hip_l"),
    ("hip_rotation_r", "hip_rotation_l", "hip_r", "hip_l"),
    ("knee_angle_r", "knee_angle_l", "walker_knee_r", "walker_knee_l"),
    ("ankle_angle_r", "ankle_angle_l", "ankle_r", "ankle_l"),
    ("arm_flex_r", "arm_flex_l", "acromial_r", "acromial_l"),
    ("arm_add_r", "arm_add_l", "acromial_r", "acromial_l"),
    ("arm_rot_r", "arm_rot_l", "acromial_r", "acromial_l"),
    ("elbow_flex_r", "elbow_flex_l", "elbow_r", "elbow_l"),
]

# Coords that return to their own start value (mid-line, symmetric).
SELF_COORDS = [
    ("pelvis_tilt", "ground_pelvis"),
    ("pelvis_ty", "ground_pelvis"),
    ("pelvis_list", "ground_pelvis"),
    ("pelvis_rotation", "ground_pelvis"),
    ("pelvis_tz", "ground_pelvis"),
    ("lumbar_extension", "back"),
    ("lumbar_bending", "back"),
    ("lumbar_rotation", "back"),
]

# CoordinateActuator controls that swap left/right (arms) or self-periodic
# (lumbar). NB: these are ACTUATOR names, not coordinate names -- the shoulder
# actuators are named shoulder_* even though they drive the arm_* coordinates.
SWAP_CONTROLS = ["shoulder_flex", "shoulder_add", "shoulder_rot", "elbow_flex"]
SELF_CONTROLS = ["lumbar_ext", "lumbar_bend", "lumbar_rot"]


def add_periodicity(problem, model):
    per = osim.MocoPeriodicityGoal("periodicity")

    # pelvis_tx position advances, but its SPEED must be periodic.
    per.addStatePair(osim.MocoPeriodicityGoalPair(
        jp("ground_pelvis", "pelvis_tx", "speed")))

    for coord, joint in SELF_COORDS:
        per.addStatePair(osim.MocoPeriodicityGoalPair(jp(joint, coord, "value")))
        per.addStatePair(osim.MocoPeriodicityGoalPair(jp(joint, coord, "speed")))

    for cr, cl, jr, jl in SWAP_COORDS:
        for kind in ("value", "speed"):
            a = jp(jr, cr, kind)
            b = jp(jl, cl, kind)
            per.addStatePair(osim.MocoPeriodicityGoalPair(a, b))
            per.addStatePair(osim.MocoPeriodicityGoalPair(b, a))

    # Muscle activations swap left/right (every muscle ends in _r or _l).
    for mus in model.getMuscles():
        nm = mus.getName()
        if nm.endswith("_r"):
            base = nm[:-2]
            ar = f"/forceset/{base}_r/activation"
            al = f"/forceset/{base}_l/activation"
            per.addStatePair(osim.MocoPeriodicityGoalPair(ar, al))
            per.addStatePair(osim.MocoPeriodicityGoalPair(al, ar))
            per.addControlPair(osim.MocoPeriodicityGoalPair(
                f"/forceset/{base}_r", f"/forceset/{base}_l"))
            per.addControlPair(osim.MocoPeriodicityGoalPair(
                f"/forceset/{base}_l", f"/forceset/{base}_r"))

    for c in SWAP_CONTROLS:
        per.addControlPair(osim.MocoPeriodicityGoalPair(
            f"/forceset/{c}_r", f"/forceset/{c}_l"))
        per.addControlPair(osim.MocoPeriodicityGoalPair(
            f"/forceset/{c}_l", f"/forceset/{c}_r"))
    for c in SELF_CONTROLS:
        per.addControlPair(osim.MocoPeriodicityGoalPair(f"/forceset/{c}"))

    problem.addGoal(per)


def seed_running_guess(solver):
    """Replace the flat default guess with a running-shaped one: a forward
    pelvis ramp, anti-phase hip/arm swing, bent knees in swing, and elbows held
    flexed. A good guess is the difference between converging to running vs.
    stalling or collapsing to a walk."""
    guess = solver.createGuess("bounds")
    t = guess.getTime()
    n = t.size()
    tf = t.get(n - 1) if n > 1 else HALF_STRIDE_MIN

    def setv(path, fn):
        col = osim.Vector(n, 0.0)
        for i in range(n):
            col.set(i, fn(t.get(i)))
        guess.setState(path, col)

    omega = math.pi / tf  # half a stride spans pi
    setv(jp("ground_pelvis", "pelvis_tx", "value"), lambda x: RUN_SPEED * x)
    setv(jp("ground_pelvis", "pelvis_tx", "speed"), lambda x: RUN_SPEED)
    setv(jp("ground_pelvis", "pelvis_ty", "value"), lambda x: 0.95)
    setv(jp("hip_r", "hip_flexion_r", "value"), lambda x: D(35) * math.cos(omega * x))
    setv(jp("hip_l", "hip_flexion_l", "value"), lambda x: -D(35) * math.cos(omega * x))
    setv(jp("walker_knee_r", "knee_angle_r", "value"),
         lambda x: D(45) * 0.5 * (1 - math.cos(2 * omega * x)) + D(10))
    setv(jp("walker_knee_l", "knee_angle_l", "value"),
         lambda x: D(45) * 0.5 * (1 + math.cos(2 * omega * x)) + D(10))
    # Arms counter-swing the legs; elbows held flexed (~75 deg) so they READ as
    # bent from the first iteration.
    setv(jp("acromial_r", "arm_flex_r", "value"), lambda x: -D(30) * math.cos(omega * x))
    setv(jp("acromial_l", "arm_flex_l", "value"), lambda x: D(30) * math.cos(omega * x))
    setv(jp("elbow_r", "elbow_flex_r", "value"), lambda x: D(75))
    setv(jp("elbow_l", "elbow_flex_l", "value"), lambda x: D(75))

    solver.setGuess(guess)


# =========================================================================
def main():
    print(f"PROJECT_ROOT : {os.path.abspath(PROJECT_ROOT)}")
    print(f"MODEL_PATH   : {os.path.abspath(MODEL_PATH)}")
    print(f"OUTPUT       : {os.path.abspath(OUT_SOLUTION)}")
    print("Building model processor (weld + DeGrooteFregly conversion)...")
    mp = build_model_processor()
    model = mp.process()
    model.initSystem()
    print(f"  processed model: {model.getCoordinateSet().getSize()} coords, "
          f"{model.getMuscles().getSize()} muscles")

    study = osim.MocoStudy()
    study.setName("run_3d")
    problem = study.updProblem()
    problem.setModelProcessor(mp)

    problem.setTimeBounds(0.0, [HALF_STRIDE_MIN, HALF_STRIDE_MAX])
    set_state_bounds(problem)

    # --- Goals -----------------------------------------------------------
    # Effort: low weight so the optimizer prioritizes actually running.
    problem.addGoal(osim.MocoControlGoal("effort", 1.0))

    speed = osim.MocoAverageSpeedGoal("speed")
    speed.set_desired_average_speed(RUN_SPEED)
    problem.addGoal(speed)

    add_periodicity(problem, model)

    # --- Solver ----------------------------------------------------------
    solver = study.initCasADiSolver()
    solver.set_num_mesh_intervals(MESH_INTERVALS)
    solver.set_optim_convergence_tolerance(CONVERGENCE_TOL)
    solver.set_optim_constraint_tolerance(CONSTRAINT_TOL)
    solver.set_optim_max_iterations(MAX_ITERATIONS)
    # Patellofemoral coupler constraints -> let Moco enforce constraint
    # derivatives for a clean, drift-free solution.
    solver.set_enforce_constraint_derivatives(True)
    # IMPLICIT multibody dynamics: the single biggest robustness aid for
    # contact-driven gait. Instead of forward dynamics (invert the mass matrix
    # every evaluation, which is stiff and ill-conditioned with hard foot
    # contact), accelerations become variables and Newton's law becomes a
    # constraint. This is what stops the feasibility cycling we saw at coarse
    # mesh. A small acceleration regularizer keeps the accelerations smooth.
    solver.set_multibody_dynamics_mode("implicit")
    solver.set_minimize_implicit_multibody_accelerations(True)
    solver.set_implicit_multibody_accelerations_weight(1e-4)
    # Thread count: 0 leaves Moco's default (all cores); >0 caps it.
    if NUM_THREADS > 0:
        solver.set_parallel(NUM_THREADS)
        print(f"  capped solver to {NUM_THREADS} threads")
    else:
        print("  using all available CPU cores")

    seed_running_guess(solver)

    print(f"\nSolving 3D running half-stride at {RUN_SPEED} m/s "
          f"({MESH_INTERVALS} mesh intervals)...")
    print("This is the slow step. Watch for IPOPT 'EXIT: Optimal Solution Found.'\n")
    solution = study.solve()

    # Even an unconverged ("...sealed") solution is worth saving as a warm start.
    if not solution.success():
        print("\n[!] Solver did NOT fully converge. Unsealing to save the best "
              "trajectory anyway (use it as a warm start with a finer mesh).")
        solution.unseal()
    solution.write(OUT_SOLUTION)
    print(f"\nWrote {OUT_SOLUTION}")
    print("Download this .sto, load Rajagopal2016_4regions_fingers.osim in the")
    print(f"  GUI, then File -> Load Motion -> {os.path.basename(OUT_SOLUTION)}")

    # ----------------------------- TUNING -------------------------------
    # If the result looks wrong, in rough order of effectiveness:
    #   * It collapses / can't support itself -> raise STRENGTH_SCALE (3 -> 4).
    #   * It walks instead of runs            -> raise RUN_SPEED toward 3.5-4.0
    #                                            and lower HALF_STRIDE_MAX.
    #   * It stalls / hits max iterations     -> it likely still saved an
    #                                            unsealed guess; re-run with
    #                                            MESH_INTERVALS bumped to 40-50
    #                                            using this .sto as the guess
    #                                            (replace createGuess with
    #                                            solver.setGuessFile(OUT_SOLUTION)).
    #   * Want true full 3D                   -> widen TIGHT_STATE_BOUNDS and
    #                                            drop welds, warm-started here.


if __name__ == "__main__":
    main()
