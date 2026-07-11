"""
Step 11: 3D predictive running for Rajagopal2016_4regions_fingers.osim.

This is the REAL physics solve -- the slow one. Unlike script 10 (which just
writes hand-made sine waves to a .sto and appears instantly), this script
hands the model to Moco's optimal-control solver, which searches for a
dynamically consistent running half-stride: muscles generate forces, the
4-region foot contacts generate the GRF, and the optimizer finds the muscle
and arm-actuator excitations that propel the model FORWARD at the target
speed while minimizing effort. Expect a long solve (see SOLVE NOTES below).

It is the 3D successor to script 05 (the 2D predictive run). The formulation
is the proven half-stride / anti-symmetric-periodicity recipe from script 05,
extended to 3D with arm swing, following Falisse 2019 and the OpenSim Moco
`example3DWalking`.

WHY this fixes the three complaints about the script-10 preview:
  * "moving in one spot"  -> here pelvis_tx is FREE and a MocoAverageSpeedGoal
                             drives it forward at RUN_SPEED m/s.
  * "elbows don't bend"   -> elbow_flex_{r,l} are FREE, actuator-driven, and
                             tied left<->right by periodicity, so the optimizer
                             swings and bends them to counter leg angular
                             momentum (Hamner & Delp 2013).
  * "unnatural"           -> the motion is now produced by physics + an effort
                             cost, not by a scripted sinusoid.

DESIGN (tractability vs. fidelity)
  Full 3D predictive running with every frontal/transverse DOF free AND 28
  finger DOFs is a research-grade, multi-hour-to-days solve that often fails
  to converge on a laptop. So we make the standard reductions:
    - WELD finger / wrist / forearm-pronation / subtalar / MTP joints. These
      contribute <1% of running angular momentum but cost a lot of DOFs and
      convergence trouble. Welded => bodies (and the toe contact sphere) stay
      attached and visible; the fingers simply hold a fixed relaxed pose.
    - Keep the model 3D but TIGHTLY BOUND the out-of-plane DOFs (pelvis
      list/rotation/sway, hip ab/rotation, lumbar bend/rotation, shoulder
      ad/rotation) near their defaults. The sagittal DOFs (incl. shoulder
      flexion + elbow) are free with running-sized ranges.
  To go to full 3D later, widen those bounds and remove welds -- but warm-start
  from THIS solution, don't start from scratch.

OUTPUT
  E:/OpenSim/output/run3d_solution.sto   -- the solved half-stride trajectory.
  Load Rajagopal2016_4regions_fingers.osim in the GUI, then
  File -> Load Motion -> run3d_solution.sto to watch it. (For a full stride,
  mirror/tile it the same way scripts 06/08 did for the 2D solution.)

SOLVE NOTES
  Watch the console for IPOPT's "EXIT: Optimal Solution Found." On a laptop
  this can take from ~20 min to a few hours depending on cores. If it stalls
  or returns a non-running gait, see the TUNING block at the bottom of main().

Run with the opensim conda env:
  E:/conda/envs/opensim_env/python.exe E:/OpenSim/scripts/11_predictive_running_3d_moco.py
"""

import math
import os

# --- Make CasADi's IPOPT plugin loadable -------------------------------------
# Running this conda env's python.exe directly (instead of `conda activate`)
# leaves the env's Library/bin off the DLL search path, so at solve() time
# CasADi can't find casadi_nlpsol_ipopt.dll's dependency ipopt-3.dll and the
# solve dies with "Plugin 'ipopt' is not found" (WIN32 error 126). Putting
# Library/bin on PATH/CASADIPATH (CasADi searches both) fixes it. Must happen
# BEFORE `import opensim`.
_CONDA_BIN = r"E:/conda/envs/opensim_env/Library/bin"
if os.path.isdir(_CONDA_BIN):
    os.environ["PATH"] = _CONDA_BIN + os.pathsep + os.environ.get("PATH", "")
    os.environ["CASADIPATH"] = _CONDA_BIN
    try:
        os.add_dll_directory(_CONDA_BIN)
    except (AttributeError, OSError):
        pass

import opensim as osim

MODEL_PATH = "E:/OpenSim/models/Rajagopal2016_4regions_fingers.osim"
OUT_DIR = "E:/OpenSim/output"
TEMP_MODEL = os.path.join(OUT_DIR, "_rajagopal_for_moco.osim")
OUT_SOLUTION = os.path.join(OUT_DIR, "run3d_solution.sto")

# --- Gait targets --------------------------------------------------------
RUN_SPEED = 3.0            # m/s average forward speed (jog; raise toward 3.5+
                           # once a solution exists to use as a warm start).
HALF_STRIDE_MIN = 0.22     # s   (right contact -> left contact)
HALF_STRIDE_MAX = 0.40     # s

# --- Solver coarseness (start coarse; refine via warm start) -------------
MESH_INTERVALS = 25
CONVERGENCE_TOL = 1e-2
CONSTRAINT_TOL = 1e-4
MAX_ITERATIONS = 1500

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
    """Return (list of jointset paths to weld, set of coordinate names that
    those joints own). Coordinate actuators on those coords must be removed
    before welding or finalizeConnections will fail on dangling sockets."""
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

# Out-of-plane DOFs kept but pinned near default with tight bounds (keeps the
# model genuinely 3D / dynamically valid without exploding the search space).
TIGHT_STATE_BOUNDS = [
    ("ground_pelvis", "pelvis_list",     (D(-6), D(6))),
    ("ground_pelvis", "pelvis_rotation", (D(-8), D(8))),
    ("ground_pelvis", "pelvis_tz",       (-0.10, 0.10)),
    ("hip_r", "hip_adduction_r", (D(-12), D(12))),
    ("hip_l", "hip_adduction_l", (D(-12), D(12))),
    ("hip_r", "hip_rotation_r",  (D(-15), D(15))),
    ("hip_l", "hip_rotation_l",  (D(-15), D(15))),
    ("back", "lumbar_bending",   (D(-10), D(10))),
    ("back", "lumbar_rotation",  (D(-10), D(10))),
    ("acromial_r", "arm_add_r",  (D(-20), D(20))),
    ("acromial_l", "arm_add_l",  (D(-20), D(20))),
    ("acromial_r", "arm_rot_r",  (D(-30), D(30))),
    ("acromial_l", "arm_rot_l",  (D(-30), D(30))),
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
    print("Load Rajagopal2016_4regions_fingers.osim in the GUI, then")
    print(f"  File -> Load Motion -> {OUT_SOLUTION}")

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
