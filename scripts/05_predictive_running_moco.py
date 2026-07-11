"""
Step 5: Use Moco to predict a running cycle on 2D_gait_4regions.osim.

This is a HALF-stride formulation with proper left-right anti-symmetric
periodicity. The half-stride is mirrored into a full stride during post-
processing (see script 06).

Critical fix vs the first attempt:
   The MocoPeriodicityGoal now wires every left/right state and control
   pair so that, at the end of the half-stride, the right side's value
   equals the left side's value at the start (and vice versa). Without
   these pairs, the optimizer's cheapest periodic solution is "both legs
   do the same thing", which looks like bilateral hopping rather than
   running. The fix forces left-right alternation = real gait.

This script ONLY solves and saves run_solution.sto. Per-region GRF
extraction is done by script 06 against the saved solution. That way the
~11-min solve isn't wasted if a post-processing bug appears.

Expect 10-30 min solve time. Watch for IPOPT's "EXIT: Optimal Solution Found."
"""
import opensim as osim

MODEL = "E:/OpenSim/models/2D_gait_4regions.osim"
OUT_SOLUTION = "E:/OpenSim/output/run_solution.sto"


def build_model_with_strength_scaling():
    m = osim.Model(MODEL)
    # Boost max isometric force so the muscles can drive running.
    for mus in m.getMuscles():
        mus.set_max_isometric_force(3.0 * mus.get_max_isometric_force())
    return m


study = osim.MocoStudy()
study.setName("run_2D")
problem = study.updProblem()
problem.setModel(build_model_with_strength_scaling())

# Half-stride duration (right heel strike to left heel strike).
problem.setTimeBounds(0.0, [0.25, 0.40])

# Coordinate bounds.
problem.setStateInfo("/jointset/groundPelvis/pelvis_tilt/value", [-20*3.14/180, 20*3.14/180])
problem.setStateInfo("/jointset/groundPelvis/pelvis_tx/value",   [0, 3])
problem.setStateInfo("/jointset/groundPelvis/pelvis_ty/value",   [0.75, 1.30])
problem.setStateInfo("/jointset/hip_r/hip_flexion_r/value",      [-30*3.14/180, 60*3.14/180])
problem.setStateInfo("/jointset/hip_l/hip_flexion_l/value",      [-30*3.14/180, 60*3.14/180])
problem.setStateInfo("/jointset/knee_r/knee_angle_r/value",      [-110*3.14/180, 0])
problem.setStateInfo("/jointset/knee_l/knee_angle_l/value",      [-110*3.14/180, 0])
problem.setStateInfo("/jointset/ankle_r/ankle_angle_r/value",    [-40*3.14/180, 40*3.14/180])
problem.setStateInfo("/jointset/ankle_l/ankle_angle_l/value",    [-40*3.14/180, 40*3.14/180])

# Goals ----------------------------------------------------------------
problem.addGoal(osim.MocoControlGoal("effort", 10.0))

speed = osim.MocoAverageSpeedGoal("speed")
speed.set_desired_average_speed(3.5)            # m/s - running
problem.addGoal(speed)

# Anti-symmetric periodicity: at the half-stride boundary the right side
# becomes the new left side and vice versa.
per = osim.MocoPeriodicityGoal("periodicity")

# 1. Pelvis tilt + vertical + lumbar are symmetric: same value at end as start.
for s in [
    "/jointset/groundPelvis/pelvis_tilt/value",
    "/jointset/groundPelvis/pelvis_tilt/speed",
    "/jointset/groundPelvis/pelvis_ty/value",
    "/jointset/groundPelvis/pelvis_ty/speed",
    "/jointset/groundPelvis/pelvis_tx/speed",   # tx position advances; speed periodic.
    "/jointset/lumbar/lumbar/value",
    "/jointset/lumbar/lumbar/speed",
]:
    per.addStatePair(osim.MocoPeriodicityGoalPair(s))

# 2. Left/right joint values and speeds swap.
swap_pairs = [
    ("/jointset/hip_l/hip_flexion_l/value",  "/jointset/hip_r/hip_flexion_r/value"),
    ("/jointset/hip_l/hip_flexion_l/speed",  "/jointset/hip_r/hip_flexion_r/speed"),
    ("/jointset/knee_l/knee_angle_l/value",  "/jointset/knee_r/knee_angle_r/value"),
    ("/jointset/knee_l/knee_angle_l/speed",  "/jointset/knee_r/knee_angle_r/speed"),
    ("/jointset/ankle_l/ankle_angle_l/value","/jointset/ankle_r/ankle_angle_r/value"),
    ("/jointset/ankle_l/ankle_angle_l/speed","/jointset/ankle_r/ankle_angle_r/speed"),
]
for a, b in swap_pairs:
    per.addStatePair(osim.MocoPeriodicityGoalPair(a, b))
    per.addStatePair(osim.MocoPeriodicityGoalPair(b, a))

# 3. Muscle activation states swap left/right; controls (excitations) too.
muscles = ["hamstrings", "bifemsh", "glut_max", "iliopsoas",
           "rect_fem", "vasti", "gastroc", "soleus", "tib_ant"]
for m in muscles:
    a_l = f"/{m}_l/activation"
    a_r = f"/{m}_r/activation"
    per.addStatePair(osim.MocoPeriodicityGoalPair(a_l, a_r))
    per.addStatePair(osim.MocoPeriodicityGoalPair(a_r, a_l))
    per.addControlPair(osim.MocoPeriodicityGoalPair(f"/{m}_l", f"/{m}_r"))
    per.addControlPair(osim.MocoPeriodicityGoalPair(f"/{m}_r", f"/{m}_l"))

# 4. Lumbar actuator control is symmetric.
per.addControlPair(osim.MocoPeriodicityGoalPair("/lumbarAct"))

problem.addGoal(per)

# Solver ---------------------------------------------------------------
solver = study.initCasADiSolver()
solver.set_num_mesh_intervals(50)
solver.set_optim_convergence_tolerance(1e-3)
solver.set_optim_constraint_tolerance(1e-3)

print("Solving running half-stride (this is the slow step)...")
solution = study.solve()
solution.write(OUT_SOLUTION)
print(f"Wrote {OUT_SOLUTION}")
print("\nNext: run 06_extract_grf_from_solution.py to get per-region GRF.")
