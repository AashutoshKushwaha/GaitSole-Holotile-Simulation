"""
Step 6: Extract per-region foot GRF from a previously-solved Moco running
solution (run_solution.sto), without re-running the 11-minute solver.

Reads:   E:/OpenSim/output/run_solution.sto   (kinematics + activations)
Writes:  E:/OpenSim/output/run_GRF.sto        (per-region force triples)

Each timestep:
  1. Sets every joint coordinate value and speed from the saved trajectory.
  2. Realizes the state to the Velocity stage (needed for contact damping).
  3. Calls getRecordValues() on every SmoothSphereHalfSpaceForce to read
     (Fx, Fy, Fz) on each sphere -- the per-region load-cell numbers.
"""
import opensim as osim

MODEL    = "E:/OpenSim/models/2D_gait_4regions.osim"
SOLUTION = "E:/OpenSim/output/run_solution.sto"
OUT_GRF  = "E:/OpenSim/output/run_GRF.sto"

# Match the model that was used during the solve (script 05 boosted muscle
# strengths by 3x, so we replicate that here for a consistent replay).
model = osim.Model(MODEL)
for mus in model.getMuscles():
    mus.set_max_isometric_force(3.0 * mus.get_max_isometric_force())
state = model.initSystem()

# Load the Moco trajectory and mirror it into a full periodic stride.
traj = osim.MocoTrajectory(SOLUTION)
try:
    traj = osim.createPeriodicTrajectory(traj)
    print("Mirrored half-stride into full periodic stride.")
except Exception as e:
    print(f"Note: could not build periodic trajectory ({e}); using as-is.")

times = list(traj.getTimeMat())
state_names = list(traj.getStateNames())
state_mat = traj.getStatesTrajectoryMat()       # shape (n_time, n_states)

# Map coordinate name -> column index for /value and /speed.
val_col = {}
spd_col = {}
for i, n in enumerate(state_names):
    if n.endswith("/value"):
        val_col[n[:-6].split("/")[-1]] = i
    elif n.endswith("/speed"):
        spd_col[n[:-6].split("/")[-1]] = i

# Find every contact sphere force.
spheres = [(c.getName(), osim.SmoothSphereHalfSpaceForce.safeDownCast(c))
           for c in model.getComponentsList()
           if c.getConcreteClassName() == "SmoothSphereHalfSpaceForce"]
print(f"Replaying {len(times)} timesteps through {len(spheres)} contact regions...")

coords = model.getCoordinateSet()

labels = ["time"]
for name, _ in spheres:
    labels += [f"{name}.Fx", f"{name}.Fy", f"{name}.Fz"]

rows = []
for ti, t in enumerate(times):
    for cname, col in val_col.items():
        try:
            coords.get(cname).setValue(state, float(state_mat[ti, col]), False)
        except Exception:
            pass
    for cname, col in spd_col.items():
        try:
            coords.get(cname).setSpeedValue(state, float(state_mat[ti, col]))
        except Exception:
            pass
    state.setTime(t)
    model.realizeVelocity(state)
    row = [t]
    for _, f in spheres:
        rec = f.getRecordValues(state)
        row += [rec.get(0), rec.get(1), rec.get(2)]
    rows.append(row)

with open(OUT_GRF, "w") as fh:
    fh.write("run_per_region_GRF\n")
    fh.write("version=1\n")
    fh.write(f"nRows={len(rows)}\n")
    fh.write(f"nColumns={len(labels)}\n")
    fh.write("inDegrees=no\n")
    fh.write("endheader\n")
    fh.write("\t".join(labels) + "\n")
    for r in rows:
        fh.write("\t".join(f"{v:0.6f}" for v in r) + "\n")

print(f"Wrote {OUT_GRF}  ({len(rows)} rows x {len(labels)} columns)")
print("\nNext: open this .sto in the OpenSim GUI Plotter to see per-region GRF curves,")
print("or re-use script 04's plotting code to make a PNG.")
