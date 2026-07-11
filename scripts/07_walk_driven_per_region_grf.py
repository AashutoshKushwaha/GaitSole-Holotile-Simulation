"""
Step 7 (fast, reliable): drive the 4-region model with the SHIPPED walking
reference motion and extract per-region GRF for a real biomechanical gait.

Reads:   E:/OpenSim/4.5/Code/Matlab/Moco/example2DWalking/referenceCoordinates.sto
         (50 timesteps, 10 coordinate values, no speeds)
Writes:  E:/OpenSim/output/walk_GRF.sto
         (per-region force triples for all 8 spheres across the cycle)

Procedure each timestep:
  1. Set every coordinate value from the reference.
  2. Set every coordinate speed from a central-difference of the values.
  3. Realize state to Velocity stage (needed for contact damping).
  4. Call getRecordValues() on each SmoothSphereHalfSpaceForce.
"""
import opensim as osim
import numpy as np

MODEL = "E:/OpenSim/models/2D_gait_4regions.osim"
REF   = "E:/OpenSim/4.5/Code/Matlab/Moco/example2DWalking/referenceCoordinates.sto"
OUT   = "E:/OpenSim/output/walk_GRF.sto"

model = osim.Model(MODEL)
state = model.initSystem()

# Load the kinematics table.
tbl = osim.TimeSeriesTable(REF)
times = np.array(list(tbl.getIndependentColumn()))
labels = list(tbl.getColumnLabels())
values = np.column_stack([tbl.getDependentColumn(l).to_numpy() for l in labels])

# Numerically differentiate to get speeds (central differences).
speeds = np.gradient(values, times, axis=0)

# Build coordinate-name -> column-index map. Labels look like
# "/jointset/hip_l/hip_flexion_l/value"; the coord name is the leaf folder.
coord_cols = {}
for i, lab in enumerate(labels):
    if lab.endswith("/value"):
        coord_name = lab[:-6].split("/")[-1]
        coord_cols[coord_name] = i

# Find every contact sphere force.
spheres = [(c.getName(), osim.SmoothSphereHalfSpaceForce.safeDownCast(c))
           for c in model.getComponentsList()
           if c.getConcreteClassName() == "SmoothSphereHalfSpaceForce"]
print(f"Driving {len(times)} timesteps of walking through {len(spheres)} contact regions.")

coords = model.getCoordinateSet()
out_labels = ["time"]
for name, _ in spheres:
    out_labels += [f"{name}.Fx", f"{name}.Fy", f"{name}.Fz"]

rows = []
for ti, t in enumerate(times):
    for coord_name, col in coord_cols.items():
        try:
            coords.get(coord_name).setValue(state, float(values[ti, col]), False)
            coords.get(coord_name).setSpeedValue(state, float(speeds[ti, col]))
        except Exception:
            pass
    state.setTime(t)
    model.realizeVelocity(state)
    row = [t]
    for _, f in spheres:
        rec = f.getRecordValues(state)
        row += [rec.get(0), rec.get(1), rec.get(2)]
    rows.append(row)

with open(OUT, "w") as fh:
    fh.write("walk_per_region_GRF\n")
    fh.write("version=1\n")
    fh.write(f"nRows={len(rows)}\n")
    fh.write(f"nColumns={len(out_labels)}\n")
    fh.write("inDegrees=no\n")
    fh.write("endheader\n")
    fh.write("\t".join(out_labels) + "\n")
    for r in rows:
        fh.write("\t".join(f"{v:0.6f}" for v in r) + "\n")
print(f"Wrote {OUT}  ({len(rows)} rows x {len(out_labels)} columns)")

# Also write a motion file so you can replay walking in the GUI.
# (referenceCoordinates.sto already plays as a motion, but copying it next
# to walk_GRF.sto keeps the result-folder self-contained.)
import shutil
shutil.copyfile(REF, "E:/OpenSim/output/walk_motion.sto")
print("Copied walking motion to E:/OpenSim/output/walk_motion.sto for convenience.")
