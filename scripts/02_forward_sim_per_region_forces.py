"""
Step 2: Forward simulation + read per-contact (per-region) force.

Uses the SHIPPED 2D_gait.osim directly. That model already has 4 contact
spheres (heel + front on each foot) wired in the top-level <components>
block. Each sphere reports its own force vector, which is your per-region
"load-cell" reading.

We retrieve the SmoothSphereHalfSpaceForce components by absolute path with
model.getComponent('/contactHeel_r') because they are NOT in the model's
ForceSet -- they are siblings of the ForceSet under the Model root.

Two readouts are produced:
  A) Live, in the integration loop:  f.getRecordValues(state)
        -> 12 numbers per sphere (Fx, Fy, Fz, Tx, Ty, Tz on the sphere,
           then equal-and-opposite on the half-space), in ground frame.
  B) Logged to a .sto file via the ForceReporter analysis.
"""
import opensim as osim
import os, sys

# Pass "4regions" as the first CLI arg to use the 4-spheres-per-foot model
# built by script 03. Default is the shipped 2-spheres-per-foot model.
if len(sys.argv) > 1 and sys.argv[1] == "4regions":
    MODEL = "E:/OpenSim/models/2D_gait_4regions.osim"
else:
    MODEL = "E:/OpenSim/4.5/Code/Matlab/Moco/example2DWalking/2D_gait.osim"

OUT_DIR = "E:/OpenSim/output"
os.makedirs(OUT_DIR, exist_ok=True)
print(f"Using model: {MODEL}")

model = osim.Model(MODEL)

# Attach a ForceReporter BEFORE initSystem so it sees every force.
reporter = osim.ForceReporter(model)
reporter.setName("forces")
model.addAnalysis(reporter)

state = model.initSystem()
model.equilibrateMuscles(state)

# Drop the model from just above ground so the spheres definitely make contact.
model.getCoordinateSet().get("pelvis_ty").setValue(state, 0.95)

# Auto-discover every SmoothSphereHalfSpaceForce in the model -- works for
# both the shipped 2-sphere variant and the 4-region variant.
sphere_forces = []
sphere_names = []
for c in model.getComponentsList():
    if c.getConcreteClassName() == "SmoothSphereHalfSpaceForce":
        sphere_forces.append(osim.SmoothSphereHalfSpaceForce.safeDownCast(c))
        sphere_names.append(c.getName())

manager = osim.Manager(model)
state.setTime(0.0)
manager.initialize(state)

short = [n.replace("contact", "") for n in sphere_names]
print(f"{'time':>6s} | " + " | ".join(f"{s:>10s}" for s in short))
for step in range(20):                          # 20 samples across 0.4 s
    state = manager.integrate(0.02 * (step + 1))
    row = []
    for f in sphere_forces:
        rec = f.getRecordValues(state)
        fy = rec.get(1)                         # vertical force on the foot
        row.append(f"{fy:+9.1f}")
    print(f"{state.getTime():6.3f} | " + " | ".join(f"{r:>10s}" for r in row))

reporter.printResults("fwd", OUT_DIR, 0.001, ".sto")
# The output filename is built from the prefix above + the analysis name.
print(f"\nWrote per-force time series to {OUT_DIR}/fwd_forces_forces.sto")
