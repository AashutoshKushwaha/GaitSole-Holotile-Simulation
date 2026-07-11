"""
Step 1: Inspect the Moco-ready 2D gait model.

Loads 2D_gait.osim (ships with OpenSim 4.5 in the example2DWalking folder)
and prints every joint, coordinate, muscle, and contact sphere.

NOTE: this model puts its forces (and muscles) in the top-level <components>
block rather than the legacy <ForceSet>, so we enumerate via
model.getComponentsList(), not model.getForceSet() / getMuscles().
"""
import opensim as osim
from collections import defaultdict

MODEL = "E:/OpenSim/4.5/Code/Matlab/Moco/example2DWalking/2D_gait.osim"

model = osim.Model(MODEL)
model.initSystem()

print(f"Model: {model.getName()}")
print(f"Bodies: {model.getBodySet().getSize()}, "
      f"Joints: {model.getJointSet().getSize()}\n")

# Group components by class for a clean print.
by_class = defaultdict(list)
for c in model.getComponentsList():
    by_class[c.getConcreteClassName()].append(c)

# 1. Joints (these are what you'll actuate during running).
print("--- JOINTS ---")
for j in model.getJointSet():
    print(f"  {j.getName():25s}  {j.getConcreteClassName()}")

# 2. Coordinates (every DOF you can drive).
print("\n--- COORDINATES ---")
for c in model.getCoordinateSet():
    print(f"  {c.getName():25s}  default={c.getDefaultValue():+0.3f}")

# 3. Muscles (DeGrooteFregly2016Muscle is Moco-ready).
muscles = by_class.get("DeGrooteFregly2016Muscle", [])
print(f"\n--- MUSCLES ({len(muscles)} DeGrooteFregly2016) ---")
for m in muscles:
    print(f"  {m.getName()}")

# 4. Other actuators (CoordinateActuator for lumbar in this model).
actuators = by_class.get("CoordinateActuator", [])
if actuators:
    print(f"\n--- COORDINATE ACTUATORS ({len(actuators)}) ---")
    for a in actuators:
        print(f"  {a.getName():25s}  path={a.getAbsolutePathString()}")

# 5. Foot contact forces (your virtual load cells).
forces = by_class.get("SmoothSphereHalfSpaceForce", [])
print(f"\n--- FOOT CONTACT FORCES ({len(forces)} spheres) ---")
for f in forces:
    print(f"  {f.getName():25s}  path={f.getAbsolutePathString()}")

# 6. Contact geometries (the floor + spheres themselves).
spheres = by_class.get("ContactSphere", [])
halfsp  = by_class.get("ContactHalfSpace", [])
print(f"\n--- CONTACT GEOMETRY ({len(spheres)} spheres + {len(halfsp)} half-spaces) ---")
for s in spheres + halfsp:
    print(f"  {s.getConcreteClassName():18s}  {s.getName():14s}  path={s.getAbsolutePathString()}")
