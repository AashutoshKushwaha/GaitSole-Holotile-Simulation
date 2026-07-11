"""
Step 4: Read the per-region force time series from the .sto file produced
by ForceReporter (script 02) and plot it.

Each SmoothSphereHalfSpaceForce contributes 12 columns to the .sto file:
   <name>.sphere.force.X / .Y / .Z       force on the foot (in ground frame)
   <name>.sphere.torque.X / .Y / .Z      torque on the foot
   <name>.half_space.force.X / .Y / .Z   equal-and-opposite on the floor
   <name>.half_space.torque.X / .Y / .Z

For a vertical-only "load cell" reading, take *.sphere.force.Y per region.
"""
import opensim as osim
import pandas as pd
import matplotlib.pyplot as plt

STO = "E:/OpenSim/output/fwd_forces_forces.sto"

tbl = osim.TimeSeriesTable(STO)
times = list(tbl.getIndependentColumn())
labels = list(tbl.getColumnLabels())

# Stuff every column into a DataFrame indexed by time.
data = {lab: tbl.getDependentColumn(lab).to_numpy() for lab in labels}
df = pd.DataFrame(data, index=times)
df.index.name = "time_s"

# Filter to just the vertical force on each region of the right foot.
# Column names in the .sto look like "contactHeel_r.Sphere.force.Y".
right_regions = [c for c in df.columns
                 if c.endswith(".Sphere.force.Y") and "_r." in c]
print("Right-foot vertical-force columns found:")
for c in right_regions:
    print(" ", c)

# Plot.
ax = df[right_regions].plot(figsize=(10, 4), linewidth=1.2)
ax.set_xlabel("time (s)")
ax.set_ylabel("vertical ground reaction force (N)")
ax.set_title("Right foot - per-region GRF (load-cell equivalent)")
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig("E:/OpenSim/output/per_region_GRF_right.png", dpi=120)
print("\nSaved plot to E:/OpenSim/output/per_region_GRF_right.png")

# Dump the right-foot regional forces as a tidy CSV too.
df[right_regions].to_csv("E:/OpenSim/output/per_region_GRF_right.csv")
print("Saved tidy CSV  to E:/OpenSim/output/per_region_GRF_right.csv")
