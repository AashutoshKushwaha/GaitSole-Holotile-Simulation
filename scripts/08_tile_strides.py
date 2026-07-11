"""
Step 8: Tile the periodic running cycle into N strides.

Usage:
    python 08_tile_strides.py            (default N=5)
    python 08_tile_strides.py 10

Reads:   E:/OpenSim/output/run_solution.sto   (Moco half-stride)
         E:/OpenSim/output/run_GRF.sto        (per-region forces, full stride)
Writes:  E:/OpenSim/output/run_solution_Nstrides.sto
         E:/OpenSim/output/run_GRF_Nstrides.sto

For the solution we first mirror the half-stride into a full stride
(createPeriodicTrajectory), then tile. For the GRF file we tile directly
because script 06 already wrote a full stride.
"""
import sys
import opensim as osim
import numpy as np

N = int(sys.argv[1]) if len(sys.argv) > 1 else 5
print(f"Tiling {N} strides.\n")


# --- 1. Solution: half-stride -> full stride -> tile -------------------
traj = osim.MocoTrajectory("E:/OpenSim/output/run_solution.sto")
traj = osim.createPeriodicTrajectory(traj)
times = np.array(list(traj.getTimeMat()))
state_names = list(traj.getStateNames())
state_mat = traj.getStatesTrajectoryMat()
ctrl_names = list(traj.getControlNames())
ctrl_mat   = traj.getControlsTrajectoryMat()

stride_T = times[-1] - times[0]
ptx_col = state_names.index("/jointset/groundPelvis/pelvis_tx/value")
stride_dx = state_mat[-1, ptx_col] - state_mat[0, ptx_col]
print(f"Solution single full stride: duration {stride_T:.3f} s, advance {stride_dx:+0.3f} m")

# Drop last row when concatenating so adjacent strides don't share a timestamp.
inner_t   = times[:-1]
inner_s   = state_mat[:-1, :]
inner_c   = ctrl_mat[:-1, :]
new_times, new_states, new_ctrls = [], [], []
for k in range(N):
    t_shift = k * stride_T
    if k < N - 1:
        new_times.append(inner_t + t_shift)
        new_states.append(inner_s.copy())
        new_ctrls.append(inner_c.copy())
    else:
        new_times.append(times + t_shift)
        new_states.append(state_mat.copy())
        new_ctrls.append(ctrl_mat.copy())
    # Shift pelvis_tx for this stride
    new_states[-1][:, ptx_col] += k * stride_dx

T_all = np.concatenate(new_times)
S_all = np.vstack(new_states)
C_all = np.vstack(new_ctrls)
print(f"  -> {len(T_all)} rows, total {T_all[-1]:.3f} s")

cols = ["time"] + state_names + ctrl_names
out_path = "E:/OpenSim/output/run_solution_Nstrides.sto"
with open(out_path, "w") as fh:
    fh.write("run_solution_Nstrides\nversion=1\n")
    fh.write(f"nRows={len(T_all)}\nnColumns={len(cols)}\n")
    fh.write("inDegrees=no\nendheader\n")
    fh.write("\t".join(cols) + "\n")
    for i in range(len(T_all)):
        row = [T_all[i]] + list(S_all[i, :]) + list(C_all[i, :])
        fh.write("\t".join(f"{v:0.6f}" for v in row) + "\n")
print(f"Wrote {out_path}")


# --- 2. GRF: already a full stride, just tile --------------------------
def read_sto_table(path):
    with open(path) as fh:
        lines = fh.readlines()
    for i, l in enumerate(lines):
        if l.strip() == "endheader":
            hdr_end = i
            break
    cols = lines[hdr_end + 1].rstrip("\r\n").split("\t")
    data = np.array(
        [[float(x) for x in l.split()] for l in lines[hdr_end + 2:] if l.strip()]
    )
    return cols, data


cols2, data2 = read_sto_table("E:/OpenSim/output/run_GRF.sto")
t_idx2 = cols2.index("time")
grf_T = data2[-1, t_idx2] - data2[0, t_idx2]
print(f"\nGRF single full stride: duration {grf_T:.3f} s")

inner = data2[:-1, :]
parts = []
for k in range(N):
    if k < N - 1:
        seg = inner.copy()
    else:
        seg = data2.copy()
    seg[:, t_idx2] += k * grf_T
    parts.append(seg)
out2 = np.vstack(parts)
print(f"  -> {out2.shape[0]} rows, total {out2[-1, t_idx2]:.3f} s")

with open("E:/OpenSim/output/run_GRF_Nstrides.sto", "w") as fh:
    fh.write("run_GRF_Nstrides\nversion=1\n")
    fh.write(f"nRows={out2.shape[0]}\nnColumns={out2.shape[1]}\n")
    fh.write("inDegrees=no\nendheader\n")
    fh.write("\t".join(cols2) + "\n")
    for row in out2:
        fh.write("\t".join(f"{v:0.6f}" for v in row) + "\n")
print("Wrote E:/OpenSim/output/run_GRF_Nstrides.sto")
