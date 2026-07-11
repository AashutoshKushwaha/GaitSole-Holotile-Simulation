import opensim as osim
tbl = osim.TimeSeriesTable("E:/OpenSim/output/run_GRF.sto")
times = list(tbl.getIndependentColumn())
def col(n): return tbl.getDependentColumn(n).to_numpy()

# show 1 row every ~20 timesteps so the table is short
step = max(1, len(times) // 20)
print(f"time   HeelR  MidR  ForeR  ToeR  |  HeelL  MidL  ForeL  ToeL")
for i in range(0, len(times), step):
    t = times[i]
    vals = [col(f"contact{r}_{s}.Fy")[i]
            for s in ("r","l") for r in ("Heel","Midfoot","Forefoot","Toe")]
    print(f"{t:5.3f}  " + "  ".join(f"{v:5.0f}" for v in vals[:4])
          + "  |  " + "  ".join(f"{v:5.0f}" for v in vals[4:]))
print(f"\nrows: {len(times)}, time span: {times[0]:.3f} -> {times[-1]:.3f} s")
