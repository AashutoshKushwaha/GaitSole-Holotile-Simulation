# Workspace layout

The git repository contains **source code, models, docs, and curated screenshots** only. Large generated data, screen recordings, and scratch files stay on disk under `local/` (gitignored).

## Committed (in git)

```
├── README.md, documentation.md, PROJECT_PROGRESSION.md
├── docs/
│   ├── assets/           # README result screenshots
│   ├── references/       # patent PDF, related papers
│   ├── PUBLISHING.md
│   └── WORKSPACE_LAYOUT.md
├── models/               # 2D + 3D .osim models
├── scripts/              # OpenSim pipeline + export_walk_replay_for_unity.py
├── motion_predictor/
├── holotile_sim/
├── gazebo_gait/
├── holotile_unity/       # Unity project (no Library/Temp)
└── output/.gitkeep       # OpenSim .sto outputs regenerated locally
```

## Local only (gitignored under `local/`)

| Folder | Contents |
|--------|----------|
| `local/recordings/` | Screen captures and demo `.mp4` files |
| `local/datasets/` | Large zip downloads (e.g. Camargo CSV) and `motion_predictor/data/` |
| `local/captures/` | Extra PNG/JPG frames not used in README |
| `local/logs/` | `opensim.log`, optimization stop files |
| `local/experiments/` | Early scratch Python / XML |
| `local/archive/` | Old `.sto`, duplicate `.osim`, output subfolders |
| `local/tools/` | Machine-specific helper scripts |

Also gitignored: `4.5/` (OpenSim install), `output/*.sto`, Unity `Library/`, conda envs, `.pt` checkpoints.

## Regenerating outputs

1. Run `scripts/07_walk_driven_per_region_grf.py` → `output/walk_motion.sto`, `walk_GRF.sto`
2. Run `scripts/export_walk_replay_for_unity.py` → `holotile_unity/Assets/StreamingAssets/OpenSim/walk_replay.json`
3. Unity menu **HoloTile → Export OpenSim Walk Replay** (optional re-export from editor)
