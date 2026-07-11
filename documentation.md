# OpenSim to HoloTile Documentation

## Overview
This repository documents the full development path of the project from biomechanical gait simulation to an interactive HoloTile-style omnidirectional floor simulation.

The work progressed through four major stages:

1. `OpenSim` musculoskeletal modeling and gait generation
2. `Python` tooling for data extraction, conversion, and HoloTile-oriented preprocessing
3. `Gazebo` and practicality testing for sensor-driven embodied simulation concepts
4. `Unity` implementation of the final HoloTile platform simulation with visual character replay, GRF-driven tile behavior, and physics-coupled foot interaction

The central idea across all stages is to use detailed lower-limb motion and plantar loading information to control an omnidirectional floor that keeps a person centered while preserving natural stepping behavior.

---

## 1. OpenSim Foundation

### 1.1 Objective
The first goal was to build a reliable gait-simulation pipeline that did more than report one force per foot. Instead, the foot was split into multiple plantar regions so the system could observe where force was being applied:

- heel
- midfoot
- forefoot
- toe

This regional loading was important because HoloTile control should eventually react not just to a whole-foot contact event, but to how the foot is rolling across the ground.

### 1.2 2D gait pipeline
The initial OpenSim work focused on a 2D gait model:

- inspected stock model structure and contact definitions
- created a 4-region foot contact model from the original 2D gait model
- ran forward simulations and predictive solves
- extracted per-region ground reaction forces from `.sto` outputs
- generated tiled stride outputs for longer motion playback

Important outputs from this stage included:

- `2D_gait_4regions.osim`
- `walk_GRF.sto`
- `run_solution.sto`
- `run_GRF.sto`
- multi-stride tiled motion/force outputs

### 1.3 3D gait pipeline
After validating the 2D workflow, the project expanded to a full 3D model:

- built `Rajagopal2016_4regions_fingers.osim`
- added four plantar contact regions per foot
- included articulated fingers/hands for a richer whole-body model
- ran a full predictive 3D Moco solve

Important outputs:

- `Rajagopal2016_4regions_fingers.osim`
- `_rajagopal_for_moco.osim`
- `run3d_solution.sto`
- `run3d_solve_log.txt`

### 1.4 Why OpenSim mattered
OpenSim established the biomechanical truth source for the entire project:

- joint kinematics
- pelvis trajectory
- gait timing
- per-region GRF

Later systems in Python, Gazebo, MuJoCo, and Unity all depended on these OpenSim-generated quantities either directly or indirectly.

---

## 2. Python Scripts for HoloTile

### 2.1 Purpose of the Python layer
Python was used as the glue between simulation domains. It handled:

- OpenSim automation
- `.sto` parsing
- GRF extraction
- stride tiling
- predictor training/evaluation
- HoloTile data preparation
- Unity replay export

In practice, the Python scripts became the data-engineering backbone of the project.

### 2.2 OpenSim script pipeline
The numbered scripts implemented the OpenSim workflow in sequence:

- inspect models and contacts
- split foot contacts into 4 regions
- run predictive or prescribed simulations
- replay solutions
- export regional GRF signals
- tile partial cycles into longer usable sequences

This made the pipeline reproducible instead of manually driven through the OpenSim GUI.

### 2.3 Motion predictor work
Python also powered a lightweight motion-prediction system:

- built a compact MLP-based predictor
- trained on synthetic gait first
- moved to real treadmill data from Camargo et al. 2021
- predicted near-future pose, root motion, GRF, free moment, and COP
- achieved sub-millisecond to low-millisecond inference suitable for 100 Hz control loops

This stage proved that expensive optimal-control solutions could be approximated by a fast online model.

### 2.4 HoloTile-oriented exports
For the Unity phase, Python was used to transform OpenSim outputs into an easy runtime format.

Key script:

- `scripts/export_walk_replay_for_unity.py`

What it does:

- reads `output/walk_motion.sto`
- reads `output/walk_GRF.sto`
- flattens the data into JSON compatible with Unity `JsonUtility`
- writes `holotile_unity/Assets/StreamingAssets/OpenSim/walk_replay.json`

This export was the bridge from biomechanical simulation to the final interactive visual demo.

### 2.5 Why the Python stage was critical
Without the Python tooling, every later environment would have had to re-solve, re-interpret, or manually reconstruct the gait information. Instead, Python standardized the data flow:

`OpenSim -> processed motion/GRF -> predictor/replay -> HoloTile control simulation`

---

## 3. Gazebo: Sensor Inputs and Practicality Testing

### 3.1 Why Gazebo was explored
Gazebo was used as an intermediate practical experimentation step to think about:

- embodied character presentation
- sensor-oriented simulation ideas
- real-world deployment practicality
- what a clothed human representation might look like before the final Unity version

This was less about final visuals and more about feasibility and integration thinking.

### 3.2 What Gazebo contributed
Gazebo helped evaluate the practical side of the project:

- how a simulated agent might be observed by virtual sensors
- how motion and foot interactions could be interpreted in a robotics-style simulation environment
- how realistic embodiment compares to simpler debug geometry
- what is and is not practical for real-time closed-loop floor control

It also served as a reference point for later visual expectations. During the Unity phase, one of the explicit goals became to surpass the earlier Gazebo-style clothed capsule presentation with a better, more believable human replay.

### 3.3 Practical lessons from Gazebo
The Gazebo stage reinforced several conclusions:

- live in-loop biomechanical simulation is too expensive for responsive floor control on typical hardware
- export-and-replay is much more practical than full online musculoskeletal solving
- a visual human model is useful, but it must stay synchronized with foot interaction
- sensor realism and interaction timing matter more than just showing an animated model

These lessons directly shaped the final Unity architecture.

---

## 4. MuJoCo HoloTile Research Simulation

Before the final Unity implementation, a major HoloTile simulation phase was built in `holotile_sim/` using MuJoCo. This was the first serious floor-control sandbox.

### 4.1 What was built
The MuJoCo work implemented Disney HoloTile concepts:

- modular floor of tiles
- each tile containing multiple tilted rotating disks
- tile azimuth and spin control
- friction-based moving-surface approximation
- person/foot centering control
- live velocity plus model-predicted velocity fusion

### 4.2 Milestones
The MuJoCo path advanced through milestone-style stages:

- `M1`: physics proof that disk commands generate expected transport direction
- `M2`: production-style frictionless support floor plus disk visual overlay
- `M3`: predictor-driven 3D skeleton walking on the floor
- `M4`: centered walking controller with stance/swing behavior
- `M5`: fusion of live and predicted velocity for better turn handling

This stage established that the HoloTile idea was controllable and that predictive control improved turn behavior over delayed live sensing alone.

### 4.3 What carried forward to Unity
The final Unity build inherited key ideas from MuJoCo:

- centered-walker floor control
- hybrid belt-style interpretation instead of literal full disk contact everywhere
- stance/swing reasoning
- per-foot and later per-region interaction logic
- importance of tuning responsiveness vs stability

---

## 5. Final Unity Simulation of the Platform

### 5.1 Why Unity became the final platform
Unity was chosen for the final simulation because it is better suited than the earlier environments for:

- polished real-time visualization
- easier scene setup
- character import workflows
- interactive playback and tuning
- eventual demonstration and presentation

The Unity work aimed to keep the engineering logic from earlier stages while making the system usable as a clear, visible simulation.

### 5.2 Early Unity phases
The Unity implementation was built incrementally.

#### Mechanism demo
The earliest Unity floor work validated platform geometry and actuation:

- tiled floor construction
- disk assemblies
- hemisphere/socket visual style based on patent figures
- scripted azimuth/spin behavior

#### Belt physics and walker demo
The next step created a controllable walker-floor interaction layer:

- frictionless tile support
- analytic belt-drive surface model
- foot puck interaction
- two-foot walker stance/swing logic
- centered-floor controller
- tuning controls for walk speed, spin max, and controller gain

These phases proved that the floor could keep a walker near the center before adding a human replay body.

### 5.3 Human replay phase
The major final step was `HumanWalkerDemo`, which connected OpenSim replay data to the Unity HoloTile floor.

Core components added:

- `OpenSimReplayData.cs`
- `OpenSimWalkFK.cs`
- `RegionForceMapper.cs`
- `RegionalTileCommands.cs`
- `RegionalFoot.cs`
- `RegionalBeltDrive.cs`
- `ImportedCharacterDriver.cs`
- `HumanWalkerDemo.cs`

### 5.4 What the Unity replay now does
The final Unity stack supports:

- loading OpenSim replay data from `walk_replay.json`
- reconstructing body pose from OpenSim kinematics
- mapping 4-region GRF to local tile commands
- replaying a clothed character on the HoloTile floor
- keeping the walker centered on the platform
- physics-coupled or kinematic foot modes
- tiled disk actuation under the active foot regions

### 5.5 Character and animation integration
Substantial work was done to make the human representation believable and synchronized:

- added a procedural clothed mannequin fallback
- supported custom FBX character import
- auto-scaled imported character to OpenSim body height
- fixed floor alignment and sole clearance
- disabled conflicting physics on imported FBX hierarchies
- replaced invalid Humanoid sampling with PlayableGraph-based clip scrubbing
- added ping-pong replay handling for the OpenSim half-cycle
- aligned visible feet and debug/physics feet more closely

One important discovery was that the OpenSim walk export represented a half gait cycle, so naïve looping created visible discontinuities. This was addressed through ping-pong style playback and smoother cycle handling.

### 5.6 Regional foot interaction and floor control
Another major focus was making the floor react to how the foot is loaded:

- separate regional foot patches for heel, midfoot, forefoot, and toe
- GRF-driven tile commands
- stance-vs-swing thresholds
- spring-guided replay feet for smoother physics-coupled behavior
- hidden foot bodies used for control and debug alignment

This moved the simulation closer to the intended HoloTile concept rather than a simple animation-over-floor visualization.

### 5.7 Camera, usability, and editor support
To make the Unity workflow practical:

- camera defaults were standardized
- an editor menu was added to regenerate replay JSON
- FBX validation tooling was added
- runtime tuning options were exposed in the Inspector

This made the demo easier to run repeatedly while iterating on gait, alignment, and tile-control behavior.

---

## 6. Key Files and Outputs

### OpenSim / output
- `output/walk_motion.sto`
- `output/walk_GRF.sto`
- `output/run_solution.sto`
- `output/run_GRF.sto`
- `output/run3d_solution.sto`
- `2D_gait_4regions.osim`
- `Rajagopal2016_4regions_fingers.osim`

### Python
- `scripts/export_walk_replay_for_unity.py`
- numbered OpenSim processing scripts
- `motion_predictor/` training, inference, and evaluation code

### MuJoCo / HoloTile research
- `holotile_sim/`
- floor controller
- fusion logic
- predictor bridge
- milestone demo outputs

### Unity
- `holotile_unity/Assets/HoloTile/Scripts/...`
- `holotile_unity/Assets/StreamingAssets/OpenSim/walk_replay.json`
- `HumanWalkerDemo`
- optional imported FBX character support

---

## 7. Current State

As of the latest work:

- the OpenSim pipeline is established for both 2D and 3D models
- Python tooling can export replay-ready motion and GRF data
- the MuJoCo HoloTile concept has already demonstrated centered locomotion and velocity fusion
- Unity now contains the main presentation/demo simulation of the HoloTile platform
- the simulation includes a visual character, regional foot loading logic, GRF-driven tile behavior, and iterative work toward better synchronization between visible legs and floor interaction

The project has therefore evolved from a pure biomechanics pipeline into a full interactive omnidirectional-floor simulation stack.

---

## 8. Recommended Next Steps

The natural next steps are:

- finalize perfect synchronization between visible feet, hidden control feet, and tile actuation
- extend the Unity replay path from `walk_GRF` to `run3d_solution`
- bring more of the predictor/fusion logic from MuJoCo into Unity when the visual/physics stack is stable
- improve final visuals, materials, and character polish once motion correctness is locked
- optionally reconnect real sensor pipelines after the simulation behavior is fully stable

---

## 9. Summary

From the beginning to now, the project has followed a clear technical arc:

- `OpenSim` generated biomechanically meaningful motion and per-region foot loading
- `Python` turned those results into reusable data products and control-ready assets
- `Gazebo` helped evaluate practicality, embodiment, and sensor-oriented thinking
- `MuJoCo` proved the HoloTile control concept and predictive fusion strategy
- `Unity` became the final integrated simulation platform combining floor visuals, character replay, and regional foot-driven platform behavior

This makes the repository not just a single simulation, but a complete research-and-engineering pipeline for studying gait-aware omnidirectional floor control.

---

## 10. GitHub Repository and Documentation

### 10.1 Public repository

The consolidated project is published at:

**https://github.com/AashutoshKushwaha/GaitSole-Holotile-Simulation**

### 10.2 Documentation map

| Document | Contents |
|----------|----------|
| `README.md` | Overview, results gallery, file tree, quick start for all subsystems |
| `documentation.md` | This file — full pipeline narrative |
| `docs/PUBLISHING.md` | Git init, commit, push; owner-only commits |
| `docs/WORKSPACE_LAYOUT.md` | What is committed vs kept in gitignored `local/` |
| `holotile_unity/README.md` | Unity 6 setup, HumanWalkerDemo, FBX import |
| `gazebo_gait/README.md` | Gazebo Harmonic + ROS 2 Jazzy pipeline |
| `motion_predictor/README.md` | Predictor architecture and training |

Result screenshots for the README live in `docs/assets/` (copies from `holotile_sim/output/`, `gazebo_gait/output/`, OpenSim plots, and Unity captures).

### 10.3 Folder naming

The OpenSim script directory is **`scripts/`** (not the older name `simulation_scripts/`). Generated `.osim` models live in **`models/`**. Screen recordings and large downloads live in **`local/`** (gitignored). The Unity project is **`holotile_unity/`**; MuJoCo reference code is **`holotile_sim/`**; embodied testing is **`gazebo_gait/`**.

### 10.4 Publishing policy

All git commits and pushes to GitHub are performed **by the repository owner**. AI coding assistants are not listed as contributors: no `Co-authored-by` trailers for Cursor, Claude, or similar tools. See `docs/PUBLISHING.md` for step-by-step instructions.
