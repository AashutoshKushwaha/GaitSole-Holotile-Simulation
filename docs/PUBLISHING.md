# Publishing to GitHub

This document describes how to push this workspace to
[https://github.com/AashutoshKushwaha/GaitSole-Holotile-Simulation](https://github.com/AashutoshKushwaha/GaitSole-Holotile-Simulation).

**You (the repository owner) perform all commits and pushes.** Documentation in this repo is written so that no AI assistant is listed as a contributor.

---

## One-time setup

From PowerShell (adjust paths if your clone lives elsewhere):

```powershell
cd E:\OpenSim

# If this folder is not yet a git repo:
git init
git remote add origin https://github.com/AashutoshKushwaha/GaitSole-Holotile-Simulation.git

# If the remote already exists on GitHub with an initial commit, pull first:
# git pull origin main --allow-unrelated-histories
# (or use `master` if that is the default branch on GitHub)
```

Review `.gitignore` before the first commit. Large generated files (`.sto`, `.mp4`, Unity `Library/`, OpenSim `4.5/` install) are excluded by design.

---

## What to commit

| Include | Exclude (regenerate locally) |
|---------|----------------------------|
| `scripts/`, `motion_predictor/`, `holotile_sim/`, `gazebo_gait/` | `output/*.sto` (OpenSim results) |
| `holotile_unity/Assets/`, `Packages/`, `ProjectSettings/` | `holotile_unity/Library/`, `Temp/` |
| `documentation.md`, `README.md`, `docs/` | `*.mp4` screen recordings |
| `docs/assets/*.png` (README screenshots) | Full OpenSim 4.5 install under `4.5/` |
| Generated `.osim` at repo root if present | Conda envs, `.pt` checkpoints |

**Mixamo FBX:** `holotile_unity/Assets/HoloTile/Characters/Walking.fbx` is ~55 MB. GitHub accepts files up to 100 MB. Commit it if you want the demo to work out of the box; otherwise document that users import their own FBX and add `*.fbx` to `.gitignore`.

---

## Commit and push (owner only — no Co-authored-by)

**Do not use Cursor Agent to run `git commit`.** In this environment, agent-initiated commits can receive an automatic `Co-authored-by: Cursor` trailer. Run git **only in your own terminal** (Windows Terminal, PowerShell outside chat):

```powershell
cd E:\OpenSim
git status
git commit -m "Organize HoloTile repo: scripts, models, Unity, Gazebo, MuJoCo, docs"
git log -1 --format=full
```

Confirm the output shows **only your name and email** — there must be **no** `Co-authored-by` line.

Then push:

```powershell
git branch -M main
git push -u origin main
```

If the remote already has commits:

```powershell
git pull origin main --allow-unrelated-histories
git push -u origin main
```

If GitHub already has commits on `main`, you may need `git pull --rebase origin main` before pushing.

---

## Keeping AI tools off the contributor graph

GitHub attributes commits to the **author** and **committer** in `git commit`. To avoid AI appearing as a contributor:

1. Run `git commit` yourself in your terminal (not via an agent that sets a bot identity).
2. Do not use `--author` flags pointing at AI services.
3. Do not add trailers such as `Co-authored-by: Claude ...` or `Co-authored-by: Cursor ...`.
4. If you use `git config user.name` / `user.email`, ensure they are **your** credentials before committing.

Optional check before push:

```powershell
git log -1 --format=full
```

The `Author` and `Commit` lines should show only you.

---

## Updating screenshots in the README

Result images for the README live in `docs/assets/`. After new Unity or simulation runs:

1. Export PNG frames from MuJoCo (`holotile_sim/output/`), OpenSim plots (`output/`), Gazebo (`gazebo_gait/output/`), or Unity captures.
2. Copy into `docs/assets/` with descriptive names.
3. Reference them in root `README.md` as `docs/assets/your_image.png`.
4. Commit and push using the steps above.

---

## Related docs

- [README.md](../README.md) — project overview and quick start
- [documentation.md](../documentation.md) — full pipeline narrative
- [holotile_unity/README.md](../holotile_unity/README.md) — Unity M6 human walker setup
