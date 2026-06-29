# Docker Guide — Running ROSMOSIS Experiments on a Server

This guide covers running ROSMOSIS as a **self-assembling Docker image** on a remote
server (the reference machine is `nrg-alpha`, an Exxact Tensor TS4, Ubuntu 22.04,
RTX A6000). It is the operational companion to [README.md](README.md): the README
documents the workspace, launch args, and mission tuning; this guide documents
**SSH access, the image build/run lifecycle, data persistence, parallel experiments,
visualization, and offline analysis** — everything specific to running headless on a
shared server rather than on your laptop.

> **Edit locally, push, rebuild on the server.** The Dockerfile re-clones every repo
> from GitHub on each build (see [The self-assembling image](#the-self-assembling-image)),
> so the server only ever sees *pushed* commits. Never edit source directly on the
> server expecting it to survive a rebuild — it won't. The canonical loop is:
> **edit on laptop → `git push` → `docker build` on server**.

---

## 1. Connecting to the server

### SSH from a terminal

```bash
ssh <user>@<server>      # e.g. ssh you@nrg-alpha.me.utexas.edu  (or the IP)
```

### SSH key for GitHub clones (needed for the build)

The image clones private repos over SSH using a **forwarded agent key** (BuildKit
`--mount=type=ssh`). The key never gets baked into a layer. Before building, load your
GitHub key into the agent **on the server**:

```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519        # the key registered with your GitHub account
ssh-add -l                       # verify it's loaded
```

If `git@github.com` is unreachable, test with `ssh -T git@github.com` — it should
greet you by GitHub username.

### VS Code Remote-SSH (recommended for browsing results + analysis)

1. Install the **Remote - SSH** extension locally.
2. Command Palette → *Remote-SSH: Connect to Host* → enter `ssh <user>@<server>`.
3. VS Code installs a small server-side component automatically and opens a window
   rooted on the server.

Use the VS Code window for: browsing `$HOME/ROSMOSIS/data/`, running the analysis
notebook with inline matplotlib (see [§7](#7-analyzing-results)), and opening a
server-side terminal. **Still do all source edits + pushes from your laptop** —
direct server edits get wiped on the next `docker build`.

> **No drag-and-drop to local.** VS Code Remote-SSH cannot drag files to your
> machine. To pull a file down: right-click it in the Explorer → **Download**, or
> use `scp` from a *local* terminal (see [§7](#7-analyzing-results)).

---

## 2. The self-assembling image

`Dockerfile` is a bootstrap: you clone ROSMOSIS once to get the Dockerfile, and every
`docker build` re-clones the outer repo **and** the four nested dependency repos fresh
from GitHub at their pinned branches. Nothing is `COPY`'d from the host working tree.

| Repo | Branch | Why |
|---|---|---|
| ROSMOSIS | `experiment-docker` | the workspace (`src/`, vendored pydubins, entrypoint) |
| nbv_cpp | `feature/alpha-server` | TSDF/NBV server; `CMAKE_CUDA_ARCHITECTURES "75 86 89"` (adds 86 = A6000) |
| perception_open3d | `ros2` | provides `open3d_conversions` for nbv_cpp |
| sample_nbv_behaviors | `rosmosis` | NBV behavior-tree nodes |
| nrg_behaviors | `main` | utility BT nodes |

Base image: `nvidia/cuda:12.4.1-devel-ubuntu22.04` (ships `nvcc`, so the GPU kernels
compile). ROS 2 Humble + CycloneDDS are installed on top.

### One-time: get the bootstrap clone on the server

```bash
cd ~                       # or wherever you keep it; the docs assume ~/ROSMOSIS
git clone git@github.com:UTNuclearRobotics/ROSMOSIS.git
cd ROSMOSIS
git checkout experiment-docker
```

Re-run `git pull` here **only when the Dockerfile itself changes** — `src/` and param
changes are picked up by the in-build re-clone, not by this outer clone.

---

## 3. Building the image

```bash
cd ~/ROSMOSIS
eval "$(ssh-agent -s)" && ssh-add ~/.ssh/id_ed25519     # if not already loaded
docker build --ssh default --build-arg CACHEBUST=$(date +%s) -t rosmosis:test0 .
```

| Flag | Why |
|---|---|
| `--ssh default` | forwards your SSH agent into the build for the private-repo clones |
| `--build-arg CACHEBUST=$(date +%s)` | **forces a fresh re-clone.** `$(date)` is unique each run, so the clone layer is never cached. **Omit this and Docker reuses the stale clone → your latest pushes are ignored.** |
| `-t rosmosis:test0` | image tag; reuse it in `docker run` |

**The numpy pin (why the build constrains `numpy<1.24`).** open3d drags in a heavy
pip ML stack that, unconstrained, upgrades numpy past what the rest of the workspace
tolerates: the apt `transforms3d 0.3.1` uses `np.float` at import (removed in NumPy
1.24), so every `tf_transformations` import would die with *"module 'numpy' has no
attribute 'float'"*. The Dockerfile writes `/etc/pip-constraints.txt` with
`numpy<1.24` + `matplotlib<3.6` and points `PIP_CONSTRAINT` at it, so the cap applies
to **every** pip install transitively. (Do not relax this to `numpy<2` — 1.24–1.26
still break the import.)

---

## 4. Running an experiment

### Headless NBV mission with data persistence (the standard run)

```bash
docker run --rm --gpus all \
  -v "$HOME/ROSMOSIS/data:/workspace/data" \
  --user $(id -u):$(id -g) \
  rosmosis:test0 \
  ros2 launch demo_behaviors demo_mission_launch.py \
      start_rviz:=false debug_gui:=false record:=true \
      alpha:=0.25 bag_prefix:=nbv_cone_alpha0.25
```

### Boustrophedon baseline

The same image runs the baseline — it's just a different launch file (no source
change needed):

```bash
docker run --rm --gpus all \
  -v "$HOME/ROSMOSIS/data:/workspace/data" \
  --user $(id -u):$(id -g) \
  rosmosis:test0 \
  ros2 launch baseline_mission baseline_mission_launch.py \
      start_rviz:=false debug_gui:=false record:=true \
      environment:=env_50x50_cluster_seabed bag_prefix:=boustrophedon_cluster
```

### `docker run` flags that matter here

| Flag | Why |
|---|---|
| `--rm` | removes the container **and its writable layer** on exit. Anything written *inside* the container (no mount) is gone forever — hence the `-v` bind mount below. |
| `--gpus all` | exposes the GPU for nbv_cpp's CUDA TSDF/ray-cast kernels. |
| `-v "$HOME/ROSMOSIS/data:/workspace/data"` | **bind mount — the only way data survives `--rm`.** Maps the server's `data/` onto the container's `/workspace/data`, where the mission writes bags + reconstructions. Use an **absolute** host path (`$HOME` expands to one). |
| `--user $(id -u):$(id -g)` | run as *you*, not root, so written files are owned by your user. **Without this, output is root-owned and you can't delete/edit it** (see [§6](#6-file-ownership-the-root-trap)). |
| launch args | standard ROSMOSIS args — see the [README launch table](README.md#key-launch-parameters). `start_rviz:=false debug_gui:=false` ⇒ headless (no display needed). |

### Launch-arg notes for server runs

- **`start_rviz:=false`** and **`debug_gui:=false`** are required for headless / batch
  / parallel runs — both want a display (see [§5](#5-visualization-rviz)).
- **`record:=true`** writes the MCAP bag. The container has
  `ros-humble-rosbag2-storage-mcap` installed.
- **`bag_prefix`** must be **unique per run** — a timestamp is appended, but distinct
  prefixes keep runs (and parallel experiments) from colliding in the data dir.
- **`alpha`** (NBV only) is the CI-NBV cost weight — the experiment-sweep knob; no XML
  edit needed.

---

## 5. Visualization (RViz)

**Live RViz over SSH does not work on this server**, and `start_rviz:=true` will crash
the run. Findings:

- `ssh -Y` (trusted X forwarding) sets up `DISPLAY` and the Xauth cookie correctly, but
  modern X servers disable **indirect GLX**, which is what a forwarded OpenGL app needs.
  RViz's OGRE backend fails with *"Failed to create an OpenGL context / BadValue /
  Unable to create a suitable GLXContext"*.
- `LIBGL_ALWAYS_SOFTWARE=1` alone is insufficient — the CUDA devel base image has no
  Mesa software-GL driver (`libgl1-mesa-dri`) to fall back to.

**Verdict: run headless** (`start_rviz:=false`) and visualize **offline** — copy the
output PLY meshes down and inspect locally (see [§7](#7-analyzing-results)). Live
monitoring of the behavior tree is still possible via Groot2 on TCP 1667 if you
forward that port, independent of X11 (see the README's *Live Monitoring* section).

---

## 6. File ownership (the root trap)

A container without `--user` runs as **root**, so files it writes to the bind-mounted
`data/` are owned by `root:root`. As a non-root user you can *read* them but cannot
delete or modify them — VS Code shows `EACCES: permission denied, unlink ...` and the
shell `rm` fails.

**Fix going forward:** always pass `--user $(id -u):$(id -g)` (in the run commands
above). New runs then write files owned by you.

**Fixing already-root-owned files:** `chown` needs root. If you have sudo:

```bash
sudo chown -R $(id -u):$(id -g) ~/ROSMOSIS/data/
```

On a locked-down shared server your user may **not be in sudoers** (this is the case on
`nrg-alpha`), so you cannot `chown` existing root-owned files yourself — ask the admin
to run the `chown` once, or just leave the old root-owned runs in place (they're still
readable / copyable) and rely on `--user` for all new runs.

---

## 7. Analyzing results

### Where the data lands

After a run, on the server (paths relative to the `-v` host dir, `~/ROSMOSIS/data`):

```
data/
├── bags/<bag_prefix>_<YYYYMMDD_HHMMSS>/
│   ├── metadata.yaml
│   └── <bag_prefix>_<...>.mcap
└── reconstructions/<bag_prefix>_<YYYYMMDD_HHMMSS>/
    └── target_<N>.ply
```

The bag and the reconstruction dir share the same timestamped name, so a run's bag and
meshes are co-located. Runs **accumulate** — nothing is overwritten.

### Analysis environment (server-side venv)

`scripts/analyze_run.py` uses **`rosbags`** (pure Python — no ROS runtime needed) and
reads the custom `uuv_interfaces/msg/FaceHits` definition straight from the repo's
`.msg` text file, so the repo clone on the server is all it needs. Set up a venv once:

```bash
cd ~/ROSMOSIS
python3 -m venv venv          # 'venv' is gitignored; keep ONE, don't make duplicates
source venv/bin/activate
pip install rosbags pandas matplotlib numpy scipy
```

> **`rosbags` version pin:** `analyze_run.py` imports `from rosbags.dataframe import
> get_dataframe`. If your installed `rosbags` errors with *"No module named
> rosbags.dataframe"*, pin to a version that ships it (e.g. `pip install
> "rosbags==0.10.4"`) and **restart the Jupyter kernel** — swapping the pip package
> does not reload a live kernel.

Then open `scripts/analyze_run.py` in the VS Code Remote-SSH window. It is a
**percent-cell (`# %%`) script**: with the Python + Jupyter extensions installed,
select `~/ROSMOSIS/venv` as the interpreter and run cells in the Interactive window —
matplotlib plots render inline, no X11 needed. Set `BAG_NAME` (the run folder) and `M`
(total targets in the scene — the CIR denominator; keep it at the scene's `box_count`
even if a run inspected fewer) at the top before running.

> First kernel connect to a remote venv is slow (~30–60 s cold start over SSH while it
> imports pandas/numpy/scipy/rosbags). Let it finish without clicking — closing/reopening
> mid-connect restarts the clock. Subsequent cell runs are fast.

### Pulling files to your laptop

Run these from a **local** terminal — *not* the SSH session. `scp`/`rsync` open their
own connection to the server; "the destination" is wherever you run them. (Run them
inside the SSH session and both ends resolve to the server, copying it next to itself.)

```bash
# the entire data folder (bags + reconstructions + plots) in one go
rsync -avz <user>@<server>:~/ROSMOSIS/data/ ./data/

# or with scp -- note the /* and trailing slash so it merges INTO ./data
scp -r <user>@<server>:~/ROSMOSIS/data/* ./data/

# just one run's reconstructions (much lighter -- skips the big .mcap bags)
scp -r <user>@<server>:~/ROSMOSIS/data/reconstructions/<run> .
```

**Pull everything, not just one subfolder** — grab all of `data/` so bags *and*
reconstructions (and plots) come down together; don't cherry-pick one and lose the
matching run data.

**It merges, it doesn't wipe.** Neither tool deletes your local folder: both copy new
files in and overwrite same-named ones, leaving unrelated local files alone. Because
runs are timestamped (`nbv_cone_alpha0.25_<stamp>`), names don't collide between runs,
so each pull just adds the new run folders.

- Prefer **`rsync`** for repeated pulls — it transfers only what changed and resumes
  cleanly, so re-running it as runs accumulate is cheap. The trailing slashes
  (`data/` → `data/`) mean "sync the *contents* into here."
- With **`scp -r`**, watch the trailing path: `scp -r remote:.../data ./data` nests as
  `./data/data/...` if `./data` already exists. Use `.../data/*` → `./data/` (above)
  to merge into an existing local `data/`.
- **Bags are big.** Pulling all of `data/` includes every `.mcap`; if you only need
  meshes for offline viewing, the `reconstructions/` subfolder is far lighter.

---

## 8. Running experiments in parallel

Each experiment is **one container with its own launch args** — a clean isolation
model. To run several at once on the shared host, give each its own ROS graph and
output namespace:

```bash
# Experiment A
docker run --rm --gpus all \
  -e ROS_DOMAIN_ID=1 \
  -v "$HOME/ROSMOSIS/data:/workspace/data" \
  --user $(id -u):$(id -g) \
  rosmosis:test0 \
  ros2 launch demo_behaviors demo_mission_launch.py \
      start_rviz:=false debug_gui:=false record:=true \
      alpha:=0.25 bag_prefix:=nbv_alpha0.25 &

# Experiment B (different domain ID + different bag prefix)
docker run --rm --gpus all \
  -e ROS_DOMAIN_ID=2 \
  -v "$HOME/ROSMOSIS/data:/workspace/data" \
  --user $(id -u):$(id -g) \
  rosmosis:test0 \
  ros2 launch demo_behaviors demo_mission_launch.py \
      start_rviz:=false debug_gui:=false record:=true \
      alpha:=0.75 bag_prefix:=nbv_alpha0.75 &
```

Rules for parallel runs:

- **Distinct `ROS_DOMAIN_ID` per container** (`-e ROS_DOMAIN_ID=N`) so their DDS graphs
  don't cross-talk. Containers are isolated network-wise by default (don't use
  `--net=host` for parallel runs — that shares the host network and re-introduces
  cross-talk; `--net=host` was only ever needed for X forwarding, which we've abandoned).
- **Distinct `bag_prefix` per container** so outputs don't collide in the shared
  `data/` mount.
- **GPU:** `--gpus all` lets every container see the GPU; they time-share it. For a
  cleaner split you can pin one GPU per container with `--gpus '"device=0"'` etc. (the
  A6000 box has the GPUs to spread across).

A shell script that loops over an `alpha` (or environment) list, assigning each run an
incrementing domain ID and a matching `bag_prefix`, is the natural way to launch a
sweep.

---

## 9. Quick reference

```bash
# --- on the server, one time ---
git clone git@github.com:UTNuclearRobotics/ROSMOSIS.git ~/ROSMOSIS
cd ~/ROSMOSIS && git checkout experiment-docker

# --- build (re-run after every push you want picked up) ---
eval "$(ssh-agent -s)" && ssh-add ~/.ssh/id_ed25519
docker build --ssh default --build-arg CACHEBUST=$(date +%s) -t rosmosis:test0 .

# --- run (headless, persisted, owned-by-you) ---
docker run --rm --gpus all \
  -v "$HOME/ROSMOSIS/data:/workspace/data" \
  --user $(id -u):$(id -g) \
  rosmosis:test0 \
  ros2 launch demo_behaviors demo_mission_launch.py \
      start_rviz:=false debug_gui:=false record:=true \
      alpha:=0.25 bag_prefix:=nbv_cone_alpha0.25

# --- analyze (server venv + VS Code Interactive window) ---
source ~/ROSMOSIS/venv/bin/activate     # set BAG_NAME in analyze_run.py, run cells

# --- pull ALL results down (from a LOCAL terminal, merges into ./data) ---
rsync -avz <user>@<server>:~/ROSMOSIS/data/ ./data/
```
