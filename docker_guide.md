# Docker Guide — Running ROSMOSIS Experiments on a Server

This guide covers running ROSMOSIS as a **self-assembling Docker image** on a remote
server (the reference machine is `nrg-alpha`, an Exxact Tensor TS4, Ubuntu 22.04,
RTX A6000). It is the operational companion to [README.md](README.md): the README
documents the workspace, launch args, and mission tuning; this guide documents
**SSH access, the image build/run lifecycle, data persistence, parallel experiments,
visualization, and offline analysis**: everything specific to running headless on a
shared server rather than on your laptop.

> **Edit locally, push, rebuild on the server.** The Dockerfile re-clones every repo
> from GitHub on each build (see [The self-assembling image](#the-self-assembling-image)),
> so the server only ever sees *pushed* commits. Never edit source directly on the
> server expecting it to survive a rebuild; it won't. The canonical loop is:
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

If `git@github.com` is unreachable, test with `ssh -T git@github.com`; it should
greet you by GitHub username.

### VS Code Remote-SSH (recommended for browsing results + analysis)

1. Install the **Remote - SSH** extension locally.
2. Command Palette → *Remote-SSH: Connect to Host* → enter `ssh <user>@<server>`.
3. VS Code installs a small server-side component automatically and opens a window
   rooted on the server.

Use the VS Code window for: browsing `$HOME/ROSMOSIS/data/`, running the analysis
notebook with inline matplotlib (see [§5](#5-analyzing-results)), and opening a
server-side terminal. **Still do all source edits + pushes from your laptop**:
direct server edits get wiped on the next `docker build`.

> **No drag-and-drop to local.** VS Code Remote-SSH cannot drag files to your
> machine. To pull a file down: right-click it in the Explorer → **Download**, or
> use `scp` from a *local* terminal (see [§5](#5-analyzing-results)).

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

Re-run `git pull` here **only when the Dockerfile itself changes**; `src/` and param
changes are picked up by the in-build re-clone, not by this outer clone.

---

## 3. Building the image

The convenience wrapper `build_image.sh` does the agent-load + `CACHEBUST` build for
you — just give it a tag (use the sampler name so it pairs with `SAMPLER`):

```bash
cd ~/ROSMOSIS
TAG=cone ./build_image.sh        # -> rosmosis:cone   (agent-load + fresh re-clone)
```

Or the raw command it runs:

```bash
eval "$(ssh-agent -s)" && ssh-add ~/.ssh/id_ed25519     # if not already loaded
docker build --ssh default --build-arg CACHEBUST=$(date +%s) -t rosmosis:<tag> .
```

| Flag | Why |
|---|---|
| `--ssh default` | forwards your SSH agent into the build for the private-repo clones |
| `--build-arg CACHEBUST=$(date +%s)` | **forces a fresh re-clone.** `$(date)` is unique each run, so the clone layer is never cached. **Omit this and Docker reuses the stale clone → your latest pushes are ignored.** |
| `-t rosmosis:<tag>` | image tag — **your choice** (`test0`, `cluster`, `v2`, ...). The examples below write `rosmosis:<tag>`; substitute the tag you built, and use the **same** tag in every `docker run`. The sweep scripts take it via `IMAGE=rosmosis:<tag> ./run_...sh`. |

**The numpy pin (why the build constrains `numpy<1.24`).** open3d drags in a heavy
pip ML stack that, unconstrained, upgrades numpy past what the rest of the workspace
tolerates: the apt `transforms3d 0.3.1` uses `np.float` at import (removed in NumPy
1.24), so every `tf_transformations` import would die with *"module 'numpy' has no
attribute 'float'"*. The Dockerfile writes `/etc/pip-constraints.txt` with
`numpy<1.24` + `matplotlib<3.6` and points `PIP_CONSTRAINT` at it, so the cap applies
to **every** pip install transitively. (Do not relax this to `numpy<2` — 1.24–1.26
still break the import.)

### Cone vs helix: one image per sampler

The viewpoint sampler is **baked into the behavior tree** (`nbv_on_target.xml`); there
is no launch arg for it. So each sampler is a **separate image**:

1. In `src/demo_behaviors/behavior_trees/nbv_on_target.xml`, activate the sampler you
   want (comment out the other block), **commit and push**.
2. Rebuild with a **sampler-matching tag**, e.g. `rosmosis:cone` or `rosmosis:helix`.

```bash
# after pushing the BT with the cone block active
TAG=cone ./build_image.sh
# ...switch the BT to helix, push, then:
TAG=helix ./build_image.sh
```

> The sweep script's `SAMPLER=cone|helix` is a **bag label only** — it does not change
> the sampler. You must pair it with the matching image (`IMAGE=rosmosis:cone
> SAMPLER=cone`). Mismatching them runs the wrong sampler under the wrong name and
> silently corrupts the comparison.

---

## 4. Running an experiment

### Headless NBV mission with data persistence (the standard run)

```bash
docker run --rm --gpus all \
  -v "$HOME/ROSMOSIS/data:/workspace/data" \
  rosmosis:<tag> \
  ros2 launch demo_behaviors demo_mission_launch.py \
      start_rviz:=false debug_gui:=false record:=true \
      environment:=env_1000x1000_cluster_seabed \
      alpha:=0.25 bag_prefix:=nbv_cone_alpha0.25_fullscale
```

### Boustrophedon baseline

The same image runs the baseline: it's just a different launch file (no source
change needed):

```bash
docker run --rm --gpus all \
  -v "$HOME/ROSMOSIS/data:/workspace/data" \
  rosmosis:<tag> \
  ros2 launch baseline_mission baseline_mission_launch.py \
      start_rviz:=false debug_gui:=false record:=true \
      environment:=env_1000x1000_cluster_seabed bag_prefix:=boustrophedon_fullscale
```

### `docker run` flags that matter here

| Flag | Why |
|---|---|
| `--rm` | removes the container after exit, so it doesn't pile up alongside every run. |
| `--gpus all` | exposes **all** GPUs to the container. Fine for a **single** run. nbv_cpp only ever uses the default device (device 0), so with `all` it lands on the first visible card — and if you launch several `--gpus all` containers at once, they **all pile onto that same card** and contend. |
| `--gpus '"device=N"'` | exposes **only** physical GPU `N`, renumbered to device 0 inside the container. This is how you **pin one mission to one card** — use it (not `all`) for concurrent runs so each mission gets a dedicated GPU. Note the nested quotes: bash strips the outer `'...'`, Docker needs the inner `"device=N"`. |
| `-v "$HOME/ROSMOSIS/data:/workspace/data"` | **bind mount**, so bags + reconstructions land on the host instead of only inside the container's writable layer. Maps the server's `data/` onto the container's `/workspace/data`. Use an **absolute** host path (`$HOME` expands to one). |
| launch args | standard ROSMOSIS args — see the [README launch table](README.md#key-launch-parameters). `start_rviz:=false debug_gui:=false` ⇒ headless (no display needed). |

### Launch-arg notes for server runs

- **`start_rviz:=false`** and **`debug_gui:=false`** are required for headless / batch
  / parallel runs. Live RViz over SSH doesn't work on this server (X forwarding fails
  on indirect GLX); visualize offline instead by pulling down the output PLY meshes
  (see [§5](#5-analyzing-results)). Live BT monitoring is still possible via Groot2 on
  TCP 1667, independent of X11 (see the README's *Live Monitoring* section).
- **`record:=true`** writes the MCAP bag. The container has
  `ros-humble-rosbag2-storage-mcap` installed.
- **`bag_prefix`** must be **unique per run**: a timestamp is appended, but distinct
  prefixes keep runs (and parallel experiments) from colliding in the data dir.
- **`alpha`** (NBV only) is the CI-NBV cost weight, the experiment-sweep knob; no XML
  edit needed.
- **`environment`** selects the scene yaml (default `environment_basic` if omitted —
  see the note in [§4](#4-running-an-experiment) above). Set this explicitly every
  time; a forgotten `environment:=` still runs cleanly, just against the wrong scene.

---

## 5. Analyzing results

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
meshes are co-located. Runs **accumulate**; nothing is overwritten.

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

**Pull everything, not just one subfolder**: grab all of `data/` so bags *and*
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

## 6. Running experiments in parallel

`run_CINBV_experiment.sh` sweeps alpha in {0, 0.25, 0.5, 0.75, 1.0}. It runs missions
in **batches of `MAX_PARALLEL` (default 3), one per GPU**, with staggered startup, a
dedicated CPU block + resource caps per mission, and a distinct `ROS_DOMAIN_ID` each.
It prints live `mapped X/M` progress per mission while a batch runs.
`run_boustrophedon_experiment.sh` is the single CPU-only baseline.

Host-side scripts (`git pull` + `chmod +x`, **no rebuild**):

```bash
chmod +x run_CINBV_experiment.sh run_boustrophedon_experiment.sh

# 5-alpha cone sweep, 3-at-a-time (SAMPLER is required; pair with the cone image)
IMAGE=rosmosis:cone SAMPLER=cone ./run_CINBV_experiment.sh

# 5-alpha helix sweep (pair SAMPLER=helix with the helix image)
IMAGE=rosmosis:helix SAMPLER=helix ./run_CINBV_experiment.sh

# boustrophedon baseline (CPU-only)
IMAGE=rosmosis:cone ./run_boustrophedon_experiment.sh
```

Env-overridable knobs:

| Var | Default | Meaning |
|---|---|---|
| `IMAGE` | (required) | built image tag — must be the image whose BT matches `SAMPLER` |
| `SAMPLER` | (required, `cone`\|`helix`) | **CINBV only.** Bag label; must match the sampler baked into `IMAGE` |
| `MAX_PARALLEL` | 3 | concurrent missions; must be ≤ 3 (1 per GPU) |
| `CORES_PER` | 32 | dedicated cores per mission (`--cpuset-cpus`) |
| `OMP_THREADS` | 10 | OpenMP pool for the Open3D TSDF work |
| `MEM_LIMIT` | 16g | RAM ceiling (backstop; ~9g used) |
| `M_TARGETS` | 10 | scene target count, for the progress bar |
| `STAGGER` | 15 | seconds between launches in a batch |
| `ENVIRONMENT` | env_1000x1000_cluster_seabed | scene yaml |

**Batch ≤3, not all-5:** only 3 GPUs (so no card is shared), and the host must keep
cores, so running 5 at once oversubscribes and starves the wall-clock sim, which
corrupts timing *and* reconstructions.

---

## 7. Quick reference

```bash
# --- on the server, one time ---
git clone git@github.com:UTNuclearRobotics/ROSMOSIS.git ~/ROSMOSIS
cd ~/ROSMOSIS && git checkout experiment-docker

# --- build one image per sampler (re-run after every push you want picked up) ---
TAG=cone  ./build_image.sh       # -> rosmosis:cone   (edit BT to cone, push, then build)
TAG=helix ./build_image.sh       # -> rosmosis:helix  (edit BT to helix, push, then build)

# --- single run (headless, persisted) ---
docker run --rm --gpus '"device=0"' \
  -v "$HOME/ROSMOSIS/data:/workspace/data" \
  rosmosis:cone \
  ros2 launch demo_behaviors demo_mission_launch.py \
      start_rviz:=false debug_gui:=false record:=true \
      environment:=env_1000x1000_cluster_seabed \
      alpha:=0.25 bag_prefix:=nbv_cone_alpha0.25_fullscale

# --- alpha sweep (batched 1-per-GPU, live progress); pair SAMPLER with the image ---
IMAGE=rosmosis:cone  SAMPLER=cone  ./run_CINBV_experiment.sh
IMAGE=rosmosis:helix SAMPLER=helix ./run_CINBV_experiment.sh
IMAGE=rosmosis:cone  ./run_boustrophedon_experiment.sh

# --- pull ALL results down (from a LOCAL terminal, merges into ./data) ---
rsync -avzP <user>@<server>:~/ROSMOSIS/data/ ./data/
```
