# ROSMOSIS

**Robot Operating System — Multiple Object Surface Inspection Survey** — a ROS 2 Humble simulation workspace for UUV-based NBV inspection and Bayesian search missions using BT.cpp v4 behavior trees.

The system combines:
- Dubins-path vehicle simulation with idle drift dynamics
- Next-Best-View (NBV) inspection of detected targets on a cone or helix viewpoint manifold
- Bayesian occupancy search to find targets in unexplored space
- A boustrophedon (lawnmower) coverage baseline for non-adaptive comparison
- FLS (Forward-Looking Sonar) sensor simulation via ray casting

## Repository Structure

```
src/
├── demo_behaviors/       # BT.cpp v4 nodes, behavior trees, and full NBV mission launch
├── baseline_mission/     # Boustrophedon lawnmower coverage baseline (non-adaptive)
├── vista_sim/            # Dubins action server, vehicle kinematics, environment launch
├── sensor_model/         # FLS ray-cast sensor publisher and environment configs
├── uuv_interfaces/       # ROS 2 message, service, and action definitions
├── pose_generator/       # Python services: helix and cone viewpoint samplers
├── eca_a9_description/   # URDF/xacro for the ECA A9 UUV model
├── bayesian_search/      # Bayesian occupancy map and search pose server
├── nbv_cpp/              # TSDF + NBV scoring server (submodule)
├── sample_nbv_behaviors/ # NBV behavior tree nodes (submodule, rosmosis branch)
└── nrg_behaviors/        # Utility BT nodes: PublishTransform, RepeatWhile, etc. (submodule)
```

## Prerequisites

Clone the three external submodules into `src/`:

```bash
cd ~/projects/rosmosis_ws/src

# NBV behaviors
git clone git@github.com:UTNuclearRobotics/sample_nbv_behaviors.git
cd sample_nbv_behaviors && git checkout rosmosis && cd ..

# NRG utility behaviors
git clone git@github.com:UTNuclearRobotics/nrg_behaviors.git

# NBV TSDF server
git clone git@github.com:UTNuclearRobotics/nbv_cpp.git
```

Follow each submodule's README for any additional setup.

## Build

```bash
cd ~/projects/rosmosis_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`-DCMAKE_BUILD_TYPE=Release` is required for `nbv_cpp` — its TSDF integration and ray-cast loops are significantly slower in debug mode. The Python packages (`vista_sim`, `pose_generator`, `sensor_model`) are unaffected.

For a partial build (sim + mission stack only, skipping nbv_cpp):

```bash
colcon build --packages-up-to demo_behaviors --symlink-install
source install/setup.bash
```

## Launch

### Full mission

```bash
ros2 launch demo_behaviors demo_mission_launch.py
```

Starts: vehicle sim, FLS sensor, NBV server, helix service, Bayesian search server, behavior tree runner, RViz, rqt_console, rqt_graph.

### With a specific environment

```bash
ros2 launch demo_behaviors demo_mission_launch.py environment:=env_1000x1000_cluster_seabed
```

| Environment | Description |
|---|---|
| `env_1000x1000_cluster_seabed` | **ARL full-scale (default).** 1000×1000 m procedural rolling seabed, 10 lobster pots in 5 isolated clusters, 200 m separation |
| `environment_basic` | Flat 200×200 m seabed, two pots |
| `environment_seabed_basic` | Procedural rolling seabed |
| `environment_50x50` | Flat 50×50m arena, four boxes at corners |
| `env_50x50_centroidBox` | Flat 50×50m arena, single box offset from centroid |
| `env_50x50_cluster` | Flat 50×50m arena, three boxes clustered mid-arena + one corner |
| `env_50x50_cluster_seabed` | Procedural seabed with divot at cluster site, same box layout |

### Key launch parameters

```bash
ros2 launch demo_behaviors demo_mission_launch.py \
    environment:=env_1000x1000_cluster_seabed \
    drift_velocity:=0.1 \
    constant_velocity:=2.0 \
    log_level:=debug
```

| Parameter | Default | Description |
|---|---|---|
| `environment` | `env_1000x1000_cluster_seabed` | Environment yaml from `sensor_model/config/` |
| `start_rviz` | `true` | Launch RViz (also gates the Bayesian belief-grid publish) |
| `drift_velocity` | `0.25` | Idle drift speed between goals (m/s) |
| `constant_velocity` | `1.5` | Navigation speed along Dubins paths (m/s). ARL full-scale spec. |
| `turn_radius_m` | `10.0` | Vehicle minimum turn radius (m); planner inflates ~20%. ARL full-scale spec. |
| `max_pitch_deg` | `15.0` | Vehicle maximum pitch angle (deg) for planning and dynamics. |
| `time_step` | `0.1` | Simulation dt (s) |
| `log_level` | `info` | ROS log level for all nodes (debug/info/warn/error/fatal) |
| `record` | `false` | Record an MCAP bag of the mission (`/face_hits`, `/detected_boxes`, `/tf`, `/tf_static`) into `data/bags/`. Requires `ros-humble-rosbag2-storage-mcap`. |
| `bag_prefix` | `nbv` | Run label for the output. A timestamp is appended automatically, and the same label tags both the bag (`data/bags/`) and the per-run reconstruction folder (`data/reconstructions/`). |
| `alpha` | `0.5` | CI-NBV cost weight in [0,1] for `GetBestViewWithCost`: utility = (1-alpha)·IG_norm − alpha·cost_norm. 0 = pure info-gain, 1 = pure cost. Plumbed CLI → `run_bt` → blackboard → BT, so it is the experiment-sweep knob (no XML edit). NBV mission only. |
| `debug_gui` | `false` | Start the `rqt_console` / `rqt_graph` GUIs. Leave `false` for headless / batch / parallel runs (they need a display); `true` while debugging. Present in both the NBV and baseline launches. |

See all args: `ros2 launch demo_behaviors demo_mission_launch.py --show-args`

### Baseline mission (boustrophedon)

The `baseline_mission` package provides the **non-adaptive comparison baseline**: an
exhaustive boustrophedon (lawnmower) survey that sweeps the whole arena at a fixed
clearance, ignoring where the targets are. It reuses the same simulator, Dubins
action server, and sensor model as the NBV mission, and records the **same topic
set and bag layout**, so the two are directly comparable.

```bash
ros2 launch baseline_mission baseline_mission_launch.py \
    environment:=env_1000x1000_cluster_seabed record:=true bag_prefix:=boustrophedon_cluster
```

| Parameter | Default | Description |
|---|---|---|
| `environment` | `env_1000x1000_cluster_seabed` | Environment yaml from `sensor_model/config/` |
| `start_rviz` | `true` | Launch RViz |
| `drift_velocity` | `0.25` | Idle drift speed between goals (m/s) |
| `constant_velocity` | `1.5` | Navigation speed along Dubins paths (m/s). ARL full-scale spec. |
| `turn_radius_m` | `10.0` | Vehicle minimum turn radius (m); planner inflates ~20%. ARL full-scale spec. |
| `max_pitch_deg` | `15.0` | Vehicle maximum pitch angle (deg) for planning and dynamics. |
| `time_step` | `0.1` | Simulation dt (s) |
| `log_level` | `info` | ROS log level (debug/info/warn/error/fatal) |
| `record` | `false` | Record an MCAP bag (`/face_hits`, `/detected_boxes`, `/tf`, `/tf_static`) into `data/bags/` |
| `bag_prefix` | `boustrophedon` | Run label; a timestamp is appended; tags the bag folder under `data/bags/` |
| `debug_gui` | `false` | Start the `rqt_console` / `rqt_graph` GUIs. Leave `false` for headless / batch / parallel runs |

The baseline takes the same launch args as the NBV mission **except `alpha`** (which is
NBV-only). The **survey geometry** (clearance, FOV, mount angle, ROI extents) is set as
members in `boustrophedon_run.py`, not via launch args — see the package README. See all
args: `ros2 launch baseline_mission baseline_mission_launch.py --show-args`.

It runs a standalone `rclpy` action client (no behavior tree, no NBV/search servers):
it computes the lawnmower waypoints in-file from the FLS swath geometry (lane spacing
= half the cross-track footprint for 50% overlap), then dispatches them one at a time
as `PoseToPose` goals. Lane spacing and the along-track running start are derived from
the sensor FOV and mount angle, so they stay consistent with the NBV sensor. The
launch self-terminates when the survey finishes. See
[baseline_mission/README.md](src/baseline_mission/README.md) for the geometry and
tunable parameters.

## Mission Architecture

### Behavior Trees

**MainTree** — top-level mission loop (10 cycles, one per pot):
- `DetectAndSortQueue` runs continuously in parallel, accumulating detected targets sorted by distance
- `ReactiveFallback` switches between two branches each tick:
  - **Inspection branch**: if queue is non-empty, run `NBVOnTarget` on the front target
  - **Search branch**: if queue is empty and within time budget, run `BayesianSearch` to drive toward high-probability unexplored regions

**NBVOnTarget** — per-target inspection:
1. Publish ROI frame at target position
2. Activate NBV policy, generate cone (or helix) viewpoints, score with TSDF
3. Iterate: drive to best view → re-score → repeat until saturation
4. Conclude policy, mark target complete

**BayesianSearch** — occupancy-driven exploration:
- Queries `bayesian_search_server` for the next high-utility pose
- Drives there via `DubinsClient`
- Sensor observations update the occupancy map, reducing uncertainty

### Vehicle Simulation

`vehicle_sim_server` is the sole publisher of `ned→base_link`. It:
- Plans Dubins paths to action goals and follows them with an LOS controller
- Propagates idle drift between goals at `drift_velocity`
- Declares a goal complete when within 3.0m + 0.3 rad, or when the path is fully consumed and the vehicle is receding from closest approach (miss detection)

### Sensor Model

Ray-cast FLS sensor in `sensor_model`. Publishes:
- `/sonar/depth/image_raw` and `/sonar/depth/camera_info`: synthetic depth image in the `sonar_optical` frame
- `/detected_boxes`: detected object poses (box centroids) tagged with geometry IDs, in the `map` frame
- `/face_hits`: per-frame (geometry_id, primitive_id) hit pairs used for the CIR metric, in the `map` frame
- Mesh markers for RViz visualization

### Live Monitoring

The BT runner publishes its state on TCP port 1667 via `BT::Groot2Publisher`:

1. Download Groot2 AppImage from [behaviortree.dev](https://www.behaviortree.dev/groot/)
2. Monitor → Connect to `localhost:1667`
3. Watch nodes light up live as the tree ticks

## Related Package READMEs

- [vista_sim/README.md](src/vista_sim/README.md): vehicle sim parameters, environments, TF verification
- [demo_behaviors/README.md](src/demo_behaviors/README.md): behavior node reference, custom behavior table, architecture details
- [baseline_mission/README.md](src/baseline_mission/README.md): boustrophedon lawnmower coverage baseline, including survey geometry, tunable parameters, and bag recording

## Mission Parameters (non-NBV) Tuning Reference

This section lists the parameters that configure the mission outside of the NBV
scoring algorithm itself (the TSDF resolution, viewpoint sampler geometry,
convergence threshold, and cost-weighting are documented separately). The
parameters below are grouped by concern: environment, arena, targets, vehicle,
sensor, mission timing, the boustrophedon baseline, the Bayesian search, and
output. Each entry gives the current value, the file it lives in, and what it
controls. Several quantities are duplicated across files; the final subsection
lists those and the values that must stay consistent for a valid comparison.

### Environment selection

- **`environment`** (launch argument, default `env_1000x1000_cluster_seabed`):
  selects the scene YAML from `sensor_model/config/`. A single argument determines
  the terrain mesh, the target boxes, and the box count. The full-scale default is
  `env_1000x1000_cluster_seabed` (10 pots in 5 clusters on a 1000×1000 m procedural
  seabed); smaller examples are `env_50x50_centroidBox` (one box on a flat plane)
  and `env_50x50_cluster_seabed` (four boxes on a procedural seabed). This is the
  primary switch for changing the test scenario.

### Arena and world geometry

- **Seabed depth / NED anchor**: the `map` to `ned` static transform has
  z = 50 m (`src/vista_sim/launch/vista_sim_launch.py`, the `--z` arg on the
  `static_transform_publisher` line). This sets the seafloor depth used to
  resolve vehicle clearance, and it caps how deep the vehicle can fly, so any
  clearance value must be smaller than this. Increase it to model a deeper water
  column.
- **Arena size and origin**: 1000 by 1000 m with origin at (0, 0)
  (`src/bayesian_search/bayesian_search/bayesian_search_server.py`, the
  `arena_size` and `grid_origin` members). The same extents are mirrored by the
  boustrophedon ROI and by the terrain mesh, and all three must agree.
- **Terrain mesh**: `assets/seabed_1000x1000m.obj` for the full-scale procedural
  seabed (env YAML `terrain.file_path`); smaller `assets/flatPlane_50x50m.obj` /
  `assets/seabed_50x50m.obj` remain for the 50×50 test scenes.

### Targets (lobster pots)

- **`box_count`** (env YAML): the number of pots in the scene. This must equal
  the mission's `Repeat num_cycles` (see Mission timing), otherwise the mission
  either stops before all pots are inspected or searches out the remaining time
  budget.
- **`box_positions`** (env YAML): per-pot (x, y, z) position in the map frame.
- **`box_size`** (env YAML): 1.0 by 1.0 by 0.5 m for each pot.

### Vehicle kinematics and dynamics

These are shared by every mission because all of them drive through the same
Dubins action server.

- **`turn_radius_m`** = 10.0 m (launch argument): minimum turn radius. The path
  planner uses 1.2 times this value to leave yaw-rate headroom for cross-track
  correction.
- **`max_pitch_deg`** = 15 degrees (launch argument): the Dubins-airplane
  flight-path-angle bound used in both planning and dynamics.
- **`constant_velocity`** = 1.5 m/s (launch argument): navigation speed along
  Dubins paths.
- **`drift_velocity`** = 0.25 m/s (launch argument): idle drift speed applied
  between goals.
- **`time_step`** = 0.1 s (launch argument): simulation integration timestep.
- **Spawn pose** = `Eta(-30, 0, 0, yaw=0)`, that is north = -30, east = 0, facing
  north (`src/vista_sim/vista_sim/dubins_pose_to_pose_action_server.py`,
  hardcoded). Shared by all missions. Set south of the boustrophedon's first lane
  (north = -row_extension ≈ -7.58 m at clearance=15) with ~22 m of run-up so the
  vehicle reaches cruise speed and settles before the first sweep lane.
- **Goal tolerance** = within 3.0 m position and 0.3 rad yaw
  (`dubins_pose_to_pose_action_server.py`, hardcoded). Controls when a waypoint
  counts as reached.
- **Controller constants**: `roll_ratio` = 0.2, `max_acceleration_mps2` = 1.0,
  `look_ahead_dist` = 1.0 (`dubins_pose_to_pose_action_server.py`, hardcoded).
  These tune the line-of-sight follower and are rarely changed.

### Sensor and mount geometry

These describe the forward-looking sonar. Several values are duplicated across
files (see Cross-file consistency).

- **Mount angle** = 20 degrees of downward tilt
  (`src/eca_a9_description/urdf/eca_a9_visual.urdf.xacro`, the
  `fls_mount_angle_deg` property). This is the physical source of truth. The same
  angle is also hardcoded in the boustrophedon node, the NBV sampler, and the
  search server, all of which re-aim viewpoints assuming this tilt.
- **Field of view** = 150 degrees horizontal by 25 degrees vertical
  (`src/sensor_model/sensor_model/compute_intrinsics.py`, `FLS_CONFIG`). The
  boustrophedon node hardcodes the same values for its swath math; the NBV and
  search read it from `camera_info`.
- **Beam count / image width** = 512 px (`compute_intrinsics.py`).
- **Maximum range** = 100 m (`src/sensor_model/sensor_model/ray_casting_core.py`,
  `self.max_depth`). Returns beyond this are discarded.
- **Bow offset**: the sonar is mounted 0.7 m forward of `base_link`
  (`eca_a9_visual.urdf.xacro`, the `base_to_sonar` joint origin).

### Mission timing and structure (NBV mission)

- **`Repeat num_cycles`** (`src/demo_behaviors/behavior_trees/main_tree.xml`,
  currently 10): the number of targets the mission inspects before it ends. Set
  this equal to the scene `box_count`.
- **`isWithinTimeLimit hours`** (`main_tree.xml`, currently 24): the mission time
  budget. It guards the search branch, so it acts as a backstop that ends the
  mission if the targets are not all found. For short test runs, lower it.
- **`DubinsClient timeout_sec`**: 1200 s in the NBV inspection subtree
  (`nbv_on_target.xml`) and 1500 s in the search subtree (`bayesian_search.xml`).
  This is the per-maneuver timeout, sized for the long transits of the 1000×1000 m
  arena. For short time-boxed missions, consider lowering it so a single stalled
  maneuver cannot consume the whole budget.

### Boustrophedon baseline (non-adaptive coverage)

Configured by the members at the top of
`src/baseline_mission/baseline_mission/boustrophedon_run.py`.

- **`clearance`** = 15.0 m above the seabed. The survey depth is derived from this
  and the resolved seafloor depth (50 − 15 = 35 m).
- **`fov_x_deg` / `fov_y_deg`** = 150 / 25. These drive the swath and lane
  spacing and must match the sensor model.
- **`mount_angle_deg`** = 20. Used for the along-track running start, and must
  match the URDF.
- **ROI bounds** (`roi_north_min/max`, `roi_east_min/max`) = 0 to 1000. The survey
  rectangle, which must match the arena.

### Bayesian search (non-adaptive search)

Configured by the members of
`src/bayesian_search/bayesian_search/bayesian_search_server.py`.

- **`grid_resolution`** = 1.0 m per cell (1M cells at 1000×1000; ~31 ms per query).
- **`arena_size`** = (1000, 1000), **`grid_origin`** = (0, 0). Must match the arena.
- **prior sigma** = `arena_size[0] / 5` (200 m at full scale): the width of the
  prior probability bump, set inline at the `gaussian_bump` call in `_build_grid`
  (no longer a stored member). The prior centroid defaults to the arena center;
  the 0.01 floor + swept-cell zeroing guarantee exhaustive coverage regardless.
- **`search_depth`** = 35.0 m NED, from which the search clearance is derived
  (15 m, matching the baseline).
- **`mount_angle`** = 20 degrees. Must match the URDF.

### Recording and output (both missions)

- **`record`** (launch argument, default `false`): toggles MCAP bag recording of
  `/face_hits`, `/detected_boxes`, `/tf`, and `/tf_static`.
- **`bag_prefix`** (launch argument, default `nbv` or `boustrophedon`): the run
  label. A timestamp is appended automatically. The same label tags both the bag
  directory (`data/bags/`) and the reconstruction directory
  (`data/reconstructions/`), so a run's bag and meshes are co-located.

### Cross-file consistency requirements

Several quantities appear in more than one file with no single source of truth,
so they can drift apart silently. For a valid method comparison, keep the
following aligned.

- **Mount angle (20 degrees)** appears in four places: the URDF (the physical
  source of truth), the boustrophedon node, the NBV sampler, and the search
  server. Changing one means changing all four.
- **Field of view (150 by 25 degrees)** is hardcoded in the boustrophedon node
  for its swath math, while the NBV and search read it from `camera_info`. If the
  sensor FOV changes, update the boustrophedon node to match.
- **Arena size (1000 by 1000) and origin (0, 0)** appear in the boustrophedon ROI,
  the search arena, and the terrain mesh, and all must agree.
- **Clearance** is currently 15 m for both the baseline and the search; keep them
  equal for a fair altitude comparison.
- **Seabed depth (50 m)** caps the clearance, so any clearance must be smaller
  than it.
