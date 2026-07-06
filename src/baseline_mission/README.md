# baseline_mission

Boustrophedon (lawnmower) coverage baseline for the VISTA UUV mission.

This is the **non-adaptive** comparison baseline for the NBV mission (`demo_behaviors`). It runs an exhaustive lawnmower survey over a rectangular ROI: a single standalone action client computes the coverage path in-file and dispatches the waypoints one at a time as `PoseToPose` goals to the shared Dubins action server. No behavior tree, no sampling/scoring servers.

It reuses the **same simulator, Dubins action server, and sensor model** as the NBV mission, so the two are directly comparable.

## Quick Start

### Build

```bash
cd ~/projects/rosmosis_ws
colcon build --symlink-install --packages-select baseline_mission
source install/setup.bash
```

(With `--symlink-install`, later edits to the node, launch file, and configs are picked up without rebuilding; a rebuild is only needed when adding files or changing `setup.py`/`package.xml`.)

### Launch

```bash
ros2 launch baseline_mission baseline_mission_launch.py environment:=env_1000x1000_cluster_seabed
```

This starts:
- `vista_sim` (simulator, Dubins action server, sensor model, RViz, static TFs)
- `boustrophedon_run` (the lawnmower action client)
- `rqt_console` (filterable log viewer) and `rqt_graph` (node/topic topology) — only when `debug_gui:=true`
- `ros2 bag record` (only when `record:=true`; see [Recording a run](#recording-a-run))

The client self-gates: it waits for the `ned<-map` TF (to resolve its survey depth from clearance) and for the `pose_to_pose` action server, so no launch-side delay is needed.

## How it works

`boustrophedon_run.py` is a plain `rclpy` action client (not a BT node):

1. **Resolve depth from the TF** — looks up `ned<-map`, reads `seafloor_depth = t.z`, and sets `survey_depth = seafloor_depth - clearance` so the vehicle flies a fixed clearance above the seabed.
2. **Build the lawnmower** — rows run along north, lanes step along east; alternating direction (boustrophedon). Each lane emits only its two endpoints; the Dubins server plans the straight run between them and the U-turn to the next lane.
3. **Dispatch sequentially** — send one `PoseToPose` goal, block until the result, send the next. A near-miss at a lane end (server returns `"Missed"`) is treated as success, so an overshoot never aborts the survey.

### Geometry

The forward-looking sonar footprint sets the lane layout. With the sensor at `clearance` `z` above the floor:

```
swath_cross  = 2 * z * tan(HFOV / 2)       # cross-track footprint width
lane_spacing = swath_cross / 2             # 50% side overlap
row_extension = z * (tan(mount + VFOV/2) - tan(mount - VFOV/2))   # along-track "running start"
```

The mount angle does **not** affect the swath/lane spacing (the FOV does); it only sets the along-track running start by which each row is extended past the ROI edges, so the swath fully sweeps the boundaries before the vehicle turns.

### Frame convention

Poses are in **NED** (matches the Dubins action server): `position.x = north`, `position.y = east`, `position.z = depth` (down +); `yaw = 0` heads north, `yaw = pi` heads south. The vehicle spawns at `Eta(-30, 0, 0, yaw=0)` (shared with the NBV mission), south of the first lane (which starts at north ≈ -7.58 m at clearance=15) with ~22 m of run-up so it reaches cruise speed and settles before the first sweep lane.

## Parameters

### Launch arguments (CLI-overridable, `param:=value`)

| Parameter | Default | Description |
|---|---|---|
| `environment` | `env_1000x1000_cluster_seabed` | Environment yaml name (no extension) from `sensor_model/config/` |
| `start_rviz` | `true` | Start RViz with the simulation |
| `drift_velocity` | `0.25` | Idle drift velocity when not navigating (m/s) |
| `constant_velocity` | `1.5` | Navigation velocity for Dubins paths (m/s). ARL full-scale spec. |
| `turn_radius_m` | `10.0` | Vehicle minimum turn radius (m); planner inflates ~20%. Forwarded to vista_sim. ARL full-scale spec. |
| `max_pitch_deg` | `15.0` | Vehicle maximum pitch angle (deg) for planning and dynamics. Forwarded to vista_sim. |
| `time_step` | `0.1` | Simulation time step (s) |
| `log_level` | `info` | ROS log level (debug/info/warn/error/fatal) |
| `record` | `false` | Record a survey rosbag (see [Recording a run](#recording-a-run)) |
| `bag_prefix` | `boustrophedon` | Output bag directory prefix; a timestamp is appended |
| `debug_gui` | `false` | Start the `rqt_console` / `rqt_graph` GUIs. Leave `false` for headless / batch / parallel runs (they need a display); set `true` while debugging interactively |

### Survey geometry (edit in [boustrophedon_run.py](baseline_mission/boustrophedon_run.py))

These are members at the top of `__init__`, not launch args. They must stay consistent with the sensor model and the NBV/search config for a fair comparison.

| Member | Default | Notes |
|---|---|---|
| `fov_x_deg` | `150.0` | Horizontal (cross-track) FOV; must match the sensor model (`compute_intrinsics.py`) |
| `fov_y_deg` | `25.0` | Vertical (along-track) FOV |
| `mount_angle_deg` | `20.0` | Sensor down-tilt; must match the URDF `fls_mount_angle_deg` |
| `clearance` | `15.0` | Altitude above the seabed (m); `survey_depth` is derived from it. ARL full-scale spec (50 m seabed − 15 m = 35 m survey depth). |
| `roi_north_min/max`, `roi_east_min/max` | `0 / 1000` | Survey rectangle (NED m); matches the 1000x1000 arena in bayesian_search |

## Recording a run

Pass `record:=true` to capture a rosbag for offline analysis (CIR, coverage, detection timing):

```bash
ros2 launch baseline_mission baseline_mission_launch.py environment:=env_1000x1000_cluster_seabed record:=true
```

Each run is saved as its **own directory** under `data/bags/`, named `<bag_prefix>_<timestamp>/`. The timestamp (`YYYYMMDD_HHMMSS`) is **always appended automatically**, so runs never collide. `bag_prefix` is just a human-readable label on the front; it defaults to `boustrophedon`, so the command above produces e.g. `data/bags/boustrophedon_20260617_151640/`. Override it to tag the run's configuration:

```bash
ros2 launch baseline_mission baseline_mission_launch.py environment:=env_1000x1000_cluster_seabed record:=true bag_prefix:=boustrophedon_clearance15
# -> data/bags/boustrophedon_clearance15_20260617_151640/
```

Inside each run directory you get the `.mcap` file plus a `metadata.yaml` (written last, on clean close — its presence means the bag finalized correctly).

- **Topics**: `/face_hits` (CIR `(geometry_id, primitive_id)` pairs), `/detected_boxes` (box centroids, published only when something is detected), `/tf`, `/tf_static`.
- **Format**: MCAP — self-describing (the custom `FaceHits` decodes in Python without sourcing ROS) and crash-robust. Requires `sudo apt install ros-humble-rosbag2-storage-mcap`.
- **Location**: `<workspace-root>/data/bags/<bag_prefix>_<timestamp>/` — run `ros2 launch` from the workspace root. `data/bags/` is gitignored; keeper bags are uploaded to external storage where needed.
- **Auto-stop**: when the survey finishes, `boustrophedon_run` exits; an `OnProcessExit` handler shuts the launch down, which SIGINTs every process and cleanly finalizes the bag. So a recorded run is hands-off (no Ctrl-C needed).

Inspect a finished bag:

```bash
ros2 bag info data/bags/<bag_prefix>_<timestamp>
```

This is the same topic set, format, and layout as the NBV mission launch (`demo_behaviors`), so baseline and NBV runs are directly comparable. The boustrophedon survey duration is the natural time budget to give the NBV+search mission for a time-bounded comparison.

## Related Packages

- [vista_sim](../vista_sim/README.md) — simulator, Dubins `pose_to_pose` action server, sensor model
- [demo_behaviors](../demo_behaviors/README.md) — the NBV mission this baseline is compared against
- `uuv_interfaces` — `PoseToPose.action`, `FaceHits.msg`
