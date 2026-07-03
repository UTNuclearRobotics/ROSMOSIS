# demo_behaviors

> **Note:** this README is an LLM-written first draft based on the package's code and comments. Sections may need refinement as the package evolves.

Behavior tree nodes and mission orchestration for the VISTA UUV NBV mission.

This package contains:
- Custom behavior tree (BT.cpp v4) nodes for detection, sampling, planning, and target lifecycle management
- Two behavior trees: `MainTree` (full mission) and `NBVOnTarget` (per-target inspection subtree)
- A combined launch file that brings up the full mission stack (sim + NBV server + helix service + BT runner)
- A test tree (`TestDetect`) for verifying detection and queue logic in isolation

## Prerequisites

This package depends on three external repos that need to be cloned into the workspace:

```bash
cd ~/projects/rosmosis_ws/src

# 1. sample_nbv_behaviors (provides the nbv_behaviors package).
# Switch to the rosmosis branch — main depends on magellan_interfaces, which is not public.
git clone git@github.com:UTNuclearRobotics/sample_nbv_behaviors.git
cd sample_nbv_behaviors && git checkout rosmosis && cd ..
```

- **[nrg_behaviors](https://github.com/UTNuclearRobotics/nrg_behaviors)** — clone and follow its README for setup
- **[nbv_cpp](https://github.com/UTNuclearRobotics/nbv_cpp)** — clone and follow its README for setup

Then return to the workspace root and build everything.

## Quick Start

### Build

```bash
cd ~/projects/rosmosis_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

`-DCMAKE_BUILD_TYPE=Release` is important for nbv_cpp — without it, its C++ TSDF integration and ray-cast loops run unoptimized (no inlining, no SIMD, etc.) and become noticeably slower. The Python parts of the stack (vista_sim, pose_generator) are unaffected by this flag.

### Launch the full mission

```bash
ros2 launch demo_behaviors demo_mission_launch.py
```

This starts:
- `vista_sim` (simulator, Dubins action server, sensor publisher, RViz, static TFs)
- `helix_service` and `cone_service` (Python viewpoint samplers from pose_generator; the tree selects one)
- `next_best_view_server` (from nbv_cpp, configured via `sensor_model/config/nbv_params.yaml`)
- `bayesian_search_server` (probability map + next-waypoint service for the search subtree)
- `run_bt` (behavior tree runner; `MainTree`'s `CheckForServers` gates startup, so no launch-side delay is needed)
- `rqt_console` (filterable log viewer for all nodes) and `rqt_graph` (node/topic topology) — only when `debug_gui:=true`
- `ros2 bag record` (only when `record:=true`; see [Recording a run](#recording-a-run))

### Example: override launch parameters

```bash
# Slower drift, faster nav, no RViz
ros2 launch demo_behaviors demo_mission_launch.py \
    drift_velocity:=0.1 \
    constant_velocity:=0.8 \
    start_rviz:=false
```

See all available args:

```bash
ros2 launch demo_behaviors demo_mission_launch.py --show-args
```

### Inspect with Groot2

The BT runner publishes its state on TCP port 1667 via `BT::Groot2Publisher`. After launching:

1. Open Groot2 (download AppImage from [behaviortree.dev](https://www.behaviortree.dev/groot/))
2. Monitor → Connect to `localhost:1667`
3. Watch nodes light up live as the tree ticks

The TreeNodesModel for Groot's palette is auto-written to `config/models.xml` on every build by `BT::writeTreeNodesModelXML(factory)`.

## Mission Launch Parameters

All parameters are declared in [launch/demo_mission_launch.py](launch/demo_mission_launch.py) and can be either:
- **Overridden at the command line** (`ros2 launch ... param:=value`) for one-off runs
- **Edited as defaults** in the launch file itself, making it the one-stop shop for persistent config

| Parameter | Default | Description |
|---|---|---|
| `environment` | `env_1000x1000_cluster_seabed` | Environment yaml name from sensor_model/config/ |
| `start_rviz` | `true` | Start RViz with the simulation. Also gates the Bayesian belief-grid publish (headless runs skip the 2 Hz full-grid publish). |
| `drift_velocity` | `0.25` | Idle drift velocity when not navigating (m/s) |
| `constant_velocity` | `1.5` | Navigation velocity for Dubins paths (m/s). ARL full-scale spec. |
| `turn_radius_m` | `10.0` | Vehicle minimum turn radius (m); planner inflates ~20%. Forwarded to vista_sim. ARL full-scale spec. |
| `max_pitch_deg` | `15.0` | Vehicle maximum pitch angle (deg) for planning and dynamics. Forwarded to vista_sim. |
| `time_step` | `0.1` | Simulation time step (s) |
| `log_level` | `info` | ROS log level (debug/info/warn/error/fatal) applied to demo_bt, helix_service, nbv_server, and all vista_sim Python nodes |
| `record` | `false` | Record a mission rosbag for offline analysis (see [Recording a run](#recording-a-run)) |
| `bag_prefix` | `nbv` | Output bag directory prefix; a timestamp is appended |
| `alpha` | `0.5` | CI-NBV cost weight in [0,1] for `GetBestViewWithCost`: utility = (1-alpha)·IG_norm − alpha·cost_norm. 0 = pure info-gain (greedy), 1 = pure cost. The knob to sweep across experiments. |
| `debug_gui` | `false` | Start the `rqt_console` / `rqt_graph` GUIs. Leave `false` for headless / batch / parallel runs (they need a display); set `true` while debugging interactively. |

`alpha` is plumbed through to the behavior tree: the launch passes it to `run_bt`
(node `demo_bt`), which seeds it onto the root blackboard, and `GetBestViewWithCost`
reads it as `{alpha}`. So you sweep it purely from the command line, no XML edit:

```bash
ros2 launch demo_behaviors demo_mission_launch.py \
    environment:=env_1000x1000_cluster_seabed record:=true \
    alpha:=0.25 bag_prefix:=nbv_cone_alpha0.25 debug_gui:=false
```

### Recording a run

Pass `record:=true` to capture a rosbag of the mission for offline analysis (CIR, coverage, detection timing):

```bash
ros2 launch demo_behaviors demo_mission_launch.py record:=true
```

Each run is saved as its **own directory** under `data/bags/`, named `<bag_prefix>_<timestamp>/`. The timestamp (`YYYYMMDD_HHMMSS`) is **always appended automatically**, so runs never collide. `bag_prefix` is just a human-readable label on the front; it defaults to `nbv`, so the command above produces e.g. `data/bags/nbv_20260617_163756/`. Override it to tag the run's configuration:

```bash
ros2 launch demo_behaviors demo_mission_launch.py record:=true bag_prefix:=nbv_alpha0.5_helix
# -> data/bags/nbv_alpha0.5_helix_20260617_163756/
```

Inside each run directory you get the `.mcap` file plus a `metadata.yaml` (written last, on clean close — its presence means the bag finalized correctly).

- **Topics**: `/face_hits` (CIR `(geometry_id, primitive_id)` pairs), `/detected_boxes` (per-detection box centroids, published only when something is seen), `/tf`, `/tf_static`.
- **Format**: MCAP. It is self-describing (embeds message schemas, so the custom `FaceHits` decodes in Python without sourcing ROS) and crash-robust. Requires the storage plugin: `sudo apt install ros-humble-rosbag2-storage-mcap`.
- **Location**: `<workspace-root>/data/bags/<bag_prefix>_<timestamp>/` — run `ros2 launch` from the workspace root. `data/bags/` is gitignored; keeper bags are uploaded to external storage where needed.
- **Auto-stop**: when the mission ends (the tree returns SUCCESS/FAILURE at the root — mission timeout or all targets inspected — and `run_bt` exits), the launch shuts down via an `OnProcessExit` handler. This SIGINTs every process, which cleanly finalizes the bag, so a recorded run is hands-off (no Ctrl-C needed).

This is the same topic set, format, and `data/bags/` layout as the `baseline_mission` boustrophedon launch, so NBV and baseline runs are directly comparable. For a time-bounded comparison, set the mission budget (`isWithinTimeLimit` in `main_tree.xml`) equal to the baseline survey duration.

### Log level modes

| `log_level` | What you see |
|---|---|
| `debug` | Everything: per-frame detection, queue every tick, Dubins progress, BT state transitions |
| `info` (default) | Queue 1Hz, Dubins progress 10Hz, BT state transitions, no detection spam |
| `warn` | Only warnings/errors. Use for clean demos / thesis recordings. |

The same `log_level` is forwarded to the included `vista_sim_launch.py` so the entire stack uses one consistent level.

## Trees

### MainTree (root, in `behavior_trees/main_tree.xml`)

Top-level mission orchestration. Runs `DetectAndSortQueue` continuously in parallel with a target-inspection loop that pops queued targets and invokes `NBVOnTarget`.

The inspection loop is wrapped in `RetryUntilSuccessful` as a **placeholder**: when the queue is empty, `isQueueNotEmpty` fails and the retry decorator absorbs the failure so the tree keeps re-ticking until a target appears. This will be replaced by a proper `Search` subtree as the second Fallback alternative (TODO in the XML).

### NBVOnTarget (subtree, in `behavior_trees/nbv_on_target.xml`)

Per-target inspection cycle:

1. `SetROIExtents` — build the ROI extents Vector3
2. `MakeShared` — wrap PoseStamped to SharedPtr for PublishTransform
3. `PublishTransform` — broadcast `roi_frame` at the target position relative to `map`
4. `ActivatePolicy` — initialize NBV solver for this target
5. `SampleViewPosesHelix` — generate candidate viewpoint poses on a helical manifold (a commented-out `SampleViewPosesCone` block in the XML offers an apex-at-origin cone manifold as a drop-in alternative; both write `{poses}`)
6. `SetViews` — initial scoring of all candidates
7. `RepeatWhile`(not saturated):
   - `GetBestView` — argmax over scores, remove from candidate list
   - `CalculatePlanningPose` — convert sonar_optical view pose to base_link planning pose
   - `DubinsClient` — convert to NED and drive to planning pose (cancels on timeout)
   - `SetViews` — re-score remaining candidates with updated TSDF
8. `ConcludePolicy` — finalize the reconstruction for this target
9. `MarkTargetComplete` — signal DetectAndSortQueue to retire the target

### TestDetect (in `behavior_trees/test_detect.xml`)

Minimal tree that just runs `DetectAndSortQueue` in a loop. Useful for verifying detection and queue management without the rest of the NBV pipeline.

## Custom Behaviors

| Node | Type | Description |
|---|---|---|
| `DetectAndSortQueue` | SyncActionNode | Subscribes to `/detected_boxes`, accumulates and sorts targets by distance, dedupes via visited_hashmap |
| `isQueueNotEmpty` | SyncActionNode | Pops front of queue, exposes current target's geometry ID and PoseStamped |
| `SampleViewPosesHelix` | StatefulActionNode | Async client to `generate_helix` service (Python). Outputs PoseArray of viewpoints in roi_frame |
| `SampleViewPosesCone` | StatefulActionNode | Async client to `generate_cone` service (Python). Apex-at-origin cone manifold; outputs PoseArray of viewpoints in roi_frame |
| `DubinsClient` | StatefulActionNode | Async action client to `pose_to_pose`. TF-transforms planning pose to NED, sends goal, polls result. Client-side timeout cancels if exceeded |
| `MarkTargetComplete` | SyncActionNode | Reads current_id, writes completed_id (signals DetectAndSortQueue to retire) |

Plus external behaviors registered:
- All from `nbv_behaviors` (the sample_nbv_behaviors directory) — ActivatePolicy, SetROIExtents, SetViews, CheckScoreSaturation, GetBestView, CalculatePlanningPose, ConcludePolicy
- All from `nrg_behaviors` via `nrg_behaviors::registerBehaviors(factory, config)` — PublishTransform, RepeatWhile, etc.
- `MakeShared<PoseStamped>` — registered explicitly because it's a class template; the convenience function can't auto-register without knowing the type

## Architecture

`run_bt.cpp` uses a two-thread architecture:

- **Main thread**: runs `tree.tickWhileRunning()`. Polls futures with `wait_for(0ms)`. Never spins the ROS executor itself.
- **jthread**: continuously runs `executor->spin_some()`. Dispatches all incoming ROS work (service responses, action feedback/results, subscriptions, TF, timers). `std::jthread` (C++20) provides RAII cleanup.

Custom behaviors that need ROS access take a shared `rclcpp::Node::SharedPtr` in their constructor (third arg). The shared node is the one spun by the jthread, so all async work goes through a single executor.

Sample NBV behaviors (from sample_nbv_behaviors) use a different pattern — each owns its own internal node and uses `spin_until_future_complete` from the BT thread. Both patterns coexist in the same tree.

## Related Packages

- [vista_sim](../vista_sim/README.md) — simulator, Dubins action server. See its README for sim-specific parameters and environments.
- `pose_generator` — Python services publishing sampled viewpoints (helix, cone)
- `nbv_cpp` — TSDF + NBV scoring server
- `uuv_interfaces` — message, service, and action definitions (`GenerateHelix.srv`, `PoseToPose.action`, etc.)
- `sample_nbv_behaviors` (package name `nbv_behaviors`) — base NBV behaviors (ActivatePolicy, SetViews, etc.)
- `nrg_behaviors` — utility behaviors (MakeShared, PublishTransform, RepeatWhile, etc.)