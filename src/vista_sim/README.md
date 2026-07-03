# vista_sim

Simulation package for UUV kinematics with Dubins path planning and drift dynamics.

## Quick Start

### Build

```bash
cd ~/projects/rosmosis_ws
colcon build --packages-up-to demo_behaviors --symlink-install
source install/setup.bash
```

### Launch

```bash
# Launch with default flat seabed
ros2 launch vista_sim vista_sim_launch.py

# Launch with a specific environment
ros2 launch vista_sim vista_sim_launch.py environment:=environment_50x50

# Launch without RViz
ros2 launch vista_sim vista_sim_launch.py start_rviz:=false

# Override velocity and time step
ros2 launch vista_sim vista_sim_launch.py drift_velocity:=0.1 constant_velocity:=1.0 time_step:=0.05

# Override vehicle turn radius / max pitch (the planner stays feasible automatically)
ros2 launch vista_sim vista_sim_launch.py turn_radius_m:=7.0 max_pitch_deg:=20.0
```

The `environment` argument selects a config yaml from `sensor_model/config/` (omit the `.yaml` extension).

| Environment | Description |
|---|---|
| `environment_basic` | Flat seabed, no objects (default) |
| `environment_seabed_basic` | Procedural rolling seabed with roughness |
| `environment_50x50` | Flat 50×50m arena, four boxes at corners |
| `env_50x50_centroidBox` | Flat 50×50m arena, single box offset from centroid |
| `env_50x50_cluster` | Flat 50×50m arena, three boxes clustered mid-arena + one corner |
| `env_50x50_cluster_seabed` | Procedural seabed with divot at cluster site, same box layout |
| `env_1000x1000_cluster_seabed` | **ARL full-scale.** 1000×1000 m procedural rolling seabed, 10 lobster pots in 5 isolated clusters (tight/spread/collinear/near-corner/flat-control), 200 m cluster separation |

### Send a Goal (separate terminal)

```bash
source ~/projects/VISTA_ws/install/setup.bash
ros2 action send_goal --feedback /pose_to_pose uuv_interfaces/action/PoseToPose \
  "{goal_pose: {header: {frame_id: 'ned'}, pose: {position: {x: 25.0, y: 25.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### Verify TF broadcast

```bash
ros2 run tf2_ros tf2_echo ned base_link
ros2 run tf2_ros tf2_monitor ned base_link
```

Expected: ~10 Hz, single publisher (`vehicle_sim_server`).

## Nodes

| Node | Description |
|------|-------------|
| `dubins_pose_to_pose_action_server` | Plans and executes Dubins paths to goal poses; sole owner of `ned→base_link` TF |
| `dubins_pose_to_pose_action_client` | Sends a parameterized goal to the action server |

## Interfaces

| Name | Type | Description |
|------|------|-------------|
| `pose_to_pose` | Action (`PoseToPose`) | Navigate to a goal pose |

## Parameters

### Launch-controlled (set via launch file or command line)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `frame_id` | `ned` | TF parent frame for `base_link` |
| `time_step` | `0.1` | Simulation dt (seconds) |
| `constant_velocity` | `1.5` | Navigation speed along Dubins paths (m/s). ARL full-scale spec. |
| `drift_velocity` | `0.25` | Idle drift speed between goals (m/s) |
| `turn_radius_m` | `10.0` | Vehicle minimum turn radius (m). ARL full-scale spec. The planner plans at **1.2×** this value so paths stay within the vehicle's yaw-rate limit and remain trackable (see note below). |
| `max_pitch_deg` | `15.0` | Vehicle maximum pitch angle (deg); shared by the planner and the dynamics model |
| `log_level` | `info` | ROS log level for all Python nodes (debug/info/warn/error/fatal) |

### Hardcoded vehicle dynamics (in `execute_cb`)

These govern the `SimpleVehicleModel` and require a code change to tune. (`turn_radius_m` and `max_pitch_deg` were promoted to launch args — see the table above.)

| Parameter | Value | Description |
|-----------|-------|-------------|
| `look_ahead_dist` | `1.0` | LOS follower look-ahead distance (m); affects path tracking tightness |
| `speed_time_constant` | `2.0` | First-order lag on surge speed response |
| `yaw_time_constant` | `0.2` | First-order lag on yaw response |
| `pitch_time_constant` | `1.5` | First-order lag on pitch response |
| `max_acceleration_mps2` | `1.0` | Surge acceleration limit (m/s²) |
| `pitch_proportional_gain` | `0.5` | Proportional gain for depth/pitch control |
| `roll_ratio` | `0.2` | Roll coupling ratio during turns |

**Planner / vehicle feasibility.** The vehicle's achievable turn rate is `ω_max = constant_velocity / turn_radius_m`. The Dubins planner is given a turn radius of `1.2 × turn_radius_m`. It plans paths ~20% gentler than the vehicle's hard limit. That headroom lets the LOS tracker spend yaw authority correcting cross-track error instead of saturating just to hold the nominal arc. Planning *at* the limit makes the tightest turns (especially the final heading alignment) untrackable, so the vehicle arrives off-heading and the goal is flagged as a miss. Both the planner radius and the vehicle's `turn_radius_m` derive from the single launch parameter, so they can never drift apart.

### Log level usage

```bash
# Default (info)
ros2 launch vista_sim vista_sim_launch.py

# Debug all Python nodes
ros2 launch vista_sim vista_sim_launch.py log_level:=debug

# Quiet (warnings/errors only)
ros2 launch vista_sim vista_sim_launch.py log_level:=warn
```

When launched via `demo_behaviors/demo_mission_launch.py`, the `log_level` arg is forwarded from the parent, so a single launch arg controls verbosity across the whole stack.