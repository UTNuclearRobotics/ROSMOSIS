# bayesian_search

Bayesian occupancy-map search server for the ROSMOSIS mission stack. Maintains a 2D probability grid over the search arena, updates it continuously from FLS depth observations, and exposes a service that returns the next vehicle waypoint to minimize uncertainty.

All tunable parameters are hardcoded in `__init__` of `BayesianSearchServer` ([bayesian_search/bayesian_search_server.py](bayesian_search/bayesian_search_server.py)).

---

## Depth convention

The server operates in two coordinate frames simultaneously:

| Frame | Convention | z=0 means |
|---|---|---|
| `map` | z-up (ENU-style) | seafloor surface |
| `ned` | z-down (NED) | NED frame origin |

The static TF published by the mission launch places the `ned` origin **50 m above the seafloor** (`map → ned`: x=0, y=0, z=50 m, yaw=90°, roll=180°) — the ARL full-scale seabed depth. This means:

| NED depth | Altitude above seafloor |
|---|---|
| 0 m | 50 m (NED origin) |
| 35 m | 15 m (search altitude / clearance) |
| 50 m | 0 m (seafloor) |

`search_depth` is a **NED depth** — a positive number that grows as the vehicle descends. It must be strictly less than `seafloor_depth` (resolved at startup from the `map → ned` TF; 50.0 m in the ARL full-scale launch config) or the vehicle would be at or below the seafloor.

---

## Parameters

### Grid

All grid parameters are in the `map` frame. The grid is axis-aligned with `map` (x = east, y = north).

| Parameter | Default | Description |
|---|---|---|
| `grid_resolution` | `1.0` m/cell | Size of one grid cell. Finer resolution increases map fidelity but scales memory and update cost as O(n²). |
| `arena_size` | `(1000.0, 1000.0)` m | Physical extent of the search region as `(width_x, height_y)` in the map frame. Grid shape = `arena_size / grid_resolution` (1M cells at 1 m resolution). Must match the deployed environment. |
| `grid_origin` | `(0.0, 0.0)` | `(x_min, y_min)` of the grid's southwest corner in the map frame. Set to `(0.0, 0.0)` to align with the `env_1000x1000_cluster_seabed` environment. |

### Prior distribution

The probability grid is initialized with a Gaussian centered on `prior_centroid` plus a small uniform baseline floor of **0.01** to ensure every unswept cell has non-zero utility.

| Parameter | Default | Description |
|---|---|---|
| `prior_centroid` | center of arena | `(cx, cy)` in the map frame. Defaults to `grid_origin + arena_size / 2`. Override when targets are expected in a specific sub-region. |
| prior sigma | `arena_size[0] / 5` | Std dev of the initial Gaussian prior, set inline at the `gaussian_bump` call in `_build_grid` (not a stored member). Auto-scales with the arena (200 m at 1000×1000): a broad central bias that mildly prioritizes the center-weighted target field, while the 0.01 floor + swept-cell zeroing guarantee exhaustive coverage regardless. |

### Cluster bumps

When the BT reports a completed inspection target via the service request's `visited_targets` field, the server adds a Gaussian bump centered at that target's map position to redirect the search toward likely neighbors. These values are set inline in `get_next_pose_callback`.

| Parameter | Value | Description |
|---|---|---|
| Bump amplitude (`alpha`) | `1.0` | Peak height of the bump added to the probability grid. |
| Bump spread (`sigma`) | `25.0` m | Standard deviation of the cluster bump. Sized at roughly half a 50 m tether length to cover the neighborhood of a detected target. |

### Vehicle / FLS geometry

These parameters govern how the server converts a high-utility grid cell into a concrete vehicle waypoint. All search poses are north-facing in NED.

| Parameter | Default | Description |
|---|---|---|
| `search_depth` | `35.0` m (NED) | Depth at which the vehicle holds while searching. In NED convention: positive = deeper. For the ARL full-scale launch (seafloor at NED depth 50.0 m), this gives 15 m altitude above the seafloor. **Must be less than `seafloor_depth`** (resolved from TF at startup). |
| `mount_angle` | `20.0°` | FLS down-tilt from horizontal (matches the URDF `fls_mount_angle_deg` and the samplers). Determines `x_adjustment` = clearance / tan(mount_angle): how far north of the target cell the vehicle must sit for the FLS center beam to land on it. Changing this without updating the sensor model will produce incorrect back-projections. |

**Derived values** (computed once at startup from the TF and the parameters above; logged at `INFO` level):

| Derived value | Formula | Example (defaults) |
|---|---|---|
| `seafloor_depth` | from `map → ned` TF translation z | `50.0` m |
| `clearance` | `seafloor_depth − search_depth` | `15.0` m |
| `x_adjustment` | `clearance / tan(mount_angle)` | `41.2` m (15 / tan 20°) |

### Cost function

The server picks the next waypoint by maximizing utility = `probability / cost`, where cost approximates the Dubins travel expense to each cell.

| Parameter | Default | Description |
|---|---|---|
| `turning_radius` | `10.0` m/rad | Converts heading error (rad) into equivalent arc length (m) for the yaw penalty term: `cost = Euclidean_distance + turning_radius × |yaw|`. Set to the Dubins planner's minimum turning radius (`turn_radius_m` in `dubins_pose_to_pose_action_server.py`, now `10.0` m under the ARL full-scale spec), so the search cost model and the true planner radius agree. |

---

## ROS interfaces

| Name | Type | Description |
|---|---|---|
| `~/get_next_search_pose` | Service (`BayesianSearch`) | Returns the next NED waypoint. Request carries completed inspection targets for cluster bumping. |
| `~/probability_grid` | `nav_msgs/OccupancyGrid` | Belief map for RViz visualization (2 Hz). Values normalized to [0, 100]. |
| `/sonar/depth/image_raw` | `sensor_msgs/Image` | FLS depth image subscription for continuous map updates. |
| `/sonar/depth/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics for back-projection (cached on first message). |
