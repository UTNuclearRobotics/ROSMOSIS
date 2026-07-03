# sensor_model

Ray-casting based FLS sensor simulation using Open3D.

## Launch

```bash
ros2 launch sensor_model sensor_test.launch.py [environment:=<name>]
```

The `environment` argument selects a config yaml from `sensor_model/config/` (omit the `.yaml` extension). Defaults to `environment_basic`.

| Environment | Description |
|---|---|
| `environment_basic` | Flat 200x200m seabed, two lobster pots at (0,10) and (0,20) |
| `environment_seabed_basic` | Procedural rolling seabed with roughness, boxes at z=0 |
| `environment_hill_valley` | Seabed with a valley at box_0 (0,10) and hill at box_1 (0,20) |
| `env_1000x1000_cluster_seabed` | **ARL full-scale.** 1000×1000 m procedural rolling seabed (±2.5 m band), 10 lobster pots in 5 isolated clusters, 200 m separation. Pairs with `bayesian_search` `arena_size=(1000,1000)`. |

### Examples

```bash
# Default flat seabed
ros2 launch sensor_model sensor_test.launch.py

# Procedural rolling seabed
ros2 launch sensor_model sensor_test.launch.py environment:=environment_seabed_basic

# Hill/valley terrain
ros2 launch sensor_model sensor_test.launch.py environment:=environment_hill_valley
```

## Generating a new seabed mesh

Edit parameters in `assets/generate_seabed.py` then run:

```bash
python3 assets/generate_seabed.py
```

Outputs `assets/seabed_200x200m.obj`. Inspect with:

```bash
python3 assets/inspect_mesh.py
```
