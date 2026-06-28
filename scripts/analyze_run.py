# %% [markdown]
# # CIR run analysis
# Percent-format script (run cell-by-cell in VSCode's interactive window, or as a
# plain `.py`). Computes the per-target Cumulative Information Ratio CIR_target(t)
# and the normalized CIR_total(t) = (1/M) * sum_j CIR_target_j(t) from a recorded
# `/face_hits` MCAP bag. Works for both the NBV+search and boustrophedon runs.

# %%
from pathlib import Path
import pandas as pd

# Core rosbags imports
from rosbags.highlevel import AnyReader
from rosbags.dataframe import get_dataframe
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

# %% Parameters - set the run to analyze
BAG_NAME = "boustrophedon_cluster_20260626_192732"
M = 4   # total targets in the scene (box_count); undetected targets still count as CIR 0

# %%
# Find the repo root (this file lives in scripts/; the .msg is under src/)
root = Path.cwd()
while not (root / "src" / "uuv_interfaces").exists() and root != root.parent:
    root = root.parent

# Build a Humble typestore and register the custom FaceHits message
typestore = get_typestore(Stores.ROS2_HUMBLE)
msg_path = root / "src" / "uuv_interfaces" / "msg" / "FaceHits.msg"
typestore.register(
    get_types_from_msg(msg_path.read_text(), "uuv_interfaces/msg/FaceHits")
)
print("Registered uuv_interfaces/msg/FaceHits")

# %% Resolve bag directory from BAG_NAME
root = Path.cwd()
while not (root / "data" / "bags").exists() and root != root.parent:
    root = root.parent

bag_dir = root / "data" / "bags" / BAG_NAME
print("Analyzing:", bag_dir)

# %%
with AnyReader([bag_dir], default_typestore=typestore) as reader:
    df_face = get_dataframe(reader, "/face_hits", ["geometry_ids", "primitive_ids"])

# %%
pd.set_option('display.max_columns', None)
df_face.head()

# %%
# explode arrays into individual entries down columns, explicitly name timestamp column
# and create time column to start t = 0
# finally reorder the columns for legibility
hits = df_face.explode(["geometry_ids", "primitive_ids"]).reset_index()
hits = hits.rename(columns={"index": "timestamp"})
hits["t"] = (hits["timestamp"] - hits["timestamp"].min()).dt.total_seconds()

hits = hits[['timestamp', 't', 'geometry_ids', 'primitive_ids']]

# %%
## 2 triangles per face
## integer floor division to get face
# then create (geo_id,face)  mappings
hits['face'] = hits["primitive_ids"] // 2
hits["target_face_mapping"] = list(zip(hits["geometry_ids"], hits["face"]))

# %% [markdown]
# Deduplicate (timestamp, geometry_id, face) triples: each box face and each 1 m
# seabed grid cell is split into 2 triangles (the seabed's diagonal split gives
# a shared edge of length sqrt(2) m), so one sonar return can hit both triangles
# in the same frame. Collapsing on target_face_mapping = (geometry_ids, face)
# counts that as one observation of the face, not two, matching the "i-th hit
# on face f" semantics of the CIR reward (each frame advances k_f by at most 1,
# regardless of triangle count).

# %%
# deduplicate the pairs that share a timestamp
# as we used the pairs to find duplicates, we can drop that column
hits = hits.drop_duplicates(subset=["timestamp", "target_face_mapping"])
hits = hits.drop(columns="target_face_mapping")


# %%
# the mask which finds seabed entries. anything that is not a seabed
# must be a target
selection_mask = hits["geometry_ids"] < 1
df_seabed  = hits[selection_mask].copy()
df_targets = hits[~selection_mask].copy()

# --- FILTER OUT BOTTOM FACE ARTIFACTS ---
# Face 5 is the occluded bottom face resting on the seabed. 
# We remove it so it does not falsely inflate the CIR metric.
df_targets = df_targets[df_targets["face"] != 5]
# ----------------------------------------
# %%
# Group rows by (geometry_ids, face): pandas builds one composite key per row
# from these two columns, and rows sharing a key fall into the same bucket.
#
# a bucket meaning all row entries under this key
# cumcount() then numbers each bucket's rows 0, 1, 2, ... in time order, giving
# k_f - 1 (the hit index minus one) for that target's face, independently of
# every other (target, face) bucket. This is the exponent in the CIR reward
# (1/2)^(k_f - 1): the first hit on a face gets exponent 0 (reward 1), the
# second gets exponent 1 (reward 0.5), and so on.
df_targets["target_prior_hits_on_face_f"] = df_targets.groupby(["geometry_ids", "face"]).cumcount()

# %%
# score applied per face per target over time.
df_targets['instance_score_per_target_on_face_f'] = (1/2) ** df_targets['target_prior_hits_on_face_f']
# information_target is the sum of scores of all faces per target (geo_ID)
df_targets['cumulative_Information_target'] = df_targets.groupby("geometry_ids")["instance_score_per_target_on_face_f"].cumsum()

N = 5  # cuboid with 5 visible faces as the base is sitting on the seabed
# CIR = cumulative information ratio for target, as each face has a convergent series
# which gives a score of 2. so score 2*5 (2*N) generally per target
df_targets["CIR_target"] = df_targets["cumulative_Information_target"] / (2 * N)

# %%
cir_timeseries = df_targets.pivot_table(index="t", columns="geometry_ids",
                                        values="CIR_target", aggfunc="last")

# Bracket the per-target series to the FULL mission, t=0 -> simulation shutoff.
# The pivot comes from df_targets, so on its own it only spans [first hit, last hit].
# The seabed is hit every frame, so hits["t"] gives the true mission endpoints:
#   t=0      -> all targets 0, held until each target's first hit
#   t=T_end  -> hold every target's final plateau out to shutoff. Needed for the
#              boustrophedon: it does not stop when targets are found, it finishes
#              the whole sweep, so that flat tail is real mission time on the axis.
t_start, t_end = hits["t"].min(), hits["t"].max()
if t_start < cir_timeseries.index.min():
    cir_timeseries.loc[t_start] = 0.0            # origin
if t_end > cir_timeseries.index.max():
    cir_timeseries.loc[t_end] = float("nan")     # shutoff placeholder, carried by ffill below

# every scene target gets a column (1..M); a never-hit pot stays a flat-0 line
cir_timeseries = (cir_timeseries
                  .reindex(columns=range(1, M + 1))
                  .sort_index().ffill().fillna(0))  # 0 before first hit; step-hold (incl. flat tail) after

# %%
# CIR_total = (1/M) * sum_j CIR_target_j, evaluated at every t (undetected -> 0).
# Sum only the 1..M target columns explicitly (not axis=1 over the whole frame) so
# re-running this cell a second time can't fold CIR_total back into its own sum.
cir_timeseries["CIR_total"] = cir_timeseries[list(range(1, M + 1))].sum(axis=1) / M

# %%
cir_timeseries

# %%
import matplotlib.pyplot as plt

plt.figure(figsize=(20, 10))
for col in cir_timeseries.columns:
    style = dict(linewidth=2.5, color="black") if col == "CIR_total" else dict(linewidth=1)
    plt.plot(cir_timeseries.index, cir_timeseries[col],
             drawstyle="steps-post", label=str(col), **style)

plt.xlabel("t (s)"); plt.ylabel(""); plt.xlim(left=0); plt.ylim(-0.02, 1.02)
plt.legend(title="per target & total"); plt.grid(alpha=0.3)

# save keyed on BAG_NAME so different runs don't clobber each other's plot
plots_dir = root / "data" / "plots"
plots_dir.mkdir(parents=True, exist_ok=True)
plt.savefig(plots_dir / f"cir_{BAG_NAME}.png", dpi=150, bbox_inches="tight")
print("Saved:", plots_dir / f"cir_{BAG_NAME}.png")

plt.show()


# %% [markdown]
# # Diagnostic: Unique faces detected per target
# Check if bottom face (5) or any unexpected faces were hit.

# %%
print("\n--- Diagnostic: Unique faces hit per geometry_id ---")

# Group by geometry_ids and get the unique face IDs as an array
detected_faces_per_target = df_targets.groupby("geometry_ids")["face"].unique()

# Print them out clearly sorted
for geom_id, faces in detected_faces_per_target.items():
    print(f"Target ID: {geom_id} | Faces Hit: {sorted(faces)}")
    
    # Optional: flag if the bottom face was hit
    if 5 in faces:
        print(f"  -> [WARNING] Target {geom_id} registered a hit on bottom Face 5!")

print("----------------------------------------------------\n")

# %% [markdown]
# # Position Heatmap (map frame) - VECTORIZED
# Extracts bare translations and uses vectorized NumPy operations
# for massive speedups on large-scale datasets.

# %%
import numpy as np
import matplotlib.pyplot as plt
import tf_transformations

map_to_ned_matrix = np.eye(4)
raw_ned_positions = []

print("\n--- Extracting TF data for Heatmap ---")

with AnyReader([bag_dir], default_typestore=typestore) as reader:
    # 1. Fetch static map -> ned transform (Only happens once)
    static_conns = [c for c in reader.connections if c.topic == '/tf_static']
    for conn, timestamp, rawdata in reader.messages(connections=static_conns):
        msg = reader.deserialize(rawdata, conn.msgtype)
        for t in msg.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'ned':
                q = t.transform.rotation
                trans = t.transform.translation
                map_to_ned_matrix = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
                map_to_ned_matrix[:3, 3] = [trans.x, trans.y, trans.z]

    # 2. Fetch dynamic ned -> base_link transforms
    tf_conns = [c for c in reader.connections if c.topic == '/tf']
    for conn, timestamp, rawdata in reader.messages(connections=tf_conns):
        msg = reader.deserialize(rawdata, conn.msgtype)
        for t in msg.transforms:
            if t.header.frame_id == 'ned' and t.child_frame_id == 'base_link':
                trans = t.transform.translation
                # OPTIMIZATION: Ignore quaternions entirely.
                # Store as homogeneous coordinate [X, Y, Z, 1.0] for matrix math
                raw_ned_positions.append([trans.x, trans.y, trans.z, 1.0])

print(f"Extracted {len(raw_ned_positions)} position samples.")

# %%
# --- THE MASSIVE OPTIMIZATION (Vectorized Math) ---

# Convert the python list into an Nx4 NumPy array, then transpose to 4xN
# Shape: 4 rows (X, Y, Z, 1), N columns (time steps)
ned_points = np.array(raw_ned_positions).T 

# Multiply the 4x4 static map matrix by all N positions simultaneously!
# This runs in C, taking milliseconds instead of minutes.
map_points = map_to_ned_matrix @ ned_points 

# Extract the X and Y rows
x_coords = map_points[0, :]
y_coords = map_points[1, :]

# --- PLOTTING ---
plt.figure(figsize=(12, 10))

# Create a hexbin heatmap
hb = plt.hexbin(x_coords, y_coords, gridsize=50, cmap='inferno', mincnt=1)

cb = plt.colorbar(hb)
cb.set_label('Observation Count (Relative Time Spent)')

plt.xlabel("X Position (m) [Map Frame]")
plt.ylabel("Y Position (m) [Map Frame]")
plt.title(f"2D Position Heatmap: {BAG_NAME}")
plt.axis('equal') 
plt.grid(alpha=0.3)

plt.savefig(plots_dir / f"heatmap_{BAG_NAME}.png", dpi=150, bbox_inches="tight")
print("Saved:", plots_dir / f"heatmap_{BAG_NAME}.png")

plt.show()
# %%
