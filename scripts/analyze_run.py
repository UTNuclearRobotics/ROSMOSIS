# %% [markdown]
# # CIR run analysis
# Percent-format script (run cell-by-cell in VSCode's interactive window, or as a
# plain `.py`). Computes the per-target Cumulative Information Ratio CIR_target(t)
# and the normalized CIR_total(t) = (1/M) * sum_j CIR_target_j(t) from a recorded
# `/face_hits` MCAP bag. Works for both the NBV+search and boustrophedon runs.
#
# Two sections:
#   1. SINGLE-RUN  -- set BAG_NAME + M, get the per-target CIR plot, face
#      diagnostic, and position heatmap for one run (the standalone plots).
#   2. COMPARE     -- list several runs, overlay their CIR_total on one axis and
#      print/save a summary table. Reuses the exact same functions, so the
#      overlay can never diverge from the single-run plots. Optionally also
#      re-emits each run's standalone plots (PER_RUN_PLOTS).

# %%
import re
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from rosbags.highlevel import AnyReader
from rosbags.dataframe import get_dataframe
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

# %% [markdown]
# ## Setup: repo root, typestore, output dir (shared by both sections)

# %%
# Find the workspace root (this file lives in scripts/; the .msg is under src/,
# data/bags/ and data/plots/ sit beside src/).
root = Path.cwd()
while not (root / "src" / "uuv_interfaces").exists() and root != root.parent:
    root = root.parent

# Build a Humble typestore and register the custom FaceHits message.
typestore = get_typestore(Stores.ROS2_HUMBLE)
msg_path = root / "src" / "uuv_interfaces" / "msg" / "FaceHits.msg"
typestore.register(
    get_types_from_msg(msg_path.read_text(), "uuv_interfaces/msg/FaceHits")
)
print("Registered uuv_interfaces/msg/FaceHits")

plots_dir = root / "data" / "plots"
plots_dir.mkdir(parents=True, exist_ok=True)

N_FACES = 5  # cuboid visible faces (bottom face 5 rests on the seabed -> excluded)


# %% [markdown]
# ## Functions (single source of truth for the CIR metric + plots)

# %%
def clean_name(bag_name):
    """Strip the trailing _YYYYMMDD_HHMMSS timestamp for a cleaner plot title."""
    return re.sub(r"_\d{8}_\d{6}$", "", bag_name)


def resolve_bag(prefix, require_finalized=True):
    """Resolve a run reference to a bag directory under data/bags/.

    Exact directory name -> used as-is. Otherwise treated as a PREFIX and the
    NEWEST matching bag (by mtime) is returned. With require_finalized, only
    bags that have metadata.yaml (recording closed cleanly) are considered, so a
    crashed or still-running bag is never picked up. Returns a Path or None.
    """
    bags = root / "data" / "bags"
    exact = bags / prefix
    if exact.is_dir():
        return exact
    matches = [p for p in bags.glob(f"{prefix}*") if p.is_dir()]
    if require_finalized:
        matches = [p for p in matches if (p / "metadata.yaml").exists()]
    if not matches:
        return None
    return max(matches, key=lambda p: p.stat().st_mtime)


def compute_cir_timeseries(bag_dir, M, n_faces=N_FACES):
    """Per-target CIR_target(t) + CIR_total(t) from a run's /face_hits bag.

    Returns (cir_timeseries, df_targets):
      cir_timeseries -- indexed by mission time t, columns 1..M (per target)
                        plus 'CIR_total'; bracketed to [0, t_end].
      df_targets     -- the exploded/scored per-hit target frame (for the
                        face diagnostic).

    Method (the face-hit geometric-series reward):
      - explode (geometry_ids, primitive_ids) arrays to one hit per row
      - face = primitive_id // 2 (2 triangles/face); dedupe (timestamp, geo,
        face) so one ping hitting both triangles of a face counts once
      - drop seabed (geometry_id < 1) and the occluded bottom face (5)
      - k-th hit on a face scores (1/2)^(k-1); sum over faces -> per-target
        cumulative information; each face's series converges to 2, so the
        per-target max is 2 * n_faces
      - CIR_target = info / (2 * n_faces); CIR_total = mean over M targets
      - bracket to [0, t_end] so pre-detection reads 0 and the final plateau
        (incl. boustrophedon's flat post-detection tail out to shutoff) holds
    """
    with AnyReader([bag_dir], default_typestore=typestore) as reader:
        df_face = get_dataframe(reader, "/face_hits", ["geometry_ids", "primitive_ids"])

    hits = df_face.explode(["geometry_ids", "primitive_ids"]).reset_index()
    hits = hits.rename(columns={"index": "timestamp"})
    hits["t"] = (hits["timestamp"] - hits["timestamp"].min()).dt.total_seconds()
    hits = hits[["timestamp", "t", "geometry_ids", "primitive_ids"]]

    hits["face"] = hits["primitive_ids"] // 2
    hits["target_face_mapping"] = list(zip(hits["geometry_ids"], hits["face"]))
    hits = hits.drop_duplicates(subset=["timestamp", "target_face_mapping"])
    hits = hits.drop(columns="target_face_mapping")

    df_targets = hits[hits["geometry_ids"] >= 1].copy()   # >=1 -> target, not seabed
    df_targets = df_targets[df_targets["face"] != 5]       # drop occluded bottom face

    # cumcount() numbers each (target, face) bucket 0,1,2,... in time = k_f - 1,
    # the exponent in the (1/2)^(k-1) reward.
    df_targets["k_minus_1"] = df_targets.groupby(["geometry_ids", "face"]).cumcount()
    df_targets["face_score"] = (1 / 2) ** df_targets["k_minus_1"]
    df_targets["info_target"] = df_targets.groupby("geometry_ids")["face_score"].cumsum()
    df_targets["CIR_target"] = df_targets["info_target"] / (2 * n_faces)

    cir = df_targets.pivot_table(index="t", columns="geometry_ids",
                                 values="CIR_target", aggfunc="last")

    # Bracket to the full mission: t=0 (all 0) and t_end (hold final plateau).
    # The seabed is hit every frame, so hits["t"] gives true mission endpoints.
    t_start, t_end = hits["t"].min(), hits["t"].max()
    if cir.empty or t_start < cir.index.min():
        cir.loc[t_start] = 0.0
    if t_end > cir.index.max():
        cir.loc[t_end] = float("nan")   # shutoff placeholder, carried by ffill

    cir = (cir.reindex(columns=range(1, M + 1))
              .sort_index().ffill().fillna(0))   # 0 before first hit; step-hold after
    cir["CIR_total"] = cir[list(range(1, M + 1))].sum(axis=1) / M
    return cir, df_targets


def time_to_threshold(cir_total, thr):
    """First mission time t where CIR_total >= thr, or NaN if never reached."""
    reached = cir_total.index[cir_total.values >= thr]
    return float(reached[0]) if len(reached) else float("nan")


def plot_cir(cir, bag_name, save=True):
    """Standalone per-target + CIR_total plot for one run (the original figure)."""
    plt.figure(figsize=(20, 10))
    for col in cir.columns:
        if col == "CIR_total":
            style = dict(linewidth=2.5, color="black")
            label = "CIR$_{total}$ (mission)"
        else:
            style = dict(linewidth=1)
            label = f"Target {col}"
        plt.plot(cir.index, cir[col], drawstyle="steps-post", label=label, **style)
    plt.xlabel("Mission time t (s)")
    plt.ylabel("Cumulative Information Ratio (CIR)")
    plt.title(f"Cumulative Information Ratio vs. Mission Time\n{clean_name(bag_name)}")
    plt.xlim(left=0); plt.ylim(-0.02, 1.02)
    plt.legend(title="Per-target & mission total"); plt.grid(alpha=0.3)
    if save:
        out = plots_dir / f"cir_{bag_name}.png"
        plt.savefig(out, dpi=150, bbox_inches="tight")
        print("Saved:", out)
    plt.show()


def face_diagnostic(df_targets):
    """Print unique faces hit per target; flag any bottom-face (5) leakage."""
    print("\n--- Diagnostic: Unique faces hit per geometry_id ---")
    for geom_id, faces in df_targets.groupby("geometry_ids")["face"].unique().items():
        print(f"Target ID: {geom_id} | Faces Hit: {sorted(faces)}")
        if 5 in faces:
            print(f"  -> [WARNING] Target {geom_id} registered a hit on bottom Face 5!")
    print("----------------------------------------------------\n")


def _quaternion_matrix(q_xyzw):
    """4x4 homogeneous rotation from a quaternion [x, y, z, w] (ROS-free)."""
    x, y, z, w = q_xyzw
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.identity(4)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz,         xz + wy,         0.0],
        [xy + wz,         1.0 - (xx + zz), yz - wx,         0.0],
        [xz - wy,         yz + wx,         1.0 - (xx + yy), 0.0],
        [0.0,             0.0,             0.0,             1.0],
    ])


def extract_xy_map(bag_dir):
    """Vehicle (x, y) track in the map frame, vectorized from /tf + /tf_static."""
    map_to_ned = np.eye(4)
    raw = []
    with AnyReader([bag_dir], default_typestore=typestore) as reader:
        static_conns = [c for c in reader.connections if c.topic == "/tf_static"]
        for conn, _, rawdata in reader.messages(connections=static_conns):
            msg = reader.deserialize(rawdata, conn.msgtype)
            for t in msg.transforms:
                if t.header.frame_id == "map" and t.child_frame_id == "ned":
                    q, tr = t.transform.rotation, t.transform.translation
                    map_to_ned = _quaternion_matrix([q.x, q.y, q.z, q.w])
                    map_to_ned[:3, 3] = [tr.x, tr.y, tr.z]
        tf_conns = [c for c in reader.connections if c.topic == "/tf"]
        for conn, _, rawdata in reader.messages(connections=tf_conns):
            msg = reader.deserialize(rawdata, conn.msgtype)
            for t in msg.transforms:
                if t.header.frame_id == "ned" and t.child_frame_id == "base_link":
                    tr = t.transform.translation
                    raw.append([tr.x, tr.y, tr.z, 1.0])
    pts = map_to_ned @ np.array(raw).T          # 4xN, all samples at once
    return pts[0, :], pts[1, :]


def plot_heatmap(x, y, bag_name, arena=1000.0, gridsize=50, save=True, margin_frac=0.05):
    """Fixed-cell-size hexbin dwell map. The hex WIDTH is fixed at arena/gridsize
    (so counts stay comparable across runs of the same scene), but the extent
    FOLLOWS the actual trajectory (+margin) so out-of-bounds coverage is not
    clipped -- e.g. the boustrophedon's running-start overshoot and the edge lane
    beyond the ROI are real coverage and must show. To keep the width fixed while
    the extent varies, gridsize is passed as (nx, ny) sized from the data range.
    Count ~ dwell time ~ path length (constant speed)."""
    hex_width = arena / gridsize                       # target cell width (m)
    xmin, xmax, ymin, ymax = float(x.min()), float(x.max()), float(y.min()), float(y.max())
    mx = (xmax - xmin) * margin_frac + hex_width       # whitespace so overshoot is legible
    my = (ymax - ymin) * margin_frac + hex_width
    ext = (xmin - mx, xmax + mx, ymin - my, ymax + my)
    nx = max(1, round((ext[1] - ext[0]) / hex_width))
    ny = max(1, round((ext[3] - ext[2]) / hex_width))

    plt.figure(figsize=(12, 10))
    hb = plt.hexbin(x, y, gridsize=(nx, ny), extent=ext, cmap="inferno", mincnt=1)
    plt.colorbar(hb).set_label("Observation Count (Relative Time Spent)")
    plt.xlabel("X Position (m) [Map Frame]"); plt.ylabel("Y Position (m) [Map Frame]")
    plt.title(f"2D Position Heatmap: {clean_name(bag_name)}\n(hex bin width = {hex_width:.0f} m)")
    plt.axis("equal"); plt.grid(alpha=0.3)
    if save:
        out = plots_dir / f"heatmap_{bag_name}.png"
        plt.savefig(out, dpi=150, bbox_inches="tight")
        print("Saved:", out)
    plt.show()


# %% [markdown]
# # 1. SINGLE-RUN analysis  (DISABLED -- uncomment the cell below to use)
# Inspect one run in detail: per-target CIR plot, face diagnostic, and position
# heatmap. Set BAG_NAME (run folder under data/bags/, or a prefix), M (scene
# box_count = CIR_total denominator), and ARENA (scene size, m). Off by default
# so running the file goes straight to the COMPARE section.

# %%
# --- SINGLE-RUN (commented out) ---------------------------------------------
# BAG_NAME = "nbv_cone_alpha0.5_50x50"   # run to analyze (name or prefix)
# M = 4                                  # scene box_count
# ARENA = 50.0                           # scene size for the heatmap extent (m)
#
# bag_dir = resolve_bag(BAG_NAME)
# if bag_dir is None:
#     raise FileNotFoundError(f"No finalized bag matching '{BAG_NAME}' under data/bags/.")
# print("Analyzing:", bag_dir)
#
# cir, df_targets = compute_cir_timeseries(bag_dir, M)   # CIR timeseries
# plot_cir(cir, bag_dir.name)                            # standalone CIR plot
# face_diagnostic(df_targets)                            # unique faces per target
# x_coords, y_coords = extract_xy_map(bag_dir)           # position heatmap
# plot_heatmap(x_coords, y_coords, bag_dir.name, arena=ARENA)
# ----------------------------------------------------------------------------


# %% [markdown]
# # 2. COMPARE runs
# Overlay CIR_total(t) for a curated set of runs + a summary table. Each run is a
# (label, prefix) pair: the label is what appears in the legend/table, the prefix
# resolves to the newest finalized bag matching it (so no timestamp-pasting, and
# aborted runs are skipped). Set M_COMPARE to the scene's box_count.
#
# >>> PLACEHOLDERS: the prefixes below are dummies so they DON'T resolve -- every
#     run is skipped until you replace them. After the full-scale sweep finishes,
#     swap in the real prefixes (from run_CINBV_experiment.sh /
#     run_boustrophedon_experiment.sh): boustrophedon_fullscale and
#     nbv_<sampler>_alpha{0,0.25,0.5,0.75,1.0}_fullscale.

# %%
M_COMPARE = 10           # full-scale scene box_count
ARENA_COMPARE = 1000.0   # full-scale scene size for per-run heatmaps
PER_RUN_PLOTS = True     # also emit each run's standalone CIR + heatmap plots
CIR_LEVELS = [0.9]       # time-to-CIR thresholds (1.0 is asymptotic -> use Final CIR)

# Full-scale CONE sweep + boustrophedon baseline. For the helix sweep, swap
# "cone" -> "helix" in the labels and prefixes (bags: nbv_helix_alpha<val>_fullscale).
RUNS = [
    # (label, prefix)
    ("Boustrophedon",   "boustrophedon_fullscale"),
    ("NBV cone α=0.0",  "nbv_cone_alpha0_fullscale"),
    ("NBV cone α=0.25", "nbv_cone_alpha0.25_fullscale"),
    ("NBV cone α=0.5",  "nbv_cone_alpha0.5_fullscale"),
    ("NBV cone α=0.75", "nbv_cone_alpha0.75_fullscale"),
    ("NBV cone α=1.0",  "nbv_cone_alpha1.0_fullscale"),
]

# %% Compute every run once; collect CIR_total curves + summary stats.
overlay = []   # (label, cir_total_series)
rows = []      # summary-table records
for label, prefix in RUNS:
    bdir = resolve_bag(prefix)
    if bdir is None:
        print(f"[skip] no finalized bag for '{prefix}'")
        continue
    cir_r, df_t_r = compute_cir_timeseries(bdir, M_COMPARE)
    ct = cir_r["CIR_total"]
    overlay.append((label, ct))
    cmax = float(ct.max())                       # highest CIR (= final; CIR is monotonic)
    rows.append({
        "Run": label,
        "Highest CIR_total": f"{cmax:.4f}",           # fixed 4 dp (keeps trailing zeros)
        "t @ highest CIR (s)": round(time_to_threshold(ct, cmax), 1),  # when plateau reached
        "End time (s)": round(float(ct.index[-1]), 1),                 # mission end / total duration
        "bag": bdir.name,
    })
    if PER_RUN_PLOTS:
        plot_cir(cir_r, bdir.name)
        xr, yr = extract_xy_map(bdir)
        plot_heatmap(xr, yr, bdir.name, arena=ARENA_COMPARE)

# %% Overlay plot: CIR_total(t) for all runs on one axis.
plt.figure(figsize=(14, 8))
for label, ct in overlay:
    plt.plot(ct.index, ct.values, drawstyle="steps-post", linewidth=2, label=label)
plt.xlabel("Mission time t (s)")
plt.ylabel("CIR$_{total}$ (mission)")
plt.title("CIR$_{total}$ vs. Mission Time -- run comparison")
plt.xlim(left=0); plt.ylim(-0.02, 1.02)
plt.legend(title="Run"); plt.grid(alpha=0.3)
out = plots_dir / "compare_cir_total.png"
plt.savefig(out, dpi=150, bbox_inches="tight")
print("Saved:", out)
plt.show()

# %% Summary table: print + save CSV.
table = pd.DataFrame(rows)
pd.set_option("display.max_columns", None)
pd.set_option("display.width", 200)
print("\n", table.to_string(index=False), "\n")
csv_out = plots_dir / "compare_summary.csv"
table.to_csv(csv_out, index=False)
print("Saved:", csv_out)
# %%
