#%% [markdown]
# inspect_boxes.py
# Sample the seabed height at each lobster-pot (x, y) and render a labeled
# plan-view of the 1000x1000 m arena. Produces the (x, y, z) tuples for the env
# yaml and a thesis-ready figure (islands marked, spacings implied by layout).
#
# z reported is the SEABED SURFACE height at the pot. The env yaml's box_position
# z is the box CENTRE, so it = surface + box_half_height (0.25 m for box_size
# z=0.5) if the sim uses centre-convention -- both are printed below.

#%%
import os
import numpy as np
import matplotlib.pyplot as plt

MESH       = "/home/talal/projects/rosmosis_ws/src/sensor_model/assets/seabed_1000x1000m.obj"
N, RES     = 501, 2.0        # grid dims / resolution the generator used
SIZE       = 1000
BOX_HALF_H = 0.25            # box_size z = 0.5 -> half height (centre-rest offset)
OUT_PNG    = "/home/talal/projects/rosmosis_ws/data/plots/pot_layout_1000x1000.png"

# island -> (label, marker, colour) ; pots grouped by island
ISLANDS = {
    "A": ("tight-3",    "o", "red"),
    "B": ("spread-3",   "s", "orange"),
    "C": ("collinear-2","^", "magenta"),
    "D": ("lone (divot)","D", "white"),
    "E": ("lone (flat)", "D", "cyan"),
}
# (box, island, x, y)
POTS = [
    ("box_0", "A", 288, 293), ("box_1", "A", 312, 293), ("box_2", "A", 300, 314),
    ("box_3", "B", 675, 686), ("box_4", "B", 725, 686), ("box_5", "B", 700, 729),
    ("box_6", "C", 480, 500), ("box_7", "C", 520, 500),
    ("box_8", "D", 100, 900),
    ("box_9", "E", 900, 100),
]
# where to anchor each island's text box (offset from its centroid, in map m)
LABEL_ANCHOR = {
    "A": (150, 150), "B": (830, 830), "C": (330, 560),
    "D": (230, 820), "E": (720, 210),
}

#%% load the mesh into a Z grid (Z[j, i] with x = i*RES, y = j*RES)
zs = []
with open(MESH) as f:
    for line in f:
        if line.startswith("v "):
            zs.append(float(line.split()[3]))
Z = np.asarray(zs, dtype=float).reshape(N, N)

def surface_z(x, y):
    """Bilinear seabed height at (x, y)."""
    fi, fj = x / RES, y / RES
    i0, j0 = int(np.floor(fi)), int(np.floor(fj))
    i1, j1 = min(i0 + 1, N - 1), min(j0 + 1, N - 1)
    di, dj = fi - i0, fj - j0
    return (Z[j0, i0] * (1 - di) * (1 - dj) + Z[j0, i1] * di * (1 - dj)
            + Z[j1, i0] * (1 - di) * dj + Z[j1, i1] * di * dj)

#%% compute z per pot and print the table
print(f"{'box':7s} {'isl':3s} {'x':>4s} {'y':>4s} {'surface_z':>10s} {'box_z(+0.25)':>13s}")
recs = []
for box, isl, x, y in POTS:
    sz = surface_z(x, y)
    bz = sz + BOX_HALF_H
    recs.append((box, isl, x, y, sz, bz))
    print(f"{box:7s} {isl:3s} {x:4d} {y:4d} {sz:10.3f} {bz:13.3f}")

#%% thesis figure: elevation heatmap + pot markers + per-island (x,y,z) tuples
fig, ax = plt.subplots(figsize=(11, 9))
im = ax.imshow(Z, origin="lower", extent=[0, SIZE, 0, SIZE], cmap="viridis", aspect="equal")
fig.colorbar(im, ax=ax, label="Seabed elevation (m)")

drawn = set()
for box, isl, x, y, sz, bz in recs:
    lbl, mk, col = ISLANDS[isl]
    legend_lbl = f"{isl}: {lbl}" if isl not in drawn else None
    drawn.add(isl)
    ax.scatter(x, y, marker=mk, s=110, c=col, edgecolors="black",
               linewidths=1.3, zorder=4, label=legend_lbl)

# per-island text box listing member (x, y, z) tuples, with a leader line
for isl, (lbl, mk, col) in ISLANDS.items():
    members = [(x, y, sz) for (b, i, x, y, sz, bz) in recs if i == isl]
    cx = np.mean([m[0] for m in members]); cy = np.mean([m[1] for m in members])
    txt = f"{isl}  ({lbl})\n" + "\n".join(f"({x}, {y}, {sz:.2f})" for x, y, sz in members)
    ax.annotate(txt, xy=(cx, cy), xytext=LABEL_ANCHOR[isl],
                fontsize=8, ha="left", va="center",
                bbox=dict(boxstyle="round,pad=0.3", fc="white", ec=col, lw=1.4, alpha=0.9),
                arrowprops=dict(arrowstyle="->", color=col, lw=1.3))

ax.set_xlabel("X (m)   [map frame, origin at bottom-left corner]")
ax.set_ylabel("Y (m)")
ax.set_title("Lobster-pot layout on the 1000×1000 m seabed\n"
             "markers = pots; boxes list (x, y, seabed z);  islands ≥200 m apart")
ax.set_xlim(0, SIZE); ax.set_ylim(0, SIZE)
ax.legend(title="Island (scenario)", loc="upper center", bbox_to_anchor=(0.5, -0.09),
          ncol=5, framealpha=0.95)
ax.grid(alpha=0.2)
plt.tight_layout()

os.makedirs(os.path.dirname(OUT_PNG), exist_ok=True)
plt.savefig(OUT_PNG, dpi=150, bbox_inches="tight")
print("Saved:", OUT_PNG)
plt.show()
