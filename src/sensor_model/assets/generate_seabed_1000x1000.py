#!/usr/bin/env python3
"""Generate the full-scale 1000x1000 m procedural seabed as an OBJ file.

Derived from generate_seabed.py (the 50x50 scene). Key differences:
  - 1000x1000 m at 2 m resolution (501x501 verts, 500k triangles). Coarser than
    the 50x50's 1 m grid so Open3D's BVH build + per-ping raycast stay light at
    400x the area (raycast cost is ~log in triangle count, so 2 m is plenty).
  - Rolling background is preserved but CONTROLLED: broad hills summed then
    NORMALIZED to +-BG_AMPLITUDE, so overlapping Gaussians can't spike into the
    big mounds/basins that eat the vehicle's 15 m clearance or swamp the divots.
  - Each island keeps only a fraction of the local roll (so it reads as a hollow
    nested in rolling terrain, not a flat disc), then a deliberate divot is
    carved. The flat control pot (E) keeps 0% roll -> genuinely flat. This makes
    every cluster a reliable depression without killing the rolling character.

Run:  python3 generate_seabed_1000x1000.py   ->  writes seabed_1000x1000m.obj
"""

import os
import numpy as np

# ── Global terrain ──────────────────────────────────────────────────────────
SIZE         = 1000     # metres (square)
RESOLUTION   = 2.0      # metres between vertices (501x501 grid, 500k triangles)
BG_AMPLITUDE = 2.5      # metres: background NORMALIZED to +-this peak (rolls,
                        #         but bounded -> mounds <= 2.5 m, clearance safe)
NUM_HILLS    = 8        # broad undulations (normalized afterwards, so count/amp
HILL_SIGMA   = 150.0    # only shape the roll; the peak is set by BG_AMPLITUDE)
ROUGHNESS    = 0.12     # metres, small-scale seabed texture
NUM_ROUGH    = 12       # roughness sine components
SEED         = 42
OUTPUT       = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "seabed_1000x1000m.obj")

# ── Islands: (cx, cy, divot_depth_m, divot_sigma_m, roll_sigma_m, keep_roll) ─
# keep_roll = fraction of background roll retained at the island (0 -> flat).
# divot_sigma ~ the island's circumradius so pots sit on the gentle slope.
# roll_sigma (~2x divot_sigma) sets how far the roll-suppression reaches.
# Divots are deep enough that the retained roll can never turn a hollow into a
# bump. E has depth 0 and keep_roll 0 -> the flat occlusion control.
ISLANDS = [
    (300.0, 300.0, 2.5, 18.0, 40.0, 0.35),   # A  tight-3     (footprint R ~14.4 m)
    (700.0, 700.0, 2.5, 30.0, 65.0, 0.35),   # B  spread-3    (footprint R ~28.9 m)
    (500.0, 500.0, 2.5, 25.0, 55.0, 0.35),   # C  collinear-2 (footprint R ~20 m)
    (100.0, 900.0, 3.5, 10.0, 24.0, 0.35),   # D  lone (deeper/steeper -> hardest)
    (900.0, 100.0, 0.0,  0.0, 30.0, 0.00),   # E  lone FLAT (occlusion control)
]
# ────────────────────────────────────────────────────────────────────────────

rng = np.random.default_rng(SEED)

n = int(SIZE / RESOLUTION) + 1
xs = np.linspace(0, SIZE, n)
ys = np.linspace(0, SIZE, n)
xx, yy = np.meshgrid(xs, ys)


def gauss(cx, cy, sigma):
    return np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2 * sigma ** 2))


# 1) Broad rolling background, then NORMALIZE to +-BG_AMPLITUDE. Normalizing
#    (not per-hill amplitude) is what keeps overlapping hills from summing into
#    clearance-eating mounds while preserving the rolling shape.
bg = np.zeros((n, n))
for _ in range(NUM_HILLS):
    cx = rng.uniform(60, SIZE - 60)
    cy = rng.uniform(60, SIZE - 60)
    a  = rng.uniform(0.5, 1.0) * rng.choice([-1, 1])
    bg += a * gauss(cx, cy, HILL_SIGMA)
peak = np.max(np.abs(bg))
if peak > 0:
    bg *= BG_AMPLITUDE / peak

# 2) Retain only keep_roll of the background at each island (gradual fade), so a
#    cluster reads as a hollow within rolling terrain and E stays flat.
for cx, cy, depth, dsig, rsig, keep in ISLANDS:
    bg *= (1.0 - (1.0 - keep) * gauss(cx, cy, rsig))

zz = bg

# 3) Carve the deliberate island depressions (debris-collection hollows).
for cx, cy, depth, dsig, rsig, keep in ISLANDS:
    if depth > 0:
        zz += -depth * gauss(cx, cy, dsig)

# 4) Light small-scale roughness (texture) everywhere.
for _ in range(NUM_ROUGH):
    fx    = rng.uniform(0.02, 0.10)
    fy    = rng.uniform(0.02, 0.10)
    phase = rng.uniform(0, 2 * np.pi)
    amp   = rng.uniform(0.3, 1.0) * ROUGHNESS
    zz   += amp * np.sin(2 * np.pi * fx * xx + 2 * np.pi * fy * yy + phase)

# Write OBJ (row-major grid, two triangles per cell)
with open(OUTPUT, "w") as f:
    f.write(f"# Procedural seabed {SIZE}x{SIZE}m @ {RESOLUTION}m res\n")
    for j in range(n):
        for i in range(n):
            f.write(f"v {xx[j,i]:.3f} {yy[j,i]:.3f} {zz[j,i]:.3f}\n")
    for j in range(n - 1):
        for i in range(n - 1):
            v00 = j * n + i + 1
            v10 = j * n + (i + 1) + 1
            v01 = (j + 1) * n + i + 1
            v11 = (j + 1) * n + (i + 1) + 1
            f.write(f"f {v00} {v10} {v11}\n")
            f.write(f"f {v00} {v11} {v01}\n")

print(f"Written {OUTPUT}  ({n}x{n} vertices, {2*(n-1)**2} triangles)")
