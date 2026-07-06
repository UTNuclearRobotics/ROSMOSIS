#!/usr/bin/env python3
"""Generate the full-scale 1000x1000 m procedural seabed as an OBJ file.

Scaled up from generate_seabed.py (the 50x50 scene) and given a fractal
roughness layer. The heightfield is built in four ADDITIVE stages on a 501x501
grid (2 m resolution -> 500k triangles; coarse enough that Open3D's BVH build +
per-ping raycast stay light at 400x the area, raycast cost being ~log in
triangle count):

  1. BROAD ROLLING BACKGROUND -- NUM_HILLS wide Gaussians (HILL_SIGMA) are summed
     and then the whole field is NORMALIZED to +-BG_AMPLITUDE. Normalizing the
     sum (instead of capping each hill) stops overlapping Gaussians from spiking
     into mounds/basins that would eat the vehicle's 15 m clearance or swamp the
     divots, while preserving the rolling shape. Long-wavelength, gentle (~1deg).

  2. ISLAND ROLL-SUPPRESSION -- at each island the background is multiplied down
     to `keep_roll` of its local value via a Gaussian fade (roll_sigma). A
     cluster then reads as a hollow nested in rolling terrain rather than a flat
     disc; island E (keep_roll=0) stays genuinely flat as the occlusion control.

  3. DIVOT CARVING -- a Gaussian well (depth, divot_sigma) is subtracted at each
     island, making every cluster a reliable debris-collection depression.
     Depths are set so the retained roll can never turn a hollow into a bump.

  4. FRACTAL (fBm) ROUGHNESS -- summed value-noise octaves (fbm()), scaled so its
     standard deviation is ROUGHNESS metres, added everywhere. Real seafloor
     roughness is fractal / power-law (Fox & Hayes 1985; Goff & Jordan 1988), so
     fBm gives physically-motivated, IRREGULAR texture with no repeating pattern
     -- unlike summed sines, which produce a synthetic diagonal corrugation.

The ISLANDS table is (cx, cy, divot_depth_m, divot_sigma_m, roll_sigma_m,
keep_roll). Pot heights are NOT set here: run inspect_boxes.py after regenerating
to sample surface_z at each pot, then write surface_z + half-box-height into the
env YAML's box_positions (the mine base rests on the seabed).

Run:  python3 generate_seabed_1000x1000.py
      -> writes seabed_1000x1000m.obj, and (if SHOW_PLOT) pops a hillshade view
         for a realism eyeball. Set SHOW_PLOT=False for headless/server regen.
"""

import os
import numpy as np
from scipy.ndimage import zoom

# ── Global terrain ──────────────────────────────────────────────────────────
SIZE         = 1000     # metres (square)
RESOLUTION   = 2.0      # metres between vertices (501x501 grid, 500k triangles)
BG_AMPLITUDE = 2.5      # metres: background NORMALIZED to +-this peak (rolls,
                        #         but bounded -> mounds <= 2.5 m, clearance safe)
NUM_HILLS    = 8        # broad undulations (normalized afterwards, so count/amp
HILL_SIGMA   = 150.0    # only shape the roll; the peak is set by BG_AMPLITUDE)
# ── Fractal (fBm) roughness -- plain-language guide to the 3 knobs ───────────
# The seabed texture is fractal Brownian motion (self-affine, matching real
# seafloor roughness: Fox & Hayes 1985; Goff & Jordan 1988). Picture stacking a
# few layers of bumps, each layer HALF as wide and HALF as tall as the one below:
#
#   ROUGHNESS         = HOW TALL the bumps are (std, metres). The height knob.
#                       Bigger = rougher. Keep < ~0.17 m so the 0.5 m mine stays
#                       TALLER than the roughness peaks (-> mine discriminable).
#                       0.15 -> peaks ~0.45 m, mine (0.5 m) just wins.
#
#   ROUGH_BASE_CELLS  = HOW WIDE the biggest bumps are. Splits the 1000 m into
#                       this many tiles for the coarsest layer, so the largest
#                       bump wavelength = SIZE / base_cells (12 -> ~83 m).
#                       Fewer tiles -> broader swells; more tiles -> smaller bumps.
#
#   ROUGH_OCTAVES     = HOW MANY layers of detail. Each extra octave adds a finer
#                       layer at half the width and half the height of the prior
#                       one. 4 octaves -> detail from ~83 m down to ~10 m. More =
#                       finer grain, but don't exceed ~5 (the 2 m mesh can't show
#                       bumps < ~8 m). Fewer = smoother.
#
# The per-octave "half width, half height" is lacunarity=2 / persistence=0.5 --
# the standard natural-terrain recipe -- hard-coded in the fbm() call below.
ROUGHNESS        = 0.15   # texture HEIGHT   (std, m)            -- see guide above
ROUGH_BASE_CELLS = 12     # biggest-bump WIDTH = SIZE/this (~83 m)
ROUGH_OCTAVES    = 4      # LAYERS of detail (down to ~10 m)
SHOW_PLOT    = True     # True: also pop a hillshade plot AFTER writing the OBJ
                        # (view-only; set False for headless server/Docker regen)
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


def fbm(shape, base_cells, octaves, persistence, lacunarity, rng):
    """Fractal Brownian motion via summed value-noise octaves. Each octave is a
    random coarse lattice cubic-upsampled to the full grid; halving amplitude and
    doubling frequency per octave yields a self-affine (power-law) field -- the
    roughness character of real seafloor (Goff & Jordan 1988), with no repeating
    pattern. Returns a ~unit-std array of `shape`."""
    out, amp, cells, norm = np.zeros(shape), 1.0, base_cells, 0.0
    for _ in range(octaves):
        coarse = rng.standard_normal((cells + 1, cells + 1))
        layer = zoom(coarse, (shape[0] / (cells + 1), shape[1] / (cells + 1)), order=3)
        out += amp * layer[:shape[0], :shape[1]]
        norm += amp
        amp *= persistence
        cells = int(cells * lacunarity)
    out /= norm
    return out / (out.std() + 1e-9)


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

# 4) Fractal (fBm) roughness everywhere -- irregular, natural texture with a
#    power-law spectrum matching real seafloor, instead of a repeating sine
#    corrugation. Scaled so its standard deviation is ROUGHNESS metres.
zz += ROUGHNESS * fbm((n, n), ROUGH_BASE_CELLS, ROUGH_OCTAVES, 0.5, 2.0, rng)

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

# Hillshade plot (SHOW_PLOT=True) -- eyeball terrain realism. The OBJ is already
# written above, so this is view-only; close the window to finish. Set SHOW_PLOT
# = False for headless runs (server/Docker) where no display is available.
if SHOW_PLOT:
    import matplotlib.pyplot as plt
    from matplotlib.colors import LightSource
    print(f"z range [{zz.min():.2f}, {zz.max():.2f}] m, std {zz.std():.3f}")
    ls = LightSource(azdeg=315, altdeg=45)
    rgb = ls.shade(zz, cmap=plt.cm.gist_earth, vert_exag=8, blend_mode="soft")
    plt.figure(figsize=(8, 8))
    plt.imshow(rgb, origin="lower", extent=[0, SIZE, 0, SIZE])
    for cx, cy, *_ in ISLANDS:
        plt.plot(cx, cy, "r+", ms=12)
    plt.title(f"seabed hillshade  (roughness std={ROUGHNESS} m, fBm {ROUGH_OCTAVES} oct)")
    plt.xlabel("x (m)"); plt.ylabel("y (m)")
    plt.tight_layout(); plt.show()
