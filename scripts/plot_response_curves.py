#!/usr/bin/env python3
"""Response-curve figures from the full-scale compare CSVs (no bags, fast).

Reads data/plots/compare_summary_{cone,helix}.csv (emitted by analyze_run.py's
COMPARE section) and draws the deterministic response of each metric to the
information-cost weight alpha, one line per manifold.

This is deliberately SEPARATE from analyze_run.py. That script reads the
multi-GB /face_hits bags and is slow; this one touches only the two small
compare CSVs, so it is the fast entry point for the metric-vs-alpha figures and
needs no bags present.

Method notes (so the figure can't be misread):
  * alpha is a continuous knob, so measured points are joined by straight
    segments -- NOT a fitted or smoothed curve. The alpha=0.25 structure must
    stay visible, so nothing is regressed through the points.
  * No R^2 / regression is reported. The experiment is deterministic (no error
    term, no sampling distribution), so the analysis is a descriptive read of
    the response surface, consistent with thesis Section 4.4.4.
  * The boustrophedon has no alpha. On the CIR panel it is an in-range dashed
    reference; on the time panel its ~13.7 ks dwarfs the NBV band (3.3-6.3 ks),
    so it is annotated rather than drawn (drawing it would crush the NBV detail).

Usage:  python3 scripts/plot_response_curves.py
Output: data/plots/response_curves.png
"""
import os

import matplotlib
matplotlib.use("Agg")          # headless-safe: we only ever save, never show()
import matplotlib.pyplot as plt
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
PLOTS_DIR = os.path.normpath(os.path.join(HERE, "..", "data", "plots"))

C_CONE, C_HELIX = "tab:red", "tab:blue"


def load_compare(csv_name):
    """(alpha-sorted NBV frame, boustrophedon row) parsed from a compare CSV."""
    df = pd.read_csv(os.path.join(PLOTS_DIR, csv_name))
    cir_col = next(c for c in df.columns if "CIR" in c and ("Highest" in c or "Final" in c))
    end_col = next(c for c in df.columns if "End" in c or "Duration" in c)
    df = df.rename(columns={cir_col: "cir", end_col: "end"})
    df["alpha"] = df["Run"].str.extract(r"=\s*([0-9.]+)").astype(float)
    boustro = df[df["alpha"].isna()].iloc[0] if df["alpha"].isna().any() else None
    nbv = df[df["alpha"].notna()].sort_values("alpha").reset_index(drop=True)
    return nbv, boustro


def main():
    cone, b_c = load_compare("compare_summary_cone.csv")
    helix, b_h = load_compare("compare_summary_helix.csv")
    ref = b_c if b_c is not None else b_h
    boustro_cir, boustro_end = float(ref["cir"]), float(ref["end"])

    fig, (axL, axR) = plt.subplots(1, 2, figsize=(13, 5))

    # (a) terminal CIR vs alpha -- boustro is an in-range dashed reference
    axL.plot(cone["alpha"], cone["cir"], "o-", color=C_CONE, lw=2, ms=7, label="Cone")
    axL.plot(helix["alpha"], helix["cir"], "s-", color=C_HELIX, lw=2, ms=7, label="Helix")
    axL.axhline(boustro_cir, ls="--", color="0.4", lw=1.5,
                label=f"Boustrophedon ({boustro_cir:.2f})")
    axL.set_ylabel(r"Terminal CIR$_{total}$")
    axL.set_title(r"(a) Terminal information vs. $\alpha$")
    axL.set_ylim(0.88, 1.015)

    # (b) mission time vs alpha -- boustro off-scale, annotate instead of draw
    axR.plot(cone["alpha"], cone["end"], "o-", color=C_CONE, lw=2, ms=7, label="Cone")
    axR.plot(helix["alpha"], helix["end"], "s-", color=C_HELIX, lw=2, ms=7, label="Helix")
    axR.set_ylabel("Mission completion time (s)")
    axR.set_title(r"(b) Mission time vs. $\alpha$")
    axR.set_ylim(3000, 6800)
    axR.annotate(f"Boustrophedon ≈ {boustro_end:,.0f} s  (off scale ↑)",
                 xy=(0.5, 6650), ha="center", va="top", fontsize=9, color="0.4")

    for ax in (axL, axR):
        ax.set_xlabel(r"Information-cost weight $\alpha$")
        ax.set_xticks([0, 0.25, 0.5, 0.75, 1.0])
        ax.grid(alpha=0.3)
        ax.legend()

    fig.suptitle(r"Full-scale response to the information-cost weight $\alpha$", y=1.02)
    fig.tight_layout()
    out = os.path.join(PLOTS_DIR, "response_curves.png")
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print("Saved:", out)


if __name__ == "__main__":
    main()
