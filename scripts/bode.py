#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy>=1.26", "matplotlib>=3.8"]
# ///
"""bode — plot `loop_response` CSVs (magnitude in dB + phase vs frequency).

Several CSVs overlay (one curve per loop/source/tracker combination), so
variants can be compared directly. A text table of the -3 dB point and the
magnitude peak per curve is printed as well.

Usage:
    uv run scripts/bode.py cur.csv --out cur.png
    uv run scripts/bode.py est_trk.csv est_notrk.csv --out est.png
"""
from __future__ import annotations

import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--out", default="bode.png")
    ap.add_argument("--second", action="store_true", help="also plot mag2/phase2 (dashed)")
    ap.add_argument("--title", default=None)
    a = ap.parse_args()

    plt.rcParams.update({"font.size": 13})
    fig, (am, ap_) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for path in a.csv:
        d = np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")
        keys = sorted({(r["loop"], r["source"], float(r["tracker"])) for r in d})
        for loop, source, tracker in keys:
            r = d[(d["loop"] == loop) & (d["source"] == source) & (d["tracker"] == tracker)]
            f, mag, ph = r["f"], r["mag"], r["phase_deg"]
            label = f"{loop} · {source}" + (f" · tracker {tracker:g}" if source == "observer" else "")
            db = 20 * np.log10(np.maximum(mag, 1e-9))
            (line,) = am.semilogx(f, db, "o-", ms=4, lw=1.6, label=label)
            ap_.semilogx(f, np.unwrap(np.radians(ph)) * 180 / np.pi, "o-", ms=4, lw=1.6, color=line.get_color())
            if a.second:
                am.semilogx(f, 20 * np.log10(np.maximum(r["mag2"], 1e-9)), "--", lw=1.2, color=line.get_color())
                ap_.semilogx(f, r["phase2_deg"], "--", lw=1.2, color=line.get_color())
            k = int(np.argmax(mag))
            if loop == "estimator":
                # Error transfer: small at low f, rises toward 1 — report
                # where the frame starts missing 1/3 of the rotor's motion.
                above = np.nonzero(db > -10)[0]
                fx = f[above[0]] if len(above) else float("nan")
                what = f"|E| > -10 dB from ~{fx:7.1f} Hz"
            else:
                below = np.nonzero(db < -3)[0]
                fx = f[below[0]] if len(below) else float("nan")
                what = f"-3 dB at ~{fx:7.1f} Hz"
            print(f"{label:40s} {what}, peak |H| {mag[k]:.3f} at {f[k]:.1f} Hz")
    am.axhline(0, color="grey", lw=0.8)
    am.axhline(-3, color="grey", lw=0.6, ls=":")
    am.set_ylabel("|H| (dB)")
    am.grid(alpha=0.3, which="both")
    am.legend(fontsize=10)
    ap_.set_ylabel("phase (deg)")
    ap_.set_xlabel("Hz")
    ap_.grid(alpha=0.3, which="both")
    fig.suptitle(a.title or ", ".join(a.csv), fontsize=12)
    fig.tight_layout()
    fig.savefig(a.out, dpi=100)
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
