#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy>=1.26", "matplotlib>=3.8"]
# ///
"""simplot — plot a `sim_trace` CSV (closed-loop FocDriver vs VirtualMotor).

Panels: speed (plant truth vs the driver's and the observer's estimates),
commutation-frame error vs the rotor at the sample instant (and the raw
observer flux angle), dq current as the controller measures it vs the TRUE
dq current (plant currents in the plant's rotor frame), dq voltage, and the
startup/trust flags. `--fft COL` adds the amplitude spectrum of one column
over the `--t0/--t1` window (mean removed, Hann window) — for spotting
hunting/limit-cycle frequencies.

Usage:
    uv run scripts/simplot.py trace.csv --out trace.png
    uv run scripts/simplot.py trace.csv --out z.png --t0 2 --t1 2.5 --fft w_true
"""
from __future__ import annotations

import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--out", default="trace.png")
    ap.add_argument("--t0", type=float, default=None)
    ap.add_argument("--t1", type=float, default=None)
    ap.add_argument("--fft", default=None, help="column to take the spectrum of")
    ap.add_argument("--fmax", type=float, default=500.0, help="spectrum x limit (Hz)")
    ap.add_argument("--title", default=None)
    a = ap.parse_args()

    d = np.genfromtxt(a.csv, delimiter=",", names=True)
    t = d["t"]
    m = np.ones_like(t, dtype=bool)
    if a.t0 is not None:
        m &= t >= a.t0
    if a.t1 is not None:
        m &= t <= a.t1
    d, t = d[m], t[m]

    rows = 5 + (1 if a.fft else 0)
    fig, ax = plt.subplots(rows, 1, figsize=(12, 2.3 * rows), sharex=False)
    tx = ax[: 5]
    for x in tx[1:]:
        x.sharex(tx[0])

    tx[0].plot(t, d["w_true"], label="ω true", lw=1.4, color="k")
    tx[0].plot(t, d["w_est"], label="ω driver", lw=0.9)
    tx[0].plot(t, d["w_obs"], label="ω observer", lw=0.7, alpha=0.8)
    tx[0].set_ylabel("el rad/s")

    tx[1].plot(t, d["frame_err"], label="Park − rotor", lw=0.9)
    tx[1].plot(t, d["obs_raw_err"], label="observer raw − rotor", lw=0.5, alpha=0.6)
    tx[1].axhline(0, color="grey", lw=0.5)
    tx[1].set_ylabel("rad")
    lim = np.nanpercentile(np.abs(d["frame_err"]), 99.5) * 1.5 + 1e-3
    tx[1].set_ylim(-lim, lim)

    tx[2].plot(t, d["iq_meas"], label="iq meas", lw=0.9)
    tx[2].plot(t, d["iq_true"], label="iq true", lw=0.9, ls="--")
    tx[2].plot(t, d["id_meas"], label="id meas", lw=0.9)
    tx[2].plot(t, d["id_true"], label="id true", lw=0.9, ls="--")
    if np.isfinite(d["iq_target"]).any():
        tx[2].plot(t, d["iq_target"], label="iq target", lw=0.8, color="grey")
    tx[2].set_ylabel("A")

    tx[3].plot(t, d["vd"], label="vd", lw=0.8)
    tx[3].plot(t, d["vq"], label="vq", lw=0.8)
    tx[3].set_ylabel("V")

    tx[4].plot(t, d["starting"], label="starting", lw=1)
    tx[4].plot(t, d["trusted"] * 0.9, label="trusted (×0.9)", lw=1)
    tx[4].set_ylabel("flag")
    tx[4].set_xlabel("t (s)")

    for x in tx:
        x.grid(alpha=0.3)
        x.legend(loc="upper right", fontsize=7, ncol=5)

    if a.fft:
        y = d[a.fft] - np.nanmean(d[a.fft])
        y = np.nan_to_num(y)
        dt = float(np.median(np.diff(t)))
        win = np.hanning(len(y))
        spec = np.abs(np.fft.rfft(y * win)) * 2 / win.sum()
        f = np.fft.rfftfreq(len(y), dt)
        sel = f <= a.fmax
        ax[5].plot(f[sel], spec[sel], lw=0.9)
        pk = np.argmax(spec[sel][1:]) + 1
        ax[5].set_title(
            f"spectrum of {a.fft} over {t[0]:.3f}–{t[-1]:.3f} s: "
            f"peak {f[sel][pk]:.1f} Hz, amplitude {spec[sel][pk]:.3g}",
            fontsize=9,
        )
        ax[5].set_xlabel("Hz")
        ax[5].grid(alpha=0.3)

    fig.suptitle(a.title or a.csv, fontsize=10)
    fig.tight_layout()
    fig.savefig(a.out, dpi=110)
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
