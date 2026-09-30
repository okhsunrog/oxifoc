#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["numpy>=1.26", "matplotlib>=3.8"]
# ///
"""simplot — plot a `sim_trace` CSV (closed-loop FocDriver vs VirtualMotor).

Panels (pick with --panels, default "speed,frame,dq,v,flags"):
  speed  plant ω vs the driver's and the observer's estimates
  frame  commutation-frame error vs the rotor at the sample instant (and the
         raw observer flux angle)
  dq     dq current as the controller measures it vs the TRUE dq current
         (plant currents in the plant's rotor frame)
  v      dq voltage
  flags  startup / trust flags
Extra analysis panels:
  --fft COL          amplitude spectrum of COL over the --t0/--t1 window (mean
                     removed, Hann); with --orders the x axis is harmonic
                     order of the electrical frequency (mean ω_true/2π)
  --spectrogram COL  time-frequency view of COL (hunting modes stay put,
                     electrical harmonics ride the speed)
A text summary (mean/RMS/peak-to-peak per plotted signal, spectral peaks)
is printed to stdout — numbers to go with the picture.

Usage:
    uv run scripts/simplot.py trace.csv --out trace.png
    uv run scripts/simplot.py trace.csv --out z.png --t0 2 --t1 2.5 --fft w_true
    uv run scripts/simplot.py trace.csv --panels frame,dq --fft id_true --orders
"""
from __future__ import annotations

import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

PANELS = {
    "speed": (["w_true", "w_est", "w_obs"], "el rad/s"),
    "frame": (["frame_err", "obs_raw_err"], "rad"),
    "dq": (["iq_meas", "iq_true", "id_meas", "id_true", "iq_target"], "A"),
    "v": (["vd", "vq"], "V"),
    "flags": (["starting", "trusted"], "flag"),
}
STYLE = {
    "w_true": dict(color="k", lw=2.0, label="ω true"),
    "w_est": dict(lw=1.4, label="ω driver"),
    "w_obs": dict(lw=1.2, alpha=0.8, label="ω observer"),
    "frame_err": dict(lw=1.4, label="Park − rotor"),
    "obs_raw_err": dict(lw=0.8, alpha=0.6, label="observer raw − rotor"),
    "iq_meas": dict(lw=1.4, label="iq meas"),
    "iq_true": dict(lw=1.4, ls="--", label="iq true"),
    "id_meas": dict(lw=1.4, label="id meas"),
    "id_true": dict(lw=1.4, ls="--", label="id true"),
    "iq_target": dict(lw=1.0, color="grey", label="iq target"),
    "vd": dict(lw=1.2, label="vd"),
    "vq": dict(lw=1.2, label="vq"),
    "starting": dict(lw=1.5, label="starting"),
    "trusted": dict(lw=1.5, label="trusted"),
}


def spectrum(y: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray]:
    y = np.nan_to_num(y - np.nanmean(y))
    win = np.hanning(len(y))
    return np.fft.rfftfreq(len(y), dt), np.abs(np.fft.rfft(y * win)) * 2 / win.sum()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--out", default="trace.png")
    ap.add_argument("--t0", type=float, default=None)
    ap.add_argument("--t1", type=float, default=None)
    ap.add_argument("--panels", default="speed,frame,dq,v,flags")
    ap.add_argument("--fft", default=None, help="column to take the spectrum of")
    ap.add_argument("--orders", action="store_true", help="spectrum x axis in electrical orders")
    ap.add_argument("--fmax", type=float, default=500.0, help="spectrum x limit (Hz or orders)")
    ap.add_argument("--spectrogram", default=None, help="column for a time-frequency panel")
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
    dt = float(np.median(np.diff(t)))

    names = [p for p in a.panels.split(",") if p in PANELS]
    extra = [x for x in (a.fft, a.spectrogram) if x]
    rows = len(names) + len(extra)
    plt.rcParams.update({"font.size": 13})
    fig, ax = plt.subplots(rows, 1, figsize=(13, 3.1 * rows), squeeze=False)
    ax = ax[:, 0]

    print(f"window {t[0]:.4f}–{t[-1]:.4f} s, {len(t)} samples, dt {dt * 1e6:.0f} µs")
    for i, name in enumerate(names):
        cols, unit = PANELS[name]
        for c in cols:
            if c not in d.dtype.names or not np.isfinite(d[c]).any():
                continue
            ax[i].plot(t, d[c], **STYLE.get(c, {"label": c}))
            y = d[c][np.isfinite(d[c])]
            print(
                f"  {c:12s} mean {y.mean():+10.4g}  rms(ac) {y.std():9.4g}  "
                f"p-p {np.ptp(y):9.4g}"
            )
        ax[i].set_ylabel(unit)
        ax[i].grid(alpha=0.3)
        ax[i].legend(loc="upper right", fontsize=10, ncol=5)
        if name == "frame":
            lim = np.nanpercentile(np.abs(d["frame_err"]), 99.5) * 1.5 + 1e-3
            ax[i].set_ylim(-lim, lim)
            ax[i].axhline(0, color="grey", lw=0.6)
        if i > 0:
            ax[i].sharex(ax[0])
    if names:
        ax[len(names) - 1].set_xlabel("t (s)")

    row = len(names)
    if a.fft:
        f, s = spectrum(d[a.fft], dt)
        f_el = float(np.nanmean(d["w_true"])) / (2 * np.pi)
        x = f / f_el if a.orders and f_el > 0 else f
        sel = (x <= a.fmax) & (f > 0)
        ax[row].plot(x[sel], s[sel], lw=1.3)
        top = np.argsort(s[sel])[::-1]
        peaks, used = [], []
        for k in top:
            if all(abs(x[sel][k] - u) > 3 * (x[1] - x[0]) for u in used):
                used.append(x[sel][k])
                peaks.append((x[sel][k], s[sel][k]))
            if len(peaks) == 4:
                break
        unit = "order" if a.orders else "Hz"
        desc = ", ".join(f"{p:.2f} {unit}: {v:.3g}" for p, v in peaks)
        print(f"spectrum of {a.fft} (f_el {f_el:.1f} Hz): peaks {desc}")
        ax[row].set_title(f"spectrum of {a.fft} — peaks {desc}", fontsize=11)
        ax[row].set_xlabel(f"harmonic order of f_el = {f_el:.1f} Hz" if a.orders else "Hz")
        ax[row].grid(alpha=0.3)
        row += 1
    if a.spectrogram:
        y = np.nan_to_num(d[a.spectrogram] - np.nanmean(d[a.spectrogram]))
        nfft = int(min(4096, max(256, 2 ** np.floor(np.log2(len(y) / 16)))))
        ax[row].specgram(y, NFFT=nfft, Fs=1 / dt, noverlap=nfft * 3 // 4, cmap="magma")
        ax[row].set_ylim(0, a.fmax)
        ax[row].set_ylabel("Hz")
        ax[row].set_xlabel("t (s, from window start)")
        ax[row].set_title(f"spectrogram of {a.spectrogram}", fontsize=11)

    fig.suptitle(a.title or a.csv, fontsize=13)
    fig.tight_layout()
    fig.savefig(a.out, dpi=100)
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
