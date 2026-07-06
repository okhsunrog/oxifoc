#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["pyarrow>=16", "pandas>=2", "numpy>=1.26", "matplotlib>=3.8"]
# ///
"""replay_observer — offline flux-observer replay against a telemetry capture.

Reconstructs v_alpha/v_beta from the recorded vd/vq/angle (the commanded
frame — angle must be the ACTIVE source, i.e. a capture from a PLAIN
firmware, NOT the obs-debug-telem mapping) and i_alpha/i_beta from the phase
currents, then re-runs the MXLEMMING flux integrator + PLL over the record
with arbitrary parameter variants (R, L_hf, eddy ladder dL/tau, lambda,
PLL kp/ki).

What it is for (2026-07-06 slip-kick investigation): parameter-sensitivity
forensics WITHOUT reflashing. Key property discovered with it: if all
variants produce the same trajectory, the failure is encoded in the
recorded v,i themselves (closed-loop self-consistent) and no observer-side
parameter change can fix it — the lever is in the control loop.

Caveats: the capture is sample-decimated (no anti-aliasing), so a 2 kHz
record aliases the 20 kHz loop; slip-scale events (~5-10 ms) are resolved,
per-PWM detail is not. The PLL portion of the replay only shows how a
hypothetical PLL would track the RECORDED flux — it cannot un-close the
loop that produced the record.

Usage:
    uv run scripts/replay_observer.py captures/sawtooth-plain-1.parquet \
        --out replay.png [--t0 0.9 --t1 2.2]
"""
from __future__ import annotations
import argparse
import numpy as np
import pyarrow.parquet as pq


def load_ab(path):
    df = pq.read_table(path).to_pandas()
    t = (df["t_s"] - df["t_s"].iloc[0]).to_numpy().copy()
    ang = df["angle_rad"].to_numpy()
    vd, vq = df["vd_v"].to_numpy(), df["vq_v"].to_numpy()
    ia, ib = df["ia_a"].to_numpy(), df["ib_a"].to_numpy()
    c, s = np.cos(ang), np.sin(ang)
    va = c * vd - s * vq
    vb = s * vd + c * vq
    i_al = ia
    i_be = (ia + 2 * ib) / np.sqrt(3.0)
    erpm = df["erpm"].to_numpy()
    return t, va, vb, i_al, i_be, erpm


def replay(t, va, vb, i_al, i_be, r, l_hf, dl=0.0, tau=0.3e-3,
           lam=1.145e-3, kp=1000.0, ki=20000.0):
    """Flux integrator (+ component clamp) + PLL over the record."""
    dtv = np.gradient(t)
    x1 = x2 = 0.0
    ifa = ifb = 0.0
    ia_l, ib_l = i_al[0], i_be[0]
    w = np.zeros(len(t))
    pll_v = pll_p = 0.0
    for k in range(len(t)):
        dt = min(max(dtv[k], 1e-4), 2e-3)
        if dl > 0.0:
            dfa = (dt / tau) * (i_al[k] - ifa)
            dfb = (dt / tau) * (i_be[k] - ifb)
            ifa += dfa
            ifb += dfb
        else:
            dfa = dfb = 0.0
        x1 += (va[k] - r * i_al[k]) * dt - l_hf * (i_al[k] - ia_l) - dl * dfa
        x2 += (vb[k] - r * i_be[k]) * dt - l_hf * (i_be[k] - ib_l) - dl * dfb
        ia_l, ib_l = i_al[k], i_be[k]
        x1 = np.clip(x1, -lam, lam)
        x2 = np.clip(x2, -lam, lam)
        err = np.angle(np.exp(1j * (np.arctan2(x2, x1) - pll_p)))
        pll_v += ki * err * dt
        pll_p = np.angle(np.exp(1j * (pll_p + (pll_v + kp * err) * dt)))
        w[k] = pll_v
    return w


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("file")
    ap.add_argument("--out", default="replay.png")
    ap.add_argument("--t0", type=float, default=None)
    ap.add_argument("--t1", type=float, default=None)
    args = ap.parse_args()

    t, va, vb, i_al, i_be, erpm = load_ab(args.file)
    m = np.ones(len(t), bool)
    if args.t0 is not None:
        m &= t >= args.t0
    if args.t1 is not None:
        m &= t <= args.t1

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 1, figsize=(13, 5))
    ax.plot(t[m], erpm[m] * 2 * np.pi / 60, lw=1.2, color="k",
            label="firmware ω̂ (active src)")
    variants = [
        ("R=0.127 L=24µ (baked)", dict(r=0.127, l_hf=24e-6)),
        ("R=0.16", dict(r=0.16, l_hf=24e-6)),
        ("+eddy ladder dL=105µ", dict(r=0.127, l_hf=24e-6, dl=105e-6)),
        ("PLL ki/4", dict(r=0.127, l_hf=24e-6, kp=500.0, ki=5000.0)),
    ]
    for lbl, kw in variants:
        w = replay(t, va, vb, i_al, i_be, **kw)
        ax.plot(t[m], w[m], lw=0.7, alpha=0.85, label=f"replay {lbl}")
    ax.legend(fontsize=8)
    ax.set_ylabel("rad/s el")
    ax.set_xlabel("t s")
    fig.tight_layout()
    fig.savefig(args.out, dpi=95)
    print(f"[plot: {args.out}]")


if __name__ == "__main__":
    main()
