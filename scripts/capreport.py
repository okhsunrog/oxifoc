#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = ["polars>=1", "pyarrow>=16", "numpy>=1.26", "matplotlib>=3.8"]
# ///
"""capreport — LLM-oriented triage report for oxifoc FOC telemetry captures.

Reads one enriched telemetry parquet (16-col schema + oxifoc.* KV metadata)
and emits a compact, structured view for an LLM (or human) to reason about
how the signals evolve across a maneuver.

Sections: MANIFEST, INTEGRITY (incl. enrichment-trust guard), SEGMENTS
(command-event-aligned when metadata exists, else auto change-point) with
per-segment shape features + regime tags, DELTAS, and an envelope-decimated
raw view. `--format json` emits the same content as structured data;
`--plot` writes a multi-panel overview PNG (+ per-segment zooms with --zooms).

Usage:
    uv run scripts/capreport.py captures/staircase-1.parquet
    uv run scripts/capreport.py cap.parquet --format json
    uv run scripts/capreport.py cap.parquet --plot out.png --zooms
"""
from __future__ import annotations
import argparse, json, sys
import numpy as np
import polars as pl
import pyarrow.parquet as pq

# ---------- loading -------------------------------------------------------
def load(path):
    pf = pq.ParquetFile(path)
    md = {k.decode(): v.decode() for k, v in (pf.metadata.metadata or {}).items()}
    return pl.read_parquet(path), md

def jget(md, key, default=None):
    v = md.get(key)
    if v is None:
        return default
    try:
        return json.loads(v)
    except (json.JSONDecodeError, TypeError):
        return v

# ---------- integrity -----------------------------------------------------
def unwrap_seq(seq, period=65536):
    """u16 device seq -> monotonic int64 (handles the ~8 wraps in a long capture)."""
    s = seq.astype(np.int64)
    d = np.diff(s)
    wraps = np.cumsum(np.where(d < -period // 2, 1, 0))
    out = s.copy()
    out[1:] += wraps * period
    return out

def integrity(df, md):
    u = unwrap_seq(df["seq"].to_numpy())
    dm = int(jget(md, "oxifoc.decimation_m", 1) or 1)
    steps = np.diff(u)
    gaps = steps[steps > dm]
    dt_ms = np.diff(df["t_s"].to_numpy()) * 1e3
    eng = ["ia_a", "iq_a", "id_a", "vbus_v"]
    nan_frac = {c: float(np.isnan(df[c].to_numpy()).mean()) for c in eng}
    cfg = jget(md, "oxifoc.config", {}) or {}
    has_dc = "dc-offsets" in cfg
    pp = (cfg.get("motor-params", {}) or {}).get("pole_pairs")
    return dict(
        rows=df.height, seq_span=int(u[-1] - u[0]),
        samples_lost=int(jget(md, "oxifoc.samples_lost", 0) or 0),
        seq_gaps_meta=int(jget(md, "oxifoc.seq_gaps", 0) or 0),
        seq_gaps_obs=int((steps > dm).sum()),
        largest_gap=int(gaps.max()) if gaps.size else 0,
        dt_ms_median=float(np.median(dt_ms)) if dt_ms.size else 0.0,
        dt_ms_p99=float(np.percentile(dt_ms, 99)) if dt_ms.size else 0.0,
        dt_ms_max=float(dt_ms.max()) if dt_ms.size else 0.0,
        nan_frac=nan_frac, nan_frac_max=max(nan_frac.values()),
        enrichment_trusted=bool(has_dc and pp),
        dc_offsets_present=has_dc, pole_pairs=pp, decimation_m=dm,
    )

# ---------- per-segment feature extraction --------------------------------
def osc_freq_hz(x, t):
    """Dominant oscillation frequency via mean-crossing rate of the AC part."""
    if len(x) < 4:
        return 0.0
    dur = float(t[-1] - t[0])
    if dur <= 0:
        return 0.0
    ac = x - x.mean()
    amp = np.abs(ac).mean()
    if amp < 1e-6:
        return 0.0
    signs = np.sign(ac)
    signs[signs == 0] = 1
    crossings = int((np.diff(signs) != 0).sum())
    return crossings / 2.0 / dur  # a full cycle == 2 mean-crossings

def seg_features(sl, iq_cmd=None, motor=None):
    t = sl["t_s"].to_numpy()
    iq = sl["iq_a"].to_numpy(); erpm = sl["erpm"].to_numpy()
    vmag = np.hypot(sl["vd_v"].to_numpy(), sl["vq_v"].to_numpy())
    iabc = np.maximum.reduce([np.abs(sl[c].to_numpy()) for c in ("ia_a", "ib_a", "ic_a")])
    # Back-EMF cross-check (the 2026-07-06 probe-session lesson distilled):
    # erpm is the ACTIVE source's claim; the physics check is whether the
    # terminal voltage actually carries the implied back-EMF,
    # |e| ≈ |v| − R·|i_dq| vs λ·ω̂. Ratio ≈ 1 → the claimed spin is real;
    # ≈ 0 → phantom / standstill behind a spinning claim. Only meaningful
    # above the dead-time distortion floor (~0.15 V on the bench board).
    bemf_ratio = None
    if motor and motor.get("resistance_ohm") and motor.get("flux_linkage_wb"):
        r, lam = motor["resistance_ohm"], motor["flux_linkage_wb"]
        idq = np.hypot(sl["id_a"].to_numpy(), iq)
        bemf_meas = float(np.mean(np.maximum(vmag - r * idq, 0.0)))
        bemf_claim = lam * abs(float(erpm.mean())) * 2.0 * np.pi / 60.0
        if bemf_claim > 0.15:
            bemf_ratio = bemf_meas / bemf_claim
    f = dict(
        t0=float(t[0]), t1=float(t[-1]), dur=float(t[-1] - t[0]), n=len(t),
        iq_cmd=iq_cmd,
        iq_mean=float(iq.mean()), iq_std=float(iq.std()),
        id_mean=float(sl["id_a"].mean()),
        erpm_mean=float(erpm.mean()), erpm_std=float(erpm.std()),
        erpm_min=float(erpm.min()), erpm_max=float(erpm.max()),
        erpm_p2p=float(erpm.max() - erpm.min()),
        erpm_osc_hz=osc_freq_hz(erpm, t),
        iq_osc_hz=osc_freq_hz(iq, t),
        vq_mean=float(sl["vq_v"].mean()), vbus_mean=float(sl["vbus_v"].mean()),
        # |v| = √(vd²+vq²): back-EMF magnitude lives here, not in vq alone —
        # a runaway rotor shows up as v_mag ≫ R·i while vq can look tame
        # (the 2026-07-06 hold-ratchet forensics needed exactly this).
        vmag_mean=float(vmag.mean()), vmag_max=float(vmag.max()),
        iabc_pk=float(iabc.max()),
        bemf_ratio=bemf_ratio,
    )
    f["regime"] = classify_regime(f)
    return f

def classify_regime(f):
    tags = []
    vbus = f["vbus_mean"]
    if vbus > 0 and abs(f["vq_mean"]) / vbus > 0.85:
        tags.append("V-SAT")
    spinning = abs(f["erpm_mean"]) > 100
    tags.append("SPIN" if spinning else "STALL")
    if f["erpm_min"] < 0 < f["erpm_max"]:
        tags.append("REVERSING")
    if spinning and f["erpm_std"] / (abs(f["erpm_mean"]) + 1e-6) > 0.4:
        tags.append("LIMIT-CYCLE")
    # phantom-lock: estimator pinned to one quantum (std~0) with ~no current
    if spinning and f["erpm_std"] < 1.0 and abs(f["iq_mean"]) < 0.05:
        tags.append("PHANTOM?")
    if f["iq_cmd"] is not None and f["iq_std"] > 0.5 * (abs(f["iq_cmd"]) + 0.2):
        tags.append("IQ-NOISY")
    # torque commanded but nothing flows and nothing spins: a tripped/latched
    # drive (fault gate, OC latch) — distinct from an honest STALL under load.
    if (f["iq_cmd"] is not None and abs(f["iq_cmd"]) > 0.05
            and abs(f["iq_mean"]) < 0.1 * abs(f["iq_cmd"]) and not spinning):
        tags.append("NO-TORQUE(TRIP?)")
    # Physics check on a spinning claim: does the terminal voltage carry the
    # implied back-EMF? (None = below the distortion floor / no params.)
    if f["bemf_ratio"] is not None:
        if 0.4 <= f["bemf_ratio"] <= 2.5:
            tags.append("BEMF-OK")
        else:
            tags.append(f"BEMF-MISMATCH({f['bemf_ratio']:.2f})")
    return tags

# ---------- segmentation --------------------------------------------------
def segments_from_events(df, md, marks=None):
    """Command-event boundaries + optional `marks` (device-side moments the
    metadata can't know about — handoff, OC trip, failsafe — read off the
    defmt log, in seconds RELATIVE to capture start). A mark splits the
    segment it lands in; the sub-segment keeps the parent's iq_cmd, so a
    "drive commanded, then tripped" story reads as two rows instead of one
    averaged-away blur."""
    motor = (jget(md, "oxifoc.config", {}) or {}).get("motor-params", {})
    ev = jget(md, "oxifoc.events", []) or []
    if not ev and not marks:
        return None
    t = df["t_s"].to_numpy()
    bounds = []  # (t_abs, label, iq_cmd or None-to-inherit)
    for e in ev:
        cmd = e["cmd"]; args = cmd[next(iter(cmd))]
        iq_cmd = args.get("iq") if isinstance(args, dict) else None
        bounds.append((e["t_acked"], next(iter(cmd)), iq_cmd, True))
    for m in marks or []:
        bounds.append((t[0] + m, "mark", None, False))
    bounds.sort(key=lambda b: b[0])
    # marks inherit the last real command's iq_cmd
    last_iq = None
    resolved = []
    for tb, label, iq_cmd, is_event in bounds:
        if is_event:
            last_iq = iq_cmd
        resolved.append((tb, label, iq_cmd if is_event else last_iq))
    idx = np.searchsorted(t, [b[0] for b in resolved])
    segs = []
    for i, (tb, label, iq_cmd) in enumerate(resolved):
        lo = idx[i]
        hi = idx[i + 1] if i + 1 < len(idx) else df.height
        if hi <= lo:
            continue
        f = seg_features(df.slice(lo, hi - lo), iq_cmd, motor)
        f["i"] = i; f["cmd"] = label
        segs.append(f)
    return segs

def segments_auto(df, motor=None, win_s=0.25, min_seg_s=0.4):
    """Event-free change-point segmentation: bucket into windows, tag each by
    regime, then coalesce adjacent windows with the same tag-set."""
    t = df["t_s"].to_numpy()
    total = float(t[-1] - t[0])
    if total <= 0:
        return []
    edges = np.arange(t[0], t[-1], win_s)
    bidx = np.searchsorted(t, edges)
    bidx = np.unique(np.append(bidx, df.height))
    wins = []
    for a, b in zip(bidx, bidx[1:]):
        if b <= a:
            continue
        f = seg_features(df.slice(a, b - a), motor=motor)
        wins.append((a, b, tuple(f["regime"])))
    # coalesce consecutive equal regimes
    segs = []
    ca, cb, cr = wins[0]
    for a, b, r in wins[1:]:
        if r == cr:
            cb = b
        else:
            segs.append((ca, cb)); ca, cb, cr = a, b, r
    segs.append((ca, cb))
    # merge tiny segments into previous
    out = []
    for a, b in segs:
        if out and (t[b - 1] - t[a]) < min_seg_s:
            out[-1] = (out[-1][0], b)
        else:
            out.append((a, b))
    result = []
    for i, (a, b) in enumerate(out):
        f = seg_features(df.slice(a, b - a), motor=motor)
        f["i"] = i; f["cmd"] = "auto"
        result.append(f)
    return result

def get_segments(df, md, marks=None):
    segs = segments_from_events(df, md, marks)
    if segs:
        return segs, "event+marks" if marks else "event"
    motor = (jget(md, "oxifoc.config", {}) or {}).get("motor-params", {})
    return segments_auto(df, motor), "auto"

# ---------- text rendering ------------------------------------------------
def fmt_manifest(md, integ):
    cfg = jget(md, "oxifoc.config", {}) or {}
    mp = cfg.get("motor-params", {}); pi = cfg.get("pi-gains", {}); lim = cfg.get("current-limits", {})
    man = jget(md, "oxifoc.maneuver", {}) or {}
    L = ["## MANIFEST",
         f"maneuver   : {man.get('name', '(none)')}"]
    if man.get("description"):
        L.append(f"  desc     : {man['description']}")
    L += [
        f"hw/mcu/sw  : {md.get('oxifoc.hw','?')} / {md.get('oxifoc.mcu','?')} / {md.get('oxifoc.sw','?')}",
        f"rates      : foc={md.get('oxifoc.foc_freq_hz','?')}Hz fast={md.get('oxifoc.fast_hz_actual','?')}Hz (req {md.get('oxifoc.fast_hz_requested','?')}) decim M={integ['decimation_m']}",
        f"motor      : R={mp.get('resistance_ohm')}Ω Ld={mp.get('inductance_d_h')}H Lq={mp.get('inductance_q_h')}H λ={mp.get('flux_linkage_wb')}Wb PP={mp.get('pole_pairs')}",
        f"pi-gains   : kp={pi.get('kp')} ki={pi.get('ki')} bw={pi.get('bandwidth_rad_s')}rad/s",
        f"limits     : max_iq={lim.get('max_iq_a')}A bus_in={lim.get('bus_in_max_a')}A regen={lim.get('bus_regen_max_a')}A",
        "caveat     : erpm/angle = ACTIVE phase source (startup openloop during"
        " starts, observer after handoff) — NOT ground-truth rotor",
    ]
    return "\n".join(L)

def fmt_integrity(integ):
    L = ["## INTEGRITY",
         f"rows={integ['rows']} seq_span={integ['seq_span']} samples_lost={integ['samples_lost']} "
         f"seq_gaps={integ['seq_gaps_meta']} (obs {integ['seq_gaps_obs']}, largest {integ['largest_gap']}smp)",
         f"dt_ms median={integ['dt_ms_median']:.1f} p99={integ['dt_ms_p99']:.1f} max={integ['dt_ms_max']:.1f}",
         f"enrichment={'OK' if integ['enrichment_trusted'] else 'SUSPECT'} "
         f"(dc_offsets={'yes' if integ['dc_offsets_present'] else 'NO'} pole_pairs={integ['pole_pairs']} "
         f"NaN_max={integ['nan_frac_max']:.3f})"]
    if not integ["enrichment_trusted"]:
        L.append("  !! engineering columns may be WRONG (mid-scale/PP fallback) — trust only *_adc")
    return "\n".join(L)

def fmt_segments(segs, kind):
    if not segs:
        return "## SEGMENTS\n(empty)"
    head = "auto change-point (no maneuver metadata)" if kind == "auto" else kind
    L = [f"## SEGMENTS ({head})",
         f"{'#':>2s} {'cmd':>6s} {'iqC':>5s} {'t0..t1':>11s} {'iq(mean±sd)':>13s} {'id':>6s} "
         f"{'erpm(mean±sd)':>15s} {'p2p':>6s} {'osc':>7s} {'vq/|v|/vb':>14s} regime"]
    for s in segs:
        iqc = f"{s['iq_cmd']:.2f}" if s.get("iq_cmd") is not None else "  -- "
        osc = f"{s['erpm_osc_hz']:.1f}Hz" if s["erpm_osc_hz"] > 0.05 else "   -  "
        L.append(
            f"{s['i']:>2d} {s['cmd']:>6s} {iqc:>5s} {s['t0']:5.1f}..{s['t1']:5.1f} "
            f"{s['iq_mean']:6.2f}±{s['iq_std']:4.2f} {s['id_mean']:6.2f} "
            f"{s['erpm_mean']:8.0f}±{s['erpm_std']:5.0f} {s['erpm_p2p']:6.0f} {osc:>7s} "
            f"{s['vq_mean']:4.2f}/{s['vmag_mean']:4.2f}/{s['vbus_mean']:4.1f} "
            f"{' '.join(s['regime'])}")
    return "\n".join(L)

def fmt_deltas(segs):
    if not segs or len(segs) < 2:
        return ""
    L = ["## DELTAS (seg N vs N-1)"]
    for a, b in zip(segs, segs[1:]):
        dcmd = ""
        if a.get("iq_cmd") is not None and b.get("iq_cmd") is not None:
            dcmd = f"iq_cmd {b['iq_cmd'] - a['iq_cmd']:+.2f}A  "
        L.append(f"  {a['i']}->{b['i']}: {dcmd}"
                 f"erpm {b['erpm_mean'] - a['erpm_mean']:+.0f}  "
                 f"osc_sd {b['erpm_std'] - a['erpm_std']:+.0f}  "
                 f"vq {b['vq_mean'] - a['vq_mean']:+.2f}")
    return "\n".join(L)

def fmt_envelope(df, n=32):
    """Envelope decimation: per time-bucket min/mean/max (honest for oscillatory signals)."""
    t = df["t_s"].to_numpy()
    edges = np.linspace(t[0], t[-1], n + 1)
    bidx = np.searchsorted(t, edges)
    iq = df["iq_a"].to_numpy(); erpm = df["erpm"].to_numpy()
    vmag = np.hypot(df["vd_v"].to_numpy(), df["vq_v"].to_numpy())
    L = [f"## ENVELOPE ({n} buckets: min⁄mean⁄max per window)",
         f"{'t_mid':>6s} {'iq[min mean max]':>21s} {'erpm[min mean max]':>24s} {'|v|_mean':>8s}"]
    for a, b in zip(bidx, bidx[1:]):
        if b <= a:
            continue
        tm = (t[a] + t[b - 1]) / 2
        L.append(f"{tm:6.2f} {iq[a:b].min():6.2f}{iq[a:b].mean():7.2f}{iq[a:b].max():7.2f} "
                 f"  {erpm[a:b].min():7.0f}{erpm[a:b].mean():8.0f}{erpm[a:b].max():8.0f} "
                 f"  {vmag[a:b].mean():7.2f}")
    return "\n".join(L)

def render_text(df, md, integ, segs, kind):
    parts = [fmt_manifest(md, integ), fmt_integrity(integ),
             fmt_segments(segs, kind), fmt_deltas(segs), fmt_envelope(df)]
    return "\n\n".join(p for p in parts if p)

def render_json(md, integ, segs, kind):
    man = jget(md, "oxifoc.maneuver", {}) or {}
    return json.dumps(dict(
        maneuver=man.get("name"), hw=md.get("oxifoc.hw"), sw=md.get("oxifoc.sw"),
        config=jget(md, "oxifoc.config", {}),
        integrity=integ, segment_kind=kind, segments=segs,
    ), indent=1)

# ---------- plotting ------------------------------------------------------
def plot_overview(df, md, segs, out, marks=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = df["t_s"].to_numpy()
    fig, ax = plt.subplots(4, 1, figsize=(11, 9), sharex=True)
    iq = df["iq_a"].to_numpy()
    ax[0].plot(t, iq, lw=0.6, label="iq"); ax[0].plot(t, df["id_a"], lw=0.5, label="id", alpha=0.7)
    ax[0].set_ylabel("A"); ax[0].legend(loc="upper right", fontsize=8)
    # Robust y-limits: a single trip spike (±10 A) otherwise crushes the
    # 0.3-0.5 A drive detail into a flat line. The clipped-off true peak is
    # annotated so the spike magnitude isn't hidden.
    lo, hi = np.percentile(iq, [0.5, 99.5])
    pad = 0.2 * max(hi - lo, 0.1)
    if iq.min() < lo - pad or iq.max() > hi + pad:
        ax[0].set_ylim(lo - pad, hi + pad)
        ax[0].annotate(f"iq clipped: true range [{iq.min():.1f}, {iq.max():.1f}] A",
                       (0.01, 0.04), xycoords="axes fraction", fontsize=7, color="tab:red")
    ax[1].plot(t, df["erpm"], lw=0.6, color="tab:green"); ax[1].set_ylabel("erpm (active src)")
    vmag = np.hypot(df["vd_v"].to_numpy(), df["vq_v"].to_numpy())
    ax[2].plot(t, df["vq_v"], lw=0.6, label="vq")
    ax[2].plot(t, vmag, lw=0.6, label="|v|", alpha=0.8)
    ax[2].plot(t, df["vbus_v"], lw=0.6, label="vbus")
    ax[2].set_ylabel("V"); ax[2].legend(loc="upper right", fontsize=8)
    ax[3].plot(t, df["ia_a"], lw=0.4); ax[3].set_ylabel("ia_a"); ax[3].set_xlabel("t_s")
    for s in segs or []:
        for a in ax:
            a.axvline(s["t0"], color="k", ls=":", lw=0.5, alpha=0.4)
        lbl = f"iq={s['iq_cmd']}" if s.get("iq_cmd") is not None else s["cmd"]
        ax[0].annotate(f"{lbl}\n{' '.join(s['regime'])}", (s["t0"], 1.02),
                       xycoords=("data", "axes fraction"), fontsize=6, rotation=0, va="bottom")
    for m in marks or []:
        for a in ax:
            a.axvline(t[0] + m, color="tab:red", ls="--", lw=0.8, alpha=0.7)
    man = jget(md, "oxifoc.maneuver", {}) or {}
    fig.suptitle(f"{man.get('name', 'capture')} — {md.get('oxifoc.hw', '')}")
    fig.tight_layout(); fig.savefig(out, dpi=90); plt.close(fig)
    return out

def plot_zoom(df, seg, out, cycles_window_s=0.6):
    """Zoom on a short window inside a segment so an LLM/human can count cycles."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = df["t_s"].to_numpy()
    c = (seg["t0"] + seg["t1"]) / 2
    m = (t >= c - cycles_window_s / 2) & (t <= c + cycles_window_s / 2)
    fig, ax = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
    ax[0].plot(t[m], df["iq_a"].to_numpy()[m], lw=0.9, label="iq")
    ax[0].plot(t[m], df["id_a"].to_numpy()[m], lw=0.7, label="id", alpha=0.7)
    ax[0].set_ylabel("A"); ax[0].legend(fontsize=8)
    ax[1].plot(t[m], df["erpm"].to_numpy()[m], lw=0.9, color="tab:green"); ax[1].set_ylabel("erpm")
    ax[1].set_xlabel("t_s")
    fig.suptitle(f"seg{seg['i']} {' '.join(seg['regime'])} zoom @ {c:.1f}s (osc {seg['erpm_osc_hz']:.1f}Hz)")
    fig.tight_layout(); fig.savefig(out, dpi=90); plt.close(fig)
    return out

# ---------- main ----------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="LLM-oriented triage report for oxifoc telemetry captures")
    ap.add_argument("file")
    ap.add_argument("--format", choices=["text", "json"], default="text")
    ap.add_argument("--marks", metavar="T1,T2,...",
                    help="extra segment boundaries (s, RELATIVE to capture start) for "
                         "device-side moments the maneuver metadata can't know — handoff, "
                         "OC trip, failsafe — read off the defmt log")
    ap.add_argument("--plot", metavar="PNG", help="write multi-panel overview PNG")
    ap.add_argument("--zooms", action="store_true", help="also write per-segment zoom PNGs next to --plot")
    args = ap.parse_args()

    marks = [float(x) for x in args.marks.split(",")] if args.marks else None
    df, md = load(args.file)
    integ = integrity(df, md)
    segs, kind = get_segments(df, md, marks)

    if args.format == "json":
        print(render_json(md, integ, segs, kind))
    else:
        print(render_text(df, md, integ, segs, kind))

    if args.plot:
        p = plot_overview(df, md, segs, args.plot, marks)
        print(f"[plot: {p}]", file=sys.stderr)
        if args.zooms:
            base = args.plot.rsplit(".", 1)[0]
            for s in segs or []:
                # Mark-created segments are the caller's declared points of
                # interest — always zoom those, plus the pathological regimes.
                if ("LIMIT-CYCLE" in s["regime"] or "V-SAT" in s["regime"]
                        or s["cmd"] == "mark"):
                    z = plot_zoom(df, s, f"{base}.seg{s['i']}.png")
                    print(f"[zoom: {z}]", file=sys.stderr)

if __name__ == "__main__":
    main()
