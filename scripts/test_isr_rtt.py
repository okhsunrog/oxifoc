import unittest

import isr_rtt as rtt

LOC = " (oxifoc_f405 src/control/foc.rs:944)"


def report(n=14997, avg=2560, mx=4000, motion=None, edges=0, telem=False, short=None, motion_first=True):
    """One second of RTT output; `motion` is (n, avg, max)."""
    lines = []
    isr = f"\x1b[32m[INFO ] isr/s: n={n} avg={avg} max={mx} over=0 load_pct=22{LOC}\n"
    mot = [f"[INFO ] isr-motion/s: n={motion[0]} avg={motion[1]} max={motion[2]}{LOC}\n"] if motion else []
    lines += mot + [isr] if motion_first else [isr] + mot
    if short is not None:
        lines.append(f"[INFO ] isri/s: n={n} avg=11200 min=11000 max=90000 long=1 short={short} expected=11200\n")
    if telem:
        lines.append("[INFO ] telem/s: push=1000 drop=0 read=1000\n")
    lines.append(f"[INFO ] hall/s: state=11 edges={edges} overcap=0{LOC}\n")
    return lines


def run(motion_first=True):
    """Attach halt, idle, streaming, +120 phase, transition, +450 phase, Stop."""
    log = ["[WARN ] DRV8301 Fault Status:\n"]
    log += report(n=14581, avg=2700, mx=9000, short=1)                   # attach halt
    log += report(avg=2557) * 3                                           # idle
    log += report(avg=2605, telem=True) * 3                               # streaming
    m = dict(telem=True, motion_first=motion_first)
    log += report(avg=5747, motion=(13024, 6044, 7051), edges=93, **m)   # start
    log += report(avg=6160, motion=(14997, 6160, 7100), edges=114, **m) * 2
    log += report(avg=6162, motion=(14997, 6162, 7200), edges=115, **m) * 3
    log += report(avg=6160, motion=(14997, 6160, 7700), edges=115, **m)  # +450 command lands
    log += report(avg=6057, motion=(14998, 6057, 7136), edges=300, **m)  # transition
    log += report(avg=6022, motion=(14997, 6022, 7065), edges=430, **m) * 5
    log += report(avg=5000, motion=(9000, 6000, 7000), edges=200, **m)  # Stop
    log += report(avg=2557, edges=12)                                     # coasting
    log += report(avg=2557) * 2
    log += ["Received Ctrl+C, exiting\n"]
    return log


class RttTests(unittest.TestCase):
    def check(self, s):
        self.assertEqual((s["nominal_n"], s["dropped"]), (14997, 1))
        c = s["classes"]
        self.assertEqual(list(c), ["idle", "stream", "phase 1", "phase 2"])
        self.assertEqual((c["idle"]["avg"], c["idle"]["windows"]), (2557, 3))
        self.assertEqual(c["stream"]["avg"], 2605)
        # First and last full window of a phase are dropped: 4 of 6 remain.
        self.assertEqual(c["phase 1"]["windows"], 4)
        self.assertEqual((c["phase 1"]["avg"], c["phase 1"]["max"]), (6161.5, 7200))
        self.assertEqual(c["phase 1"]["spread"], 2)
        self.assertEqual((c["phase 2"]["avg"], c["phase 2"]["windows"], c["phase 2"]["hall_edges"]), (6022, 3, 430))

    def test_motion_line_before_isr(self):
        self.check(rtt.summarize(run(motion_first=True)))

    def test_motion_line_after_isr(self):
        self.check(rtt.summarize(run(motion_first=False)))

    def test_attach_halt_window_is_dropped(self):
        reps = rtt.reports(report(n=14200, avg=9999) + report(avg=2557) * 3)
        self.assertEqual(rtt.summarize(report(n=14200, avg=9999) + report(avg=2557) * 3)["classes"]["idle"]["avg"], 2557)
        self.assertFalse(rtt.full(reps[0], rtt.nominal_n(reps)))
        # A full count with a short interval also marks a halt.
        reps = rtt.reports(report(short=1) + report() * 3)
        self.assertFalse(rtt.full(reps[0], rtt.nominal_n(reps)))

    def test_short_phase_is_a_transition(self):
        log = report() * 3 + report(motion=(14997, 6100, 7000), edges=115) * 3
        self.assertNotIn("phase 1", rtt.summarize(log, min_windows=3)["classes"])
        self.assertIn("phase 1", rtt.summarize(log, min_windows=1)["classes"])


if __name__ == "__main__":
    unittest.main()
