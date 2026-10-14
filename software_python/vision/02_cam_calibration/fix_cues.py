"""Correct the cue timestamps of protocol-1.0 gesture sessions.

Protocol 1.0 logged GO and STOP AFTER the blocking winsound.Beep call returned, so both were late by the beep duration
(350 ms). READY was logged before its beeps and is correct. This writes cues_fixed.csv (the original is untouched) with
go/stop shifted earlier, i.e. referenced to the beep CALL. The residual offset to the audible onset is the audio output
latency, which is not measured.

    python fix_cues.py --dir sessions/g01                  # -> sessions/g01/cues_fixed.csv
    python segments_qc.py --dir sessions/g01 --cues cues_fixed.csv --out sessions/g01/qc_fixed.png

Self-check printed: median(stop - go) - window_s should be ~ the beep duration (+ a few ms of overhead), because in
protocol 1.0 the logged GO-to-STOP interval is window_s + the STOP beep duration.
"""
import argparse
import json
import os

import numpy as np
import pandas as pd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=".")
    ap.add_argument("--cues", default="cues.csv")
    ap.add_argument("--out", default="cues_fixed.csv")
    ap.add_argument("--shift", type=float, default=0.350, help="seconds to move go/stop earlier (beep duration)")
    ap.add_argument("--force", action="store_true", help="apply even if session.json says protocol >= 1.1")
    a = ap.parse_args()
    cp, sp = os.path.join(a.dir, a.cues), os.path.join(a.dir, "session.json")
    meta = json.load(open(sp)) if os.path.exists(sp) else {}
    ver = str(meta.get("protocol_version", "1.0"))
    if ver != "1.0" and not a.force:
        raise SystemExit(f"{sp}: protocol_version {ver}; cues are already logged before the beep. Nothing to fix (--force overrides).")
    c = pd.read_csv(cp)
    g = c[c.event == "go"].set_index(["block", "trial"]).t_ns
    s = c[c.event == "stop"].set_index(["block", "trial"]).t_ns
    r = c[c.event == "ready"].set_index(["block", "trial"]).t_ns
    win = float(meta.get("timing", {}).get("window_s", 3.0))
    d_gs = (s - g).dropna() * 1e-9
    d_rg = (g - r.reindex(g.index)).dropna() * 1e-9
    print(f"{len(g)} trials. logged GO->STOP median {d_gs.median():.3f} s (window_s {win}); "
          f"=> beep+overhead estimate {d_gs.median() - win:.3f} s (shift used: {a.shift:.3f})")
    print(f"logged READY->GO median {d_rg.median():.3f} s, IQR {d_rg.quantile(.25):.3f}-{d_rg.quantile(.75):.3f}")
    if abs(d_gs.median() - win - a.shift) > 0.03:
        print("WARNING: estimate and shift differ by more than 30 ms; check --shift and session.json timing")
    m = c.event.isin(["go", "stop"])
    c.loc[m, "t_ns"] = c.loc[m, "t_ns"] - int(round(a.shift * 1e9))
    # sanity: per trial ready < go < stop
    k = c.pivot_table(index=["block", "trial"], columns="event", values="t_ns", aggfunc="first").dropna(subset=["ready", "go", "stop"])
    assert ((k.ready < k.go) & (k.go < k.stop)).all(), "ordering ready < go < stop violated after the shift"
    c.to_csv(os.path.join(a.dir, a.out), index=False)
    print("wrote", os.path.join(a.dir, a.out), f"(go/stop moved {a.shift * 1e3:.0f} ms earlier)")


if __name__ == "__main__":
    main()
