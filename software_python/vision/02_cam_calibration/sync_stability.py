"""Is the camera latency stable enough to reuse in later sessions without LED bursts?

    python sync_stability.py sync01.json sync02.json sync03.json

Groups run_sync_check.py --json results by camera configuration (backend, size, fps, exposure) and reports the
spread of the effective lag (camera stamp minus LED toggle, relative to the fastest IMU delivery). That lag
mixes camera latency with the BLE/USB minimum delay of the IMU link, so re-connect the glove between runs.
"""
import json
import sys
from collections import defaultdict

import numpy as np


def key(j):
    c = j.get("cfg") or {}
    rq = c.get("requested", {})
    return (c.get("backend", "?"), c.get("cam") or c.get("index", "?"), rq.get("w"), rq.get("h"), rq.get("fps"), rq.get("exposure"))


def main(paths, tol_ms=5.0):
    groups = defaultdict(list)
    for p in paths:
        j = json.load(open(p))
        j["_p"] = p
        groups[key(j)].append(j)
    for k, js in groups.items():
        lag = np.array([j["lag_ms"] for j in js])
        sig = np.array([j.get("sigma_total_interior_ms") or j["sigma_event_ms"] for j in js])
        print(f"\nconfig {dict(zip(('backend', 'camera', 'w', 'h', 'fps', 'exposure'), k))}: {len(js)} run(s)")
        for j in js:
            print(f"  {j['_p']}: lag {j['lag_ms']:.1f} ms, in-run std {j['lag_std_ms']:.1f}, sigma {j.get('sigma_total_interior_ms') or j['sigma_event_ms']:.1f} ms, "
                  f"edges {j['n_matched']}/{j['n_device_edges']}, usable bursts {j['usable_bursts']}/{j['n_bursts']}, drift {j['drift_ppm']:+.0f} ppm")
        if len(js) < 3:
            print("  -> need at least 3 runs (re-connect the glove each time) before judging stability.")
            continue
        spread = lag.std(ddof=1)
        within = float(np.sqrt(np.mean(sig ** 2)))
        verdict = ("STABLE: the lag may be reused (still verify with one visible burst per session)"
                   if spread <= tol_ms and spread <= 2 * within + 1 else
                   "NOT STABLE: calibrate every session, or find what changes (BLE connection, exposure, load)")
        print(f"  lag mean {lag.mean():.1f} ms, run-to-run std {spread:.1f} ms (range {lag.min():.1f}..{lag.max():.1f}), "
              f"typical within-run sigma {within:.1f} ms -> {verdict}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
