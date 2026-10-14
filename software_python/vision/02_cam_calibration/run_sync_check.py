"""Map one camera to the device clock from the LED bursts and report the error budget (run locally).

    python run_sync_check.py --imu run01_imu.csv --video run01_tb.avi --ts run01_tb_t.csv --plot sync.png --debug dbg

The LED is located by lock-in detection: every pixel is correlated with the known blink pattern, so a
moving hand or auto-exposure cannot be mistaken for it. --debug DIR writes burst<N>_roi.png (frame with the
LED ON + ROI box | ON-OFF difference | ROI crop): look at them. --roi x y w h forces a region.
"""
import argparse
import json
import os
import sys
import time

import numpy as np
import pandas as pd

import sync_tools as st


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--imu", required=True)
    ap.add_argument("--video", required=True)
    ap.add_argument("--ts", required=True)
    ap.add_argument("--roi", type=int, nargs=4, default=None)
    ap.add_argument("--plot", default=None)
    ap.add_argument("--debug", default=None, help="folder for ROI debug images")
    ap.add_argument("--min-corr", type=float, default=0.5)
    ap.add_argument("--no-dejitter", action="store_true", help="use raw frame-arrival stamps")
    ap.add_argument("--json", default=None, help="save the calibration result (for sync_stability.py)")
    ap.add_argument("--cfg", default=None, help="cam_capture config json (default: <ts prefix>_cfg.json if present)")
    a = ap.parse_args()

    df = pd.read_csv(a.imu)
    t_dev, keep = st.unwrap_device_us(df.device_us)
    led, rx = df.sync_led.values[keep], df.rx_ns.values[keep] * 1e-9
    dev_r, dev_f = st.device_led_edges(t_dev, led)
    g = st.group_bursts(dev_r)
    ah, bh, delay = st.lower_envelope_clock(t_dev, rx)             # device -> host (fastest delivery)
    ts = pd.read_csv(a.ts).t_ns.values * 1e-9
    if not a.no_dejitter:
        raw = ts
        ts, n_drop = st.dejitter_cam_times(raw)
        print(f"camera stamps de-jittered: {np.std(raw - ts) * 1e3:.2f} ms arrival jitter removed, "
              f"{1 / np.median(np.diff(ts)):.3f} fps, estimated dropped frames {n_drop}")
    print(f"device: {len(dev_r)} rising edges in {g.max() + 1} bursts; camera frames: {len(ts)}")
    import cv2
    nfr = int(cv2.VideoCapture(a.video).get(cv2.CAP_PROP_FRAME_COUNT))
    if nfr != len(ts):
        print(f"WARNING: video has {nfr} frames but timestamp file has {len(ts)} rows; frame index mapping is unsafe")

    # ON intervals on the host clock
    on_dev = np.array([(r, dev_f[dev_f > r][0]) for r in dev_r if (dev_f > r).any()])
    on_host = ah + bh * on_dev
    state = st.expected_state_fn(on_host)
    if a.debug:
        os.makedirs(a.debug, exist_ok=True)

    cam_r, cam_f, trace0, weak, good_b = [], [], None, 0, []
    for b in np.unique(g):
        sel = g == b
        r_b = dev_r[sel]
        f_b = dev_f[dev_f > r_b[-1]]
        h0, h1 = ah + bh * r_b[0], ah + bh * (f_b[0] if len(f_b) else r_b[-1] + 0.2)
        win = (h0 - 0.5, h1 + 1.2)
        if a.roi:
            roi, info = tuple(a.roi), dict(corr=float("nan"), lag=float("nan"))
        else:
            roi, info = st.find_led_roi(a.video, ts, (h0 - 1.0, h1 + 1.5), state)
        tI, I, roi = st.led_trace(a.video, ts, win, roi=roi)
        lo, hi = (None, None) if np.isnan(info["lag"]) else st.trace_levels(tI, I, state, info["lag"])
        r, f = st.edges_from_trace(tI, I, lo=lo, hi=hi)
        contrast = (hi - lo) if lo is not None else np.percentile(I, 99) - np.median(I)
        ok = bool(a.roi) or (info["corr"] >= a.min_corr and info["corr"] >= 1.25 * info["null"])
        weak += 0 if ok else 1
        print(f"  burst {b}: ROI {tuple(int(v) for v in roi)}  lock-in corr {info['corr']:.2f} (chance {info.get('null', float('nan')):.2f}) at lag {info['lag'] * 1e3:.0f} ms"
              f"{'' if ok else '  <-- WEAK: LED probably not visible here'}  | device rising {int(sel.sum())}  "
              f"camera rising {len(r)}  contrast {contrast:.0f}")
        if ok:
            good_b.append(int(b))
            cam_r += list(r); cam_f += list(f)
            trace0 = trace0 or (tI, I)
        if a.debug:
            st.save_roi_debug(os.path.join(a.debug, f"burst{b}_roi.png"), a.video, ts, on_host[sel], roi,
                              0.0 if np.isnan(info["lag"]) else info["lag"])
    cam_r, cam_f = np.sort(cam_r), np.sort(cam_f)
    print(f"usable bursts: {good_b} of {list(range(int(g.max()) + 1))}")
    if good_b and (min(good_b) != 0 or max(good_b) != int(g.max())):
        print("WARNING: the usable bursts do not bracket the trial (first and last burst needed): the clock slope is "
              "weakly constrained and any time outside the usable bursts is extrapolated.")
    if len(cam_r) < 6:
        print("\nToo few LED edges found to fit a clock. Check the debug images (--debug dbg): is the LED in view, "
              "not saturated, and not occluded? Lower exposure, move the LED toward the camera, or pass --roi.")
        sys.exit(1)
    pc, pdv = st.match_edges(cam_r, dev_r)
    print(f"matched {len(pc)} of {len(dev_r)} device edges" + ("" if len(pc) >= 0.8 * len(cam_r) else
          "  <-- many camera edges unmatched: spurious detections"))
    if len(pc) < 6:
        print("Not enough matched edges for a fit.")
        sys.exit(1)
    m = st.fit_clock(pc, pdv)
    res = (m["cam_to_dev"](pc) - pdv) * 1e3
    print(f"\nfit: single-edge scatter rms {m['rms'] * 1e3:.1f} ms | sigma_event(offset) {m['sigma_a'] * 1e3:.2f} ms | "
          f"drift {m['drift_ppm']:+.0f} ppm (sigma {m['sigma_b_ppm']:.0f})")
    loo, pooled = st.leave_one_burst_out(pc, pdv)
    for o in loo:
        print(f"  leave-out burst {o['burst']} ({o['kind']}): mean error {o['bias_ms']:+.1f} ms, scatter {o['rms_ms']:.1f} ms")
    ints = [o["bias_ms"] for o in loo if o["kind"] == "interior"]
    if ints:
        print(f"  sigma_total: interior {np.sqrt(np.mean(np.square(ints))):.1f} ms")
    elif loo:
        print(f"  sigma_total (all, extrapolated): {pooled:.1f} ms")
    else:
        print("  leave-one-burst-out needs >= 3 bursts with matched edges")
    lag_sym = delta = None
    if len(cam_f) and len(dev_f):
        pfc, pfd = st.match_edges(cam_f, dev_f)
        if len(pfc) >= 6:
            lat_r = (pc - (ah + bh * pdv)).mean()
            lat_f = (pfc - (ah + bh * pfd)).mean()
            lag_sym, delta = (lat_r + lat_f) / 2 * 1e3, (lat_f - lat_r) / 2 * 1e3
            msym = st.fit_clock_sym(pc, pdv, pfc, pfd)
            print(f"edge bias: falling edges are seen {2 * delta:+.1f} ms later than rising ones relative to the device "
                  f"(n={len(pfc)}): rising detected {delta:.1f} ms early, falling {delta:.1f} ms late. "
                  + ("Typical of a SATURATING LED (dim it / shorten exposure)." if abs(delta) > 5 else "Small."))
            print(f"symmetric fit (unbiased): sigma_event {msym['sigma_a'] * 1e3:.2f} ms, delta {msym['delta'] * 1e3:.1f} +- "
                  f"{msym['sigma_delta'] * 1e3:.1f} ms; effective lag (average of rising and falling) {lag_sym:.1f} ms")
    print(f"\nIMU reception delay (host rx - fitted device clock): p50 {np.percentile(delay, 50) * 1e3:.1f} ms, "
          f"p99 {np.percentile(delay, 99) * 1e3:.1f} ms; device/host slope {1e6 * (bh - 1):+.1f} ppm")
    lat = (pc - (ah + bh * pdv)) * 1e3
    print(f"camera timestamp latency from rising edges only (host stamp minus LED toggle, rel. to fastest IMU delivery): "
          f"mean {lat.mean():.1f} ms, std {lat.std():.1f} ms" + ("" if lag_sym is None else f"  -> use the symmetric value {lag_sym:.1f} ms"))
    if a.json:
        cfg_path = a.cfg or (a.ts[:-6] + "_cfg.json" if a.ts.endswith("_t.csv") else None)
        cfg = json.load(open(cfg_path)) if cfg_path and os.path.exists(cfg_path) else None
        json.dump(dict(time=time.strftime("%Y-%m-%d %H:%M:%S"), imu=a.imu, video=a.video, cfg=cfg,
                       lag_ms=float(lag_sym if lag_sym is not None else lat.mean()), lag_rise_ms=float(lat.mean()),
                       edge_delta_ms=(None if delta is None else float(delta)), lag_std_ms=float(lat.std()),
                       sigma_event_ms=float(m["sigma_a"] * 1e3), edge_scatter_ms=float(m["rms"] * 1e3),
                       sigma_total_interior_ms=(float(np.sqrt(np.mean(np.square(ints)))) if ints else None),
                       drift_ppm=float(m["drift_ppm"]), n_matched=int(len(pc)), n_device_edges=int(len(dev_r)),
                       usable_bursts=good_b, n_bursts=int(g.max()) + 1, dejitter=not a.no_dejitter,
                       fps=float(1 / np.median(np.diff(ts)))), open(a.json, "w"), indent=1)
        print("saved", a.json)
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 3, figsize=(15, 3.8))
        tI, I = trace0
        ax[0].plot(tI - tI[0], I, lw=0.8)
        inv = lambda td: m["t0"] + (td - m["a"]) / m["b"]
        for e in dev_r[g == g[np.isin(dev_r, pdv)][0]]:
            ax[0].axvline(inv(e) - tI[0], color="r", ls="--", lw=0.6)
        ax[0].set(title="LED trace, first good burst (red: device edges mapped)", xlabel="s", ylabel="brightness")
        ax[1].scatter(pdv - pdv[0], res, c=g[np.isin(dev_r, pdv)] if len(res) == np.isin(dev_r, pdv).sum() else "C0", s=14)
        ax[1].axhline(0, color="k", lw=0.5)
        ax[1].set(title="fit residual per edge", xlabel="device s", ylabel="ms")
        ax[2].hist(delay * 1e3, bins=60)
        ax[2].set(title="IMU reception delay", xlabel="ms")
        fig.tight_layout()
        fig.savefig(a.plot, dpi=130)
        print("saved", a.plot)


if __name__ == "__main__":
    main()