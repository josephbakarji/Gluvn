"""W2 sync tools: map camera time -> device clock (micros) using firmware LED events.

Design: the device clock is the master. Offline analysis stays in that domain, so the host
clock is only needed for diagnostics (reception delay) and live use.

  device edges : `sync_led` bit carried in imu2's LSB (exact device_us of each toggle)
  camera edges : sub-frame 0.5-crossings of an LED brightness trace
  mapping      : t_dev = a + b * (t_cam - t0), robust affine fit over matched rising edges
"""
import csv
import queue
import subprocess
import threading

import numpy as np
from scipy.optimize import least_squares, linprog


# ----------------------------------------------------------------------------
# Device side
# ----------------------------------------------------------------------------
def unwrap_device_us(us):
    """uint32 micros -> unwrapped seconds. Drops stale duplicates (firmware re-sends the
    last good sample when M5.Imu.update() fails, so device_us repeats). Returns (t, keep)."""
    us = np.asarray(us, dtype=np.int64)
    d = np.diff(us)
    d[d < -2**31] += 2**32
    keep = np.concatenate([[True], d != 0])
    t = (us[0] + np.concatenate([[0], np.cumsum(d)])) * 1e-6
    return t[keep], keep


def device_led_edges(t, led, max_gap_factor=3.5, return_idx=False):
    """(rising, falling) device-time edges. Edges that follow a long frame gap are discarded
    because their timestamp is late by up to the gap length (default 3.5 ticks = 28 ms; the
    device loop stalls for 24 ms regularly, so a tighter limit would discard real edges).
    With return_idx=True also returns the frame indices of the rising and falling edges."""
    led = np.asarray(led, int)
    dt = np.median(np.diff(t))
    i = np.flatnonzero(np.diff(led)) + 1
    i = i[(t[i] - t[i - 1]) < max_gap_factor * dt]
    r, f = i[led[i] == 1], i[led[i] == 0]
    return (t[r], t[f], r, f) if return_idx else (t[r], t[f])


# ----------------------------------------------------------------------------
# Camera side
# ----------------------------------------------------------------------------
def pts_from_ffprobe(path):
    """Per-frame presentation times (s) of a video file; phones record variable fps."""
    for key in ("pts_time", "best_effort_timestamp_time"):
        out = subprocess.run(
            ["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
             f"frame={key}", "-of", "csv=p=0", path],
            capture_output=True, text=True).stdout.split()
        vals = [float(v.strip(",")) for v in out if v.strip(",") not in ("", "N/A")]
        if len(vals) > 10:
            return np.sort(np.array(vals))
    raise RuntimeError("ffprobe returned no frame timestamps")


def edges_from_trace(t, I, lo=None, hi=None):
    """Sub-frame 0.5-crossings of a brightness trace (use a window with ~20-40 % ON time).
    Returns (rising, falling) in the units of t."""
    t, I = np.asarray(t, float), np.asarray(I, float)
    lo = np.median(I) if lo is None else lo
    hi = np.percentile(I, 99) if hi is None else hi
    u = np.clip((I - lo) / max(hi - lo, 1e-9), 0, 1)
    s = u >= 0.5
    i = np.flatnonzero(s[1:] != s[:-1]) + 1
    frac = (0.5 - u[i - 1]) / (u[i] - u[i - 1])
    te = t[i - 1] + frac * (t[i] - t[i - 1])
    return te[s[i]], te[~s[i]]


def expected_state_fn(on_intervals):
    """on_intervals: (n,2) host-clock [start, end] of each LED ON period -> f(t) giving 0/1."""
    on = np.asarray(on_intervals, float)
    starts, ends = on[:, 0], on[:, 1]

    def f(t):
        t = np.asarray(t, float)
        i = np.searchsorted(starts, t, side="right") - 1
        return ((i >= 0) & (t < ends[np.clip(i, 0, None)])).astype(np.float32)
    return f


def find_led_roi(video_path, frame_t, t_window, state_fn, lags=None, scale=2, half=12):
    """Lock-in LED finder. The firmware blink pattern is known, so correlate every pixel's brightness
    with the expected ON/OFF waveform (scanned over camera lags) and take the best pixel. Unlike a
    'largest temporal variance' pick, moving hands, auto-exposure and noise do not follow the
    pattern. Returns (roi, info) with info = corr (max correlation), lag (s), score_map, scale.
    info['null'] is the best correlation any pixel reaches with the TIME-REVERSED pattern (chance level);
    trust the LED only when corr is clearly above it. lag may be negative: it is (camera stamp - mapped LED
    time), and the mapping uses the fastest IMU delivery, so it is camera latency minus IMU path delay."""
    import cv2
    lags = np.arange(-0.50, 0.60, 1 / 60) if lags is None else np.asarray(lags)
    frame_t = np.asarray(frame_t)
    k = np.flatnonzero((frame_t >= t_window[0]) & (frame_t <= t_window[1]))
    cap = cv2.VideoCapture(video_path)
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(k[0]))
    G = []
    for _ in k:
        ok, f = cap.read()
        if not ok:
            break
        g = f.max(axis=2)
        G.append(cv2.resize(g, (g.shape[1] // scale, g.shape[0] // scale), interpolation=cv2.INTER_AREA))
    cap.release()
    k = k[:len(G)]
    U = np.stack(G)
    T, H, W = U.shape
    U = U.reshape(T, -1)
    tk = frame_t[k]

    def pattern(rev):
        P = np.stack([state_fn((tk[0] + tk[-1] - tk if rev else tk) - lag) for lag in lags], axis=1)
        P = P - P.mean(0)
        n_ = np.linalg.norm(P, axis=0)
        return P / np.maximum(n_, 1e-9), n_ > 1e-6

    S, good = pattern(False)
    if not good.any():
        raise RuntimeError("expected LED state is constant inside this window")
    S, lags = S[:, good], lags[good]
    Sn, _ = pattern(True)                  # time-reversed pattern: same statistics, wrong order -> null level
    Sn = Sn[:, good]
    score = np.empty(U.shape[1], np.float32)
    best = np.empty(U.shape[1], np.int16)
    nullmap = np.empty(U.shape[1], np.float32)
    for s0 in range(0, U.shape[1], 60000):
        X = U[:, s0:s0 + 60000].astype(np.float32)
        X -= X.mean(0)
        nx = np.linalg.norm(X, axis=0)[:, None] + 1e-3
        C = (X.T @ S) / nx
        best[s0:s0 + 60000], score[s0:s0 + 60000] = C.argmax(1), C.max(1)
        nullmap[s0:s0 + 60000] = ((X.T @ Sn) / nx).max(1)
    sm = score.reshape(H, W)       # no smoothing: a blur would erase a 1-2 px LED
    y, x = np.unravel_index(np.argmax(sm), sm.shape)
    cx, cy = int(x * scale + scale // 2), int(y * scale + scale // 2)
    roi = (max(cx - half, 0), max(cy - half, 0), 2 * half, 2 * half)
    yy, xx = np.mgrid[0:H, 0:W]
    far = ((yy - y) ** 2 + (xx - x) ** 2 > 8 ** 2).ravel()      # exclude the LED's own neighbourhood
    null = float(nullmap[far].max())
    return roi, dict(corr=float(score[y * W + x]), lag=float(lags[best[y * W + x]]), null=null, score_map=sm, scale=scale)


def save_roi_debug(path, video_path, frame_t, on_intervals, roi, lag):
    """PNG: frame while the LED should be ON (ROI boxed) | |ON - OFF| difference x4 | ROI crop x8."""
    import cv2
    frame_t = np.asarray(frame_t)
    on = np.asarray(on_intervals, float)
    t_on = on[0].mean() + lag
    t_off = (on[0, 1] + (on[1, 0] if len(on) > 1 else on[0, 1] + 0.3)) / 2 + lag
    cap = cv2.VideoCapture(video_path)

    def grab(t):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(np.argmin(np.abs(frame_t - t))))
        return cap.read()[1]
    f_on, f_off = grab(t_on), grab(t_off)
    cap.release()
    x, y, w, h = roi
    diff = np.clip(cv2.absdiff(f_on, f_off).astype(np.int32) * 4, 0, 255).astype(np.uint8)
    vis = f_on.copy()
    cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), max(2, f_on.shape[1] // 400))
    crop = cv2.resize(f_on[y:y + h, x:x + w], (w * 8, h * 8), interpolation=cv2.INTER_NEAREST)
    H = 540
    rs = lambda im: cv2.resize(im, (int(im.shape[1] * H / im.shape[0]), H))
    cv2.imwrite(path, np.hstack([rs(vis), rs(diff), rs(crop)]))


def trace_levels(t, I, state_fn, lag, guard=0.05):
    """OFF/ON brightness from frames well inside known OFF/ON periods (median), so a hand or shadow crossing
    the ROI cannot set the normalisation the way a percentile of the whole trace does."""
    t = np.asarray(t)
    on = (state_fn(t - lag - guard) > 0) & (state_fn(t - lag + guard) > 0)
    off = (state_fn(t - lag - guard) == 0) & (state_fn(t - lag + guard) == 0)
    if on.sum() < 3 or off.sum() < 3:
        return None, None
    return float(np.median(I[off])), float(np.median(I[on]))


def led_trace(video_path, frame_t, t_window, roi=None, half=12, n_top=10):
    """Brightness of the LED inside t_window=(t0,t1) at `roi` (x,y,w,h). Without a roi it falls back to the
    pixel with the largest temporal std (only reliable if nothing else moves). Only the ROI crop is kept
    in memory. Returns (t, I, roi)."""
    import cv2
    frame_t = np.asarray(frame_t)
    k = np.flatnonzero((frame_t >= t_window[0]) & (frame_t <= t_window[1]))

    def frames():
        cap = cv2.VideoCapture(video_path)
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(k[0]))
        for _ in k:
            ok, f = cap.read()
            if not ok:
                return
            yield f.max(axis=2)
        cap.release()

    if roi is None:
        s1 = s2 = None
        n = 0
        for g in frames():
            g = g.astype(np.float32)
            s1, s2, n = (g, g * g, 1) if s1 is None else (s1 + g, s2 + g * g, n + 1)
        m = cv2.GaussianBlur(np.sqrt(np.maximum(s2 / n - (s1 / n) ** 2, 0)), (0, 0), 3)
        y, x = np.unravel_index(np.argmax(m), m.shape)
        roi = (max(x - half, 0), max(y - half, 0), 2 * half, 2 * half)
    x, y, w, h = roi
    I = [np.sort(g[y:y + h, x:x + w].ravel())[-n_top:].mean() for g in frames()]
    return frame_t[k][:len(I)], np.array(I), roi


def dejitter_cam_times(t, step_frac=0.6, win=15):
    """Replace jittery frame-ARRIVAL stamps by a regular grid t_k = a + T (k + dropped_k).

    Real webcam stamps are a steady clock plus bursty arrival jitter (observed: 29.88 fps, 5 ms
    std, running-median wander < 2 ms, lag-1 autocorrelation of dt < 0). Regressing on the frame
    index removes the jitter. A persistent residual step of n*T (n frames lost upstream) is
    detected from running medians and absorbed into the index. Returns (t_clean, n_dropped)."""
    import pandas as pd
    t = np.asarray(t, float)
    k = np.arange(len(t), dtype=float)
    corr = np.zeros(len(t))
    for _ in range(4):
        kk = k + corr
        T, a = np.polyfit(kk, t, 1)
        r = t - (a + T * kk)
        keep = np.abs(r) < 3 * (np.median(np.abs(r)) * 1.4826 + 1e-9)
        T, a = np.polyfit(kk[keep], t[keep], 1)
        r = t - (a + T * kk)
        s = pd.Series(r)
        before = s.rolling(win, min_periods=win).median().shift(1)
        after = s[::-1].rolling(win, min_periods=win).median()[::-1]
        step = (after - before).values
        cand = np.flatnonzero(np.nan_to_num(step) > step_frac * T)
        if cand.size == 0:
            break
        groups = np.split(cand, np.flatnonzero(np.diff(cand) > win) + 1)
        for g in groups:
            p = g[np.argmax(step[g])]
            corr[p:] += max(int(round(step[p] / T)), 1)
    kk = k + corr
    T, a = np.polyfit(kk, t, 1)
    return a + T * kk, int(corr[-1])


# ----------------------------------------------------------------------------
# Matching and fitting
# ----------------------------------------------------------------------------
def match_edges(cam, dev, tol=0.03):
    """Pair camera and device rising edges when the clock origins are unrelated: every pair
    votes for an offset, the random interval pattern makes one offset win."""
    cam, dev = np.asarray(cam), np.asarray(dev)
    off = np.sort((dev[None, :] - cam[:, None]).ravel())
    votes = np.searchsorted(off, off + tol) - np.searchsorted(off, off - tol)
    j = np.argmax(votes)
    coarse = np.median(off[(off >= off[j] - tol) & (off <= off[j] + tol)])
    pc, pd = [], []
    for c in cam:
        d = dev[np.argmin(np.abs(dev - (c + coarse)))]
        if abs(d - (c + coarse)) < tol and d not in pd:
            pc.append(c)
            pd.append(d)
    return np.array(pc), np.array(pd)


def fit_clock(cam, dev, f_scale=0.004, b_prior_ppm=None):
    """Robust affine map t_dev = a + b (t_cam - t0). Returns dict incl. sigma_a (s) from the
    residual covariance and a callable cam_to_dev.

    A single burst spans only seconds, so its slope b is poorly determined (~1000 ppm) and
    extrapolating over a trial is unsafe. Either bracket the trial with >=2 bursts or pass
    b_prior_ppm (e.g. 200) to pull b toward 1."""
    cam, dev = np.asarray(cam), np.asarray(dev)
    t0 = cam.mean()

    def resid(p):
        r = p[0] + p[1] * (cam - t0) - dev
        if b_prior_ppm:   # one pseudo-residual, weighted like a ~3 ms measurement
            r = np.append(r, (p[1] - 1) / (b_prior_ppm * 1e-6) * 3e-3)
        return r

    r = least_squares(resid, [np.median(dev - cam) + t0, 1.0], loss="soft_l1", f_scale=f_scale)
    res = r.fun[:len(cam)]
    dof = max(len(cam) - 2, 1)
    cov = np.linalg.inv(r.jac.T @ r.jac) * (res @ res / dof)
    a, b = r.x
    return dict(a=a, b=b, t0=t0, n=len(cam), rms=float(np.sqrt(np.mean(res**2))),
                sigma_a=float(np.sqrt(cov[0, 0])), sigma_b_ppm=float(np.sqrt(cov[1, 1]) * 1e6),
                drift_ppm=(b - 1) * 1e6, cam_to_dev=lambda t: a + b * (np.asarray(t) - t0))


def cam_to_dev_from_lag(t_cam, t_dev, rx, lag):
    """Camera stamps -> device clock with a PREVIOUSLY calibrated effective lag (no LED needed):
    t_dev = (t_cam - lag - a) / b, with (a, b) from this session's IMU lower-envelope fit.
    Valid only if the lag was shown stable (sync_stability.py); the lag contains the IMU link's minimum delay."""
    a, b, _ = lower_envelope_clock(t_dev, rx)
    return (np.asarray(t_cam, float) - lag - a) / b


def fit_clock_sym(cam_r, dev_r, cam_f, dev_f, f_scale=0.004):
    """Clock fit using rising AND falling edges with opposite detection biases +-delta.

    A saturating LED (e.g. very bright in a dark room) looks ON for longer: rising edges are detected early and
    falling edges late by the same amount. Model: dev = a + b (cam - t0) + s*delta, s = +1 rising, -1 falling.
    `a` (and cam_to_dev) are then free of that bias; delta is returned in seconds."""
    cr, dr, cf, df_ = (np.asarray(x, float) for x in (cam_r, dev_r, cam_f, dev_f))
    t0 = np.concatenate([cr, cf]).mean()

    def res(p):
        return np.concatenate([p[0] + p[1] * (cr - t0) + p[2] - dr, p[0] + p[1] * (cf - t0) - p[2] - df_])

    r = least_squares(res, [np.median(dr - cr) + t0, 1.0, 0.0], loss="soft_l1", f_scale=f_scale)
    n = len(cr) + len(cf)
    cov = np.linalg.inv(r.jac.T @ r.jac) * (r.fun @ r.fun / max(n - 3, 1))
    a, b, delta = r.x
    return dict(a=a, b=b, t0=t0, delta=delta, n=n, rms=float(np.sqrt(np.mean(r.fun ** 2))),
                sigma_a=float(np.sqrt(cov[0, 0])), sigma_delta=float(np.sqrt(cov[2, 2])),
                cam_to_dev=lambda t: a + b * (np.asarray(t) - t0))


def lower_envelope_clock(t_dev, t_host):
    """One-way-delay clock fit t_host = a + b t_dev + delay, delay >= 0 (LP, Moon et al.).
    Returns (a, b, delay_series). The delay series characterises BLE/USB reception latency."""
    t_dev, t_host = np.asarray(t_dev, float), np.asarray(t_host, float)
    m = t_dev.mean()
    x = t_dev - m
    n = len(x)
    r = linprog([-n, -x.sum()], A_ub=np.column_stack([np.ones(n), x]), b_ub=t_host,
                bounds=[(None, None), (0.9, 1.1)])
    a, b = r.x
    return a - b * m, b, t_host - (a + b * x)


def group_bursts(dev, gap=3.0):
    """Burst index per device edge: a new burst starts after a gap longer than `gap` s."""
    dev = np.asarray(dev)
    return np.concatenate([[0], np.cumsum(np.diff(dev) > gap)])


def leave_one_burst_out(cam, dev, gap=3.0, **fit_kw):
    """sigma_total: fit on all bursts but one, predict the held-out burst's device edges.

    This includes what the fit covariance cannot see (latency steps, non-linear drift,
    quantization, jitter). Interior bursts test interpolation; first/last test extrapolation.
    Score each held-out burst by its MEAN error (bias_ms): per-edge scatter is single-event
    noise (~sigma_event * sqrt(n_edges) larger than the map error) and is reported separately
    as rms_ms. Returns (list of dicts, pooled RMS of the burst biases in ms)."""
    cam, dev = np.asarray(cam), np.asarray(dev)
    g = group_bursts(dev, gap)
    out = []
    for b in np.unique(g):
        tr, te = g != b, g == b
        if len(np.unique(g[tr])) < 2:
            continue                                 # slope unconstrained without >=2 bursts
        m = fit_clock(cam[tr], dev[tr], **fit_kw)
        e = (m["cam_to_dev"](cam[te]) - dev[te]) * 1e3
        out.append(dict(burst=int(b), kind="interior" if 0 < b < g.max() else "extrapolated",
                        rms_ms=float(np.sqrt(np.mean(e**2))), bias_ms=float(e.mean())))
    pooled = float(np.sqrt(np.mean([o["bias_ms"] ** 2 for o in out]))) if out else float("nan")
    return out, pooled


# ----------------------------------------------------------------------------
# Live logger (tap into ParseSerial.parse via parser.taps)
# ----------------------------------------------------------------------------
class FrameLogger:
    """Non-blocking CSV logger: `parser.taps.append(FrameLogger('run.csv'))`.

    Call close() when the session ends (also registered with atexit): the writer thread is a
    daemon, so without this the last buffered rows are lost (a real run lost its final 1.8 s)."""
    TUPLES = ("imu", "flex", "press", "quat", "nav_quat", "velocity", "position", "lin_accel")
    _STOP = object()

    def __init__(self, path):
        import atexit
        self.q = queue.SimpleQueue()
        self.path = path
        self._th = threading.Thread(target=self._run, daemon=True)
        self._th.start()
        atexit.register(self.close)

    def __call__(self, frame):
        self.q.put(frame)

    def close(self, timeout=5.0):
        if self._th.is_alive():
            self.q.put(self._STOP)
            self._th.join(timeout)

    @classmethod
    def _flat(cls, f):
        out = {k: v for k, v in f.items() if k not in cls.TUPLES}
        for k in cls.TUPLES:
            for i, v in enumerate(f.get(k, ())):
                out[f"{k}_{i}"] = v
        return out

    def _run(self):
        with open(self.path, "w", newline="") as fh:
            w = None
            n = 0
            while True:
                item = self.q.get()
                if item is self._STOP:
                    fh.flush()
                    return
                row = self._flat(item)
                if w is None:
                    w = csv.DictWriter(fh, fieldnames=list(row), restval="", extrasaction="ignore")
                    w.writeheader()
                w.writerow(row)
                n += 1
                if n % 50 == 0:
                    fh.flush()


# ----------------------------------------------------------------------------
# Synthetic self-test: recover a known clock map from simulated LED bursts
# ----------------------------------------------------------------------------
def _simulate(rng, n_bursts=3, pulses=8, fps=30.0, t_exp=0.015, eps=25e-6, latency=0.055,
              cam_jit=0.002, noise=0.02, span=100.0, latency_step=0.0):
    dev_r, dev_f = [], []
    for b in range(n_bursts):
        e = 100.0 + b * span / max(n_bursts - 1, 1)
        for _ in range(pulses):
            e += rng.uniform(0.25, 0.65); dev_r.append(e)
            e += rng.uniform(0.10, 0.16); dev_f.append(e)
    dev_r, dev_f = np.array(dev_r), np.array(dev_f)
    tau = 95.0 + rng.uniform(0, 1 / fps) + np.arange(0, span + 15, 1 / fps)   # true mid-exposure
    c0 = rng.uniform(1e3, 1e5)
    t_cam = c0 + (1 + eps) * tau + latency + rng.normal(0, cam_jit, tau.size)
    t_cam = t_cam + latency_step * (tau > 100.0 + span / 4)   # latency jumps after burst 1
    lo, hi = tau[:, None] - t_exp / 2, tau[:, None] + t_exp / 2
    on = np.clip(np.minimum(hi, dev_f[None, :]) - np.maximum(lo, dev_r[None, :]), 0, None).sum(1) / t_exp
    I = 20 + 200 * on + rng.normal(0, noise * 200, tau.size)
    truth = lambda tc: (tc - c0 - latency) / (1 + eps)       # t_dev = truth(t_cam) (before any step)
    return t_cam, I, dev_r, truth


def selftest(trials=300, prior=None, dejitter=False, **kw):
    rng = np.random.default_rng(0)
    errs, sig, loo_i, loo_e = [], [], [], []
    for _ in range(trials):
        t_cam, I, dev_r, truth = _simulate(rng, **kw)
        tc = dejitter_cam_times(t_cam)[0] if dejitter else t_cam
        cam_r, _ = edges_from_trace(tc, I)
        pc, pd = match_edges(cam_r, dev_r)
        m = fit_clock(pc, pd, b_prior_ppm=prior)
        tt = np.linspace(t_cam[0] + 5, t_cam[-1] - 5, 200)
        errs.append(m["cam_to_dev"](tt) - truth(tt))
        sig.append(m["sigma_a"])
        res, _ = leave_one_burst_out(pc, pd, b_prior_ppm=prior)
        loo_i += [r["bias_ms"] for r in res if r["kind"] == "interior"]
        loo_e += [r["bias_ms"] for r in res if r["kind"] == "extrapolated"]
    e = np.array(errs) * 1e3
    return dict(rms_ms=float(np.sqrt((e**2).mean())), p95_ms=float(np.percentile(np.abs(e), 95)),
                reported_sigma_a_ms=float(np.median(sig) * 1e3),
                loo_interior_ms=float(np.sqrt(np.mean(np.square(loo_i)))) if loo_i else None,
                loo_extrapolated_ms=float(np.sqrt(np.mean(np.square(loo_e)))) if loo_e else None)


if __name__ == "__main__":
    for label, kw in [("30 fps, 15 ms exposure, 3 bursts x 8", {}),
                      ("30 fps, 33 ms exposure, 3 bursts x 8", dict(t_exp=0.033)),
                      ("30 fps, 15 ms exposure, 1 burst x 8", dict(n_bursts=1)),
                      ("60 fps, 8 ms exposure, 3 bursts x 8", dict(fps=60.0, t_exp=0.008)),
                      ("30 fps, 15 ms exposure, 1 burst x 8 + 200 ppm prior", dict(n_bursts=1, prior=200)),
                      ("30 fps, 15 ms exposure, 2 bursts x 8 (bracketing)", dict(n_bursts=2)),
                      ("30 fps, 15 ms exposure, 3 bursts x 8, 100 ppm", dict(eps=100e-6)),
                      ("3 bursts, 8 ms latency step after burst 1", dict(latency_step=0.008))]:
        print(f"{label:52s}", selftest(**kw))