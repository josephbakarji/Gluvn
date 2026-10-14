"""Turn a recorded session (run_imu.csv + cues.csv + session.json) into gesture segments.

Works with what the logger actually contains today: Mahony Euler (imu_0..2, decoded exactly as
pose_provider.decode_mahony_euler) and raw accelerometer (imu_3..5). NavEKF quaternion and lin_accel are
used only if the EKF was armed during the session; otherwise gravity-free acceleration is rebuilt as
    a_W = (s * R f_body - z_hat) * g,      s = 1 / median|f| over the static holds,
where R = Rz(yaw) Ry(pitch) Rx(roll) is the firmware attitude (world <- sensor).

Segments are resampled to a uniform rate (the device loop stalls for 24 ms and then catches up with
3-7 ms steps, so raw timestamps are irregular).
"""
import json
import os

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot, Slerp

G = 9.80665


def decode(df):
    """Frame table -> arrays. Drops frames whose device_us repeats (stale IMU sample)."""
    us = df.device_us.values.astype(np.int64)
    d = np.diff(us)
    d[d < -2**31] += 2**32
    keep = np.concatenate([[True], d != 0])
    df = df[keep].reset_index(drop=True)
    us = us[keep]
    t = np.concatenate([[0], np.cumsum(np.diff(us) % 2**32)]) * 1e-6
    yaw = df.imu_0.values * 180 / 32767 - 180
    pitch = df.imu_1.values * 180 / 32767 - 90
    roll = df.imu_2.values * 180 / 32767 - 180
    R = Rot.from_euler("ZYX", np.stack([yaw, pitch, roll], 1), degrees=True)
    f = np.stack([df.imu_3.values, df.imu_4.values, df.imu_5.values], 1) / 16383.5 - 2.0   # g (LSB of imu_5 = sync bit)
    return dict(t=t, rx=df.rx_ns.values, R=R, f=f, df=df, led=df.sync_led.values)


def accel_scale_from_holds(S, cues):
    """s = 1/median|f| over the static holds (measured: |f| ~ 0.90-0.93 g at rest, i.e. a ~8 % scale error)."""
    norms = []
    for _, h in cues[cues.event == "hold_start"].iterrows():
        he = cues[(cues.event == "hold_end") & (cues.block == h.block)].t_ns.values[0]
        m = (S["rx"] >= h.t_ns + 1e9) & (S["rx"] <= he)
        if m.sum() > 20:
            norms.append(np.median(np.linalg.norm(S["f"][m], axis=1)))
    return 1.0 / np.median(norms) if norms else 1.0


def resample(t, R, cols, fs):
    tu = t[0] + np.arange(int((t[-1] - t[0]) * fs)) / fs
    q = Slerp(t, R)(tu).as_quat()[:, [3, 0, 1, 2]]
    out = {k: np.column_stack([np.interp(tu, t, v[:, j]) for j in range(v.shape[1])]) for k, v in cols.items()}
    return tu - tu[0], q, out


def _activity(q, flex, fs):
    """Angular speed (rad/s) and finger activity on a uniform grid, low-passed at 4 Hz."""
    from scipy.signal import butter, sosfiltfilt
    R = Rot.from_quat(q[:, [1, 2, 3, 0]])
    w = np.linalg.norm((R[:-1].inv() * R[1:]).as_rotvec(), axis=1) * fs
    sos = butter(2, 4.0, fs=fs, output="sos")
    fa = np.abs(np.diff(sosfiltfilt(sos, flex, axis=0), axis=0)).sum(1) * fs
    pad = lambda x: np.append(x, x[-1])
    return sosfiltfilt(sos, pad(w)), pad(fa)


def find_burst(wn, fa, noise_w, noise_f, fs, merge=0.4, pad=0.25, min_len=0.8, max_len=4.0):
    """Main activity burst in a trial window: threshold = 3x the noise level measured in the static holds;
    gaps shorter than `merge` s are bridged; the burst with the largest integrated activity wins."""
    act = np.maximum(wn / max(3 * noise_w, 0.15), fa / max(3 * noise_f, 5.0))
    on = act > 1.0
    idx = np.flatnonzero(on)
    if len(idx) == 0:
        return None
    cuts = np.flatnonzero(np.diff(idx) > merge * fs)
    starts = np.concatenate([[idx[0]], idx[cuts + 1]])
    ends = np.concatenate([idx[cuts], [idx[-1]]])
    k = np.argmax([act[a:b + 1].sum() for a, b in zip(starts, ends)])
    a, b = starts[k], ends[k]
    a, b = max(0, a - int(pad * fs)), min(len(act) - 1, b + int(pad * fs))
    if (b - a) / fs < min_len:
        c = (a + b) // 2
        a, b = max(0, c - int(min_len * fs / 2)), min(len(act) - 1, c + int(min_len * fs / 2))
    if (b - a) / fs > max_len:
        c = a + int(np.argmax(act[a:b + 1]))
        a, b = max(0, c - int(max_len * fs / 2)), min(len(act) - 1, c + int(max_len * fs / 2))
    return a, b


def load_session(path_imu, path_cues, path_json=None, fs=100.0, use_nav=False, segmentation="activity",
                 pre_pad=0.8, post_pad=0.2):
    """segmentation: 'activity' (default) finds the main burst inside each ready..next-ready trial window;
    'cue' uses the fixed GO..STOP window (reliable only if the performer follows the beeps exactly).
    'padded' is 'activity' with the burst search restricted to [GO - pre_pad, STOP + post_pad]. It is label-agnostic
    (cue times are the same protocol for every gesture; the gesture label is never used), and it keeps the return-to-rest
    motion after STOP from being selected as 'the' burst.
    'window' is a deterministic cue-locked window [GO - pre_pad, STOP + post_pad] with NO burst detection (also label-agnostic).
    Pass pre_pad/post_pad explicitly for it; the 0.8/0.2 defaults belong to 'padded'."""
    df = pd.read_csv(path_imu)
    cues = pd.read_csv(path_cues)
    S = decode(df)
    s = accel_scale_from_holds(S, cues)
    aW = (np.einsum("nij,nj->ni", S["R"].as_matrix(), S["f"] * s) - np.array([0, 0, 1.0])) * G
    t0_host = cues[cues.event == "hold_start"].t_ns.min()
    FL = S["df"][[f"flex_{i}" for i in range(5)]].values.astype(float)
    PR = S["df"][[f"press_{i}" for i in range(5)]].values.astype(float)

    def grid(m):
        return resample(S["t"][m], S["R"][m], dict(a=aW[m], flex=FL[m], press=PR[m]), fs)

    # noise floor of the activity measures from the static holds
    nw, nf = [], []
    for _, h in cues[cues.event == "hold_start"].iterrows():
        he = cues[(cues.event == "hold_end") & (cues.block == h.block)].t_ns.values[0]
        m = (S["rx"] >= h.t_ns + 1e9) & (S["rx"] <= he)
        if m.sum() > 100:
            _, q, rs = grid(m)
            w_, f_ = _activity(q, rs["flex"], fs)
            win = int(2 * fs)
            starts = range(0, len(w_) - win, int(0.25 * fs))
            qi = min(starts, key=lambda i: np.percentile(w_[i:i + win], 95))   # quietest 2 s
            nw.append(np.percentile(w_[qi:qi + win], 95)); nf.append(np.percentile(f_[qi:qi + win], 95))
    noise_w, noise_f = (min(nw), min(nf)) if nw else (0.05, 5.0)

    segs, readies = [], cues[cues.event == "ready"]
    for _, g in cues[cues.event == "go"].iterrows():
        key = (cues.block == g.block) & (cues.trial == g.trial)
        rd = cues[key & (cues.event == "ready")].t_ns.values[0]
        st = cues[key & (cues.event == "stop")].t_ns.values[0]
        nxt = cues[(cues.t_ns > st) & cues.event.isin(["ready", "hold_start"])].t_ns.min()
        lo, hi = ((rd, nxt) if segmentation in ("activity", "padded") else
                  (g.t_ns - pre_pad * 1e9, st + post_pad * 1e9) if segmentation == "window" else (g.t_ns, st))
        m = (S["rx"] >= lo) & (S["rx"] <= hi)
        if m.sum() < 30:
            continue
        tu, q, rs = grid(m)
        inside = np.nan
        if segmentation in ("activity", "padded"):
            w_, f_ = _activity(q, rs["flex"], fs)
            tt = np.arange(len(tu)) / fs
            go_lo, go_hi = (g.t_ns - lo) * 1e-9, (st - lo) * 1e-9
            if segmentation == "padded":
                mk = (tt >= go_lo - pre_pad) & (tt <= go_hi + post_pad)
                w_, f_ = w_ * mk, f_ * mk
            b = find_burst(w_, f_, noise_w, noise_f, fs)
            if b is None:
                continue
            a0, b0 = b
            inside = float(np.mean((tt[a0:b0 + 1] >= go_lo) & (tt[a0:b0 + 1] <= go_hi)))
            tu, q, rs = tu[a0:b0 + 1] - tu[a0], q[a0:b0 + 1], {k: v[a0:b0 + 1] for k, v in rs.items()}
        segs.append(dict(t=tu, q=q, a=rs["a"], flex=rs["flex"], press=rs["press"], label=g.gesture, session=0,
                         block=int(g.block), facing=g.facing, t_session=(g.t_ns - t0_host) * 1e-9,
                         n_raw=int(m.sum()), frac_inside_go=inside))
    return segs, dict(accel_scale=s, fs=fs, noise_gyro=noise_w, noise_finger=noise_f)


if __name__ == "__main__":
    import sys
    d = sys.argv[1] if len(sys.argv) > 1 else "."
    segs, info = load_session(os.path.join(d, "run01_imu.csv"), os.path.join(d, "cues.csv"))
    print(f"{len(segs)} segments, accel scale correction {info['accel_scale']:.3f}, "
          f"raw frames/segment median {int(np.median([s['n_raw'] for s in segs]))}")