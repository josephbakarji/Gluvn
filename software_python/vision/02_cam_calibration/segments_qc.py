"""QC for gesture sessions: per trial (READY..next READY) angular speed, finger activity, forearm-axis deviation,
GO..STOP shaded, detected burst outlined; plus a per-gesture / per-block compliance table.

    python segments_qc.py --dir sessions/g01
    python segments_qc.py --dir sessions/g01 --out sessions/g01/qc.png     # also writes qc_compliance.csv

Inputs: imu.csv (or run01_imu.csv) and cues.csv from record_session.py. Not the camera files.

Compliance is judged per gesture CLASS with the channel that describes it (provisional criteria, not validated):
  dynamic (shake, twist, circle_*): forearm |w| peak inside GO and >= 50 % of floored |w| inside GO
  stroke  (swipe_*)               : forearm |w| peak inside GO
  finger  (pinch_tap, fist_release): finger-activity peak inside GO and >= 50 % of floored activity inside GO
  pose    (point_*)               : finger-activity peak inside GO and >= 50 % of floored activity inside GO (reference-free;
                                    pointing involves index extension). `pose_ok` is the orientation check: the forearm axis
                                    deviates > POSE_THR deg from the TRIAL'S OWN pre-GO axis (median over [GO-0.4, GO)) for
                                    >= 50 % of GO.
Forearm axis d_W = R_WB [0,1,0]^T (nominal sensor +Y; heading_ref calibration not applied); blind to pronation about it.
Hold-referenced columns (p_pre/p_go/p_post) are DIAGNOSTIC ONLY: on real data they were ~25 deg before GO for every class, i.e.
the block's static-hold pose is not the performer's trial-start pose. hold_off_el / hold_off_az split that offset into
elevation (gravity-observable) and azimuth (heading: drifts, and the body may turn); compare them across trial index.
"""
import argparse
import os

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as Rot

import load_session as ls

D_B = (0.0, 1.0, 0.0)
POSE_THR = 15.0
CLASS = {"shake": "dynamic", "twist": "dynamic", "circle_cw": "dynamic", "circle_ccw": "dynamic",
         "swipe_ud": "stroke", "swipe_lr": "stroke", "point_left": "pose", "point_right": "pose",
         "pinch_tap": "finger", "fist_release": "finger"}


def motion_split(tu, w, go_lo, go_hi, floor=0.0):
    """pre/in/post shares of floored activity, peak-in-GO flag, peak time relative to GO onset (s)."""
    wc = np.maximum(np.asarray(w, float) - floor, 0.0)
    tot = max(wc.sum(), 1e-9)
    pre = wc[tu < go_lo].sum() / tot
    inn = wc[(tu >= go_lo) & (tu <= go_hi)].sum() / tot
    post = wc[tu > go_hi].sum() / tot
    pk = tu[int(np.argmax(wc))]
    return pre, inn, post, bool(go_lo <= pk <= go_hi), float(pk - go_lo)


def rest_axis(S, cues, blk, before_ns, d_b=D_B):
    """Unit forearm axis in the world frame averaged over the block's most recent static hold before `before_ns`."""
    hs = cues[(cues.event == "hold_start") & (cues.block == blk)].sort_values("t_ns")
    if hs.empty:
        return None
    prior = hs[hs.t_ns < before_ns]
    h = prior.iloc[-1] if len(prior) else hs.iloc[0]
    he = cues[(cues.event == "hold_end") & (cues.block == blk) & (cues.t_ns > h.t_ns)].t_ns.min()
    m = (S["rx"] >= h.t_ns + 1e9) & (S["rx"] <= he)
    if m.sum() < 20:
        return None
    d = np.median(S["R"][m].apply(d_b), axis=0)
    return d / np.linalg.norm(d)


def pose_deviation(q, d_rest, d_b=D_B):
    """Angle (deg) between the forearm axis and the rest axis. q: (n,4) scalar-first, world <- sensor."""
    d = Rot.from_quat(q[:, [1, 2, 3, 0]]).apply(d_b)
    return np.degrees(np.arccos(np.clip(d @ d_rest, -1.0, 1.0)))


def pose_split(tu, th, go_lo, go_hi, thr=POSE_THR):
    med = lambda m: float(np.median(th[m])) if m.any() else np.nan
    g = (tu >= go_lo) & (tu <= go_hi)
    return med(tu < go_lo), med(g), med(tu > go_hi), (float(np.mean(th[g] > thr)) if g.any() else np.nan)


wrap = lambda x: (x + 180.0) % 360.0 - 180.0


def axis_series(q, d_b=D_B):
    return Rot.from_quat(q[:, [1, 2, 3, 0]]).apply(d_b)


def el_az(d):
    """Elevation (deg above horizontal, gravity-observable) and azimuth (deg, heading-dependent) of unit vectors."""
    return (np.degrees(np.arcsin(np.clip(d[..., 2], -1, 1))), np.degrees(np.arctan2(d[..., 1], d[..., 0])))


def local_pose(tu, d, go_lo, go_hi, pre_win=0.4, late_from=1.0, thr=POSE_THR, el_max=70.0):
    """Pose relative to the trial's own pre-GO axis. Returns dict(l_hold, d_az, d_el, ref). Azimuth is ill-conditioned near
    vertical, so d_az uses samples with |elevation| < el_max only."""
    nan = dict(l_hold=np.nan, d_az=np.nan, d_el=np.nan, pre_move=np.nan, ref=None)
    m = (tu >= go_lo - pre_win) & (tu < go_lo)
    if m.sum() < 5:
        return nan
    ref = np.median(d[m], axis=0)
    ref = ref / np.linalg.norm(ref)
    th = np.degrees(np.arccos(np.clip(d @ ref, -1.0, 1.0)))
    el, az = el_az(d)
    el0, az0 = el_az(ref)
    g = (tu >= go_lo) & (tu <= go_hi)
    late = (tu >= go_lo + late_from) & (tu <= go_hi)
    ok_az = late & (np.abs(el) < el_max) if abs(el0) < el_max else np.zeros_like(late)
    s0 = tu < 0.3
    pre_move = float(np.degrees(np.arccos(np.clip(np.median(d[s0], axis=0) @ ref / np.linalg.norm(np.median(d[s0], axis=0)), -1, 1)))) if s0.any() else np.nan
    return dict(l_hold=float(np.mean(th[g] > thr)) if g.any() else np.nan, pre_move=pre_move,
                d_az=float(np.median(wrap(az - az0)[ok_az])) if ok_az.any() else np.nan,
                d_el=float(np.median((el - el0)[late])) if late.any() else np.nan, ref=ref, th=th)


def padded_burst(tu, w, f, noise_w, noise_f, t0, t1, pre, post, fs):
    """Label-agnostic segmentation check: main burst searched only inside [GO-pre, STOP+post].
    b_in_go / b_post: share of the burst's samples inside GO..STOP / after STOP. clip_pre / clip_post: activity is already above
    threshold at the search-window edge, i.e. motion was cut off there (pad too small)."""
    mk = (tu >= t0 - pre) & (tu <= t1 + post)
    act = np.maximum(w / max(3 * noise_w, 0.15), f / max(3 * noise_f, 5.0))
    first = int(np.argmax(mk))
    last = len(mk) - 1 - int(np.argmax(mk[::-1]))
    out = dict(found=0.0, b_in_go=np.nan, b_post=np.nan, b_dur=np.nan, clip_pre=float(act[first] > 1.0), clip_post=float(act[last] > 1.0), b=None)
    b = ls.find_burst(w * mk, f * mk, noise_w, noise_f, fs)
    if b is None:
        return out
    seg = tu[b[0]:b[1] + 1]
    out.update(found=1.0, b_in_go=float(np.mean((seg >= t0) & (seg <= t1))), b_post=float(np.mean(seg > t1)),
               b_dur=(b[1] - b[0]) / fs, b=b)
    return out


def compliant(cls, r):
    if cls in ("dynamic",):
        return bool(r["w_pk_in"] and r["w_in"] >= 0.5)
    if cls == "stroke":
        return bool(r["w_pk_in"])
    if cls == "finger":
        return bool(r["f_pk_in"] and r["f_in"] >= 0.5)
    if cls == "pose":
        return bool(r["f_pk_in"] and r["f_in"] >= 0.5)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", default=".")
    ap.add_argument("--imu", default=None, help="IMU csv (default: imu.csv, else run01_imu.csv)")
    ap.add_argument("--cues", default="cues.csv")
    ap.add_argument("--out", default="segments_qc.png")
    ap.add_argument("--fs", type=float, default=100.0)
    ap.add_argument("--pre-pad", type=float, default=0.8, help="burst search starts this many s before GO")
    ap.add_argument("--post-pad", type=float, default=0.2, help="burst search ends this many s after STOP")
    a = ap.parse_args()
    imu = a.imu or next((n for n in ("imu.csv", "run01_imu.csv") if os.path.exists(os.path.join(a.dir, n))), "imu.csv")
    imu_path, cues_path = os.path.join(a.dir, imu), os.path.join(a.dir, a.cues)
    df = pd.read_csv(imu_path)
    need = {"device_us", "rx_ns", "imu_0", "flex_0"}
    if not need <= set(df.columns):
        raise SystemExit(f"{imu_path} is not an IMU log (missing columns {sorted(need - set(df.columns))}). Expected the "
                         f"FrameLogger file imu.csv from record_session.py, not a camera file (*_t.csv / *_cfg.json).")
    cues = pd.read_csv(cues_path)
    if "event" not in cues.columns:
        raise SystemExit(f"{cues_path} is not a cues file (no 'event' column). Expected cues.csv from the session folder.")
    _, info = ls.load_session(imu_path, cues_path, fs=a.fs)
    S = ls.decode(df)
    FL = S["df"][[f"flex_{i}" for i in range(5)]].values.astype(float)
    floor_w = max(3 * info["noise_gyro"], 0.15)       # same floors as find_burst
    floor_f = max(3 * info["noise_finger"], 5.0)
    gos = cues[cues.event == "go"].reset_index(drop=True)
    n = len(gos)
    cols = 6
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(int(np.ceil(n / cols)), cols, figsize=(3.4 * cols, 2.3 * np.ceil(n / cols)), squeeze=False)
    rows, store = [], []
    for ax, (_, g) in zip(axs.ravel(), gos.iterrows()):
        key = (cues.block == g.block) & (cues.trial == g.trial)
        rd = cues[key & (cues.event == "ready")].t_ns.values[0]
        st = cues[key & (cues.event == "stop")].t_ns.values[0]
        nxt = cues[(cues.t_ns > st) & cues.event.isin(["ready", "hold_start"])].t_ns.min()
        m = (S["rx"] >= rd) & (S["rx"] <= nxt)
        tu, q, rs = ls.resample(S["t"][m], S["R"][m], dict(flex=FL[m]), a.fs)
        w, f = ls._activity(q, rs["flex"], a.fs)
        t0, t1 = (g.t_ns - rd) * 1e-9, (st - rd) * 1e-9
        pb = padded_burst(tu, w, f, info["noise_gyro"], info["noise_finger"], t0, t1, a.pre_pad, a.post_pad, a.fs)
        b = pb["b"]
        store.append((CLASS.get(g.gesture, "unknown"), tu, w, f, t0, t1))
        pw, iw, qw, kw, tw = motion_split(tu, w, t0, t1, floor_w)
        pf, i_f, qf, kf, tf = motion_split(tu, f, t0, t1, floor_f)
        d0 = rest_axis(S, cues, g.block, rd)
        dser = axis_series(q)
        th = pose_deviation(q, d0) if d0 is not None else None
        pp, pg, pq, ph = pose_split(tu, th, t0, t1) if th is not None else (np.nan,) * 4
        lp = local_pose(tu, dser, t0, t1)
        ho_el = ho_az = np.nan
        if d0 is not None and lp["ref"] is not None:
            e0, a0_ = el_az(d0)
            e1, a1_ = el_az(lp["ref"])
            ho_el, ho_az = float(e1 - e0), float(wrap(a1_ - a0_))
        cls = CLASS.get(g.gesture, "unknown")
        r = dict(block=int(g.block), trial=int(g.trial), gesture=g.gesture, cls=cls, facing=g.facing,
                 w_in=iw, w_pre=pw, w_post=qw, w_pk_in=float(kw), w_pk_rel=tw,
                 f_in=i_f, f_pre=pf, f_post=qf, f_pk_in=float(kf), f_pk_rel=tf,
                 p_pre=pp, p_go=pg, p_post=pq, p_hold=ph,
                 l_hold=lp["l_hold"], pre_move=lp["pre_move"], d_az=lp["d_az"], d_el=lp["d_el"], hold_off_el=ho_el, hold_off_az=ho_az,
                 b_found=pb["found"], b_in_go=pb["b_in_go"], b_post=pb["b_post"], b_dur=pb["b_dur"], b_cap=float(pb["b_dur"] >= 3.99), clip_pre=pb["clip_pre"], clip_post=pb["clip_post"])
        r["ok"] = float(compliant(cls, r))
        r["pose_ok"] = float(r["l_hold"] >= 0.5) if np.isfinite(r["l_hold"]) else np.nan
        rows.append(r)
        ax.plot(tu, w, lw=.8, label="|w| rad/s")
        ax.plot(tu, f / max(f.max(), 1e-9) * max(w.max(), 0.3), lw=.8, color="C1", alpha=.7, label="fingers (scaled)")
        if cls == "pose" and lp.get("th") is not None:
            ax.plot(tu, lp["th"] / max(lp["th"].max(), 1.0) * max(w.max(), 0.3), lw=.9, color="k", ls="--", alpha=.7,
                    label="axis dev from pre-GO (scaled)")
        ax.axvspan(t0, t1, color="g", alpha=.15)
        if b:
            ax.axvspan(tu[b[0]], tu[b[1]], facecolor="none", edgecolor="r", lw=1.5)
        ax.set_title(f"b{g.block} {g.gesture} [{cls}] {'ok' if r['ok'] else 'NOT ok'}", fontsize=7,
                     color="k" if r["ok"] else "r")
        ax.tick_params(labelsize=6)
    for ax in axs.ravel()[n:]:
        ax.axis("off")
    axs[0, 0].legend(fontsize=6)
    fig.suptitle(f"green = GO..STOP, red box = burst found inside [GO-{a.pre_pad}, STOP+{a.post_pad}] s, dashed = forearm-axis deviation from pre-GO axis (pose class)")
    fig.tight_layout()
    fig.savefig(a.out, dpi=110)
    C = pd.DataFrame(rows)
    csv = a.out.replace(".png", "_compliance.csv")
    C.to_csv(csv, index=False)
    pd.set_option("display.width", 200)
    gk = ["cls", "gesture"]
    print(C.groupby(gk)[["ok", "pose_ok", "w_in", "w_pre", "w_post", "w_pk_in", "f_in", "f_pk_in", "l_hold"]].mean().round(2).to_string())
    print(C.groupby(gk)[["w_pk_rel", "f_pk_rel", "d_az", "d_el", "pre_move"]].median().round(2).to_string())
    print("share of trials with d_az > 0 (point_left vs point_right should differ if the gesture is performed):")
    print(C.assign(pos=(C.d_az > 0).astype(float)).groupby(gk)["pos"].mean().round(2).to_string())
    print("hold-pose vs trial-start-pose offset, median |deg| by trial index within block (growth with index => heading drift / body turning):")
    print(C.groupby("trial")[["hold_off_el", "hold_off_az"]].agg(lambda x: x.abs().median()).round(1).T.to_string())
    print("b_cap = share of trials whose burst hit find_burst max_len (4.0 s): if high, the burst is a fixed 4 s window around the peak, not an adaptive segment.")
    print(f"label-agnostic padded-window burst (pre {a.pre_pad}, post {a.post_pad}; same rule for every gesture), mean per gesture:")
    print(C.groupby(gk)[["b_found", "b_in_go", "b_post", "b_dur", "b_cap", "clip_pre", "clip_post"]].mean().round(2).to_string())
    print("pad sensitivity, mean per class (choose pads with low clip_pre without letting the post-STOP return in: b_post):")
    sens = []
    for pre in (0.3, 0.8, 1.2):
        for post in (0.0, 0.2, 0.5):
            for cls_, tu_, w_, f_, t0_, t1_ in store:
                pbx = padded_burst(tu_, w_, f_, info["noise_gyro"], info["noise_finger"], t0_, t1_, pre, post, a.fs)
                sens.append(dict(pre=pre, post=post, cls=cls_, **{k: v for k, v in pbx.items() if k != "b"}))
    SN = pd.DataFrame(sens)
    print(SN.groupby(["pre", "post", "cls"])[["found", "b_in_go", "b_post", "clip_pre", "clip_post"]].mean().round(2).to_string())
    print("w_pk_rel quantiles (s from GO; GO..STOP spans 0..%.2f):" % np.median([t1_ - t0_ for _, _, _, _, t0_, t1_ in store]))
    print(C.groupby("gesture")["w_pk_rel"].quantile([.1, .25, .5, .75, .9]).unstack().round(2).to_string())
    P = C[C.cls == "pose"]
    print("pointing, d_az by facing (targets are fixed in the room, so the required forearm rotation depends on the body facing):")
    print(P.groupby(["gesture", "facing"]).agg(n=("d_az", "size"), d_az_med=("d_az", "median"), d_az_pos=("d_az", lambda x: float((x > 0).mean()))).round(2).to_string())
    taus = np.round(np.arange(-0.4, 1.01, 0.1), 2)
    L = []
    for cls_, tu_, w_, f_, t0_, t1_ in store:
        for tau in taus:
            L.append(dict(tau=tau, cls=cls_, w=motion_split(tu_, w_, t0_ + tau, t1_ + tau, floor_w)[1],
                          f=motion_split(tu_, f_, t0_ + tau, t1_ + tau, floor_f)[1]))
    LD = pd.DataFrame(L)
    tab = LD.groupby(["tau", "cls"])[["w", "f"]].mean().unstack("cls")
    tab.columns = [f"{c[0]}_{c[1]}" for c in tab.columns]
    tab["w_all"] = LD.groupby("tau")["w"].mean()
    tab["f_all"] = LD.groupby("tau")["f"].mean()
    print("response-lag scan: mean share of floored activity inside the cue window SHIFTED by tau (same window length). Peak = response latency:")
    print(tab.round(2).to_string())
    pl = {c: (float(tab.index[tab[c] >= tab[c].max() - 0.02].min()), float(tab.index[tab[c] >= tab[c].max() - 0.02].max()), round(float(tab[c].max()), 2)) for c in tab.columns}
    print("tau plateau (within 0.02 of the max) as (from, to, max coverage) per column:")
    print(pd.DataFrame(pl, index=["tau_from", "tau_to", "max_cov"]).T.to_string())
    print("compliance rate per block (all classes / dynamic+finger only):")
    print(pd.DataFrame({"all": C.groupby("block")["ok"].mean(),
                        "dyn+fing": C[C.cls.isin(["dynamic", "finger"])].groupby("block")["ok"].mean()}).round(2).T.to_string())
    print(f"overall compliant (class-specific criteria): {C.ok.mean():.0%}. Criteria are provisional.")
    print("saved", a.out, csv)


if __name__ == "__main__":
    main()