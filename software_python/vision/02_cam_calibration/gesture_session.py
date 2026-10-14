"""Cue-driven gesture recording.

Design choices (each addresses a specific confound):
  * Randomised blocks: every block contains every gesture once, in random order. If gestures were
    recorded class-by-class, class would be confounded with elapsed time, and therefore with
    heading drift, and a clusterer could "discover" the drift instead of the gesture.
  * Facing nuisance: the body orientation changes between blocks (front / left45 / right45) while
    the pointing targets stay fixed in the room. Heading-free features must not care; absolute-
    heading features must.
  * Camera-latency calibration is a SEPARATE short test (sync_session.py); the gesture session needs
    the EKF armed and a free performer, not LED visibility. Optional bursts (sync_bursts='ends') give a
    per-session check. The static holds stay: they give the frozen heading alignment window, the
    accelerometer scale and the activity noise floor.
  * Three distinct sounds (see BEEP_LEGEND): the previous version only beeped at GO and STOP, so the
    start of each trial was ambiguous. Start moving at the LONG HIGH beep, not before.
  * Cue times are logged on the host monotonic clock (perf_counter_ns), the same clock as the
    logger's rx_ns, so segments can be cut by rx_ns and timed by device_us.
  * Protocol 1.1: every cue is logged immediately BEFORE its beep call. winsound.Beep blocks for the beep duration, and
    protocol 1.0 logged GO and STOP after the call returned, i.e. 0.35 s late (use fix_cues.py on 1.0 sessions). The
    remaining offset between the call and the audible onset is the audio output latency: not measured.
  * Optional READY->GO jitter (ready_jitter=(lo, hi) s, uniform, own RNG so the schedule is unchanged): a fixed interval
    lets the performer anticipate the GO beep.

Usage (real): run it alongside FrameLogger and cam_capture.py
    from gesture_session import run_session
    run_session(lambda c: reader.send_command('r', c), 'data/sessions/s01', meta=dict(participant='P01'))
Dry run: python gesture_session.py
"""
import argparse
import csv
import json
import os
import random
import time

# (name, group, instruction). group: finger | dynamics | orientation | spatial (needs absolute heading)
GESTURES = [
    ("fist_release", "finger", "Close the fist slowly, then release"),
    ("pinch_tap", "finger", "Pinch thumb and index twice"),
    ("circle_cw", "dynamics", "Circle in front of you, clockwise seen from behind, 2 loops"),
    ("circle_ccw", "dynamics", "Circle in front of you, counter-clockwise seen from behind, 2 loops"),
    ("swipe_lr", "dynamics", "Horizontal swipe, left to right"),
    ("swipe_ud", "dynamics", "Vertical swipe, top to bottom"),
    ("shake", "dynamics", "Rapid wrist shake, about 4 Hz, 1.5 s"),
    ("twist", "orientation", "Pronate then supinate the forearm"),
    ("point_left", "spatial", "Rotate the FOREARM to point at the LEFT wall marker, hold 1 s, return"),
    ("point_right", "spatial", "Rotate the FOREARM to point at the RIGHT wall marker, hold 1 s, return"),
]
FACINGS = ["front", "left45", "right45"]


def build_schedule(n_blocks=15, seed=0):
    rng = random.Random(seed)
    facing_pool = [FACINGS[i % 3] for i in range(n_blocks)]
    rng.shuffle(facing_pool)
    blocks, prev_last = [], None
    for b in range(n_blocks):
        order = list(range(len(GESTURES)))
        while True:
            rng.shuffle(order)
            if order[0] != prev_last:            # no immediate repeat across the block boundary
                break
        prev_last = order[-1]
        blocks.append(dict(block=b, facing=facing_pool[b], gestures=order))
    return blocks


BEEP_LEGEND = ("SOUNDS:  two SHORT beeps = READY (neutral pose, arm relaxed, stay still)\n"
               "         one LONG HIGH beep = GO (start the gesture NOW)\n"
               "         one LONG LOW beep  = STOP (finish, return to neutral and rest)")


def _beep(kind):
    try:
        import winsound
        if kind == "ready":
            winsound.Beep(600, 70); time.sleep(0.07); winsound.Beep(600, 70)
        elif kind == "go":
            winsound.Beep(1000, 350)
        else:
            winsound.Beep(400, 350)
    except Exception:
        print("\a", end="", flush=True)


def run_session(send_cmd, out_dir, n_blocks=15, seed=0, meta=None, ready_s=1.5, window_s=3.0,
                rest_s=1.5, hold_s=6.0, turn_s=3.0, burst_pulses=8, sync_bursts="none", time_scale=1.0, beep=_beep, log=print,
                ready_jitter=None,
                setup_cmds=("QUAT_STREAM_ON", "NAV_QUAT_STREAM_ON", "MOTION_STREAM_ON"), arm_wait_s=3.0):
    """Run the protocol. send_cmd(str) transmits a firmware command (e.g. SYNC_START:8:123).
    sync_bursts: 'none' (default; camera latency is calibrated separately with sync_session.py),
    'ends' (first and last hold, a per-session check) or 'all' (every hold).
    ready_jitter: None (fixed ready_s) or (lo, hi) seconds, uniform per trial, applied between the READY beeps and GO.
    time_scale < 1 compresses all waits (dry runs). Returns the schedule."""
    os.makedirs(out_dir, exist_ok=True)
    sched = build_schedule(n_blocks, seed)
    wait = lambda s: time.sleep(s * time_scale)
    with open(os.path.join(out_dir, "session.json"), "w") as f:
        json.dump(dict(meta=meta or {}, seed=seed, n_blocks=n_blocks, gestures=GESTURES,
                       timing=dict(ready_s=ready_s, window_s=window_s, rest_s=rest_s, hold_s=hold_s, turn_s=turn_s,
                                   ready_jitter=list(ready_jitter) if ready_jitter else None),
                       sync_bursts=sync_bursts,
                       schedule=sched, protocol_version="1.1"), f, indent=1)
    fh = open(os.path.join(out_dir, "cues.csv"), "w", newline="")
    cw = csv.writer(fh)
    cw.writerow(["event", "block", "trial", "gesture", "facing", "t_ns", "wall"])
    sync_seed = [1000 + seed]
    jit = random.Random(seed + 7919)          # separate RNG: the gesture schedule does not depend on the jitter
    log(BEEP_LEGEND)

    def ev(kind, block=-1, trial=-1, g="", facing=""):
        cw.writerow([kind, block, trial, g, facing, time.perf_counter_ns(), time.strftime("%H:%M:%S")])
        fh.flush()

    def hold(label, block, facing, burst=False, turn=True):
        if turn:   # turning the body moves the forearm: do it BEFORE the still hold
            extra = " and rotate the forearm so the STICK'S LED FACES THE CAMERA(S)" if burst else ""
            log(f"--- TURN ({label}): face '{facing}' now{extra}; then keep the forearm completely still ---")
            ev("turn", block, -1, "", facing)
            wait(turn_s)
        log(f"--- HOLD ({label}): forearm completely still" + (" (LED blinks for about 6 s)" if burst else "") + " ---")
        ev("hold_start", block, -1, "", facing)
        if burst:
            sync_seed[0] += 1
            send_cmd(f"SYNC_START:{burst_pulses}:{sync_seed[0]}")
        wait(hold_s)
        ev("hold_end", block, -1, "", facing)

    # NavEKF only runs (and nav_armed is only meaningful) when these streams are on; the real test
    # session had none of them, so nav_quat/pos/vel/lin_accel were all zeros. Keep the arm still.
    for c in setup_cmds:
        send_cmd(c)
    log(f"--- stay still for {arm_wait_s:.0f} s while NavEKF arms; then check nav_armed in the CSV ---")
    wait(arm_wait_s)
    hold("start", 0, sched[0]["facing"], burst=sync_bursts in ("ends", "all"))   # first hold defines the frozen heading alignment window
    for blk in sched:
        b, facing = blk["block"], blk["facing"]
        log(f"=== block {b + 1}/{n_blocks}  facing: {facing} ===")
        for trial, gi in enumerate(blk["gestures"]):
            name, _, text = GESTURES[gi]
            log(f"  [{trial + 1}/{len(GESTURES)}] READY: {name} - {text}   (arm relaxed and still until the GO beep)")
            ev("ready", b, trial, name, facing)
            beep("ready")
            wait(jit.uniform(*ready_jitter) if ready_jitter else ready_s)
            log("      >>> GO <<<")
            ev("go", b, trial, name, facing); beep("go")
            wait(window_s)
            log("      --- STOP: return to neutral ---")
            ev("stop", b, trial, name, facing); beep("stop")
            wait(rest_s)
        nxt = sched[b + 1]["facing"] if b + 1 < n_blocks else facing
        last = b + 1 >= n_blocks
        hold("end" if last else "between blocks", b + 1, nxt,
             burst=sync_bursts == "all" or (last and sync_bursts == "ends"))
    fh.close()
    return sched


def _dry_run():
    sent, logs = [], []
    sched = run_session(sent.append, "/tmp/gs_dry", n_blocks=6, time_scale=0.0, beep=lambda k: None,
                        log=logs.append, meta=dict(participant="DRY"))
    rows = list(csv.DictReader(open("/tmp/gs_dry/cues.csv")))
    gos = [r for r in rows if r["event"] == "go"]
    ok_each = all(sorted(b["gestures"]) == list(range(len(GESTURES))) for b in sched)
    no_rep = all(sched[i]["gestures"][-1] != sched[i + 1]["gestures"][0] for i in range(len(sched) - 1))
    t = [int(r["t_ns"]) for r in rows]
    n_sync = lambda cmds: sum(c.startswith("SYNC_START") for c in cmds)
    print(f"blocks={len(sched)}  go-cues={len(gos)}  default sync bursts={n_sync(sent)} (expected 0)  "
          f"turn cues={sum(r['event'] == 'turn' for r in rows)}  hold_start cues={sum(r['event'] == 'hold_start' for r in rows)}")
    for mode, exp in (("ends", 2), ("all", 7)):
        snt = []
        run_session(snt.append, "/tmp/gs_dry2", n_blocks=6, time_scale=0.0, beep=lambda k: None,
                    log=lambda *_: None, sync_bursts=mode)
        print(f"sync_bursts='{mode}': {n_sync(snt)} bursts (expected {exp})")
    print(f"each gesture once per block: {ok_each}   no repeat across boundary: {no_rep}   "
          f"cue times monotonic: {t == sorted(t)}")
    print("facings per block:", [b['facing'] for b in sched])
    print("setup commands:", sent[:3])
    full = (len(GESTURES) * (1.5 + 3.0 + 1.5)) * 15 + 16 * (6.0 + 3.0)
    print(f"full protocol (15 blocks): {full / 60:.1f} min per session")


if __name__ == "__main__":
    argparse.ArgumentParser(description="Prints a dry-run of the protocol; use run_session() for real.").parse_args()
    _dry_run()