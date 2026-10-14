"""Dedicated camera-latency calibration session (about 3 min). No gestures, no EKF needed.

Goal: measure the camera stamp -> device clock offset from LED bursts with a clean LED view.
The forearm stays completely still with the stick's LED facing the camera(s); bursts bracket the recording.

Start FIRST (separate terminals/threads), in this order:
    1. FrameLogger on the parser tap (python: parser.taps.append(FrameLogger('sync01_imu.csv')))
    2. python cam_capture.py --cam thinkbook --out sync01_tb --fps 30 --exposure -5 --preview
    3. run_sync_session(lambda c: reader.send_command('l', c), 'sessions/sync01')
Afterwards: logger.close(), stop cam_capture (Ctrl-C), then
    python run_sync_check.py --imu sync01_imu.csv --video sync01_tb.avi --ts sync01_tb_t.csv --plot sync01.png --debug dbg --json sync01.json
Repeat 3x (re-connect the glove between runs; once with another exposure) and run sync_stability.py on the
JSON files: only if the lag is stable may later gesture sessions reuse it.
Dry run: python sync_session.py
"""
import csv
import json
import os
import time


def run_sync_session(send_cmd, out_dir, n_bursts=4, gap_s=25.0, burst_pulses=8, check_pulses=4, pre_s=5.0,
                     post_s=5.0, seed=500, meta=None, time_scale=1.0, log=print):
    os.makedirs(out_dir, exist_ok=True)
    wait = lambda s: time.sleep(s * time_scale)
    burst_len = burst_pulses * 0.62 + 0.4            # ~ mean pulse period 0.62 s (ON 130 ms + OFF 450 ms) + start delay
    with open(os.path.join(out_dir, "session.json"), "w") as f:
        json.dump(dict(kind="sync_calibration", meta=meta or {}, n_bursts=n_bursts, gap_s=gap_s,
                       burst_pulses=burst_pulses, protocol_version="1.0"), f, indent=1)
    fh = open(os.path.join(out_dir, "cues.csv"), "w", newline="")
    cw = csv.writer(fh)
    cw.writerow(["event", "block", "trial", "gesture", "facing", "t_ns", "wall"])
    sd = [seed]

    def ev(kind, i=-1):
        cw.writerow([kind, i, -1, "", "", time.perf_counter_ns(), time.strftime("%H:%M:%S")]); fh.flush()

    def burst(kind, pulses, i):
        sd[0] += 1
        ev(kind, i)
        send_cmd(f"SYNC_START:{pulses}:{sd[0]}")

    log("=== VISIBILITY CHECK: look at the camera preview. Is the red LED visible while it blinks? ===")
    log("    If not: rotate the forearm / stick toward the camera, lower the exposure, or add an external LED.")
    burst("visibility_check", check_pulses, -1)
    wait(check_pulses * 0.62 + 2.5)
    log("=== RECORDING: keep the forearm COMPLETELY STILL, LED facing the camera(s). Do not touch the glove. ===")
    ev("rec_start")
    wait(pre_s)
    for i in range(n_bursts):
        log(f"--- burst {i + 1}/{n_bursts} (LED blinks about {burst_len:.0f} s) ---")
        burst("burst", burst_pulses, i)
        wait(burst_len + (gap_s if i < n_bursts - 1 else post_s))
    ev("rec_end")
    fh.close()
    log("=== DONE: stop the logger (logger.close()) and cam_capture, then run run_sync_check.py ===")
    return n_bursts


def _dry_run():
    sent, logs = [], []
    run_sync_session(sent.append, "/tmp/ss_dry", time_scale=0.0, log=logs.append)
    rows = list(csv.DictReader(open("/tmp/ss_dry/cues.csv")))
    t = [int(r["t_ns"]) for r in rows]
    ev = [r["event"] for r in rows]
    print(f"commands: {sent}")
    print(f"events: {ev}")
    print(f"monotonic: {t == sorted(t)} | bursts after rec_start: {ev.count('burst')} (first and last bracket the recording)")
    n = 4
    print(f"duration: {(5 + n * (8 * 0.62 + 0.4) + (n - 1) * 25 + 5 + 4 * 0.62 + 2.5) / 60:.1f} min")


if __name__ == "__main__":
    _dry_run()
