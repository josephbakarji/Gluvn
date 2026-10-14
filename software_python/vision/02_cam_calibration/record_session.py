"""One entry point that does the wiring for you: connects the glove, attaches the CSV logger, runs a session.

Run from your project root (the folder that contains `core/`), with the patched core/port_read.py:

    python record_session.py sync    --hand l --out sessions/sync01     # camera-latency calibration (~2 min)
    python record_session.py gesture --hand l --out sessions/g01        # gesture session (~17 min, EKF armed)
    python record_session.py selftest                                   # wiring test with a fake glove, no hardware

Order for `sync`: terminal 1 starts cam_capture.py (--preview); terminal 2 runs this script and waits for Enter.
Outputs go to <out>/: imu.csv (every frame, incl. sync_led, rx_ns), cues.csv, session.json.
The logger is flushed and closed at the end (and on Ctrl-C), so no trailing frames are lost.
"""
import argparse
import csv
import os
import sys
import threading
import time

from gesture_session import run_session
from sync_session import run_sync_session
from sync_tools import FrameLogger


def record(reader, hand, out, kind, args, pause=input, time_scale=1.0, log=print):
    os.makedirs(out, exist_ok=True)
    parser = reader.threads[hand]["parser"]
    if not hasattr(parser, "taps"):
        raise SystemExit("core/port_read.py is not patched: ParseSerial needs `self.taps = []`, the "
                         "`rx_ns`/`sync_led` fields and the tap call (see w2_patches.md, section 2).")
    logger = FrameLogger(os.path.join(out, "imu.csv"))
    parser.taps.append(logger)                      # <- this is the line the docstring in sync_session.py referred to
    send = lambda c: reader.send_command(hand, c)
    try:
        time.sleep(1.0 * time_scale)                # let frames start flowing into the logger
        meta = dict(hand=hand, participant=getattr(args, "participant", None), kind=kind)
        if kind == "sync":
            pause("Start cam_capture.py (with --preview) in another terminal, check the camera sees the glove, "
                  "then press Enter here... ")
            log("starting in 3 s: put the forearm still with the stick's LED facing the camera")
            time.sleep(3.0 * time_scale)
            run_sync_session(send, out, meta=meta, time_scale=time_scale, log=log)
        else:
            pause("Start cam_capture.py in another terminal if you want video, wear the glove, "
                  "stand ready for the first hold, then press Enter here... ")
            run_session(send, out, n_blocks=args.blocks, meta=meta, sync_bursts=args.sync_bursts,
                        time_scale=time_scale, log=log)
    finally:
        time.sleep(0.5 * time_scale)                # last frames in flight
        logger.close()
        log(f"logger closed: {out}/imu.csv")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("kind", choices=["sync", "gesture", "selftest"])
    ap.add_argument("--hand", default="l")
    ap.add_argument("--out", default="sessions/run")
    ap.add_argument("--blocks", type=int, default=15)
    ap.add_argument("--participant", default="P01")
    ap.add_argument("--sync-bursts", default="none", choices=["none", "ends", "all"])
    a = ap.parse_args()
    if a.kind == "selftest":
        return selftest()
    sys.path.insert(0, os.getcwd())
    from core.port_read import Reader                 # your module (needs the w2 patches)
    reader = Reader(sensor_config={a.hand: {"flex": True, "press": True, "imu": True}})
    reader.start_readers()
    try:
        record(reader, a.hand, a.out, a.kind, a)
    finally:
        reader.stop_readers()


# --------------------------------------------------------------------------------------------
# Self-test with a fake glove: emulates the firmware's SYNC_START LED schedule and 1 kHz frames
# --------------------------------------------------------------------------------------------
class _FakeParser:
    def __init__(self):
        self.taps = []


class _FakeReader:
    def __init__(self, scale):
        self.scale = scale
        self.threads = {"l": {"parser": _FakeParser()}}
        self.led = 0
        self.stop = False
        self.sent = []
        self.frames = 0
        self.t = threading.Thread(target=self._emit, daemon=True)
        self.t.start()

    def _emit(self):
        us = 1_000_000
        while not self.stop:
            us += 1000
            self.frames += 1
            f = dict(seq=self.frames % 256, device_us=us, rx_ns=time.perf_counter_ns(), sync_led=self.led,
                     imu=(0, 0, 0, 0, 0, 0), flex=(0,) * 5, press=(0,) * 5, quat=(1, 0, 0, 0))
            for tap in list(self.threads["l"]["parser"].taps):
                tap(f)
            time.sleep(0.001)

    def send_command(self, hand, cmd):
        self.sent.append(cmd)
        if cmd.startswith("SYNC_START"):
            n = int(cmd.split(":")[1])
            threading.Thread(target=self._pulses, args=(n,), daemon=True).start()

    def _pulses(self, n):
        time.sleep(0.2 * self.scale)
        for _ in range(n):
            self.led = 1; time.sleep(0.13 * self.scale)
            self.led = 0; time.sleep(0.45 * self.scale)


def selftest():
    import tempfile
    import types
    scale = 0.1
    out = tempfile.mkdtemp()
    rd = _FakeReader(scale)
    record(rd, "l", out, "sync", types.SimpleNamespace(participant="T"), pause=lambda m: None, time_scale=scale,
           log=lambda *_: None)
    rd.stop = True
    rows = list(csv.DictReader(open(os.path.join(out, "imu.csv"))))
    led = [int(r["sync_led"]) for r in rows]
    rising = sum(1 for a, b in zip(led, led[1:]) if b > a)
    last_dev = int(rows[-1]["device_us"])
    print(f"frames emitted {rd.frames}, rows in imu.csv {len(rows)} (difference = frames in flight at close)")
    print(f"LED rising edges seen: {rising} (expected {4 + 4 * 8}: one 4-pulse check + 4 bursts of 8)")
    print(f"commands sent: {rd.sent}")
    print(f"files in out dir: {sorted(os.listdir(out))}")
    ok = rising == 36 and len(rows) >= rd.frames - 5
    print("SELFTEST", "PASSED" if ok else "FAILED")


if __name__ == "__main__":
    main()
