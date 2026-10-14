"""Webcam capture with per-frame host timestamps.

    python cam_capture.py --probe                       # which indices work
    python cam_capture.py --cam thinkbook --preview     # index 0
    python cam_capture.py --cam webcam --preview        # index 1
    python cam_capture.py --cam webcam --out run01_webcam --fps 30 --exposure -5

Two cameras at once: run two terminals, one per camera, with different --out. Each camera is
mapped to the device clock separately from its own LED edges, so the processes need no shared clock.

Writes <out>.avi (MJPG) and <out>_t.csv (idx, t_ns). The timestamp is time.perf_counter_ns()
taken immediately after grab() returns, i.e. on the same clock as the logger's rx_ns. It
includes an unknown constant driver/USB latency, which sync_tools.fit_clock absorbs into `a`.
Never trust the AVI's own frame timing or CAP_PROP_POS_MSEC.

Exposure/auto-exposure codes are driver-dependent: check the printed read-back. For LED sync
timing, exposure close to the frame interval is better than a short one (no dead time).
"""
import argparse
import csv
import json
import queue
import threading
import time

import cv2
import numpy as np

# Camera index map (enumeration order of this machine; confirm with --probe then --preview).
CAMERAS = {"thinkbook": 0, "webcam": 1}


def open_cam(index, backend, w, h, fps, exposure):
    api = {"dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF, "any": cv2.CAP_ANY}[backend]
    cap = cv2.VideoCapture(index, api)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    if exposure is not None:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)   # DSHOW: 0.25 = manual; MSMF often 1
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)    # DSHOW: usually log2(seconds), -5 ~ 31 ms
    return cap


def probe(backend, n=4):
    """Print which camera indices open and deliver a frame, to identify them."""
    api = {"dshow": cv2.CAP_DSHOW, "msmf": cv2.CAP_MSMF, "any": cv2.CAP_ANY}[backend]
    for i in range(n):
        cap = cv2.VideoCapture(i, api)
        ok, f = cap.read() if cap.isOpened() else (False, None)
        print(f"index {i}: " + (f"OK {f.shape[1]}x{f.shape[0]}" if ok else "no frame"))
        cap.release()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", choices=list(CAMERAS), default=None,
                    help="named camera (thinkbook=0, webcam=1); overrides --index")
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--probe", action="store_true", help="list working camera indices and exit")
    ap.add_argument("--backend", default="dshow", choices=["dshow", "msmf", "any"])
    ap.add_argument("--w", type=int, default=1280)
    ap.add_argument("--h", type=int, default=720)
    ap.add_argument("--fps", type=float, default=30)
    ap.add_argument("--exposure", type=float, default=None)
    ap.add_argument("--out", default=None, help="default: camera name or cam<index>")
    ap.add_argument("--preview", action="store_true")
    a = ap.parse_args()
    if a.probe:
        probe(a.backend)
        return
    if a.cam:
        a.index = CAMERAS[a.cam]
    a.out = a.out or (a.cam or f"cam{a.index}")

    cap = open_cam(a.index, a.backend, a.w, a.h, a.fps, a.exposure)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera index {a.index} with backend {a.backend}")
    print(f"camera index {a.index} -> output prefix '{a.out}'")
    cfg = dict(index=a.index, cam=a.cam, backend=a.backend, started=time.strftime("%Y-%m-%d %H:%M:%S"),
               requested=dict(w=a.w, h=a.h, fps=a.fps, exposure=a.exposure),
               readback={n: cap.get(getattr(cv2, "CAP_PROP_" + n))
                         for n in ("FRAME_WIDTH", "FRAME_HEIGHT", "FPS", "AUTO_EXPOSURE", "EXPOSURE", "AUTOFOCUS")})
    json.dump(cfg, open(a.out + "_cfg.json", "w"), indent=1)   # a latency calibration only applies to this config
    for name in ("FRAME_WIDTH", "FRAME_HEIGHT", "FPS", "AUTO_EXPOSURE", "EXPOSURE", "AUTOFOCUS"):
        print(f"{name:14s}", cap.get(getattr(cv2, "CAP_PROP_" + name)))

    q, stop, stamps = queue.Queue(), threading.Event(), []

    def writer():
        vw, fh = None, open(a.out + "_t.csv", "w", newline="")
        cw = csv.writer(fh)
        cw.writerow(["idx", "t_ns"])
        while not (stop.is_set() and q.empty()):
            try:
                i, t, f = q.get(timeout=0.1)
            except queue.Empty:
                continue
            if vw is None:
                vw = cv2.VideoWriter(a.out + ".avi", cv2.VideoWriter_fourcc(*"MJPG"),
                                     a.fps, (f.shape[1], f.shape[0]))
            vw.write(f)
            cw.writerow([i, t])
        if vw:
            vw.release()
        fh.close()

    th = threading.Thread(target=writer)
    th.start()
    i = 0
    try:
        while True:
            if not cap.grab():
                continue
            t = time.perf_counter_ns()
            ok, f = cap.retrieve()
            if not ok:
                continue
            q.put((i, t, f))
            stamps.append(t)
            i += 1
            if a.preview and i % 3 == 0:
                cv2.imshow("preview", f)
                if cv2.waitKey(1) == ord("q"):
                    break
    except KeyboardInterrupt:
        pass
    stop.set()
    th.join()
    cap.release()
    dt = np.diff(stamps) * 1e-9
    if dt.size:
        print(f"frames {i}  mean fps {1 / dt.mean():.2f}  dt std {dt.std() * 1e3:.2f} ms  "
              f"max gap {dt.max() * 1e3:.1f} ms  gaps>1.5x {(dt > 1.5 * np.median(dt)).sum()}")


if __name__ == "__main__":
    main()
