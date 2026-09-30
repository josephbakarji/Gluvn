#!/usr/bin/env python3
"""
BLE vs USB bandwidth benchmark for Gluvn-M5.

Captures sensor frames from each transport independently, measures effective
sampling rate, packet loss (via uint8 sequence number), and inter-packet jitter.

"""

import argparse
import csv
import os
import time
import queue
import numpy as np
import matplotlib.pyplot as plt
from core.port_read import ReadSerial, ReadBLE, ParseSerial

# ── Imports from project ────────────────────────────────────────────────────
from core.__init__ import portR, portL, baud, BLE_NAME_R, BLE_NAME_L, figDir

# Reuse ParseSerial's protocol constants rather than redefining them here —
# keeps this benchmark in lockstep with the actual receiver if the framing
# ever changes again.
FRAME_SYNC = ParseSerial.FRAME_SYNC
FRAME_LENGTHS = (ParseSerial.SAMPLE_UNIT_LEN, ParseSerial.CAL_UNIT_LEN)
_crc16_ccitt  = ParseSerial._crc16_ccitt


class TimestampedReadBLE(ReadBLE):
    """
    Capture the arrival timestamp inside the GATT notification callback,
    before the data reaches the queue, parser, or CRC validation. Only the
    callback is specialized; connection and transport behavior stay inherited
    from ReadBLE.
    """
    def _notification_handler(self, sender, data: bytearray):
        self.sensq.put((time.perf_counter(), bytes(data)))


def extract_frames(buf: bytearray):
    """
    Pop complete, CRC-valid [sync][len][payload][crc16] frames from the front
    of `buf` (mutates it in place). Returns (frames, crc_errors), where each
    frame is (payload: bytes, seq: int, device_us: int), in arrival order.

    A physical frame may bundle more than one logical sample (firmware
    BUNDLE_FRAMES > 1): `length` is then a whole multiple of one sample's
    unit size, and this unpacks that many (payload, seq, device_us) tuples
    from it — one physical frame in, 1..BUNDLE_FRAMES entries out. With the
    current firmware default (BUNDLE_FRAMES=1) this is always exactly one.
    """
    frames, errors = [], 0
    while True:
        idx = buf.find(FRAME_SYNC)

        if idx < 0:
            if len(buf) > 1:
                del buf[:-1]   # keep last byte in case it's a split sync word
            return frames, errors

        if idx > 0:
            del buf[:idx]      # drop leading noise (ASCII acks, boot banner)

        if len(buf) < 3:
            return frames, errors   # need the length byte yet

        length = buf[2]
        total = 3 + length + 2      # header + payload + crc16

        if len(buf) < total:
            return frames, errors   # wait for the rest of the frame

        payload = bytes(buf[3:3 + length])
        crc_rx = (buf[3 + length] << 8) | buf[3 + length + 1]

        # Bundled frames contain whole multiples of one sample unit (29 or
        # 39 bytes), so length need not equal one unit. Their LCM is 1131,
        # larger than the wire's one-byte length field.
        unit_len = next((unit for unit in FRAME_LENGTHS if length % unit == 0), None)

        if unit_len is None or _crc16_ccitt(payload) != crc_rx:
            errors += 1
            del buf[0]         # false sync match or corruption — advance & rescan
            continue

        n_samples = length // unit_len
        for i in range(n_samples):
            sub = payload[i * unit_len:(i + 1) * unit_len]
            frames.append((sub, sub[1], int.from_bytes(sub[2:6], 'big')))  # seq, device_us
        del buf[:total]


# ── Capture ─────────────────────────────────────────────────────────────────

def capture_usb(hand, duration, port=None, warmup=2.0):
    port = port or (portR if hand == 'r' else portL)
    # qsize=0 (unbounded) to match TimestampedReadBLE below — ReadSerial's
    # own default (48) silently drops a chunk via `except queue.Full: pass`
    # if the benchmark's consumer loop ever falls more than ~0.5s behind.
    # Never observed in practice (Max interval tops out well under that),
    # but there's no reason a measurement harness should carry even a
    # theoretical silent-drop path when removing it costs nothing.
    reader = ReadSerial(port, baud, qsize=0)
    reader.start()
    reader.send_handshake()

    sensq = reader.getQ()
    buf = bytearray()
    frames = []
    crc_errors = 0

    # Warm-up: discard frames for `warmup` seconds *after the stream is
    # first observed*. This is deliberately a real-time window, not just a
    # queue drain — right after the DTR/RTS hardware reset, the USB-CDC
    # interface is still re-enumerating on the host side, which can cause
    # genuine transient loss/jitter in the first fraction of a second of
    # actual transmission (not merely stale bytes sitting in a buffer).
    print(f"USB: warm-up ({warmup}s) — letting stream settle after reset...")
    warmup_deadline = None
    warmup_discarded = 0
    while warmup_deadline is None or time.perf_counter() < warmup_deadline:
        try:
            raw = sensq.get(timeout=10.0)
        except queue.Empty:
            continue
        buf += raw
        new_frames, _ = extract_frames(buf)
        if new_frames and warmup_deadline is None:
            warmup_deadline = time.perf_counter() + warmup
        warmup_discarded += len(new_frames)
    if warmup_discarded:
        print(f"USB: discarded {warmup_discarded} warm-up frame(s)")

    print(f"USB: capturing for {duration}s...")

    t_start = None
    while t_start is None:
        try:
            raw = sensq.get(timeout=10.0)
        except queue.Empty:
            print("USB: still waiting...")
            continue

        buf += raw
        new_frames, n_err = extract_frames(buf)
        crc_errors += n_err
        for payload, seq, device_us in new_frames:
            now = time.perf_counter()
            if t_start is None:
                t_start = now
            frames.append((now, seq, device_us))

    while (time.perf_counter() - t_start) < duration:
        try:
            raw = sensq.get(timeout=1.0)
        except queue.Empty:
            continue

        buf += raw
        new_frames, n_err = extract_frames(buf)
        crc_errors += n_err
        for payload, seq, device_us in new_frames:
            frames.append((time.perf_counter(), seq, device_us))

    reader.stop()

    print(f"USB: captured {len(frames)} frames (CRC errors: {crc_errors})")

    return frames, crc_errors


def capture_ble(hand, duration, ble_name=None, warmup=2.0):
    """Capture frames over BLE for `duration` seconds."""
    ble_name = ble_name or (BLE_NAME_R if hand == 'r' else BLE_NAME_L)
    reader = TimestampedReadBLE(ble_name, qsize=0)
    reader.start()

    print(f"BLE: scanning for {ble_name} ...")
    if not reader.wait_connected(timeout=30.0):
        print("BLE: connection timeout")
        reader.stop()
        return [], 0

    buf = bytearray()

    print(f"BLE: warm-up ({warmup}s) — letting notifications settle...")
    warmup_deadline = None
    warmup_discarded = 0
    while warmup_deadline is None or time.perf_counter() < warmup_deadline:
        try:
            _, chunk = reader.sensq.get(timeout=10.0)
        except queue.Empty:
            continue
        buf += chunk
        new_frames, _ = extract_frames(buf)
        if new_frames and warmup_deadline is None:
            warmup_deadline = time.perf_counter() + warmup
        warmup_discarded += len(new_frames)
    if warmup_discarded:
        print(f"BLE: discarded {warmup_discarded} warm-up frame(s)")

    frames = []
    crc_errors = 0
    t_start = None

    print(f"BLE: capturing for {duration}s ...")
    while True:
        try:
            t_arrival, chunk = reader.sensq.get(timeout=2.0)
        except queue.Empty:
            if t_start and (time.perf_counter() - t_start) >= duration:
                break
            continue

        buf += chunk
        new_frames, n_err = extract_frames(buf)
        crc_errors += n_err
        for payload, seq, device_us in new_frames:
            if t_start is None:
                t_start = t_arrival
            # All frames extracted from this chunk share t_arrival: at
            # MTU 128 a frame is never fragmented across notifications, so
            # a chunk is either exactly one notification's payload, or
            # bytes left over from a previous partial frame that this
            # notification completed — either way t_arrival is when the
            # frame's last byte actually landed.
            frames.append((t_arrival, seq, device_us))

        if t_start and (time.perf_counter() - t_start) >= duration:
            break

    reader.stop()
    print(f"BLE: captured {len(frames)} frames (CRC errors: {crc_errors})")
    return frames, crc_errors


# ── Analysis ────────────────────────────────────────────────────────────────

def analyze(frames, crc_errors=0):
    """Compute bandwidth statistics from a list of (host_ts, seq, device_us) tuples."""
    if len(frames) < 2:
        return None

    timestamps = np.array([f[0] for f in frames])
    seqs = np.array([f[1] for f in frames], dtype=np.int32)
    device_us = np.array([f[2] for f in frames], dtype=np.int64)

    duration = timestamps[-1] - timestamps[0]
    t_rel = timestamps - timestamps[0]
    intervals = np.diff(timestamps) * 1000  # ms — host reception timing

    # True firmware-clock inter-sample interval, independent of USB/BLE
    # arrival timing. device_us wraps at 2**32 (~71 min); correct for it,
    # though a wrap is very unlikely within a single benchmark run.
    device_intervals = np.diff(device_us) / 1000.0  # ms
    device_intervals = np.where(device_intervals < 0,
                                 device_intervals + (2**32) / 1000.0,
                                 device_intervals)

    # Mean/std hide a rare but large single-event stall (e.g. Windows
    # scheduling the Python process out for 40-60 ms); max makes that
    # visible even when it's a single occurrence in the run.
    max_interval_ms = float(np.max(intervals))

    # Packet loss: count gaps in sequence numbers (uint8 wrapping)
    seq_diffs = np.diff(seqs)
    # Handle wrap: if diff is negative, add 256
    seq_diffs = np.where(seq_diffs < 0, seq_diffs + 256, seq_diffs)
    total_expected = int(np.sum(seq_diffs))
    total_received = len(frames)
    total_lost = total_expected - total_received

    # Instantaneous throughput via Gaussian-kernel smoothing, not a
    # rectangular window (sliding or fixed). A rectangular window still
    # gives every sample a binary in/out membership — a point crossing the
    # window edge causes a hard +/-1 count jump between adjacent steps.
    # That's the same artifact as fixed 1 s bins, just resampled more
    # densely; a Gaussian kernel weights each sample continuously by
    # distance from the query point, so there's no edge to cross and no
    # jump — this is what actually removes the sawtooth/zigzag.
    fine_step = 0.01  # 10 ms, matches the sample period
    if duration > fine_step:
        edges = np.arange(0, duration + fine_step, fine_step)
        counts, _ = np.histogram(t_rel, bins=edges)
        centers_fine = (edges[:-1] + edges[1:]) / 2

        sigma = 0.3  # seconds — smoothing bandwidth (~30 sample periods)
        half_width = int(np.ceil(4 * sigma / fine_step))
        k_x = np.arange(-half_width, half_width + 1) * fine_step
        kernel = np.exp(-0.5 * (k_x / sigma) ** 2)
        kernel /= kernel.sum()

        throughput = np.convolve(counts, kernel, mode='same') / fine_step
        # Renormalize: np.convolve zero-pads outside the array, so near the
        # edges the kernel is missing real mass it should have integrated
        # over — without this, the first/last ~4*sigma seconds read
        # artificially low. Dividing by the same kernel convolved against
        # an all-ones array gives the actual local kernel weight at each
        # point (<1 near edges, ~1 in the interior) and corrects for it.
        edge_weight = np.convolve(np.ones_like(counts, dtype=float), kernel, mode='same')
        throughput = throughput / edge_weight
        bin_centers = centers_fine
    else:
        bin_centers = np.array([duration / 2])
        throughput = np.array([len(t_rel) / max(duration, 1e-9)])

    # Cumulative loss timeline
    per_frame_loss = seq_diffs - 1  # 0 = no gap, >0 = lost frames
    per_frame_loss = np.clip(per_frame_loss, 0, None)
    cum_loss = np.cumsum(per_frame_loss)
    loss_time = t_rel[1:]  # aligns with diff

    return {
        'duration': duration,
        'n_received': total_received,
        'n_expected': total_expected,
        'n_lost': max(0, total_lost),
        'loss_pct': max(0, total_lost) / max(1, total_expected) * 100,
        'crc_errors': crc_errors,
        'mean_hz': total_received / duration,
        'intervals_ms': intervals,
        'jitter_ms': np.std(intervals),
        'max_interval_ms': max_interval_ms,
        'device_jitter_ms': np.std(device_intervals),
        'throughput_hz': throughput,
        'throughput_time': bin_centers,
        'cum_loss': cum_loss,
        'loss_time': loss_time,
    }


# ── Plotting ────────────────────────────────────────────────────────────────

def plot_comparison(results, output_path):
    """2x2 figure comparing USB and BLE performance."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle('Gluvn BLE vs USB Bandwidth Comparison', fontsize=14, fontweight='bold')

    colors = {'usb': '#3498db', 'ble': '#e67e22'}
    labels = {'usb': 'USB Serial', 'ble': 'BLE'}

    # Top-left: throughput over time
    ax = axes[0, 0]
    for mode in ('usb', 'ble'):
        if mode in results and results[mode]:
            r = results[mode]
            ax.plot(r['throughput_time'], r['throughput_hz'],
                    color=colors[mode], label=labels[mode], linewidth=1.5)
    ax.axhline(100, color='gray', linestyle='--', alpha=0.6, label='Target (100 Hz)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Packets / sec')
    ax.set_title('Throughput over time')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Top-right: inter-packet interval histogram
    ax = axes[0, 1]
    for mode in ('usb', 'ble'):
        if mode in results and results[mode]:
            r = results[mode]
            ax.hist(r['intervals_ms'], bins=100, alpha=0.6,
                    color=colors[mode], label=labels[mode], density=True)
    ax.axvline(10, color='gray', linestyle='--', alpha=0.6, label='Expected (10 ms)')
    ax.set_xlabel('Host inter-packet interval (ms)')
    ax.set_ylabel('Density')
    ax.set_title('Inter-packet interval distribution')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Bottom-left: cumulative packet loss
    ax = axes[1, 0]
    for mode in ('usb', 'ble'):
        if mode in results and results[mode]:
            r = results[mode]
            ax.plot(r['loss_time'], r['cum_loss'],
                    color=colors[mode], label=labels[mode], linewidth=1.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Cumulative lost packets')
    ax.set_title('Packet loss timeline')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # Bottom-right: summary bar chart
    ax = axes[1, 1]
    metrics = ['Mean Hz', 'Loss %', 'Jitter (ms)', 'Max interval (ms)']
    x = np.arange(len(metrics))
    width = 0.35
    for i, mode in enumerate(('usb', 'ble')):
        if mode in results and results[mode]:
            r = results[mode]
            vals = [r['mean_hz'], r['loss_pct'], r['jitter_ms'], r['max_interval_ms']]
            ax.bar(x + i * width, vals, width, color=colors[mode], label=labels[mode])
    ax.set_xticks(x + width / 2)
    ax.set_xticklabels(metrics)
    ax.set_title('Summary')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')

    plt.tight_layout(rect=[0, 0.03, 1, 1])
    plt.savefig(output_path, dpi=150)
    print(f"Plot saved to {output_path}")
    plt.show()


def print_summary(results):
    """Console summary table."""
    header = f"{'':>16} {'USB':>12} {'BLE':>12}"
    sep = '-' * 44
    print(f"\n{sep}\n  Benchmark Results\n{sep}")
    print(header)

    def _fmt(res, key, fmt):
        if not res:
            return '—'
        val = res.get(key)
        return 'n/a' if val is None else format(val, fmt)

    for label, key, fmt in [
        ('Mean Hz',    'mean_hz',    '.1f'),
        ('Loss %',     'loss_pct',   '.2f'),
        ('Lost pkts',  'n_lost',     'd'),
        ('CRC errors', 'crc_errors', 'd'),
        ('Jitter (ms)', 'jitter_ms',  '.2f'),
        ('Max interval (ms)', 'max_interval_ms', '.2f'),
        ('Dev jitter (ms)', 'device_jitter_ms', '.2f'),
        ('Duration (s)','duration',  '.1f'),
        ('Frames',     'n_received', 'd'),
    ]:
        usb_val = _fmt(results.get('usb'), key, fmt)
        ble_val = _fmt(results.get('ble'), key, fmt)
        print(f"  {label:>14} {usb_val:>12} {ble_val:>12}")
    print(sep)


def save_raw_csv(frames, path):
    """Dump raw (host_ts, seq, device_us) tuples for offline re-analysis/reproducibility."""
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['host_ts', 'seq', 'device_us'])
        writer.writerows(frames)


def aggregate_trials(results_list):
    """Aggregate a list of per-trial analyze() dicts into summary stats.
    A single 0%-loss run is promising but not proof; this turns N independent
    trials into mean/worst-case figures suitable for reporting."""
    valid = [r for r in results_list if r is not None]
    if not valid:
        return None

    loss = np.array([r['loss_pct'] for r in valid])
    hz = np.array([r['mean_hz'] for r in valid])
    jitter = np.array([r['jitter_ms'] for r in valid])
    dev_jitter = np.array([r['device_jitter_ms'] for r in valid])
    crc = np.array([r['crc_errors'] for r in valid])
    lost = np.array([r['n_lost'] for r in valid])
    max_interval = np.array([r['max_interval_ms'] for r in valid])

    return {
        'n_trials': len(valid),
        'loss_pct_mean': float(loss.mean()),
        'loss_pct_max': float(loss.max()),
        'mean_hz_mean': float(hz.mean()),
        'mean_hz_std': float(hz.std()),
        'jitter_ms_mean': float(jitter.mean()),
        'device_jitter_ms_mean': float(dev_jitter.mean()),
        # Worst single-event stall across all trials — a mean/max of per-trial
        # values would still hide a one-off 40-60 ms stall; take the max of
        # maxes instead.
        'max_interval_ms_worst': float(max_interval.max()),
        'total_crc_errors': int(crc.sum()),
        'total_lost': int(lost.sum()),
        'any_loss': bool((lost > 0).any()),
        'any_crc_errors': bool((crc > 0).any()),
    }


def print_aggregate(label, agg):
    sep = '-' * 44
    if agg is None:
        print(f"\n{label}: no valid trials to aggregate")
        return
    print(f"\n{sep}\n  {label} — Aggregate over {agg['n_trials']} trial(s)\n{sep}")
    print(f"  Loss %:            mean {agg['loss_pct_mean']:.2f}   max {agg['loss_pct_max']:.2f}")
    print(f"  Mean Hz:           mean {agg['mean_hz_mean']:.2f}   std {agg['mean_hz_std']:.3f}")
    print(f"  Jitter (ms):       mean {agg['jitter_ms_mean']:.2f}")
    print(f"  Dev jitter (ms):   mean {agg['device_jitter_ms_mean']:.3f}")
    print(f"  Max interval (ms): worst {agg['max_interval_ms_worst']:.2f}")
    print(f"  Total CRC errors:  {agg['total_crc_errors']}")
    print(f"  Total lost pkts:   {agg['total_lost']}")
    print(f"  Any trial w/ loss:        {'YES' if agg['any_loss'] else 'no'}")
    print(f"  Any trial w/ CRC errors:  {'YES' if agg['any_crc_errors'] else 'no'}")
    print(sep)


def run_transport(mode, args):
    """Run `args.trials` captures for one transport. Returns (last_frames, results_list)."""
    results_list = []
    last_frames = None

    for t in range(1, args.trials + 1):
        if args.trials > 1:
            print(f"\n--- {mode.upper()} trial {t}/{args.trials} ---")
            if t > 1:
                # BLE: onDisconnect() only restarts advertising once the
                # previous NimBLE connection has actually torn down; giving
                # that a beat before the next wait_connected() scan avoids
                # a race against our own just-closed connection. USB:
                # ReadSerial's DTR/RTS reset needs a moment before the next
                # trial's own reset+re-enumeration begins. 1s covers both
                # without meaningfully lengthening a multi-trial run.
                time.sleep(1.0)

        if mode == 'usb':
            frames, crc_errors = capture_usb(
                args.hand, args.duration, port=args.port, warmup=args.warmup)
        else:
            frames, crc_errors = capture_ble(
                args.hand, args.duration, ble_name=args.ble_name, warmup=args.warmup)

        result = analyze(frames, crc_errors)
        results_list.append(result)
        last_frames = frames

        if args.save_raw:
            os.makedirs(args.save_raw, exist_ok=True)
            csv_path = os.path.join(args.save_raw, f"{mode}_trial{t}.csv")
            save_raw_csv(frames, csv_path)
            print(f"{mode.upper()}: raw frames saved to {csv_path}")

        if result and args.trials > 1:
            print(f"{mode.upper()} trial {t}: loss={result['loss_pct']:.2f}%  "
                  f"hz={result['mean_hz']:.1f}  jitter={result['jitter_ms']:.2f}ms  "
                  f"dev_jitter={result['device_jitter_ms']:.2f}ms")

    return last_frames, results_list


# ── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Gluvn BLE vs USB bandwidth benchmark')
    parser.add_argument('--duration', type=int, default=30, help='Seconds per transport (default: 30)')
    parser.add_argument('--hand', choices=['r', 'l'], default='r', help='Hand to test (default: r)')
    parser.add_argument('--mode', choices=['usb', 'ble', 'both'], default='both', help='Transport to test')
    parser.add_argument('--output', default=os.path.join(figDir, 'benchmark_results.png'),
                        help='Output plot filename (default: figures/benchmark_results.png)')
    parser.add_argument('--port', default=None, help='Serial port override')
    parser.add_argument('--ble-name', default=None, help='BLE device name override')
    parser.add_argument('--warmup', type=float, default=2.0,
                        help='Seconds of post-connect stream to discard before measuring (default: 2.0)')
    parser.add_argument('--trials', type=int, default=1,
                        help='Repeat each transport capture N times and report aggregate stats (default: 1)')
    parser.add_argument('--save-raw', default=None,
                        help='Directory to save raw per-frame (host_ts, seq, device_us) CSVs, one per trial')
    args = parser.parse_args()

    results = {}

    if args.mode in ('usb', 'both'):
        print("=" * 44)
        print("  USB Serial Benchmark")
        print("=" * 44)
        _, usb_trials = run_transport('usb', args)
        results['usb'] = usb_trials[-1] if usb_trials else None
        if args.trials > 1:
            print_aggregate('USB', aggregate_trials(usb_trials))

        if args.mode == 'both':
            print("\nPausing 3s before BLE test ...\n")
            time.sleep(3)

    if args.mode in ('ble', 'both'):
        print("=" * 44)
        print("  BLE Benchmark")
        print("=" * 44)
        _, ble_trials = run_transport('ble', args)
        results['ble'] = ble_trials[-1] if ble_trials else None
        if args.trials > 1:
            print_aggregate('BLE', aggregate_trials(ble_trials))

    if any(results.values()):
        print_summary(results)
        plot_comparison(results, args.output)
    else:
        print("No data captured — check connections.")


if __name__ == '__main__':
    main()