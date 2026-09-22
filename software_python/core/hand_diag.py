"""
core/hand_diag.py -- per-hand runtime health report, owned by the PoseProvider.

You do not call this. Pass ``diagnostics=True`` to MusicApp (or PoseProvider) and the
provider feeds it from its dispatcher threads, so EVERY app gets the report every 2 s:

    app = ChoirMovingWindow(..., diagnostics=True)

One line per hand:
  fps          frames/s dispatched by the provider
  no_imu/no_la frames missing the imu slot / lin_accel (a stream not enabled or not confirmed)
  armed%       fraction of frames with nav_armed True (NavEKF's position/accel are only
               meaningful while armed)
  accel_pk     peak windowed accel this interval (0-127) vs the burst threshold: tells you
               whether a strike is too weak or the burst path is starved
  fw y/p/r     the FIRMWARE's own yaw/pitch/roll, unmodified
  head/tilt/twist   the same angles named for the arm (see core.pose_provider.MOUNTINGS)
  press/flex   max raw value seen this interval (do this hand's finger channels move?)

TO CHECK YOUR MOUNTING (20 seconds): raise your forearm and lower it. `tilt` must rise
and fall while `twist` stays put; rotate your wrist and `twist` must move while `tilt`
stays put. If they are swapped or `tilt` falls when you raise, change
MotionConfig.sensor_mounting (or the choir's sensor_mounting=) to the row that fits.
"""

import time


class HandDiag:
    def __init__(self, hands, interval_sec=2.0, enabled=True):
        self.hands = list(hands)
        self.interval = interval_sec
        self.enabled = enabled
        self._t0 = time.monotonic()
        self._reset()
        self._fw = {h: None for h in self.hands}
        self._arm = {h: None for h in self.hands}

    def _reset(self):
        z = lambda v=0: {h: v for h in self.hands}
        self.n = z()
        self.no_imu = z()
        self.no_la = z()
        self.armed = z()
        self.armed_seen = z()
        self.accel_pk = z(0)
        self.press_max = {h: [0] * 5 for h in self.hands}
        self.flex_max = {h: [0] * 5 for h in self.hands}

    def frame(self, hand, d):
        if not self.enabled or hand not in self.n:
            return
        self.n[hand] += 1
        if 'imu' not in d:
            self.no_imu[hand] += 1
        if 'lin_accel' not in d:
            self.no_la[hand] += 1
        if 'nav_armed' in d:
            self.armed_seen[hand] += 1
            self.armed[hand] += bool(d['nav_armed'])
        for key, store in (('press', self.press_max), ('flex', self.flex_max)):
            vals = d.get(key)
            if vals is not None:
                for i, v in enumerate(vals[:5]):
                    if v > store[hand][i]:
                        store[hand][i] = v

    def accel(self, hand, scaled):
        if self.enabled and hand in self.accel_pk and scaled > self.accel_pk[hand]:
            self.accel_pk[hand] = scaled

    def angles(self, hand, fw_ypr, arm_ypr):
        if self.enabled and hand in self._fw:
            self._fw[hand], self._arm[hand] = fw_ypr, arm_ypr

    def maybe_report(self, accel_thresh):
        if not self.enabled:
            return
        now = time.monotonic()
        dt = now - self._t0
        if dt < self.interval:
            return
        lines = [f"[diag {dt:.1f}s] burst fires when accel_pk > {accel_thresh}"]
        for h in self.hands:
            n = self.n[h]
            if n == 0:
                lines.append(f"  {h.upper()}: NO FRAMES reached the provider this interval")
                continue
            armed = (100.0 * self.armed[h] / self.armed_seen[h]) if self.armed_seen[h] else float('nan')
            fw, ar = self._fw[h], self._arm[h]
            ang = ("angles n/a" if fw is None else
                   f"fw y/p/r={fw[0]:+6.1f}/{fw[1]:+6.1f}/{fw[2]:+6.1f}  "
                   f"head={ar[0]:+6.1f} tilt={ar[1]:+6.1f} twist={ar[2]:+6.1f}")
            lines.append(
                f"  {h.upper()}: fps={n / dt:5.1f} no_imu={self.no_imu[h]} no_la={self.no_la[h]} "
                f"armed={armed:5.1f}% accel_pk={self.accel_pk[h]:3d} {ang} "
                f"press_max={self.press_max[h]} flex_max={self.flex_max[h]}")
        print("\n".join(lines))
        self._t0 = now
        self._reset()
