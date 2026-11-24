# LIVE3: instrument transport (added 14 Nov 2026 for Tut-Tonnetz v2)

LIVE3 is an additive mode. Research capture v2 (`CaptureV2*.h`) and the legacy stream are unchanged, and LIVE3 is mutually exclusive with research capture.

## Why

These findings come from the capture logs of the live sessions on 12–13 Nov:
- **One large frame per sample.** Each sample went out as its own 177-byte notification at 125 Hz, plus seven JSON telemetry frames per second. That is 22 kB/s per glove on one Windows radio.
- **Stalls dropped data.** In F02, a stall of 1.6 s hit both gloves at once and 387 frames were lost. Transport latency reached p99 0.62 s.
- **Disconnects were fatal.** The firmware stopped the capture and the host had reconnection disabled.

## What

| Aspect | LIVE3 behaviour |
|---|---|
| Sample size | 34 bytes: bias-corrected IMU as int16 (g×4096, dps×16) and raw 12-bit ADC |
| Packet | 12-byte header, CRC16-CCITT, little-endian |
| Batching | Normally one sample per notification. When the queue backs up, a packet carries up to 6 samples |
| Latency bound | Samples older than `max_lag` (default 300 ms) are skipped in real time. They stay in a 3 s history ring, and `LIVE3_RESEND` can recover them |
| Disconnect | Not fatal. Sampling continues, advertising restarts at 20–40 ms, and streaming resumes on re-subscription with continuous sequence numbers (RESUMED flag) |
| Status | One 22-byte binary status packet per second: battery, queue, drops, retries, reconnects, connection interval, MTU |
| Radio | Requests a 7.5–10 ms interval with a 2 s supervision timeout. TX power is raised to +9 dBm |
| Idle in LIVE3 | Mahony/NavEKF and the SPI display redraw |
| Sample rate | Set by `LIVE3_START:<hz>:<batch>:<max_lag_ms>`; the default is 100 Hz |
| Glove identity | `-DGLUVN_LEFT` builds the left glove; the default is the right glove |

The full byte layout is in the comment at the top of `LiveV3.h`. The host side is `Tut-Tonnetz/software_python/tut_tonnetz_v2/tt2/live/{protocol_v3,ble_v3}.py`.

## Status

The code is written but has not been compiled on this PC, because NimBLE-Arduino is not installed. It has not been flashed or tested with the gloves. Build, flash and test commands are in `Tut-Tonnetz/software_python/tut_tonnetz_v2/COMMANDS.md`, section 5.

If the research recorder is used with this build, regenerate `CaptureV2Build.h` (`prepare_capture_build.py`). Until then the reported source hash predates LIVE3.
