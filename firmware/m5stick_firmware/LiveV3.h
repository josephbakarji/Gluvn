#pragma once
// =====================================================================================================================
// LIVE3: compact, reconnect-tolerant BLE transport for live instruments (Tut-Tonnetz v2, 14 Nov 2026).
//
// Why (evidence from the 12-13 Nov live sessions, capture logs):
//   * research capture v2 sends one 177-byte frame per 125 Hz sample (22 kB/s per glove, 44 kB/s on one Windows
//     radio) plus 7 JSON control frames per second; during radio stalls the queue filled (F02: 1.6 s stalls on BOTH
//     gloves at once, 387 frames lost, transport latency p99 0.62 s) and a backlog drains one sample per notify;
//   * research rules make any disconnect fatal on both ends (firmware stops the capture, host disables reconnect).
// LIVE3 keeps research capture v2 untouched and adds an instrument mode:
//   * 34-byte samples (bias-corrected IMU as int16 + raw 12-bit ADC), 12-byte header, CRC16: 46 B for one sample;
//   * adaptive batching: normally 1 sample per notification (lowest latency); a backlog drains up to 6 samples per
//     notification (6x faster catch-up);
//   * latency bound: samples older than maxLagUs are skipped in the real-time stream (kept in the history ring);
//   * history ring (RING samples) + LIVE3_RESEND so the host can recover any gap for the take log;
//   * a disconnect is NOT fatal: sampling continues into the ring, fast advertising restarts, and streaming resumes
//     on re-subscription with continuous sequence numbers (flag RESUMED on the first packet);
//   * 1 Hz binary status packet (battery, queue, drops, retries, reconnects) instead of JSON telemetry;
//   * estimators (Mahony / NavEKF) and the SPI display are idle while LIVE3 is active (steadier 100 Hz timing).
// Commands (ASCII, BLE RX or USB):  LIVE3_INFO | LIVE3_START:<hz>:<batch>[:<max_lag_ms>] | LIVE3_STOP |
//                                   LIVE3_RESEND:<seq>:<n> | LIVE3_PING
// Packets (first byte = type, little-endian):
//   0xB3 samples  [type][hand][flags][n][seq0 u32][t0_us u32] n x {dt u16 (8 us units from t0), ax ay az i16
//                 (g*4096), gx gy gz i16 (dps*16), flex0..4 u16 (ADC), fsr0..4 u16 (ADC)} [crc16 u16]
//                 flags: 1 RESENT, 2 SKIPPED_BEFORE (gap), 4 LOW_BATTERY, 8 IMU_STALE(any sample), 16 RESUMED
//   0xB4 status   [type][hand][battery %][queue_high][dropped u16][retries u16][reconnects u16][skipped u16]
//                 [seq u32][conn_interval_us/10 u16][mtu u16][crc16]
//   0xB5 info     [type][hand][part][parts][len][utf-8 json ...][crc16]
// CRC16-CCITT (0x1021, init 0xFFFF) over all preceding bytes of the packet.
// =====================================================================================================================
#include <Arduino.h>
#include <atomic>

class LiveV3Runtime {
public:
  static constexpr uint8_t PKT_SAMPLES = 0xB3, PKT_STATUS = 0xB4, PKT_INFO = 0xB5;
  static constexpr uint8_t F_RESENT = 1, F_SKIPPED = 2, F_LOWBAT = 4, F_IMU_STALE = 8, F_RESUMED = 16;
  static constexpr size_t SAMPLE_BYTES = 34, HEADER_BYTES = 12, MAX_BATCH = 6;
  static constexpr size_t RING = 300;                    // 3 s at 100 Hz (10.2 kB + 1.2 kB)
  static constexpr uint32_t IDLE_STOP_MS = 120000;       // stop LIVE3 after 2 min without a connection

  std::atomic<bool> active{false};
  uint32_t intervalUs = 10000;
  uint8_t batch = 1;
  uint32_t maxLagUs = 300000;
  std::atomic<bool> subscribed{false};
  std::atomic<bool> connected{false};
  std::atomic<uint16_t> mtu{23};
  std::atomic<uint32_t> connIntervalUs{0};
  std::atomic<uint16_t> connHandle{0xFFFF};
  std::atomic<uint16_t> reconnects{0};

  // ------------------------------------------------------------------ commands
  bool command(const String& raw, NimBLEServer* server) {
    String cmd = raw; cmd.trim(); cmd.toUpperCase();
    if (!cmd.startsWith("LIVE3_")) return false;
    if (cmd == "LIVE3_INFO") { queueInfo(); return true; }
    if (cmd == "LIVE3_PING") { statusDue = true; return true; }
    if (cmd == "LIVE3_STOP") { stop(); return true; }
    if (cmd.startsWith("LIVE3_START")) {
      uint32_t hz = 100, b = 1, lag = 300;
      int p1 = cmd.indexOf(':'), p2 = p1 >= 0 ? cmd.indexOf(':', p1 + 1) : -1, p3 = p2 >= 0 ? cmd.indexOf(':', p2 + 1) : -1;
      if (p1 >= 0) hz = cmd.substring(p1 + 1, p2 >= 0 ? p2 : cmd.length()).toInt();
      if (p2 >= 0) b = cmd.substring(p2 + 1, p3 >= 0 ? p3 : cmd.length()).toInt();
      if (p3 >= 0) lag = cmd.substring(p3 + 1).toInt();
      if (hz < 25 || hz > 200 || 1000000UL % hz != 0) hz = 100;
      if (b < 1 || b > MAX_BATCH) b = 1;
      if (lag < 50 || lag > 3000) lag = 300;
      start(hz, (uint8_t)b, lag * 1000UL, server);
      return true;
    }
    if (cmd.startsWith("LIVE3_RESEND:")) {
      int p1 = cmd.indexOf(':'), p2 = cmd.indexOf(':', p1 + 1);
      if (p2 > p1) {
        uint32_t s = (uint32_t)strtoul(cmd.substring(p1 + 1, p2).c_str(), nullptr, 10);
        uint32_t n = (uint32_t)cmd.substring(p2 + 1).toInt();
        if (n > 0 && resendCount < RESEND_SLOTS) {
          resendSeq[(resendHead + resendCount) % RESEND_SLOTS] = s;
          resendN[(resendHead + resendCount) % RESEND_SLOTS] = n > RING ? RING : n;
          ++resendCount;
        }
      }
      return true;
    }
    return true;   // unknown LIVE3_ command: consumed, ignored
  }

  void start(uint32_t hz, uint8_t b, uint32_t lagUs, NimBLEServer* server) {
    intervalUs = 1000000UL / hz; batch = b; maxLagUs = lagUs;
    if (!active) { resendCount = 0; resumedFlag = false; skippedFlag = false; seq = 0; sentUpTo = 0; dropped = 0; skipped = 0; retries = 0; queueHigh = 0; }
    active = true; idleSinceMs = 0; statusDue = true;
    uint16_t h = connHandle.load();
    // 7.5-10 ms interval, no peripheral latency, 2 s supervision timeout (faster loss detection -> faster recovery)
    if (server && h != 0xFFFF) server->updateConnParams(h, 6, 8, 0, 200);
  }

  void stop() { active = false; resendCount = 0; }

  // ------------------------------------------------------------------ BLE callbacks (NimBLE task)
  void onConnect(uint16_t handle, uint16_t m, uint32_t intervalUs_) {
    connHandle = handle; mtu = m; connIntervalUs = intervalUs_; connected = true;
    if (active) { ++reconnects; resumedFlag = true; }
  }
  void onDisconnect() { connected = false; subscribed = false; connHandle = 0xFFFF; }
  void onSubscribe(bool on) { subscribed = on; if (on && active) statusDue = true; }

  // ------------------------------------------------------------------ acquisition (loop task, every sample tick)
  void emit(uint32_t sampleUs, float ax_, float ay_, float az_, float gx_, float gy_, float gz_,
            const uint16_t* flex, const uint16_t* fsr, bool imuFresh) {
    if (!active) return;
    size_t slot = seq % RING;
    uint8_t* p = ring[slot];
    int16_t v[6] = {clamp16(ax_ * 4096.0f), clamp16(ay_ * 4096.0f), clamp16(az_ * 4096.0f),
                    clamp16(gx_ * 16.0f), clamp16(gy_ * 16.0f), clamp16(gz_ * 16.0f)};
    size_t k = 2;                                  // bytes 0..1 = dt, written at packet build time
    for (int i = 0; i < 6; ++i) { p[k++] = (uint8_t)(v[i] & 0xFF); p[k++] = (uint8_t)((uint16_t)v[i] >> 8); }
    for (int i = 0; i < 5; ++i) { p[k++] = flex[i] & 0xFF; p[k++] = flex[i] >> 8; }
    for (int i = 0; i < 5; ++i) { p[k++] = fsr[i] & 0xFF; p[k++] = fsr[i] >> 8; }
    ringUs[slot] = sampleUs; ringStale[slot] = !imuFresh;
    ++seq;
    // overflow: the oldest unsent sample is about to be overwritten
    if (seq - sentUpTo > RING - 1) { uint32_t lost = seq - sentUpTo - (RING - 1); sentUpTo += lost; dropped += lost; skippedFlag = true; }
    uint32_t backlog = seq - sentUpTo;
    if (backlog > queueHigh) queueHigh = backlog > 255 ? 255 : backlog;
  }

  // ------------------------------------------------------------------ transmission (loop task, every iteration)
  void service(NimBLECharacteristic* tx, uint8_t handChar, bool lowBattery, int batteryPct) {
    // Calibration handshake must work before acquisition is started.
    if (connected && subscribed && tx && infoLen) { sendInfo(tx, handChar); return; }
    if (!active) { return; }
    uint32_t nowMs = millis();
    if (!connected) {
      if (!idleSinceMs) idleSinceMs = nowMs;
      if (nowMs - idleSinceMs > IDLE_STOP_MS) stop();
      return;
    }
    idleSinceMs = 0;
    if (!subscribed || !tx) return;
    if (statusDue || nowMs - lastStatusMs >= 1000) { if (sendStatus(tx, handChar, batteryPct)) { statusDue = false; lastStatusMs = nowMs; } return; }
    uint32_t nowUs = micros();
    // latency bound: skip samples older than maxLagUs (they stay in the ring for LIVE3_RESEND)
    while (seq - sentUpTo > 1 && (uint32_t)(nowUs - ringUs[sentUpTo % RING]) > maxLagUs) { ++sentUpTo; ++skipped; skippedFlag = true; }
    uint32_t backlog = seq - sentUpTo;
    bool due = backlog >= batch || (backlog > 0 && (uint32_t)(nowUs - ringUs[sentUpTo % RING]) >= intervalUs * batch);
    if (due) {
      uint8_t n = backlog > MAX_BATCH ? MAX_BATCH : (uint8_t)backlog;
      size_t cap = mtu.load() > 3 ? mtu.load() - 3 : 20;
      while (n > 1 && HEADER_BYTES + n * SAMPLE_BYTES + 2 > cap) --n;
      if (HEADER_BYTES + n * SAMPLE_BYTES + 2 > cap) return;      // MTU too small for LIVE3 (needs >= 51)
      uint8_t flags = (skippedFlag ? F_SKIPPED : 0) | (resumedFlag ? F_RESUMED : 0) | (lowBattery ? F_LOWBAT : 0);
      size_t len = build(sentUpTo, n, flags, handChar);
      if (tx->notify(packet, len)) { sentUpTo += n; skippedFlag = false; resumedFlag = false; }
      else { ++retries; }
      return;
    }
    if (resendCount) {                                   // idle radio time: serve one resend request
      uint32_t s = resendSeq[resendHead], n = resendN[resendHead];
      if (seq - s > RING - 1 || seq == s) { popResend(); return; }   // no longer (or not yet) in history
      uint8_t m = n > MAX_BATCH ? MAX_BATCH : (uint8_t)n;
      size_t cap = mtu.load() > 3 ? mtu.load() - 3 : 20;
      while (m > 1 && HEADER_BYTES + m * SAMPLE_BYTES + 2 > cap) --m;
      if (HEADER_BYTES + m * SAMPLE_BYTES + 2 > cap) return;
      if (m > seq - s) m = (uint8_t)(seq - s);
      size_t len = build(s, m, F_RESENT, handChar);
      if (tx->notify(packet, len)) {
        if (n > m) { resendSeq[resendHead] = s + m; resendN[resendHead] = n - m; } else popResend();
      } else ++retries;
    }
  }

  void queueInfo() {
    char finger[300];
    snprintf(finger, sizeof(finger), "\"min_flex\":[%d,%d,%d,%d,%d],\"max_flex\":[%d,%d,%d,%d,%d],\"min_press\":[%d,%d,%d,%d,%d],\"max_press\":[%d,%d,%d,%d,%d]",
             MIN_FLEX[0], MIN_FLEX[1], MIN_FLEX[2], MIN_FLEX[3], MIN_FLEX[4], MAX_FLEX[0], MAX_FLEX[1], MAX_FLEX[2], MAX_FLEX[3], MAX_FLEX[4],
             MIN_PRESS[0], MIN_PRESS[1], MIN_PRESS[2], MIN_PRESS[3], MIN_PRESS[4], MAX_PRESS[0], MAX_PRESS[1], MAX_PRESS[2], MAX_PRESS[3], MAX_PRESS[4]);
    infoLen = snprintf(info, sizeof(info),
      "{\"protocol\":\"live3\",\"version\":1,\"boot_id\":\"%s\",\"active\":%s,\"hand\":\"%c\",\"rate_hz\":%lu,\"batch\":%u,\"ring\":%u,"
      "\"acc_lsb_per_g\":4096,\"gyro_lsb_per_dps\":16,\"dt_unit_us\":8,\"gyro_bias_dps\":[%.4f,%.4f,%.4f],"
      "\"accel_bias_g\":[%.4f,%.4f,%.4f],\"calibration_result\":\"%s\",%s,\"build\":\"%s %s\"}",
      captureV2.boot, active ? "true" : "false", hand, (unsigned long)(1000000UL / intervalUs), batch, (unsigned)RING,
      ACTIVE_GYRO_BIAS[0], ACTIVE_GYRO_BIAS[1], ACTIVE_GYRO_BIAS[2], ACCEL_BIAS[0], ACCEL_BIAS[1], ACCEL_BIAS[2],
      captureV2.calibrationResult, finger, __DATE__, __TIME__);
    if (infoLen >= sizeof(info)) infoLen = sizeof(info) - 1;
    infoOffset = 0;
  }

  uint32_t sequence() const { return seq; }
  uint32_t droppedCount() const { return dropped; }

private:
  static constexpr size_t RESEND_SLOTS = 8;
  uint8_t ring[RING][SAMPLE_BYTES];
  uint32_t ringUs[RING];
  bool ringStale[RING];
  uint8_t packet[256];
  char info[700];
  size_t infoLen = 0, infoOffset = 0;
  uint32_t seq = 0, sentUpTo = 0, dropped = 0, skipped = 0, retries = 0;
  uint8_t queueHigh = 0;
  bool skippedFlag = false;
  std::atomic<bool> resumedFlag{false}, statusDue{false};
  uint32_t lastStatusMs = 0, idleSinceMs = 0;
  uint32_t resendSeq[RESEND_SLOTS], resendN[RESEND_SLOTS];
  size_t resendHead = 0, resendCount = 0;

  static int16_t clamp16(float x) { return x > 32767.f ? 32767 : (x < -32768.f ? -32768 : (int16_t)lrintf(x)); }
  void popResend() { resendHead = (resendHead + 1) % RESEND_SLOTS; --resendCount; }
  static void put32(uint8_t* p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }
  static void put16(uint8_t* p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
  static uint16_t crc(const uint8_t* d, size_t n) {
    uint16_t c = 0xFFFF;
    for (size_t i = 0; i < n; ++i) { c ^= (uint16_t)d[i] << 8; for (int b = 0; b < 8; ++b) c = (c & 0x8000) ? (c << 1) ^ 0x1021 : (c << 1); }
    return c;
  }

  size_t build(uint32_t s0, uint8_t n, uint8_t flags, uint8_t handChar) {
    uint32_t t0 = ringUs[s0 % RING];
    bool stale = false;
    packet[0] = PKT_SAMPLES; packet[1] = handChar; packet[3] = n;
    put32(packet + 4, s0); put32(packet + 8, t0);
    size_t k = HEADER_BYTES;
    for (uint8_t i = 0; i < n; ++i) {
      size_t slot = (s0 + i) % RING;
      memcpy(packet + k, ring[slot], SAMPLE_BYTES);
      uint32_t dt = (ringUs[slot] - t0) / 8U;
      put16(packet + k, dt > 65535 ? 65535 : (uint16_t)dt);
      stale |= ringStale[slot];
      k += SAMPLE_BYTES;
    }
    packet[2] = flags | (stale ? F_IMU_STALE : 0);
    put16(packet + k, crc(packet, k)); k += 2;
    return k;
  }

  bool sendStatus(NimBLECharacteristic* tx, uint8_t handChar, int batteryPct) {
    uint8_t* p = packet;
    p[0] = PKT_STATUS; p[1] = handChar; p[2] = batteryPct < 0 ? 255 : (uint8_t)batteryPct; p[3] = queueHigh;
    put16(p + 4, dropped > 65535 ? 65535 : dropped); put16(p + 6, retries > 65535 ? 65535 : retries);
    put16(p + 8, reconnects.load()); put16(p + 10, skipped > 65535 ? 65535 : skipped);
    put32(p + 12, seq); put16(p + 16, (uint16_t)(connIntervalUs.load() / 10)); put16(p + 18, mtu.load());
    put16(p + 20, crc(p, 20));
    queueHigh = 0;
    return tx->notify(p, 22);
  }

  bool sendInfo(NimBLECharacteristic* tx, uint8_t handChar) {
    size_t cap = mtu.load() > 3 ? mtu.load() - 3 : 20;
    size_t room = 13;  // safe at the default ATT MTU of 23, independent of MTU changes
    uint8_t parts = (uint8_t)((infoLen + room - 1) / room), part = (uint8_t)(infoOffset / room);
    size_t n = infoLen - infoOffset; if (n > room) n = room;
    packet[0] = PKT_INFO; packet[1] = handChar; packet[2] = part; packet[3] = parts; packet[4] = (uint8_t)n;
    memcpy(packet + 5, info + infoOffset, n);
    put16(packet + 5 + n, crc(packet, 5 + n));
    if (!tx->notify(packet, 7 + n)) { ++retries; return true; }
    infoOffset += n;
    if (infoOffset >= infoLen) { infoLen = 0; infoOffset = 0; }
    return true;
  }
};

LiveV3Runtime live3;
