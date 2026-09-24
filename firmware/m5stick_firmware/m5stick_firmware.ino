/*
 * Gluvn-M5 — M5StickC Plus 1.1
 * BLE NUS (Nordic UART Service) + USB Serial
 *
 * Hardware:
 *   5x FSR  on CD74HC4067 MUX CH0–4   (10K pull-down, 103 cap)
 *   5x Flex on CD74HC4067 MUX CH5–9   (47K pull-down, 104 cap)
 *   Built-in IMU (BMI270 / MPU6886) via M5Unified
 *
 * Payload (normal mode, base len=29; +8 QUAT, +8 NAV_QUAT, +19 MOTION, any combination):
 *   [hand:1B] [flags:1B] [seq:1B] [device_us:4B] [yaw:2B] [pitch:2B] [roll:2B]
 *   [imu0:2B][imu1:2B][imu2:2B] [flex0-4:5B] [fsr0-4:5B]
 *   [qw,qx,qy,qz:2B each]     <- QUAT_STREAM_ON only
 *   [nqw,nqx,nqy,nqz:2B each] <- NAV_QUAT_STREAM_ON only
 *   [velX,velY,velZ,posX,posY,posZ,laX,laY,laZ:2B each][zupt_quality:1B] <- MOTION_STREAM_ON only
 *
 *   vel/pos: int16, world-frame, /32767*RANGE recovers float (8.0 m/s, 2.0 m
 *   — MOTION_VEL_RANGE_MPS/MOTION_POS_RANGE_M). la (linear accel): int16,
 *   world-frame, gravity already subtracted by NavEKF::getWorldAcceleration()
 *   — zero at rest, /32767*LIN_ACCEL_RANGE_MPS2 (+-4g) recovers m/s^2.
 *   zupt_quality: uint8, NavEKF::getZuptMahalanobis() (chi-square fit for the
 *   last ZUPT candidate; near 0 = confident stationary fit) clamped to
 *   [0,ZUPT_MAHALANOBIS_CAP] and scaled to [0,254]; 255 = "never computed
 *   yet" sentinel, kept out of the numeric range on purpose. Soft-scaling
 *   companion to the isZuptStarved flag bit. Position is relative
 *   displacement since the last navEkf.reset() (RESET_POSITION), not an
 *   absolute session position.
 *
 *   flags (byte 2, always present): bit0=calibration_mode, bit1=send_quat,
 *   bit2=send_nav_quat, bit3=send_motion, bit4=isArmed, bit5=isZuptActive,
 *   bit6=isZuptStarved, bit7=low_battery. Bits 4-6 are meaningful ONLY when
 *   bit3 is set — NavEKF may not even be running otherwise (see
 *   navEkfNeeded gating); a host should ignore bits 4-6 when bit3 is 0.
 *   isArmed clear = vel/pos/lin_accel not yet trustworthy (~0.3s re-arm
 *   window) — hard-gate on this bit.
 *
 *   Frame length alone does NOT determine which optional blocks are present
 *   (several base+block combinations share a total length) — a host must
 *   decode flags first and use that, not length, to pick the struct; length
 *   is still checked against what flags imply, as an integrity cross-check.
 *
 *   payload (calibration mode, len=39, +8/+8/+19 for the same additions):
 *   same 19-byte header; sensors sent as raw uint16 [0,4095] instead of the
 *   8-bit calibrated form; same optional trailing blocks as above.
 *
 * Serial / BLE commands (ASCII lines, '\n'-terminated):
 *   HAND_R / HAND_L       set active hand
 *   CAL_START / CAL_STOP  enter/exit calibration mode (raw 16-bit output)
 *   RAW_MODE / CAL_MODE   print raw/calibrated values to serial; pauses binary stream
 *   NORMAL_MODE           resume binary streaming
 *   RESET_YAW             display-only: zero ahrs's yaw/pitch/roll readout at current
 *                         heading. Does NOT touch ahrs's quaternion or NavEKF's world
 *                         frame. See RESET_POSITION / RECALIBRATE_GYRO for those.
 *   RESET_POSITION        zero navEkf_'s position AND velocity; re-arms from the next
 *                         confirmed-stationary sample. On-demand drift clear — position
 *                         is deliberately not auto-reset by ZUPT. Trigger at rest.
 *   QUAT_STREAM_ON/OFF    include Mahony qw,qx,qy,qz in every frame. Opt-in,
 *                         mode-independent.
 *   NAV_QUAT_STREAM_ON/OFF include NavEKF qw,qx,qy,qz in every frame. Opt-in.
 *   MOTION_STREAM_ON/OFF  include world-frame vel+pos+lin_accel+zupt_quality in every
 *                         frame (+19B), sets flags bits 4-6 meaningfully. Opt-in,
 *                         composable with QUAT/NAV_QUAT/USE_GYRO.
 *   USE_GYRO_ON/OFF       telemetry encoding only: selects whether imu0/1/2 slots carry
 *                         gyro or accel counts. Does not gate estimator input — Mahony
 *                         and NavEKF always consume both regardless.
 *   RECALIBRATE_GYRO      re-run stationary startup gyro calibration on demand.
 *                         Resets AHRS's residual-bias tracker and re-anchors yaw.
 *   NAV_DIAG_ON/OFF       text-only per-sample nav estimator diagnostics (10 Hz;
 *                         requires binary streaming off).
 *   SYS_DIAG_ON/OFF       text-only system/stability diagnostics (1 Hz; requires
 *                         binary streaming off): reset reason, uptime, heap, stack
 *                         high-water mark, EKF/loop/calib timing, IMU sample counts,
 *                         dt extremes, NaN/Inf counters, quaternion norms, covariance
 *                         range + blowup tripwire, battery.
 *   STATUS                one-shot SYS_DIAG line, independent of the toggle above —
 *                         still requires binary streaming off.
 *   DIAG_RESET_MAX        clear since-boot diagnostic maxes/counts. Does not touch
 *                         estimator state or cumulative identity counters.
 *   PREFS_SAVE            persist flex/press/accel calibration to NVS. Gyro bias is
 *                         never persisted (runtime-only, always).
 *   SET_CAL:H:TYPE:v0,v1,v2,v3,v4   (flex/press: 5 comma-separated ints)
 *   SET_CAL:H:ACCEL_BIAS:ax,ay,az   (3 floats, g)
 *     H = R | L
 *     TYPE = MIN_FLEX | MAX_FLEX | MIN_PRESS | MAX_PRESS | ACCEL_BIAS
 *   GYRO_BIAS is intentionally NOT host-settable: it is solved exclusively by
 *   RECALIBRATE_GYRO (runStartupGyroCalibration), session-only, never persisted.
 *   SET_CAL:H:GYRO_BIAS is still accepted but rejected as a stub for compatibility.
 */

#include <M5Unified.h>
#include <Preferences.h>
#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <esp_task_wdt.h>  // reset during the blocking gyro-calibration loop
#include <string.h>        // memcpy
#include "calibration.h"   // generated by calibrate_eeprom.py — NVS fallback defaults only

#include "MahonyAHRS.h"
#include "NavEKF.h"        // position/velocity — owned by firmware, not MahonyAHRS
#include "Diagnostics.h"   // all firmware/estimator diagnostics

#define NUM_SENSORS 5

// Nordic UART Service UUIDs
#define NUS_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX_CHAR_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_CHAR_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// CD74HC4067 MUX pins
const int MUX_SIG       = 36;
const int MUX_S[4]      = {26, 32, 33, 0};
const int MUX_SETTLE_US = 50;
const int MUX_AVG_READS = 2;   // oversampling per channel

// Loop timing
const unsigned long SENSOR_INTERVAL_MS  = 1000 / 120;   // 120 Hz target
const uint32_t      SENSOR_INTERVAL_US  = SENSOR_INTERVAL_MS * 1000UL;
const float         MAX_NAV_GAP_SEC     = 0.5f;   // discard pathological gaps; never compress time
const unsigned long DISPLAY_INTERVAL_MS = 200;    // 5 Hz

// Power management
const uint8_t  DISPLAY_BRIGHTNESS_ACTIVE = 100;
const uint8_t  DISPLAY_BRIGHTNESS_DIM    = 100;
const unsigned long DISPLAY_DIM_AFTER_MS = 15000;
const unsigned long BATTERY_POLL_MS      = 5000;
const int      LOW_BATTERY_PCT           = 15;

const uint8_t FRAME_SYNC0 = 0xA5;
const uint8_t FRAME_SYNC1 = 0x5A;

// CRC16-CCITT (poly 0x1021, init 0xFFFF) over the payload only.
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

const unsigned long USB_BAUD = 500000;  // keep in sync with __init__.py on the host side

// -------------------------------------------------------
// Runtime config
// -------------------------------------------------------
char hand                 = 'r';
bool streamGyroInImuSlots = false;   // telemetry encoding only — see USE_GYRO_ON/OFF doc above
bool send_quat            = false;   // QUAT_STREAM_ON/OFF
bool send_nav_quat        = false;   // NAV_QUAT_STREAM_ON/OFF
bool calibration_mode     = false;
bool write_binary         = true;
int  print_mode           = 0;       // 0=off, 1=raw, 2=calibrated
bool send_motion          = false;   // MOTION_STREAM_ON/OFF
bool gyroCalibToggle      = false;   // flips after each completed gyro calibration
// nav_diagnostics, sys_diagnostics, and diagnostic counters/printers live in Diagnostics.h

const size_t FLAGS_LEN           = 1;    // always-present stream-flags byte
const size_t SAMPLE_PAYLOAD_LEN  = 28;   // normal-mode payload bytes (excludes flags byte)
const size_t CAL_PAYLOAD_LEN     = 38;   // calibration-mode payload bytes (excludes flags byte)
const size_t QUAT_EXTRA_LEN      = 8;    // 4x int16 (qw,qx,qy,qz)
const size_t NAV_QUAT_EXTRA_LEN  = 8;    // 4x int16 (nqw,nqx,nqy,nqz)
const size_t MOTION_EXTRA_LEN    = 19;   // 6x int16 (vel+pos) + 3x int16 (lin accel) + 1x uint8 (zupt quality)
const size_t MAX_PAYLOAD_LEN     = FLAGS_LEN + CAL_PAYLOAD_LEN + QUAT_EXTRA_LEN + NAV_QUAT_EXTRA_LEN + MOTION_EXTRA_LEN;  // 74

const float MOTION_VEL_RANGE_MPS = 8.0f;
const float MOTION_POS_RANGE_M   = 2.0f;
const float LIN_ACCEL_RANGE_MPS2 = 4.0f * G_MS2;  // +-4g headroom for a gesture strike, encoding-only
const float ZUPT_MAHALANOBIS_CAP = 64.0f;         // 4x NavEKF's own zuptGateChiSq_ (16.0)

const size_t BUNDLE_FRAMES   = 1;
const size_t BUNDLE_BUF_SIZE = 3 + MAX_PAYLOAD_LEN * BUNDLE_FRAMES + 2;  // sync+len, payload, crc16

uint8_t bundleBuffer[MAX_PAYLOAD_LEN * BUNDLE_FRAMES];  // raw payloads only — header/CRC added at flush
uint8_t bundledFrameCount = 0;
size_t  bundleUnitLen = 0;

// -------------------------------------------------------
// Sensor data
// -------------------------------------------------------
uint16_t flexValues[NUM_SENSORS], fsrValues[NUM_SENSORS];
float ax, ay, az;          // accel (g,  M5Unified +-2g)
float gx, gy, gz;          // gyro  (deg/s, +-2000)
float roll, pitch, yaw;    // attitude (deg)
uint16_t ax_cal, ay_cal, az_cal;
uint16_t gx_cal, gy_cal, gz_cal;
uint16_t roll_cal, pitch_cal, yaw_cal;

uint32_t      nextSampleUs      = 0;   // deadline scheduler — absolute time of next sample
uint32_t      lastSampleUs      = 0;   // for dt computation
unsigned long lastDisplayMillis = 0;
bool  dispEverDrawn   = false;
bool  dispBleConn     = false;
char  dispHand        = 0;
int   dispBattery     = -1;
bool  dispCalibMode   = false;
bool  dispResetNotice = false;
uint8_t dispNavState  = 255;
int   dispPitch10     = 0;
int   dispRoll10      = 0;
int   dispYaw10       = 0;
int   dispDt10        = -1;
bool  navEkfNeeded    = false;
unsigned long resetNoticeUntilMs = 0;
unsigned long lastRawPrintMillis = 0;
const unsigned long RAW_PRINT_INTERVAL_MS = 200;   // 5 Hz — readable in serial monitor
uint8_t       frame_seq = 0;   // wrapping frame counter for loss detection
MahonyAHRS ahrs;   // gx,gy,gz in deg/s; ax,ay,az in g — matches M5.Imu.getImuData() units
NavEKF navEkf;     // position/velocity/navigation biases

// -------------------------------------------------------
// Calibration (RAM mirrors of NVS)
// -------------------------------------------------------
int L_MIN_FLEX_RAM[NUM_SENSORS],  L_MAX_FLEX_RAM[NUM_SENSORS];
int L_MIN_PRESS_RAM[NUM_SENSORS], L_MAX_PRESS_RAM[NUM_SENSORS];
int R_MIN_FLEX_RAM[NUM_SENSORS],  R_MAX_FLEX_RAM[NUM_SENSORS];
int R_MIN_PRESS_RAM[NUM_SENSORS], R_MAX_PRESS_RAM[NUM_SENSORS];
int *MIN_FLEX, *MAX_FLEX, *MIN_PRESS, *MAX_PRESS;
float ACTIVE_GYRO_BIAS[3] = {0.0f, 0.0f, 0.0f};
float L_ACCEL_BIAS_RAM[3] = {0, 0, 0};
float R_ACCEL_BIAS_RAM[3] = {0, 0, 0};
float *ACCEL_BIAS;

Preferences prefs;

// -------------------------------------------------------
// BLE
// -------------------------------------------------------
NimBLEServer*         bleServer = nullptr;
NimBLECharacteristic* txChar    = nullptr;
NimBLECharacteristic* rxChar    = nullptr;

#define BLE_CMD_QUEUE_LEN 4
#define BLE_CMD_MAX_LEN   64
struct BleCmdMsg { char text[BLE_CMD_MAX_LEN]; };
QueueHandle_t bleCommandQueue = nullptr;
bool          bleConnected    = false;

int           cachedBatteryPct   = 100;
bool          lowBatteryFlag     = false;
unsigned long lastBatteryPollMs  = 0;
unsigned long lastActivityMs     = 0;
bool          displayDimmed      = false;

void processCommand(String cmd);
void runStartupGyroCalibration();

class ConnCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    bleConnected = true;
    server->updateConnParams(connInfo.getConnHandle(), 8, 8, 0, 400);
  }
  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    bleConnected = false;
    NimBLEDevice::startAdvertising();
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  // BLE writes are queued and drained in loop(), never run inline in this
  // callback — keeps command handling off the NimBLE stack's task/stack.
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
    std::string v = c->getValue();
    BleCmdMsg msg;
    strncpy(msg.text, v.c_str(), BLE_CMD_MAX_LEN - 1);
    msg.text[BLE_CMD_MAX_LEN - 1] = '\0';
    xQueueSend(bleCommandQueue, &msg, 0);   // non-blocking; drops if the queue is full
  }
};

// -------------------------------------------------------
// Calibration — NVS
// -------------------------------------------------------
void saveCalibrationToPrefs() {
  prefs.begin("gluvn", false);
  for (int i = 0; i < NUM_SENSORS; i++) {
    prefs.putInt(("RNF"+String(i)).c_str(), R_MIN_FLEX_RAM[i]);
    prefs.putInt(("RXF"+String(i)).c_str(), R_MAX_FLEX_RAM[i]);
    prefs.putInt(("RNP"+String(i)).c_str(), R_MIN_PRESS_RAM[i]);
    prefs.putInt(("RXP"+String(i)).c_str(), R_MAX_PRESS_RAM[i]);
    prefs.putInt(("LNF"+String(i)).c_str(), L_MIN_FLEX_RAM[i]);
    prefs.putInt(("LXF"+String(i)).c_str(), L_MAX_FLEX_RAM[i]);
    prefs.putInt(("LNP"+String(i)).c_str(), L_MIN_PRESS_RAM[i]);
    prefs.putInt(("LXP"+String(i)).c_str(), L_MAX_PRESS_RAM[i]);
  }
  for (int i = 0; i < 3; i++) {
    // Gyro bias deliberately NOT persisted — ACTIVE_GYRO_BIAS is runtime-only by design.
    prefs.putFloat(("RAB"+String(i)).c_str(), R_ACCEL_BIAS_RAM[i]);
    prefs.putFloat(("LAB"+String(i)).c_str(), L_ACCEL_BIAS_RAM[i]);
  }
  prefs.end();
  Serial.println("PREFS_SAVED");
}

void loadCalibrationFromPrefs() {
  prefs.begin("gluvn", true);
  for (int i = 0; i < NUM_SENSORS; i++) {
    R_MIN_FLEX_RAM[i]  = prefs.getInt(("RNF"+String(i)).c_str(), R_MIN_FLEX[i]);
    R_MAX_FLEX_RAM[i]  = prefs.getInt(("RXF"+String(i)).c_str(), R_MAX_FLEX[i]);
    R_MIN_PRESS_RAM[i] = prefs.getInt(("RNP"+String(i)).c_str(), R_MIN_PRESS[i]);
    R_MAX_PRESS_RAM[i] = prefs.getInt(("RXP"+String(i)).c_str(), R_MAX_PRESS[i]);
    L_MIN_FLEX_RAM[i]  = prefs.getInt(("LNF"+String(i)).c_str(), L_MIN_FLEX[i]);
    L_MAX_FLEX_RAM[i]  = prefs.getInt(("LXF"+String(i)).c_str(), L_MAX_FLEX[i]);
    L_MIN_PRESS_RAM[i] = prefs.getInt(("LNP"+String(i)).c_str(), L_MIN_PRESS[i]);
    L_MAX_PRESS_RAM[i] = prefs.getInt(("LXP"+String(i)).c_str(), L_MAX_PRESS[i]);
  }
  for (int i = 0; i < 3; i++) {
    R_ACCEL_BIAS_RAM[i] = prefs.getFloat(("RAB"+String(i)).c_str(), 0.0f);
    L_ACCEL_BIAS_RAM[i] = prefs.getFloat(("LAB"+String(i)).c_str(), 0.0f);
  }
  prefs.end();
}

void setHandPointers() {
  if (hand == 'r') {
    MIN_FLEX  = R_MIN_FLEX_RAM;  MAX_FLEX  = R_MAX_FLEX_RAM;
    MIN_PRESS = R_MIN_PRESS_RAM; MAX_PRESS = R_MAX_PRESS_RAM;
    ACCEL_BIAS = R_ACCEL_BIAS_RAM;
  } else {
    MIN_FLEX  = L_MIN_FLEX_RAM;  MAX_FLEX  = L_MAX_FLEX_RAM;
    MIN_PRESS = L_MIN_PRESS_RAM; MAX_PRESS = L_MAX_PRESS_RAM;
    ACCEL_BIAS = L_ACCEL_BIAS_RAM;
  }
  M5.Display.setRotation(hand == 'r' ? 1 : 3);
}

// -------------------------------------------------------
// MUX read (with settle + oversampling)
// -------------------------------------------------------
int readMux(int channel) {
  for (int i = 0; i < 4; i++) digitalWrite(MUX_S[i], (channel >> i) & 0x01);
  delayMicroseconds(MUX_SETTLE_US);
  analogRead(MUX_SIG);  // discard: flush ADC sample-hold from previous channel
  int sum = 0;
  for (int i = 0; i < MUX_AVG_READS; i++) sum += analogRead(MUX_SIG);
  return sum / MUX_AVG_READS;
}

// -------------------------------------------------------
// Frame transmission — BLE notify + USB serial
// -------------------------------------------------------
unsigned long lastBundleFlush = 0;
const unsigned long BUNDLE_MAX_HOLD_MS = 15;  // don't hold a lone frame past this

void sendData(uint8_t* buf, size_t len) {
  if (write_binary) Serial.write(buf, len);
  if (bleConnected && txChar) {
    txChar->setValue(buf, len);
    txChar->notify();
  }
}

size_t buildPayload(uint8_t* out) {
  size_t idx = 0;

  out[idx++] = (uint8_t)hand;
  out[idx++] = (uint8_t)(
      (calibration_mode ? 0x01 : 0x00) |
      (send_quat        ? 0x02 : 0x00) |
      (send_nav_quat    ? 0x04 : 0x00) |
      (send_motion      ? 0x08 : 0x00) |
      // Bits 4-6: NavEKF health, meaningful only when bit3 (send_motion) is set.
      // Read unconditionally (cheap accessors) even though the MEANING is conditional.
      (send_motion && navEkf.isArmed()       ? 0x10 : 0x00) |
      (send_motion && navEkf.isZuptActive()  ? 0x20 : 0x00) |
      (send_motion && navEkf.isZuptStarved() ? 0x40 : 0x00) |
      (lowBatteryFlag                        ? 0x80 : 0x00));
  out[idx++] = frame_seq++;

  // Device-side timestamp (micros() at this sample's deadline), big-endian —
  // lets the host separate firmware/scheduler jitter from reception jitter.
  out[idx++] = (uint8_t)(lastSampleUs >> 24);
  out[idx++] = (uint8_t)(lastSampleUs >> 16);
  out[idx++] = (uint8_t)(lastSampleUs >> 8);
  out[idx++] = (uint8_t)(lastSampleUs);

  out[idx++] = yaw_cal >> 8;   out[idx++] = yaw_cal & 0xFF;
  out[idx++] = pitch_cal >> 8; out[idx++] = pitch_cal & 0xFF;
  out[idx++] = roll_cal >> 8;  out[idx++] = roll_cal & 0xFF;

  uint16_t i0 = streamGyroInImuSlots ? gx_cal : ax_cal;
  uint16_t i1 = streamGyroInImuSlots ? gy_cal : ay_cal;
  uint16_t i2 = streamGyroInImuSlots ? gz_cal : az_cal;
  i0 = (i0 & 0xFFFE) | (streamGyroInImuSlots ? 0x0001 : 0x0000);   // sensor_type in bit0
  i1 = (i1 & 0xFFFE) | (gyroCalibToggle ? 0x0001 : 0x0000);        // calib-complete toggle in bit0

  out[idx++] = i0 >> 8; out[idx++] = i0 & 0xFF;
  out[idx++] = i1 >> 8; out[idx++] = i1 & 0xFF;
  out[idx++] = i2 >> 8; out[idx++] = i2 & 0xFF;

  if (!calibration_mode) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (MIN_FLEX[i] == MAX_FLEX[i]) { out[idx++] = 0; continue; }
      out[idx++] = (uint8_t)constrain(
        map(flexValues[i], MIN_FLEX[i], MAX_FLEX[i], 0, 255), 0, 255);
    }
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (MIN_PRESS[i] == MAX_PRESS[i]) { out[idx++] = 0; continue; }
      out[idx++] = (uint8_t)constrain(
        map(fsrValues[i], MIN_PRESS[i], MAX_PRESS[i], 0, 255), 0, 255);
    }
  } else {
    for (int i = 0; i < NUM_SENSORS; i++) {
      out[idx++] = flexValues[i] >> 8; out[idx++] = flexValues[i] & 0xFF;
    }
    for (int i = 0; i < NUM_SENSORS; i++) {
      out[idx++] = fsrValues[i] >> 8;  out[idx++] = fsrValues[i] & 0xFF;
    }
  }

  if (send_quat) {
    float qw, qx, qy, qz;
    ahrs.getQuaternion(qw, qx, qy, qz);
    int16_t q[4] = {
      (int16_t)constrain(qw * 32767.0f, -32768.0f, 32767.0f),
      (int16_t)constrain(qx * 32767.0f, -32768.0f, 32767.0f),
      (int16_t)constrain(qy * 32767.0f, -32768.0f, 32767.0f),
      (int16_t)constrain(qz * 32767.0f, -32768.0f, 32767.0f),
    };
    for (int i = 0; i < 4; i++) {
      out[idx++] = (uint8_t)((uint16_t)q[i] >> 8);
      out[idx++] = (uint8_t)((uint16_t)q[i] & 0xFF);
    }
  }

  if (send_nav_quat) {
    float q0, q1, q2, q3;
    navEkf.getQuaternion(q0, q1, q2, q3);
    int16_t q[4] = {
      (int16_t)constrain(q0 * 32767.0f, -32768.0f, 32767.0f),
      (int16_t)constrain(q1 * 32767.0f, -32768.0f, 32767.0f),
      (int16_t)constrain(q2 * 32767.0f, -32768.0f, 32767.0f),
      (int16_t)constrain(q3 * 32767.0f, -32768.0f, 32767.0f),
    };
    for (int i = 0; i < 4; i++) {
      out[idx++] = (uint8_t)((uint16_t)q[i] >> 8);
      out[idx++] = (uint8_t)((uint16_t)q[i] & 0xFF);
    }
  }

  if (send_motion) {
    float velX, velY, velZ, posX, posY, posZ;
    navEkf.getVelocity(velX, velY, velZ);
    navEkf.getPosition(posX, posY, posZ);

    int16_t m[6] = {
      (int16_t)constrain(velX * 32767.0f / MOTION_VEL_RANGE_MPS, -32768.0f, 32767.0f),
      (int16_t)constrain(velY * 32767.0f / MOTION_VEL_RANGE_MPS, -32768.0f, 32767.0f),
      (int16_t)constrain(velZ * 32767.0f / MOTION_VEL_RANGE_MPS, -32768.0f, 32767.0f),
      (int16_t)constrain(posX * 32767.0f / MOTION_POS_RANGE_M,   -32768.0f, 32767.0f),
      (int16_t)constrain(posY * 32767.0f / MOTION_POS_RANGE_M,   -32768.0f, 32767.0f),
      (int16_t)constrain(posZ * 32767.0f / MOTION_POS_RANGE_M,   -32768.0f, 32767.0f),
    };
    for (int i = 0; i < 6; i++) {
      out[idx++] = (uint8_t)((uint16_t)m[i] >> 8);
      out[idx++] = (uint8_t)((uint16_t)m[i] & 0xFF);
    }

    // Linear accel travels with motion, not as its own opt-in — flags (not
    // length) disambiguate every combination regardless. World-frame,
    // gravity already subtracted by NavEKF; zero at rest.
    float laX, laY, laZ;
    navEkf.getWorldAcceleration(laX, laY, laZ);
    int16_t la[3] = {
      (int16_t)constrain(laX * 32767.0f / LIN_ACCEL_RANGE_MPS2, -32768.0f, 32767.0f),
      (int16_t)constrain(laY * 32767.0f / LIN_ACCEL_RANGE_MPS2, -32768.0f, 32767.0f),
      (int16_t)constrain(laZ * 32767.0f / LIN_ACCEL_RANGE_MPS2, -32768.0f, 32767.0f),
    };
    for (int i = 0; i < 3; i++) {
      out[idx++] = (uint8_t)((uint16_t)la[i] >> 8);
      out[idx++] = (uint8_t)((uint16_t)la[i] & 0xFF);
    }

    // ZUPT-quality byte: chi-square goodness-of-fit for the last ZUPT
    // candidate, near 0 = confident stationary fit. -1.0f ("never computed
    // yet") encodes as 255, kept out of [0,254] so it stays unambiguous.
    float zuptD2 = navEkf.getZuptMahalanobis();
    uint8_t zuptQuality;
    if (zuptD2 < 0.0f) {
      zuptQuality = 255;
    } else {
      float clamped = constrain(zuptD2, 0.0f, ZUPT_MAHALANOBIS_CAP);
      zuptQuality = (uint8_t)constrain(
          roundf(clamped / ZUPT_MAHALANOBIS_CAP * 254.0f), 0.0f, 254.0f);
    }
    out[idx++] = zuptQuality;
  }

  return idx;   // base(+1 flags), +QUAT_EXTRA_LEN, +NAV_QUAT_EXTRA_LEN, +MOTION_EXTRA_LEN as applicable
}

void flushBundle() {
  if (bundledFrameCount == 0) return;

  size_t payloadLen = (size_t)bundledFrameCount * bundleUnitLen;

  uint8_t buf[BUNDLE_BUF_SIZE];
  buf[0] = FRAME_SYNC0;
  buf[1] = FRAME_SYNC1;
  buf[2] = (uint8_t)payloadLen;
  memcpy(&buf[3], bundleBuffer, payloadLen);

  uint16_t crc = crc16_ccitt(&buf[3], payloadLen);
  buf[3 + payloadLen]     = (uint8_t)(crc >> 8);
  buf[3 + payloadLen + 1] = (uint8_t)(crc & 0xFF);

  sendData(buf, 3 + payloadLen + 2);

  bundledFrameCount = 0;
  lastBundleFlush = millis();
}

void buildAndSendFrame() {
  bundleUnitLen = buildPayload(bundleBuffer + (size_t)bundledFrameCount * bundleUnitLen);
  bundledFrameCount++;

  if (bundledFrameCount >= BUNDLE_FRAMES ||
      (millis() - lastBundleFlush) >= BUNDLE_MAX_HOLD_MS) {
    flushBundle();
  }
}

// -------------------------------------------------------
// Command processing (USB serial + BLE RX)
// -------------------------------------------------------
// Fixed buffer instead of a String grown via += per char (avoids a
// heap-fragmentation source over long sessions). processCommand() still
// takes a String, built once per completed line, not incrementally.
char   serialCmdBuf[BLE_CMD_MAX_LEN];
size_t serialCmdLen = 0;
bool   commandComplete = false;

void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "HAND_R") {
    hand = 'r'; setHandPointers(); Serial.println("HAND_SET_R");
  } else if (cmd == "HAND_L") {
    hand = 'l'; setHandPointers(); Serial.println("HAND_SET_L");
  } else if (cmd == "CAL_START") {
    bundledFrameCount = 0;   // flush partial bundle before size changes
    calibration_mode = true; print_mode = 0; write_binary = true;
    Serial.println("CAL_ON");
  } else if (cmd == "CAL_STOP") {
      bundledFrameCount = 0;
      calibration_mode = false; Serial.println("CAL_OFF");
  } else if (cmd == "RAW_MODE") {
    print_mode = 1; write_binary = false; Serial.println("RAW_MODE_ON");
  } else if (cmd == "CAL_MODE") {
    print_mode = 2; write_binary = false; Serial.println("CAL_MODE_ON");
  } else if (cmd == "NORMAL_MODE") {
    print_mode = 0; write_binary = true; calibration_mode = false;
    Serial.println("NORMAL_MODE_ON");
  } else if (cmd == "USE_GYRO_ON") {
    streamGyroInImuSlots = true; Serial.println("USE_GYRO_ON_OK");
  } else if (cmd == "USE_GYRO_OFF") {
    streamGyroInImuSlots = false; Serial.println("USE_GYRO_OFF_OK");
  } else if (cmd == "QUAT_STREAM_ON") {
    send_quat = true; Serial.println("QUAT_STREAM_ON_OK");
  } else if (cmd == "QUAT_STREAM_OFF") {
     send_quat = false; Serial.println("QUAT_STREAM_OFF_OK");
  } else if (cmd == "NAV_QUAT_STREAM_ON") {
    send_nav_quat = true; Serial.println("NAV_QUAT_STREAM_ON_OK");
  } else if (cmd == "NAV_QUAT_STREAM_OFF") {
    send_nav_quat = false; Serial.println("NAV_QUAT_STREAM_OFF_OK");
  } else if (cmd == "TANGENT_GRAVITY_ON") {
     navEkf.setUseTangentPlaneGravityUpdate(true);  Serial.println("TANGENT_GRAVITY_ON_OK");
  } else if (cmd == "TANGENT_GRAVITY_OFF") {
     navEkf.setUseTangentPlaneGravityUpdate(false); Serial.println("TANGENT_GRAVITY_OFF_OK");
  } else if (cmd == "MOTION_STREAM_ON") {
    send_motion = true; Serial.println("MOTION_STREAM_ON_OK");
  } else if (cmd == "MOTION_STREAM_OFF") {
    send_motion = false; Serial.println("MOTION_STREAM_OFF_OK");
  } else if (cmd == "NAV_DIAG_ON") {
    nav_diagnostics = true; Serial.println("NAV_DIAG_ON_OK");
  } else if (cmd == "NAV_DIAG_OFF") {
    nav_diagnostics = false; Serial.println("NAV_DIAG_OFF_OK");
  } else if (cmd == "SYS_DIAG_ON") {
    sys_diagnostics = true; Serial.println("SYS_DIAG_ON_OK");
  } else if (cmd == "SYS_DIAG_OFF") {
    sys_diagnostics = false; Serial.println("SYS_DIAG_OFF_OK");
  } else if (cmd == "STATUS") {
    if (!diagPrintSysLineOnDemand(ahrs, navEkf, write_binary)) {
      Serial.println("STATUS_ERR_BINARY_MODE_ACTIVE");
    }
  } else if (cmd == "DIAG_RESET_MAX") {
    diagResetMax(navEkf);
    Serial.println("DIAG_RESET_MAX_OK");
  } else if (cmd == "RESET_YAW") {
    ahrs.resetYaw();   // display/telemetry yaw only — does not touch navEkf's world frame
    Serial.println("YAW_RESET");
  } else if (cmd == "RESET_POSITION") {
    navEkf.reset(); Serial.println("POSITION_RESET");
  } else if (cmd == "RECALIBRATE_GYRO") {
    runStartupGyroCalibration();   // blocks; prints GYRO_CAL_START/DONE itself
  } else if (cmd == "PREFS_SAVE") {
    saveCalibrationToPrefs();
  } else if (cmd.startsWith("SET_CAL:")) {
      int c1 = cmd.indexOf(':', 8);
      int c2 = cmd.indexOf(':', c1 + 1);
      if (c1 < 0 || c2 < 0) { Serial.println("SET_CAL_FMT_ERR"); return; }

      bool   isR  = cmd.substring(8, c1) == "R";
      String type = cmd.substring(c1 + 1, c2);
      String vals = cmd.substring(c2 + 1);

      if (type == "GYRO_BIAS") {
        // Removed: gyro bias is solved exclusively by runStartupGyroCalibration()
        // (RECALIBRATE_GYRO) and is deliberately session-only/non-persisted.
        // A host-writable path here created two uncoordinated sources of truth
        // for ACTIVE_GYRO_BIAS. Use RECALIBRATE_GYRO instead.
        Serial.println("SET_CAL_GYRO_BIAS_REMOVED_USE_RECALIBRATE_GYRO");
        return;
      }

      if (type == "ACCEL_BIAS") {
        float ab[3]; int vi = 0, start = 0;
        for (int i = 0; i <= (int)vals.length() && vi < 3; i++) {
          if (i == (int)vals.length() || vals[i] == ',') {
            ab[vi++] = vals.substring(start, i).toFloat();
            start = i + 1;
          }
        }
        if (vi < 3) { Serial.println("SET_CAL_PARSE_ERR"); return; }

        const float ACCEL_BIAS_SANITY_LIMIT_G = 0.5f;   // generous single-position offset bound
        for (int i = 0; i < 3; i++) {
          if (fabsf(ab[i]) > ACCEL_BIAS_SANITY_LIMIT_G) {
            Serial.println("SET_CAL_ACCEL_BIAS_RANGE_ERR");
            return;
          }
        }

        float* arr = isR ? R_ACCEL_BIAS_RAM : L_ACCEL_BIAS_RAM;
        for (int i = 0; i < 3; i++) arr[i] = ab[i];
        Serial.printf("SET_%c_ACCEL_BIAS_OK\n", isR ? 'R' : 'L');
        return;
      }

      int v[NUM_SENSORS], vi = 0, start = 0;
      for (int i = 0; i <= (int)vals.length() && vi < NUM_SENSORS; i++) {
        if (i == (int)vals.length() || vals[i] == ',') {
          v[vi++] = vals.substring(start, i).toInt();
          start = i + 1;
        }
      }
      if (vi < NUM_SENSORS) { Serial.println("SET_CAL_PARSE_ERR"); return; }

      int* arr = nullptr;
      if      (type == "MIN_FLEX")  arr = isR ? R_MIN_FLEX_RAM  : L_MIN_FLEX_RAM;
      else if (type == "MAX_FLEX")  arr = isR ? R_MAX_FLEX_RAM  : L_MAX_FLEX_RAM;
      else if (type == "MIN_PRESS") arr = isR ? R_MIN_PRESS_RAM : L_MIN_PRESS_RAM;
      else if (type == "MAX_PRESS") arr = isR ? R_MAX_PRESS_RAM : L_MAX_PRESS_RAM;
      else { Serial.println("SET_CAL_TYPE_ERR"); return; }

      for (int i = 0; i < NUM_SENSORS; i++) arr[i] = v[i];
      Serial.printf("SET_%c_%s_OK\n", isR ? 'R' : 'L', type.c_str());
  }
}

M5Canvas canvas(&M5.Display);

void updateBatteryStatus() {
  unsigned long now = millis();
  if (now - lastBatteryPollMs < BATTERY_POLL_MS && lastBatteryPollMs != 0) return;
  lastBatteryPollMs = now;
  cachedBatteryPct = M5.Power.getBatteryLevel();
  lowBatteryFlag = cachedBatteryPct >= 0 && cachedBatteryPct <= LOW_BATTERY_PCT;
}

void updateDisplayDimming() {
  unsigned long now = millis();
  if (bleConnected) {
    lastActivityMs = now;
    if (displayDimmed) {
      M5.Display.setBrightness(DISPLAY_BRIGHTNESS_ACTIVE);
      displayDimmed = false;
    }
    return;
  }
  if (!displayDimmed && (now - lastActivityMs >= DISPLAY_DIM_AFTER_MS)) {
    M5.Display.setBrightness(DISPLAY_BRIGHTNESS_DIM);
    displayDimmed = true;
  }
}

void resetPoseFromButton() {
  ahrs.resetYaw();
  navEkf.reset();
  resetNoticeUntilMs = millis() + 1000;
}

// -------------------------------------------------------
// Display update (10 Hz)
// -------------------------------------------------------
void updateDisplay() {
  int battery = cachedBatteryPct;
  int pitch10 = (int)lroundf(pitch * 10.0f);
  int roll10  = (int)lroundf(roll  * 10.0f);
  int yaw10   = (int)lroundf(yaw   * 10.0f);
  bool resetNotice = (int32_t)(resetNoticeUntilMs - millis()) > 0;
  uint8_t navState = !navEkfNeeded ? 0 : (navEkf.isArmed() ? 2 : 1);

  bool changed = !dispEverDrawn ||
      bleConnected      != dispBleConn ||
      hand              != dispHand ||
      battery           != dispBattery ||
      calibration_mode  != dispCalibMode ||
      resetNotice       != dispResetNotice ||
      navState          != dispNavState ||
      pitch10 != dispPitch10 || roll10 != dispRoll10 || yaw10 != dispYaw10;

  if (!changed) return;   // nothing visible would differ — skip the redraw+push cost

  dispEverDrawn = true;
  dispBleConn   = bleConnected;
  dispHand      = hand;
  dispBattery   = battery;
  dispCalibMode = calibration_mode;
  dispResetNotice = resetNotice;
  dispNavState = navState;
  dispPitch10 = pitch10;
  dispRoll10  = roll10;
  dispYaw10   = yaw10;

  canvas.fillSprite(BLACK);

  // Glove identity — always visible regardless of connection state
  canvas.setTextColor(WHITE, BLACK);
  canvas.setTextSize(2);
  canvas.setCursor(6, 4);
  canvas.print(hand == 'r' ? "Gluvn Right" : "Gluvn Left");

  // Status bar — connection state
  if (bleConnected) {
    canvas.fillRect(0, 26, 240, 34, GREEN);
    canvas.setTextColor(BLACK, GREEN);
    canvas.setTextSize(3);
    canvas.setCursor(6, 30);
    canvas.print("CONNECTED");
  } else {
    canvas.fillRect(0, 26, 240, 34, RED);
    canvas.setTextColor(WHITE, RED);
    canvas.setTextSize(3);
    canvas.setCursor(6, 30);
    canvas.print("ADVERTISING");
  }

  // Battery — always visible
  canvas.setTextColor(battery >= 0 && battery <= LOW_BATTERY_PCT ? RED : WHITE, BLACK);
  canvas.setTextSize(3);
  canvas.setCursor(30, 66);
  if (battery >= 0) {
    canvas.printf("BAT %3d%%", battery);
  } else {
    canvas.print("BAT ---");
  }

  // Reset confirmation — transient, own row, doesn't displace battery
  if (resetNotice) {
    canvas.setTextColor(CYAN, BLACK);
    canvas.setTextSize(2);
    canvas.setCursor(30, 96);
    canvas.print("POSE RESET");
  }

  // Button legend — always visible, bottom of screen
  canvas.setTextColor(WHITE, BLACK);
  canvas.setTextSize(1);
  canvas.setCursor(6, 116);
  canvas.print("SHORT PRESS: reset pose");
  canvas.setCursor(6, 126);
  canvas.print("LONG PRESS (1s): recal gyro");

  canvas.pushSprite(0, 0);
}

uint32_t bumpAndGetBootCount() {
  prefs.begin("gluvn", false);
  uint32_t n = prefs.getUInt("bootCount", 0) + 1;
  prefs.putUInt("bootCount", n);
  prefs.end();
  return n;
}

// -------------------------------------------------------
// Startup gyro calibration
// -------------------------------------------------------
const uint16_t STARTUP_GYRO_CAL_SAMPLES = 2000;   // ~3-4s @ ~500-700 Hz achievable poll rate
const unsigned long GYRO_CAL_TIMEOUT_MS = 10000;

void runStartupGyroCalibration() {
  diagCalibTimingBegin();

  Serial.println("GYRO_CAL_START");

  M5.Display.fillScreen(BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(WHITE, BLACK);
  M5.Display.setCursor(10, 20);
  M5.Display.println("Gyro Calib");
  M5.Display.setTextSize(1);
  M5.Display.setCursor(10, 50);
  M5.Display.println("Keep glove still...");

  ahrs.beginGyroCalibration(STARTUP_GYRO_CAL_SAMPLES);

  unsigned long calibStartMs = millis();
  unsigned long lastProgressMs = calibStartMs;
  bool done = false;
  bool timedOut = false;

  while (!done) {
    // Busy-waits via delayMicroseconds() rather than yielding to FreeRTOS,
    // so nothing here naturally feeds the task watchdog — reset it explicitly.
    esp_task_wdt_reset();

    if (millis() - calibStartMs >= GYRO_CAL_TIMEOUT_MS) {
      timedOut = true;
      break;
    }

    if (!M5.Imu.update()) {
      delayMicroseconds(500);
      continue;
    }
    auto imu = M5.Imu.getImuData();

    // Feed RAW gyro — no bias subtraction here, bias is what we're solving for.
    done = ahrs.updateGyroCalibration(imu.gyro.x, imu.gyro.y, imu.gyro.z);

    if (millis() - lastProgressMs >= 100) {
      lastProgressMs = millis();
      M5.Display.fillRect(10, 70, 220, 16, BLACK);
      M5.Display.setCursor(10, 70);
      M5.Display.printf("Sample %u/%u", ahrs.getCalibCount(), STARTUP_GYRO_CAL_SAMPLES);
    }

    delayMicroseconds(500);   // pace sampling without starving I2C/BLE stack
  }

  if (timedOut) {
    Serial.printf("GYRO_CAL_TIMEOUT %u\n", ahrs.getCalibCount());
  }

  ahrs.endGyroCalibration();

  float bx, by, bz;
  ahrs.getGyroBias(bx, by, bz);
  ACTIVE_GYRO_BIAS[0] = bx;
  ACTIVE_GYRO_BIAS[1] = by;
  ACTIVE_GYRO_BIAS[2] = bz;
  ahrs.setGyroBias(0.0f, 0.0f, 0.0f);
  ahrs.reinitializeAfterBiasUpdate();   // clears stale integral/motion/filter state under the old bias
  ahrs.resetYaw();
  navEkf.reset();   // re-arms from scratch; requires a fresh confirmed-stationary window

  Serial.printf("GYRO_CAL_DONE %.4f %.4f %.4f\n", bx, by, bz);
  gyroCalibToggle = !gyroCalibToggle;

  M5.Display.fillRect(10, 70, 220, 16, BLACK);
  M5.Display.setCursor(10, 70);
  M5.Display.println("Calib done.");
  delay(400);

  diagCalibTimingEnd();
}

// -------------------------------------------------------
// Setup
// -------------------------------------------------------
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setBrightness(DISPLAY_BRIGHTNESS_ACTIVE);
  lastActivityMs = millis();

  loadCalibrationFromPrefs();
  setHandPointers();
  M5.Display.fillScreen(BLACK);

  M5.Display.setTextSize(2);
  M5.Display.setTextColor(WHITE, BLACK);
  M5.Display.setCursor(15, 5);
  M5.Display.printf("Gluvn %c", toupper(hand));

  M5.Display.setTextSize(1);
  M5.Display.setCursor(15, 30);
  M5.Display.printf("FW: %s", __DATE__);
  M5.Display.setCursor(15, 42);
  M5.Display.printf("Batt: %d%%", M5.Power.getBatteryLevel());

  Serial.begin(USB_BAUD);
  uint32_t bootCount = bumpAndGetBootCount();
  diagSetBootCount(bootCount);
  Serial.printf("BOOT #%u\n", bootCount);
  M5.Imu.begin();
  // Mahony's detector adapts attitude bias; NavEKF owns the authoritative
  // navigation stationary flag and ZUPT timing.
  ahrs.setVarianceWindowTimeConstant(0.05f);
  // Startup gyro calibration removed: the host's bring_up() sequence
  // (Reader.start_readers -> ensure_fresh_gyro_calibration) already forces
  // a fresh RECALIBRATE_GYRO before any session streams data, and the
  // long-press path (see button legend) covers manual re-cal otherwise.
  // Running it here too only delayed BLE advertising by ~3-4s for a
  // calibration nothing was yet connected to consume.
  // NOTE: bypassing the host's bring_up() (e.g. a bare serial/BLE client
  // that skips ensure_fresh_gyro_calibration) will leave ACTIVE_GYRO_BIAS
  // at its last value (zero on a cold boot) until RECALIBRATE_GYRO runs.

  M5.Display.fillScreen(BLACK);

  canvas.setColorDepth(16);
  canvas.createSprite(240, 135);
  canvas.setTextSize(2);
  canvas.setTextWrap(false);

  for (int i = 0; i < 4; i++) pinMode(MUX_S[i], OUTPUT);
  pinMode(MUX_SIG, INPUT);
  analogSetPinAttenuation(MUX_SIG, ADC_11db);
  analogReadResolution(12);

  loadCalibrationFromPrefs();
  setHandPointers();

  bleCommandQueue = xQueueCreate(BLE_CMD_QUEUE_LEN, sizeof(BleCmdMsg));
  NimBLEDevice::init(hand == 'r' ? "Gluvn_R" : "Gluvn_L");
  NimBLEDevice::setMTU(128);
  NimBLEDevice::setPower(ESP_PWR_LVL_N0);

  bleServer = NimBLEDevice::createServer();
  bleServer->setCallbacks(new ConnCallbacks());

  NimBLEService* nus = bleServer->createService(NUS_SERVICE_UUID);

  txChar = nus->createCharacteristic(NUS_TX_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
  rxChar = nus->createCharacteristic(
    NUS_RX_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  rxChar->setCallbacks(new RxCallbacks());

  nus->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();

  NimBLEAdvertisementData advData;
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  advData.setCompleteServices(NimBLEUUID(NUS_SERVICE_UUID));
  adv->setAdvertisementData(advData);

  NimBLEAdvertisementData scanRespData;
  scanRespData.setName(hand == 'r' ? "Gluvn_R" : "Gluvn_L");
  adv->setScanResponseData(scanRespData);

  NimBLEDevice::startAdvertising();
  Serial.println("Gluvn ready");

  nextSampleUs = micros();
  lastSampleUs = nextSampleUs;
}

// -------------------------------------------------------
// Loop
// -------------------------------------------------------
void loop() {
  diagLoopTimingBegin();

  M5.update();

  static bool btnHoldHandled = false;
  static bool btnPressSeen = false;
  if (M5.BtnA.isPressed()) {
    btnPressSeen = true;
    if (M5.BtnA.pressedFor(1000) && !btnHoldHandled) {
      btnHoldHandled = true;
      runStartupGyroCalibration();
    }
  } else if (btnPressSeen) {
    if (!btnHoldHandled) {
      resetPoseFromButton();
    }
    btnPressSeen = false;
    btnHoldHandled = false;
  }

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      commandComplete = true;
    } else if (serialCmdLen < sizeof(serialCmdBuf) - 1) {
      serialCmdBuf[serialCmdLen++] = c;
    }
    // else: overlong line — drop the char rather than grow unbounded
  }
  if (commandComplete) {
    serialCmdBuf[serialCmdLen] = '\0';
    processCommand(String(serialCmdBuf));
    serialCmdLen = 0;
    commandComplete = false;
  }

  BleCmdMsg bleMsg;
  while (xQueueReceive(bleCommandQueue, &bleMsg, 0) == pdTRUE) {
    processCommand(String(bleMsg.text));
  }

  // NavEKF is needed by any consumer of nav output: binary motion, NavEKF
  // quaternion telemetry, or either diagnostic stream. Reset once on a
  // rising edge so the 0.3s arm window starts fresh, not from stale state.
  static bool navEkfWasNeeded = false;
  // Diagnostics (nav_diagnostics/sys_diagnostics) intentionally excluded:
  // they're an offline debugging aid, not a normal-operation data path.
  // Estimators now wake only for actual consumers (motion/nav_quat stream).
  navEkfNeeded = send_motion || send_nav_quat;
  if (navEkfNeeded && !navEkfWasNeeded) {
    navEkf.reset();
  }
  navEkfWasNeeded = navEkfNeeded;

  unsigned long now  = millis();   // 10 Hz display timer
  uint32_t      nowUs = micros();  // 100 Hz sample deadline scheduler

  if ((int32_t)(nowUs - nextSampleUs) >= 0) {
    if ((int32_t)(nowUs - nextSampleUs) > (int32_t)(3 * SENSOR_INTERVAL_US)) {
      nextSampleUs = nowUs;
    }
    nextSampleUs += SENSOR_INTERVAL_US;

    // A missed IMU sample no longer skips the whole loop — MUX reads, frame
    // send, and display still run this tick on the last good sample; only
    // the block that would consume a fresh sample is skipped.
    if (M5.Imu.update()) {
      diagNoteImuFresh();

      auto imu = M5.Imu.getImuData();
      uint32_t sampleUs = micros();
      float dt = (float)(sampleUs - lastSampleUs) / 1e6f;
      lastSampleUs = sampleUs;
      bool usableDt = dt > 0.0f && dt <= MAX_NAV_GAP_SEC;
      if (!usableDt) diagNoteDt(dt, false);

      bool gyroFinite  = isfinite(imu.gyro.x)  && isfinite(imu.gyro.y)  && isfinite(imu.gyro.z);
      bool accelFinite = isfinite(imu.accel.x) && isfinite(imu.accel.y) && isfinite(imu.accel.z);
      diagNoteRawGyro(imu.gyro.x, imu.gyro.y, imu.gyro.z);
      diagNoteRawAccel(imu.accel.x, imu.accel.y, imu.accel.z);

      // A non-finite sample is dropped here rather than fed downstream —
      // neither Mahony nor NavEKF recovers from a NaN state on its own.
      if (gyroFinite && accelFinite) {
        ax = imu.accel.x - ACCEL_BIAS[0];
        ay = imu.accel.y - ACCEL_BIAS[1];
        az = imu.accel.z - ACCEL_BIAS[2];
        gx = imu.gyro.x  - ACTIVE_GYRO_BIAS[0];
        gy = imu.gyro.y  - ACTIVE_GYRO_BIAS[1];
        gz = imu.gyro.z  - ACTIVE_GYRO_BIAS[2];
      }

      if (usableDt && gyroFinite && accelFinite) {
        diagNoteDt(dt, true);

        // Mahony now gated the same as NavEKF: its output (yaw/pitch/roll,
        // quaternion, gyro bias feed into NavEKF) has no consumer unless
        // something needs nav output. Previously ran unconditionally at
        // 100 Hz regardless of BLE-connected/streaming state.
        if (navEkfNeeded) {
          ahrs.updateIMU(gx, gy, gz, ax, ay, az, dt);
          ahrs.getEuler(yaw, pitch, roll);

          float navQ0, navQ1, navQ2, navQ3;
          ahrs.getQuaternion(navQ0, navQ1, navQ2, navQ3);
          diagNoteMahonyQuat(navQ0, navQ1, navQ2, navQ3);

          float navBgx, navBgy, navBgz;
          ahrs.getGyroBias(navBgx, navBgy, navBgz);

          diagEkfTimingBegin();
          navEkf.update(gx, gy, gz, ax, ay, az, dt,
                        navQ0, navQ1, navQ2, navQ3,
                        navBgx, navBgy, navBgz,
                        ahrs.getAccelConfidence());
          diagEkfTimingEnd();
          diagNoteNavEkfState(navEkf);

          diagPrintNavLineIfDue(ahrs, navEkf, ax, ay, az, roll, pitch, write_binary, sampleUs);
        }
      }
    } else {
      diagNoteImuMissed();
    }

    yaw_cal   = (uint16_t)constrain((yaw   + 180.0f) * 32767.0f / 180.0f, 0.0f, 65535.0f);
    pitch_cal = (uint16_t)constrain((pitch + 90.0f)  * 32767.0f / 180.0f, 0.0f, 65535.0f);
    roll_cal  = (uint16_t)constrain((roll  + 180.0f) * 32767.0f / 180.0f, 0.0f, 65535.0f);

    ax_cal = (uint16_t)constrain((ax + 2.0f) * 16383.5f, 0.0f, 65535.0f);
    ay_cal = (uint16_t)constrain((ay + 2.0f) * 16383.5f, 0.0f, 65535.0f);
    az_cal = (uint16_t)constrain((az + 2.0f) * 16383.5f, 0.0f, 65535.0f);

    gx_cal = (uint16_t)constrain((gx + 2000.0f) * 65535.0f / 4000.0f, 0.0f, 65535.0f);
    gy_cal = (uint16_t)constrain((gy + 2000.0f) * 65535.0f / 4000.0f, 0.0f, 65535.0f);
    gz_cal = (uint16_t)constrain((gz + 2000.0f) * 65535.0f / 4000.0f, 0.0f, 65535.0f);

    // MUX sensor reads run every sample tick regardless of IMU freshness.
    for (int i = 0; i < NUM_SENSORS; i++) {
      fsrValues[i]  = readMux(i);
      flexValues[i] = readMux(i + 5);
    }

    if (print_mode == 0) buildAndSendFrame();
  }

  if ((print_mode == 1 || print_mode == 2) &&
      (millis() - lastRawPrintMillis >= RAW_PRINT_INTERVAL_MS)) {
    lastRawPrintMillis = millis();

    if (print_mode == 1) {
      for (int i = 0; i < NUM_SENSORS; i++) { Serial.print(flexValues[i]); Serial.print('\t'); }
      for (int i = 0; i < NUM_SENSORS; i++) { Serial.print(fsrValues[i]);  Serial.print('\t'); }
      float qw, qx, qy, qz;
      ahrs.getQuaternion(qw, qx, qy, qz);
      Serial.printf("%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.4f\t%.4f\t%.4f\t%.4f\n",
        yaw, pitch, roll, gx, gy, gz, ax, ay, az, qw, qx, qy, qz);
    } else if (print_mode == 2) {
      for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(constrain(map(flexValues[i], MIN_FLEX[i], MAX_FLEX[i], 0, 255), 0, 255));
        Serial.print('\t');
      }
      for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(constrain(map(fsrValues[i], MIN_PRESS[i], MAX_PRESS[i], 0, 255), 0, 255));
        Serial.print('\t');
      }
      Serial.printf("%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\n",
        yaw_cal, pitch_cal, roll_cal, gx_cal, gy_cal, gz_cal, ax_cal, ay_cal, az_cal);
    }
  }

  if (now - lastDisplayMillis >= DISPLAY_INTERVAL_MS) {
    lastDisplayMillis = now;
    updateBatteryStatus();
    updateDisplayDimming();
    updateDisplay();
  }

  diagPrintSysLineIfDue(ahrs, navEkf, write_binary);

  diagLoopTimingEnd();
}
