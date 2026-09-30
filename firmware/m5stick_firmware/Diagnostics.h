#ifndef GLUVN_DIAGNOSTICS_H
#define GLUVN_DIAGNOSTICS_H

/*
 * Diagnostics.h -- ALL firmware/estimator diagnostics live here, and only
 * here. Nothing here changes estimator behavior, gains, or Q/R -- observes
 * and reports only.
 *
 * Design so future diagnostics don't require touching the main .ino:
 *   - All state (counters, EMAs, toggles, throttle timers) lives in this file.
 *   - The .ino calls small `diagNote*()` / `diagBegin*()`/`diagEnd*()` hooks
 *     at the exact point each event happens (IMU sample arrives/missed, dt
 *     computed, a filter runs, etc) -- these must stay at their call sites
 *     since that's the only place the event is known to happen, but each is
 *     a single line, no logic beyond "record this".
 *   - The .ino calls `diagPrintSysLineIfDue(...)` / `diagPrintNavLineIfDue(...)`
 *     once each per loop() iteration; whether anything prints, and what/how,
 *     lives here.
 *   - To add a diagnostic: add its state, a `diagNote*()` hook if it needs
 *     capturing at a specific point, and a field in the relevant print
 *     function. Only touch the .ino if the new diagnostic needs a hook site
 *     that doesn't already exist.
 *
 * Single-TU Arduino sketch (the .ino + all headers compile as one
 * translation unit), included exactly once from the main .ino -- so plain
 * (non-extern) global definitions here are ODR-safe, no manual `extern`
 * bookkeeping needed.
 *
 * Commands (handled in the .ino's processCommand(); trivial forwards into
 * this file's toggles/printers):
 *   NAV_DIAG_ON/OFF  -- per-sample estimator diagnostics, 10 Hz, text only
 *   SYS_DIAG_ON/OFF  -- system/stability diagnostics, 1 Hz, text only
 *   STATUS           -- one-shot SYS line, ignores the toggle and the throttle
 *
 * NAV and SYS output both require write_binary==false: Serial text output
 * would otherwise interleave with and corrupt the BLE/serial binary frame
 * protocol on the same stream.
 *
 * loop_us vs calib_us: runStartupGyroCalibration() is a deliberate,
 * multi-second BLOCKING routine (collects STARTUP_GYRO_CAL_SAMPLES gyro
 * samples while stationary), called from inside loop() at multiple sites.
 * Left unaccounted for, its duration got folded into loop_us_max, pinning a
 * diagnostic meant to catch UNEXPECTED stalls at a huge, calibration-dominated
 * number. Now timed separately (diagCalibTimingBegin/End, reported as
 * calib_us/calib_us_max/calib_count) and excluded from loop_us/loop_us_max --
 * see diagCalibTimingEnd().
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>   // uxTaskGetStackHighWaterMark
#include <esp_system.h>      // esp_reset_reason
#include "MahonyAHRS.h"
#include "NavEKF.h"

// ---------------- toggles (set by processCommand(), read here) ----------------
bool nav_diagnostics = false;
bool sys_diagnostics = false;

// ---------------- cumulative counters/state (since boot unless noted) ----------------
uint32_t diagBootCount       = 0;

uint32_t diagImuFreshCount   = 0;   // M5.Imu.update() returned true
uint32_t diagImuMissedCount  = 0;   // M5.Imu.update() returned false at a sample deadline
uint32_t diagDtRejectCount   = 0;   // dt <=0 or > MAX_NAV_GAP_SEC: sample not fed to estimators
uint32_t diagDtUnder2msCount = 0;   // accepted dt < 2 ms, quantifies catch-up/back-to-back reads
float    diagDtMin           = -1.0f;   // seconds; -1 = no usable sample yet
float    diagDtMax           = -1.0f;
float    diagDtRejectMax     = -1.0f;   // largest dt among REJECTED samples -- dt_max only covers
                                          // accepted ones, so a correctly-rejected stall (e.g.
                                          // blocking gyro calib) was otherwise invisible here.

uint32_t diagEkfExecUsLast   = 0;
uint32_t diagEkfExecUsMax    = 0;
uint32_t diagLoopExecUsLast  = 0;
uint32_t diagLoopExecUsMax   = 0;

// Startup/re-calibration is a deliberate, multi-second BLOCKING op, called
// from loop() at up to 3 sites. Timed separately rather than left to fall
// into loop_us/loop_us_max, which exist to catch UNEXPECTED stalls --
// conflating the two made loop_us_max meaningless.
uint32_t diagCalibExecUsLast = 0;
uint32_t diagCalibExecUsMax  = 0;
uint32_t diagCalibCount      = 0;

// loop_us_max=28,357us was observed on real hardware -- far above the
// ~200-2500us range everything else in loop() normally costs. These narrow
// down WHICH subsystem is responsible next time it happens.
uint32_t diagMuxExecUsLast    = 0, diagMuxExecUsMax    = 0;
uint32_t diagFrameExecUsLast  = 0, diagFrameExecUsMax  = 0;
uint32_t diagDisplayExecUsLast = 0, diagDisplayExecUsMax = 0;
uint32_t diagNavPrintExecUsLast = 0, diagNavPrintExecUsMax = 0;

uint32_t diagNanGyroCount    = 0;   // raw gx,gy,gz non-finite
uint32_t diagNanAccelCount   = 0;   // raw ax,ay,az non-finite
uint32_t diagNanMahonyCount  = 0;   // ahrs quaternion non-finite
uint32_t diagNanNavEkfCount  = 0;   // navEkf position/velocity/attitude non-finite

static unsigned long diagLastSysMillis = 0;
static uint32_t      diagLastNavUs     = 0;
static const unsigned long DIAG_SYS_INTERVAL_MS = 1000;    // 1 Hz -- keeps the 100 Hz loop undisturbed
static const uint32_t      DIAG_NAV_INTERVAL_US = 100000UL; // 10 Hz

static uint32_t diagEkfStartUs_   = 0;
static uint32_t diagLoopStartUs_  = 0;
static uint32_t diagCalibStartUs_ = 0;
static uint32_t diagMuxStartUs_ = 0, diagFrameStartUs_ = 0, diagDisplayStartUs_ = 0;
static uint32_t diagNavPrintStartUs_ = 0;

// ---------------- hooks: call at the point of occurrence in the .ino ----------------

inline void diagSetBootCount(uint32_t n) { diagBootCount = n; }

inline void diagNoteImuFresh()  { diagImuFreshCount++; }
inline void diagNoteImuMissed() { diagImuMissedCount++; }

// Call with accepted=false the moment a sample's dt fails the usability
// check; call with accepted=true (and only then) once the sample has
// actually been fed to Mahony/NavEKF, so dt_min/dt_max reflect what the
// estimators actually saw, not raw scheduler jitter.
inline void diagNoteDt(float dt, bool accepted) {
    if (!accepted) {
        diagDtRejectCount++;
        if (dt > diagDtRejectMax) diagDtRejectMax = dt;
        return;
    }
    if (diagDtMin < 0.0f || dt < diagDtMin) diagDtMin = dt;
    if (dt > diagDtMax) diagDtMax = dt;
    if (dt < 0.002f) diagDtUnder2msCount++;
}

inline void diagNoteRawGyro(float gx, float gy, float gz) {
    if (!isfinite(gx) || !isfinite(gy) || !isfinite(gz)) diagNanGyroCount++;
}
inline void diagNoteRawAccel(float ax, float ay, float az) {
    if (!isfinite(ax) || !isfinite(ay) || !isfinite(az)) diagNanAccelCount++;
}
inline void diagNoteMahonyQuat(float q0, float q1, float q2, float q3) {
    if (!isfinite(q0) || !isfinite(q1) || !isfinite(q2) || !isfinite(q3)) diagNanMahonyCount++;
}
inline void diagNoteNavEkfState(NavEKF &navEkf) {
    float px, py, pz, vx, vy, vz, qw, qx, qy, qz;
    navEkf.getPosition(px, py, pz);
    navEkf.getVelocity(vx, vy, vz);
    navEkf.getAttitude(qw, qx, qy, qz);
    if (!isfinite(px) || !isfinite(py) || !isfinite(pz) ||
        !isfinite(vx) || !isfinite(vy) || !isfinite(vz) ||
        !isfinite(qw) || !isfinite(qx) || !isfinite(qy) || !isfinite(qz))
        diagNanNavEkfCount++;
}

inline void diagEkfTimingBegin() { diagEkfStartUs_ = micros(); }
inline void diagEkfTimingEnd() {
    diagEkfExecUsLast = micros() - diagEkfStartUs_;
    if (diagEkfExecUsLast > diagEkfExecUsMax) diagEkfExecUsMax = diagEkfExecUsLast;
}
inline void diagLoopTimingBegin() { diagLoopStartUs_ = micros(); }
inline void diagLoopTimingEnd() {
    diagLoopExecUsLast = micros() - diagLoopStartUs_;
    if (diagLoopExecUsLast > diagLoopExecUsMax) diagLoopExecUsMax = diagLoopExecUsLast;
}

// Wrap the ENTIRE body of runStartupGyroCalibration() with these two calls.
// diagCalibTimingEnd() also fast-forwards diagLoopStartUs_ to "now", excluding
// the calibration's multi-second duration from loop_us/loop_us_max -- see
// top-of-file comment.
inline void diagCalibTimingBegin() { diagCalibStartUs_ = micros(); }
inline void diagCalibTimingEnd() {
    diagCalibExecUsLast = micros() - diagCalibStartUs_;
    if (diagCalibExecUsLast > diagCalibExecUsMax) diagCalibExecUsMax = diagCalibExecUsLast;
    diagCalibCount++;
    diagLoopStartUs_ = micros();
}

inline void diagMuxTimingBegin() { diagMuxStartUs_ = micros(); }
inline void diagMuxTimingEnd() {
    diagMuxExecUsLast = micros() - diagMuxStartUs_;
    if (diagMuxExecUsLast > diagMuxExecUsMax) diagMuxExecUsMax = diagMuxExecUsLast;
}
inline void diagFrameTimingBegin() { diagFrameStartUs_ = micros(); }
inline void diagFrameTimingEnd() {
    diagFrameExecUsLast = micros() - diagFrameStartUs_;
    if (diagFrameExecUsLast > diagFrameExecUsMax) diagFrameExecUsMax = diagFrameExecUsLast;
}
inline void diagDisplayTimingBegin() { diagDisplayStartUs_ = micros(); }
inline void diagDisplayTimingEnd() {
    diagDisplayExecUsLast = micros() - diagDisplayStartUs_;
    if (diagDisplayExecUsLast > diagDisplayExecUsMax) diagDisplayExecUsMax = diagDisplayExecUsLast;
}
inline void diagNavPrintTimingBegin() { diagNavPrintStartUs_ = micros(); }
inline void diagNavPrintTimingEnd() {
    diagNavPrintExecUsLast = micros() - diagNavPrintStartUs_;
    if (diagNavPrintExecUsLast > diagNavPrintExecUsMax) diagNavPrintExecUsMax = diagNavPrintExecUsLast;
}

// Clears every since-boot max/count (not cumulative identity counters like
// diagBootCount/diagImuFreshCount, which describe uptime totals, not
// anomalies) so a test window's diagnostics aren't contaminated by earlier
// events (e.g. an old loop_us_max from calibration). Call via DIAG_RESET_MAX
// right before a test you want clean diagnostics for.
inline void diagResetMax(NavEKF &navEkf) {
    diagDtRejectCount  = 0;
    diagDtUnder2msCount = 0;
    diagDtMin          = -1.0f;
    diagDtMax          = -1.0f;
    diagDtRejectMax    = -1.0f;
    diagEkfExecUsLast  = 0;
    diagEkfExecUsMax   = 0;
    diagLoopExecUsLast = 0;
    diagLoopExecUsMax  = 0;
    diagCalibExecUsLast = 0;
    diagCalibExecUsMax  = 0;
    diagCalibCount      = 0;
    diagMuxExecUsLast = 0; diagMuxExecUsMax = 0;
    diagFrameExecUsLast = 0; diagFrameExecUsMax = 0;
    diagDisplayExecUsLast = 0; diagDisplayExecUsMax = 0;
    diagNavPrintExecUsLast = 0; diagNavPrintExecUsMax = 0;
    diagNanGyroCount   = 0;
    diagNanAccelCount  = 0;
    diagNanMahonyCount = 0;
    diagNanNavEkfCount = 0;
    navEkf.resetPDiagBlowupStats();
}

// ---------------- reset reason ----------------
inline const char* diagResetReasonToString(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_UNKNOWN:   return "UNKNOWN";
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXT_PIN";
        case ESP_RST_SW:        return "SW_RESET";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "OTHER_WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP_WAKE";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "OTHER";
    }
}

// ---------------- SYS line ----------------
// One consolidated line covering: (1) stack/heap, (2) watchdog/brownout
// reset info, (3) quaternion/attitude norms + NaN counters, (4) timing/IMU
// sample handling, (5) estimator behavior (ekf_us*, Pdiag_min/max; see also
// the NAV line).
inline void diagPrintSysLine(MahonyAHRS &ahrs, NavEKF &navEkf) {
    uint32_t freeHeap    = ESP.getFreeHeap();
    uint32_t minFreeHeap = ESP.getMinFreeHeap();
    UBaseType_t stackHwm = uxTaskGetStackHighWaterMark(NULL);   // bytes, ESP32 FreeRTOS port

    float mq0, mq1, mq2, mq3;
    ahrs.getQuaternion(mq0, mq1, mq2, mq3);
    float mahonyQNorm = sqrtf(mq0 * mq0 + mq1 * mq1 + mq2 * mq2 + mq3 * mq3);

    float nq0, nq1, nq2, nq3;
    navEkf.getAttitude(nq0, nq1, nq2, nq3);
    float navQNorm = sqrtf(nq0 * nq0 + nq1 * nq1 + nq2 * nq2 + nq3 * nq3);

    float pMinDiag, pMaxDiag;
    navEkf.getCovarianceDiagRange(pMinDiag, pMaxDiag);

    float pVarActual = navEkf.getPositionVarianceMax();
    float pVarExpected = navEkf.getExpectedPositionVarFromNoise();
    float vVarActual = navEkf.getVelocityVarianceMax();
    float vVarExpected = navEkf.getExpectedVelocityVarFromNoise();
    float tSinceZupt = navEkf.getTimeSinceZupt();

    uint32_t accelClampCount, gyroClampCount;
    navEkf.getBiasClampCounts(accelClampCount, gyroClampCount);

    float gravWorstDetS = navEkf.getGravityWorstDetS();
    float zuptBaOvershootMax = navEkf.getZuptBaOvershootMax();
    float zuptBgOvershootMax = navEkf.getZuptBgOvershootMax();

    int32_t battPct = M5.Power.getBatteryLevel();
    int16_t battMv  = M5.Power.getBatteryVoltage();

    Serial.printf(
        "SYS,boot=%u,reset=%s,up_ms=%lu,heap_free=%u,heap_min=%u,stack_hwm=%u,"
        "loop_us=%u,loop_us_max=%u,ekf_us=%u,ekf_us_max=%u,"
        "calib_us=%u,calib_us_max=%u,calib_count=%u,"
        "mux_us=%u,mux_us_max=%u,frame_us=%u,frame_us_max=%u,disp_us=%u,disp_us_max=%u,"
        "navprint_us=%u,navprint_us_max=%u,"
        "imu_fresh=%u,imu_missed=%u,dt_reject=%u,dt_under2ms=%u,dt_min=%.5f,dt_max=%.5f,dt_reject_max=%.5f,"
        "nan_gyro=%u,nan_accel=%u,nan_mahony=%u,nan_navekf=%u,"
        "mahonyQnorm=%.5f,navQnorm=%.5f,Pdiag_min=%.3e,Pdiag_max=%.3e,"
        "armed=%d,t_since_zupt=%.3f,zupt_starved=%d,pvar_actual=%.4e,pvar_expected=%.4e,"
        "vvar_actual=%.4e,vvar_expected=%.4e,"
        "ba_clamp=%u,bg_clamp=%u,"
        "grav_invalid_gate=%u,zupt_invalid_gate=%u,grav_worst_detS=%.4e,"
        "zupt_ba_overshoot_max=%.4f,zupt_bg_overshoot_max=%.4f,"
        "block_bias_correction=%d,"
        "batt_pct=%ld,batt_mv=%d\n",
        (unsigned)diagBootCount,
        diagResetReasonToString(esp_reset_reason()),
        (unsigned long)millis(),
        (unsigned)freeHeap, (unsigned)minFreeHeap, (unsigned)stackHwm,
        (unsigned)diagLoopExecUsLast, (unsigned)diagLoopExecUsMax,
        (unsigned)diagEkfExecUsLast, (unsigned)diagEkfExecUsMax,
        (unsigned)diagCalibExecUsLast, (unsigned)diagCalibExecUsMax, (unsigned)diagCalibCount,
        (unsigned)diagMuxExecUsLast, (unsigned)diagMuxExecUsMax,
        (unsigned)diagFrameExecUsLast, (unsigned)diagFrameExecUsMax,
        (unsigned)diagDisplayExecUsLast, (unsigned)diagDisplayExecUsMax,
        (unsigned)diagNavPrintExecUsLast, (unsigned)diagNavPrintExecUsMax,
        (unsigned)diagImuFreshCount, (unsigned)diagImuMissedCount, (unsigned)diagDtRejectCount,
        (unsigned)diagDtUnder2msCount,
        diagDtMin, diagDtMax, diagDtRejectMax,
        (unsigned)diagNanGyroCount, (unsigned)diagNanAccelCount,
        (unsigned)diagNanMahonyCount, (unsigned)diagNanNavEkfCount,
        mahonyQNorm, navQNorm, pMinDiag, pMaxDiag,
        (int)navEkf.isArmed(), tSinceZupt, (int)navEkf.isZuptStarved(), pVarActual, pVarExpected,
        vVarActual, vVarExpected,
        (unsigned)accelClampCount, (unsigned)gyroClampCount,
        (unsigned)navEkf.getGravityInvalidGateCount(), (unsigned)navEkf.getZuptInvalidGateCount(),
        gravWorstDetS,
        zuptBaOvershootMax, zuptBgOvershootMax,
        (int)navEkf.isBlockingBiasCorrectionFromGravityUpdate(),
        (long)battPct, (int)battMv);
}

// Throttled (1 Hz) form for the main loop; requires binary streaming off.
inline void diagPrintSysLineIfDue(MahonyAHRS &ahrs, NavEKF &navEkf, bool write_binary) {
    if (!sys_diagnostics || write_binary) return;
    unsigned long now = millis();
    if (now - diagLastSysMillis < DIAG_SYS_INTERVAL_MS) return;
    diagLastSysMillis = now;
    diagPrintSysLine(ahrs, navEkf);
}

// One-shot form for STATUS: ignores sys_diagnostics and the throttle, but
// still requires write_binary==false -- printing text into an active binary
// stream would corrupt that frame regardless of why. Returns false (prints
// nothing) if binary mode is active, so the caller can report that back.
inline bool diagPrintSysLineOnDemand(MahonyAHRS &ahrs, NavEKF &navEkf, bool write_binary) {
    if (write_binary) return false;
    diagPrintSysLine(ahrs, navEkf);
    return true;
}

// ---------------- NAV line (per-sample estimator diagnostics) ----------------
// fLinearBody/|fLinearBody|/gravity-valid/gravity-active/ZUPT-active/
// stationary-state/velocity.
inline void diagPrintNavLineIfDue(MahonyAHRS &ahrs, NavEKF &navEkf,
                                   float ax, float ay, float az,
                                   float roll, float pitch,
                                   bool write_binary, uint32_t sampleUs) {
    if (!nav_diagnostics || write_binary) return;
    if (sampleUs - diagLastNavUs < DIAG_NAV_INTERVAL_US) return;
    diagLastNavUs = sampleUs;

    float worldAx, worldAy, worldAz;
    float navVx, navVy, navVz;
    float navBaX, navBaY, navBaZ;
    float innovX, innovY, innovZ;
    float navQw, navQx, navQy, navQz;
    float navPx, navPy, navPz;
    float mahonyGx, mahonyGy, mahonyGz;
    float navGx, navGy, navGz;
    float fLinX, fLinY, fLinZ;
    float gravityD2, gravityR0, gravityR1, gravityR2;
    float gravityS0, gravityS1, gravityS2;
    bool gravityAttempted;
    bool gravityBypass;
    float rawNorm = sqrtf(ax * ax + ay * ay + az * az);
    float rawGx = ax / rawNorm, rawGy = ay / rawNorm, rawGz = az / rawNorm;

    navEkf.getAttitude(navQw, navQx, navQy, navQz);
    navEkf.getPosition(navPx, navPy, navPz);
    navEkf.getWorldAcceleration(worldAx, worldAy, worldAz);
    navEkf.getVelocity(navVx, navVy, navVz);
    navEkf.getAccelBias(navBaX, navBaY, navBaZ);
    navEkf.getLastAttitudeInnovationRad(innovX, innovY, innovZ);
    navEkf.getPredictedBodyGravity(navGx, navGy, navGz);
    navEkf.getLinearAccelBody(fLinX, fLinY, fLinZ);
    navEkf.getGravityUpdateDiagnostics(gravityD2,
                                       gravityR0, gravityR1, gravityR2,
                                       gravityS0, gravityS1, gravityS2,
                                       gravityAttempted,
                                       gravityBypass);
    ahrs.getBodyGravity(mahonyGx, mahonyGy, mahonyGz);

    float dotM = rawGx * mahonyGx + rawGy * mahonyGy + rawGz * mahonyGz;
    float dotN = rawGx * navGx + rawGy * navGy + rawGz * navGz;
    float angMahony = acosf(constrain(dotM, -1.0f, 1.0f));
    float angNav    = acosf(constrain(dotN, -1.0f, 1.0f));
    float worldAMag = sqrtf(worldAx * worldAx + worldAy * worldAy + worldAz * worldAz);

    diagNavPrintTimingBegin();
    Serial.printf(
        "NAV,nq=%.5f,%.5f,%.5f,%.5f,mrp=%.2f,%.2f,gvalid=%d,gactive=%d,conf=%.3f,"
        "p=%.4f,%.4f,%.4f,"
        "aW=%.4f,%.4f,%.4f,|aW|=%.4f,v=%.4f,%.4f,%.4f,st=%d,zupt=%d,stationaryTime=%.3f,"
        "fLin=%.4f,%.4f,%.4f,|fLin|=%.4f,gyroEMA=%.4f,fLinEMA=%.4f,"
        "grav_attempted=%d,grav_d2=%.4f,grav_bypass=%d,grav_linTrust=%.4f,grav_tangent=%d,zupt_d2=%.4f,"
        "grav_Rdiag=%.4e,%.4e,%.4e,"
        "grav_Sdiag=%.4e,%.4e,%.4e,"
        "ba=%.5f,%.5f,%.5f,att=%.5f,%.5f,%.5f,"
        "rawG=%.4f,%.4f,%.4f,mG=%.4f,%.4f,%.4f,nG=%.4f,%.4f,%.4f,angM=%.4f,angN=%.4f\n",
        navQw, navQx, navQy, navQz, roll, pitch,
        navEkf.isGravityMeasurementValid(), navEkf.isGravityUpdateActive(), ahrs.getAccelConfidence(),
        navPx, navPy, navPz,
        worldAx, worldAy, worldAz, worldAMag, navVx, navVy, navVz,
        navEkf.isOwnStationary(), navEkf.isZuptActive(), navEkf.getStationaryTime(),
        fLinX, fLinY, fLinZ, navEkf.getLinearAccelNorm(),
        navEkf.getGyroMagEMA(), navEkf.getFLinNormEMA(),
        gravityAttempted, gravityD2, gravityBypass, navEkf.getGravityLinAccelTrust(),
        (int)navEkf.isUsingTangentPlaneGravityUpdate(), navEkf.getZuptMahalanobis(),
        gravityR0, gravityR1, gravityR2,
        gravityS0, gravityS1, gravityS2,
        navBaX, navBaY, navBaZ, innovX, innovY, innovZ,
        rawGx, rawGy, rawGz,
        mahonyGx, mahonyGy, mahonyGz,
        navGx, navGy, navGz,
        angMahony, angNav);
    diagNavPrintTimingEnd();
}

#endif // GLUVN_DIAGNOSTICS_H
