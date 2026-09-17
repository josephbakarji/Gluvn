#ifndef MAHONY_AHRS_H
#define MAHONY_AHRS_H

// Mahony AHRS with adaptive gain, motion/stationary detection, gyro bias
// tracking, jerk-gated accel confidence, output smoothing, and startup
// gyro calibration (Welford online mean/variance, streaming outlier
// rejection). Part 1: class + state. Part 2: updateIMU(). Part 3: Euler
// conversion and utilities.

#include <Arduino.h>
#include <math.h>

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.01745329251994329577f
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876f
#endif

#ifndef G_MS2
#define G_MS2 9.80665f
#endif

class MahonyAHRS
{

public:
    MahonyAHRS()
    {
        reset();
    }

    void reset()
    {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;

        integralX = 0.0f;
        integralY = 0.0f;
        integralZ = 0.0f;

        yaw = 0.0f;
        pitch = 0.0f;
        roll = 0.0f;
        rawYawDeg_ = 0.0f;

        yawFiltered = 0.0f;
        pitchFiltered = 0.0f;
        rollFiltered = 0.0f;

        yawOffset = 0.0f;

        //---------------- Motion ----------------
        stationary = true;
        accelConfidence = 1.0f;
        gyroMagnitude = 0.0f;
        accelMagnitude = 1.0f;

        //---------------- Gains ----------------
        kpStatic = 1.5f;    // standard Mahony range: 0.5-2.0
        kpDynamic = 0.30f;  // reduced trust in accel during motion

        kiStatic = 0.05f;   // corrects slow gyro bias when still
        kiDynamic = 0.0f;   // no integral correction while moving (avoids wind-up)

        kp = kpStatic;
        ki = kiStatic;

        //---------------- Thresholds ----------------
        stationaryGyroThreshold = 6.0f;
        stationaryAccelThreshold = 0.08f;

        stationaryGyroExitThreshold = 7.5f;
        stationaryAccelExitThreshold = 0.12f;

        accelRejectThreshold = 0.25f;

        //---------------- Output filter ----------------
        outputAlpha = 0.15f;
        outputTau = 0.12f;
        lastDt = 0.0f;

        //---------------- Musical yaw ----------------
        yawMusical = 0.0f;
        yawMusicalTau = 8.0f; // s -- tune per instrument: shorter = snappier/twitchier

        //---------------- Gyro bias tracking ----------------
        gyroBiasX = 0.0f;
        gyroBiasY = 0.0f;
        gyroBiasZ = 0.0f;

        biasTauFast = 20.0f;   // roll/pitch bias: cross-checked by gravity, can adapt faster
        biasTauSlowZ = 60.0f;  // yaw bias: no absolute reference, adapt conservatively

        zBiasEngageTime = 2.0f; // s of unbroken stillness before trusting Z bias update

        stationaryTime = 0.0f;

        //---------------- Startup calibration ----------------
        calibrating = false;
        calibCount = 0;
        calibTargetSamples = 3000;

        calibMeanX = calibMeanY = calibMeanZ = 0.0f;
        calibM2X = calibM2Y = calibM2Z = 0.0f;

        //---------------- Auto-timestep ----------------
        lastMicros = 0;

        //---------------- Gyro/accel pre-filtering ----------------
        gyroFilterTau = 0.02f;   // ~8 Hz cutoff, light touch
        gyroFilteredX = 0.0f;
        gyroFilteredY = 0.0f;
        gyroFilteredZ = 0.0f;

        accelFilterTau = 0.015f; // ~10 Hz cutoff
        accelFilteredX = 0.0f;
        accelFilteredY = 0.0f;
        accelFilteredZ = 1.0f;

        //---------------- Jerk / high-rate rejection ----------------
        prevRawAx = 0.0f;
        prevRawAy = 0.0f;
        prevRawAz = 1.0f;
        haveJerkPrev = false;

        jerkRejectThreshold = 3.0f;      // g/s
        gyroRateRejectThreshold = 300.0f; // deg/s

        //---------------- Variance-gated stationary detection ----------------
        varianceWindowTau = 0.5f; // s

        gyroMagMeanEMA = 0.0f;
        gyroMagVarEMA = 0.0f;

        accelMagMeanEMA = 1.0f;
        accelMagVarEMA = 0.0f;

        gyroVarThreshold = 9.0f;   // (deg/s)^2
        accelVarThreshold = 0.02f; // g^2
    }

    // Main update (implemented in Part 2)

    void updateIMU(
        float gx,
        float gy,
        float gz,
        float ax,
        float ay,
        float az,
        float dt);

    void update(
        float gx,
        float gy,
        float gz,
        float ax,
        float ay,
        float az)
    {
        uint32_t now = micros();
        if (lastMicros == 0)
        {
            lastMicros = now;
            return; // first call only establishes t0
        }

        float dt = (float)(now - lastMicros) * 1.0e-6f;
        lastMicros = now;
        if (dt <= 0.0f)
            return;

        updateIMU(gx, gy, gz, ax, ay, az, dt);
    }

    // Startup gyro calibration

    void beginGyroCalibration(uint16_t targetSamples = 3000)
    {
        calibrating = true;
        calibCount = 0;
        calibTargetSamples = targetSamples;
        calibConsecutiveRejects = 0;

        calibMeanX = calibMeanY = calibMeanZ = 0.0f;
        calibM2X = calibM2Y = calibM2Z = 0.0f;
    }

    bool updateGyroCalibration(float gx, float gy, float gz)
    {
        if (!calibrating)
            return true;

        if (calibCount > 50)
        {
            float sx = sqrtf(calibM2X / calibCount);
            float sy = sqrtf(calibM2Y / calibCount);
            float sz = sqrtf(calibM2Z / calibCount);

            bool outlier = fabsf(gx - calibMeanX) > 4.0f * sx + 0.01f ||
                           fabsf(gy - calibMeanY) > 4.0f * sy + 0.01f ||
                           fabsf(gz - calibMeanZ) > 4.0f * sz + 0.01f;

            if (outlier)
            {
                calibConsecutiveRejects++;
                if (calibConsecutiveRejects < 50)
                    return false;

                // 50 straight rejects means the running mean/variance no
                // longer describes the true rest state (e.g. it locked onto
                // a transient during the first ~50 samples) -- restart
                // accumulation from this sample rather than rejecting forever.
                calibCount = 0;
                calibConsecutiveRejects = 0;
                calibMeanX = calibMeanY = calibMeanZ = 0.0f;
                calibM2X = calibM2Y = calibM2Z = 0.0f;
            }
        }

        calibConsecutiveRejects = 0;
        calibCount++;

        float dx = gx - calibMeanX;
        calibMeanX += dx / calibCount;
        calibM2X += dx * (gx - calibMeanX);

        float dy = gy - calibMeanY;
        calibMeanY += dy / calibCount;
        calibM2Y += dy * (gy - calibMeanY);

        float dz = gz - calibMeanZ;
        calibMeanZ += dz / calibCount;
        calibM2Z += dz * (gz - calibMeanZ);

        return calibCount >= calibTargetSamples;
    }

    void endGyroCalibration()
    {
        calibrating = false;

        if (calibCount < 2)
            return;

        gyroBiasX = calibMeanX;
        gyroBiasY = calibMeanY;
        gyroBiasZ = calibMeanZ;
    }

    uint16_t getCalibCount() const { return calibCount; }

    // Call once, right after loading a freshly-computed bias via setGyroBias().
    // Clears state computed under the OLD bias (integral windup, motion-
    // detector EMAs, filter memory, jerk history, musical yaw) so it isn't
    // treated as valid evidence under the new one. Deliberately leaves q0..q3
    // (still a valid attitude) and yawOffset (owned by resetYaw(), called
    // separately by the firmware's recalibration flow) untouched.
    void reinitializeAfterBiasUpdate()
    {
        integralX = integralY = integralZ = 0.0f;

        gyroMagMeanEMA = 0.0f; gyroMagVarEMA = 0.0f;
        accelMagMeanEMA = 1.0f; accelMagVarEMA = 0.0f;
        stationaryTime = 0.0f;
        stationary = true;

        gyroFilteredX = gyroFilteredY = gyroFilteredZ = 0.0f;
        accelFilteredX = accelFilteredY = 0.0f;
        accelFilteredZ = 1.0f;

        prevRawAx = 0.0f; prevRawAy = 0.0f; prevRawAz = 1.0f;
        haveJerkPrev = false;

        yawMusical = 0.0f;
    }

    // Gyro bias access (persist to flash / restore across power cycles)

    void getGyroBias(float &bx, float &by, float &bz) const
    {
        bx = gyroBiasX;
        by = gyroBiasY;
        bz = gyroBiasZ;
    }

    void setGyroBias(float bx, float by, float bz)
    {
        gyroBiasX = bx;
        gyroBiasY = by;
        gyroBiasZ = bz;
    }

    void setGyroBiasTimeConstants(float tauFastXY, float tauSlowZ)
    {
        biasTauFast = constrain(tauFastXY, 0.1f, 600.0f);
        biasTauSlowZ = constrain(tauSlowZ, 0.1f, 600.0f);
    }

    void setZBiasEngageTime(float seconds)
    {
        zBiasEngageTime = constrain(seconds, 0.0f, 60.0f);
    }

    // Parameter setters

    void setStaticGain(float p, float i)
    {
        p = constrain(p, 0.0f, 100.0f);
        i = constrain(i, 0.0f, 10.0f);

        kpStatic = p;
        kiStatic = i;
    }

    void setDynamicGain(float p, float i)
    {
        p = constrain(p, 0.0f, 100.0f);
        i = constrain(i, 0.0f, 10.0f);

        kpDynamic = p;
        kiDynamic = i;
    }

    void setOutputFilter(float alpha)
    {
        alpha = constrain(alpha,0.0f,1.0f);
        outputAlpha = alpha;
    }

    void setOutputFilterTimeConstant(float tau)
    {
        outputTau = constrain(tau, 0.0f, 10.0f);
    }

    void setStationaryThresholds(
        float gyroDeg,
        float accelG)
    {
        gyroDeg = constrain(gyroDeg, 0.0f, 360.0f);
        accelG = constrain(accelG, 0.0f, 2.0f);

        stationaryGyroThreshold = gyroDeg;
        stationaryAccelThreshold = accelG;
    }

    void setStationaryExitThresholds(
        float gyroDeg,
        float accelG)
    {
        gyroDeg = constrain(gyroDeg, 0.0f, 360.0f);
        accelG = constrain(accelG, 0.0f, 2.0f);

        stationaryGyroExitThreshold = gyroDeg;
        stationaryAccelExitThreshold = accelG;
    }

    void setAccelerationRejectThreshold(float value)
    {
        value = constrain(value, 0.0f, 2.0f);
        accelRejectThreshold = value;
    }

    // Gyro / accel pre-filtering (dt-scaled EMA, applied before Mahony)

    void setGyroFilterTimeConstant(float tau)
    {
        gyroFilterTau = constrain(tau, 0.0f, 5.0f);
    }

    void setAccelFilterTimeConstant(float tau)
    {
        accelFilterTau = constrain(tau, 0.0f, 5.0f);
    }

    // Variance-gated stationary detection

    void setVarianceWindowTimeConstant(float tau)
    {
        varianceWindowTau = constrain(tau, 0.01f, 10.0f);
    }

    void setStationaryVarianceThresholds(float gyroVarDegPerSSq, float accelVarGSq)
    {
        gyroVarThreshold = constrain(gyroVarDegPerSSq, 0.0f, 100000.0f);
        accelVarThreshold = constrain(accelVarGSq, 0.0f, 100.0f);
    }

    // Jerk / high-rate acceleration rejection

    void setJerkRejectThreshold(float gPerSecond)
    {
        jerkRejectThreshold = constrain(gPerSecond, 0.0f, 1000.0f);
    }

    void setGyroRateRejectThreshold(float degPerSecond)
    {
        gyroRateRejectThreshold = constrain(degPerSecond, 0.0f, 20000.0f);
    }

    // Yaw reset -- DISPLAY/reference only: shifts yawOffset so getEuler()'s
    // yaw reads ~0 at the current heading. Does NOT touch q0..q3, so
    // getQuaternion()/getBodyGravity() and anything derived from actual
    // attitude are unaffected; NavEKF is never reset by this call.
    // rawYawDeg_ (raw, pre-offset yaw as of the last computeEuler()) is used
    // as the new offset rather than the already-offset-corrected `yaw` --
    // using the corrected value here was the old bug: only the first reset
    // from zero worked, every later one fed a moving target back in.
    void resetYaw()
    {
        yawOffset = rawYawDeg_;
        // Recompute displayed Euler output (raw + smoothed) immediately so a
        // caller reading getEuler() right after this sees the reset value
        // instead of a stale one, and smoothOutputs() doesn't glide toward
        // zero over ~outputTau seconds on the next cycle.
        yaw = 0.0f;
        yawFiltered = 0.0f;
    }

    float getYawMusical() const
    {
        return yawMusical;
    }

    void setYawMusicalTimeConstant(float tau)
    {
        yawMusicalTau = constrain(tau, 0.1f, 300.0f);
    }

    float getYawMusicalTimeConstant() const
    {
        return yawMusicalTau;
    }

    bool isStationary() const { return stationary; }

    float getStationaryTime() const { return stationaryTime; }

    float getAccelConfidence() const { return accelConfidence; }

    void getEuler(
        float &y,
        float &p,
        float &r)
    {
        y = yawFiltered;
        p = pitchFiltered;
        r = rollFiltered;
    }

    void getQuaternion(
        float &w,
        float &x,
        float &y,
        float &z) const
    {
        w = q0;
        x = q1;
        y = q2;
        z = q3;
    }

    void getBodyGravity(float &gx, float &gy, float &gz) const
    {
        // Body-to-world convention q * v_body = v_world -> gravity in body
        // coords is R^T * [0,0,1], the third column of R.
        gx = 2*(q1*q3 - q0*q2);
        gy = 2*(q2*q3 + q0*q1);
        gz = 1 - 2*(q1*q1+q2*q2);
    }

protected:

    // Quaternion
    float q0;
    float q1;
    float q2;
    float q3;

    // Euler
    float yaw;
    float pitch;
    float roll;

    float rawYawDeg_;   // pre-offset sensor-frame yaw, as of the last computeEuler(); see resetYaw()

    float yawFiltered;
    float pitchFiltered;
    float rollFiltered;

    float yawOffset;

    // Integral feedback
    float integralX;
    float integralY;
    float integralZ;

    // Gains
    float kp;
    float ki;

    float kpStatic;
    float kpDynamic;

    float kiStatic;
    float kiDynamic;

    // Motion detector
    bool stationary;

    float gyroMagnitude;
    float accelMagnitude;

    float accelConfidence;

    float stationaryGyroThreshold;
    float stationaryAccelThreshold;

    float stationaryGyroExitThreshold;
    float stationaryAccelExitThreshold;

    float accelRejectThreshold;

    // Output smoothing
    float outputAlpha;
    float outputTau;
    float lastDt;

    // Musical yaw
    float yawMusical;
    float yawMusicalTau;

    // Gyro bias tracking
    float gyroBiasX;
    float gyroBiasY;
    float gyroBiasZ;

    float biasTauFast;    // s, roll/pitch bias EMA time constant
    float biasTauSlowZ;   // s, yaw bias EMA time constant (conservative)

    float zBiasEngageTime; // s of continuous stillness required before Z updates
    float stationaryTime;  // s, running duration of current stationary streak

    // Startup calibration state (Welford online moments)
    bool calibrating;
    uint16_t calibCount;
    uint16_t calibTargetSamples;
    uint16_t calibConsecutiveRejects;

    float calibMeanX, calibMeanY, calibMeanZ;
    float calibM2X, calibM2Y, calibM2Z;

    // Auto-timestep state
    uint32_t lastMicros;

    // Gyro / accel pre-filtering
    float gyroFilterTau;
    float gyroFilteredX, gyroFilteredY, gyroFilteredZ;

    float accelFilterTau;
    float accelFilteredX, accelFilteredY, accelFilteredZ;

    // Jerk / high-rate rejection
    float prevRawAx, prevRawAy, prevRawAz;
    bool haveJerkPrev;

    float jerkRejectThreshold;
    float gyroRateRejectThreshold;

    // Variance-gated stationary detection
    float varianceWindowTau;

    float gyroMagMeanEMA, gyroMagVarEMA;
    float accelMagMeanEMA, accelMagVarEMA;

    float gyroVarThreshold;
    float accelVarThreshold;

    // Internal utilities (implemented in Part 3)
    void updateGyroBias(
        float gx,
        float gy,
        float gz,
        float dt);

    void updateMotionState(
        float gx,
        float gy,
        float gz,
        float ax,
        float ay,
        float az,
        float jerk,
        float dt);

    void updateAdaptiveGain();
    float computeJerk(float ax, float ay, float az, float dt);
    void filterGyro(float &gx, float &gy, float &gz, float dt);
    void filterAccel(float &ax, float &ay, float &az, float dt);

    void applyCorrection(
        float gxRad,
        float gyRad,
        float gzRad,
        float ex,
        float ey,
        float ez,
        float dt);

    void computeEuler();
    void smoothOutputs();
    float invSqrt(float x);
    float clamp(float x,float lo,float hi);
    float lerp(float a,float b,float t);
};

// Part 2 updateIMU()

void MahonyAHRS::updateIMU(
    float gx,
    float gy,
    float gz,
    float ax,
    float ay,
    float az,
    float dt)
{
    if (dt <= 0.0f)
        return;

    lastDt = dt;

    float jerk = computeJerk(ax, ay, az, dt);
    filterGyro(gx, gy, gz, dt);
    filterAccel(ax, ay, az, dt);
    updateMotionState(gx, gy, gz, ax, ay, az, jerk, dt);

    updateGyroBias(gx, gy, gz, dt);

    gx -= gyroBiasX;
    gy -= gyroBiasY;
    gz -= gyroBiasZ;

    float gxRad = gx * DEG_TO_RAD;
    float gyRad = gy * DEG_TO_RAD;
    float gzRad = gz * DEG_TO_RAD;
    updateAdaptiveGain();

    float norm =
        sqrtf(ax * ax +
              ay * ay +
              az * az);

    if (norm < 1e-6f)
        return;

    ax /= norm;
    ay /= norm;
    az /= norm;

    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;


    float ex = (ay * vz - az * vy) * accelConfidence;
    float ey = (az * vx - ax * vz) * accelConfidence;
    float ez = (ax * vy - ay * vx) * accelConfidence;

    applyCorrection(gxRad, gyRad, gzRad, ex, ey, ez, dt);
}

void MahonyAHRS::applyCorrection(
    float gxRad,
    float gyRad,
    float gzRad,
    float ex,
    float ey,
    float ez,
    float dt)
{

    if (ki > 0.0f)
    {
        integralX += ki * ex * dt;
        integralY += ki * ey * dt;
        integralZ += ki * ez * dt;

        integralX = clamp(integralX, -0.30f, 0.30f);
        integralY = clamp(integralY, -0.30f, 0.30f);
        integralZ = clamp(integralZ, -0.30f, 0.30f);
    }

    gxRad += kp * ex + integralX;
    gyRad += kp * ey + integralY;
    gzRad += kp * ez + integralZ;

    yawMusical += gzRad * RAD_TO_DEG * dt;
    yawMusical -= yawMusical * (dt / yawMusicalTau);

    // q is scalar-first, maps body vectors to world vectors.
    // q_dot = 0.5 q (*) [0, omega]
    float dq0 =
        0.5f *
        (-q1 * gxRad -
         q2 * gyRad -
         q3 * gzRad);

    float dq1 =
        0.5f * ( q0 * gxRad +     q2 * gzRad -     q3 * gyRad);

    float dq2 =
        0.5f * ( q0 * gyRad -     q1 * gzRad +     q3 * gxRad);

    float dq3 =
        0.5f * ( q0 * gzRad +     q1 * gyRad -     q2 * gxRad);

    // Integrate quaternion
    q0 += dq0 * dt;
    q1 += dq1 * dt;
    q2 += dq2 * dt;
    q3 += dq3 * dt;

    // Normalize quaternion
    float norm =
        q0 * q0 +
        q1 * q1 +
        q2 * q2 +
        q3 * q3;

    if (norm < 1e-12f)
    {
        reset();
        return;
    }
    norm = invSqrt(norm);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
    computeEuler();
    smoothOutputs();
}

// Part 3
// Utility functions

void MahonyAHRS::computeEuler()
{
    float R00 = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    float R10 =        2.0f * (q1 * q2 + q0 * q3);
    float R20 =        2.0f * (q1 * q3 - q0 * q2);
    float R21 =        2.0f * (q2 * q3 + q0 * q1);
    float R22 = 1.0f - 2.0f * (q1 * q1 + q2 * q2);

    // Yaw (outer axis, full +-180deg range)
    yaw = atan2f(R10, R00) * RAD_TO_DEG;
    rawYawDeg_ = yaw;   // pre-offset; resetYaw() needs this, not the offset-corrected value below

    // Pitch (bounded middle axis, +-90deg)
    float s = clamp(R20, -1.0f, 1.0f);
    pitch = -asinf(s) * RAD_TO_DEG;

    // Roll (inner axis, full +-180deg range)
    roll = atan2f(R21, R22) * RAD_TO_DEG;

    // Software yaw reset
    yaw -= yawOffset;
    while (yaw > 180.0f)
        yaw -= 360.0f;
    while (yaw < -180.0f)
        yaw += 360.0f;
}

void MahonyAHRS::updateGyroBias(
    float gx,
    float gy,
    float gz,
    float dt)
{
    // Stationary streak duration
    if (stationary)
        stationaryTime += dt;
    else
        stationaryTime = 0.0f;

    if (!stationary)
        return;

    float alphaFast = dt / (biasTauFast + dt);
    gyroBiasX += alphaFast * (gx - gyroBiasX);
    gyroBiasY += alphaFast * (gy - gyroBiasY);
    if (stationaryTime >= zBiasEngageTime)
    {
        float alphaSlowZ = dt / (biasTauSlowZ + dt);
        gyroBiasZ += alphaSlowZ * (gz - gyroBiasZ);
    }
}

void MahonyAHRS::updateMotionState(
    float gx,
    float gy,
    float gz,
    float ax,
    float ay,
    float az,
    float jerk,
    float dt)
{
    // Gyro / accel magnitude
    gyroMagnitude =
        sqrtf(
            gx * gx +
            gy * gy +
            gz * gz);

    accelMagnitude =
        sqrtf(
            ax * ax +
            ay * ay +
            az * az);

    float accelError =
        fabsf(accelMagnitude - 1.0f);

    float varAlpha = dt / (varianceWindowTau + dt);

    float dG = gyroMagnitude - gyroMagMeanEMA;
    gyroMagMeanEMA += varAlpha * dG;
    gyroMagVarEMA += varAlpha * (dG * dG - gyroMagVarEMA);

    float dA = accelMagnitude - accelMagMeanEMA;
    accelMagMeanEMA += varAlpha * dA;
    accelMagVarEMA += varAlpha * (dA * dA - accelMagVarEMA);

    // Stationary detector: magnitude hysteresis AND low variance
    bool magnitudeStill;

    if (stationary)
    {
        magnitudeStill =
            (gyroMagnitude < stationaryGyroExitThreshold) &&
            (accelError < stationaryAccelExitThreshold);
    }
    else
    {
        magnitudeStill =
            (gyroMagnitude < stationaryGyroThreshold) &&
            (accelError < stationaryAccelThreshold);
    }

    bool varianceStill =
        (gyroMagVarEMA < gyroVarThreshold) &&
        (accelMagVarEMA < accelVarThreshold);

    stationary = magnitudeStill && varianceStill;

    // Accelerometer confidence: distance from 1 g ...
    if (accelRejectThreshold > 1e-6f && accelError <= accelRejectThreshold)
    {
        float normalized = accelError / accelRejectThreshold;
        normalized = clamp(normalized, 0.0f, 1.0f);
        accelConfidence = 1.0f - normalized * normalized * (3.0f - 2.0f * normalized);
    }
    else
    {
        accelConfidence = 0.0f;
    }

    if (jerkRejectThreshold > 1e-6f && jerk > 0.0f)
    {
        float n = clamp(jerk / jerkRejectThreshold, 0.0f, 1.0f);
        accelConfidence *= 1.0f - n * n * (3.0f - 2.0f * n);
    }

    accelConfidence =
        clamp(accelConfidence, 0.0f, 1.0f);
}


float MahonyAHRS::computeJerk(
    float ax,
    float ay,
    float az,
    float dt)
{
    float jerk = 0.0f;

    if (haveJerkPrev && dt > 1e-6f)
    {
        float dax = ax - prevRawAx;
        float day = ay - prevRawAy;
        float daz = az - prevRawAz;
        jerk = sqrtf(dax * dax + day * day + daz * daz) / dt;
    }

    prevRawAx = ax;
    prevRawAy = ay;
    prevRawAz = az;
    haveJerkPrev = true;
    return jerk;
}


void MahonyAHRS::filterGyro(
    float &gx,
    float &gy,
    float &gz,
    float dt)
{
    if (gyroFilterTau <= 1e-6f)
    {
        gyroFilteredX = gx;
        gyroFilteredY = gy;
        gyroFilteredZ = gz;
        return;
    }

    float alpha = dt / (gyroFilterTau + dt);

    gyroFilteredX += alpha * (gx - gyroFilteredX);
    gyroFilteredY += alpha * (gy - gyroFilteredY);
    gyroFilteredZ += alpha * (gz - gyroFilteredZ);

    gx = gyroFilteredX;
    gy = gyroFilteredY;
    gz = gyroFilteredZ;
}

void MahonyAHRS::filterAccel(
    float &ax,
    float &ay,
    float &az,
    float dt)
{
    if (accelFilterTau <= 1e-6f)
    {
        accelFilteredX = ax;
        accelFilteredY = ay;
        accelFilteredZ = az;
        return;
    }

    float alpha = dt / (accelFilterTau + dt);

    accelFilteredX += alpha * (ax - accelFilteredX);
    accelFilteredY += alpha * (ay - accelFilteredY);
    accelFilteredZ += alpha * (az - accelFilteredZ);

    ax = accelFilteredX;
    ay = accelFilteredY;
    az = accelFilteredZ;
}

void MahonyAHRS::updateAdaptiveGain()
{
    float t = accelConfidence;
    t = clamp(t, 0.0f, 1.0f);
    if (stationary)
        t = 1.0f;

    kp = clamp(lerp(kpDynamic, kpStatic, t), 0.0f, 100.0f);
    ki = clamp(lerp(0.0f, kiStatic, t), 0.0f, 10.0f);
}

void MahonyAHRS::smoothOutputs()
{
    float alpha = outputAlpha;
    if (outputTau > 1e-6f && lastDt > 0.0f)
    {
        alpha = lastDt / (outputTau + lastDt);
    }

    alpha = clamp(alpha, 0.0f, 1.0f);

    if (alpha >= 0.999f)
    {
        rollFiltered = roll;
        pitchFiltered = pitch;
        yawFiltered = yaw;
        return;
    }

    // Roll
    float dr = roll - rollFiltered;
    if (dr > 180.0f)
        dr -= 360.0f;
    if (dr < -180.0f)
        dr += 360.0f;
    rollFiltered += alpha * dr;
    while (rollFiltered > 180.0f)
        rollFiltered -= 360.0f;
    while (rollFiltered < -180.0f)
        rollFiltered += 360.0f;

    // Pitch (asinf-derived, +/-90deg -- the wrap branches below are dead
    // code by construction since pitch can never reach +/-180deg, same as
    // yaw's smoothing used to be before the axis convention fix. Left as
    // shortest-path smoothing for structural symmetry with roll/yaw; would
    // only matter if pitch's range assumption changes again.)
    float dp = pitch - pitchFiltered;
    if (dp > 180.0f)
        dp -= 360.0f;
    if (dp < -180.0f)
        dp += 360.0f;
    pitchFiltered += alpha * dp;
    while (pitchFiltered > 180.0f)
        pitchFiltered -= 360.0f;
    while (pitchFiltered < -180.0f)
        pitchFiltered += 360.0f;

    // Yaw
    float dy = yaw - yawFiltered;
    if (dy > 180.0f)
        dy -= 360.0f;
    if (dy < -180.0f)
        dy += 360.0f;
    yawFiltered += alpha * dy;
    while (yawFiltered > 180.0f)
        yawFiltered -= 360.0f;
    while (yawFiltered < -180.0f)
        yawFiltered += 360.0f;
}

float MahonyAHRS::invSqrt(float x)
{
    if (x <= 0.0f)
        return 0.0f;

    union
    {
        float f;
        uint32_t i;
    } conv = { x };
    conv.i = 0x5f3759df - (conv.i >> 1);
    float y = conv.f;
    const float halfX = 0.5f * x;
    y = y * (1.5f - halfX * y * y);   // 1st Newton-Raphson iteration
    y = y * (1.5f - halfX * y * y);   // 2nd iteration -- ~0.17% worst-case error -> ~1e-6
    return y;
}

float MahonyAHRS::clamp(float x, float lo, float hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

float MahonyAHRS::lerp(float a, float b, float t)
{
    t = clamp(t, 0.0f, 1.0f);
    return a + (b - a) * t;
}

#endif
