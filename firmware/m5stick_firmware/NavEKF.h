#ifndef NAV_EKF_H
#define NAV_EKF_H

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

// 15-state error-state EKF (p, v, body-frame dtheta, ba[g], bg[deg/s]) with ZUPT.

class NavEKF
{
public:

    NavEKF() { reset(); }
    void reset()
    {
        armed_ = false;
        pNav_[0] = pNav_[1] = pNav_[2] = 0.0f;
        vNav_[0] = vNav_[1] = vNav_[2] = 0.0f;
        qNav0_ = 1.0f; qNav1_ = qNav2_ = qNav3_ = 0.0f;
        baNav_[0] = baNav_[1] = baNav_[2] = 0.0f;
        bgNav_[0] = bgNav_[1] = bgNav_[2] = 0.0f;
        aWorldPrev_[0] = aWorldPrev_[1] = aWorldPrev_[2] = 0.0f;
        gyroMagEMA_ = 0.0f; gyroMagVarEMA_ = 0.0f;
        fLinearBody_[0] = fLinearBody_[1] = fLinearBody_[2] = 0.0f;
        fLinearNorm_ = 0.0f;
        fLinearNormEMA_ = 0.0f; fLinearVarEMA_ = 0.0f;
        ownStationary_ = false;
        ownStationaryTime_ = 0.0f;
        timeSinceZuptSec_ = 0.0f;
        gravityMeasurementValid_ = false;
        gravityUpdateActive_ = false;
        zuptActive_ = false;
        lastGravityMahalanobis_ = -1.0f;
        lastZuptMahalanobis_ = -1.0f;
        lastGravityRDiag_[0] = lastGravityRDiag_[1] = lastGravityRDiag_[2] = 0.0f;
        lastGravitySDiag_[0] = lastGravitySDiag_[1] = lastGravitySDiag_[2] = 0.0f;
        lastGravityAttempted_ = false;
        lastGravityBypass_ = false;
        attitudeRejectStreak_ = 0;
        resetArmAccumulator();

        for (int i = 0; i < 15; i++)
            for (int j = 0; j < 15; j++)
                P_[i][j] = 0.0f;

        accelNoiseDensity_    = 0.00429716f; // g / sqrt(Hz), provisional hardware fit
        gyroNoiseDensity_     = 0.0140f;  // deg/s / sqrt(Hz)
        accelBiasRandomWalk_  = 5.0e-5f;  // g / sqrt(s)
        gyroBiasRandomWalk_   = 0.0050f;  // deg/s / sqrt(s)
        zuptVelNoiseStd_      = 0.05f;    // m/s, 1-sigma ZUPT measurement noise
        attitudeMeasNoiseStd_ = 2.0f * DEG_TO_RAD;  // rad-equivalent vector noise
        attitudeYawVarianceRad2_ = 1.0e4f;   // dimensionless vector variance along gravity
        accelBiasLimitG_      = 0.05f;    // clamp bound, g
        gyroBiasLimitDps_     = 50.0f;    // clamp bound, deg/s
        navGyroMagThresholdDps_   = 3.0f;    // deg/s
        navGyroVarThresholdDps2_  = 3.0f;    // (deg/s)^2
        navFLinearVarThresholdG2_ = 0.0010f; // g^2, variance gate on |fLinearBody|
        // fLinearBody = aBody - R^T*[0,0,g]: gravity-compensated specific-force
        // residual; thresholds below gate on ITS norm, not |aBody|-1g.
        navVectorResidualThresholdG_ = 0.05f; // g, enter stationary threshold
        navVectorResidualExitThresholdG_ = 0.10f; // g, exit stationary threshold
        navDetectorTauSec_        = 0.05f;   // s, responsive detector smoothing
        navGyroMagExitThresholdDps_ = 5.0f;  // hysteretic exit threshold
        accelBiasInitWindowSec_ = 0.3f;   // s of confirmed stationarity accumulated
        zuptMinStationaryTime_ = 0.20f;   // s -- debounce beyond both stationary
        zuptGateBypassTime_    = 1.0f;    // s -- force ZUPT after this much stillness
        // Once ZUPT-starved (isZuptStarved()), every extra sample spent
        // waiting adds more pos/vel uncertainty, so a shorter confirmed-still
        // window is accepted before forcing a correction through. Kept above
        // zuptMinStationaryTime_ so a single-sample flicker can't trigger it.
        zuptGateBypassTimeStarved_ = 0.30f; // s -- starting point, tune on hardware
        zuptGateChiSq_         = 16.0f;   // 3-DoF Mahalanobis gate
        attitudeGateChiSq_     = 16.0f;   // 3-DoF gravity-innovation gate
        attitudeGateBypassStreak_ = 30;   // consecutive rejected samples (~0.5s @ 100Hz)
        gravityLinAccelTrustFloorG_ = 0.04f; // g, fLinearNorm_ below this: no attenuation
        gravityLinAccelTrustCeilG_  = 0.18f; // g, fLinearNorm_ above this: fully distrusted
        lastGravityLinAccelTrust_   = 1.0f;
        blockBiasCorrectionFromGravityUpdate_ = false;   // ablation switch, default off -- see member comment
    }

    void update(
        float gx, float gy, float gz,
        float ax, float ay, float az,
        float dt,
        float mahonyQ0, float mahonyQ1, float mahonyQ2, float mahonyQ3,
        float mahonyBiasGX, float mahonyBiasGY, float mahonyBiasGZ,
        float mahonyAccelConfidence)
    {
        if (dt <= 0.0f)
            return;

        // updateOwnStationaryDetector() runs off the EXTERNAL mahonyQ0..3,
        // never qNav_ -- its output (bothStationary) gates ZUPT and pre-arm
        // accumulation only, never gravityAttitudeUpdate()'s own independent
        // gate. Coupling that to qNav_ would let a bad attitude estimate
        // suppress its own fix (large fLinearBody -> "must be moving" ->
        // distrust the very correction that would fix it).
        //
        // Use Mahony's attitude for this detector so NavEKF cannot suppress
        // the ZUPT correction that would recover a drifting navigation state.
        updateOwnStationaryDetector(gx, gy, gz, ax, ay, az, dt,
                         mahonyQ0, mahonyQ1, mahonyQ2, mahonyQ3);
        bool bothStationary = ownStationary_;
        gravityUpdateActive_ = false;
        zuptActive_ = false;

        if (!armed_)
        {
            if (!bothStationary)
            {
                resetArmAccumulator();
                return;
            }

            accumulateArmSample(ax, ay, az);

            if (ownStationaryTime_ < accelBiasInitWindowSec_)
                return;   // still accumulating; not armed yet

            arm(mahonyQ0, mahonyQ1, mahonyQ2, mahonyQ3,
                mahonyBiasGX, mahonyBiasGY, mahonyBiasGZ);
            return;
        }

        propagate(gx, gy, gz, ax, ay, az, dt, bothStationary);
        timeSinceZuptSec_ += dt;

        lastGravityAttempted_ = false;
        lastGravityBypass_ = false;
        gravityMeasurementValid_ = gravityMeasurementValid(ax, ay, az, mahonyAccelConfidence);
        if (gravityMeasurementValid_)
        {
            gravityUpdateActive_ = gravityAttitudeUpdate(ax, ay, az, mahonyAccelConfidence);
        }

        if (bothStationary && ownStationaryTime_ >= zuptMinStationaryTime_)
        {
            zuptActive_ = zuptUpdate(ownStationaryTime_);
            if (zuptActive_) timeSinceZuptSec_ = 0.0f;
        }

    }

    void getVelocity(float &vx, float &vy, float &vz) const
    {
        vx = vNav_[0]; vy = vNav_[1]; vz = vNav_[2];
    }

    void getPosition(float &px, float &py, float &pz) const
    {
        px = pNav_[0]; py = pNav_[1]; pz = pNav_[2];
    }

    void getAccelBias(float &bx, float &by, float &bz) const
    {
        bx = baNav_[0]; by = baNav_[1]; bz = baNav_[2];
    }

    void getGyroBias(float &bx, float &by, float &bz) const
    {
        bx = bgNav_[0]; by = bgNav_[1]; bz = bgNav_[2];
    }

    void getQuaternion(float &q0, float &q1, float &q2, float &q3) const
    {
        q0 = qNav0_; q1 = qNav1_; q2 = qNav2_; q3 = qNav3_;
    }

    void getAttitude(float &q0, float &q1, float &q2, float &q3) const
    {
        getQuaternion(q0, q1, q2, q3);
    }

    void getPredictedBodyGravity(float &gx, float &gy, float &gz) const
    {
        float R[3][3];
        quatToDCM(qNav0_, qNav1_, qNav2_, qNav3_, R);
        // Body gravity is R^T*[0,0,1], the third row of the body-to-world DCM.
        gx = R[2][0];
        gy = R[2][1];
        gz = R[2][2];
    }

    void getLastAttitudeInnovationRad(float &rx, float &ry, float &rz) const
    {
        rx = lastAttitudeInnovation_[0];
        ry = lastAttitudeInnovation_[1];
        rz = lastAttitudeInnovation_[2];
    }

    void getVelocityStdDev(float &sx, float &sy, float &sz) const
    {
        sx = sqrtf(fmaxf(P_[3][3], 0.0f));
        sy = sqrtf(fmaxf(P_[4][4], 0.0f));
        sz = sqrtf(fmaxf(P_[5][5], 0.0f));
    }

    void getPositionStdDev(float &sx, float &sy, float &sz) const
    {
        sx = sqrtf(fmaxf(P_[0][0], 0.0f));
        sy = sqrtf(fmaxf(P_[1][1], 0.0f));
        sz = sqrtf(fmaxf(P_[2][2], 0.0f));
    }

    bool isArmed() const { return armed_; }
    bool isOwnStationary() const { return ownStationary_; }
    float getStationaryTime() const { return ownStationaryTime_; }
    bool isGravityUpdateActive() const { return gravityUpdateActive_; }
    bool isZuptActive() const { return zuptActive_; }
    void getWorldAcceleration(float &ax, float &ay, float &az) const
    {
        ax = aWorldPrev_[0]; ay = aWorldPrev_[1]; az = aWorldPrev_[2];
    }
    void getOwnDetectorState(float &gyroMagEMA, float &gyroMagVarEMA,
                              float &fLinearNormEMA, float &fLinearVarEMA) const
    {
        gyroMagEMA = gyroMagEMA_; gyroMagVarEMA = gyroMagVarEMA_;
        fLinearNormEMA = fLinearNormEMA_; fLinearVarEMA = fLinearVarEMA_;
    }

    // fLinearBody = aBody - R^T*[0,0,g]: nonzero only for genuine linear
    // acceleration, not attitude change alone. Drives ZUPT gating.
    void getLinearAccelBody(float &fx, float &fy, float &fz) const
    {
        fx = fLinearBody_[0]; fy = fLinearBody_[1]; fz = fLinearBody_[2];
    }
    float getLinearAccelNorm() const { return fLinearNorm_; }

    float getGyroMagEMA() const { return gyroMagEMA_; }
    float getFLinNormEMA() const { return fLinearNormEMA_; }

    // Whether the last sample passed the gravity-dominated sanity check
    // (norm + confidence), independent of whether the correction actually
    // applied (see isGravityUpdateActive()).
    bool isGravityMeasurementValid() const { return gravityMeasurementValid_; }

    // Trust multiplier from the most recent gravityAttitudeUpdate() call
    // (1.0 = no attenuation, 0.0 = fully distrusted). Only meaningful when
    // isGravityMeasurementValid() is true this cycle.
    float getGravityLinAccelTrust() const { return lastGravityLinAccelTrust_; }
    bool isBlockingBiasCorrectionFromGravityUpdate() const { return blockBiasCorrectionFromGravityUpdate_; }
    // Diagnostics-only. Valid whenever zuptUpdate() actually ran (bothStationary
    // && ownStationaryTime_ >= zuptMinStationaryTime_) -- pairs with
    // getStationaryTime()/isZuptActive() to distinguish "not called" from
    // "called and rejected."
    float getZuptMahalanobis() const { return lastZuptMahalanobis_; }

    void getGravityUpdateDiagnostics(float &mahalanobis,
                                     float &r0, float &r1, float &r2,
                                     float &s0, float &s1, float &s2,
                                     bool &attempted, bool &bypass) const
    {
        mahalanobis = lastGravityMahalanobis_;
        r0 = lastGravityRDiag_[0]; r1 = lastGravityRDiag_[1]; r2 = lastGravityRDiag_[2];
        s0 = lastGravitySDiag_[0]; s1 = lastGravitySDiag_[1]; s2 = lastGravitySDiag_[2];
        attempted = lastGravityAttempted_;
        bypass = lastGravityBypass_;
    }

    // Scans the 15 diagonal entries of P. Persistently pinned at P_MAX
    // indicates divergence; at P_MIN, an overconfident/frozen state.
    void getCovarianceDiagRange(float &minDiag, float &maxDiag) const
    {
        minDiag = P_[0][0];
        maxDiag = P_[0][0];
        for (int i = 1; i < 15; i++)
        {
            if (P_[i][i] < minDiag) minDiag = P_[i][i];
            if (P_[i][i] > maxDiag) maxDiag = P_[i][i];
        }
    }

    float getExpectedPositionVarFromNoise() const
    {
        float sigmaA = accelNoiseDensity_ * G_MS2;
        float T = timeSinceZuptSec_;
        return sigmaA * sigmaA * T * T * T / 3.0f;
    }
    float getExpectedVelocityVarFromNoise() const
    {
        // White accel noise (density accelNoiseDensity_, g/sqrt(Hz)) accumulates
        // velocity variance as sigma_a^2*T -- same qv term propagate() injects.
        float sigmaA = accelNoiseDensity_ * G_MS2;
        float T = timeSinceZuptSec_;
        return sigmaA * sigmaA * T;
    }
    float getTimeSinceZupt() const { return timeSinceZuptSec_; }
    bool isZuptStarved() const { return timeSinceZuptSec_ > zuptStarvationThresholdSec_; }
    void setZuptStarvationThreshold(float seconds) { zuptStarvationThresholdSec_ = constrain(seconds, 0.1f, 3600.0f); }

    float getPositionVarianceMax() const
    {
        float m = P_[0][0];
        if (P_[1][1] > m) m = P_[1][1];
        if (P_[2][2] > m) m = P_[2][2];
        return m;
    }

    float getVelocityVarianceMax() const
    {
        float m = P_[3][3];
        if (P_[4][4] > m) m = P_[4][4];
        if (P_[5][5] > m) m = P_[5][5];
        return m;
    }

    void resetPDiagBlowupStats()
    {
        accelBiasClampCount_ = 0;
        gyroBiasClampCount_ = 0;
        gravityInvalidGateCount_ = 0;
        zuptInvalidGateCount_ = 0;
        gravWorstDetS_ = 1.0f;
        zuptBaOvershootCount_ = 0;
        zuptBgOvershootCount_ = 0;
        zuptBaOvershootMax_ = 0.0f;
        zuptBgOvershootMax_ = 0.0f;
    }

    void getBiasClampCounts(uint32_t &accelCount, uint32_t &gyroCount) const
    {
        accelCount = accelBiasClampCount_;
        gyroCount = gyroBiasClampCount_;
    }
    uint32_t getGravityInvalidGateCount() const { return gravityInvalidGateCount_; }
    uint32_t getZuptInvalidGateCount() const { return zuptInvalidGateCount_; }
    float getGravityWorstDetS() const { return gravWorstDetS_; }
    bool isUsingTangentPlaneGravityUpdate() const { return useTangentPlaneGravityUpdate_; }
    void setUseTangentPlaneGravityUpdate(bool enabled) { useTangentPlaneGravityUpdate_ = enabled; }
    uint32_t getZuptBaOvershootCount() const { return zuptBaOvershootCount_; }
    uint32_t getZuptBgOvershootCount() const { return zuptBgOvershootCount_; }
    float getZuptBaOvershootMax() const { return zuptBaOvershootMax_; }
    float getZuptBgOvershootMax() const { return zuptBgOvershootMax_; }

    void setAccelNoiseDensity(float gPerSqrtHz)      { accelNoiseDensity_   = constrain(gPerSqrtHz, 1e-5f, 1.0f); }
    void setGyroNoiseDensity(float dpsPerSqrtHz)      { gyroNoiseDensity_    = constrain(dpsPerSqrtHz, 1e-4f, 50.0f); }
    void setAccelBiasRandomWalk(float gPerSqrtSec)    { accelBiasRandomWalk_ = constrain(gPerSqrtSec, 0.0f, 1.0f); }
    void setGyroBiasRandomWalk(float dpsPerSqrtSec)   { gyroBiasRandomWalk_  = constrain(dpsPerSqrtSec, 0.0f, 50.0f); }
    void setZuptVelocityNoiseStd(float mps)           { zuptVelNoiseStd_     = constrain(mps, 1e-4f, 5.0f); }
    void setZuptMinStationaryTime(float seconds)      { zuptMinStationaryTime_ = constrain(seconds, 0.0f, 5.0f); }
    void setZuptGateBypassTime(float seconds)         { zuptGateBypassTime_  = constrain(seconds, 0.0f, 60.0f); }
    void setZuptGateBypassTimeStarved(float seconds)  { zuptGateBypassTimeStarved_ = constrain(seconds, 0.0f, 60.0f); }
    void setZuptGateChiSquare(float chiSq)            { zuptGateChiSq_       = constrain(chiSq, 0.1f, 1000.0f); }
    void setAttitudeMeasurementNoiseStd(float radians){ attitudeMeasNoiseStd_ = constrain(radians, 1e-4f, 1.0f); }
    void setAttitudeYawVariance(float rad2)           { attitudeYawVarianceRad2_ = constrain(rad2, 1.0f, 1.0e8f); }
    void setAttitudeGateChiSquare(float chiSq)        { attitudeGateChiSq_   = constrain(chiSq, 0.1f, 1000.0f); }
    void setAttitudeGateBypassStreak(int samples)     { attitudeGateBypassStreak_ = samples < 1 ? 1 : samples; }
    void setGravityLinAccelTrustFloor(float g) { gravityLinAccelTrustFloorG_ = constrain(g, 0.0f, 2.0f); }
    void setGravityLinAccelTrustCeil(float g)  { gravityLinAccelTrustCeilG_  = constrain(g, gravityLinAccelTrustFloorG_ + 1e-4f, 2.0f); }
    // Ablation switch, default off -- see member comment on blockBiasCorrectionFromGravityUpdate_.
    void setBlockBiasCorrectionFromGravityUpdate(bool enabled) { blockBiasCorrectionFromGravityUpdate_ = enabled; }
    void setAccelBiasLimit(float g)                   { accelBiasLimitG_     = constrain(g, 0.0f, 2.0f); }
    void setGyroBiasLimit(float dps)                  { gyroBiasLimitDps_    = constrain(dps, 0.0f, 500.0f); }
    void setAccelBiasInitWindow(float seconds)        { accelBiasInitWindowSec_ = constrain(seconds, 0.0f, 5.0f); }
    void setNavStationaryGyroThreshold(float dps)     { navGyroMagThresholdDps_  = constrain(dps, 0.0f, 100.0f); }
    void setNavStationaryGyroExitThreshold(float dps)  { navGyroMagExitThresholdDps_ = constrain(dps, 0.0f, 100.0f); }
    void setNavStationaryGyroVarThreshold(float dps2) { navGyroVarThresholdDps2_ = constrain(dps2, 0.0f, 1000.0f); }
    void setNavStationaryFLinearVarThreshold(float g2) { navFLinearVarThresholdG2_ = constrain(g2, 0.0f, 10.0f); }
    void setNavStationaryVectorResidualThreshold(float g) { navVectorResidualThresholdG_ = constrain(g, 0.0f, 2.0f); }
    void setNavStationaryVectorResidualExitThreshold(float g) { navVectorResidualExitThresholdG_ = constrain(g, 0.0f, 2.0f); }
    void setNavDetectorTimeConstant(float seconds)    { navDetectorTauSec_ = constrain(seconds, 0.005f, 2.0f); }

private:
    bool  armed_;
    float pNav_[3];
    float vNav_[3];
    float qNav0_, qNav1_, qNav2_, qNav3_;   // THE nominal attitude -- see class comment
    float baNav_[3];
    float bgNav_[3];
    float aWorldPrev_[3];   // previous step's world-frame linear accel
    float lastAttitudeInnovation_[3];   // diagnostic, see getLastAttitudeInnovationRad()
    float gyroMagEMA_, gyroMagVarEMA_;
    float fLinearBody_[3];      // aBody - R^T*[0,0,g], body frame, g units
    float fLinearNorm_;
    float fLinearNormEMA_, fLinearVarEMA_;
    bool  ownStationary_;
    float ownStationaryTime_;
    float timeSinceZuptSec_;   // seconds since ZUPT last actually fired -- see getExpectedPositionVarFromNoise()
    float zuptStarvationThresholdSec_ = 5.0f;   // see isZuptStarved()
    int   attitudeRejectStreak_;
    float accelAccum_[3];
    int   accelAccumCount_;
    float P_[15][15];

    // Scratch storage for 15x15 temporaries -- moved from stack locals to
    // member (BSS) storage: at ~900B each, several alive at once in one
    // nested call chain (update -> gravityAttitudeUpdate ->
    // applyCorrectionAndJoseph -> resetAttitudeErrorCovariance) was a real
    // stack-margin risk on the ESP32 Arduino loop task. Bound to local
    // reference-to-array aliases at each use site; see comments at each site
    // for why reuse across functions is safe.
    float scratchA_[15][15];
    float scratchB_[15][15];
    float scratchC_[15][15];
    float scratchD_[15][15];
    float accelNoiseDensity_;
    float gyroNoiseDensity_;
    float accelBiasRandomWalk_;
    float gyroBiasRandomWalk_;
    float zuptVelNoiseStd_;
    float zuptMinStationaryTime_;
    float zuptGateBypassTime_;
    float zuptGateBypassTimeStarved_;
    float zuptGateChiSq_;
    float attitudeMeasNoiseStd_;
    float attitudeYawVarianceRad2_;
    float attitudeGateChiSq_;
    int   attitudeGateBypassStreak_;
    float accelBiasLimitG_;
    float gyroBiasLimitDps_;
    float accelBiasInitWindowSec_;

    float navGyroMagThresholdDps_;
    float navGyroMagExitThresholdDps_;
    float navGyroVarThresholdDps2_;
    float navFLinearVarThresholdG2_;
    float navDetectorTauSec_;
    float navVectorResidualThresholdG_;
    float navVectorResidualExitThresholdG_;

    // fLinearNorm_ (direction-sensitive gravity residual) vs. accelConfidence
    // (Mahony's magnitude-only |a|-1g metric): a lateral/cross-axis
    // acceleration shifts measured gravity DIRECTION substantially (first
    // order) while barely moving |a| (second order only). That gap let a
    // translation-corrupted "gravity" reading pass the confidence gate and
    // feed gravityAttitudeUpdate() at near-full trust, gradually corrupting
    // bias/attitude via gate-passing but systematically wrong corrections.
    // This trust factor is independent of accelConfidence; both must be high
    // for full trust in gravityAttitudeUpdate().
    float gravityLinAccelTrustFloorG_;   // fLinearNorm_ below this: no attenuation
    float gravityLinAccelTrustCeilG_;    // fLinearNorm_ above this: fully distrusted
    float lastGravityLinAccelTrust_;     // last computed factor, [0,1] -- diagnostics only

    // Ablation switch, default off: blocks gravityAttitudeUpdate()'s
    // correction from reaching the accel/gyro bias rows (state indices
    // 9-14), leaving that update to correct attitude only. A single
    // ambiguous gravity vector (no magnetometer, yaw unobservable) has poor
    // joint observability into bias -- a bias offset along an axis is nearly
    // indistinguishable from a small tilt error on that axis. Unlike this,
    // ZUPT's velocity->bias coupling (zuptUpdate() -> applyCorrectionAndJoseph())
    // is direct and well-conditioned (see propagate()'s Phi), and is NOT
    // affected by this switch. Diagnostic/experimental tool for isolating
    // the two coupling paths; leave off otherwise.
    bool  blockBiasCorrectionFromGravityUpdate_;

    bool gravityMeasurementValid_;
    bool gravityUpdateActive_;
    bool zuptActive_;
    float lastGravityMahalanobis_;
    // zuptUpdate()'s own last d2, set unconditionally before the gate
    // decides accept/reject -- mirrors lastGravityMahalanobis_. Pairs with
    // getStationaryTime()/isZuptActive() to distinguish two reasons ZUPT can
    // stay inactive: the stationary detector flickering false (ownStationaryTime_
    // resetting near-zero) vs. the innovation staying above zuptGateChiSq_
    // even past zuptGateBypassTimeStarved_ (gate declining a large correction).
    float lastZuptMahalanobis_;
    float lastGravityRDiag_[3];
    float lastGravitySDiag_[3];
    bool  lastGravityAttempted_;
    bool  lastGravityBypass_;

    uint32_t accelBiasClampCount_ = 0;   // how often a correction actually hit the clamp
    uint32_t gyroBiasClampCount_ = 0;

    uint32_t gravityInvalidGateCount_ = 0;  // count of numerically invalid gravity gates (neg/NaN/Inf d2 or near-singular S)
    uint32_t zuptInvalidGateCount_ = 0;     // same, for the ZUPT gate

    float gravWorstDetS_ = 1.0f;  // smallest |det(S)| seen since last reset -- proximity to singular
    // Selects the gravity-attitude update's residual formulation. The 2D
    // tangent-plane path (default) avoids the artificial R-inflation the 3D
    // vector-difference path needs to suppress its unobservable yaw
    // component, which otherwise leaves S poorly conditioned. The 3D path
    // remains available via setUseTangentPlaneGravityUpdate(false) /
    // TANGENT_GRAVITY_OFF, for comparison or fallback.
    bool  useTangentPlaneGravityUpdate_ = true;

    uint32_t zuptBaOvershootCount_ = 0;
    uint32_t zuptBgOvershootCount_ = 0;
    float zuptBaOvershootMax_ = 0.0f;
    float zuptBgOvershootMax_ = 0.0f;

    void resetArmAccumulator()
    {
        accelAccum_[0] = accelAccum_[1] = accelAccum_[2] = 0.0f;
        accelAccumCount_ = 0;
    }

    void accumulateArmSample(float ax, float ay, float az)
    {
        accelAccum_[0] += ax; accelAccum_[1] += ay; accelAccum_[2] += az;
        accelAccumCount_++;
    }

    // Gravity-compensated specific-force motion/ZUPT test.
    //   gBodyPred = R^T*[0,0,g], fLinearBody = aBody - gBodyPred
    // A full VECTOR difference, not a scalar |a|-1g deviation and not a
    // gravity-axis projection -- both alternatives are blind to different
    // failure modes: |a|-1g misses a translation that rotates the measured
    // vector at constant norm (e.g. a swing trading along-gravity for
    // cross-axis component); projecting out the along-gravity component
    // misses linear acceleration ALONG gravity (e.g. a vertical bob), since
    // that component gets silently attributed to gravity and subtracted
    // away. fLinearBody catches both: whatever aBody has that gBodyPred
    // doesn't predict is genuine specific force.
    //
    // Pure rotation: gBodyPred tracks the measured gravity vector, so
    // fLinearBody stays near zero even as aBody changes. Pure translation:
    // gBodyPred doesn't move, so any linear acceleration shows up directly.
    //
    // Gyro magnitude is an INDEPENDENT criterion: rotation about the gravity
    // axis itself (e.g. a yaw spin while upright) leaves aBody essentially
    // unchanged -- the accelerometer structurally can't see it. Gyro
    // magnitude is the only signal that catches that case.
    //
    // Both criteria use entry/exit hysteresis + EMA-variance gating (no
    // single fragile instantaneous threshold).
    void updateOwnStationaryDetector(float gxDeg, float gyDeg, float gzDeg,
                                      float axG, float ayG, float azG,
                                      float dt,
                                      float q0, float q1, float q2, float q3)
    {
        float gyroMag = sqrtf(gxDeg * gxDeg + gyDeg * gyDeg + gzDeg * gzDeg);

        float R[3][3];
        quatToDCM(q0, q1, q2, q3, R);
        // Body-frame gravity direction = THIRD ROW of the body-to-world DCM
        // (R[2][0..2]), not the third column -- see getPredictedBodyGravity().
        float gBodyPredX = R[2][0], gBodyPredY = R[2][1], gBodyPredZ = R[2][2];

        fLinearBody_[0] = axG - gBodyPredX;
        fLinearBody_[1] = ayG - gBodyPredY;
        fLinearBody_[2] = azG - gBodyPredZ;
        fLinearNorm_ = sqrtf(fLinearBody_[0] * fLinearBody_[0] +
                              fLinearBody_[1] * fLinearBody_[1] +
                              fLinearBody_[2] * fLinearBody_[2]);

        float alpha = dt / (navDetectorTauSec_ + dt);

        float dG = gyroMag - gyroMagEMA_;
        gyroMagEMA_ += alpha * dG;
        gyroMagVarEMA_ += alpha * (dG * dG - gyroMagVarEMA_);

        float dF = fLinearNorm_ - fLinearNormEMA_;
        fLinearNormEMA_ += alpha * dF;
        fLinearVarEMA_ += alpha * (dF * dF - fLinearVarEMA_);

        bool magnitudeStill = ownStationary_
                        ? (gyroMagEMA_ < navGyroMagExitThresholdDps_) &&
                            (fLinearNormEMA_ < navVectorResidualExitThresholdG_)
                        : (gyroMagEMA_ < navGyroMagThresholdDps_) &&
                            (fLinearNormEMA_ < navVectorResidualThresholdG_);

        ownStationary_ = magnitudeStill &&
            (gyroMagVarEMA_ < navGyroVarThresholdDps2_) &&
            (fLinearVarEMA_ < navFLinearVarThresholdG2_);

        if (ownStationary_)
            ownStationaryTime_ += dt;
        else
            ownStationaryTime_ = 0.0f;
    }

    void arm(float mahonyQ0, float mahonyQ1, float mahonyQ2, float mahonyQ3,
             float mahonyBgx, float mahonyBgy, float mahonyBgz)
    {
        float n = sqrtf(mahonyQ0 * mahonyQ0 + mahonyQ1 * mahonyQ1 +
                         mahonyQ2 * mahonyQ2 + mahonyQ3 * mahonyQ3);

        if (n < 1e-9f)
        {
            qNav0_ = 1.0f; qNav1_ = qNav2_ = qNav3_ = 0.0f;
        }
        else
        {
            qNav0_ = mahonyQ0 / n; qNav1_ = mahonyQ1 / n;
            qNav2_ = mahonyQ2 / n; qNav3_ = mahonyQ3 / n;
        }

        pNav_[0] = pNav_[1] = pNav_[2] = 0.0f;
        vNav_[0] = vNav_[1] = vNav_[2] = 0.0f;

        // ACCEL_BIAS already removed by firmware. Don't use Mahony's attitude
        // to init the residual bias: tilt error would be absorbed as a false
        // body-frame acceleration bias.
        baNav_[0] = baNav_[1] = baNav_[2] = 0.0f;
        const float sigmaBa0 = 0.02f;   // residual bias prior, g

        bgNav_[0] = mahonyBgx; bgNav_[1] = mahonyBgy; bgNav_[2] = mahonyBgz;

        {
            float Rarm[3][3];
            quatToDCM(qNav0_, qNav1_, qNav2_, qNav3_, Rarm);
            float fx = -baNav_[0], fy = -baNav_[1], fz = -baNav_[2];
            if (accelAccumCount_ > 0)
            {
                fx += accelAccum_[0] / (float)accelAccumCount_;
                fy += accelAccum_[1] / (float)accelAccumCount_;
                fz += accelAccum_[2] / (float)accelAccumCount_;
            }
            float fBodyMs2[3] = { fx * G_MS2, fy * G_MS2, fz * G_MS2 };
            aWorldPrev_[0] = Rarm[0][0] * fBodyMs2[0] + Rarm[0][1] * fBodyMs2[1] + Rarm[0][2] * fBodyMs2[2];
            aWorldPrev_[1] = Rarm[1][0] * fBodyMs2[0] + Rarm[1][1] * fBodyMs2[1] + Rarm[1][2] * fBodyMs2[2];
            aWorldPrev_[2] = Rarm[2][0] * fBodyMs2[0] + Rarm[2][1] * fBodyMs2[1] + Rarm[2][2] * fBodyMs2[2] - G_MS2;
        }
        lastAttitudeInnovation_[0] = lastAttitudeInnovation_[1] = lastAttitudeInnovation_[2] = 0.0f;
        initCovariance(sigmaBa0);
        resetArmAccumulator();
        armed_ = true;
    }

    void initCovariance(float sigmaBa0)
    {
        for (int i = 0; i < 15; i++)
            for (int j = 0; j < 15; j++)
                P_[i][j] = 0.0f;

        const float sigmaP0 = 0.001f;   // m
        const float sigmaV0 = 0.01f;    // m/s
        const float sigmaTheta0 = 2.0f * DEG_TO_RAD;   // rad

        const float sigmaBg0 = 0.10f;   // deg/s, residual after startup calibration

        for (int i = 0; i < 3; i++) P_[i][i]         = sigmaP0 * sigmaP0;
        for (int i = 3; i < 6; i++) P_[i][i]         = sigmaV0 * sigmaV0;
        for (int i = 6; i < 9; i++) P_[i][i]         = sigmaTheta0 * sigmaTheta0;
        for (int i = 9; i < 12; i++) P_[i][i]        = sigmaBa0 * sigmaBa0;
        for (int i = 12; i < 15; i++) P_[i][i]       = sigmaBg0 * sigmaBg0;
    }

    //------------------------------------------------------------------
    // Nominal-state propagation + error-covariance propagation.
    //------------------------------------------------------------------

    void propagate(float gxDeg, float gyDeg, float gzDeg,
                    float axG, float ayG, float azG,
                    float dt, bool stationary)
    {

        float wx = (gxDeg - bgNav_[0]) * DEG_TO_RAD;
        float wy = (gyDeg - bgNav_[1]) * DEG_TO_RAD;
        float wz = (gzDeg - bgNav_[2]) * DEG_TO_RAD;

        float fx = (axG - baNav_[0]);   // g, body frame, NOT normalized
        float fy = (ayG - baNav_[1]);  
        float fz = (azG - baNav_[2]);

        float fBodyMs2[3] = { fx * G_MS2, fy * G_MS2, fz * G_MS2 };

        float wNorm = sqrtf(wx * wx + wy * wy + wz * wz);
        float dq0, dq1, dq2, dq3;

        if (wNorm > 1e-8f)
        {
            float halfAngle = 0.5f * wNorm * dt;
            float s = sinf(halfAngle) / wNorm;
            dq0 = cosf(halfAngle);
            dq1 = wx * s;
            dq2 = wy * s;
            dq3 = wz * s;
        }
        else
        {
            dq0 = 1.0f;
            dq1 = 0.5f * wx * dt;
            dq2 = 0.5f * wy * dt;
            dq3 = 0.5f * wz * dt;
        }

        float qNew0, qNew1, qNew2, qNew3;
        quatMultiply(qNav0_, qNav1_, qNav2_, qNav3_,
                     dq0, dq1, dq2, dq3,
                     qNew0, qNew1, qNew2, qNew3);
        quatNormalize(qNew0, qNew1, qNew2, qNew3);

        qNav0_ = qNew0; qNav1_ = qNew1; qNav2_ = qNew2; qNav3_ = qNew3;


        float R[3][3];
        quatToDCM(qNav0_, qNav1_, qNav2_, qNav3_, R);

        float aWorld[3];
        aWorld[0] = R[0][0] * fBodyMs2[0] + R[0][1] * fBodyMs2[1] + R[0][2] * fBodyMs2[2];
        aWorld[1] = R[1][0] * fBodyMs2[0] + R[1][1] * fBodyMs2[1] + R[1][2] * fBodyMs2[2];
        aWorld[2] = R[2][0] * fBodyMs2[0] + R[2][1] * fBodyMs2[1] + R[2][2] * fBodyMs2[2] - G_MS2;

        float vNew[3];
        vNew[0] = vNav_[0] + 0.5f * (aWorldPrev_[0] + aWorld[0]) * dt;
        vNew[1] = vNav_[1] + 0.5f * (aWorldPrev_[1] + aWorld[1]) * dt;
        vNew[2] = vNav_[2] + 0.5f * (aWorldPrev_[2] + aWorld[2]) * dt;

        pNav_[0] += 0.5f * (vNav_[0] + vNew[0]) * dt;
        pNav_[1] += 0.5f * (vNav_[1] + vNew[1]) * dt;
        pNav_[2] += 0.5f * (vNav_[2] + vNew[2]) * dt;

        vNav_[0] = vNew[0]; vNav_[1] = vNew[1]; vNav_[2] = vNew[2];

        aWorldPrev_[0] = aWorld[0];
        aWorldPrev_[1] = aWorld[1];
        aWorldPrev_[2] = aWorld[2];


        float skewF[3][3], skewW[3][3];
        skew3(fBodyMs2, skewF);
        skew3Vec(wx, wy, wz, skewW);

        // scratchA_/B_/C_ safe here -- propagate() never nests with
        // applyCorrectionAndJoseph()/resetAttitudeErrorCovariance() (update()
        // calls them sequentially, not from within propagate()).
        float (&Phi)[15][15] = scratchA_;
        for (int i = 0; i < 15; i++)
            for (int j = 0; j < 15; j++)
                Phi[i][j] = (i == j) ? 1.0f : 0.0f;

        for (int i = 0; i < 3; i++)
            Phi[0 + i][3 + i] = dt;

        float RskewF[3][3];
        mat3Mul(R, skewF, RskewF);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                Phi[3 + i][6 + j] = -RskewF[i][j] * dt;

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                Phi[3 + i][9 + j] = -G_MS2 * R[i][j] * dt;

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                Phi[6 + i][6 + j] = (i == j ? 1.0f : 0.0f) - skewW[i][j] * dt;

        for (int i = 0; i < 3; i++)
            Phi[6 + i][12 + i] = -DEG_TO_RAD * dt;


        float sa2 = (accelNoiseDensity_ * G_MS2) * (accelNoiseDensity_ * G_MS2);
        float qv  = sa2 * dt;
        float qp  = sa2 * dt * dt * dt / 3.0f;
        float qpv = sa2 * dt * dt / 2.0f;
        float qth = (gyroNoiseDensity_ * DEG_TO_RAD) * (gyroNoiseDensity_ * DEG_TO_RAD) * dt;
        (void)stationary;
        float qba = accelBiasRandomWalk_ * accelBiasRandomWalk_ * dt;
        float qbg = gyroBiasRandomWalk_ * gyroBiasRandomWalk_ * dt;

        float (&PhiP)[15][15] = scratchB_;
        matMulAB(Phi, P_, PhiP);

        float (&PhiPPhiT)[15][15] = scratchC_;
        matMulABt(PhiP, Phi, PhiPPhiT);

        for (int i = 0; i < 15; i++)
            for (int j = 0; j < 15; j++)
                P_[i][j] = PhiPPhiT[i][j];

        for (int i = 0; i < 3; i++)
        {
            P_[i][i]         += qp;
            P_[3 + i][3 + i] += qv;
            P_[i][3 + i]     += qpv;
            P_[3 + i][i]     += qpv;
        }
        for (int i = 6; i < 9; i++)   P_[i][i] += qth;
        for (int i = 9; i < 12; i++)  P_[i][i] += qba;
        for (int i = 12; i < 15; i++) P_[i][i] += qbg;

        symmetrizeAndClampP();
    }

    bool gravityMeasurementValid(float axG, float ayG, float azG,
                                 float accelConfidence) const
    {
        float norm = sqrtf(axG * axG + ayG * ayG + azG * azG);
        return accelConfidence > 0.05f && norm >= 0.5f && norm <= 1.5f;
    }

    bool gravityAttitudeUpdate(float axG, float ayG, float azG,
                               float accelConfidence)
    {
        lastGravityAttempted_ = true;
        float fx = axG - baNav_[0];
        float fy = ayG - baNav_[1];
        float fz = azG - baNav_[2];
        float n = sqrtf(fx * fx + fy * fy + fz * fz);
        if (n < 0.5f || n > 1.5f)
            return false;

        float ax = fx / n, ay = fy / n, az = fz / n;

        float Rnav[3][3];
        quatToDCM(qNav0_, qNav1_, qNav2_, qNav3_, Rnav);
        // body gravity = R^T*[0,0,1] = THIRD ROW of the body-to-world DCM
        // The third column represents world +Z in world coordinates and is
        // not the gravity vector expressed in the body frame.
        float gx = Rnav[2][0], gy = Rnav[2][1], gz = Rnav[2][2];

        // For R_true = R_nom Exp(dtheta), a_hat - g_hat = [g_hat]x dtheta.
        float y[3] = {
            ax - gx,
            ay - gy,
            az - gz
        };

        float H[3][15] = {};
        H[0][7] = -gz; H[0][8] =  gy;
        H[1][6] =  gz; H[1][8] = -gx;
        H[2][6] = -gy; H[2][7] =  gx;

        lastAttitudeInnovation_[0] = y[0];
        lastAttitudeInnovation_[1] = y[1];
        lastAttitudeInnovation_[2] = y[2];

        // fLinearNorm_ already computed this cycle by
        // updateOwnStationaryDetector() (called before this in update()).
        // Smoothstep to zero trust between floor and ceiling -- same
        // Hermite-smoothstep idiom MahonyAHRS.h uses for accelConfidence.
        float linTrust;
        if (fLinearNorm_ <= gravityLinAccelTrustFloorG_)
        {
            linTrust = 1.0f;
        }
        else if (fLinearNorm_ >= gravityLinAccelTrustCeilG_)
        {
            linTrust = 0.0f;
        }
        else
        {
            float t = (fLinearNorm_ - gravityLinAccelTrustFloorG_) /
                      (gravityLinAccelTrustCeilG_ - gravityLinAccelTrustFloorG_);
            linTrust = 1.0f - t * t * (3.0f - 2.0f * t);
        }
        lastGravityLinAccelTrust_ = linTrust;

        // Both accelConfidence (magnitude) and linTrust (direction) must be
        // high for full trust. Keep the lower bound positive so
        // vectorNoiseStd remains finite.
        float confidence = constrain(accelConfidence * linTrust, 0.0025f, 1.0f);
        float vectorNoiseStd = sinf(attitudeMeasNoiseStd_) / sqrtf(confidence);
        float s2 = vectorNoiseStd * vectorNoiseStd;

        if (useTangentPlaneGravityUpdate_)
        {
            // 2D tangent-plane gravity update.
            //
            // The 3D formulation (below, when this flag is off) inflates R
            // with attitudeYawVarianceRad2_ (1e4) along gBodyPred to make the
            // yaw-null dimension harmless. That rank-one term is what spans
            // S's eigenvalues across many orders of magnitude, occasionally
            // producing a negative Mahalanobis distance from a fragile
            // float32 3x3 inversion (see gravityInvalidGateCount_). A 2D
            // tangent-plane residual has no yaw dimension at all -- yaw is
            // structurally unobservable from a single gravity vector without
            // a magnetometer -- so no R-inflation is needed and S stays
            // well-conditioned by construction.
            //
            // Tangent-plane basis: two orthonormal vectors e1, e2 spanning
            // the plane perpendicular to gBodyPred, built by Gram-Schmidt
            // from a reference vector chosen to avoid the near-parallel
            // degenerate case. Varies smoothly with attitude, no
            // discontinuous axis-swap logic.
            float gLen = sqrtf(gx * gx + gy * gy + gz * gz);
            if (gLen < 1e-6f)
                return false;
            float gnx = gx / gLen, gny = gy / gLen, gnz = gz / gLen;

            float ref[3] = {0.0f, 0.0f, 1.0f};
            if (fabsf(gnz) > 0.9f)
                ref[0] = 1.0f, ref[1] = 0.0f, ref[2] = 0.0f;

            float e1x = gny * ref[2] - gnz * ref[1];
            float e1y = gnz * ref[0] - gnx * ref[2];
            float e1z = gnx * ref[1] - gny * ref[0];
            float e1Len = sqrtf(e1x * e1x + e1y * e1y + e1z * e1z);
            if (e1Len < 1e-6f)
                return false;
            e1x /= e1Len; e1y /= e1Len; e1z /= e1Len;

            float e2x = gny * e1z - gnz * e1y;
            float e2y = gnz * e1x - gnx * e1z;
            float e2z = gnx * e1y - gny * e1x;

            float y3[3] = { ax - gnx, ay - gny, az - gnz };
            float r[2] = { y3[0] * e1x + y3[1] * e1y + y3[2] * e1z,
                           y3[0] * e2x + y3[1] * e2y + y3[2] * e2z };

            float H2[2][15] = {};
            // H = B * [gBodyPred]x where B = [e1^T; e2^T]
            H2[0][6] =  e1y * gnz - e1z * gny;
            H2[0][7] =  e1z * gnx - e1x * gnz;
            H2[0][8] =  e1x * gny - e1y * gnx;
            H2[1][6] =  e2y * gnz - e2z * gny;
            H2[1][7] =  e2z * gnx - e2x * gnz;
            H2[1][8] =  e2x * gny - e2y * gnx;

            lastAttitudeInnovation_[0] = r[0];
            lastAttitudeInnovation_[1] = r[1];
            lastAttitudeInnovation_[2] = 0.0f;

            // R_2x2 = B * R_3D * B^T. Because R_3D = s2*I + attitudeYawVarianceRad2_*(g⊗g)
            // and e1,e2 are perpendicular to g, the yaw-variance rank-1 term
            // projects to zero, leaving R = s2*I_2. Well-conditioned by construction.
            float R2[2][2] = {{s2, 0.0f}, {0.0f, s2}};

            float S2[2][2];
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 2; j++)
                {
                    S2[i][j] = R2[i][j];
                    for (int k = 0; k < 3; k++)
                        for (int l = 0; l < 3; l++)
                            S2[i][j] += H2[i][6 + k] * P_[6 + k][6 + l] * H2[j][6 + l];
                }

            // Populated unconditionally, mirroring the 3D path below, so
            // getGravityUpdateDiagnostics() reads consistently regardless of
            // which path is active. Index 2 is an explicit 0.0f sentinel:
            // no third (yaw) dimension exists in a 2D residual.
            lastGravityRDiag_[0] = R2[0][0];
            lastGravityRDiag_[1] = R2[1][1];
            lastGravityRDiag_[2] = 0.0f;
            lastGravitySDiag_[0] = S2[0][0];
            lastGravitySDiag_[1] = S2[1][1];
            lastGravitySDiag_[2] = 0.0f;

            float detS2 = S2[0][0] * S2[1][1] - S2[0][1] * S2[1][0];
            if (fabsf(detS2) < gravWorstDetS_)
                gravWorstDetS_ = fabsf(detS2);

            if (fabsf(detS2) < 1e-10f || !isfinite(detS2))
            {
                gravityInvalidGateCount_++;
                lastGravityAttempted_ = true;
                lastGravityBypass_ = false;
                attitudeRejectStreak_++;
                return false;
            }

            float Sinv2[2][2];
            if (!invert2x2(S2, Sinv2))
                return false;

            float d2 = mahalanobis2(r, Sinv2);
            lastGravityMahalanobis_ = d2;
            if (!isfinite(d2) || d2 < 0.0f)
            {
                gravityInvalidGateCount_++;
                lastGravityAttempted_ = true;
                lastGravityBypass_ = false;
                attitudeRejectStreak_++;
                return false;
            }
            lastGravityBypass_ = d2 > attitudeGateChiSq_ &&
                         attitudeRejectStreak_ >= attitudeGateBypassStreak_;
            if (d2 > attitudeGateChiSq_ && attitudeRejectStreak_ < attitudeGateBypassStreak_)
            {
                attitudeRejectStreak_++;
                return false;
            }
            attitudeRejectStreak_ = 0;

            float K2[15][2];
            for (int i = 0; i < 15; i++)
                for (int j = 0; j < 2; j++)
                {
                    float sum = 0.0f;
                    for (int k = 0; k < 3; k++)
                        for (int l = 0; l < 2; l++)
                            sum += P_[i][6 + k] * H2[l][6 + k] * Sinv2[l][j];
                    K2[i][j] = sum;
                }

            if (blockBiasCorrectionFromGravityUpdate_)
            {
                for (int i = 9; i < 15; i++)
                    for (int j = 0; j < 2; j++)
                        K2[i][j] = 0.0f;
            }

            float dx2[15];
            for (int i = 0; i < 15; i++)
            {
                float sum = 0.0f;
                for (int j = 0; j < 2; j++)
                    sum += K2[i][j] * r[j];
                dx2[i] = sum;
            }

            applyCorrectionAndJoseph2D(dx2, K2, H2, R2);
            return true;
        }

        // Existing 3D path (unchanged when useTangentPlaneGravityUpdate_ is false)
        float Rbody[3][3];
        for (int i = 0; i < 3; i++)
        {
            float gi = (i == 0) ? gx : (i == 1) ? gy : gz;
            for (int j = 0; j < 3; j++)
            {
                float gj = (j == 0) ? gx : (j == 1) ? gy : gz;
                Rbody[i][j] = ((i == j) ? s2 : 0.0f) + attitudeYawVarianceRad2_ * gi * gj;
            }

            lastGravityRDiag_[0] = Rbody[0][0];
            lastGravityRDiag_[1] = Rbody[1][1];
            lastGravityRDiag_[2] = Rbody[2][2];
        }

        float S[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                S[i][j] = Rbody[i][j];
                for (int k = 0; k < 3; k++)
                    for (int l = 0; l < 3; l++)
                        S[i][j] += H[i][6 + k] * P_[6 + k][6 + l] * H[j][6 + l];
            }

                lastGravitySDiag_[0] = S[0][0];
                lastGravitySDiag_[1] = S[1][1];
                lastGravitySDiag_[2] = S[2][2];

        // Tracks the worst (smallest-magnitude) |det(S)| since the last
        // DIAG_RESET_MAX -- how close this update's S has come to singular,
        // independent of whether any sample was actually rejected.
        float detS = S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1])
                   - S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0])
                   + S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);
        if (fabsf(detS) < gravWorstDetS_)
            gravWorstDetS_ = fabsf(detS);

        // A valid Mahalanobis distance from PD S is never negative. An
        // ill-conditioned S can invert to a non-PD matrix, producing a
        // negative/non-finite d2 that would pass an upper-bound-only gate
        // regardless of magnitude -- reject that and near-singular S here.
        if (fabsf(detS) < 1e-10f || !isfinite(detS))
        {
            gravityInvalidGateCount_++;
            lastGravityAttempted_ = true;
            lastGravityBypass_ = false;
            attitudeRejectStreak_++;
            return false;
        }

        float Sinv[3][3];
        if (!invert3x3(S, Sinv))
            return false;

        float d2 = mahalanobis3(y, Sinv);
        lastGravityMahalanobis_ = d2;
        if (!isfinite(d2) || d2 < 0.0f)
        {
            gravityInvalidGateCount_++;
            lastGravityAttempted_ = true;
            lastGravityBypass_ = false;
            attitudeRejectStreak_++;
            return false;
        }
        lastGravityBypass_ = d2 > attitudeGateChiSq_ &&
                     attitudeRejectStreak_ >= attitudeGateBypassStreak_;
        if (d2 > attitudeGateChiSq_ && attitudeRejectStreak_ < attitudeGateBypassStreak_)
        {
            attitudeRejectStreak_++;
            return false;
        }
        attitudeRejectStreak_ = 0;

        float K[15][3];
        for (int i = 0; i < 15; i++)
            for (int j = 0; j < 3; j++)
            {
                float sum = 0.0f;
                for (int k = 0; k < 3; k++)
                    for (int l = 0; l < 3; l++)
                        sum += P_[i][6 + k] * H[l][6 + k] * Sinv[l][j];
                K[i][j] = sum;
            }

        // Ablation switch, default off -- see member comment on
        // blockBiasCorrectionFromGravityUpdate_. Zeroing K's bias rows means
        // dx[9..14] comes out zero and, in the Joseph update, IKH's bias
        // rows reduce to identity and KRKt's bias rows/columns to zero for
        // this call -- leaving P's bias rows exactly as propagate() left
        // them. zuptUpdate() computes its own K and is unaffected.
        if (blockBiasCorrectionFromGravityUpdate_)
        {
            for (int i = 9; i < 15; i++)
                for (int j = 0; j < 3; j++)
                    K[i][j] = 0.0f;
        }

        float dx[15];
        for (int i = 0; i < 15; i++)
        {
            float sum = 0.0f;
            for (int j = 0; j < 3; j++)
                sum += K[i][j] * y[j];
            dx[i] = sum;
        }

        applyCorrectionAndJoseph(dx, K, H, Rbody);
        return true;
    }

    bool zuptUpdate(float stationaryTime)
    {
        float S[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                S[i][j] = P_[3 + i][3 + j] + ((i == j) ? (zuptVelNoiseStd_ * zuptVelNoiseStd_) : 0.0f);

        // Same numerical-robustness defense as gravityAttitudeUpdate(): a
        // valid Mahalanobis distance can't be negative/non-finite; reject
        // near-singular S or a garbage d2 before an upper-bound gate check
        // could let it through unchecked.
        float detS = S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1])
                   - S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0])
                   + S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);
        if (fabsf(detS) < 1e-10f || !isfinite(detS))
        {
            zuptInvalidGateCount_++;
            return false;
        }

        float Sinv[3][3];
        if (!invert3x3(S, Sinv))
            return false;

        float y[3] = { -vNav_[0], -vNav_[1], -vNav_[2] };

        float d2 = mahalanobis3(y, Sinv);
        lastZuptMahalanobis_ = d2;
        if (!isfinite(d2) || d2 < 0.0f)
        {
            zuptInvalidGateCount_++;
            return false;
        }
        // Shortens the confirmation window once already ZUPT-starved -- see
        // isZuptStarved() and zuptGateBypassTimeStarved_ member comment.
        float effectiveBypassTime = isZuptStarved() ? zuptGateBypassTimeStarved_
                                                     : zuptGateBypassTime_;
        if (d2 > zuptGateChiSq_ && stationaryTime < effectiveBypassTime)
            return false;

        float K[15][3];
        for (int i = 0; i < 15; i++)
            for (int j = 0; j < 3; j++)
            {
                float sum = 0.0f;
                for (int k = 0; k < 3; k++)
                    sum += P_[i][3 + k] * Sinv[k][j];
                K[i][j] = sum;
            }

        float dx[15];
        for (int i = 0; i < 15; i++)
        {
            float sum = 0.0f;
            for (int j = 0; j < 3; j++)
                sum += K[i][j] * y[j];
            dx[i] = sum;
        }

        // Tracks how far the unclamped correction overshoots the bias
        // limits, before the clamp below. Distinguishes true bias magnitude
        // exceeding the bound (large, consistent overshoot) from chattering
        // at the boundary (small, oscillating overshoot).
        {
            float abOvershoot[3] = {
                fabsf(baNav_[0] + dx[9]) - accelBiasLimitG_,
                fabsf(baNav_[1] + dx[10]) - accelBiasLimitG_,
                fabsf(baNav_[2] + dx[11]) - accelBiasLimitG_
            };
            float bgOvershoot[3] = {
                fabsf(bgNav_[0] + dx[12]) - gyroBiasLimitDps_,
                fabsf(bgNav_[1] + dx[13]) - gyroBiasLimitDps_,
                fabsf(bgNav_[2] + dx[14]) - gyroBiasLimitDps_
            };
            for (int i = 0; i < 3; i++)
            {
                if (abOvershoot[i] > zuptBaOvershootMax_) zuptBaOvershootMax_ = abOvershoot[i];
                if (bgOvershoot[i] > zuptBgOvershootMax_) zuptBgOvershootMax_ = bgOvershoot[i];
                if (abOvershoot[i] > 0.0f) zuptBaOvershootCount_++;
                if (bgOvershoot[i] > 0.0f) zuptBgOvershootCount_++;
            }
        }

        float Rzupt[3][3] = {
            { zuptVelNoiseStd_ * zuptVelNoiseStd_, 0.0f, 0.0f },
            { 0.0f, zuptVelNoiseStd_ * zuptVelNoiseStd_, 0.0f },
            { 0.0f, 0.0f, zuptVelNoiseStd_ * zuptVelNoiseStd_ }
        };
        float H[3][15] = {};
        H[0][3] = 1.0f;
        H[1][4] = 1.0f;
        H[2][5] = 1.0f;
        applyCorrectionAndJoseph(dx, K, H, Rzupt);
        return true;
    }

    void applyCorrectionAndJoseph(const float dx[15], const float K[15][3],
                                  const float H[3][15], const float R[3][3])
    {
        pNav_[0] += dx[0]; pNav_[1] += dx[1]; pNav_[2] += dx[2];
        vNav_[0] += dx[3]; vNav_[1] += dx[4]; vNav_[2] += dx[5];

        float dq0 = 1.0f, dq1 = 0.5f * dx[6], dq2 = 0.5f * dx[7], dq3 = 0.5f * dx[8];
        float qc0, qc1, qc2, qc3;
        quatMultiply(qNav0_, qNav1_, qNav2_, qNav3_, dq0, dq1, dq2, dq3,
                     qc0, qc1, qc2, qc3);
        quatNormalize(qc0, qc1, qc2, qc3);
        qNav0_ = qc0; qNav1_ = qc1; qNav2_ = qc2; qNav3_ = qc3;

        float baxRaw = baNav_[0] + dx[9], bayRaw = baNav_[1] + dx[10], bazRaw = baNav_[2] + dx[11];
        baNav_[0] = constrain(baxRaw, -accelBiasLimitG_, accelBiasLimitG_);
        baNav_[1] = constrain(bayRaw, -accelBiasLimitG_, accelBiasLimitG_);
        baNav_[2] = constrain(bazRaw, -accelBiasLimitG_, accelBiasLimitG_);
        if (baNav_[0] != baxRaw || baNav_[1] != bayRaw || baNav_[2] != bazRaw)
            accelBiasClampCount_++;

        float bgxRaw = bgNav_[0] + dx[12], bgyRaw = bgNav_[1] + dx[13], bgzRaw = bgNav_[2] + dx[14];
        bgNav_[0] = constrain(bgxRaw, -gyroBiasLimitDps_, gyroBiasLimitDps_);
        bgNav_[1] = constrain(bgyRaw, -gyroBiasLimitDps_, gyroBiasLimitDps_);
        bgNav_[2] = constrain(bgzRaw, -gyroBiasLimitDps_, gyroBiasLimitDps_);
        if (bgNav_[0] != bgxRaw || bgNav_[1] != bgyRaw || bgNav_[2] != bgzRaw)
            gyroBiasClampCount_++;

        for (int i = 0; i < 3; i++)
        {
            if (!isfinite(vNav_[i])) vNav_[i] = 0.0f;
            if (!isfinite(pNav_[i])) pNav_[i] = 0.0f;
        }

        // scratchA_/B_/C_/D_ safe here -- propagate() has already returned by
        // the time update() calls gravityAttitudeUpdate()/zuptUpdate(), so
        // its Phi/PhiP/PhiPPhiT are dead. The nested resetAttitudeErrorCovariance()
        // call below reuses scratchA_/B_ only AFTER IKH/IKH_P are fully
        // consumed into P_ a few lines down.
        float (&IKH)[15][15] = scratchA_;
        for (int i = 0; i < 15; i++)
        {
            for (int j = 0; j < 15; j++)
            {
                float khij = 0.0f;
                for (int k = 0; k < 3; k++)
                    khij += K[i][k] * H[k][j];
                IKH[i][j] = (i == j ? 1.0f : 0.0f) - khij;
            }
        }

        float (&IKH_P)[15][15] = scratchB_;
        matMulAB(IKH, P_, IKH_P);

        float (&IKH_P_IKHt)[15][15] = scratchC_;
        matMulABt(IKH_P, IKH, IKH_P_IKHt);

        float (&KRKt)[15][15] = scratchD_;
        for (int i = 0; i < 15; i++)
        {
            for (int j = 0; j < 15; j++)
            {
                float sum = 0.0f;
                for (int k = 0; k < 3; k++)
                    for (int l = 0; l < 3; l++)
                        sum += K[i][k] * R[k][l] * K[j][l];
                KRKt[i][j] = sum;
            }
        }

        for (int i = 0; i < 15; i++)
            for (int j = 0; j < 15; j++)
                P_[i][j] = IKH_P_IKHt[i][j] + KRKt[i][j];

        resetAttitudeErrorCovariance(dx);
        symmetrizeAndClampP();
    }

    void applyCorrectionAndJoseph2D(const float dx[15], const float K[15][2],
                                    const float H[2][15], const float R[2][2])
    {
        pNav_[0] += dx[0]; pNav_[1] += dx[1]; pNav_[2] += dx[2];
        vNav_[0] += dx[3]; vNav_[1] += dx[4]; vNav_[2] += dx[5];

        float dq0 = 1.0f, dq1 = 0.5f * dx[6], dq2 = 0.5f * dx[7], dq3 = 0.5f * dx[8];
        float qc0, qc1, qc2, qc3;
        quatMultiply(qNav0_, qNav1_, qNav2_, qNav3_, dq0, dq1, dq2, dq3,
                     qc0, qc1, qc2, qc3);
        quatNormalize(qc0, qc1, qc2, qc3);
        qNav0_ = qc0; qNav1_ = qc1; qNav2_ = qc2; qNav3_ = qc3;

        float baxRaw = baNav_[0] + dx[9], bayRaw = baNav_[1] + dx[10], bazRaw = baNav_[2] + dx[11];
        baNav_[0] = constrain(baxRaw, -accelBiasLimitG_, accelBiasLimitG_);
        baNav_[1] = constrain(bayRaw, -accelBiasLimitG_, accelBiasLimitG_);
        baNav_[2] = constrain(bazRaw, -accelBiasLimitG_, accelBiasLimitG_);
        if (baNav_[0] != baxRaw || baNav_[1] != bayRaw || baNav_[2] != bazRaw)
            accelBiasClampCount_++;

        float bgxRaw = bgNav_[0] + dx[12], bgyRaw = bgNav_[1] + dx[13], bgzRaw = bgNav_[2] + dx[14];
        bgNav_[0] = constrain(bgxRaw, -gyroBiasLimitDps_, gyroBiasLimitDps_);
        bgNav_[1] = constrain(bgyRaw, -gyroBiasLimitDps_, gyroBiasLimitDps_);
        bgNav_[2] = constrain(bgzRaw, -gyroBiasLimitDps_, gyroBiasLimitDps_);
        if (bgNav_[0] != bgxRaw || bgNav_[1] != bgyRaw || bgNav_[2] != bgzRaw)
            gyroBiasClampCount_++;

        for (int i = 0; i < 3; i++)
        {
            if (!isfinite(vNav_[i])) vNav_[i] = 0.0f;
            if (!isfinite(pNav_[i])) pNav_[i] = 0.0f;
        }

        float (&IKH)[15][15] = scratchA_;
        for (int i = 0; i < 15; i++)
        {
            for (int j = 0; j < 15; j++)
            {
                float khij = 0.0f;
                for (int k = 0; k < 2; k++)
                    khij += K[i][k] * H[k][j];
                IKH[i][j] = (i == j ? 1.0f : 0.0f) - khij;
            }
        }

        float (&IKH_P)[15][15] = scratchB_;
        matMulAB(IKH, P_, IKH_P);

        float (&IKH_P_IKHt)[15][15] = scratchC_;
        matMulABt(IKH_P, IKH, IKH_P_IKHt);

        float (&KRKt)[15][15] = scratchD_;
        for (int i = 0; i < 15; i++)
        {
            for (int j = 0; j < 15; j++)
            {
                float sum = 0.0f;
                for (int k = 0; k < 2; k++)
                    for (int l = 0; l < 2; l++)
                        sum += K[i][k] * R[k][l] * K[j][l];
                KRKt[i][j] = sum;
            }
        }

        for (int i = 0; i < 15; i++)
            for (int j = 0; j < 15; j++)
                P_[i][j] = IKH_P_IKHt[i][j] + KRKt[i][j];

        resetAttitudeErrorCovariance(dx);
        symmetrizeAndClampP();
    }

    void resetAttitudeErrorCovariance(const float dx[15])
    {
        // scratchA_/B_ safe here -- only called from applyCorrectionAndJoseph(),
        // after IKH/IKH_P (also scratchA_/B_) have already been folded into P_.
        float (&G)[15][15] = scratchA_;
        for (int i = 0; i < 15; i++)
            for (int j = 0; j < 15; j++)
                G[i][j] = (i == j) ? 1.0f : 0.0f;

        float theta[3] = { dx[6], dx[7], dx[8] };
        float skewTheta[3][3];
        skew3(theta, skewTheta);
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                G[6 + i][6 + j] -= 0.5f * skewTheta[i][j];

        float (&GP)[15][15] = scratchB_;
        matMulAB(G, P_, GP);
        matMulABt(GP, G, P_);
    }

    void symmetrizeAndClampP()
    {
        for (int i = 0; i < 15; i++)
        {
            for (int j = i + 1; j < 15; j++)
            {
                float avg = 0.5f * (P_[i][j] + P_[j][i]);
                P_[i][j] = avg;
                P_[j][i] = avg;
            }
        }

        const float P_MIN = 1e-10f;
        const float P_MAX = 1.0e6f;

        for (int i = 0; i < 15; i++)
            P_[i][i] = constrain(P_[i][i], P_MIN, P_MAX);

        for (int i = 0; i < 15; i++)
        {
            for (int j = i + 1; j < 15; j++)
            {
                float limit = sqrtf(P_[i][i] * P_[j][j]);
                float v = constrain(P_[i][j], -limit, limit);
                P_[i][j] = v;
                P_[j][i] = v;
            }
        }
    }


    static void quatMultiply(
        float a0, float a1, float a2, float a3,
        float b0, float b1, float b2, float b3,
        float &o0, float &o1, float &o2, float &o3)
    {
        o0 = a0 * b0 - a1 * b1 - a2 * b2 - a3 * b3;
        o1 = a0 * b1 + a1 * b0 + a2 * b3 - a3 * b2;
        o2 = a0 * b2 - a1 * b3 + a2 * b0 + a3 * b1;
        o3 = a0 * b3 + a1 * b2 - a2 * b1 + a3 * b0;
    }

    static void quatNormalize(float &q0, float &q1, float &q2, float &q3)
    {
        float n2 = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;

        if (n2 < 1e-12f)
        {
            q0 = 1.0f; q1 = q2 = q3 = 0.0f;
            return;
        }

        float invN = 1.0f / sqrtf(n2);
        q0 *= invN; q1 *= invN; q2 *= invN; q3 *= invN;
    }

    static void quatToDCM(float q0, float q1, float q2, float q3, float R[3][3])
    {
        R[0][0] = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
        R[0][1] =        2.0f * (q1 * q2 - q0 * q3);
        R[0][2] =        2.0f * (q1 * q3 + q0 * q2);

        R[1][0] =        2.0f * (q1 * q2 + q0 * q3);
        R[1][1] = 1.0f - 2.0f * (q1 * q1 + q3 * q3);
        R[1][2] =        2.0f * (q2 * q3 - q0 * q1);

        R[2][0] =        2.0f * (q1 * q3 - q0 * q2);
        R[2][1] =        2.0f * (q2 * q3 + q0 * q1);
        R[2][2] = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    }

    static void skew3(const float v[3], float S[3][3])
    {
        S[0][0] = 0.0f;   S[0][1] = -v[2];  S[0][2] =  v[1];
        S[1][0] =  v[2];  S[1][1] = 0.0f;   S[1][2] = -v[0];
        S[2][0] = -v[1];  S[2][1] =  v[0];  S[2][2] = 0.0f;
    }

    static void skew3Vec(float vx, float vy, float vz, float S[3][3])
    {
        float v[3] = { vx, vy, vz };
        skew3(v, S);
    }

    static void mat3Mul(const float A[3][3], const float B[3][3], float O[3][3])
    {
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
            {
                float sum = 0.0f;
                for (int k = 0; k < 3; k++)
                    sum += A[i][k] * B[k][j];
                O[i][j] = sum;
            }
    }

    static bool invert3x3(const float M[3][3], float Inv[3][3])
    {
        float a = M[0][0], b = M[0][1], c = M[0][2];
        float d = M[1][0], e = M[1][1], f = M[1][2];
        float g = M[2][0], h = M[2][1], i = M[2][2];

        float A =  (e * i - f * h);
        float B = -(d * i - f * g);
        float C =  (d * h - e * g);

        float det = a * A + b * B + c * C;

        if (fabsf(det) < 1e-12f)
            return false;

        float invDet = 1.0f / det;

        float D = -(b * i - c * h);
        float E =  (a * i - c * g);
        float F = -(a * h - b * g);
        float G =  (b * f - c * e);
        float H = -(a * f - c * d);
        float I =  (a * e - b * d);

        Inv[0][0] = A * invDet; Inv[0][1] = D * invDet; Inv[0][2] = G * invDet;
        Inv[1][0] = B * invDet; Inv[1][1] = E * invDet; Inv[1][2] = H * invDet;
        Inv[2][0] = C * invDet; Inv[2][1] = F * invDet; Inv[2][2] = I * invDet;

        return true;
    }

    static float mahalanobis3(const float y[3], const float Sinv[3][3])
    {
        float d2 = 0.0f;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                d2 += y[i] * Sinv[i][j] * y[j];
        return d2;
    }

    static bool invert2x2(const float M[2][2], float Inv[2][2])
    {
        float a = M[0][0], b = M[0][1], c = M[1][0], d = M[1][1];
        float det = a * d - b * c;
        if (fabsf(det) < 1e-12f)
            return false;
        float invDet = 1.0f / det;
        Inv[0][0] =  d * invDet; Inv[0][1] = -b * invDet;
        Inv[1][0] = -c * invDet; Inv[1][1] =  a * invDet;
        return true;
    }

    static float mahalanobis2(const float y[2], const float Sinv[2][2])
    {
        return y[0] * (Sinv[0][0] * y[0] + Sinv[0][1] * y[1]) +
               y[1] * (Sinv[1][0] * y[0] + Sinv[1][1] * y[1]);
    }

    static void matMulAB(const float A[15][15], const float B[15][15], float O[15][15])
    {
        for (int i = 0; i < 15; i++)
        {
            for (int j = 0; j < 15; j++)
            {
                float sum = 0.0f;
                for (int k = 0; k < 15; k++)
                    sum += A[i][k] * B[k][j];
                O[i][j] = sum;
            }
        }
    }

    static void matMulABt(const float A[15][15], const float B[15][15], float O[15][15])
    {
        for (int i = 0; i < 15; i++)
        {
            for (int j = 0; j < 15; j++)
            {
                float sum = 0.0f;
                for (int k = 0; k < 15; k++)
                    sum += A[i][k] * B[j][k];
                O[i][j] = sum;
            }
        }
    }
};

#endif
