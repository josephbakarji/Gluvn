#pragma once
#include "CaptureV2Wire.h"
#include "CaptureV2Build.h"
#include <esp_app_desc.h>
#include <esp_system.h>
#include <mbedtls/sha256.h>
#include <atomic>
#include "CaptureV2Metadata.h"

// Included after sketch globals. No estimator equations/configuration are changed here.
class CaptureV2Runtime {
public:
    // 10 Nov (glove_T02_sequence): the right glove's radio stalled ~600 ms (NimBLE ENOMEM retries) 1 s after START;
    // the 500 ms queue overflowed and the capture stopped. Budget raised to 1.5 s (189 slots, +22.6 KB static RAM;
    // free heap was 87 KB while streaming). On a longer stall the OLDEST queued sample is dropped and counted
    // (tx_rejected, frame_seq gap visible to the host) and acquisition continues; the host decides whether that ends
    // the capture (strict research profile: yes; live instrument: no).
    static constexpr uint32_t BLE_STALL_BUDGET_US=1500000U;
    static constexpr size_t SAMPLE_QUEUE_FRAMES=(BLE_STALL_BUDGET_US+SENSOR_INTERVAL_US-1)/SENSOR_INTERVAL_US+1;
    static constexpr size_t EVENT_QUEUE_FRAMES=16;
    static_assert(SAMPLE_QUEUE_FRAMES<=255,"sample queue indices must fit uint8");
    std::atomic<uint32_t> bleConnIntervalUs{0};
    bool active=false, stopPending=false;
    bool researchSelected=false;
    bool researchBle=false, legacyBle=false;
    bool legacyEnabled=CAPTURE_V2_USB_TX_BUFFER_BYTES==0;
    std::atomic<bool> disconnected{false};
    std::atomic<bool> bleSubscribed{false};
    std::atomic<int> bleStatusError{0};
    uint32_t notifyRetries=0;
    int notifyLastStatus=0;
    std::atomic<uint16_t> mtu{23};
    uint16_t navEpoch=0, calibrationEpoch=0;
    uint32_t imuSeq=0, frameSeq=0, lastMahonyReset=0;
    uint32_t txAttempted=0, txEnqueued=0, txRejected=0, txSent=0, txFailed=0, notifyFailed=0;
    std::atomic<uint32_t> commandDrops{0};
    uint32_t controlRejected=0, serialBytes=0, notifyBytes=0;
    size_t usbTxBufferBytes=0;
    uint32_t timedStartUs=0,timedDurationUs=0;
    const char* calibrationResult="unknown";
    uint16_t calibrationSamples=0;
    uint32_t calibrationStartedUs=0, calibrationCompletedUs=0;
    const char* calibrationOrigin="unknown";
    capture_v2::Sample sample;
    char boot[33]={}, nonce[33]={};

    void init() {
        for(int i=0;i<4;++i) snprintf(boot+i*8,9,"%08lx",(unsigned long)esp_random());
        for(int i=0;i<3;++i) sample.accel_pre_g[i]=sample.gyro_pre_dps[i]=sample.accel_input_g[i]=sample.gyro_input_dps[i]=NAN;
        lastMahonyReset=ahrs.getResetCount();
    }
    bool busy() const { return packetLen>0; }
    bool hasWork() const { return busy()||count||eventCount||messageLen||stopPending; }
    bool legacyReady() const { return legacyEnabled&&!active&&!hasWork(); }
    bool legacyCommand(const String& cmd, bool fromBle=false) {
        if(active||stopPending) {
            ++commandDrops; event("fatal","legacy_command_during_capture"); return false;
        }
        const bool starts=cmd=="NORMAL_MODE"||cmd=="CAL_START"||cmd=="CAL_STOP"||
            cmd=="RAW_MODE"||cmd=="CAL_MODE"||cmd=="QUAT_STREAM_ON"||
            cmd=="NAV_QUAT_STREAM_ON"||cmd=="MOTION_STREAM_ON"||
            (cmd=="RECALIBRATE_GYRO"&&(!researchSelected||fromBle!=researchBle));
        if(starts) {
            if(hasWork()) return false;
            legacyEnabled=true; legacyBle=fromBle; researchSelected=false;
        }
        return true;
    }
    void resetBleTransport() {
        bleSubscribed.store(false); bleStatusError.store(0);
        bleFault.store(false);
        bleAttempted=false; bleRetrying=false;
    }
    void onBleStatus(int code) {
        // NimBLE invokes this inside notify(), before radio delivery; it is not a TX credit.
        if(code) bleStatusError.store(code);
    }
    void event(const char* name,const char* reason="") {
        if(!active&&!stopPending) return;
        char json[240];
        int n=snprintf(json,sizeof(json),"{\"event\":\"%s\",\"reason\":\"%s\",\"nonce\":\"%s\",\"frame_seq\":%lu,\"nav_epoch\":%u,\"calibration_epoch\":%u}",
                 name,reason,nonce,(unsigned long)frameSeq,navEpoch,calibrationEpoch);
        if(n<0||(size_t)n>=sizeof(json)) { ++controlRejected; return; }
        enqueueEvent(json);
    }
    void reset(const char* reason) {
        if(navEpoch==65535) { event("fatal","nav_epoch_overflow"); active=false; return; }
        ++navEpoch; event("reset",reason);
    }
    void calibration(const char* reason) {
        if(calibrationEpoch==65535) { event("fatal","calibration_epoch_overflow"); active=false; return; }
        ++calibrationEpoch; event("calibration_applied",reason);
    }
    void beginTick() { sample.status=0; sample.imu_dt_us=0; }
    void read(float ax0,float ay0,float az0,float gx0,float gy0,float gz0,uint32_t readUs,uint32_t dtUs,bool finite,bool validDt) {
        ++imuSeq;
        sample.imu_seq=imuSeq; sample.imu_read_us=readUs;
        if(!active) return;
        sample.imu_seq=imuSeq; sample.imu_read_us=readUs; sample.imu_dt_us=dtUs;
        sample.status=1|(finite?2:0)|(validDt?4:0);
        const float a[3]={ax0,ay0,az0},g[3]={gx0,gy0,gz0},ac[3]={ax,ay,az},gc[3]={gx,gy,gz};
        for(int i=0;i<3;++i) {
            sample.accel_pre_g[i]=a[i]; sample.gyro_pre_dps[i]=g[i];
            sample.accel_input_g[i]=finite&&validDt?ac[i]:NAN;
            sample.gyro_input_dps[i]=finite&&validDt?gc[i]:NAN;
        }
        if(!finite) event("nonfinite_input");
        if(!validDt) event("dt_rejected");
    }
    void emit(uint32_t scanStart,uint32_t scanEnd) {
        if(lastMahonyReset!=ahrs.getResetCount()) {
            lastMahonyReset=ahrs.getResetCount(); event("mahony_reset","internal_or_explicit");
        }
        bool armed=navEkf.isArmed();
        bool snapshotDue=armed!=lastArmed;
        if(armed!=lastArmed) { lastArmed=armed; event(armed?"armed":"unarmed"); }
        sample.hand=hand; sample.frame_seq=frameSeq++; sample.frame_us=scanEnd;
        sample.nav_epoch=navEpoch; sample.calibration_epoch=calibrationEpoch;
        uint32_t span=scanEnd-scanStart; sample.finger_scan_span_us=span>=65535?65535:span;
        ahrs.getQuaternion(sample.q_mahony_wxyz[0],sample.q_mahony_wxyz[1],sample.q_mahony_wxyz[2],sample.q_mahony_wxyz[3]);
        navEkf.getQuaternion(sample.q_nav_wxyz[0],sample.q_nav_wxyz[1],sample.q_nav_wxyz[2],sample.q_nav_wxyz[3]);
        navEkf.getVelocity(sample.velocity_nav_mps[0],sample.velocity_nav_mps[1],sample.velocity_nav_mps[2]);
        navEkf.getPosition(sample.position_nav_m[0],sample.position_nav_m[1],sample.position_nav_m[2]);
        navEkf.getWorldAcceleration(sample.lin_accel_nav_mps2[0],sample.lin_accel_nav_mps2[1],sample.lin_accel_nav_mps2[2]);
        sample.zupt_mahalanobis=navEkf.getZuptMahalanobis();
        bool mf=true,nf=true;
        for(int i=0;i<4;++i) { mf &= isfinite(sample.q_mahony_wxyz[i]); nf &= isfinite(sample.q_nav_wxyz[i]); }
        for(int i=0;i<3;++i) nf &= isfinite(sample.velocity_nav_mps[i])&&isfinite(sample.position_nav_m[i])&&isfinite(sample.lin_accel_nav_mps2[i]);
        sample.status |= (navEkf.isArmed()?32:0)|(navEkf.isZuptActive()?64:0)|(navEkf.isZuptStarved()?128:0)|
                         (syncLedOn?256:0)|(lowBatteryFlag?512:0)|(mf?1024:0)|(nf?2048:0);
        for(int i=0;i<5;++i) { sample.flex_raw[i]=flexValues[i]; sample.press_raw[i]=fsrValues[i]; }
        uint8_t payload[172]; capture_v2::encode(payload,sample);
        ++txAttempted;
        if(count==SAMPLE_QUEUE_FRAMES) {
            // Drop the oldest queued frame (the packet being sent was already copied out), keep sampling.
            ++txRejected; head=(head+1)%SAMPLE_QUEUE_FRAMES; --count;
            if(!dropping) { dropping=true; if(eventCount<EVENT_QUEUE_FRAMES) event("sample_dropped","ble_stall"); }
        } else if(count<SAMPLE_QUEUE_FRAMES/2) dropping=false;
        size_t slot=(head+count)%SAMPLE_QUEUE_FRAMES;
        capture_v2::frame(samples[slot],payload,172); sampleQueuedUs[slot]=micros(); ++count; ++txEnqueued;
        if(count>queueHigh) queueHigh=count;
        // Telemetry is 7 control frames per second; skip it while the event queue cannot hold it (radio stall)
        // instead of overflowing it (control_rejected).
        const bool room=eventCount+7<=EVENT_QUEUE_FRAMES;
        if(millis()-lastMetrics>=1000 && room) { lastMetrics=millis(); metrics();snapshotDue=true; }
        if(snapshotDue && eventCount+4<=EVENT_QUEUE_FRAMES) estimatorSnapshot();
    }
    bool command(const String& cmd, bool fromBle=false) {
        if(!cmd.startsWith("CAPTURE_V2_")) return false;
        int sep=cmd.indexOf(':');
        if(sep<0) return true;
        String verb=cmd.substring(0,sep), value=cmd.substring(sep+1);
        uint32_t durationUs=0;
        if(verb=="CAPTURE_V2_START_TIMED") {
            int split=value.indexOf(':');
            if(split!=32) return true;
            String duration=value.substring(split+1); value=value.substring(0,split);
            if(!duration.length()||duration.length()>7) return true;
            for(size_t i=0;i<duration.length();++i) if(!isdigit(duration[i])) return true;
            unsigned long ms=strtoul(duration.c_str(),nullptr,10);
            if(!ms||ms>3615000UL) return true;
            durationUs=(uint32_t)ms*1000UL;
        }
        if(value.length()!=32) return true;
        for(int i=0;i<32;++i) if(!isxdigit(value[i])) return true;
        if(verb=="CAPTURE_V2_INFO") {
            if(active&&(value!=nonce||fromBle!=researchBle)) return true;
            if(!active&&value!=nonce) { strncpy(nonce,value.c_str(),33); stopped=false; }
            legacyEnabled=false; researchSelected=true; researchBle=fromBle;
            metadata(); return true;
        }
        if(value!=nonce||!nonce[0]) return true;
        if(verb=="CAPTURE_V2_START"||verb=="CAPTURE_V2_START_TIMED") {
            if(stopped||stopPending) return true;
            if(!active) {
                active=true; lastArmed=navEkf.isArmed();
                timedStartUs=micros(); timedDurationUs=durationUs;
                // Periodic diagnostics start one second after START, not on the first sample.
                lastMetrics=millis();
            }
            event("capture_started");
        } else if(verb=="CAPTURE_V2_STOP") {
            if(stopped) { enqueueEvent(lastStop); return true; }
            if(!stopPending) stopMetricsSent=false;
            active=false; stopPending=true;
        }
        return true;
    }
    void service() {
        int status=bleStatusError.exchange(0);
        if(researchSelected && researchBle && status && !retryableBleStatus(status)) failBle("notify_status_failed");
        if(active&&timedDurationUs&&(uint32_t)(micros()-timedStartUs)>=timedDurationUs) {
            active=false; stopPending=true; stopMetricsSent=false;
        }
        if(disconnected.exchange(false)) {
            if(researchBle&&active) {
                event("fatal","ble_disconnected"); active=false; stopPending=true; stopMetricsSent=false;
            }
            if(researchBle) researchSelected=false;
            if(legacyBle) legacyEnabled=false;
        }
        if(!packetLen) {
            // Bound telemetry priority so snapshots cannot starve waiting sensor frames.
            if(eventCount && (!count || consecutiveControls<2)) {
                makeControl(3,eventIds[eventHead],0,1,eventTimes[eventHead],events[eventHead],strlen(events[eventHead]));
                eventHead=(eventHead+1)%EVENT_QUEUE_FRAMES; --eventCount;
                if(consecutiveControls<2) ++consecutiveControls;
            } else if(count) {
                uint32_t wait=micros()-sampleQueuedUs[head]; if(wait>maxQueueWaitUs) maxQueueWaitUs=wait;
                memcpy(packet,samples[head],177); packetLen=177;
                head=(head+1)%SAMPLE_QUEUE_FRAMES; --count; sendingSample=true; consecutiveControls=0;
            } else if(messageLen) {
                size_t n=messageLen-messageOffset; if(n>240) n=240;
                makeControl(2,messageId,messageOffset/240,(messageLen+239)/240,messageUs,message+messageOffset,n);
                messageOffset+=n;
                if(messageOffset==messageLen) messageLen=messageOffset=0;
            } else if(stopPending) {
                if(!stopMetricsSent) { stopMetricsSent=true; metrics(); return; }
                snprintf(lastStop,sizeof(lastStop),"{\"event\":\"capture_stopped\",\"nonce\":\"%s\",\"tx_rejected\":%lu,\"notify_failed\":%lu,\"control_rejected\":%lu,\"command_drops\":%lu}",
                         nonce,(unsigned long)txRejected,(unsigned long)notifyFailed,(unsigned long)controlRejected,(unsigned long)commandDrops);
                stopPending=false; stopped=true; enqueueEvent(lastStop); return;
            } else return;
            usbOffset=researchBle?packetLen:0; bleOffset=0;
            sendBle=researchBle&&!bleFault; packetFailed=researchBle&&bleFault;
        }
        int available=Serial.availableForWrite();
#if CAPTURE_V2_USB_TX_BUFFER_BYTES > 0
        // Submit complete packets through the driver's TX ring, as the legacy sender does.
        if(usbOffset==0 && available>=(int)packetLen) {
            size_t sent=Serial.write(packet,packetLen); serialBytes+=sent;
            if(sent!=packetLen) { packetFailed=true; event("fatal","usb_short_write"); }
            usbOffset=packetLen;
        }
#else
        if(available>0&&usbOffset<packetLen) {
            size_t n=packetLen-usbOffset; if(n>(size_t)available) n=available;
            size_t sent=Serial.write(packet+usbOffset,n); usbOffset+=sent; serialBytes+=sent;
        }
#endif
        if(sendBle) {
            if(!bleConnected || !bleSubscribed.load()) {
                failBle(bleConnected?"ble_not_subscribed":"ble_disconnected");
            } else if(bleOffset<packetLen &&
                      (!bleAttempted || (uint32_t)(micros()-lastBleAttemptUs)>=2000U)) {
                size_t n=packetLen-bleOffset, cap=mtu>3?mtu-3:20; if(n>cap) n=cap;
                lastBleAttemptUs=micros(); bleAttempted=true;
                // Copying notify() owns accepted bytes. Retry only a rejected submission.
                bleStatusError.store(0);
                bool accepted=txChar->notify(packet+bleOffset,n);
                int code=bleStatusError.exchange(0);
                if(accepted && !code) {
                    bleOffset+=n; notifyBytes+=n; bleRetrying=false;
                } else if(!accepted && retryableBleStatus(code)) {
                    ++notifyRetries; notifyLastStatus=code;
                    if(!bleRetrying) { bleRetrying=true; bleRetryStartedUs=micros(); }
                    if((uint32_t)(micros()-bleRetryStartedUs)>=BLE_STALL_BUDGET_US) failBle("notify_retry_timeout");
                } else {
                    notifyLastStatus=code; failBle("notify_rejected");
                }
            }
        }
        if(usbOffset==packetLen&&(!sendBle||bleOffset==packetLen)) {
            if(sendingSample) { if(packetFailed) ++txFailed; else ++txSent; }
            packetLen=0; sendingSample=false;
        }
    }
private:
    uint8_t samples[SAMPLE_QUEUE_FRAMES][177],packet[260]; size_t packetLen=0,usbOffset=0,bleOffset=0;
    uint32_t sampleQueuedUs[SAMPLE_QUEUE_FRAMES]={},maxQueueWaitUs=0;
    uint8_t queueHigh=0,consecutiveControls=0;
    uint8_t head=0,count=0,eventHead=0,eventCount=0;
    bool sendBle=false,sendingSample=false,lastArmed=false,stopped=false,stopMetricsSent=false,packetFailed=false;
    bool dropping=false;
    std::atomic<bool> bleFault{false};
    char events[EVENT_QUEUE_FRAMES][240],message[12288],lastStop[240]={};
    uint32_t eventIds[EVENT_QUEUE_FRAMES],eventTimes[EVENT_QUEUE_FRAMES];
    size_t messageLen=0,messageOffset=0; uint32_t id=0,messageId=0,messageUs=0,lastMetrics=0;
    bool bleAttempted=false,bleRetrying=false;
    uint32_t lastBleAttemptUs=0,bleRetryStartedUs=0;
    bool retryableBleStatus(int code) const {
        // NimBLE host codes: EAGAIN=1, ENOMEM=6, EBUSY=15. Allocation can fail before onStatus.
        return code==0 || code==1 || code==6 || code==15;
    }
    void failBle(const char* reason) {
        if(bleFault) return;
        ++notifyFailed; bleFault=true; sendBle=false; packetFailed=true;
        event("fatal",reason);
        if(active) { active=false; stopPending=true; stopMetricsSent=false; }
    }
    void enqueueEvent(const char* json) {
        if(eventCount==EVENT_QUEUE_FRAMES||strlen(json)>240) { ++controlRejected; return; }
        size_t i=(eventHead+eventCount)%EVENT_QUEUE_FRAMES; strncpy(events[i],json,239); events[i][239]=0;
        eventIds[i]=++id; eventTimes[i]=micros(); ++eventCount;
    }
    void makeControl(uint8_t kind,uint32_t ident,uint16_t part,uint16_t parts,uint32_t us,const char* data,size_t n) {
        uint8_t payload[255]; capture_v2::Writer w{payload}; w.u8(2); w.u8(kind); w.u8(hand);
        w.u32(ident); w.u16(part); w.u16(parts); w.u32(us); memcpy(payload+w.n,data,n); w.n+=n;
        packetLen=capture_v2::frame(packet,payload,w.n);
    }
    void metrics() {
        char json[240]; int n=snprintf(json,sizeof(json),"{\"event\":\"tx_metrics\",\"tx_attempted\":%lu,\"tx_enqueued\":%lu,\"tx_rejected\":%lu,\"tx_sent\":%lu,\"tx_failed\":%lu,\"tx_pending\":%u}",
            (unsigned long)txAttempted,(unsigned long)txEnqueued,(unsigned long)txRejected,(unsigned long)txSent,
            (unsigned long)txFailed,count+(sendingSample?1:0));
        if(n<0||(size_t)n>=sizeof(json)) { ++controlRejected; return; }
        enqueueEvent(json);
        n=snprintf(json,sizeof(json),"{\"event\":\"tx_queue_metrics\",\"sample_capacity\":%u,\"sample_high\":%u,\"max_wait_us\":%lu,\"conn_interval_us\":%lu,\"notify_last_status\":%d}",
            (unsigned)SAMPLE_QUEUE_FRAMES,(unsigned)queueHigh,(unsigned long)maxQueueWaitUs,
            (unsigned long)bleConnIntervalUs.load(),notifyLastStatus);
        if(n<0||(size_t)n>=sizeof(json)) { ++controlRejected; return; }
        enqueueEvent(json);
        n=snprintf(json,sizeof(json),"{\"event\":\"metrics\",\"notify_failed\":%lu,\"control_rejected\":%lu,\"command_drops\":%lu,\"heap_bytes\":%lu,\"stack_bytes\":%lu,\"serial_bytes\":%lu,\"notify_bytes\":%lu,\"notify_retries\":%lu}",
            (unsigned long)notifyFailed,(unsigned long)controlRejected,(unsigned long)commandDrops,
            (unsigned long)ESP.getFreeHeap(),(unsigned long)uxTaskGetStackHighWaterMark(nullptr),(unsigned long)serialBytes,(unsigned long)notifyBytes,(unsigned long)notifyRetries);
        if(n<0||(size_t)n>=sizeof(json)) { ++controlRejected; return; }
        enqueueEvent(json);
    }
    // Four bounded event pages share a frame/time identity; no commands or sensor reads.
    void estimatorSnapshot() {
        float mg[3],ba[3],bg[3];
        ahrs.getGyroBias(mg[0],mg[1],mg[2]);navEkf.getAccelBias(ba[0],ba[1],ba[2]);navEkf.getGyroBias(bg[0],bg[1],bg[2]);
        for(int page=0;page<4;++page) {
            char json[240];capture_v2::MetadataJson j(json,sizeof(json));j.begin();
            j.string("event","estimator_snapshot");j.count("frame_seq",sample.frame_seq);j.count("device_us",sample.frame_us);
            j.count("nav_epoch",navEpoch);j.count("calibration_epoch",calibrationEpoch);j.number("page",page);
            if(page==0) {
                j.vector("bias_dps",mg,3);j.boolean("stationary",ahrs.isStationary());j.number("confidence",ahrs.getAccelConfidence());
            } else if(page==1) j.vector("accel_bias_g",ba,3);
            else if(page==2) j.vector("gyro_bias_dps",bg,3);
            else {
                j.number("gravity_trust",navEkf.getGravityLinAccelTrust());j.number("zupt_d2",navEkf.getZuptMahalanobis());
                j.number("since_zupt_s",navEkf.getTimeSinceZupt());
                j.number("flags",(navEkf.isArmed()?1:0)|(navEkf.isZuptActive()?2:0)|
                                 (navEkf.isZuptStarved()?4:0)|(navEkf.isGravityMeasurementValid()?8:0));
            }
            j.end();
            if(!j.ok) { ++controlRejected;return; }
            enqueueEvent(json);
        }
    }

    void metadata() {
        if(messageLen) { ++controlRejected; return; }
        char elf[65],hash[65]; uint8_t digest[32];
        const esp_app_desc_t* app=esp_app_get_description();
        for(int i=0;i<32;++i) snprintf(elf+2*i,3,"%02x",app->app_elf_sha256[i]);
        char finger[512];
        int f=snprintf(finger,sizeof(finger),"{\"min_flex\":[%d,%d,%d,%d,%d],\"max_flex\":[%d,%d,%d,%d,%d],\"min_press\":[%d,%d,%d,%d,%d],\"max_press\":[%d,%d,%d,%d,%d]}",
            MIN_FLEX[0],MIN_FLEX[1],MIN_FLEX[2],MIN_FLEX[3],MIN_FLEX[4],MAX_FLEX[0],MAX_FLEX[1],MAX_FLEX[2],MAX_FLEX[3],MAX_FLEX[4],
            MIN_PRESS[0],MIN_PRESS[1],MIN_PRESS[2],MIN_PRESS[3],MIN_PRESS[4],MAX_PRESS[0],MAX_PRESS[1],MAX_PRESS[2],MAX_PRESS[3],MAX_PRESS[4]);
        if(f<0||(size_t)f>=sizeof(finger)) { ++controlRejected; return; }
        uint8_t registers[5]={},who=0,power[3]={},lp=0,fifo=0; bool registerOk=false;
        float aRes=NAN,gRes=NAN;
        auto sensor=M5.Imu.getImuInstancePtr(0);
        if(sensor && M5.Imu.getType()==m5::imu_mpu6886) {
            registerOk=sensor->readRegister(0x19,registers,5)&&sensor->readRegister(0x75,&who,1)
                       &&sensor->readRegister(0x6a,power,3)&&sensor->readRegister(0x1e,&lp,1)
                       &&sensor->readRegister(0x23,&fifo,1);
            m5::IMU_Base::imu_convert_param_t conversion;
            sensor->getConvertParam(&conversion);aRes=conversion.accel_res;gRes=conversion.gyro_res;
        }
        capture_v2::MetadataJson j(message,sizeof(message));j.begin();
        j.integer("metadata_schema",1);
        j.integer("ble_notify_retries",notifyRetries);
        j.number("ble_notify_last_status",notifyLastStatus);
        j.begin("ble_tx");j.count("sample_queue_frames",SAMPLE_QUEUE_FRAMES);
        j.count("sample_queue_budget_us",SAMPLE_QUEUE_FRAMES*SENSOR_INTERVAL_US);
        j.count("conn_interval_us",bleConnIntervalUs.load());j.number("requested_ll_octets",251);
        j.string("ll_octets_scope","request only; negotiated data length unverified");j.end();
        j.begin("configuration");size_t configStart=j.n-1;
        char handName[2]={hand,0};j.string("hand",handName);
        j.vector("accel_bias_g",ACCEL_BIAS,3);j.vector("active_gyro_bias_dps",ACTIVE_GYRO_BIAS,3);
        j.raw("finger_calibration",finger);
        j.begin("sensor");
        bool sensorComplete=capture_v2::sensorMetadata(j,who,registers,power,lp,fifo,registerOk,aRes,gRes,(int)M5.Imu.getType(),(int)M5.getBoard());
        j.key("driver_offsets_counts_q16");j.append("[");
        for(int i=0;i<6;++i) j.append(i?",%ld":"%ld",(long)M5.Imu.getOffsetData(i));
        j.append("]");j.end();
        j.begin("estimators");
        j.string("source_sha256",CAPTURE_V2_SOURCE_SHA);
        j.string("orientation","world_from_sensor; scalar-first wxyz; no magnetometer");
        j.begin("mahony");ahrs.captureConfiguration(j);j.end();
        j.begin("nav");navEkf.captureConfiguration(j);j.end();
        j.string("initialization","constructor/reset source-hashed; nav arms from confirmed stationary accumulation");
        j.string("fixed_constants","source-hashed; not duplicated as runtime parameters");
        j.end();
        j.begin("acquisition");j.count("scheduled_interval_us",SENSOR_INTERVAL_US);j.number("max_nav_gap_s",MAX_NAV_GAP_SEC);
        j.string("sample_timing","latest-register polling; no FIFO; arrival is not sensor exposure time");j.end();
        j.end();size_t configEnd=j.n;
        if(!j.ok) { ++controlRejected;return; }
        if(mbedtls_sha256((const unsigned char*)message+configStart,configEnd-configStart,digest,0)!=0) { ++controlRejected;return; }
        for(int i=0;i<32;++i) snprintf(hash+2*i,3,"%02x",digest[i]);
        j.string("config_sha256",hash);j.string("config_hash_scope","exact UTF-8 configuration object bytes, including braces");
        j.string("nonce",nonce);j.string("contract_version","2.0.0-draft.1");j.string("boot_id",boot);
        j.integer("reset_reason",esp_reset_reason());j.string("firmware_elf_sha256",elf);
        j.string("firmware_source_sha256",CAPTURE_V2_SOURCE_SHA);j.string("contract_sha256",CAPTURE_V2_CONTRACT_SHA);
        j.string("calibration_result",calibrationResult);j.integer("calibration_samples",calibrationSamples);
        j.vector("accel_bias_g",ACCEL_BIAS,3);j.vector("active_gyro_bias_dps",ACTIVE_GYRO_BIAS,3);j.raw("finger_calibration",finger);
        j.begin("sensor");j.boolean("complete",sensorComplete);j.string("parameters_path","configuration.sensor");j.end();
        j.begin("estimator_config");j.boolean("complete",true);j.string("parameters_path","configuration.estimators");
        float bg[3],ba[3],mg[3];navEkf.getGyroBias(bg[0],bg[1],bg[2]);navEkf.getAccelBias(ba[0],ba[1],ba[2]);ahrs.getGyroBias(mg[0],mg[1],mg[2]);
        j.vector("mahony_bias_dps",mg,3);j.vector("nav_bias_g",ba,3);j.vector("nav_bias_dps",bg,3);j.end();
        j.begin("state_snapshot");
        j.string("scope","INFO snapshot; additional four-page events at 1 s and arming changes; not exact replay checkpoint");
        j.count("device_us",micros());j.boolean("nav_armed",navEkf.isArmed());j.boolean("mahony_stationary",ahrs.isStationary());
        j.number("mahony_still_s",ahrs.getStationaryTime());j.number("mahony_accel_confidence",ahrs.getAccelConfidence());
        float q[4];ahrs.getQuaternion(q[0],q[1],q[2],q[3]);j.vector("q_mahony_wxyz",q,4);
        navEkf.getQuaternion(q[0],q[1],q[2],q[3]);j.vector("q_nav_wxyz",q,4);
        j.boolean("zupt_active",navEkf.isZuptActive());j.boolean("zupt_starved",navEkf.isZuptStarved());
        j.number("zupt_d2",navEkf.getZuptMahalanobis());j.number("time_since_zupt_s",navEkf.getTimeSinceZupt());
        j.boolean("gravity_measurement_valid",navEkf.isGravityMeasurementValid());
        j.number("gravity_linear_accel_trust",navEkf.getGravityLinAccelTrust());
        j.raw("full_filter_state","null");j.raw("full_covariance","null");j.boolean("exact_replay_checkpoint",false);j.end();
        j.begin("capabilities");j.boolean("six_axis",true);j.boolean("reset_events",true);j.boolean("calibration_outcome",true);
        j.boolean("estimator_snapshots",true);j.end();
        j.count("nav_epoch",navEpoch);j.count("calibration_epoch",calibrationEpoch);j.count("mahony_reset_count",ahrs.getResetCount());
        j.number("legacy_flags",(send_quat?2:0)|(send_nav_quat?4:0)|(send_motion?8:0));j.boolean("research_active",active);
        j.number("att_mtu",mtu.load());j.count("frame_seq",frameSeq);j.count("imu_seq",imuSeq);
        j.string("dt_origin","setup_then_previous_successful_driver_read");
        j.string("calibration_origin",calibrationOrigin);j.count("calibration_started_us",calibrationStartedUs);j.count("calibration_completed_us",calibrationCompletedUs);
        j.begin("usb_tx");j.string("mode",CAPTURE_V2_USB_TX_BUFFER_BYTES>0?"buffered_whole_frame":"fifo_fragments");
        j.number("requested_bytes",CAPTURE_V2_USB_TX_BUFFER_BYTES);j.count("configured_bytes",(uint32_t)usbTxBufferBytes);
        j.boolean("timed_capture",true);j.boolean("idle_between_captures",CAPTURE_V2_USB_TX_BUFFER_BYTES>0);j.end();j.end();
        if(!j.ok) { ++controlRejected;message[0]=0;return; }
        messageLen=j.n;messageOffset=0;messageId=++id;messageUs=micros();
    }

};
CaptureV2Runtime captureV2;
