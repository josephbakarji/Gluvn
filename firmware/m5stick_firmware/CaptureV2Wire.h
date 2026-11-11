#ifndef CAPTURE_V2_WIRE_H
#define CAPTURE_V2_WIRE_H
#include <stdint.h>
#include <stddef.h>
#include <string.h>
namespace capture_v2 {
constexpr size_t SAMPLE_BYTES=172, FRAME_BYTES=177, MAX_FRAME_BYTES=260;
static_assert(sizeof(float)==4, "v2 requires float32");
struct Writer {
    uint8_t* p; size_t n=0;
    void u8(uint8_t v) { p[n++]=v; }
    void u16(uint16_t v) { u8(v>>8); u8(v); }
    void u32(uint32_t v) { u16(v>>16); u16(v); }
    void f32(float v) { uint32_t bits; memcpy(&bits,&v,4); u32(bits); }
    void floats(const float* v,size_t count) { for(size_t i=0;i<count;++i) f32(v[i]); }
};
inline uint16_t crc16(const uint8_t* p,size_t n) {
    uint16_t crc=0xffff;
    for(size_t i=0;i<n;++i) { crc^=(uint16_t)p[i]<<8; for(int j=0;j<8;++j) crc=(crc&0x8000)?(crc<<1)^0x1021:crc<<1; }
    return crc;
}
inline size_t frame(uint8_t* out,const uint8_t* payload,size_t n) {
    if(!n||n>255) return 0;
    out[0]=0xa6; out[1]=0x5a; out[2]=(uint8_t)n; memcpy(out+3,payload,n);
    uint16_t crc=crc16(payload,n); out[n+3]=crc>>8; out[n+4]=crc; return n+5;
}
struct Sample {
    uint8_t hand='l'; uint16_t status=0;
    uint32_t frame_seq=0,imu_seq=0,imu_read_us=0,frame_us=0;
    uint16_t nav_epoch=0,calibration_epoch=0,finger_scan_span_us=65535;
    uint32_t imu_dt_us=0;
    float accel_pre_g[3],gyro_pre_dps[3],accel_input_g[3],gyro_input_dps[3];
    float q_mahony_wxyz[4],q_nav_wxyz[4];
    float velocity_nav_mps[3],position_nav_m[3],lin_accel_nav_mps2[3];
    float zupt_mahalanobis=-1; uint16_t flex_raw[5],press_raw[5]; uint8_t sensor_saturation_mask=255;
};
inline size_t encode(uint8_t* payload,const Sample& s) {
    Writer w{payload}; w.u8(2); w.u8(1); w.u8(s.hand); w.u16(s.status);
    w.u32(s.frame_seq); w.u32(s.imu_seq); w.u32(s.imu_read_us); w.u32(s.frame_us);
    w.u16(s.nav_epoch); w.u16(s.calibration_epoch); w.u16(s.finger_scan_span_us); w.u32(s.imu_dt_us);
    w.floats(s.accel_pre_g,3); w.floats(s.gyro_pre_dps,3); w.floats(s.accel_input_g,3); w.floats(s.gyro_input_dps,3);
    w.floats(s.q_mahony_wxyz,4); w.floats(s.q_nav_wxyz,4);
    w.floats(s.velocity_nav_mps,3); w.floats(s.position_nav_m,3); w.floats(s.lin_accel_nav_mps2,3);
    w.f32(s.zupt_mahalanobis); for(int i=0;i<5;++i) w.u16(s.flex_raw[i]); for(int i=0;i<5;++i) w.u16(s.press_raw[i]);
    w.u8(s.sensor_saturation_mask); return w.n;
}
}
#endif
