#pragma once
#include <cstdarg>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace capture_v2 {
// Bounded, locale-independent Arduino JSON. Nine decimal digits round-trip float32.
class MetadataJson {
public:
    char* data; size_t capacity, n=0; bool ok=true;
    MetadataJson(char* dst, size_t cap): data(dst), capacity(cap) { if(cap) data[0]=0; }
    void append(const char* format, ...) {
        if(!ok) return;
        va_list args; va_start(args,format);
        int count=vsnprintf(data+n,capacity-n,format,args); va_end(args);
        if(count<0 || (size_t)count>=capacity-n) { ok=false; return; }
        n+=count;
    }
    void key(const char* name) {
        if(depth && !first[depth-1]) append(",");
        if(depth) first[depth-1]=false;
        if(name) append("\"%s\":",name);
    }
    void begin(const char* name=nullptr) {
        key(name); append("{");
        if(depth>=16) { ok=false; return; }
        first[depth++]=true;
    }
    void end() { if(!depth) { ok=false; return; } --depth;append("}"); }
    void string(const char* name,const char* value) {
        key(name); append("\"");
        for(const unsigned char* p=(const unsigned char*)value;*p && ok;++p) {
            if(*p=='"'||*p=='\\') append("\\%c",*p);
            else if(*p<32) append("\\u%04x",*p);
            else append("%c",*p);
        }
        append("\"");
    }
    void number(const char* name,double value) {
        if(!std::isfinite(value)) { ok=false;return; }
        key(name);append("%.9g",value);
    }
    void number(const char* name,int value) { key(name);append("%d",value); }
    void count(const char* name,uint32_t value) { key(name);append("%lu",(unsigned long)value); }
    void integer(const char* name,long value) { key(name);append("%ld",value); }
    void boolean(const char* name,bool value) { key(name);append(value?"true":"false"); }
    void raw(const char* name,const char* value) { key(name);append("%s",value); }
    void vector(const char* name,const float* values,size_t count) {
        key(name);append("[");
        for(size_t i=0;i<count;++i) {
            if(!std::isfinite(values[i])) { ok=false;return; }
            append(i?",%.9g":"%.9g",values[i]);
        }
        append("]");
    }
private:
    bool first[16]={}; size_t depth=0;
};

// DS-000193 rev 1.1, sections 8.14--8.18. Unsupported modes stay incomplete.
inline bool sensorMetadata(MetadataJson& j,uint8_t who,const uint8_t* r,const uint8_t* power,
                           uint8_t lp,uint8_t fifo,bool readOk,float aRes,float gRes,int type,int board) {
    const unsigned gc=r[2]&3, gd=r[1]&7, ac=(r[4]>>3)&1, ad=r[4]&7;
    const float ar=float(2u<<((r[3]>>3)&3)),gr=float(250u<<((r[2]>>3)&3));
    const bool normal=readOk && who==0x19 && !(power[1]&0x70) && !(power[2]&0x3f)
                      && !(power[0]&0x70) && !fifo && !(r[2]&0xe0) && !(r[3]&0xe0) && !lp;
    const bool scales=std::isfinite(aRes) && std::isfinite(gRes) && aRes==ar/32768.f && gRes==gr/32768.f;
    // Restrict completion to the installed, reviewed low-noise divider mode.
    const bool known=normal && !gc && gd>0 && gd<7 && !ac;
    j.string("model","MPU6886");j.string("driver","M5Unified");j.integer("type",type);j.integer("board",board);
    j.string("driver_source_sha256",CAPTURE_V2_DRIVER_SHA);
    j.boolean("read_ok",readOk);j.boolean("complete",known&&scales);
    j.begin("registers");
    j.integer("who_am_i",who);
    const char* names[]={"smplrt_div","config","gyro_config","accel_config","accel_config2"};
    for(int i=0;i<5;++i) j.integer(names[i],r[i]);
    j.integer("user_ctrl",power[0]);j.integer("pwr_mgmt_1",power[1]);j.integer("pwr_mgmt_2",power[2]);
    j.integer("lp_mode_cfg",lp);j.integer("fifo_en",fifo);j.end();
    if(readOk && who==0x19) {
        j.number("accel_range_g",ar);j.number("gyro_range_dps",gr);
        j.number("accel_register_g_per_count",ar/32768.f);j.number("gyro_register_dps_per_count",gr/32768.f);
    } else {
        j.raw("accel_range_g","null");j.raw("gyro_range_dps","null");
        j.raw("accel_register_g_per_count","null");j.raw("gyro_register_dps_per_count","null");
    }
    if(std::isfinite(aRes)) j.number("driver_accel_g_per_count",aRes);else j.raw("driver_accel_g_per_count","null");
    if(std::isfinite(gRes)) j.number("driver_gyro_dps_per_count",gRes);else j.raw("driver_gyro_dps_per_count","null");
    j.boolean("driver_scale_matches_registers",readOk&&who==0x19&&scales);
    if(known) {
        const float gyroBw[]={250,176,92,41,20,10,5,3281};
        const float accelBw[]={218.1f,218.1f,99.f,44.8f,21.2f,10.2f,5.1f,420.f};
        j.number("gyro_output_hz",1000.f/(1+r[0]));j.number("accel_output_hz",1000.f/(1+r[0]));
        j.number("gyro_dlpf_3db_hz",gyroBw[gd]);j.number("accel_dlpf_3db_hz",accelBw[ad]);
    } else {
        j.raw("gyro_output_hz","null");j.raw("accel_output_hz","null");
        j.raw("gyro_dlpf_3db_hz","null");j.raw("accel_dlpf_3db_hz","null");
    }
    j.string("interpretation_source","DS-000193 rev1.1 sections8.14-8.18; M5Unified source hash");
    j.string("physical_scale_verification","pending");
    j.string("axis_order","driver default XYZ; source-derived, physical check pending");
    j.string("driver_auto_calibration","disabled by reviewed M5Unified begin; offsets retained");
    return known&&scales;
}
}
