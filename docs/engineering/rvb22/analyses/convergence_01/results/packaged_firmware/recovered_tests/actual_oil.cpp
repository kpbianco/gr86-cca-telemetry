#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <map>
#include <limits>
#include "runtime_timing.h"
#include "driver_loss.h"
#include "oil_model.h"
#include "oil_fault_recovery.h"
#include "gps_nmea.h"
#include "gps_solution_epoch.h"
#include "gps_mode_config.h"
#include "io_budget.h"
namespace io=cca::io;
using std::isfinite;
static uint64_t clockUs=0, checks=0;
uint32_t millis(){return static_cast<uint32_t>(clockUs/1000u);}
uint32_t micros(){return static_cast<uint32_t>(clockUs);}
void check(bool ok){if(!ok){std::fprintf(stderr,"FAILED check %llu\n",(unsigned long long)checks);std::abort();}++checks;}
struct Log {template<class...T>void printf(const char*,T...){};void println(const char*){};}g_log;
static inline float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static uint16_t g_oilPublishPeriodMs=20;
// 100 Hz acquisition, alpha 0.5; response/alias models bind this exact source.
static constexpr uint32_t OIL_SAMPLE_PERIOD_MS = 10u;
static constexpr uint32_t OIL_SERVICE_LIMIT_US = 5000u;
static constexpr float ADC_IIR_ALPHA = 0.5f;
static oil::Calibration g_oilCalibration{};
static float g_oilRawAdcVolts = 0;
static float g_oilExcitationAdcVolts = 0;
static float g_oilFilteredVolts = 0; // Latest raw median for diagnostics only.
static float g_oilPsi = NAN;
static uint8_t g_oilFlags = 1u << 5;
static bool g_oilHaveValidSample = false;
static timing::PeriodicSchedule g_oilSampleSchedule, g_oilPublishSchedule;
static uint64_t g_oilSkippedSamples = 0, g_oilSkippedPublishes = 0;
static oil::Reading g_oilReading{};
static oil::FaultRecovery g_oilFaultRecovery{};
static timing::ServiceGapMonitor g_oilServiceGap{OIL_SERVICE_LIMIT_US};


struct Event{uint32_t timestamp; uint16_t pressure; uint8_t flags;};
static std::vector<Event> events;
static std::vector<uint64_t> sampleEntries,sampleReturns;
static uint32_t acquisitionDelayUs=0;
static float inputPsi=100.0f;
static uint8_t inputFlags=0;
void sampleOil(){
 sampleEntries.push_back(clockUs); clockUs+=acquisitionDelayUs;sampleReturns.push_back(clockUs);
 oil::Reading r;r.psi=inputPsi;r.flags=inputFlags;
 g_oilReading=g_oilFaultRecovery.apply(r);g_oilFlags=g_oilReading.flags;
}
void publishVirtualCan(uint32_t pid,const uint8_t* data,uint8_t length,uint32_t now){
 check(pid==0x710u&&length==8);events.push_back({now,static_cast<uint16_t>((data[0]<<8)|data[1]),data[2]});
}
static void invalidateOilTiming() {
  oil::Reading fault = g_oilReading;
  fault.flags |= 1u << 7;
  fault.psi = NAN;
  g_oilReading = g_oilFaultRecovery.apply(fault);
  g_oilFlags = g_oilReading.flags;
  g_oilPsi = NAN;
  g_oilHaveValidSample = false;
}
static void serviceOil(uint32_t now) {
  const uint32_t entryUs = micros();
  now = millis(); // Callers must not publish an old pre-service timestamp.
  bool timingFault = g_oilServiceGap.observe(entryUs);
  if (timingFault) invalidateOilTiming();
  uint32_t skipped = 0;
  if (g_oilSampleSchedule.due(now, OIL_SAMPLE_PERIOD_MS, skipped)) {
    diagnostics::saturatingAdd(g_oilSkippedSamples, skipped);
    sampleOil();
    // Detect a slow acquisition before filtering or forwarding its value.
    if (g_oilServiceGap.recordDuration(micros() - entryUs)) {
      timingFault = true;
      invalidateOilTiming();
    }
    now = millis();
    if (g_oilFlags) {
      g_oilPsi = NAN;
      g_oilHaveValidSample = false;
    } else {
      g_oilPsi = g_oilHaveValidSample ?
          g_oilPsi + ADC_IIR_ALPHA * (g_oilReading.psi - g_oilPsi) : g_oilReading.psi;
      g_oilHaveValidSample = true;
    }
  }
  const bool due = g_oilPublishSchedule.due(now, g_oilPublishPeriodMs, skipped);
  diagnostics::saturatingAdd(g_oilSkippedPublishes, skipped);
  if (due || timingFault) {
    const uint16_t pressureTenths = g_oilFlags || !isfinite(g_oilPsi) ? 0xFFFF :
        static_cast<uint16_t>(clampFloat(g_oilPsi * 10, 0, 1500) + 0.5f);
    uint8_t payload[8] = {static_cast<uint8_t>(pressureTenths >> 8),
                         static_cast<uint8_t>(pressureTenths), g_oilFlags, 0, 0, 0, 0, 0};
    publishVirtualCan(0x710u, payload, sizeof(payload), now);
  }
}

void reset(uint64_t start=0){
 clockUs=start;events.clear();sampleEntries.clear();sampleReturns.clear();acquisitionDelayUs=0;inputPsi=100;inputFlags=0;
 g_oilSampleSchedule={};g_oilPublishSchedule={};g_oilServiceGap=timing::ServiceGapMonitor{OIL_SERVICE_LIMIT_US};
 g_oilFaultRecovery={};g_oilReading={};g_oilFlags=32;g_oilPsi=NAN;g_oilHaveValidSample=false;
 g_oilSkippedSamples=g_oilSkippedPublishes=0;g_oilPublishPeriodMs=20;
}
void serviceAt(uint64_t t){clockUs=t;serviceOil(static_cast<uint32_t>(t/1000u)-123u);}
int main(int argc,char**argv){
 if(argc==5&&std::string(argv[1])=="--trace"){
  const unsigned pattern=std::stoul(argv[2]),duration=std::stoul(argv[3]),length=std::stoul(argv[4]);
  reset();acquisitionDelayUs=duration;uint32_t rng=1234567u+pattern;uint64_t entry=0;
  while(entry<=length*1000ull){
   clockUs=entry;serviceOil(millis());
   rng=1664525u*rng+1013904223u;
   const uint32_t gap=pattern<5?(pattern+1)*1000u:1000u+(rng%5u)*1000u;
   entry=std::max(entry+gap,clockUs);
  }
  check(g_oilServiceGap.violations()==0);
  for(size_t i=0;i<sampleEntries.size();++i)std::printf("S,%llu,%llu\n",(unsigned long long)sampleEntries[i],(unsigned long long)sampleReturns[i]);
  for(const auto&e:events)std::printf("P,%u\n",e.timestamp);
  return 0;
 }
 for(unsigned d:{0u,1000u,5000u,6000u,20000u,100000u}){
  reset();for(unsigned t=0;t<20;++t)serviceAt(t*1000ull);acquisitionDelayUs=d;serviceAt(20000);
  check(!events.empty());const auto e=events.back();check(e.timestamp==(20000u+d)/1000u);
  if(d>5000){check(e.pressure==0xFFFF&&bool(e.flags&128));check(!g_oilHaveValidSample);check(g_oilServiceGap.violations()==1);}
  else {check(e.pressure==1000&&e.flags==0);check(g_oilServiceGap.violations()==0);}
 }
 reset();for(unsigned t=0;t<=21;++t)serviceAt(t*1000ull);const auto before=events.size();serviceAt(28000);
 check(events.size()==before+1&&events.back().pressure==0xFFFF&&(events.back().flags&128));
 for(unsigned t=29;t<=49;++t)serviceAt(t*1000ull);
 check(g_oilFlags&128);serviceAt(50000);check(g_oilFlags==0&&g_oilPsi==100.0f);
 reset();for(unsigned t=0;t<=20;++t)serviceAt(t*1000ull);inputFlags=2;for(unsigned t=21;t<=30;++t)serviceAt(t*1000ull);
 check(g_oilFlags==2&&std::isnan(g_oilPsi));inputFlags=0;inputPsi=0;
 for(unsigned t=31;t<=59;++t)serviceAt(t*1000ull);check(g_oilFlags==2);
 serviceAt(60000);check(g_oilFlags==0&&g_oilPsi==0&&events.back().pressure==0);
 for(unsigned gap=1;gap<=5;++gap){
  reset();for(uint64_t t=0;t<=120000000;t+=gap*1000u)serviceAt(t);
  check(sampleEntries.size()==12000&&events.size()==6000);check(g_oilServiceGap.violations()==0);
  for(size_t i=1;i<events.size();++i)check(events[i].timestamp-events[i-1].timestamp<=24u);
  for(const auto&e:events)check(e.flags==0&&e.pressure==1000);
 }
 for(uint64_t start:{0xFFFFFF00ull,0xFFFFFFFFull*1000ull-15000ull}){
  reset(start);for(uint64_t t=start;t<=start+100000;t+=1000)serviceAt(t);
  check(g_oilServiceGap.violations()==0&&events.size()==5);for(const auto&e:events)check(e.flags==0);
 }
 std::printf("PASS actual oil scheduler %llu checks\n",(unsigned long long)checks);
}
