from pathlib import Path
import hashlib,json,subprocess,os,re
import argparse
ap=argparse.ArgumentParser(description='Host-test selected actual firmware functions with explicit stubs and ASan/UBSan.')
ap.add_argument('--firmware-dir',type=Path,required=True)
ap.add_argument('--out',type=Path,required=True)
args=ap.parse_args();L=args.out.resolve();L.mkdir(parents=True,exist_ok=True);C=args.firmware_dir.resolve();H=L/'recovered_tests';H.mkdir(exist_ok=True)
S=(C/'cca_telemetry/cca_telemetry.ino').read_text()
def function(name):
    m=re.search(r'^static [^\n]*\b'+re.escape(name)+r'\(',S,re.M)
    assert m,name
    a=m.start();i=S.index('{',m.end());depth=1;j=i+1
    # Functions selected here contain no braces inside their literal strings.
    while depth:
        depth+=(S[j]=='{')-(S[j]=='}');j+=1
    return S[a:j]+'\n'
def section(start,end):return S[S.index(start):S.index(end,S.index(start))]
COMMON='''#include <algorithm>
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
void check(bool ok){if(!ok){std::fprintf(stderr,"FAILED check %llu\\n",(unsigned long long)checks);std::abort();}++checks;}
struct Log {template<class...T>void printf(const char*,T...){};void println(const char*){};}g_log;
'''
oil=COMMON+function('clampFloat')+'''
static uint16_t g_oilPublishPeriodMs=20;
'''+section('// 100 Hz acquisition','static float medianAdcVolts')+'''
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
'''+function('invalidateOilTiming')+function('serviceOil')+'''
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
  for(size_t i=0;i<sampleEntries.size();++i)std::printf("S,%llu,%llu\\n",(unsigned long long)sampleEntries[i],(unsigned long long)sampleReturns[i]);
  for(const auto&e:events)std::printf("P,%u\\n",e.timestamp);
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
 std::printf("PASS actual oil scheduler %llu checks\\n",(unsigned long long)checks);
}
'''
(H/'actual_oil.cpp').write_text(oil)

gps=COMMON+'''
#define GPS_PPS_GPIO -1
enum hardwareSerial_error_t{UART_FIFO_OVF_ERROR,UART_BUFFER_FULL_ERROR};
static constexpr int SERIAL_8N1=0,GPS_RX_GPIO=18,GPS_TX_GPIO=17;
struct HardwareSerial{
 int space=256;size_t calls=0;bool shortWrite=false;std::vector<std::string> writes;std::string input;size_t cursor=0;
 HardwareSerial(int){} int availableForWrite(){return space;}
 size_t write(const uint8_t*p,size_t n){++calls;size_t accepted=shortWrite&&n?n-1:n;writes.emplace_back((const char*)p,accepted);return accepted;}
 void end(){}size_t setRxBufferSize(size_t n){return n;}void setTxBufferSize(size_t){}
 void onReceiveError(void(*)(hardwareSerial_error_t)){}void begin(uint32_t,int,int,int){}
 int available(){return static_cast<int>(input.size()-cursor);}int read(){return cursor<input.size()?static_cast<unsigned char>(input[cursor++]):-1;}
};
'''+function('timeReached')+section('static HardwareSerial g_gpsSerial','static void IRAM_ATTR onGpsPps')+'''
#endif
'''+function('millisSinceMidnight')+function('startGpsProbe')+function('enterGpsRetryWait')+function('parseGpsSentence')+function('readGpsBytes')+function('serviceGpsStateMachine')+'''
std::string sentence(const std::string& body){unsigned c=0;for(char x:body)c^=x;char tail[8];std::snprintf(tail,sizeof(tail),"*%02X",c);return "$"+body+tail;}
void feed(const std::string& body,uint32_t now){auto line=sentence(body);clockUs=now*1000ull;parseGpsSentence(line.c_str(),now);}
void reset(){g_gpsSolutionEpoch={};g_gpsRmcSequence=0;g_gpsDuplicateRmcEpochs=g_gpsBackwardRmcEpochs=0;g_gpsSerial.input.clear();g_gpsSerial.cursor=0;g_gpsSerial.space=256;g_gpsSerial.shortWrite=false;g_gpsSerial.calls=0;g_gpsSerial.writes.clear();g_gpsValidFix=false;g_gpsTxDeferred=g_gpsTxShortWrites=0;startGpsProbe(0,0);}
void rmc(unsigned t,uint32_t now,const char* talker="GPRMC",const char* validity="A"){
 char b[180];std::snprintf(b,sizeof(b),"%s,1200%02u.%03u,%s,3745.000,N,12227.000,W,0.0,0.0,090926,,,A",talker,(t/1000)%60,t%1000,validity);feed(b,now);
}
void mode(){feed("PMTK001,353,3",0);feed("PMTK001,313,3",0);feed("PMTK001,301,3",0);feed("PMTK001,220,3",0);feed("PMTK513,0",0);}
int main(){
 reset();const char*cmd=gps::kGpsOnly;const size_t n=std::strlen(cmd);
 for(int room=-1;room<static_cast<int>(n);++room){g_gpsSerial.space=room;check(!queueGpsCommand(cmd));check(g_gpsSerial.calls==0);}
 g_gpsSerial.space=n;check(queueGpsCommand(cmd));check(g_gpsSerial.writes.back()==cmd);
 g_gpsSerial.shortWrite=true;check(!queueGpsCommand(cmd)&&g_gpsTxShortWrites==1);g_gpsSerial.shortWrite=false;
 reset();g_gpsCurrentBaud=9600;g_gpsSentenceSeenAtCurrentBaud=true;g_gpsSerial.space=0;
 serviceGpsStateMachine(2000);check(g_gpsState==GpsState::Probe&&g_gpsSerial.calls==0);
 g_gpsSerial.space=256;serviceGpsStateMachine(2001);check(g_gpsState==GpsState::SwitchTo115200);check(g_gpsSerial.writes.back()==PMTK_115200);
 reset();g_gpsSentenceSeenAtCurrentBaud=true;serviceGpsStateMachine(0);check(g_gpsState==GpsState::Configure);
 g_gpsSerial.space=0;serviceGpsStateMachine(0);check(g_gpsConfigCommandIndex==0);
 g_gpsSerial.space=256;for(unsigned i=0;i<8;++i)serviceGpsStateMachine(i*150);check(g_gpsState==GpsState::Active);check(g_gpsSerial.writes.size()==7);
 for(const auto&w:g_gpsSerial.writes){auto x=w.substr(0,w.size()-2);check(gps::checksumOk(x.c_str(),x.size()));}
 reset();for(unsigned i=0;i<=10;++i)rmc(i*100,i*100);check(!g_gpsConfigured);check(g_gpsRmcSequence==11);
 reset();mode();for(unsigned i=0;i<=10;++i)rmc(i*100,i*100);check(g_gpsConfigured&&g_gpsValidFix);
 const auto seq=g_gpsRmcSequence,capture=g_rmcCaptureMillis;rmc(1000,1800,"GNRMC");check(g_gpsRmcSequence==seq&&g_rmcCaptureMillis==capture&&g_gpsDuplicateRmcEpochs==1);
 rmc(900,1900);check(!g_gpsValidFix&&!g_gpsConfigured&&g_gpsBackwardRmcEpochs==1);
 feed("PMTK513,1",2000);check(!g_gpsModeEvidence.confirmed());
 for(unsigned missing=0;missing<4;++missing){reset();mode();if(missing==0)g_gpsModeEvidence.ack353=255;if(missing==1)g_gpsModeEvidence.ack313=2;if(missing==2)g_gpsModeEvidence.ack301=255;if(missing==3)g_gpsRateAck=255;for(unsigned i=0;i<=10;++i)rmc(i*100,i*100);check(!g_gpsConfigured);}
 uint8_t v=255;
 for(const auto&body:{"PMTK513,0","PMTK513,1"}){auto x=sentence(body);check(gps::parseSbasReadback(x.c_str(),v));}
 for(const auto&body:{"PMTK513,2","PMTK513,00","PMTK513,0,0","PMTK513,","PMTK513,-1"}){auto x=sentence(body);check(!gps::parseSbasReadback(x.c_str(),v));}
 auto bad=sentence("PMTK513,0");bad.back()=bad.back()=='0'?'1':'0';check(!gps::parseSbasReadback(bad.c_str(),v));
 reset();std::string x=sentence("GPRMC,120000.000,A,3745.000,N,12227.000,W,0.0,0.0,090926,,,A");
 g_gpsSerial.input=x+std::string(1,'\\0')+"hidden\\n"+x+"\\r\\n";readGpsBytes(10);check(g_gpsRmcSequence==1&&g_gpsInvalidCharacters==1);
 reset();uint32_t random=1234;
 for(unsigned batch=0;batch<2000;++batch){g_gpsSerial.input.clear();g_gpsSerial.cursor=0;for(unsigned i=0;i<512;++i){random=random*1664525u+1013904223u;g_gpsSerial.input+=static_cast<char>(random>>24);}readGpsBytes(batch);check(g_gpsLineLength<sizeof(g_gpsLine));}
 std::printf("PASS actual GPS state/command/epoch/parser %llu checks\\n",(unsigned long long)checks);
}
'''
(H/'actual_gps.cpp').write_text(gps)

cache=COMMON+'''
struct NimBLECharacteristic{std::vector<std::pair<uint32_t,std::vector<uint8_t>>> sent;std::vector<uint8_t>pending;bool accept=true;void setValue(const uint8_t*p,size_t n){pending.assign(p,p+n);}bool notify(){if(!accept)return false;sent.push_back({millis(),pending});return true;}};
static NimBLECharacteristic canObject,gpsObject,timeObject;
static NimBLECharacteristic* g_canCharacteristic=&canObject;
static bool g_canSubscribed=true;
static bool isBleConnected(){return true;}
static uint32_t g_canSlotEvictions=0,g_canCacheCoalesced=0,g_canStaleDropCount=0;
static constexpr uint32_t CAN_MAX_CACHE_AGE_MS=500;
struct RouteDecision{bool allowed;uint32_t minimumIntervalMs;};
static RouteDecision routeDecision(uint32_t pid){return {true,pid==0x710?20u:10u};}
'''+section('static uint32_t g_bleNotifySuccesses','static bool isBleConnected()')+function('takeBleToken')+function('notifyCharacteristic')+section('struct CanSlot {','static void updateCanStatus')+function('sendCanSlot')+function('serviceCanForwarding')+'''
void reset(){discardCanCache();g_bleBudget=io::TokenBucket(BLE_TOKEN_CAPACITY,BLE_TOKEN_RATE_PER_SECOND);g_bleNotifyBackoff={};g_bleNotifySuccesses=g_bleNotifyFailures=g_bleNotifyRateDrops=0;canObject.sent.clear();gpsObject.sent.clear();timeObject.sent.clear();canObject.accept=true;}
int main(){
 reset();uint8_t data[8]={};cacheCanFrame(0x710,false,data,8,1);cacheCanFrame(0x777,false,data,8,1);
 for(unsigned i=0;i<5000;++i)cacheCanFrame(0x100+i%180,false,data,8,i+2);
 check(findCanSlot(0x710,false)&&findCanSlot(0x777,false));check(g_canSlotEvictions>0);
 reset();unsigned arrivals=0;uint8_t gpsData[20]={},timeData[3]={};
 for(unsigned t=0;t<120000;++t){clockUs=t*1000ull;
  if(t%100==0)check(notifyCharacteristic(&gpsObject,true,gpsData,20,t));
  if(t%1000==0)check(notifyCharacteristic(&timeObject,true,timeData,3,t));
  if(t%20==0)publishVirtualCan(0x710,data,8,t);
  if(t%2000==0)publishVirtualCan(0x777,data,8,t);
  const unsigned target=(t+1)*1800/1000;
  while(arrivals<target){data[0]=arrivals;cacheCanFrame(0x100+arrivals%180,false,data,8,t);++arrivals;}
  serviceCanForwarding(t);
 }
 unsigned oilCount=0,lastOil=0,maxOilGap=0;for(const auto&e:canObject.sent){const auto&v=e.second;if(v.size()>=4&&v[0]==0x10&&v[1]==7){if(oilCount)maxOilGap=std::max(maxOilGap,e.first-lastOil);lastOil=e.first;++oilCount;}}
 check(oilCount==6000&&maxOilGap==20);check(gpsObject.sent.size()==1200&&timeObject.sent.size()==120);
 check(g_bleNotifySuccesses<=120u*120u+24u);check(arrivals==216000);
 std::printf("TRAFFIC oil=%u gps=%zu time=%zu total=%u oil_max_gap=%u physical_arrivals=%u\\n",oilCount,gpsObject.sent.size(),timeObject.sent.size(),g_bleNotifySuccesses,maxOilGap,arrivals);
 reset();clockUs=1000;data[0]=1;publishVirtualCan(0x710,data,8,1);canObject.accept=false;serviceCanForwarding(1);check(findCanSlot(0x710,false)->dirty);
 data[0]=2;publishVirtualCan(0x710,data,8,2);clockUs=100000;canObject.accept=true;serviceCanForwarding(100);check(canObject.sent.empty());
 clockUs=101000;serviceCanForwarding(101);check(canObject.sent.size()==1&&canObject.sent.back().second[4]==2);
 std::printf("PASS actual cache/notification/backpressure %llu checks\\n",(unsigned long long)checks);
}
'''
(H/'actual_cache.cpp').write_text(cache)

boundary=COMMON+'''
int main(){oil::Calibration c;c.signal={{0,2.75f},{0,5.5f},.001f,3};c.excitation=c.signal;check(oil::ready(c));
 auto read=[&](float r,float v=5.0f){return oil::evaluate(r*v*.5f,v*.5f,c);};
 for(unsigned i=0;i<=10000;++i){float r=.05f+i*.00009f;auto x=read(r);bool bad=x.ratio<.08f||x.ratio>.92f;check(bool(x.flags&4)==bad);if(!bad)check(!x.flags&&std::isfinite(x.psi));}
 for(float r:{.0837f,.9163f})check(read(r).flags==0);
 check(read(.025f).flags&2);check(read(.975f).flags&1);
 c.pressurePoints=3;
 for(float z:{.08f,.1f,.12f})for(float f:{.88f,.9f,.92f}){c.ratio0=z;c.ratio150=f;check(oil::ready(c));}
 c.ratio0=std::nextafter(.08f,-INFINITY);check(!oil::ready(c));c.ratio0=std::nextafter(.12f,INFINITY);check(!oil::ready(c));c.ratio0=.1f;
 c.ratio150=std::nextafter(.88f,-INFINITY);check(!oil::ready(c));c.ratio150=std::nextafter(.92f,INFINITY);check(!oil::ready(c));c.ratio150=.9f;
 c.pressurePoints=0;c.ratio0=.11f;check(!oil::ready(c));c.ratio0=.1f;check(oil::ready(c));
 c.signal.maxError=c.excitation.maxError=.025f;check(read(.9f).flags&64);c.signal.maxError=c.excitation.maxError=.001f;
 check(read(.5f,4.756f).flags&16);check(!(read(.5f,4.758f).flags&16));
 c.pressurePoints=3;c.ratio0=.12f;c.ratio150=.88f;
 c.signal.maxError=c.excitation.maxError=.00585f*5/(1.5f+.00585f);
 check(read(.5f).flags&64);check(std::fabs(oil::maximumRatioError(c)-.0057f)<1e-8f);
 c.ratio0=.1f;c.ratio150=.9f;check(read(.5f).flags==0);
 c.ratio0=.08f;c.ratio150=.92f;check(oil::maximumRatioError(c)==.006f);
 c.signal.maxError=c.excitation.maxError=.00601f*5/(1.5f+.00601f);check(read(.5f).flags&64);
 runtime_config_not_used:;
 std::printf("PASS oil sensor/calibration boundary %llu checks\\n",(unsigned long long)checks);
}
'''
(H/'oil_boundaries.cpp').write_text(boundary)

results=[]
include=['-I'+str(C/'cca_telemetry/src')]
cases=[('actual_oil',H/'actual_oil.cpp',[]),('actual_gps',H/'actual_gps.cpp',[C/'cca_telemetry/src/gps_nmea.cpp']),('actual_cache',H/'actual_cache.cpp',[]),('oil_boundaries',H/'oil_boundaries.cpp',[])]
for name in ['oil_model_test','io_budget_test','protocol_test','erb_diagnostics_test','erb_parser_test']:
    cases.append((name,C/'tests'/f'{name}.cpp',[C/'cca_telemetry/src/gps_nmea.cpp'] if name in ['erb_parser_test','protocol_test'] else []))
for name,src,extras in cases:
    command=['g++','-std=c++17','-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer','-Wall','-Wextra',*include,str(src),*[str(x) for x in extras],'-o',str(H/name)]
    comp=subprocess.run(command,capture_output=True,text=True)
    item={'name':name,'source_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),'compile_command':command,'compile_returncode':comp.returncode,'compile_output':comp.stdout+comp.stderr}
    if comp.returncode==0:
        run=subprocess.run([str(H/name)],capture_output=True,text=True,env={**os.environ,'ASAN_OPTIONS':'detect_leaks=0'},timeout=60)
        item.update(returncode=run.returncode,output=run.stdout+run.stderr,passed=run.returncode==0)
    else:item['passed']=False
    results.append(item);print(name,'PASS' if item['passed'] else 'FAIL',item.get('output','')[-300:])
report={'status':'PASS' if all(x['passed'] for x in results) else 'FAIL','build':'2.1.4-revb-recovered-20260909','sketch_sha256':hashlib.sha256((C/'cca_telemetry/cca_telemetry.ino').read_bytes()).hexdigest(),'source_files_sha256':{str(p.relative_to(C)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (C/'cca_telemetry').rglob('*') if p.is_file()},'sanitizers':{'address':True,'undefined_behavior':True,'leak':False,'leak_not_run_reason':'This harness explicitly runs ASan/UBSan without LeakSanitizer; leak testing is not claimed.'},'results':results,'scope':'Host compilation of selected actual source functions with explicit driver/ADC/time stubs plus retained parser/model tests; no native target image or driver WCET, radio/phone, hydraulic or assembly pass.'}
(L/'RECOVERED_FIRMWARE_TEST_RESULTS.json').write_text(json.dumps(report,indent=2)+'\n')
raise SystemExit(0 if report['status']=='PASS' else 1)
