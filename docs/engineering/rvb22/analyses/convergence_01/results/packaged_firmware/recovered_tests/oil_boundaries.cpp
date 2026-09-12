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
 std::printf("PASS oil sensor/calibration boundary %llu checks\n",(unsigned long long)checks);
}
