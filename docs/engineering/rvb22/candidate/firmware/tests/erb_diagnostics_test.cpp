#include "../cca_telemetry/src/boot_diagnostics.h"
#include "../cca_telemetry/src/oil_model.h"
#include "../cca_telemetry/src/runtime_timing.h"
#include <cassert>
#include <cstdio>
int main(){
 unsigned n=0;auto check=[&](bool v){assert(v);++n;};
 diagnostics::BootRecord r{};
 check(!diagnostics::valid(r));r=diagnostics::nextBoot(r,false);check(r.count==1&&diagnostics::valid(r));
 r=diagnostics::nextBoot(r,false);check(r.count==2&&diagnostics::valid(r));
 r=diagnostics::nextBoot(r,true);check(r.count==1);
 r.inverse^=1;r=diagnostics::nextBoot(r,false);check(r.count==1&&diagnostics::valid(r));
 r={diagnostics::kBootMagic,UINT32_MAX,0};r=diagnostics::nextBoot(r,false);check(r.count==UINT32_MAX&&diagnostics::valid(r));
 r.magic^=1;r=diagnostics::nextBoot(r,false);check(r.count==1);
 oil::Calibration c;c.deviceId=0x112233445566ull;
 check(oil::identityMatches(c,0x112233445566ull));check(!oil::identityMatches(c,0x223344556677ull));
 check(!oil::identityMatches(c,0));c.version=3;check(!oil::identityMatches(c,0x112233445566ull));
 c.version=4;c.deviceId=0;check(!oil::identityMatches(c,0));
 timing::Backoff b;
 check(!b.pending(0));check(!b.pending(0x80000000u));check(!b.pending(UINT32_MAX));
 b.start(1000,100);check(b.pending(1099));check(!b.pending(1100));
 check(!b.pending(0x80001000u));
 b.start(0xFFFFFFF0u,100);check(b.pending(0xFFFFFFFFu));check(b.pending(0));
 check(b.pending(83));check(!b.pending(84));check(!b.pending(0x80000100u));
 b.start(20,100);b.clear();check(!b.pending(21));
 std::printf("PASS: %u RTC-record, calibration identity and BLE backoff wrap checks.\n",n);
}
