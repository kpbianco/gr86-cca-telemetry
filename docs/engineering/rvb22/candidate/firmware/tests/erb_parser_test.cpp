#include "../cca_telemetry/src/gps_nmea.h"
#include <string>
#include <cstdio>
#include <cmath>
static std::string nmea(const std::string& body) {
  unsigned c=0; for(unsigned char x:body)c^=x;
  char end[4]; std::snprintf(end,sizeof(end),"*%02X",c); return "$"+body+end;
}
int main(){
  unsigned failures=0,checks=0;
  auto check=[&](bool ok,const char* label){++checks; if(!ok){++failures;std::printf("FAIL %s\n",label);}};
  for(const char* speed:{"junk","12junk","1e2","","-1","nan","inf"}){
    auto s=nmea(std::string("GPRMC,123519.100,A,4807.038,N,01131.000,E,")+speed+",84.4,070926,,,A");
    gps::RmcData r; bool parsed=gps::parseRmcSentence(s.data(),r);
    check(!parsed||!std::isfinite(r.speed_kmh),speed);
  }
  for(const char* course:{"junk","84junk","","-1","360"}){
    auto s=nmea(std::string("GPRMC,123519.100,A,4807.038,N,01131.000,E,22.4,")+course+",070926,,,A");
    gps::RmcData r; bool parsed=gps::parseRmcSentence(s.data(),r);
    check(!parsed||!std::isfinite(r.course_deg),course);
  }
  for(const char* utc:{"123519.10junk","123519..100","123519."}){
    auto s=nmea(std::string("GPRMC,")+utc+",A,4807.038,N,01131.000,E,22.4,84.4,070926,,,A");
    gps::RmcData r; check(!gps::parseRmcSentence(s.data(),r)||!r.valid,utc);
  }
  for(const char* altitude:{"junk","12junk","nan","inf",""}){
    auto s=nmea(std::string("GPGGA,123519.100,4807.038,N,01131.000,E,1,08,0.9,")+altitude+",M,0,M,,");
    gps::GgaData r; bool parsed=gps::parseGgaSentence(s.data(),r); check(!parsed||!r.has_altitude,altitude);
  }
  for(const char* hdop:{"junk","0.9junk","nan","inf","-1",""}){
    auto s=nmea(std::string("GPGGA,123519.100,4807.038,N,01131.000,E,1,08,")+hdop+",10,M,0,M,,");
    gps::GgaData r; bool parsed=gps::parseGgaSentence(s.data(),r); check(!parsed||!r.has_hdop,hdop);
  }
  std::printf("%u checks, %u failures\n",checks,failures);return failures?1:0;
}
