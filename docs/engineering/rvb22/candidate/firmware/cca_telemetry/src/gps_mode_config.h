#pragma once
#include <cstdint>
#include <cstring>
#include "gps_nmea.h"
namespace gps {
// GPS L1 accessory allocation; PA1616D SBAS mode is limited to <=5Hz.
inline constexpr const char* kGpsOnly = "$PMTK353,1,0,0,0,0*2A\r\n";
inline constexpr const char* kSbasOff = "$PMTK313,0*2F\r\n";
inline constexpr const char* kDgpsNone = "$PMTK301,0*2C\r\n";
inline constexpr const char* kSbasQuery = "$PMTK413*34\r\n";
struct ModeEvidence {
  uint8_t ack353=0xFF, ack313=0xFF, ack301=0xFF, sbas=0xFF;
  void observeAck(uint16_t command, uint8_t result) {
    if (command==353) ack353=result;
    else if (command==313) ack313=result;
    else if (command==301) ack301=result;
  }
  bool confirmed() const { return ack353==3 && ack313==3 && ack301==3 && sbas==0; }
};
inline bool parseSbasReadback(const char* line, uint8_t& state) {
  if (!line || std::strlen(line)!=13 || std::strncmp(line,"$PMTK513,",9)!=0 ||
      (line[9]!='0' && line[9]!='1') || line[10]!='*' || !checksumOk(line,13)) return false;
  state=static_cast<uint8_t>(line[9]-'0'); return true;
}
}
