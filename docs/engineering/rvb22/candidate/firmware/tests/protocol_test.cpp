#include "../cca_telemetry/src/gps_nmea.h"
#include "../cca_telemetry/src/racechrono_codec.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

std::string sentence(const std::string& body) {
  unsigned sum = 0;
  for (auto c : body) sum ^= static_cast<unsigned char>(c);
  char suffix[4]; std::snprintf(suffix, sizeof(suffix), "*%02X", sum);
  return "$" + body + suffix;
}
bool rmc(const std::string& body, gps::RmcData& out) {
  auto value = sentence(body);
  return gps::parseRmcSentence(value.data(), out);
}
int main() {
  unsigned vectors = 0;
  auto check = [&](bool condition) { assert(condition); ++vectors; };
  gps::RmcData r;
  check(rmc("GPRMC,123519.100,A,4807.038,N,01131.000,E,022.4,084.4,070926,,,A",r));
  check(r.valid && r.millis == 100 && r.year == 2026);
  check(std::abs(r.latitude_deg - 48.1173) < 1e-7);
  check(std::abs(r.longitude_deg - 11.5166666667) < 1e-7);
  check(std::abs(r.speed_kmh - 41.4848) < 1e-5);
  check(!rmc("GPRMC,246000.0,A,4807.038,N,01131.000,E,0,0,070926,,,A",r));
  check(!rmc("GPRMC,120000.0,A,4807.038,N,01131.000,E,0,0,310226,,,A",r));
  check(rmc("GNRMC,235959.900,A,4807.038,S,01131.000,W,0,0,070926,,,A",r));
  check(r.latitude_deg < 0 && r.longitude_deg < 0 && r.millis == 900);
  check(rmc("GPRMC,120000.0,A,4861.000,N,01131.000,E,0,0,070926,,,A",r) && !r.valid);
  check(rmc("GPRMC,120000.0,A,,,,,0,0,070926,,,A",r) && !r.valid);
  check(rmc("GPRMC,120000.0,V,,,,,0,0,070926,,,N",r) && !r.valid);
  check(!rmc("GPGGA,120000.0,A,4807.038,N,01131.000,E,0,0,070926,,,A",r));
  // Bounded inputs have no hidden trailing storage for checksum bytes.
  const char truncated[] = {'$', 'A', '*'};
  check(!gps::checksumOk(truncated, sizeof(truncated)));
  const char truncated2[] = {'$', 'A', '*', '4'};
  check(!gps::checksumOk(truncated2, sizeof(truncated2)));
  check(gps::checksumOk("$A*41", 5));
  check(!gps::checksumOk("$A*40", 5));
  check(!gps::checksumOk("$A*41X", 6));
  uint8_t antenna = 0;
  for (int i = 1; i <= 3; ++i) {
    auto a = sentence("PCD,9," + std::to_string(i));
    check(gps::parseAntennaStatus(a.c_str(), antenna) && antenna == i);
  }
  auto bad = sentence("PCD,9,4");
  check(!gps::parseAntennaStatus(bad.c_str(), antenna));
  auto legacy = sentence("PGTOP,11,3");
  check(!gps::parseAntennaStatus(legacy.c_str(), antenna));
  uint16_t command = 0; uint8_t flag = 0;
  auto ack = sentence("PMTK001,220,3");
  check(gps::parsePmtkAck(ack.c_str(), command, flag) && command == 220 && flag == 3);
  for (auto body : {"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0", "PMTK220,100", "PMTK251,115200", "CDCMD,9,0"}) {
    auto a = sentence(body); check(gps::checksumOk(a.c_str(), a.size()));
  }
  check(sentence("CDCMD,9,0") == "$CDCMD,9,0*44");
  check(sentence("PMTK220,100") == "$PMTK220,100*2F");
  check(sentence("PMTK251,115200") == "$PMTK251,115200*1F");
  check(racechrono::altitudeWord(-500) == 0);
  check(racechrono::altitudeWord(0) == 5000);
  check(racechrono::altitudeWord(2776.7) == 0x7FFF);
  check(racechrono::altitudeWord(3000) == (0x8000 | 3500));
  check(racechrono::altitudeWord(-501) == 0xFFFF);
  check(racechrono::altitudeWord(NAN) == 0xFFFF);
  check(racechrono::speedWord(327.67) == 0x7FFF);
  check(racechrono::speedWord(400) == (0x8000 | 4000));
  check(racechrono::speedWord(-1) == 0xFFFF);
  check(racechrono::speedWord(INFINITY) == 0xFFFF);
  check(racechrono::bearingWord(359.999) == 0);
  check(racechrono::bearingWord(360) == 0xFFFF);
  // Dense independent decoding sweep catches the old saturation discontinuities.
  for (double speed = 0; speed <= 1200; speed += 0.37) {
    const auto word = racechrono::speedWord(speed);
    const double decoded = (word & 0x7FFF) / ((word & 0x8000) ? 10.0 : 100.0);
    check(std::abs(decoded - speed) <= ((word & 0x8000) ? 0.050001 : 0.005001));
  }
  for (double altitude = -500; altitude <= 9000; altitude += 1.17) {
    const auto word = racechrono::altitudeWord(altitude);
    const double decoded = (word & 0x7FFF) / ((word & 0x8000) ? 1.0 : 10.0) - 500;
    check(std::abs(decoded - altitude) <= ((word & 0x8000) ? 0.500001 : 0.050001));
  }
  std::printf("PASS: %u protocol/parser/encoding assertions; address/undefined sanitizer enabled.\n", vectors);
}
