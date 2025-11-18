#include "../firmware/src/gps_nmea.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <random>
#include <string>
#include <vector>

namespace {

void expectNear(double lhs, double rhs, double tol = 1e-3) {
  assert(std::fabs(lhs - rhs) <= tol);
}

void test_valid_rmc() {
  char line[] = "$GPRMC,123519.123,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*74";
  gps::RmcData data;
  bool ok = gps::parseRmcSentence(line, data);
  assert(ok);
  assert(data.has_time);
  assert(data.hour == 12);
  assert(data.minute == 35);
  assert(data.second == 19);
  assert(data.millis == 123);
  assert(data.valid);
  assert(data.has_latitude);
  expectNear(data.latitude_deg, 48.1173, 1e-4);
  assert(data.has_longitude);
  expectNear(data.longitude_deg, 11.516666, 1e-4);
  expectNear(data.speed_kmh, 41.4848, 1e-4);
  expectNear(data.course_deg, 84.4, 1e-4);
  assert(data.has_date);
  assert(data.day == 23);
  assert(data.month == 3);
  assert(data.year == 2094);
}

void test_valid_gga() {
  char line[] = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
  gps::GgaData data;
  bool ok = gps::parseGgaSentence(line, data);
  assert(ok);
  assert(data.has_sats);
  assert(data.sats == 8);
  assert(data.has_hdop);
  expectNear(data.hdop, 0.9, 1e-4);
  assert(data.has_altitude);
  expectNear(data.altitude_m, 545.4, 1e-4);
}

void test_rmc_clamp_and_missing_fields() {
  char line[] = "$GPRMC,235959.9999,A,,,,,,,010203,,*09";
  gps::RmcData data;
  bool ok = gps::parseRmcSentence(line, data);
  assert(ok);
  assert(data.has_time);
  assert(data.hour == 23);
  assert(data.minute == 59);
  assert(data.second == 59);
  assert(data.millis == 999);
  assert(data.valid);
  assert(!data.has_latitude);
  assert(!data.has_longitude);
  assert(data.has_date);
  assert(data.day == 1);
  assert(data.month == 2);
  assert(data.year == 2003);
}

void test_gga_clamp() {
  char line[] = "$GPGGA,010101,4807.038,N,01131.000,E,1,99,33.3,10.0,M,0.0,M,,*73";
  gps::GgaData data;
  bool ok = gps::parseGgaSentence(line, data);
  assert(ok);
  assert(data.has_sats);
  assert(data.sats == 63);
  assert(data.has_hdop);
  expectNear(data.hdop, 25.4, 1e-4);
  assert(data.has_altitude);
  expectNear(data.altitude_m, 10.0, 1e-4);
}

void test_missing_checksum() {
  char line[] = "$GPRMC,123519,A,,,,,,,,,";
  gps::RmcData data;
  bool ok = gps::parseRmcSentence(line, data);
  assert(!ok);
}

void test_extra_commas() {
  char line[] = "$GPRMC,123519,V,,,,,,,,,,*10";
  gps::RmcData data;
  bool ok = gps::parseRmcSentence(line, data);
  assert(ok);
  assert(!data.valid);
  assert(!data.has_latitude);
  assert(!data.has_longitude);
}

void test_line_too_long() {
  std::string payload = "$GPRMC," + std::string(115, 'A') + "*00";
  gps::RmcData data;
  bool ok = gps::parseRmcSentence(payload.data(), data);
  assert(!ok);
}

char randomFieldChar(std::mt19937 &rng) {
  static const char charset[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ,.-";
  std::uniform_int_distribution<std::size_t> dist(0, sizeof(charset) - 2);
  return charset[dist(rng)];
}

std::string makeRandomSentence(std::mt19937 &rng, bool rmc) {
  const std::string prefix = rmc ? "GPRMC," : "GPGGA,";
  std::uniform_int_distribution<int> extraDist(0, static_cast<int>(gps::kMaxSentenceLength) - 4 - static_cast<int>(prefix.size()));
  int extraLen = extraDist(rng);
  std::string body = prefix;
  body.reserve(prefix.size() + extraLen);
  for (int i = 0; i < extraLen; ++i) {
    char c = randomFieldChar(rng);
    if (c == '$' || c == '*') c = ',';
    body.push_back(c);
  }
  std::string sentence;
  sentence.reserve(body.size() + 4);
  sentence.push_back('$');
  sentence += body;
  sentence.push_back('*');
  sentence.push_back('0');
  sentence.push_back('0');
  unsigned checksum = 0;
  for (std::size_t i = 1; i < sentence.size() - 3; ++i) {
    checksum ^= static_cast<unsigned char>(sentence[i]);
  }
  const char hex[] = "0123456789ABCDEF";
  sentence[sentence.size() - 2] = hex[(checksum >> 4) & 0xF];
  sentence[sentence.size() - 1] = hex[checksum & 0xF];
  return sentence;
}

void run_fuzz() {
  std::mt19937 rng(0xC0FFEEu);
  for (int i = 0; i < 512; ++i) {
    bool rmc = (i % 2) == 0;
    std::string sentence = makeRandomSentence(rng, rmc);
    std::vector<char> buf(sentence.begin(), sentence.end());
    buf.push_back('\0');
    if (rmc) {
      gps::RmcData data;
      bool ok = gps::parseRmcSentence(buf.data(), data);
      if (ok) {
        assert(data.millis >= 0 && data.millis <= 999);
      }
    } else {
      gps::GgaData data;
      bool ok = gps::parseGgaSentence(buf.data(), data);
      if (ok) {
        assert(!data.has_sats || (data.sats >= 0 && data.sats <= 63));
        if (data.has_hdop) {
          assert(data.hdop >= 0.0 && data.hdop <= 25.4 + 1e-6);
        }
      }
    }
  }
}

}  // namespace

int main() {
  test_valid_rmc();
  test_valid_gga();
  test_rmc_clamp_and_missing_fields();
  test_gga_clamp();
  test_missing_checksum();
  test_extra_commas();
  test_line_too_long();
  run_fuzz();
  std::cout << "All tests passed\n";
  return 0;
}

