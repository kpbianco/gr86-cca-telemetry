#pragma once

#include <cstddef>

namespace gps {

constexpr std::size_t kMaxSentenceLength = 120;
constexpr int kMaxFields = 20;

struct RmcData {
  bool has_time = false;
  int hour = 0;
  int minute = 0;
  int second = 0;
  int millis = 0;
  bool valid = false;
  bool has_latitude = false;
  double latitude_deg = 0.0;
  bool has_longitude = false;
  double longitude_deg = 0.0;
  double speed_kmh = 0.0;
  double course_deg = 0.0;
  bool has_date = false;
  int day = 0;
  int month = 0;
  int year = 0;
};

struct GgaData {
  bool has_sats = false;
  int sats = 0;
  bool has_hdop = false;
  double hdop = 0.0;
  bool has_altitude = false;
  double altitude_m = 0.0;
};

bool parseRmcSentence(char *line, RmcData &out);
bool parseGgaSentence(char *line, GgaData &out);

}  // namespace gps

