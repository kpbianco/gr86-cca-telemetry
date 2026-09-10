#include "gps_nmea.h"

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdio>

namespace gps {
namespace detail {

namespace {
inline int hexValue(int c) {
  if (c >= 'A') return c - 'A' + 10;
  return c - '0';
}

inline int clampMillis(int value) {
  if (value < 0) return 0;
  if (value > 999) return 999;
  return value;
}

inline int clampSats(int value) {
  if (value < 0) return 0;
  if (value > 63) return 63;
  return value;
}

bool decimal(const char* text, double& value, bool signedValue = false) {
  if (!text || !*text) return false;
  const char* p = text;
  if (signedValue && (*p == '-' || *p == '+')) ++p;
  if (!std::isdigit(static_cast<unsigned char>(*p))) return false;
  while (std::isdigit(static_cast<unsigned char>(*p))) ++p;
  if (*p == '.') {
    ++p;
    if (!std::isdigit(static_cast<unsigned char>(*p))) return false;
    while (std::isdigit(static_cast<unsigned char>(*p))) ++p;
  }
  if (*p) return false; // Reject partial numbers, exponent notation and NaN/Inf.
  char* end = nullptr;
  value = std::strtod(text, &end);
  return end && !*end && std::isfinite(value);
}

bool unsignedInteger(const char* text, unsigned limit, int& value) {
  if (!text || !*text) return false;
  unsigned parsed = 0;
  for (const char* p = text; *p; ++p) {
    if (!std::isdigit(static_cast<unsigned char>(*p))) return false;
    const unsigned digit = static_cast<unsigned>(*p - '0');
    if (digit > limit || parsed > (limit - digit) / 10u) return false;
    parsed = parsed * 10u + digit;
  }
  value = static_cast<int>(parsed);
  return true;
}
}  // namespace

bool nmeaChecksumOk(const char *line, std::size_t len) {
  if (!line || len == 0 || line[0] != '$') return false;
  const char *star = nullptr;
  for (std::size_t i = 1; i < len; ++i) {
    if (line[i] == '*') {
      star = line + i;
      break;
    }
    if (line[i] == '\0') {
      break;
    }
  }
  if (!star || static_cast<std::size_t>(star - line) + 3 != len)
    return false;
  int hi = std::toupper(static_cast<unsigned char>(star[1]));
  int lo = std::toupper(static_cast<unsigned char>(star[2]));
  if (!std::isxdigit(hi) || !std::isxdigit(lo)) return false;
  int expected = (hexValue(hi) << 4) | hexValue(lo);
  unsigned checksum = 0;
  for (const char *p = line + 1; p < star && *p; ++p) {
    checksum ^= static_cast<unsigned char>(*p);
  }
  return static_cast<unsigned>(expected & 0xFF) == (checksum & 0xFF);
}

bool nmeaCoordToDegrees(const char *ddmm, const char *hemi, double &out) {
  if (!ddmm || !hemi || *ddmm == '\0' || *hemi == '\0') return false;
  if (hemi[1] != '\0' || !std::strchr("NSEW", *hemi)) return false;
  for (const char *p = ddmm; *p; ++p)
    if (!std::isdigit(static_cast<unsigned char>(*p)) && *p != '.')
      return false;
  const char *dot = std::strchr(ddmm, '.');
  int len = dot ? static_cast<int>(dot - ddmm)
                : static_cast<int>(std::strlen(ddmm));
  if (len < 3) return false;
  int degLen = len - 2;
  if (degLen <= 0 || degLen >= 10) return false;
  char degBuf[12];
  std::memset(degBuf, 0, sizeof(degBuf));
  std::strncpy(degBuf, ddmm, static_cast<std::size_t>(degLen));
  int degrees = std::atoi(degBuf);
  char *end = nullptr;
  double minutes = std::strtod(ddmm + degLen, &end);
  if (!end || *end || !std::isfinite(minutes) || minutes >= 60.0)
    return false;
  double val = static_cast<double>(degrees) + (minutes / 60.0);
  const bool latitude = *hemi == 'N' || *hemi == 'S';
  if (degLen != (latitude ? 2 : 3) || val > (latitude ? 90 : 180))
    return false;
  if (*hemi == 'S' || *hemi == 'W') val = -val;
  out = val;
  return true;
}

int splitCsv(char *s, std::size_t len, const char *fields[], int maxFields) {
  if (!s || !fields || maxFields <= 0) return 0;
  int count = 0;
  fields[count++] = s;
  for (std::size_t i = 0; i < len && s[i] != '\0'; ++i) {
    if (s[i] == ',' || s[i] == '*') {
      char current = s[i];
      s[i] = '\0';
      if (current == ',') {
        if (count < maxFields) {
          fields[count++] = &s[i + 1];
        }
      } else {
        break;
      }
    }
  }
  return count;
}

}  // namespace detail

bool checksumOk(const char *line, std::size_t len) {
  return detail::nmeaChecksumOk(line, len);
}

bool parseAntennaStatus(const char *line, uint8_t &status) {
  if (!line) return false;
  const size_t len = ::strnlen(line, kMaxSentenceLength + 1);
  if (!checksumOk(line, len) || len != 11 ||
      std::strncmp(line, "$PCD,9,", 7) || line[7] < '1' || line[7] > '3')
    return false;
  status = static_cast<uint8_t>(line[7] - '0');
  return true;
}

bool parsePmtkAck(const char *line, uint16_t &command, uint8_t &result) {
  if (!line) return false;
  const size_t len = ::strnlen(line, kMaxSentenceLength + 1);
  if (!checksumOk(line, len)) return false;
  unsigned cmd = 0, flag = 0;
  int consumed = 0;
  if (std::sscanf(line, "$PMTK001,%u,%u*%n", &cmd, &flag, &consumed) != 2 ||
      consumed != static_cast<int>(len) - 2 || cmd > 999 || flag > 3)
    return false;
  command = static_cast<uint16_t>(cmd);
  result = static_cast<uint8_t>(flag);
  return true;
}

bool parseRmcSentence(char *line, RmcData &out) {
  if (!line) return false;
  std::size_t len = ::strnlen(line, kMaxSentenceLength + 1);
  if (len == 0 || len > kMaxSentenceLength) return false;
  if (!detail::nmeaChecksumOk(line, len)) return false;

  out = RmcData{};
  const char *fields[kMaxFields];
  int nf = detail::splitCsv(line, len, fields, kMaxFields);
  if (nf < 10) return false;
  if (std::strcmp(fields[0], "$GPRMC") && std::strcmp(fields[0], "$GNRMC"))
    return false;

  const char *time = fields[1];
  if (time && std::strlen(time) >= 6) {
    for (int i = 0; i < 6; ++i)
      if (!std::isdigit(static_cast<unsigned char>(time[i]))) return false;
    out.has_time = true;
    out.hour = (time[0] - '0') * 10 + (time[1] - '0');
    out.minute = (time[2] - '0') * 10 + (time[3] - '0');
    out.second = (time[4] - '0') * 10 + (time[5] - '0');
    if (out.hour > 23 || out.minute > 59 || out.second > 59) return false;
    if (time[6] && time[6] != '.') return false;
    if (time[6] == '.') {
      if (!time[7]) return false;
      for (const char* p = time + 7; *p; ++p)
        if (!std::isdigit(static_cast<unsigned char>(*p))) return false;
    }
    int ms = 0;
    const char *dot = std::strchr(time, '.');
    if (dot) {
      int scale = 100;
      for (const char *p = dot + 1;
           *p && std::isdigit(static_cast<unsigned char>(*p)) && scale > 0; ++p) {
        ms += (*p - '0') * scale;
        scale /= 10;
      }
    }
    out.millis = detail::clampMillis(ms);
  }

  out.valid = std::strcmp(fields[2], "A") == 0;

  double coord = 0.0;
  if (detail::nmeaCoordToDegrees(fields[3], fields[4], coord)) {
    out.has_latitude = true;
    out.latitude_deg = coord;
  }
  if (detail::nmeaCoordToDegrees(fields[5], fields[6], coord)) {
    out.has_longitude = true;
    out.longitude_deg = coord;
  }

  double speed = 0, course = 0;
  out.speed_kmh = detail::decimal(fields[7], speed) &&
      std::isfinite(speed * 1.852) ? speed * 1.852 : NAN;
  out.course_deg = detail::decimal(fields[8], course) && course < 360.0 ? course : NAN;

  const char *date = fields[9];
  if (date && std::strlen(date) >= 6) {
    if (std::strlen(date) != 6) return false;
    for (int i = 0; i < 6; ++i)
      if (!std::isdigit(static_cast<unsigned char>(date[i]))) return false;
    out.has_date = true;
    out.day = (date[0] - '0') * 10 + (date[1] - '0');
    out.month = (date[2] - '0') * 10 + (date[3] - '0');
    out.year = 2000 + (date[4] - '0') * 10 + (date[5] - '0');
    const int monthDays[] = {31, 28 + (out.year % 4 == 0), 31, 30, 31, 30,
                             31, 31, 30, 31, 30, 31};
    if (out.month < 1 || out.month > 12 || out.day < 1 ||
        out.day > monthDays[out.month - 1]) return false;
  }

  out.valid = out.valid && out.has_time && out.has_date &&
              out.has_latitude && out.has_longitude &&
              (fields[4][0] == 'N' || fields[4][0] == 'S') &&
              (fields[6][0] == 'E' || fields[6][0] == 'W');
  // Optional speed/course have their own invalid encodings. Missing or malformed
  // optional fields must not become a plausible zero or erase a valid position.

  return true;
}

bool parseGgaSentence(char *line, GgaData &out) {
  if (!line) return false;
  std::size_t len = ::strnlen(line, kMaxSentenceLength + 1);
  if (len == 0 || len > kMaxSentenceLength) return false;
  if (!detail::nmeaChecksumOk(line, len)) return false;

  out = GgaData{};
  const char *fields[kMaxFields];
  int nf = detail::splitCsv(line, len, fields, kMaxFields);
  if (nf < 11) return false;
  if (std::strcmp(fields[0], "$GPGGA") && std::strcmp(fields[0], "$GNGGA"))
    return false;
  if (!detail::unsignedInteger(fields[6], 8, out.fix_quality)) return false;

  if (fields[7] && fields[7][0]) {
    out.has_sats = detail::unsignedInteger(fields[7], 99, out.sats);
    out.sats = out.has_sats ? detail::clampSats(out.sats) : 0;
  } else {
    out.has_sats = false;
    out.sats = 0;
  }

  if (fields[8] && fields[8][0]) {
    out.has_hdop = detail::decimal(fields[8], out.hdop) && out.hdop <= 25.4;
    if (!out.has_hdop) out.hdop = NAN;
  } else {
    out.has_hdop = false;
    out.hdop = 0.0;
  }

  if (fields[9] && fields[9][0]) {
    out.has_altitude = detail::decimal(fields[9], out.altitude_m, true) && out.fix_quality > 0 &&
                       std::strcmp(fields[10], "M") == 0;
  } else {
    out.has_altitude = false;
    out.altitude_m = 0.0;
  }

  return true;
}

}  // namespace gps
