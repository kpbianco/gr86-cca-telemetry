#include "gps_nmea.h"

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>

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

inline double clampHdop(double value) {
  if (value < 0.0) return 0.0;
  if (value > 25.4) return 25.4;
  return value;
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
  if (!star || star[1] == '\0' || star[2] == '\0') return false;
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
  double minutes = std::atof(ddmm + degLen);
  double val = static_cast<double>(degrees) + (minutes / 60.0);
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

bool parseRmcSentence(char *line, RmcData &out) {
  if (!line) return false;
  std::size_t len = ::strnlen(line, kMaxSentenceLength + 1);
  if (len == 0 || len > kMaxSentenceLength) return false;
  if (!detail::nmeaChecksumOk(line, len)) return false;

  out = RmcData{};
  const char *fields[kMaxFields];
  int nf = detail::splitCsv(line, len, fields, kMaxFields);
  if (nf < 10) return false;

  const char *time = fields[1];
  if (time && std::strlen(time) >= 6) {
    out.has_time = true;
    out.hour = (time[0] - '0') * 10 + (time[1] - '0');
    out.minute = (time[2] - '0') * 10 + (time[3] - '0');
    out.second = (time[4] - '0') * 10 + (time[5] - '0');
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

  out.valid = (fields[2] && fields[2][0] == 'A');

  double coord = 0.0;
  if (detail::nmeaCoordToDegrees(fields[3], fields[4], coord)) {
    out.has_latitude = true;
    out.latitude_deg = coord;
  }
  if (detail::nmeaCoordToDegrees(fields[5], fields[6], coord)) {
    out.has_longitude = true;
    out.longitude_deg = coord;
  }

  out.speed_kmh = (fields[7] && fields[7][0]) ? std::atof(fields[7]) * 1.852 : 0.0;
  out.course_deg = (fields[8] && fields[8][0]) ? std::atof(fields[8]) : 0.0;

  const char *date = fields[9];
  if (date && std::strlen(date) >= 6) {
    out.has_date = true;
    out.day = (date[0] - '0') * 10 + (date[1] - '0');
    out.month = (date[2] - '0') * 10 + (date[3] - '0');
    out.year = 2000 + (date[4] - '0') * 10 + (date[5] - '0');
  }

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

  if (fields[7] && fields[7][0]) {
    out.has_sats = true;
    out.sats = detail::clampSats(std::atoi(fields[7]));
  } else {
    out.has_sats = false;
    out.sats = 0;
  }

  if (fields[8] && fields[8][0]) {
    out.has_hdop = true;
    out.hdop = detail::clampHdop(std::atof(fields[8]));
  } else {
    out.has_hdop = false;
    out.hdop = 0.0;
  }

  if (fields[9] && fields[9][0]) {
    out.has_altitude = true;
    out.altitude_m = std::atof(fields[9]);
  } else {
    out.has_altitude = false;
    out.altitude_m = 0.0;
  }

  return true;
}

}  // namespace gps
