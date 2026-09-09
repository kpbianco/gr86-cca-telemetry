#pragma once

#include <cmath>
#include <cstdint>

namespace racechrono {
// RaceChrono's high bit selects the coarse scale; only 15 bits hold magnitude.
inline uint16_t altitudeWord(double meters) {
  if (!std::isfinite(meters) || meters < -500.0 || meters > 32266.0)
    return 0xFFFF;
  const long fine = std::lround((meters + 500.0) * 10.0);
  if (fine <= 0x7FFF) return static_cast<uint16_t>(fine);
  const long coarse = std::lround(meters + 500.0);
  if (coarse >= 0x7FFF) return 0xFFFF; // Reserved invalid encoding.
  return static_cast<uint16_t>(coarse) | 0x8000;
}

inline uint16_t speedWord(double kmh) {
  if (!std::isfinite(kmh) || kmh < 0.0 || kmh > 3276.6) return 0xFFFF;
  const long fine = std::lround(kmh * 100.0);
  if (fine <= 0x7FFF) return static_cast<uint16_t>(fine);
  const long coarse = std::lround(kmh * 10.0);
  if (coarse >= 0x7FFF) return 0xFFFF;
  return static_cast<uint16_t>(coarse) | 0x8000;
}

inline uint16_t bearingWord(double degrees) {
  if (!std::isfinite(degrees) || degrees < 0.0 || degrees >= 360.0)
    return 0xFFFF;
  return static_cast<uint16_t>(std::lround(degrees * 100.0) % 36000);
}
}  // namespace racechrono
