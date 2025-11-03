#pragma once

#include "pidmaps/pidmap_defs.h"

namespace pidmaps {
namespace gr86_2022 {

// ---------- Whitelist ----------
inline bool isCanIdWhitelisted(uint32_t id) {
  // Virtual / diagnostic
  if (id == 0x710) return true;   // Oil pressure (virtual)
  if (id == 0x777) return true;   // Timeout counter / diag

  // OBD-II replies (forward all)
  if (id >= 0x700 && id <= 0x7FF) return true;

  // FT86 Gen2 common IDs
  switch (id) {
    // 100 Hz
    case 0x040: case 0x041:
    // 50 Hz
    case 0x118: case 0x138: case 0x139: case 0x13A: case 0x13B:
    case 0x13C: case 0x143: case 0x146:
    // 33.3 Hz
    case 0x2D2:
    // 20 Hz
    case 0x241:
    // ~10 Hz
    case 0x328: case 0x32B: case 0x345: case 0x390:
    case 0x393: case 0x3A7: case 0x3AC:
    case 0x6E1: case 0x6E2:
    // ~8–9 per 10 s group
    case 0x330: case 0x332: case 0x33A: case 0x33B:
    case 0x640: case 0x651: case 0x652: case 0x654:
    case 0x658: case 0x663:
    case 0x68C: case 0x68D: case 0x6B1: case 0x6BB:
    case 0x6CF: case 0x6DF:
    // ~5 Hz / 50 per 10 s
    case 0x39A: case 0x3D9:
    // 100 Hz (ASC only)
    case 0x6FC:
    // reverse/gear block, etc.
    case 0x228:
      return true;
    default:
      return false;
  }
}

// ---------- Rate policy (1 out of N) ----------
inline uint8_t policyDividerForId(uint32_t id) {
  // Virtual/diagnostic/OBD → full rate
  if (id == 0x710 || id == 0x777) return 1;
  if (id >= 0x700 && id <= 0x7FF) return 1;

  // Critical: 0x139 brake/speed cluster → full rate
  if (id == 0x139) return 1;

  switch (id) {
    // 100 Hz => ~25 Hz
    case 0x040: case 0x041: case 0x6FC:
      return 4;

    // 50 Hz => ~25 Hz
    case 0x118: case 0x138: case 0x13A: case 0x13B:
    case 0x13C: case 0x143: case 0x146:
    case 0x39A: case 0x3D9:
      return 2;

    // 33.3 Hz => ~16–17 Hz
    case 0x2D2:
      return 2;

    // 20 Hz
    case 0x241:
      return 1;

    // 10 Hz group
    case 0x328: case 0x32B: case 0x345: case 0x390:
    case 0x393: case 0x3A7: case 0x3AC:
    case 0x6E1: case 0x6E2:
      return 1;

    // ~8–9 per 10 s group
    case 0x330: case 0x332: case 0x33A: case 0x33B:
    case 0x640: case 0x651: case 0x652: case 0x654:
    case 0x658: case 0x663:
    case 0x68C: case 0x68D: case 0x6B1: case 0x6BB:
    case 0x6CF: case 0x6DF:
      return 1;

    // other listed
    case 0x228:
      return 1;

    default:
      // Fallback heuristic:
      if (id < 0x100) return 4;   // likely high-rate
      if (id < 0x200) return 2;   // mid-rate
      return 10;                  // slow/unknown
  }
}

inline constexpr PidRule RULES[] = {
  // 100 Hz drivetrain block
  { 0x040, 4 },   // engine RPM / pedals -> 25 Hz
  { 0x041, 4 },   // clutch / AC -> 25 Hz

  // 50 Hz chassis dynamics block
  { 0x118, 2 },   // mixed engine data -> 25 Hz
  { 0x138, 2 },   // steering angle / yaw -> 25 Hz
  { 0x139, 1 },   // vehicle speed / brake pressure (keep full rate)
  { 0x13A, 2 },   // wheel speeds (ASC bus)
  { 0x13B, 1 },   // accelerometers
  { 0x13C, 2 },   // misc chassis signals
  { 0x143, 2 },   // secondary speed source
  { 0x146, 2 },   // additional dynamics block

  // 33-20 Hz drivetrain / controls
  { 0x228, 1 },   // reverse gear
  { 0x241, 1 },   // clutch / gear selection
  { 0x2D2, 2 },   // ABS / stability payload (~33 Hz)

  // 10 Hz body / environmental sensors
  { 0x328, 1 }, { 0x32B, 1 }, { 0x330, 1 }, { 0x332, 1 },
  { 0x33A, 1 }, { 0x33B, 1 },
  { 0x345, 1 },   // oil / coolant temps
  { 0x390, 1 },   // intake air temp
  { 0x393, 1 },   // fuel level
  { 0x39A, 1 },
  { 0x3A7, 1 }, { 0x3AC, 1 }, { 0x3D9, 1 },

  // Body / HVAC / telematics (<=10 Hz unless noted)
  { 0x500, 1 }, { 0x506, 1 }, { 0x509, 1 }, { 0x50B, 1 },
  { 0x513, 1 }, { 0x515, 1 }, { 0x517, 1 },
  { 0x640, 1 }, { 0x651, 1 }, { 0x652, 1 }, { 0x654, 1 },
  { 0x658, 1 }, { 0x660, 1 }, { 0x663, 1 },
  { 0x68C, 1 }, { 0x68D, 1 }, { 0x6B1, 1 }, { 0x6BB, 1 },
  { 0x6CF, 1 }, { 0x6DF, 1 },
  { 0x6E1, 1 }, { 0x6E2, 1 },   // TPMS
  { 0x6FB, 1 }, { 0x6FC, 1 },

  // Custom sensors
  { 0x710, 1 },   // oil pressure synthetic CAN frame
};

inline constexpr PidMapDefinition MAP = {
  "Toyota GR86 (2022)",
  RULES,
  sizeof(RULES) / sizeof(RULES[0]),
  isCanIdWhitelisted,
  policyDividerForId,
};

}  // namespace gr86_2022

inline constexpr const PidMapDefinition &GR86_2022 = gr86_2022::MAP;

}  // namespace pidmaps
