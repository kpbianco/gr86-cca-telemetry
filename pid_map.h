#pragma once
#include <stdint.h>

// ---------- Whitelist ----------
static inline bool isCanIdWhitelisted(uint32_t id) {
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
static inline uint8_t policy_divider_for_id(uint32_t id) {
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

// Convenience: 0 if blocked, else the divider.
static inline uint8_t pid_policy_divider_or_zero(uint32_t id) {
  if (!isCanIdWhitelisted(id)) return 0;
  return policy_divider_for_id(id);
}
