#pragma once
#include <cstdint>

namespace diagnostics {
// Retained only while RTC memory survives. No NVS writes and no assertion of
// retention across complete power loss or a brownout that erases RTC memory.
struct BootRecord {
  uint32_t magic;
  uint32_t count;
  uint32_t inverse;
};
constexpr uint32_t kBootMagic = 0x43434142u;
inline bool valid(const BootRecord& r) {
  return r.magic == kBootMagic && r.count != 0 && r.inverse == ~r.count;
}
inline BootRecord nextBoot(const BootRecord& prior, bool powerOn) {
  uint32_t count = powerOn || !valid(prior) ? 1u :
      (prior.count == UINT32_MAX ? UINT32_MAX : prior.count + 1u);
  return {kBootMagic, count, ~count};
}
}
