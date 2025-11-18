#pragma once

#include <cstdint>

// Returns true when the given period has elapsed since the last invocation.
// Updates \p tLast to preserve deterministic cadence even if multiple
// periods have passed.
inline bool every(uint32_t now, uint32_t* tLast, uint32_t periodMs) {
  if (!tLast || periodMs == 0) {
    return false;
  }

  const uint32_t last = *tLast;
  const uint32_t elapsed = now - last;
  if (elapsed < periodMs) {
    return false;
  }

  const uint32_t remainder = elapsed % periodMs;
  const uint32_t scheduled = now - remainder;
  *tLast = scheduled;
  return true;
}

