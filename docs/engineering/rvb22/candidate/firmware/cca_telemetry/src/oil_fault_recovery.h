#pragma once
#include "oil_model.h"

namespace oil {
// Temporal hysteresis: every raw fault enters immediately. Recovery requires
// three consecutive fully valid acquisitions; no voltage threshold is relaxed.
// During recovery, flags retain the most recent fault episode and psi is NaN.
// This state does not classify the physical cause of a low/high electrical input.
class FaultRecovery {
 public:
  static constexpr uint8_t kCleanSamplesToRecover = 3;

  Reading apply(Reading raw) {
    if (raw.flags) {
      heldFlags_ = raw.flags;
      cleanSamples_ = 0;
    } else if (heldFlags_) {
      if (++cleanSamples_ >= kCleanSamplesToRecover) {
        heldFlags_ = 0;
        cleanSamples_ = 0;
      }
    }
    if (heldFlags_) {
      raw.flags = heldFlags_;
      raw.psi = NAN;
    }
    return raw;
  }

 private:
  uint8_t heldFlags_ = 0;
  uint8_t cleanSamples_ = 0;
};
}  // namespace oil
