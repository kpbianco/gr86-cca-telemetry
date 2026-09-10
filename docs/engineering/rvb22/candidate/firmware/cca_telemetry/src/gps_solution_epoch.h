#pragma once
#include "gps_nmea.h"

namespace gps {
enum class EpochResult : uint8_t { Missing, First, Advanced, Duplicate, Backward };

// Separate solution time from serial reception time. The RMC parser validates
// dates in 2000..2099. Keep the accepted epoch across automatic baud probes so a
// stuck receiver cannot repeatedly renew an old solution by being reprobed.
class SolutionEpoch {
 public:
  EpochResult observe(const RmcData& rmc) {
    if (!rmc.has_time || !rmc.has_date) return EpochResult::Missing;
    const uint32_t years = static_cast<uint32_t>(rmc.year - 2000);
    const uint16_t precedingMonthDays[] = {0,31,59,90,120,151,181,212,243,273,304,334};
    const uint32_t days = 365u * years + (years + 3u) / 4u +
        precedingMonthDays[rmc.month - 1] + (rmc.day - 1) +
        ((rmc.year % 4 == 0 && rmc.month > 2) ? 1u : 0u);
    const uint64_t epoch = uint64_t(days) * 86400000u +
        uint32_t(rmc.hour) * 3600000u + uint32_t(rmc.minute) * 60000u +
        uint32_t(rmc.second) * 1000u + uint32_t(rmc.millis);
    if (haveEpoch_ && epoch == lastEpoch_) return EpochResult::Duplicate;
    if (haveEpoch_ && epoch < lastEpoch_) return EpochResult::Backward;
    const EpochResult result = haveEpoch_ ? EpochResult::Advanced : EpochResult::First;
    haveEpoch_ = true;
    lastEpoch_ = epoch;
    return result;
  }

 private:
  bool haveEpoch_ = false;
  uint64_t lastEpoch_ = 0;
};
}  // namespace gps
