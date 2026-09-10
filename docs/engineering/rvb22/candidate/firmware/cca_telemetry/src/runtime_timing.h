#pragma once
#include <cstdint>
namespace timing {
// For short intervals serviced at least once per 2^31ms. Expired state is cleared
// so a past timestamp never becomes a future deadline after half-counter wrap.
struct Backoff {
  bool active = false;
  uint32_t deadline = 0;
  void start(uint32_t now, uint32_t interval) {deadline=now+interval;active=true;}
  void clear() {active=false;}
  bool pending(uint32_t now) {
    if (active && static_cast<int32_t>(now-deadline)>=0) active=false;
    return active;
  }
};

// Maintains a fixed deadline phase under service jitter. The first service and
// a period change anchor a new epoch; the first deadline is one period later.
// At most one actual operation per due call. Skipped full deadlines are exposed
// rather than synthesized as duplicate ADC samples/publications. Unsigned elapsed
// arithmetic permits one millis wrap between services; period must be nonzero.
class PeriodicSchedule {
 public:
  bool due(uint32_t now, uint32_t period, uint32_t& skipped) {
    skipped = 0;
    if (period == 0) return false;
    if (!initialized_ || period_ != period) {
      phase_ = now; period_ = period; initialized_ = true; return false;
    }
    const uint32_t periods = (now - phase_) / period_;
    if (periods == 0) return false;
    phase_ += periods * period_;
    skipped = periods - 1u;
    return true;
  }
 private:
  uint32_t phase_ = 0, period_ = 0;
  bool initialized_ = false;
};
// One uint32 micros wrap between observations; startup has no earlier service.
// This monitor exposes missed time contracts; it does not establish driver WCET.
class ServiceGapMonitor {
 public:
  explicit ServiceGapMonitor(uint32_t limitUs) : limitUs_(limitUs) {}
  bool observe(uint32_t now) {
    if (!initialized_) { initialized_=true; previousUs_=now; return false; }
    const uint32_t duration=now-previousUs_; previousUs_=now;
    return recordDuration(duration);
  }
  bool recordDuration(uint32_t duration) {
    if (duration>maximumUs_) maximumUs_=duration;
    if (duration<=limitUs_) return false;
    if (violations_!=UINT64_MAX) ++violations_;
    return true;
  }
  uint32_t maximumUs() const { return maximumUs_; }
  uint64_t violations() const { return violations_; }
 private:
  uint32_t limitUs_, previousUs_=0, maximumUs_=0;
  uint64_t violations_=0;
  bool initialized_=false;
};

}
