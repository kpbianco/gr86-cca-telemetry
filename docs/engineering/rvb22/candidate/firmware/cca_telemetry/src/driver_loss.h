#pragma once
#include <cstdint>
#include <limits>
namespace diagnostics {
inline void saturatingAdd(uint64_t& total, uint64_t amount) {
  const auto maximum = std::numeric_limits<uint64_t>::max();
  total = amount > maximum - total ? maximum : total + amount;
}
// Main-loop-owned boot-lifetime counter. Call beginDriver only after installing
// a fresh TWAI driver. At most one uint32 wrap may occur between observations;
// a decrease within one driver is a modulo counter wrap, not an implicit reset.
class DriverLossCounter {
 public:
  void beginDriver() { previous_ = 0; }
  void observe(uint32_t current) {
    saturatingAdd(total_, static_cast<uint32_t>(current - previous_));
    previous_ = current;
  }
  uint64_t total() const { return total_; }
 private:
  uint64_t total_ = 0;
  uint32_t previous_ = 0;
};
}
