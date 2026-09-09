#pragma once
#include <cstddef>
#include <cstdint>
#include <algorithm>

namespace cca { namespace io {
// Units are 1/1000 token. Every elapsed millisecond is credited exactly once.
// Unsigned elapsed subtraction supports one millis rollover between services.
class TokenBucket {
 public:
  TokenBucket(uint16_t capacity, uint16_t rate)
      : cap_(uint32_t(capacity) * 1000u), rate_(rate), credit_(cap_) {}
  void reset(uint32_t now) { credit_ = cap_; last_ = now; initialized_ = true; }
  void refill(uint32_t now) {
    if (!initialized_) { reset(now); return; }
    const uint32_t elapsed = now - last_;
    last_ = now;
    const uint64_t next = uint64_t(credit_) + uint64_t(elapsed) * rate_;
    credit_ = uint32_t(std::min<uint64_t>(cap_, next));
  }
  bool take(uint32_t now, uint16_t reserve = 0) {
    refill(now);
    if (credit_ < (uint32_t(reserve) + 1u) * 1000u) return false;
    credit_ -= 1000u;
    return true;
  }
  uint32_t credit() const { return credit_; }
 private:
  uint32_t cap_, rate_, credit_, last_ = 0;
  bool initialized_ = false;
};

// Single main-loop producer/consumer; never called from interrupts/callbacks.
// Full queues drop diagnostic bytes, not acquisition work. No heap allocation.
template <size_t N> class ByteQueue {
 public:
  size_t append(const uint8_t* data, size_t count) {
    const size_t accepted = std::min(count, N - size_);
    for (size_t i = 0; i < accepted; ++i) data_[(head_ + size_ + i) % N] = data[i];
    size_ += accepted;
    high_ = std::max(high_, size_);
    dropped_ += count - accepted;
    return accepted;
  }
  size_t size() const { return size_; }
  size_t highWater() const { return high_; }
  uint64_t dropped() const { return dropped_; }
  uint8_t front() const { return data_[head_]; }
  const uint8_t* contiguousData() const { return data_ + head_; }
  size_t contiguousSize() const { return std::min(size_, N - head_); }
  void consume(size_t n) { n = std::min(n, size_); head_ = (head_ + n) % N; size_ -= n; }
 private:
  uint8_t data_[N] = {};
  size_t head_ = 0, size_ = 0, high_ = 0;
  uint64_t dropped_ = 0;
};
}}
