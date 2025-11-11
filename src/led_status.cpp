#include "led_status.h"

#include <Arduino.h>

namespace {

enum class LedIndex : uint8_t {
  Power = 0,
  Ble,
  Can,
  Gps,
  Sys,
  Oil,
  Count,
};

struct LedState {
  int pin;
  bool activeLow;
  LedPattern pattern;
  uint32_t cycleStartMs;
  bool outputState;
  bool outputValid;
};

constexpr size_t kLedCount = static_cast<size_t>(LedIndex::Count);

LedState g_leds[kLedCount] = {
    {LED_PWR_GPIO, LED_PWR_ACTIVE_LOW != 0, LedPattern::Off, 0, false, false},
    {LED_BLE_GPIO, LED_BLE_ACTIVE_LOW != 0, LedPattern::Off, 0, false, false},
    {LED_CAN_GPIO, LED_CAN_ACTIVE_LOW != 0, LedPattern::Off, 0, false, false},
    {LED_GPS_GPIO, LED_GPS_ACTIVE_LOW != 0, LedPattern::Off, 0, false, false},
    {LED_SYS_GPIO, LED_SYS_ACTIVE_LOW != 0, LedPattern::Off, 0, false, false},
    {LED_OIL_GPIO, LED_OIL_ACTIVE_LOW != 0, LedPattern::Off, 0, false, false},
};

inline LedState &stateFor(LedIndex index) {
  return g_leds[static_cast<size_t>(index)];
}

void applyOutput(LedState &state, bool on) {
  if (state.pin < 0) {
    state.outputState = on;
    state.outputValid = true;
    return;
  }

  bool level = state.activeLow ? !on : on;
  digitalWrite(state.pin, level ? HIGH : LOW);
  state.outputState = on;
  state.outputValid = true;
}

bool patternOn(LedPattern pattern, uint32_t elapsed) {
  switch (pattern) {
    case LedPattern::Off:
      return false;
    case LedPattern::Solid:
      return true;
    case LedPattern::BlinkFast: {
      uint32_t phase = elapsed % 200u;
      return phase < 100u;
    }
    case LedPattern::BlinkSlow: {
      uint32_t phase = elapsed % 1000u;
      return phase < 200u;
    }
    case LedPattern::Pulse2Every2s: {
      uint32_t phase = elapsed % 2000u;
      return (phase < 80u) || (phase >= 200u && phase < 280u);
    }
    case LedPattern::Pulse3Every2s: {
      uint32_t phase = elapsed % 2000u;
      return (phase < 80u) ||
             (phase >= 200u && phase < 280u) ||
             (phase >= 400u && phase < 480u);
    }
  }
  return false;
}

void updateLed(LedState &state, uint32_t now) {
  if (!state.outputValid) {
    state.cycleStartMs = now;
  }

  bool shouldOn;
  switch (state.pattern) {
    case LedPattern::Off:
      shouldOn = false;
      break;
    case LedPattern::Solid:
      shouldOn = true;
      break;
    default: {
      uint32_t elapsed = now - state.cycleStartMs;
      shouldOn = patternOn(state.pattern, elapsed);
      break;
    }
  }

  if (!state.outputValid || shouldOn != state.outputState) {
    applyOutput(state, shouldOn);
  }
}

void setPattern(LedIndex index, LedPattern pattern) {
  LedState &state = stateFor(index);
  uint32_t now = millis();
  if (state.pattern != pattern) {
    state.pattern = pattern;
    state.cycleStartMs = now;
    state.outputValid = false;
  }
  updateLed(state, now);
}

}  // namespace

void led_init() {
  uint32_t now = millis();
  for (size_t i = 0; i < kLedCount; ++i) {
    LedState &state = g_leds[i];
    state.pattern = LedPattern::Off;
    state.cycleStartMs = now;
    state.outputValid = false;
    state.outputState = false;
    if (state.pin >= 0) {
      pinMode(state.pin, OUTPUT);
      digitalWrite(state.pin, state.activeLow ? HIGH : LOW);
    }
  }
}

void led_set_power(LedPattern pattern) { setPattern(LedIndex::Power, pattern); }
void led_set_ble(LedPattern pattern)   { setPattern(LedIndex::Ble, pattern); }
void led_set_can(LedPattern pattern)   { setPattern(LedIndex::Can, pattern); }
void led_set_gps(LedPattern pattern)   { setPattern(LedIndex::Gps, pattern); }
void led_set_sys(LedPattern pattern)   { setPattern(LedIndex::Sys, pattern); }
void led_set_oil(LedPattern pattern)   { setPattern(LedIndex::Oil, pattern); }

void led_service(uint32_t now_ms) {
  for (size_t i = 0; i < kLedCount; ++i) {
    LedState &state = g_leds[i];
    updateLed(state, now_ms);
  }
}
