#pragma once
#include <cmath>
#include <cstdint>

namespace oil {
constexpr float kNominalGain = 0.5004880381f;
constexpr uint32_t kVersion = 4;
// Honeywell MIPAN2XX150PSAAX: sealed gauge referenced to 14.7 psia,
// 150 psi span, 10..90% excitation. No key-on automatic zero is valid.
constexpr float kSealedReferencePsia = 14.7f;
constexpr float kCalibrationZeroMin = 0.08f, kCalibrationZeroMax = 0.12f;
constexpr float kCalibrationFullMin = 0.88f, kCalibrationFullMax = 0.92f;
// Engineering allocation: 15mA sensor x two 0.2ohm wires =6mV.
// 15mA is not a guaranteed manufacturer hot/startup current maximum.
constexpr float kRemoteExcitationDrop = 0.006f;
struct Channel {
  float raw[2] = {0, 0};
  float volts[2] = {0, 0};
  float maxError = -1; // Connector-domain residual bound; negative=unverified.
  uint32_t points = 0;
};
struct Calibration {
  uint32_t version = kVersion;
  uint32_t reserved = 0;
  uint64_t deviceId = 0; // Factory eFuse base MAC; never infer identity from CRC.
  Channel signal;
  Channel excitation;
  float ratio0 = 0.1f;
  float ratio150 = 0.9f;
  uint32_t pressurePoints = 0;
  uint32_t crc = 0;
};
inline bool identityMatches(const Calibration& c, uint64_t currentDeviceId) {
  return c.version == kVersion && currentDeviceId != 0 && c.deviceId == currentDeviceId;
}
inline bool fitted(const Channel& c) {
  return c.points == 3 && std::isfinite(c.raw[0]) && std::isfinite(c.raw[1]) &&
      std::isfinite(c.volts[0]) && std::isfinite(c.volts[1]) &&
      c.raw[0] >= 0 && c.raw[1] <= 3.1f && c.raw[1] - c.raw[0] >= 0.1f &&
      c.volts[0] >= 0 && c.volts[1] <= 5.5f && c.volts[1] > c.volts[0];
}
inline bool verified(const Channel& c) {
  return fitted(c) && std::isfinite(c.maxError) && c.maxError >= 0.001f && c.maxError <= 0.025f;
}
inline float connectorVolts(const Channel& c, float adc) {
  if (!fitted(c)) return adc / kNominalGain;
  return c.volts[0] + (adc - c.raw[0]) *
         (c.volts[1] - c.volts[0]) / (c.raw[1] - c.raw[0]);
}
inline bool ready(const Calibration& c) {
  return c.version == kVersion && verified(c.signal) && verified(c.excitation) &&
      ((c.pressurePoints == 0 && c.ratio0 == 0.1f && c.ratio150 == 0.9f) || c.pressurePoints == 3) &&
      std::isfinite(c.ratio0) && std::isfinite(c.ratio150) &&
      c.ratio0 >= kCalibrationZeroMin && c.ratio0 <= kCalibrationZeroMax &&
      c.ratio150 >= kCalibrationFullMin && c.ratio150 <= kCalibrationFullMax;
}
// Preserve the original1.125psi electronic-error allocation when a measured
// pressure calibration increases ratio-to-pressure gain. Never relax0.006.
inline float maximumRatioError(const Calibration& c) {
  const float span=c.ratio150-c.ratio0;
  return c.pressurePoints==3 && std::isfinite(span) && span>0 && span<0.8f ?
      0.006f*(span/0.8f) : 0.006f;
}
struct Reading {
  float signal = 0, excitation = 0, ratio = NAN, psi = NAN;
  uint8_t flags = 0;
};
inline Reading evaluate(float signalAdc, float excitationAdc, const Calibration& c) {
  Reading r;
  if (!std::isfinite(signalAdc) || !std::isfinite(excitationAdc) ||
      signalAdc < 0 || signalAdc > 3.1f || excitationAdc < 0 || excitationAdc > 3.1f) {
    r.flags = 1u << 3; return r;
  }
  r.signal = connectorVolts(c.signal, signalAdc);
  r.excitation = connectorVolts(c.excitation, excitationAdc);
  if (!ready(c)) r.flags |= 1u << 5;
  const float excitationError = verified(c.excitation) ? c.excitation.maxError : 0.1f;
  if (r.excitation - excitationError - kRemoteExcitationDrop < 4.75f || r.excitation + excitationError > 5.25f)
    r.flags |= 1u << 4;
  if (r.excitation <= 0.1f) { r.flags |= (1u << 1) | (1u << 2); return r; }
  r.ratio = r.signal / r.excitation;
  if (verified(c.signal) && verified(c.excitation)) {
    const float ratioError = (c.signal.maxError + std::fabs(r.ratio) * c.excitation.maxError) /
                             (r.excitation - c.excitation.maxError);
    if (!std::isfinite(ratioError) || ratioError > maximumRatioError(c)) r.flags |= 1u << 6;
  }
  if (r.ratio > 0.94f) r.flags |= 1u << 0;
  if (r.ratio < 0.06f) r.flags |= 1u << 1;
  if (r.ratio < 0.08f || r.ratio > 0.92f) r.flags |= 1u << 2;
  if (r.flags == 0) {
    r.psi = 150.0f * (r.ratio - c.ratio0) / (c.ratio150 - c.ratio0);
    // Small excursions inside the valid diagnostic window are sensor tolerance.
    if (r.psi < 0) r.psi = 0;
    if (r.psi > 150) r.psi = 150;
  }
  return r;
}
bool load(Calibration& c);
bool save(Calibration& c);
} // namespace oil
