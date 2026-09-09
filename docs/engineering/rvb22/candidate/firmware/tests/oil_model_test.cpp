#include "../cca_telemetry/src/oil_model.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
int main() {
  unsigned checks = 0;
  auto check = [&](bool x) { assert(x); ++checks; };
  oil::Calibration c;
  check(!oil::ready(c));
  auto uncal = oil::evaluate(0.250244f, 2.50244f, c);
  check((uncal.flags & 32) && std::isnan(uncal.psi));
  // Different gain/offset in each channel: fit measurements in connector volts.
  c.signal.raw[0] = 0.27f; c.signal.raw[1] = 2.23f;
  c.signal.volts[0] = 0.5f; c.signal.volts[1] = 4.5f;
  c.signal.points = 3; c.signal.maxError = 0.01f;
  c.excitation.raw[0] = 2.36f; c.excitation.raw[1] = 2.61f;
  c.excitation.volts[0] = 4.75f; c.excitation.volts[1] = 5.25f;
  c.excitation.points = 3; c.excitation.maxError = 0.01f;
  check(oil::ready(c)); // Nominal Honeywell ratios after verified ADC calibration.
  c.ratio0 = 0.11f; check(!oil::ready(c));
  c.ratio0 = 0.1f;
  c.pressurePoints = 1; check(!oil::ready(c));
  c.pressurePoints = 3;
  check(oil::ready(c));
  auto adc = [](const oil::Channel& c, float v) {
    return c.raw[0] + (v - c.volts[0]) * (c.raw[1] - c.raw[0]) / (c.volts[1] - c.volts[0]);
  };
  auto read = [&](float sig, float exc) {
    return oil::evaluate(adc(c.signal, sig), adc(c.excitation, exc), c);
  };
  for (float supply : {4.8f, 5.0f, 5.2f}) {
    for (float pressure : {0.0f, 37.5f, 75.0f, 112.5f, 150.0f}) {
      auto r = read(supply * (0.1f + 0.8f * pressure / 150), supply);
      check(r.flags == 0 && std::fabs(r.psi - pressure) < 0.001f);
    }
  }
  auto low = read(0.0f, 5.0f);
  check((low.flags & 2) && std::isnan(low.psi));
  auto sensorDiagnostic = read(0.025f * 5, 5);
  check((sensorDiagnostic.flags & 2) && std::isnan(sensorDiagnostic.psi));
  auto internalFailure = read(0.975f * 5, 5);
  check((internalFailure.flags & 1) && std::isnan(internalFailure.psi));
  auto droop = read(0.5f * 4.5f, 4.5f);
  check((droop.flags & 16) && std::isnan(droop.psi));
  auto boundary = read(0.5f * 4.75f, 4.75f);
  check((boundary.flags & 16) && std::isnan(boundary.psi));
  auto ceiling = read(0.9f * 5.25f, 5.25f);
  check((ceiling.flags & 16) && std::isnan(ceiling.psi));
  check(oil::evaluate(NAN, 2.5f, c).flags & 8);
  check(oil::evaluate(3.2f, 2.5f, c).flags & 8);
  c.signal.maxError = c.excitation.maxError = 0.025f;
  check(read(4.5f, 5.0f).flags & 64);
  c.signal.maxError = 0.2f;
  check(!oil::ready(c) && (read(2.5f, 5.0f).flags & 32));
  c.signal.raw[1] = c.signal.raw[0];
  check(!oil::fitted(c.signal));
  std::printf("PASS: %u independent-channel calibration, supply sweep and fault assertions.\n", checks);
}
