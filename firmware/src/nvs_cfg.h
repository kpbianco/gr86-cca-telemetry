#pragma once

#include <stdint.h>

struct OilCalibrationRaw {
  uint32_t version;
  float    v0_adc;
  float    v1_adc;
  uint32_t stored_crc32;
  uint32_t computed_crc32;
  bool     present;
  bool     valid;
};

bool oil_calibration_load(float *out_v0, float *out_v1);
bool oil_calibration_save(float v0, float v1);
bool oil_calibration_read_raw(OilCalibrationRaw *out);

extern bool nvsWriteInProgress;

class ScopedNvsWrite {
 public:
  ScopedNvsWrite();
  ~ScopedNvsWrite();
};

