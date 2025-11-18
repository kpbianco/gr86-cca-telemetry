#include "nvs_cfg.h"

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>

namespace {

constexpr const char *kCfgNamespace = "cca_cfg";
constexpr const char *kOilKey       = "oil_cal";
constexpr uint32_t    kVersion      = 1;

struct __attribute__((packed)) OilCalibrationBlob {
  uint32_t version;
  float    v0_adc;
  float    v1_adc;
  uint32_t crc32;
};

uint32_t crc32_le(const void *data, size_t len) {
  const uint8_t *bytes = static_cast<const uint8_t*>(data);
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint32_t>(bytes[i]);
    for (int bit = 0; bit < 8; ++bit) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return crc ^ 0xFFFFFFFFu;
}

uint32_t blob_crc(const OilCalibrationBlob &blob) {
  return crc32_le(&blob, sizeof(blob) - sizeof(blob.crc32));
}

bool read_blob(OilCalibrationBlob *out) {
  if (!out) return false;
  Preferences prefs;
  if (!prefs.begin(kCfgNamespace, true)) {
    return false;
  }
  OilCalibrationBlob tmp{};
  size_t len = prefs.getBytes(kOilKey, &tmp, sizeof(tmp));
  prefs.end();
  if (len != sizeof(tmp)) {
    return false;
  }
  *out = tmp;
  return true;
}

bool write_blob(const OilCalibrationBlob &blob) {
  Preferences prefs;
  if (!prefs.begin(kCfgNamespace, false)) {
    return false;
  }
  ScopedNvsWrite guard;
  size_t written = prefs.putBytes(kOilKey, &blob, sizeof(blob));
  prefs.end();
  return written == sizeof(blob);
}

bool validate_blob(const OilCalibrationBlob &blob) {
  if (blob.version != kVersion) {
    return false;
  }
  if (!isfinite(blob.v0_adc) || !isfinite(blob.v1_adc)) {
    return false;
  }
  if (blob.v0_adc < 0.0f || blob.v1_adc <= blob.v0_adc || blob.v1_adc > 4.0f) {
    return false;
  }
  uint32_t expected = blob_crc(blob);
  return expected == blob.crc32;
}

}  // namespace

bool nvsWriteInProgress = false;

ScopedNvsWrite::ScopedNvsWrite()  { nvsWriteInProgress = true; }
ScopedNvsWrite::~ScopedNvsWrite() { nvsWriteInProgress = false; }

bool oil_calibration_load(float *out_v0, float *out_v1) {
  OilCalibrationBlob blob;
  if (!read_blob(&blob)) {
    return false;
  }
  if (!validate_blob(blob)) {
    Serial.println("[CAL] Invalid oil calibration blob; using defaults.");
    return false;
  }
  if (out_v0) *out_v0 = blob.v0_adc;
  if (out_v1) *out_v1 = blob.v1_adc;
  return true;
}

bool oil_calibration_save(float v0, float v1) {
  if (!isfinite(v0) || !isfinite(v1) || v0 < 0.0f || v1 <= v0 || v1 > 4.0f) {
    Serial.println("[CAL] Refusing to save invalid oil calibration.");
    return false;
  }
  OilCalibrationBlob blob;
  blob.version = kVersion;
  blob.v0_adc  = v0;
  blob.v1_adc  = v1;
  blob.crc32   = blob_crc(blob);
  return write_blob(blob);
}

bool oil_calibration_read_raw(OilCalibrationRaw *out) {
  if (!out) return false;
  memset(out, 0, sizeof(*out));
  OilCalibrationBlob blob;
  if (!read_blob(&blob)) {
    out->present = false;
    out->valid = false;
    return false;
  }
  out->present       = true;
  out->version       = blob.version;
  out->v0_adc        = blob.v0_adc;
  out->v1_adc        = blob.v1_adc;
  out->stored_crc32  = blob.crc32;
  out->computed_crc32 = blob_crc(blob);
  out->valid         = validate_blob(blob);
  return true;
}

