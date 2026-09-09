#include "oil_model.h"
#include "nvs_cfg.h"
#include <Arduino.h>
#include <Preferences.h>
#include <cstddef>
namespace oil {
namespace {
uint32_t checksum(const Calibration& c) {
  uint32_t crc = 0xFFFFFFFFu;
  const auto* bytes = reinterpret_cast<const uint8_t*>(&c);
  for (size_t i = 0; i < offsetof(Calibration, crc); ++i) {
    crc ^= bytes[i];
    for (int bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return crc ^ 0xFFFFFFFFu;
}
}
bool load(Calibration& c) {
  Preferences p;
  if (!p.begin("cca_cfg_b", true)) return false;
  Calibration loaded{};
  const size_t count = p.getBytes("oil_model", &loaded, sizeof(loaded));
  p.end();
  if (count != sizeof(loaded) || !identityMatches(loaded, ESP.getEfuseMac()) ||
      loaded.crc != checksum(loaded))
    return false;
  c = loaded;
  return true;
}
bool save(Calibration& c) {
  c.version = kVersion;
  c.deviceId = ESP.getEfuseMac();
  if (!c.deviceId) return false;
  c.crc = checksum(c);
  Preferences p;
  if (!p.begin("cca_cfg_b", false)) return false;
  ScopedNvsWrite guard;
  const bool ok = p.putBytes("oil_model", &c, sizeof(c)) == sizeof(c);
  p.end();
  return ok;
}
} // namespace oil
