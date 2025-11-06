#pragma once

#include <stddef.h>
#include <stdint.h>

// Initialize OTA update subsystem. Call once during setup.
void ota_update_init();

// Service routine; call periodically from loop() with current millis.
void ota_update_service(uint32_t now_ms);

// Start OTA update from given URL. Returns true if download started and completed successfully.
// On success this function does not return because the device reboots.
bool ota_update_start(const char *url);

// Returns true if an OTA update is currently running.
bool ota_update_in_progress();

// Exposed for boot crash bookkeeping when a new image is about to boot.
void ota_update_prepare_for_pending_boot();
