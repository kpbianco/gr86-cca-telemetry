#pragma once

#include <stddef.h>
#include <stdint.h>

struct esp_partition_t;

void ota_update_init();
void ota_update_mark_attempt(uint32_t sequence);
void ota_update_mark_failure();
bool ota_update_verify_image(const esp_partition_t *partition,
                             size_t                 image_size,
                             const uint8_t         *expected_digest);
const char *ota_update_failure_reason();

