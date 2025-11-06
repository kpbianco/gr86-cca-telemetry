#include "ota_update.h"

#include <Arduino.h>

#include <esp_attr.h>
#include <esp_err.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_spi_flash.h>
#include <esp_timer.h>

#include <mbedtls/sha256.h>

#include <limits.h>
#include <stdio.h>
#include <string.h>

namespace {

constexpr uint32_t OTA_ROLLBACK_MAGIC = 0xCCA0DA42u;

struct RollbackState {
  uint32_t magic;
  uint32_t sequence;
  uint32_t failure_count;
  uint64_t last_attempt_us;
  bool     pending_apply;
};

RTC_NOINIT_ATTR RollbackState s_rollback_state;

uint64_t rtc_time_us() {
  return static_cast<uint64_t>(esp_timer_get_time());
}

void reset_rollback_state() {
  memset(&s_rollback_state, 0, sizeof(s_rollback_state));
  s_rollback_state.magic = OTA_ROLLBACK_MAGIC;
  s_rollback_state.last_attempt_us = rtc_time_us();
}

bool verify_signature_for_partition(const esp_partition_t *partition,
                                    size_t                 image_size,
                                    const uint8_t         *expected_digest) {
  if (!partition || !expected_digest || image_size == 0 ||
      image_size > partition->size) {
    return false;
  }

  const void *mapped = nullptr;
  esp_partition_mmap_handle_t handle = 0;

#ifdef ESP_PARTITION_MMAP_DATA
  esp_partition_mmap_memory_t memory_type = ESP_PARTITION_MMAP_DATA;
#else
  spi_flash_mmap_memory_t memory_type = SPI_FLASH_MMAP_DATA;
#endif

  esp_err_t err = esp_partition_mmap(partition,
                                     0,
                                     image_size,
                                     memory_type,
                                     &mapped,
                                     &handle);
  if (err != ESP_OK) {
    Serial.printf("[OTA] esp_partition_mmap failed: %d\n", static_cast<int>(err));
    return false;
  }

  mbedtls_sha256_context sha;
  mbedtls_sha256_init(&sha);
  int rc = mbedtls_sha256_starts_ret(&sha, 0);
  if (rc == 0) {
    rc = mbedtls_sha256_update_ret(&sha,
                                   static_cast<const unsigned char*>(mapped),
                                   image_size);
  }

  unsigned char digest[32];
  if (rc == 0) {
    rc = mbedtls_sha256_finish_ret(&sha, digest);
  }
  mbedtls_sha256_free(&sha);
  bool matches = false;
  if (rc == 0) {
    matches = memcmp(digest, expected_digest, sizeof(digest)) == 0;
  } else {
    Serial.printf("[OTA] SHA-256 computation failed: %d\n", rc);
  }
  esp_partition_munmap(handle);
  return matches;
}

}  // namespace

void ota_update_init() {
  if (s_rollback_state.magic != OTA_ROLLBACK_MAGIC) {
    reset_rollback_state();
    return;
  }

  // If we have a pending apply and the device reached setup(), consider the
  // boot successful and clear the flag.
  if (s_rollback_state.pending_apply) {
    s_rollback_state.pending_apply = false;
    s_rollback_state.failure_count = 0;
  }
}

void ota_update_mark_attempt(uint32_t sequence) {
  if (s_rollback_state.magic != OTA_ROLLBACK_MAGIC) {
    reset_rollback_state();
  }
  s_rollback_state.sequence       = sequence;
  s_rollback_state.last_attempt_us = rtc_time_us();
  s_rollback_state.pending_apply  = true;
}

void ota_update_mark_failure() {
  if (s_rollback_state.magic != OTA_ROLLBACK_MAGIC) {
    reset_rollback_state();
    return;
  }
  if (s_rollback_state.failure_count < UINT32_MAX) {
    s_rollback_state.failure_count++;
  }
  s_rollback_state.pending_apply = false;
}

bool ota_update_verify_image(const esp_partition_t *partition,
                             size_t                 image_size,
                             const uint8_t         *expected_digest) {
  // expected_digest must point at a 32-byte SHA-256 digest of the desired
  // firmware image.
  return verify_signature_for_partition(partition, image_size, expected_digest);
}

const char *ota_update_failure_reason() {
  if (s_rollback_state.magic != OTA_ROLLBACK_MAGIC) {
    return "uninitialized";
  }
  if (s_rollback_state.failure_count == 0) {
    return nullptr;
  }
  static char reason[64];
  snprintf(reason,
           sizeof(reason),
           "seq=%lu failures=%lu",
           static_cast<unsigned long>(s_rollback_state.sequence),
           static_cast<unsigned long>(s_rollback_state.failure_count));
  return reason;
}

