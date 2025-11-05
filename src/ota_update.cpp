#include "ota_update.h"

#include <Arduino.h>
#include <esp_http_client.h>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <esp_spi_flash.h>
#include <esp_partition.h>
#include <cstring>
#include <cstdlib>

#if __has_include(<psa/crypto.h>)
#include <psa/crypto.h>
#define OTA_HAVE_PSA_CRYPTO 1
#else
#define OTA_HAVE_PSA_CRYPTO 0
#endif

#if __has_include(<esp_idf_version.h>)
#include <esp_idf_version.h>
#endif

#if __has_include(<esp_crt_bundle.h>)
#include <esp_crt_bundle.h>
#endif

#if __has_include(<esp_private/esp_clk.h>)
#include <esp_private/esp_clk.h>
#endif

#ifndef OTA_SIGNATURE_MAX_SIZE
#define OTA_SIGNATURE_MAX_SIZE 64
#endif

#ifndef OTA_ROLLBACK_MAGIC
#define OTA_ROLLBACK_MAGIC 0xCCA0TA42u
#endif

#ifndef OTA_ROLLBACK_WINDOW_SECONDS
#define OTA_ROLLBACK_WINDOW_SECONDS 60
#endif

#ifndef OTA_ROLLBACK_MAX_CRASHES
#define OTA_ROLLBACK_MAX_CRASHES 2
#endif

#ifndef OTA_DOWNLOAD_BUFFER_SIZE
#define OTA_DOWNLOAD_BUFFER_SIZE 4096
#endif

#ifndef OTA_HTTP_TIMEOUT_MS
#define OTA_HTTP_TIMEOUT_MS 30000
#endif

extern const uint8_t OTA_SIGNING_PUBLIC_KEY[32];

namespace {

RTC_DATA_ATTR static struct {
  uint32_t magic;
  uint32_t crash_count;
  uint32_t pending_active;
  uint64_t first_boot_time_us;
} s_rollback_state;

static bool s_in_progress = false;
static bool s_pending_boot = false;
static uint32_t s_pending_boot_start_ms = 0;

static uint64_t rtc_time_us() {
#if defined(ESP_IDF_VERSION)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
  return esp_rtc_get_time_us();
#else
  return esp_timer_get_time();
#endif
#else
  return esp_timer_get_time();
#endif
}

static bool is_crash_reset(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_PANIC:
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
    case ESP_RST_DEEPSLEEP:
    case ESP_RST_BROWNOUT:
    case ESP_RST_SDIO:
      return true;
    default:
      return false;
  }
}

static bool verify_signature_for_partition(const esp_partition_t *partition,
                                           size_t image_size,
                                           const uint8_t signature[OTA_SIGNATURE_MAX_SIZE]);
#if OTA_HAVE_PSA_CRYPTO
static psa_status_t ensure_psa_initialized();
#endif

static void reset_rollback_state() {
  s_rollback_state.magic = OTA_ROLLBACK_MAGIC;
  s_rollback_state.crash_count = 0;
  s_rollback_state.pending_active = 0;
  s_rollback_state.first_boot_time_us = 0;
}

} // namespace

void ota_update_prepare_for_pending_boot() {
  reset_rollback_state();
  s_rollback_state.pending_active = 1;
  s_rollback_state.first_boot_time_us = rtc_time_us();
  s_rollback_state.crash_count = 0;
  s_pending_boot = true;
  s_pending_boot_start_ms = millis();
}

void ota_update_init() {
  if (s_rollback_state.magic != OTA_ROLLBACK_MAGIC) {
    reset_rollback_state();
  }

  const esp_partition_t *running = esp_ota_get_running_partition();
  if (!running) {
    Serial.println("OTA: failed to query running partition");
    return;
  }

  esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
  if (esp_ota_get_state_partition(running, &state) != ESP_OK) {
    state = ESP_OTA_IMG_UNDEFINED;
  }

  if (state == ESP_OTA_IMG_PENDING_VERIFY) {
    uint64_t now_us = rtc_time_us();
    esp_reset_reason_t reason = esp_reset_reason();

    if (!s_rollback_state.pending_active) {
      ota_update_prepare_for_pending_boot();
    } else {
      if (now_us >= s_rollback_state.first_boot_time_us) {
        uint64_t delta = now_us - s_rollback_state.first_boot_time_us;
        if (delta > (uint64_t)OTA_ROLLBACK_WINDOW_SECONDS * 1000000ULL) {
          s_rollback_state.first_boot_time_us = now_us;
          s_rollback_state.crash_count = 0;
        }
      } else {
        s_rollback_state.first_boot_time_us = now_us;
        s_rollback_state.crash_count = 0;
      }

      if (is_crash_reset(reason)) {
        if (s_rollback_state.crash_count < 0xFFFFFFFFu) {
          s_rollback_state.crash_count++;
        }
        if (s_rollback_state.crash_count > OTA_ROLLBACK_MAX_CRASHES) {
          Serial.println("OTA: rollback triggered due to repeated crashes");
          esp_ota_mark_app_invalid_rollback_and_reboot();
        }
      }
    }

    s_pending_boot = true;
    s_pending_boot_start_ms = millis();
  } else {
    s_pending_boot = false;
    s_pending_boot_start_ms = 0;
    s_rollback_state.pending_active = 0;
    s_rollback_state.crash_count = 0;
  }
}

void ota_update_service(uint32_t now_ms) {
  if (!s_pending_boot) {
    return;
  }

  const esp_partition_t *running = esp_ota_get_running_partition();
  if (!running) {
    return;
  }

  esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
  if (esp_ota_get_state_partition(running, &state) != ESP_OK) {
    return;
  }

  if (state == ESP_OTA_IMG_PENDING_VERIFY) {
    if ((uint32_t)(now_ms - s_pending_boot_start_ms) >= (OTA_ROLLBACK_WINDOW_SECONDS * 1000U)) {
      Serial.println("OTA: image validated after soak period");
      esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
      if (err != ESP_OK) {
        Serial.printf("OTA: failed to mark image valid (%d)\n", (int)err);
      } else {
        s_pending_boot = false;
        s_rollback_state.pending_active = 0;
        s_rollback_state.crash_count = 0;
      }
    }
  } else {
    s_pending_boot = false;
    s_rollback_state.pending_active = 0;
    s_rollback_state.crash_count = 0;
  }
}

bool ota_update_in_progress() {
  return s_in_progress;
}

bool ota_update_start(const char *url) {
  if (!url || !url[0]) {
    Serial.println("OTA: invalid URL");
    return false;
  }
  if (s_in_progress) {
    Serial.println("OTA: update already in progress");
    return false;
  }

  const esp_partition_t *update_partition = esp_ota_get_next_update_partition(nullptr);
  if (!update_partition) {
    Serial.println("OTA: no OTA partition available");
    return false;
  }

  Serial.printf("OTA: starting update from %s\n", url);
  s_in_progress = true;

  esp_http_client_config_t config = {};
  config.url = url;
  config.timeout_ms = OTA_HTTP_TIMEOUT_MS;
  config.keep_alive_enable = true;
#if defined(CONFIG_MBEDTLS_CERTIFICATE_BUNDLE)
  config.crt_bundle_attach = esp_crt_bundle_attach;
#endif
  config.skip_cert_common_name_check = true;

  esp_http_client_handle_t client = esp_http_client_init(&config);
  if (!client) {
    Serial.println("OTA: failed to init HTTP client");
    s_in_progress = false;
    return false;
  }

  esp_err_t err = esp_http_client_open(client, 0);
  if (err != ESP_OK) {
    Serial.printf("OTA: http open failed (%d)\n", (int)err);
    esp_http_client_cleanup(client);
    s_in_progress = false;
    return false;
  }

  esp_http_client_fetch_headers(client);

  esp_ota_handle_t ota_handle = 0;
  err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &ota_handle);
  if (err != ESP_OK) {
    Serial.printf("OTA: esp_ota_begin failed (%d)\n", (int)err);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    s_in_progress = false;
    return false;
  }

  uint8_t *read_buf = static_cast<uint8_t*>(malloc(OTA_DOWNLOAD_BUFFER_SIZE));
  if (!read_buf) {
    Serial.println("OTA: failed to allocate download buffer");
    esp_ota_abort(ota_handle);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    s_in_progress = false;
    return false;
  }

  uint8_t tail[OTA_SIGNATURE_MAX_SIZE];
  size_t tail_len = 0;
  size_t total_received = 0;
  size_t total_written = 0;
  bool success = false;

  while (true) {
    int read = esp_http_client_read(client, reinterpret_cast<char*>(read_buf), OTA_DOWNLOAD_BUFFER_SIZE);
    if (read < 0) {
      Serial.printf("OTA: read error (%d)\n", read);
      break;
    }
    if (read == 0) {
      success = true;
      break;
    }

    esp_task_wdt_reset();

    size_t read_len = static_cast<size_t>(read);
    total_received += read_len;

    uint8_t temp[OTA_DOWNLOAD_BUFFER_SIZE + OTA_SIGNATURE_MAX_SIZE];
    size_t combined = tail_len + read_len;
    memcpy(temp, tail, tail_len);
    memcpy(temp + tail_len, read_buf, read_len);

    if (combined <= OTA_SIGNATURE_MAX_SIZE) {
      memcpy(tail, temp, combined);
      tail_len = combined;
      continue;
    }

    size_t write_len = combined - OTA_SIGNATURE_MAX_SIZE;
    err = esp_ota_write(ota_handle, temp, write_len);
    if (err != ESP_OK) {
      Serial.printf("OTA: write failed (%d)\n", (int)err);
      success = false;
      break;
    }
    total_written += write_len;
    memcpy(tail, temp + write_len, OTA_SIGNATURE_MAX_SIZE);
    tail_len = OTA_SIGNATURE_MAX_SIZE;

    Serial.printf("OTA: downloaded %u bytes\n", (unsigned)total_received);
  }

  free(read_buf);
  esp_http_client_close(client);
  esp_http_client_cleanup(client);

  if (!success) {
    esp_ota_abort(ota_handle);
    s_in_progress = false;
    return false;
  }

  if (total_received <= OTA_SIGNATURE_MAX_SIZE) {
    Serial.println("OTA: payload too small (missing signature)");
    esp_ota_abort(ota_handle);
    s_in_progress = false;
    return false;
  }

  if (tail_len != OTA_SIGNATURE_MAX_SIZE) {
    Serial.println("OTA: signature missing");
    esp_ota_abort(ota_handle);
    s_in_progress = false;
    return false;
  }

  size_t image_size = total_received - OTA_SIGNATURE_MAX_SIZE;
  Serial.printf("OTA: total firmware size %u bytes\n", (unsigned)image_size);

  if (!verify_signature_for_partition(update_partition, image_size, tail)) {
    Serial.println("OTA: signature verification failed");
    esp_ota_abort(ota_handle);
    s_in_progress = false;
    return false;
  }

  err = esp_ota_end(ota_handle);
  if (err != ESP_OK) {
    Serial.printf("OTA: esp_ota_end failed (%d)\n", (int)err);
    s_in_progress = false;
    return false;
  }

  err = esp_ota_set_boot_partition(update_partition);
  if (err != ESP_OK) {
    Serial.printf("OTA: failed to set boot partition (%d)\n", (int)err);
    s_in_progress = false;
    return false;
  }

  Serial.println("OTA: update installed, rebooting to pending image");
  reset_rollback_state();
  s_in_progress = false;
  Serial.flush();
  delay(100);
  esp_restart();
  return true;
}

static bool verify_signature_for_partition(const esp_partition_t *partition,
                                           size_t image_size,
                                           const uint8_t signature[OTA_SIGNATURE_MAX_SIZE]) {
  if (!partition || image_size == 0) {
    return false;
  }

  spi_flash_mmap_handle_t handle = 0;
  const void *mapped = nullptr;
  esp_err_t err = esp_partition_mmap(partition, 0, image_size, SPI_FLASH_MMAP_DATA, &mapped, &handle);
  if (err != ESP_OK) {
    Serial.printf("OTA: mmap failed (%d)\n", (int)err);
    return false;
  }

  const uint8_t *data = static_cast<const uint8_t*>(mapped);

  bool ok = false;
#if OTA_HAVE_PSA_CRYPTO
  if (ensure_psa_initialized() != PSA_SUCCESS) {
    Serial.println("OTA: PSA crypto init failed");
  } else {
    psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
    psa_set_key_type(&attributes, PSA_KEY_TYPE_ECC_PUBLIC_KEY(PSA_ECC_FAMILY_TWISTED_EDWARDS));
    psa_set_key_bits(&attributes, 256);
    psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_VERIFY_MESSAGE);
    psa_set_key_algorithm(&attributes, PSA_ALG_PURE_EDDSA);

    psa_key_handle_t handle_key = 0;
    psa_status_t status = psa_import_key(&attributes, OTA_SIGNING_PUBLIC_KEY, 32, &handle_key);
    if (status == PSA_SUCCESS) {
      status = psa_verify_message(handle_key, PSA_ALG_PURE_EDDSA, data, image_size, signature, OTA_SIGNATURE_MAX_SIZE);
      ok = (status == PSA_SUCCESS);
      if (status != PSA_SUCCESS) {
        Serial.printf("OTA: psa_verify_message failed (%ld)\n", (long)status);
      } else {
        Serial.println("OTA: signature verified");
      }
      psa_destroy_key(handle_key);
    } else {
      Serial.printf("OTA: psa_import_key failed (%ld)\n", (long)status);
    }
  }
#else
  Serial.println("OTA: PSA crypto unavailable; cannot verify signature");
#endif

  spi_flash_munmap(handle);
  return ok;
}

#if OTA_HAVE_PSA_CRYPTO
static psa_status_t ensure_psa_initialized() {
  static bool initialized = false;
  if (initialized) {
    return PSA_SUCCESS;
  }
  psa_status_t status = psa_crypto_init();
  if (status == PSA_SUCCESS) {
    initialized = true;
  }
  return status;
}
#endif

