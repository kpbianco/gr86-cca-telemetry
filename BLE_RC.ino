// ===== CCA Telemetry: ESP32-S3 + TWAI + RaceChrono DIY BLE (CAN + GPS) =====
// - Robust BLE reconnect (no NimBLE deinit on S3; stop/start advertiser only)
// - TWAI CAN RX burst buffering + per-PID throttle
// - Profile allow-list (car mode) vs sniff-all (bench mode)
// - Oil pressure analog on GPIO1 -> CAN 0x710 (0.1 psi/bit, big-endian)
// - Flash config (Preferences): V0_ADC, V1_ADC, PROFILE, custom dividers
// - RaceChrono DIY BLE service 0x1FF8 with:
//      0x0001 CAN main (READ+NOTIFY)
//      0x0002 CAN filter (WRITE)
//      0x0003 GPS main (READ+NOTIFY)
//      0x0004 GPS time (READ+NOTIFY)
// - Serial CLI (CR, LF, CRLF, or none)
//
// Libs:
//   ESP32-TWAI-CAN
//   NimBLE-Arduino
//   Preferences
//
// Board: ESP32S3 Dev Module

#include <ESP32-TWAI-CAN.hpp>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <cstdio>
#include <cstring>
#include <ctype.h>
#include <driver/twai.h>
#include <math.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include "config.h"   // must define DEFAULT_UPDATE_RATE_DIVIDER and DEVICE_NAME

// ---------- Version ----------
#ifndef FW_VERSION
#define FW_VERSION "0.6.1"
#endif
#ifndef FW_GITHASH
#define FW_GITHASH "esp32s3"
#endif
static const char FW_VERSION_STRING[] = FW_VERSION " (" FW_GITHASH ")";

// ===== Pins (S3 <-> SN65HVD230) =====
#define CAN_TX 5
#define CAN_RX 4

// ===== GPS pins (UART1) =====
#define GPS_RX_GPIO 18   // GPS TX -> ESP RX
#define GPS_TX_GPIO 17   // GPS RX <- ESP TX (PMTK)

// ===== GPS PMTK sequences =====
static const char *PMTK_SET_NMEA_OUTPUT_RMCGGA = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
static const char *PMTK_SET_NMEA_UPDATE_10HZ   = "$PMTK220,100*2F\r\n";
static const char *PMTK_SET_BAUD_115200        = "$PMTK251,115200*1F\r\n";

// ===== BLE UUIDs (RaceChrono DIY) =====
static const char* RC_SERVICE_UUID  = "00001ff8-0000-1000-8000-00805f9b34fb";
static const char* RC_CHAR_CAN_UUID = "00000001-0000-1000-8000-00805f9b34fb"; // READ+NOTIFY
static const char* RC_CHAR_FIL_UUID = "00000002-0000-1000-8000-00805f9b34fb"; // WRITE
static const char* RC_CHAR_GPS_UUID = "00000003-0000-1000-8000-00805f9b34fb"; // READ+NOTIFY
static const char* RC_CHAR_GTM_UUID = "00000004-0000-1000-8000-00805f9b34fb"; // READ+NOTIFY

// ====== (Fix) Governor snapshot: define early so Arduino doesn't mangle prototypes ======
struct GovernorSnapshot { uint32_t pid; uint8_t governor; };
static size_t capture_governor_snapshot(GovernorSnapshot *out);
static bool   restore_governor_snapshot(const GovernorSnapshot *snapshots, size_t count);

// ===== Global state =====
static bool     isCanBusReaderActive = false;
static uint32_t lastCanMessageReceivedMs = 0;
static uint32_t loop_iteration = 0;
static uint32_t last_time_num_can_bus_timeouts_sent_ms = 0;
static uint16_t num_can_bus_timeouts = 0;

static bool     have_seen_any_can = false;
static uint32_t lastHbMs = 0;
static uint16_t hbCounter = 0;
static const uint32_t HB_PID = 0x7AA; // BLE heartbeat

static uint32_t rxCount = 0;
static uint32_t lastRxPrintMs = 0;

static char lastReconnectCause[64] = "boot";
static char pendingBleRestartReason[64] = "";
static bool pendingBleRestart = false;

static bool nvsWriteInProgress = false;

static constexpr int TASK_WDT_TIMEOUT_SECONDS = 5;

static constexpr uint32_t CAN_STATUS_POLL_INTERVAL_MS = 100;
static constexpr uint32_t CAN_TX_FAILURE_WINDOW_MS    = 1000;
static constexpr uint8_t  CAN_TX_FAILURE_THRESHOLD    = 5;

static uint32_t lastCanStatusCheckMs    = 0;
static uint32_t canTxFailureWindowStart = 0;
static uint8_t  canTxFailuresInWindow   = 0;

// ===== GPS state =====
static HardwareSerial GPSSerial(1);
static bool gpsConfigured = false;
static uint32_t gpsLastSentenceMs = 0;
static uint32_t lastGpsNotifyMs = 0;
static uint32_t lastGpsInitAttemptMs = 0;
static constexpr uint32_t GPS_NOTIFY_PERIOD_MS = 100;  // 10 Hz
static constexpr uint32_t GPS_INIT_RETRY_MS    = 5000;

// ====== BLE objects we own (we create the full 0x1FF8 service) ======
static NimBLEServer*         g_server = nullptr;
static NimBLEAdvertising*    g_adv    = nullptr;
static NimBLEService*        g_svc    = nullptr;
static NimBLECharacteristic* g_can    = nullptr; // 0x0001
static NimBLECharacteristic* g_fil    = nullptr; // 0x0002
static NimBLECharacteristic* g_gps    = nullptr; // 0x0003
static NimBLECharacteristic* g_gtm    = nullptr; // 0x0004

// RMC fields
static int    rmc_hour = 0, rmc_min = 0, rmc_sec = 0, rmc_millis = 0;
static bool   rmc_valid = false;
static double rmc_lat_deg = 0.0, rmc_lon_deg = 0.0;
static double rmc_speed_kmh = 0.0;
static double rmc_course_deg = 0.0;

// GGA fields
static int    gga_sats = 0;
static double gga_hdop = 99.9;
static double gga_alt_m = 0.0;

// Date (UTC)
static int gps_year = 0, gps_mon = 0, gps_day = 0;

// Synchronization bits for RaceChrono DIY GPS payload
static uint8_t gpsSyncBits = 0;
static int lastDateHourPacked = -1;

static char gpsLineBuf[128];
static size_t gpsLineLen = 0;

// ===== RaceChrono pidMap extra (per-PID throttle) =====
struct PidExtra {
  uint8_t baseDivider      = DEFAULT_UPDATE_RATE_DIVIDER;
  uint8_t governorDivider  = 1;
  uint8_t skippedUpdates   = 0;
};

// Minimal pid map implementation for allow-listing + extras.
template <typename ExtraT>
class SimplePidMap {
public:
  struct Entry { uint32_t pid; uint16_t intervalMs; ExtraT extra; bool used; };
  static constexpr size_t MAX = 256;
  Entry entries[MAX];

  SimplePidMap(){ reset(); }

  void reset(){ for (auto &e: entries){ e.used=false; e.pid=0; e.intervalMs=0; e.extra=ExtraT(); } }

  bool isEmpty() const { for (auto &e: entries){ if (e.used) return false; } return true; }

  bool allowOnePid(uint32_t pid, uint16_t ms){
    if (getEntryId(pid)) return true;
    for (auto &e: entries){
      if (!e.used){ e.used=true; e.pid=pid; e.intervalMs=ms; e.extra=ExtraT(); return true; }
    }
    return false;
  }

  void allowAllPids(uint16_t ms){
    (void)ms; // not pre-filling in this simple map
  }

  void* getEntryId(uint32_t pid){
    for (auto &e: entries){ if (e.used && e.pid==pid) return &e; }
    return nullptr;
  }

  uint32_t getPid(void* entry){ return ((Entry*)entry)->pid; }
  ExtraT*  getExtra(void* entry){ return &((Entry*)entry)->extra; }

  template<typename F>
  void forEach(F f){ for (auto &e: entries){ if (e.used) f((void*)&e); } }
};

static SimplePidMap<PidExtra> pidMap;

static const char *reset_reason_to_string(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON:     return "POWERON";
    case ESP_RST_EXT:         return "EXT";
    case ESP_RST_SW:          return "SW";
    case ESP_RST_PANIC:       return "PANIC";
    case ESP_RST_INT_WDT:     return "INT_WDT";
    case ESP_RST_TASK_WDT:    return "TASK_WDT";
    case ESP_RST_WDT:         return "WDT";
    case ESP_RST_DEEPSLEEP:   return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:    return "BROWNOUT";
    case ESP_RST_SDIO:        return "SDIO";
    case ESP_RST_RTC_WDT:     return "RTC_WDT";
    case ESP_RST_USB:         return "USB";
    case ESP_RST_CPU_LOCKUP:  return "CPU_LOCKUP";
    case ESP_RST_JTAG:        return "JTAG";
    case ESP_RST_TIME_WDT:    return "TIME_WDT";
    case ESP_RST_MWDT0:       return "MWDT0";
    case ESP_RST_MWDT1:       return "MWDT1";
    case ESP_RST_RTC_MWDT0:   return "RTC_MWDT0";
    case ESP_RST_RTC_MWDT1:   return "RTC_MWDT1";
    default:                  return "UNKNOWN";
  }
}

static void init_task_watchdog() {
  esp_err_t err = esp_task_wdt_init(TASK_WDT_TIMEOUT_SECONDS, true);
  if (err == ESP_ERR_INVALID_STATE) {
    // Already initialized with a different timeout; reset and try again.
    esp_task_wdt_deinit();
    err = esp_task_wdt_init(TASK_WDT_TIMEOUT_SECONDS, true);
  }
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    Serial.printf("WARN: esp_task_wdt_init failed: %d\n", (int)err);
    return;
  }

  err = esp_task_wdt_add(NULL);
  if (err == ESP_ERR_INVALID_STATE) {
    Serial.println("Task WDT already armed for loop task.");
    return;
  }
  if (err != ESP_OK) {
    Serial.printf("WARN: esp_task_wdt_add failed: %d\n", (int)err);
  } else {
    Serial.printf("Task WDT armed (%ds).\n", TASK_WDT_TIMEOUT_SECONDS);
  }
}

static uint8_t effective_divider(const PidExtra *extra) {
  uint16_t base = extra ? extra->baseDivider : 1;
  uint16_t gov  = extra ? extra->governorDivider : 1;
  if (base == 0) base = 1;
  if (gov == 0) gov = 1;
  uint32_t eff = base * gov;
  if (eff == 0) eff = 1;
  if (eff > 255) eff = 255;
  return static_cast<uint8_t>(eff);
}

static void set_extra_base_divider(PidExtra *extra, uint8_t div, bool resetGovernor) {
  if (!extra) return;
  if (div == 0) div = 1;
  extra->baseDivider = div;
  if (resetGovernor || extra->governorDivider == 0) {
    extra->governorDivider = 1;
  }
  extra->skippedUpdates = 0;
}

static void apply_divider_to_pidmap(uint16_t pid, uint8_t div, bool resetGovernor = false) {
  void *entry = pidMap.getEntryId(pid);
  if (!entry) return;
  PidExtra *extra = pidMap.getExtra(entry);
  set_extra_base_divider(extra, div, resetGovernor);
}

// ===== Deny list (CLI-managed) =====
static const size_t DENY_MAX = 64;
static uint32_t denyList[DENY_MAX];
static size_t   denyCount = 0;

static bool isDenied(uint32_t pid) {
  for (size_t i = 0; i < denyCount; ++i) if (denyList[i] == pid) return true;
  return false;
}
static void addDeny(uint32_t pid) {
  if (isDenied(pid)) return;
  if (denyCount < DENY_MAX) denyList[denyCount++] = pid;
}
static void removeDeny(uint32_t pid) {
  for (size_t i = 0; i < denyCount; ++i) {
    if (denyList[i] == pid) {
      for (size_t j = i + 1; j < denyCount; ++j) denyList[j-1] = denyList[j];
      denyCount--;
      return;
    }
  }
}
static void clearDeny() { denyCount = 0; }

// ===== Profile allow-list =====
struct PidRule { uint32_t pid; uint8_t divider; };

// Seed list (extend as you validate):
static const PidRule CAR_PROFILE[] = {
  { 0x040, 4 },   // 100 Hz block -> 25 Hz
  { 0x710, 1 },   // your oil channel
};
static const size_t CAR_PROFILE_LEN = sizeof(CAR_PROFILE)/sizeof(CAR_PROFILE[0]);

// Divider hint
static uint8_t divider_override(uint32_t can_id) {
  if (can_id == 0x139) return 1;  // brake/speed critical
  if (can_id == 0x345) return 10; // slow group example
  return 0;
}

// Toggle: PROFILE allow-list (true) vs sniff-all (false)
static constexpr bool DEFAULT_USE_PROFILE_ALLOWLIST = true;
static bool USE_PROFILE_ALLOWLIST = DEFAULT_USE_PROFILE_ALLOWLIST;

// ======== Oil pressure (analog) ========
#define OIL_ADC_PIN         1       // GPIO1 (ADC1_CH0)
#define ADC_SAMPLES         8
#define ADC_IIR_ALPHA       0.15f
#define ADC_DEADBAND_PSI    0.5f

// Pressure range
static const float P0_PSI = 0.0f;
static const float P1_PSI = 150.0f;

// Cal endpoints at the ADC pin (defaults; persisted in flash)
static constexpr float DEFAULT_V0_ADC = 0.3482f;   // 0 psi
static constexpr float DEFAULT_V1_ADC = 3.1290f;   // 150 psi
static float V0_ADC = DEFAULT_V0_ADC;   // 0 psi
static float V1_ADC = DEFAULT_V1_ADC;   // 150 psi

// Publish to BLE CAN 0x710
static const uint32_t OIL_CAN_ID      = 0x710;  // private 11-bit
static const uint16_t OIL_SCALE_01PSI = 10;     // 0.1 psi/bit
static uint16_t       OIL_TX_RATE_MS  = 40;     // persisted in flash

// Runtime
static float    oil_psi_f  = 0.0f;
static uint8_t  oil_flags  = 0;      // bit0=open, bit1=short, bit2=oor
static uint32_t lastOilTxMs = 0;

// ===== Flash (Preferences) =====
static Preferences cfgPrefs;   // namespace "cca_cfg"
static constexpr const char *CFG_NAMESPACE = "cca_cfg";

static const size_t CUSTOM_DIVIDER_MAX = 64;

struct __attribute__((packed)) CfgPidDivider {
  uint16_t pid;
  uint8_t div;
};

struct __attribute__((packed)) CfgDividerBlob {
  uint16_t count;
  CfgPidDivider items[CUSTOM_DIVIDER_MAX];
};

static CfgPidDivider customDividers[CUSTOM_DIVIDER_MAX];
static uint16_t customDividerCount = 0;

struct ScopedNvsWrite {
  ScopedNvsWrite()  { nvsWriteInProgress = true; }
  ~ScopedNvsWrite() { nvsWriteInProgress = false; }
};

static int find_custom_divider_index(uint16_t pid) {
  for (uint16_t i = 0; i < customDividerCount; ++i) {
    if (customDividers[i].pid == pid) return i;
  }
  return -1;
}

static uint8_t compute_default_divider_for_pid(uint32_t pid) {
  for (size_t i = 0; i < CAR_PROFILE_LEN; ++i) {
    if (CAR_PROFILE[i].pid == pid) {
      uint8_t div = CAR_PROFILE[i].divider;
      return div == 0 ? 1 : div;
    }
  }
  uint8_t div = divider_override(pid);
  if (!div) {
    if (pid == 0x139) div = 1;
    else if (pid == 0x345) div = 10;
    else if (pid < 0x100) div = 4;
    else if (pid < 0x200) div = 2;
    else if (pid > 0x700) div = 1; // OBD replies
    else div = DEFAULT_UPDATE_RATE_DIVIDER;
  }
  if (!div) div = 1;
  return div;
}

bool piddiv_set(uint16_t pid, uint8_t div) {
  if (div == 0) div = 1;
  int idx = find_custom_divider_index(pid);
  if (idx >= 0) {
    customDividers[idx].div = div;
  } else {
    if (customDividerCount >= CUSTOM_DIVIDER_MAX) return false;
    customDividers[customDividerCount].pid = pid;
    customDividers[customDividerCount].div = div;
    customDividerCount++;
  }
  apply_divider_to_pidmap(pid, div, /*resetGovernor=*/true);
  return true;
}

bool piddiv_clear(uint16_t pid) {
  int idx = find_custom_divider_index(pid);
  if (idx < 0) return false;
  for (uint16_t i = idx + 1; i < customDividerCount; ++i) {
    customDividers[i - 1] = customDividers[i];
  }
  customDividerCount--;
  apply_divider_to_pidmap(pid, compute_default_divider_for_pid(pid), /*resetGovernor=*/true);
  return true;
}

void piddiv_apply_all() {
  for (uint16_t i = 0; i < customDividerCount; ++i) {
    apply_divider_to_pidmap(customDividers[i].pid, customDividers[i].div, /*resetGovernor=*/false);
  }
}

// ===== BLE TX soft governor =====
static constexpr float    BLE_TX_FPS_LIMIT                = 180.0f;
static constexpr uint8_t  BLE_TX_GOVERNOR_MAX_DIVIDER     = 8;
static constexpr uint8_t  BLE_TX_OVER_LIMIT_SECONDS       = 3;
static constexpr uint32_t BLE_TX_OVER_LIMIT_DURATION_MS   = 3000;
static constexpr uint8_t  BLE_TX_RELAX_SECONDS            = 5;
static constexpr uint32_t BLE_TX_RELAX_DURATION_MS        = 5000;
static constexpr uint32_t BLE_TX_GOVERNOR_STEP_INTERVAL_MS = 1000;
static constexpr uint8_t  BLE_TX_HISTORY_SECONDS          = 6;
static constexpr size_t   BLE_GOVERNOR_SNAPSHOT_MAX       = 128;

static bool     bleTxBucketsInitialized = false;
static uint16_t bleTxSecondBuckets[BLE_TX_HISTORY_SECONDS];
static uint8_t  bleTxBucketIndex        = BLE_TX_HISTORY_SECONDS - 1;
static uint8_t  bleTxFilledSeconds      = 0;
static uint16_t bleTxCurrentSecondCount = 0;
static uint32_t bleTxCurrentSecondStartMs = 0;

static float    bleTxMovingAverageFps   = 0.0f;
static uint32_t bleTxOverLimitSinceMs   = 0;
static uint32_t bleTxBelowLimitSinceMs  = 0;
static uint32_t bleTxLastBoostMs        = 0;
static uint32_t bleTxLastRelaxMs        = 0;
static bool     bleGovernorActive       = false;

static size_t capture_governor_snapshot(GovernorSnapshot *out) {
  size_t count = 0;
  pidMap.forEach([&](void *entry) {
    if (count >= BLE_GOVERNOR_SNAPSHOT_MAX) return;
    const PidExtra *extra = pidMap.getExtra(entry);
    if (!extra) return;
    uint8_t gov = extra->governorDivider ? extra->governorDivider : 1;
    if (gov > 1) {
      out[count].pid = pidMap.getPid(entry);
      out[count].governor = gov;
      count++;
    }
  });
  return count;
}

static bool restore_governor_snapshot(const GovernorSnapshot *snapshots, size_t count) {
  bool any = false;
  for (size_t i = 0; i < count; ++i) {
    void *entry = pidMap.getEntryId(snapshots[i].pid);
    if (!entry) continue;
    PidExtra *extra = pidMap.getExtra(entry);
    uint8_t gov = snapshots[i].governor;
    if (gov < 1) gov = 1;
    extra->governorDivider = gov;
    extra->skippedUpdates = 0;
    if (gov > 1) any = true;
  }
  return any;
}

static void bleTxAdvanceBuckets(uint32_t now) {
  if (!bleTxBucketsInitialized) {
    memset(bleTxSecondBuckets, 0, sizeof(bleTxSecondBuckets));
    bleTxBucketIndex = BLE_TX_HISTORY_SECONDS - 1;
    bleTxFilledSeconds = 0;
    bleTxCurrentSecondCount = 0;
    bleTxCurrentSecondStartMs = now;
    bleTxBucketsInitialized = true;
    return;
  }

  while (now - bleTxCurrentSecondStartMs >= 1000) {
    bleTxBucketIndex = (bleTxBucketIndex + 1) % BLE_TX_HISTORY_SECONDS;
    if (bleTxFilledSeconds < BLE_TX_HISTORY_SECONDS) {
      bleTxFilledSeconds++;
    }
    bleTxSecondBuckets[bleTxBucketIndex] = bleTxCurrentSecondCount;
    bleTxCurrentSecondCount = 0;
    bleTxCurrentSecondStartMs += 1000;
  }
}

static void noteBleTxFrame(uint32_t now) {
  bleTxAdvanceBuckets(now);
  bleTxCurrentSecondCount++;
}

static uint32_t bleTxFramesInRecentSeconds(uint8_t seconds) {
  if (!bleTxBucketsInitialized || bleTxFilledSeconds == 0 || seconds == 0) {
    return 0;
  }
  if (seconds > bleTxFilledSeconds) seconds = bleTxFilledSeconds;
  uint32_t sum = 0;
  for (uint8_t i = 0; i < seconds; ++i) {
    uint8_t idx = (bleTxBucketIndex + BLE_TX_HISTORY_SECONDS - i) % BLE_TX_HISTORY_SECONDS;
    sum += bleTxSecondBuckets[idx];
  }
  return sum;
}

static bool governorIncreaseDividers() {
  bool changed = false;
  pidMap.forEach([&](void *entry) {
    PidExtra *extra = pidMap.getExtra(entry);
    uint8_t eff = effective_divider(extra);
    if (eff < BLE_TX_GOVERNOR_MAX_DIVIDER) {
      uint8_t gov = extra->governorDivider ? extra->governorDivider : 1;
      if (gov < 128) {
        gov *= 2;
        if (gov == 0) gov = 1;
        extra->governorDivider = gov;
        extra->skippedUpdates = 0;
        changed = true;
      }
    }
  });
  if (changed) bleGovernorActive = true;
  return changed;
}

static bool governorRelaxDividers() {
  bool changed = false;
  bool stillActive = false;
  pidMap.forEach([&](void *entry) {
    PidExtra *extra = pidMap.getExtra(entry);
    uint8_t base = extra->baseDivider ? extra->baseDivider : 1;
    uint8_t eff = effective_divider(extra);
    if (eff > base) {
      uint8_t gov = extra->governorDivider ? extra->governorDivider : 1;
      if (gov > 1) {
        uint8_t newGov = gov / 2;
        if (newGov < 1) newGov = 1;
        extra->governorDivider = newGov;
        extra->skippedUpdates = 0;
        changed = true;
        eff = effective_divider(extra);
      } else {
        extra->governorDivider = 1;
        eff = effective_divider(extra);
      }
    }
    if (eff > base) {
      stillActive = true;
    }
  });
  bleGovernorActive = stillActive;
  return changed;
}

static void updateBleGovernor(uint32_t now) {
  bleTxAdvanceBuckets(now);

  float avgSeconds = static_cast<float>(bleTxFilledSeconds);
  uint32_t sumCompleted = bleTxFramesInRecentSeconds(bleTxFilledSeconds);
  uint32_t partialMs = 0;
  if (bleTxBucketsInitialized) {
    if (now >= bleTxCurrentSecondStartMs) {
      partialMs = now - bleTxCurrentSecondStartMs;
    }
  }

  float denom = avgSeconds;
  if (partialMs > 0) {
    denom += static_cast<float>(partialMs) / 1000.0f;
  } else if (avgSeconds == 0 && bleTxCurrentSecondCount > 0) {
    denom = 1.0f;
  }

  float totalFrames = static_cast<float>(sumCompleted + bleTxCurrentSecondCount);
  if (denom > 0.0f) bleTxMovingAverageFps = totalFrames / denom;
  else               bleTxMovingAverageFps = 0.0f;

  bool overLimit = false;
  bool belowLimit = false;
  if (bleTxFilledSeconds >= BLE_TX_OVER_LIMIT_SECONDS) {
    float avg = static_cast<float>(bleTxFramesInRecentSeconds(BLE_TX_OVER_LIMIT_SECONDS)) /
                static_cast<float>(BLE_TX_OVER_LIMIT_SECONDS);
    overLimit = (avg > BLE_TX_FPS_LIMIT);
  }
  if (bleTxFilledSeconds >= BLE_TX_RELAX_SECONDS) {
    float avg = static_cast<float>(bleTxFramesInRecentSeconds(BLE_TX_RELAX_SECONDS)) /
                static_cast<float>(BLE_TX_RELAX_SECONDS);
    belowLimit = (avg < BLE_TX_FPS_LIMIT);
  }

  if (overLimit) {
    if (bleTxOverLimitSinceMs == 0) bleTxOverLimitSinceMs = now;
    bleTxBelowLimitSinceMs = 0;
    if ((now - bleTxOverLimitSinceMs) >= BLE_TX_OVER_LIMIT_DURATION_MS &&
        (now - bleTxLastBoostMs) >= BLE_TX_GOVERNOR_STEP_INTERVAL_MS) {
      if (governorIncreaseDividers()) {
        bleTxLastBoostMs = now;
      }
    }
  } else {
    bleTxOverLimitSinceMs = 0;
    if (belowLimit) {
      if (bleTxBelowLimitSinceMs == 0) bleTxBelowLimitSinceMs = now;
      if ((now - bleTxBelowLimitSinceMs) >= BLE_TX_RELAX_DURATION_MS &&
          (now - bleTxLastRelaxMs) >= BLE_TX_GOVERNOR_STEP_INTERVAL_MS) {
        if (governorRelaxDividers()) {
          bleTxLastRelaxMs = now;
        }
      }
    } else {
      bleTxBelowLimitSinceMs = 0;
    }
  }
}

// ===== BLE helpers we provide (instead of RaceChronoBle.*) =====
static inline bool bleIsConnected(){
  return g_server && g_server->getConnectedCount() > 0;
}

static bool bleWaitForConnection(uint32_t timeoutMs){
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs){
    if (bleIsConnected()) return true;
    delay(20);
  }
  return bleIsConnected();
}

static void bleStartAdvertising(){
  if (!g_adv) return;
  g_adv->start();
}

static void bleStopAdvertising(){
  if (!g_adv) return;
  g_adv->stop();
}

// ===== FIL (0x0002) handler to mirror RaceChrono DIY control =====

static void handleFilWrite(const uint8_t *d, size_t L) {
  if (!d || L < 1) return;
  uint8_t cmd = d[0];
  switch (cmd) {
    case 0: // Deny all
      Serial.println("FIL: DENY ALL");
      pidMap.reset();
      break;
    case 1: // Allow all + interval
      if (L >= 3) {
        uint16_t interval = ((uint16_t)d[1] << 8) | d[2];
        Serial.printf("FIL: ALLOW ALL (interval cmd=%ums ignored)\n", interval);
      } else {
        Serial.println("FIL: ALLOW ALL");
      }
      break;
    case 2: // Allow one PID (minimal accept)
      Serial.println("FIL: ALLOW PID (accepted)");
      break;
    default: break;
  }
}

class FilCB : public NimBLECharacteristicCallbacks {
public:
  void onWrite(NimBLECharacteristic* ch){
    std::string v = ch->getValue();
    handleFilWrite((const uint8_t*)v.data(), v.size());
  }
  void onWrite(NimBLECharacteristic* ch, ble_gap_conn_desc*){
    std::string v = ch->getValue();
    handleFilWrite((const uint8_t*)v.data(), v.size());
  }
#if defined(NIMBLE_CPP_IDF) || defined(ARDUINO_ARCH_ESP32)
  void onWrite(NimBLECharacteristic* ch, NimBLEConnInfo&){
    std::string v = ch->getValue();
    handleFilWrite((const uint8_t*)v.data(), v.size());
  }
#endif
} g_filCB;

// ===== Our CAN sender over BLE 0x0001 =====
static uint8_t canBuf[20];
static void bleSendCanData(uint32_t pid, const uint8_t *data, uint8_t len) {
  if (!g_can || !bleIsConnected()) return;
  // Pack as RaceChrono DIY: 4 bytes LE ID + up to 16 data bytes
  canBuf[0] = (uint8_t)(pid & 0xFF);
  canBuf[1] = (uint8_t)((pid >> 8) & 0xFF);
  canBuf[2] = (uint8_t)((pid >> 16) & 0xFF);
  canBuf[3] = (uint8_t)((pid >> 24) & 0xFF);
  uint8_t n = len > 16 ? 16 : len;
  if (n) memcpy(&canBuf[4], data, n);
  g_can->setValue(canBuf, 4 + n);
  g_can->notify();
  noteBleTxFrame(millis());
}

// ===== GPS helpers =====
static inline int clampi(int value, int lo, int hi) {
  return value < lo ? lo : (value > hi ? hi : value);
}

static bool nmeaCoordToDegrees(const char *ddmm, const char *hemi, double &out) {
  if (!ddmm || !hemi || !*ddmm || !*hemi) return false;
  const char *dot = strchr(ddmm, '.');
  int len = dot ? static_cast<int>(dot - ddmm) : static_cast<int>(strlen(ddmm));
  if (len < 3) return false;
  int degLen = len - 2;
  if (degLen <= 0 || degLen >= 10) return false;
  char degBuf[12];
  memset(degBuf, 0, sizeof(degBuf));
  strncpy(degBuf, ddmm, degLen);
  int degrees = atoi(degBuf);
  double minutes = atof(ddmm + degLen);
  double val = degrees + (minutes / 60.0);
  if (*hemi == 'S' || *hemi == 'W') val = -val;
  out = val;
  return true;
}

static int splitCsv(char *s, const char *fields[], int maxFields) {
  int count = 0;
  if (!s || maxFields <= 0) return 0;
  fields[count++] = s;
  for (char *p = s; *p && count < maxFields; ++p) {
    if (*p == ',') {
      *p = 0;
      fields[count++] = p + 1;
    } else if (*p == '*') {
      *p = 0;
      break;
    }
  }
  return count;
}

static bool nmeaChecksumOk(const char *line) {
  if (!line || line[0] != '$') return false;
  const char *star = strrchr(line, '*');
  if (!star || (star - line) < 3) return false;
  uint8_t checksum = 0;
  for (const char *p = line + 1; p < star; ++p) checksum ^= static_cast<uint8_t>(*p);
  int hi = toupper(static_cast<unsigned char>(star[1]));
  int lo = toupper(static_cast<unsigned char>(star[2]));
  if (!isxdigit(hi) || !isxdigit(lo)) return false;
  int value = ((hi >= 'A') ? (hi - 'A' + 10) : (hi - '0')) * 16 +
              ((lo >= 'A') ? (lo - 'A' + 10) : (lo - '0'));
  return checksum == static_cast<uint8_t>(value);
}

static void parseRmcSentence(char *line) {
  if (!nmeaChecksumOk(line)) return;
  const char *fields[20];
  int nf = splitCsv(line, fields, 20);
  if (nf < 10) return;

  const char *time = fields[1];
  if (time && strlen(time) >= 6) {
    char hh[3] = { time[0], time[1], 0 };
    char mm[3] = { time[2], time[3], 0 };
    char ss[3] = { time[4], time[5], 0 };
    rmc_hour = atoi(hh);
    rmc_min  = atoi(mm);
    rmc_sec  = atoi(ss);
    const char *dot = strchr(time, '.');
    rmc_millis = dot ? atoi(dot + 1) : 0;
    if (rmc_millis > 999) rmc_millis = 999;
  }

  rmc_valid = (fields[2] && *fields[2] == 'A');

  double lat, lon;
  if (nmeaCoordToDegrees(fields[3], fields[4], lat)) rmc_lat_deg = lat;
  if (nmeaCoordToDegrees(fields[5], fields[6], lon)) rmc_lon_deg = lon;

  double speedKnots = (fields[7] && *fields[7]) ? atof(fields[7]) : 0.0;
  rmc_speed_kmh = speedKnots * 1.852;
  rmc_course_deg = (fields[8] && *fields[8]) ? atof(fields[8]) : 0.0;

  const char *date = fields[9];
  if (date && strlen(date) >= 6) {
    char dd[3] = { date[0], date[1], 0 };
    char mm[3] = { date[2], date[3], 0 };
    char yy[3] = { date[4], date[5], 0 };
    gps_day = atoi(dd);
    gps_mon = atoi(mm);
    gps_year = 2000 + atoi(yy);
  }
}

static void parseGgaSentence(char *line) {
  if (!nmeaChecksumOk(line)) return;
  const char *fields[20];
  int nf = splitCsv(line, fields, 20);
  if (nf < 11) return;

  gga_sats = (fields[7] && *fields[7]) ? atoi(fields[7]) : 0;
  gga_hdop = (fields[8] && *fields[8]) ? atof(fields[8]) : 99.9;
  gga_alt_m = (fields[9] && *fields[9]) ? atof(fields[9]) : 0.0;
}

// ===== BLE bring-up (we own the whole 0x1FF8 service) =====
static void bleInit(){
  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  g_server = NimBLEDevice::createServer();

  g_svc = g_server->createService(RC_SERVICE_UUID);

  g_can = g_svc->createCharacteristic(RC_CHAR_CAN_UUID,
            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  g_fil = g_svc->createCharacteristic(RC_CHAR_FIL_UUID,
            NIMBLE_PROPERTY::WRITE);
  g_gps = g_svc->createCharacteristic(RC_CHAR_GPS_UUID,
            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  g_gtm = g_svc->createCharacteristic(RC_CHAR_GTM_UUID,
            NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  g_fil->setCallbacks(&g_filCB);

  // Init payloads
  { uint8_t init20[20]; memset(init20, 0xFF, sizeof(init20)); g_gps->setValue(init20, 20); }
  { uint8_t init3[3] = {0,0,0}; g_gtm->setValue(init3, 3); }

  g_svc->start();

  g_adv = NimBLEDevice::getAdvertising();
  g_adv->addServiceUUID(RC_SERVICE_UUID);
  g_adv->start();

  Serial.println("BLE: advertising (RaceChrono DIY 0x1FF8)");
}

// Pack & notify GPS frames on our g_gps/g_gtm
static void gpsPackAndNotify(uint32_t now) {
  if (!bleIsConnected() || !g_gps || !g_gtm) return;

  uint8_t payload[20];
  memset(payload, 0xFF, sizeof(payload));

  int year = gps_year >= 2000 ? gps_year : 2000;
  int month = gps_mon >= 1 ? gps_mon : 1;
  int day = gps_day >= 1 ? gps_day : 1;
  int dateHour = ((year - 2000) * 8928) + ((month - 1) * 744) + ((day - 1) * 24) + rmc_hour;
  if (dateHour != lastDateHourPacked) {
    lastDateHourPacked = dateHour;
    gpsSyncBits = (gpsSyncBits + 1) & 0x7;
  }

  int timeSinceHour = (rmc_min * 30000) + (rmc_sec * 500) + (rmc_millis / 2);
  if (timeSinceHour < 0) timeSinceHour = 0;

  uint8_t fixQuality = rmc_valid ? 1 : 0;
  uint8_t sats = static_cast<uint8_t>(clampi(gga_sats, 0, 63));

  int32_t latFixed = static_cast<int32_t>(lround(rmc_lat_deg * 10000000.0));
  int32_t lonFixed = static_cast<int32_t>(lround(rmc_lon_deg * 10000000.0));

  int altWord = 0xFFFF;
  if (gga_alt_m > -7000.0 && gga_alt_m < 20000.0) {
    double alt = gga_alt_m;
    if (alt >= -500.0 && alt <= 6053.5) {
      altWord = clampi(static_cast<int>(lround((alt + 500.0) * 10.0)) & 0x7FFF, 0, 0x7FFF);
    } else {
      altWord = (static_cast<int>(lround(alt + 500.0)) & 0x7FFF) | 0x8000;
    }
  }

  int speedWord = 0xFFFF;
  double speed = rmc_speed_kmh;
  if (speed <= 655.35) {
    speedWord = clampi(static_cast<int>(lround(speed * 100.0)) & 0x7FFF, 0, 0x7FFF);
  } else {
    speedWord = (static_cast<int>(lround(speed * 10.0)) & 0x7FFF) | 0x8000;
  }

  int bearing = clampi(static_cast<int>(lround(rmc_course_deg * 100.0)), 0, 0xFFFF);
  uint8_t hdop = (gga_hdop >= 0.0 && gga_hdop <= 25.4) ? static_cast<uint8_t>(lround(gga_hdop * 10.0)) : 0xFF;
  uint8_t vdop = 0xFF;

  payload[0]  = static_cast<uint8_t>(((gpsSyncBits & 0x7) << 5) | ((timeSinceHour >> 16) & 0x1F));
  payload[1]  = static_cast<uint8_t>(timeSinceHour >> 8);
  payload[2]  = static_cast<uint8_t>(timeSinceHour);
  payload[3]  = static_cast<uint8_t>(((fixQuality & 0x3) << 6) | (sats & 0x3F));
  payload[4]  = static_cast<uint8_t>(latFixed >> 24);
  payload[5]  = static_cast<uint8_t>(latFixed >> 16);
  payload[6]  = static_cast<uint8_t>(latFixed >> 8);
  payload[7]  = static_cast<uint8_t>(latFixed);
  payload[8]  = static_cast<uint8_t>(lonFixed >> 24);
  payload[9]  = static_cast<uint8_t>(lonFixed >> 16);
  payload[10] = static_cast<uint8_t>(lonFixed >> 8);
  payload[11] = static_cast<uint8_t>(lonFixed);
  payload[12] = static_cast<uint8_t>(altWord >> 8);
  payload[13] = static_cast<uint8_t>(altWord);
  payload[14] = static_cast<uint8_t>(speedWord >> 8);
  payload[15] = static_cast<uint8_t>(speedWord);
  payload[16] = static_cast<uint8_t>(bearing >> 8);
  payload[17] = static_cast<uint8_t>(bearing);
  payload[18] = hdop;
  payload[19] = vdop;

  g_gps->setValue(payload, sizeof(payload));
  g_gps->notify();
  noteBleTxFrame(now);

  uint8_t timePayload[3];
  timePayload[0] = static_cast<uint8_t>(((gpsSyncBits & 0x7) << 5) | ((dateHour >> 16) & 0x1F));
  timePayload[1] = static_cast<uint8_t>(dateHour >> 8);
  timePayload[2] = static_cast<uint8_t>(dateHour);
  g_gtm->setValue(timePayload, sizeof(timePayload));
  g_gtm->notify();
  noteBleTxFrame(now);
}

static void gpsSendRaw(const char *cmd) {
  GPSSerial.print(cmd);
}

static bool gpsLooksLikeNmea(uint32_t timeoutMs) {
  uint32_t start = millis();
  char prev = 0;
  while (millis() - start < timeoutMs) {
    if (GPSSerial.available()) {
      char c = static_cast<char>(GPSSerial.read());
      if (prev == '$' && (c == 'G' || c == 'N')) return true;
      prev = c;
    }
  }
  return false;
}

static bool gpsConfigurePort() {
  const uint32_t bauds[] = { 9600, 38400, 57600, 115200 };
  bool detected = false;
  for (uint32_t baud : bauds) {
    GPSSerial.updateBaudRate(baud);
    delay(60);
    if (gpsLooksLikeNmea(500)) {
      Serial.printf("[GPS] Detected NMEA @ %lu\n", static_cast<unsigned long>(baud));
      detected = true;
      break;
    }
  }
  if (!detected) {
    Serial.println("[GPS] No NMEA detected");
    return false;
  }

  gpsSendRaw(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  delay(80);
  gpsSendRaw(PMTK_SET_NMEA_UPDATE_10HZ);
  delay(80);
  gpsSendRaw(PMTK_SET_BAUD_115200);
  delay(120);

  GPSSerial.end();
  delay(20);
  GPSSerial.begin(115200, SERIAL_8N1, GPS_RX_GPIO, GPS_TX_GPIO);
  delay(80);
  if (!gpsLooksLikeNmea(800)) {
    Serial.println("[GPS] Missing NMEA after baud switch");
    return false;
  }

  return true;
}

static void gpsResetSyncState() {
  gpsSyncBits = 0;
  lastDateHourPacked = -1;
  lastGpsNotifyMs = 0;
}

static void gpsInit() {
  Serial.printf("[GPS] UART init @ %u bps (RX=%d, TX=%d)\n", 9600u, GPS_RX_GPIO, GPS_TX_GPIO);
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX_GPIO, GPS_TX_GPIO);
  delay(100);

  if (gpsConfigurePort()) {
    gpsConfigured = true;
    gpsLastSentenceMs = millis();
    gpsResetSyncState();
    Serial.println("[GPS] Configured: RMC+GGA @10Hz, 115200");
  } else {
    GPSSerial.end();
    gpsConfigured = false;
    gpsResetSyncState();
  }
}

static void gpsProcessSentence(char *line) {
  if (!line || strlen(line) < 6) return;
  if (strstr(line, "GPRMC") || strstr(line, "GNRMC")) {
    parseRmcSentence(line);
  } else if (strstr(line, "GPGGA") || strstr(line, "GNGGA")) {
    parseGgaSentence(line);
  }
}

static void gpsService(uint32_t now) {
  if (!gpsConfigured) {
    if (now - lastGpsInitAttemptMs >= GPS_INIT_RETRY_MS) {
      lastGpsInitAttemptMs = now;
      gpsInit();
    }
    return;
  }

  bool sentenceSeen = false;
  while (GPSSerial.available()) {
    char c = static_cast<char>(GPSSerial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      gpsLineBuf[gpsLineLen] = 0;
      if (gpsLineLen > 0 && gpsLineBuf[0] == '$') {
        char work[sizeof(gpsLineBuf)];
        strncpy(work, gpsLineBuf, sizeof(work) - 1);
        work[sizeof(work) - 1] = 0;
        gpsProcessSentence(work);
        sentenceSeen = true;
      }
      gpsLineLen = 0;
    } else {
      if (gpsLineLen < sizeof(gpsLineBuf) - 1) {
        gpsLineBuf[gpsLineLen++] = c;
      } else {
        gpsLineLen = 0;
      }
    }
  }

  if (sentenceSeen) {
    gpsLastSentenceMs = now;
  } else if (now - gpsLastSentenceMs > 2000) {
    Serial.println("[GPS] Sentence timeout -> reinit");
    gpsConfigured = false;
    gpsResetSyncState();
    GPSSerial.end();
    lastGpsInitAttemptMs = now;
    return;
  }

  if (bleIsConnected() && (now - lastGpsNotifyMs) >= GPS_NOTIFY_PERIOD_MS) {
    lastGpsNotifyMs = now;
    gpsPackAndNotify(now);
  }
}

static void clear_custom_dividers() {
  customDividerCount = 0;
  memset(customDividers, 0, sizeof(customDividers));
}

static void cfg_apply_runtime_defaults() {
  V0_ADC = DEFAULT_V0_ADC;
  V1_ADC = DEFAULT_V1_ADC;
  USE_PROFILE_ALLOWLIST = DEFAULT_USE_PROFILE_ALLOWLIST;
  clear_custom_dividers();
}

bool cfg_load_from_nvs() {
  bool anyLoaded = false;
  if (!cfgPrefs.begin(CFG_NAMESPACE, true)) return false;

  customDividerCount = 0;

  if (cfgPrefs.isKey("v0_adc")) {
    float v0 = cfgPrefs.getFloat("v0_adc", V0_ADC);
    if (v0 >= 0.0f && v0 < 3.4f) {
      V0_ADC = v0;
      anyLoaded = true;
    }
  }
  if (cfgPrefs.isKey("v1_adc")) {
    float v1 = cfgPrefs.getFloat("v1_adc", V1_ADC);
    if (v1 > V0_ADC && v1 < 3.5f) {
      V1_ADC = v1;
      anyLoaded = true;
    }
  }
  if (cfgPrefs.isKey("profile")) {
    uint8_t prof = cfgPrefs.getUChar("profile", USE_PROFILE_ALLOWLIST ? 1 : 0);
    USE_PROFILE_ALLOWLIST = (prof != 0);
    anyLoaded = true;
  }

  size_t blobLen = cfgPrefs.getBytesLength("dividers");
  if (blobLen >= sizeof(uint16_t)) {
    CfgDividerBlob blob;
    memset(&blob, 0, sizeof(blob));
    size_t readLen = cfgPrefs.getBytes("dividers", &blob, sizeof(blob));
    if (readLen >= sizeof(uint16_t)) {
      uint16_t count = blob.count;
      if (count > CUSTOM_DIVIDER_MAX) count = CUSTOM_DIVIDER_MAX;
      for (uint16_t i = 0; i < count; ++i) {
        uint16_t pid = blob.items[i].pid;
        uint8_t div = blob.items[i].div == 0 ? 1 : blob.items[i].div;
        customDividers[customDividerCount].pid = pid;
        customDividers[customDividerCount].div = div;
        customDividerCount++;
      }
      anyLoaded = true;
    }
  }

  cfgPrefs.end();
  return anyLoaded;
}

bool cfg_save_to_nvs() {
  if (!cfgPrefs.begin(CFG_NAMESPACE, false)) return false;

  ScopedNvsWrite guard;

  bool ok = true;
  if (cfgPrefs.putFloat("v0_adc", V0_ADC) != sizeof(float)) ok = false;
  if (cfgPrefs.putFloat("v1_adc", V1_ADC) != sizeof(float)) ok = false;
  if (cfgPrefs.putUChar("profile", USE_PROFILE_ALLOWLIST ? 1 : 0) != sizeof(uint8_t)) ok = false;

  CfgDividerBlob blob;
  memset(&blob, 0, sizeof(blob));
  blob.count = customDividerCount;
  for (uint16_t i = 0; i < customDividerCount; ++i) {
    blob.items[i] = customDividers[i];
  }
  if (customDividerCount < CUSTOM_DIVIDER_MAX) {
    memset(&blob.items[customDividerCount], 0,
           sizeof(CfgPidDivider) * (CUSTOM_DIVIDER_MAX - customDividerCount));
  }
  size_t written = cfgPrefs.putBytes("dividers", &blob, sizeof(blob));
  if (written != sizeof(blob)) ok = false;

  cfgPrefs.end();
  return ok;
}

bool cfg_reset_to_defaults() {
  cfg_apply_runtime_defaults();

  if (!cfgPrefs.begin(CFG_NAMESPACE, false)) return false;
  ScopedNvsWrite guard;
  cfgPrefs.clear();
  cfgPrefs.end();
  return true;
}

static void apply_profile_and_dividers();

void cfg_boot_load_and_apply() {
  bool loaded = cfg_load_from_nvs();
  if (!loaded) {
    cfg_apply_runtime_defaults();
  }

  apply_profile_and_dividers();
}

// ===== Forward decls =====
static void waitForConnection();
static bool startCanBusReader();
static void stopCanBusReader();
static bool restartCanBusReader(const char *cause = nullptr);
static void handleCanFault(const char *cause);
static bool checkCanBusHealth(uint32_t now);
[[maybe_unused]] static void noteCanWriteResult(bool success);
[[maybe_unused]] static bool writeFrameWithWatch(CanFrame &frame, uint32_t timeout = 1);
static void setLastReconnectCause(const char *reason);

static void bufferNewPacket(uint32_t pid, const uint8_t *data, uint8_t len);
static void handleBufferedPacketsBurst(uint32_t budget_us = 2000);
static void flushBufferedPackets();
static void sendNumCanBusTimeouts();
static void resetSkippedUpdatesCounters();
static void bleSendCanData(uint32_t pid, const uint8_t *data, uint8_t len);

static void apply_allow_list_profile();
static void apply_allow_all();
static void apply_profile_and_dividers();

bool cfg_load_from_nvs();
bool cfg_save_to_nvs();
bool cfg_reset_to_defaults();
bool piddiv_set(uint16_t pid, uint8_t div);
bool piddiv_clear(uint16_t pid);
void piddiv_apply_all();
void cfg_boot_load_and_apply();

static float  filtered_adc_volts();
static uint8_t oil_health_flags_from_volts(float v);
static float  volts_to_psi_exact(float v);
static void   oil_update_and_publish_if_due();

static void   restartBle(const char *reason = nullptr);
static void   process_cli_line(char *line);
static void   show_config();      // concise SHOW
static void   show_stats();       // SHOW STATS
static void   dumpMapToSerial();  // verbose only on SHOW MAP
static void   dumpDenyToSerial(); // verbose only on SHOW DENY

// ===== Clean BLE restart (no deinit; advertiser stop/start only) =====
static void restartBle(const char *reason) {
  if (reason && reason[0]) {
    setLastReconnectCause(reason);
  }
  Serial.println("BLE: restart...");
  bleStopAdvertising();
  // Service + characteristics remain; we just restart advertising.
  bleStartAdvertising();
  Serial.println("BLE: advertising");
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis(); while (!Serial && millis() - t0 < 3000) {}

  Serial.printf("FW: %s\n", FW_VERSION_STRING);
  esp_reset_reason_t reason = esp_reset_reason();
  Serial.printf("Boot reason: %s (%d)\n", reset_reason_to_string(reason), (int)reason);

  init_task_watchdog();

  // ADC
  analogReadResolution(12);
  analogSetPinAttenuation(OIL_ADC_PIN, ADC_11db);
  pinMode(OIL_ADC_PIN, INPUT);

  // BLE
  Serial.println("BLE setup...");
  bleInit();
  Serial.println("Waiting for RaceChrono...");
  waitForConnection();

  setLastReconnectCause("boot");

  // Load persisted config + apply profile/dividers
  cfg_boot_load_and_apply();

  show_config(); // one-time concise boot print

  // GPS
  gpsInit();
  lastGpsInitAttemptMs = millis();

  // CAN bring-up deferred to loop() retry logic
}

static void waitForConnection() {
  uint32_t i=0; bool nl=true;
  while (!bleWaitForConnection(1000)) {
    esp_task_wdt_reset();
    Serial.print("."); nl=false; if ((++i % 10)==0) { Serial.println(); nl=true; }
  }
  if (!nl) Serial.println();
  Serial.println("RaceChrono connected.");
}

static void recoverRaceChronoConnection(const char *reason) {
  Serial.println("RC disconnected -> reset map + CAN + BLE.");
  pidMap.reset();
  stopCanBusReader();

  restartBle(reason);
  Serial.println("Waiting for new RC connection...");
  waitForConnection();

  gpsResetSyncState();

  apply_profile_and_dividers();

  sendNumCanBusTimeouts();
}

static void queueBleRestart(const char *reason) {
  if (pendingBleRestart) return;
  const char *src = (reason && reason[0]) ? reason : "RC disconnect";
  strncpy(pendingBleRestartReason, src, sizeof(pendingBleRestartReason) - 1);
  pendingBleRestartReason[sizeof(pendingBleRestartReason) - 1] = '\0';
  pendingBleRestart = true;
}

static void servicePendingBleRestart() {
  if (!pendingBleRestart) return;
  if (nvsWriteInProgress) return;

  pendingBleRestart = false;
  recoverRaceChronoConnection(pendingBleRestartReason[0] ? pendingBleRestartReason : nullptr);
  pendingBleRestartReason[0] = '\0';
}

// ===== CAN bring-up/teardown =====
static bool startCanBusReader() {
  Serial.println("TWAI start...");
  ESP32Can.setPins(CAN_TX, CAN_RX);
  ESP32Can.setSpeed(ESP32Can.convertSpeed(500));
  ESP32Can.setRxQueueSize(64);
  ESP32Can.setTxQueueSize(16);
  if (!ESP32Can.begin()) {
    Serial.println("ERROR: TWAI begin() failed. Check wiring/termination/RS.");
    return false;
  }
  Serial.println("CAN OK.");
  isCanBusReaderActive = true;
  have_seen_any_can = false;
  lastCanMessageReceivedMs = millis();
  return true;
}

static void stopCanBusReader() {
  ESP32Can.end();
  isCanBusReaderActive = false;
  lastCanStatusCheckMs = 0;
  canTxFailureWindowStart = 0;
  canTxFailuresInWindow = 0;
}

static void setLastReconnectCause(const char *reason) {
  const char *src = (reason && reason[0]) ? reason : "unknown";
  strncpy(lastReconnectCause, src, sizeof(lastReconnectCause) - 1);
  lastReconnectCause[sizeof(lastReconnectCause) - 1] = '\0';
}

static bool restartCanBusReader(const char *cause) {
  if (cause && cause[0]) {
    Serial.printf("CAN: restart (%s)\n", cause);
    setLastReconnectCause(cause);
  } else {
    Serial.println("CAN: restart");
    setLastReconnectCause("CAN restart");
  }

  stopCanBusReader();
  delay(50);

  bool started = startCanBusReader();
  if (started) {
    flushBufferedPackets();
    resetSkippedUpdatesCounters();
    lastCanMessageReceivedMs = millis();
  }

  return started;
}

static void handleCanFault(const char *cause) {
  char reason[64];
  if (cause && cause[0]) {
    strncpy(reason, cause, sizeof(reason) - 1);
    reason[sizeof(reason) - 1] = '\0';
  } else {
    strncpy(reason, "CAN fault", sizeof(reason) - 1);
    reason[sizeof(reason) - 1] = '\0';
  }

  Serial.printf("ERROR: %s -> restart CAN\n", reason);
  num_can_bus_timeouts++;
  sendNumCanBusTimeouts();
  restartCanBusReader(reason);
}

static bool checkCanBusHealth(uint32_t now) {
  if (!isCanBusReaderActive) return false;
  if ((now - lastCanStatusCheckMs) < CAN_STATUS_POLL_INTERVAL_MS) return false;

  lastCanStatusCheckMs = now;

  twai_status_info_t status;
  if (twai_get_status_info(&status) != ESP_OK) {
    return false;
  }

  if (status.state == TWAI_STATE_BUS_OFF) {
    char reason[64];
    snprintf(reason, sizeof(reason),
             "TWAI bus-off (tx_err=%u rx_err=%u)",
             (unsigned)status.tx_error_counter,
             (unsigned)status.rx_error_counter);
    handleCanFault(reason);
    return true;
  }

  if (status.state == TWAI_STATE_STOPPED) {
    handleCanFault("TWAI stopped");
    return true;
  }

  return false;
}

[[maybe_unused]] static void noteCanWriteResult(bool success) {
  if (success) {
    canTxFailuresInWindow = 0;
    canTxFailureWindowStart = 0;
    return;
  }

  uint32_t now = millis();
  if ((canTxFailureWindowStart == 0) ||
      (now - canTxFailureWindowStart > CAN_TX_FAILURE_WINDOW_MS)) {
    canTxFailureWindowStart = now;
    canTxFailuresInWindow = 0;
  }

  canTxFailuresInWindow++;
  if (canTxFailuresInWindow >= CAN_TX_FAILURE_THRESHOLD) {
    char reason[64];
    snprintf(reason, sizeof(reason),
             "CAN TX failure window (%u/%ums)",
             (unsigned)canTxFailuresInWindow,
             (unsigned)CAN_TX_FAILURE_WINDOW_MS);
    handleCanFault(reason);
    canTxFailureWindowStart = 0;
    canTxFailuresInWindow = 0;
  }
}

[[maybe_unused]] static bool writeFrameWithWatch(CanFrame &frame, uint32_t timeout) {
  bool ok = ESP32Can.writeFrame(frame, timeout);
  noteCanWriteResult(ok);
  return ok;
}

// ===== Helpers =====
static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

static uint8_t oil_health_flags_from_volts(float v) {
  const float OPEN_THR  = V1_ADC + 0.10f;
  const float SHORT_THR = 0.05f;
  uint8_t f = 0;
  if (v > OPEN_THR)  f |= (1 << 0);
  if (v < SHORT_THR) f |= (1 << 1);
  if (v < V0_ADC - 0.08f || v > V1_ADC + 0.08f) f |= (1 << 2);
  return f;
}

// Median-of-5 then IIR
static float filtered_adc_volts() {
  float s[5];
  for (int i = 0; i < 5; i++) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    s[i] = analogReadMilliVolts(OIL_ADC_PIN) / 1000.0f;
#else
    uint32_t acc = 0;
    for (int k = 0; k < ADC_SAMPLES; k++) acc += analogRead(OIL_ADC_PIN);
    const float ADC_VREF = 3.30f, ADC_MAX = 4095.0f;
    s[i] = ((acc / (float)ADC_SAMPLES) * ADC_VREF) / ADC_MAX;
#endif
  }
  for (int i=1;i<5;i++){ float t=s[i]; int j=i-1; while(j>=0&&s[j]>t){s[j+1]=s[j]; j--;} s[j+1]=t; }
  float med = s[2];
  static float v_filt = 0.0f;
  if (v_filt == 0.0f) v_filt = med;
  v_filt = (1.0f - ADC_IIR_ALPHA) * v_filt + ADC_IIR_ALPHA * med;
  return v_filt;
}

// Two-point exact mapping with gentle end snapping
static float volts_to_psi_exact(float v) {
  const float FLOOR_V_MARGIN = 0.020f;
  const float CEIL_V_MARGIN  = 0.010f;
  if (v <= V0_ADC + FLOOR_V_MARGIN) return P0_PSI;
  if (v >= V1_ADC - CEIL_V_MARGIN)  return P1_PSI;
  const float m = (P1_PSI - P0_PSI) / (V1_ADC - V0_ADC);
  float psi = P0_PSI + m * (v - V0_ADC);
  psi = clampf(psi, P0_PSI, P1_PSI);
  if (psi < (P0_PSI + ADC_DEADBAND_PSI)) psi = P0_PSI;
  if (psi > (P1_PSI - ADC_DEADBAND_PSI)) psi = P1_PSI;
  return psi;
}

// ===== Circular buffer =====
struct BufferedMessage {
  uint32_t pid;
  uint8_t  data[8];
  uint8_t  length;
};
static const uint8_t NUM_BUFFERS = 32;
static BufferedMessage buffers[NUM_BUFFERS];
static uint8_t bufferToWriteTo = 0;
static uint8_t bufferToReadFrom = 0;

static void bufferNewPacket(uint32_t pid, const uint8_t *data, uint8_t len) {
  if (bufferToWriteTo - bufferToReadFrom == NUM_BUFFERS) {
    Serial.println("WARNING: RX buffer overflow, dropping oldest.");
    bufferToReadFrom++;
  }
  BufferedMessage *m = &buffers[bufferToWriteTo % NUM_BUFFERS];
  m->pid = pid;
  m->length = (len > 8) ? 8 : len;
  memcpy(m->data, data, m->length);
  bufferToWriteTo++;
}

static void handleBufferedPacketsBurst(uint32_t budget_us) {
  uint32_t t0 = micros();
  while (bufferToReadFrom != bufferToWriteTo && (micros() - t0) < budget_us) {
    BufferedMessage *m = &buffers[bufferToReadFrom % NUM_BUFFERS];

    bool allowed = false;
    void *entry = pidMap.getEntryId(m->pid);
    if (USE_PROFILE_ALLOWLIST) {
      allowed = (entry != nullptr) && !isDenied(m->pid);
    } else {
      allowed = !isDenied(m->pid);
    }

    if (allowed) {
      if (entry) {
        PidExtra *extra = pidMap.getExtra(entry);
        uint8_t div = effective_divider(extra);
        if (extra->skippedUpdates == 0) {
          bleSendCanData(m->pid, m->data, m->length);
        }
        extra->skippedUpdates++;
        if (extra->skippedUpdates >= div) {
          extra->skippedUpdates = 0;
        }
      } else {
        bleSendCanData(m->pid, m->data, m->length);
      }
    }

    bufferToReadFrom++;
  }
}

static void flushBufferedPackets() {
  bufferToWriteTo = 0;
  bufferToReadFrom = 0;
}

static void sendNumCanBusTimeouts() {
  uint8_t d[2] = {
    (uint8_t)(num_can_bus_timeouts & 0xFF),
    (uint8_t)(num_can_bus_timeouts >> 8)
  };
  bleSendCanData(0x777, d, 2);
  last_time_num_can_bus_timeouts_sent_ms = millis();
}

static void resetSkippedUpdatesCounters() {
  struct { void operator()(void *entry) {
    ((SimplePidMap<PidExtra>::Entry*)entry)->extra.skippedUpdates = 0;
  }} fun;
  pidMap.forEach(fun);
}

// ===== Profile application =====
static void apply_allow_list_profile() {
  GovernorSnapshot snapshots[BLE_GOVERNOR_SNAPSHOT_MAX];
  size_t snapshotCount = capture_governor_snapshot(snapshots);

  pidMap.reset();
  clearDeny(); // optional fresh start
  bleTxOverLimitSinceMs = 0;
  bleTxBelowLimitSinceMs = 0;
  for (size_t i = 0; i < CAR_PROFILE_LEN; ++i) {
    const auto &r = CAR_PROFILE[i];
    pidMap.allowOnePid(r.pid, /*ms*/40);
    void *entry = pidMap.getEntryId(r.pid);
    if (entry) {
      // (Fix) correctly take the address of the entry's Extra
      PidExtra *ee = &((SimplePidMap<PidExtra>::Entry*)entry)->extra;
      uint8_t base = (r.divider == 0 ? 1 : r.divider);
      set_extra_base_divider(ee, base, /*resetGovernor=*/false);
    }
  }
  bool restored = restore_governor_snapshot(snapshots, snapshotCount);
  bleGovernorActive = restored;
}

static void apply_allow_all() {
  GovernorSnapshot snapshots[BLE_GOVERNOR_SNAPSHOT_MAX];
  size_t snapshotCount = capture_governor_snapshot(snapshots);

  pidMap.reset();
  clearDeny();
  bleTxOverLimitSinceMs = 0;
  bleTxBelowLimitSinceMs = 0;
  // No implicit population here; base dividers will apply lazily as PIDs appear
  pidMap.forEach([&](void *entry) {
    uint32_t pid = ((SimplePidMap<PidExtra>::Entry*)entry)->pid;
    uint8_t base = compute_default_divider_for_pid(pid);
    set_extra_base_divider(&((SimplePidMap<PidExtra>::Entry*)entry)->extra, base, /*resetGovernor=*/false);
  });
  bool restored = restore_governor_snapshot(snapshots, snapshotCount);
  bleGovernorActive = restored;
}

static void apply_profile_and_dividers() {
  if (USE_PROFILE_ALLOWLIST) {
    apply_allow_list_profile();
  } else {
    apply_allow_all();
  }
  piddiv_apply_all();
}

// ===== Oil publish =====
static void oil_update_and_publish_if_due() {
  const uint32_t now = millis();
  float v = filtered_adc_volts();
  oil_flags = oil_health_flags_from_volts(v);
  oil_psi_f = volts_to_psi_exact(v);

  if (bleIsConnected() && (now - lastOilTxMs) >= OIL_TX_RATE_MS) {
    lastOilTxMs = now;
    uint16_t psi01 = (uint16_t)(oil_psi_f * OIL_SCALE_01PSI + 0.5f);
    uint8_t d[8] = {
      (uint8_t)(psi01 >> 8), (uint8_t)(psi01 & 0xFF),
      oil_flags, 0, 0, 0, 0, 0
    };
    bleSendCanData(OIL_CAN_ID, d, 8);
  }
}

// ===== Concise config print (SHOW) =====
static void show_config() {
  Serial.println("=== CCA Config ===");
  Serial.printf("Profile:        %s\n", USE_PROFILE_ALLOWLIST ? "ALLOW-LIST" : "SNIFF-ALL");
  Serial.printf("V0_ADC:         %.4f V\n", V0_ADC);
  Serial.printf("V1_ADC:         %.4f V\n", V1_ADC);
  Serial.printf("Custom divs:    %u\n", (unsigned)customDividerCount);
  Serial.println("==================");
}

// ===== Verbose dumps (on demand only) =====
static void show_stats() {
  Serial.println("=== CCA Stats ===");
  Serial.printf("RX count:        %lu\n", (unsigned long)rxCount);
  Serial.printf("CAN timeouts:    %u\n", (unsigned)num_can_bus_timeouts);
  Serial.printf("BLE tx avg:      %.1f fps\n", bleTxMovingAverageFps);
  Serial.printf("BLE governor:    %s\n", bleGovernorActive ? "ACTIVE" : "idle");
  Serial.printf("Last reconnect:  %s\n", lastReconnectCause);
  Serial.printf("FW: %s\n", FW_VERSION_STRING);
  Serial.println("=================");
}

static void dumpMapToSerial() {
  Serial.println("PID map:");
  if (pidMap.isEmpty()) { Serial.println("  <empty>\n"); return; }
  size_t total = 0;
  size_t printed = 0;
  pidMap.forEach([&](void *entry) {
    total++;
    if (printed >= 128) return;
    uint32_t pid = ((SimplePidMap<PidExtra>::Entry*)entry)->pid;
    const PidExtra *e = &((SimplePidMap<PidExtra>::Entry*)entry)->extra;
    Serial.printf("  %03lX: base=%u eff=%u\n",
                  (unsigned long)pid,
                  (unsigned)e->baseDivider,
                  (unsigned)effective_divider(e));
    printed++;
  });
  if (total > 128) {
    Serial.printf("  ... +%u more\n", (unsigned)(total - 128));
  }
  Serial.println();
}
static void dumpDenyToSerial() {
  Serial.println("Deny list:");
  if (denyCount == 0) { Serial.println("  <empty>\n"); return; }
  for (size_t i = 0; i < denyCount; ++i) {
    Serial.printf("  0x%03lX (%lu)\n", (unsigned long)denyList[i], (unsigned long)denyList[i]);
  }
  Serial.println();
}

// ===== Serial CLI =====
static constexpr size_t CLI_BUF_SIZE = 192;
static void process_cli_line(char *line) {
  while (*line == ' ' || *line == '\t') ++line;
  size_t L = strlen(line);
  while (L && (line[L-1] == ' ' || line[L-1] == '\t')) line[--L] = 0;
  if (!L) return;

  String s(line);
  String up = s; up.toUpperCase();

  if (up == "SHOW") {
    Serial.println("SHOW subcommands:");
    Serial.println("  CFG   -> config summary");
    Serial.println("  STATS -> runtime counters");
    Serial.println("  MAP   -> enabled PID allow-list");
    return;
  }
  if (up == "SHOW CFG")   { show_config(); return; }
  if (up == "SHOW STATS") { show_stats(); return; }
  if (up == "SHOW MAP")        { dumpMapToSerial(); return; }
  if (up == "SHOW DENY")       { dumpDenyToSerial(); return; }

  if (up == "CAL 0")           { V0_ADC = filtered_adc_volts(); Serial.printf("V0_ADC=%.4f (not saved)\n", V0_ADC); return; }
  if (up == "CAL 1")           { V1_ADC = filtered_adc_volts(); Serial.printf("V1_ADC=%.4f (not saved)\n", V1_ADC); return; }

  if (up == "PROFILE ON") {
    USE_PROFILE_ALLOWLIST = true;
    apply_profile_and_dividers();
    Serial.println("Profile=ALLOW-LIST (not saved)");
    return;
  }
  if (up == "PROFILE OFF") {
    USE_PROFILE_ALLOWLIST = false;
    apply_profile_and_dividers();
    Serial.println("Profile=SNIFF-ALL (not saved)");
    return;
  }

  if (up.startsWith("RATE ")) {
    char buf[CLI_BUF_SIZE]; strncpy(buf, line, sizeof(buf)); buf[sizeof(buf)-1]=0;
    char *tok = strtok(buf + 5, " \t");
    if (tok) {
      unsigned ms = strtoul(tok, nullptr, 0);
      if (ms < 10 || ms > 2000) { Serial.println("RATE out of range (10..2000 ms)"); }
      else { OIL_TX_RATE_MS = (uint16_t)ms; Serial.printf("RATE=%u ms (not saved)\n", ms); }
    } else Serial.println("Usage: RATE <ms>");
    return;
  }

  if (up.startsWith("ALLOW ")) {
    char buf[CLI_BUF_SIZE]; strncpy(buf, line, sizeof(buf)); buf[sizeof(buf)-1]=0;
    char *tok = strtok(buf + 6, " \t");
    char *tok2 = tok ? strtok(nullptr, " \t") : nullptr;
    if (tok && tok2) {
      uint32_t pid = strtoul(tok, nullptr, 0);
      unsigned n   = strtoul(tok2, nullptr, 0);
      if (n == 0) n = 1; if (n > 255) n = 255;
      removeDeny(pid);
      if (!pidMap.allowOnePid(pid, 40)) {
        Serial.println("ALLOW failed (map full?)");
      } else {
        if (!piddiv_set(pid, (uint8_t)n)) {
          Serial.println("WARNING: Custom divider table full; divider not persisted.");
        }
        Serial.printf("ALLOW 0x%03lX div=%u\n", (unsigned long)pid, (unsigned)n);
      }
    } else {
      Serial.println("Usage: ALLOW <pid> <div>");
    }
    return;
  }

  if (up.startsWith("DENY ")) {
    char buf[CLI_BUF_SIZE]; strncpy(buf, line, sizeof(buf)); buf[sizeof(buf)-1]=0;
    char *tok = strtok(buf + 5, " \t");
    if (tok) {
      uint32_t pid = strtoul(tok, nullptr, 0);
      addDeny(pid);
      Serial.printf("DENY  0x%03lX\n", (unsigned long)pid);
    } else {
      Serial.println("Usage: DENY <pid>");
    }
    return;
  }

  if (up == "SAVE") {
    if (cfg_save_to_nvs()) Serial.println("Saved.");
    else Serial.println("SAVE failed.");
    show_config();
    return;
  }

  if (up == "LOAD") {
    bool ok = cfg_load_from_nvs();
    if (!ok) {
      Serial.println("No saved config; using defaults.");
      cfg_apply_runtime_defaults();
    }
    apply_profile_and_dividers();
    show_config();
    return;
  }

  if (up == "RESETCFG") {
    if (cfg_reset_to_defaults()) Serial.println("Config reset to defaults.");
    else                         Serial.println("Failed to reset config.");
    apply_profile_and_dividers();
    show_config();
    return;
  }

  Serial.println("Unknown cmd. Try: SHOW | SHOW CFG | SHOW STATS | SHOW MAP | SHOW DENY | CAL 0 | CAL 1 | RATE <ms> | PROFILE ON|OFF | ALLOW <pid> <div> | DENY <pid> | SAVE | LOAD | RESETCFG");
}

// ===== Main loop =====
void loop() {
  loop_iteration++;
  esp_task_wdt_reset();

  servicePendingBleRestart();

  // Handle disconnect / reconnect
  if ((loop_iteration % 100) == 0 && !bleIsConnected()) {
    if (nvsWriteInProgress && !pendingBleRestart) {
      Serial.println("RC disconnected during SAVE; deferring until NVS completes.");
    }
    queueBleRestart("RC disconnect");
    servicePendingBleRestart();
  }

  // Ensure CAN is up
  while (!isCanBusReaderActive) {
    esp_task_wdt_reset();
    if (startCanBusReader()) {
      flushBufferedPackets();
      resetSkippedUpdatesCounters();
      lastCanMessageReceivedMs = millis();
      break;
    }
    delay(500);
  }

  const uint32_t now = millis();

  if (checkCanBusHealth(now)) {
    return;
  }

  // Watchdog after first frame
  if (have_seen_any_can && (now - lastCanMessageReceivedMs > 2000)) {
    handleCanFault("CAN RX timeout");
    return;
  }

  // BLE heartbeat (2 Hz)
  if (bleIsConnected() && now - lastHbMs >= 500) {
    lastHbMs = now;
    uint8_t hb[2] = { (uint8_t)(hbCounter & 0xFF), (uint8_t)(hbCounter >> 8) };
    bleSendCanData(HB_PID, hb, 2);
    hbCounter++;
  }

  // TWAI RX -> buffer
  CanFrame rx;
  while (ESP32Can.readFrame(rx, 0)) {
    if (rx.rtr) continue;
    have_seen_any_can = true;
    lastCanMessageReceivedMs = now;

    rxCount++;
    if (now - lastRxPrintMs >= 100) {
      lastRxPrintMs = now;
      Serial.printf("RX #%lu id=%03X dlc=%d\n",
                    (unsigned long)rxCount, rx.identifier, rx.data_length_code);
    }

    if (rx.data_length_code) {
      bufferNewPacket(rx.identifier, rx.data, rx.data_length_code);
    }
  }

  // Forward bursts within a small time budget
  handleBufferedPacketsBurst(2000);

  // Oil channel
  oil_update_and_publish_if_due();

  // Periodic timeout counter
  if (now - last_time_num_can_bus_timeouts_sent_ms > 2000) {
    sendNumCanBusTimeouts();
  }

  // GPS service (read UART + notify BLE)
  gpsService(now);

  updateBleGovernor(now);

  // ---- Robust Serial CLI: accept CR, LF, CRLF, or no line ending ----
  static char cliBuf[CLI_BUF_SIZE];
  static size_t cliLen = 0;
  static bool cliOverflow = false;
  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0) break;
    if (c == '\r' || c == '\n') {
      cliBuf[cliLen] = 0;
      if (cliOverflow) {
        Serial.println("CLI line too long; ignored.");
      } else if (cliLen) {
        process_cli_line(cliBuf);
      }
      cliLen = 0;
      cliOverflow = false;
      if (Serial.peek() == '\n' || Serial.peek() == '\r') Serial.read();
    } else if (cliLen < (CLI_BUF_SIZE - 1)) {
      cliBuf[cliLen++] = (char)c;
    } else {
      cliOverflow = true;
    }
  }
}
