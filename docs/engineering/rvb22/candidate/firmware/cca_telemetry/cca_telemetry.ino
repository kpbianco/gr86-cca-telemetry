// ===== GR86 CCA Telemetry: stability-first RaceChrono bridge =====
//
// This sketch intentionally favors deterministic recovery over aggressive retries:
// - BLE advertising is event-driven and never stop/start-cycled from loop().
// - RaceChrono uses its standard unencrypted 0x1FF8 protocol (no passkey/token shim).
// - CAN is always accept-all at the controller; filtering happens in software.
// - CAN silence is normal. Only an actual TWAI fault triggers recovery.
// - CAN bring-up and recovery are non-blocking, so BLE/GPS/CLI stay alive.
// - CAN frames are coalesced by ID before BLE transmission to prevent backlog storms.
//
// Board: ESP32-S3 Dev Module
// Baseline toolchain: Arduino-ESP32 3.3.6 + NimBLE-Arduino 2.3.6
// Rev B ERB corrective source; build identity and manifest distinguish images.

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <atomic>
#include <freertos/queue.h>
#include "src/racechrono_codec.h"
#include "src/oil_model.h"
#include "src/oil_fault_recovery.h"
#include "src/boot_diagnostics.h"
#include "src/driver_loss.h"
#include "src/runtime_timing.h"
#include "src/runtime_config.h"
#include "src/io_budget.h"
#include <esp_attr.h>

#include <driver/twai.h>
#include <esp_err.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#if defined(__has_include)
#  if __has_include(<esp_idf_version.h>)
#    include <esp_idf_version.h>
#  endif
#endif

#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "config.h"
#include "src/gps_nmea.h"
#include "src/gps_solution_epoch.h"
#include "src/gps_mode_config.h"
#include "src/led_status.h"
#include "src/nvs_cfg.h"

namespace cca {

// All application diagnostics are buffered. service() writes only bytes already
// accepted by UART TX capacity, up to64 bytes/pass; no blocking flush exists.
class DiagnosticLog final : public Print {
 public:
  using Print::write;
  size_t write(uint8_t c) override { return queue_.append(&c, 1); }
  size_t write(const uint8_t* data, size_t count) override {
    return queue_.append(data, count);
  }
  void service() {
    const int available = Serial.availableForWrite();
    if (available <= 0 || queue_.size() == 0) return;
    const size_t count = std::min<size_t>(64u, std::min<size_t>(
        static_cast<size_t>(available), queue_.contiguousSize()));
    queue_.consume(Serial.write(queue_.contiguousData(), count));
  }
  size_t queued() const { return queue_.size(); }
  size_t highWater() const { return queue_.highWater(); }
  uint64_t dropped() const { return queue_.dropped(); }
 private:
  io::ByteQueue<8192> queue_;
};
static DiagnosticLog g_log;
static uint32_t g_maxLoopGapUs = 0, g_previousLoopUs = 0;
static uint32_t g_cliCommands = 0;
static uint32_t g_canCacheCoalesced = 0, g_canQueueHighWater = 0;
static uint32_t g_canFilteredCount = 0;


// -----------------------------------------------------------------------------
// Build identity
// -----------------------------------------------------------------------------

static constexpr const char* BUILD_ID = "2.1.4-revb-recovered-20260909";

// -----------------------------------------------------------------------------
// Hardware pins
// -----------------------------------------------------------------------------

static constexpr int CAN_TX_GPIO = 5;
static constexpr int CAN_RX_GPIO = 4;

static constexpr int GPS_RX_GPIO = 18;  // GPS TX -> ESP RX
static constexpr int GPS_TX_GPIO = 17;  // GPS RX <- ESP TX

static constexpr int OIL_ADC_PIN = 1;
static constexpr int OIL_EXC_ADC_PIN = 2;
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Rev B requires an ESP32-S3 target"
#endif
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
#error "Rev B debug connector uses UART0; disable USB CDC on boot"
#endif

// -----------------------------------------------------------------------------
// RaceChrono DIY BLE UUIDs
// -----------------------------------------------------------------------------

static constexpr uint16_t RC_SERVICE_UUID = 0x1FF8;
static constexpr uint16_t RC_CHAR_CAN_UUID = 0x0001;
static constexpr uint16_t RC_CHAR_FILTER_UUID = 0x0002;
static constexpr uint16_t RC_CHAR_GPS_UUID = 0x0003;
static constexpr uint16_t RC_CHAR_GPS_TIME_UUID = 0x0004;

// -----------------------------------------------------------------------------
// General helpers
// -----------------------------------------------------------------------------

static inline bool timeReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

static inline uint32_t elapsedMs(uint32_t now, uint32_t then) {
  return now - then;
}

static inline float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static const char* resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
#ifdef ESP_RST_USB
    case ESP_RST_USB:       return "USB";
#endif
#ifdef ESP_RST_JTAG
    case ESP_RST_JTAG:      return "JTAG";
#endif
#ifdef ESP_RST_CPU_LOCKUP
    case ESP_RST_CPU_LOCKUP:return "CPU_LOCKUP";
#endif
    default:                return "OTHER";
  }
}

static esp_reset_reason_t g_bootResetReason = ESP_RST_UNKNOWN;
static RTC_NOINIT_ATTR diagnostics::BootRecord g_retainedBootRecord;

// -----------------------------------------------------------------------------
// Watchdog
// -----------------------------------------------------------------------------

static void initWatchdog() {
  esp_err_t initResult = ESP_OK;

#if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
  esp_task_wdt_config_t config = {};
  config.timeout_ms = 5000;
  config.idle_core_mask = 0;
  config.trigger_panic = true;
  initResult = esp_task_wdt_init(&config);
#else
  initResult = esp_task_wdt_init(5, true);
#endif

  if (initResult != ESP_OK && initResult != ESP_ERR_INVALID_STATE) {
    g_log.printf("WARN: task watchdog init failed: %d\n",
                  static_cast<int>(initResult));
  }

  const esp_err_t addResult = esp_task_wdt_add(nullptr);
  if (addResult != ESP_OK && addResult != ESP_ERR_INVALID_STATE &&
      addResult != ESP_ERR_INVALID_ARG) {
    g_log.printf("WARN: task watchdog add failed: %d\n",
                  static_cast<int>(addResult));
  }
}

// -----------------------------------------------------------------------------
// Configuration and runtime filtering
// -----------------------------------------------------------------------------

static Preferences g_prefs;
static constexpr const char* CFG_NAMESPACE = "cca_cfg_b";

static bool g_profileEnabled = true;
static uint16_t g_oilPublishPeriodMs = 20;

struct __attribute__((packed)) StoredPidDivider {
  uint16_t pid;
  uint8_t divider;
};

static constexpr size_t MAX_CUSTOM_DIVIDERS = 64;

static StoredPidDivider g_customDividers[MAX_CUSTOM_DIVIDERS] = {};
static uint16_t g_customDividerCount = 0;

static int customDividerIndex(uint32_t pid) {
  if (pid > 0xFFFFu) return -1;
  for (uint16_t i = 0; i < g_customDividerCount; ++i) {
    if (g_customDividers[i].pid == static_cast<uint16_t>(pid)) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

static uint8_t customDividerFor(uint32_t pid) {
  const int index = customDividerIndex(pid);
  if (index < 0) return 0;
  uint8_t divider = g_customDividers[index].divider;
  return divider == 0 ? 1 : divider;
}

static bool setCustomDivider(uint32_t pid, uint8_t divider) {
  if (pid > 0x7FFu) return false;
  if (divider == 0) divider = 1;

  const int existing = customDividerIndex(pid);
  if (existing >= 0) {
    g_customDividers[existing].divider = divider;
    return true;
  }

  if (g_customDividerCount >= MAX_CUSTOM_DIVIDERS) return false;
  g_customDividers[g_customDividerCount].pid = static_cast<uint16_t>(pid);
  g_customDividers[g_customDividerCount].divider = divider;
  ++g_customDividerCount;
  return true;
}

static void clearCustomDividers() {
  memset(g_customDividers, 0, sizeof(g_customDividers));
  g_customDividerCount = 0;
}

static constexpr size_t MAX_DENIED_PIDS = 64;
static uint32_t g_deniedPids[MAX_DENIED_PIDS] = {};
static size_t g_deniedPidCount = 0;

static bool isDenied(uint32_t pid) {
  for (size_t i = 0; i < g_deniedPidCount; ++i) {
    if (g_deniedPids[i] == pid) return true;
  }
  return false;
}

static void denyPid(uint32_t pid) {
  if (isDenied(pid)) return;
  if (g_deniedPidCount < MAX_DENIED_PIDS) {
    g_deniedPids[g_deniedPidCount++] = pid;
  }
}

static void undenyPid(uint32_t pid) {
  for (size_t i = 0; i < g_deniedPidCount; ++i) {
    if (g_deniedPids[i] != pid) continue;
    for (size_t j = i + 1; j < g_deniedPidCount; ++j) {
      g_deniedPids[j - 1] = g_deniedPids[j];
    }
    --g_deniedPidCount;
    return;
  }
}

static void clearDeniedPids() {
  memset(g_deniedPids, 0, sizeof(g_deniedPids));
  g_deniedPidCount = 0;
}

struct RequestedPid {
  uint32_t pid;
  uint16_t intervalMs;
  bool used;
};

static constexpr size_t MAX_REQUESTED_PIDS = 128;
static RequestedPid g_requestedPids[MAX_REQUESTED_PIDS] = {};
static bool g_raceChronoFilterActive = false;
static bool g_raceChronoAllowAll = false;
static uint16_t g_raceChronoAllowAllIntervalMs = 0;
static uint32_t g_filterCommandCount = 0;

static void clearRaceChronoFilter() {
  memset(g_requestedPids, 0, sizeof(g_requestedPids));
  g_raceChronoFilterActive = false;
  g_raceChronoAllowAll = false;
  g_raceChronoAllowAllIntervalMs = 0;
}

static RequestedPid* findRequestedPid(uint32_t pid) {
  for (size_t i = 0; i < MAX_REQUESTED_PIDS; ++i) {
    if (g_requestedPids[i].used && g_requestedPids[i].pid == pid) {
      return &g_requestedPids[i];
    }
  }
  return nullptr;
}

static bool requestPid(uint32_t pid, uint16_t intervalMs) {
  if (RequestedPid* existing = findRequestedPid(pid)) {
    existing->intervalMs = intervalMs;
    return true;
  }

  for (size_t i = 0; i < MAX_REQUESTED_PIDS; ++i) {
    if (g_requestedPids[i].used) continue;
    g_requestedPids[i].pid = pid;
    g_requestedPids[i].intervalMs = intervalMs;
    g_requestedPids[i].used = true;
    return true;
  }

  return false;
}

static size_t requestedPidCount() {
  size_t count = 0;
  for (size_t i = 0; i < MAX_REQUESTED_PIDS; ++i) {
    if (g_requestedPids[i].used) ++count;
  }
  return count;
}

static uint8_t profileDividerFor(uint32_t pid) {
  const auto* map = ACTIVE_PID_MAP;
  if (map != nullptr) {
    for (size_t i = 0; i < map->ruleCount; ++i) {
      if (map->rules[i].pid == pid) {
        const uint8_t divider = map->rules[i].divider;
        return divider == 0 ? 1 : divider;
      }
    }
    if (map->policyDividerForId != nullptr) {
      const uint8_t divider = map->policyDividerForId(pid);
      if (divider != 0) return divider;
    }
  }

  return DEFAULT_UPDATE_RATE_DIVIDER == 0 ? 1
                                          : DEFAULT_UPDATE_RATE_DIVIDER;
}

static bool profileAllows(uint32_t pid) {
  const auto* map = ACTIVE_PID_MAP;
  if (map == nullptr || map->isCanIdWhitelisted == nullptr) return false;
  return map->isCanIdWhitelisted(pid);
}

struct RouteDecision {
  bool allowed;
  uint16_t minimumIntervalMs;
};

static RouteDecision routeDecision(uint32_t pid) {
  RouteDecision result = {false, 40};

  if (isDenied(pid)) return result;

  // A serial ALLOW command is an explicit operator override.
  const uint8_t customDivider = customDividerFor(pid);
  if (customDivider != 0) {
    result.allowed = true;
    uint32_t customInterval = static_cast<uint32_t>(customDivider) * 10u;
    if (customInterval < 10u) customInterval = 10u;
    if (customInterval > 2000u) customInterval = 2000u;
    result.minimumIntervalMs = static_cast<uint16_t>(customInterval);
    return result;
  }

  // PROFILE OFF is the deliberate bench/sniff mode. It ignores RaceChrono's
  // requested PID set but still honors the deny list and the BLE global cap.
  if (!g_profileEnabled) {
    result.allowed = true;
    result.minimumIntervalMs = 10;
    return result;
  }

  // Once RaceChrono sends a filter command, follow the standard protocol.
  if (g_raceChronoFilterActive) {
    if (g_raceChronoAllowAll) {
      result.allowed = true;
      result.minimumIntervalMs =
          g_raceChronoAllowAllIntervalMs == 0
              ? 10
              : g_raceChronoAllowAllIntervalMs;
      return result;
    }

    if (RequestedPid* requested = findRequestedPid(pid)) {
      result.allowed = true;
      result.minimumIntervalMs =
          requested->intervalMs == 0 ? 10 : requested->intervalMs;
      return result;
    }

    return result;
  }

  // Before RaceChrono sends filters, use the compiled GR86 profile so the
  // device still streams useful data in generic BLE clients.
  if (!profileAllows(pid)) return result;

  result.allowed = true;
  if (pid == 0x710u) {
    result.minimumIntervalMs = g_oilPublishPeriodMs;
  } else if (pid == 0x777u) {
    result.minimumIntervalMs = 2000;
  } else {
    const uint8_t divider = profileDividerFor(pid);
    uint32_t interval = static_cast<uint32_t>(divider) * 10u;
    // Cap per-ID output at 25 Hz even for profile entries with divider 1.
    if (interval < 40u) interval = 40u;
    if (interval > 2000u) interval = 2000u;
    result.minimumIntervalMs = static_cast<uint16_t>(interval);
  }

  return result;
}

static void loadRuntimeConfig() {
  runtime_config::Snapshot stored;
  const bool loaded=runtime_config::load(stored);
  g_profileEnabled=stored.profileEnabled!=0;
  g_oilPublishPeriodMs=stored.oilPeriodMs;
  clearCustomDividers();
  for (uint16_t i=0;i<stored.count;++i)
    setCustomDivider(stored.items[i].pid,stored.items[i].divider);
  if (!loaded) g_log.println("Runtime configuration missing/incompatible/CRC invalid; using defaults.");
}

static bool saveRuntimeConfig() {
  runtime_config::Snapshot value;
  value.profileEnabled=g_profileEnabled?1:0;
  value.oilPeriodMs=g_oilPublishPeriodMs;
  value.count=g_customDividerCount;
  for (uint16_t i=0;i<value.count;++i) {
    value.items[i].pid=g_customDividers[i].pid;
    value.items[i].divider=g_customDividers[i].divider;
  }
  return runtime_config::save(value);
}

// -----------------------------------------------------------------------------
// BLE transport
// -----------------------------------------------------------------------------

static NimBLEServer* g_bleServer = nullptr;
static NimBLEAdvertising* g_bleAdvertising = nullptr;
static NimBLEService* g_bleService = nullptr;
static NimBLECharacteristic* g_canCharacteristic = nullptr;
static NimBLECharacteristic* g_filterCharacteristic = nullptr;
static NimBLECharacteristic* g_gpsCharacteristic = nullptr;
static NimBLECharacteristic* g_gpsTimeCharacteristic = nullptr;

static bool g_bleConnected = false;
static bool g_canSubscribed = false;
static bool g_gpsSubscribed = false;
static bool g_gpsTimeSubscribed = false;

static uint16_t g_bleMtu = 23;
static uint32_t g_bleConnectCount = 0;
static uint32_t g_bleDisconnectCount = 0;
static int g_lastBleDisconnectReason = 0;
static uint32_t g_lastBleConnectMs = 0;
static uint32_t g_lastBleDisconnectMs = 0;
static uint32_t g_advertisingStartCount = 0;
static uint32_t g_advertisingStartFailures = 0;
static uint32_t g_nextAdvertisingCheckMs = 0;

static uint32_t g_bleNotifySuccesses = 0;
static uint32_t g_bleNotifyFailures = 0;
static uint32_t g_bleNotifyRateDrops = 0;
static uint32_t g_bleNotifyUnsubscribedDrops = 0;
static timing::Backoff g_bleNotifyBackoff;
static uint32_t g_lastBleNotifyFailureMs = 0;

// Shared token bucket. GPS consumes ~11 notifications/s; the remaining budget
// is available for CAN. This prevents a noisy bus from starving NimBLE.
static constexpr uint16_t BLE_TOKEN_CAPACITY = 24;
static constexpr uint16_t BLE_TOKEN_RATE_PER_SECOND = 120;
static io::TokenBucket g_bleBudget(BLE_TOKEN_CAPACITY, BLE_TOKEN_RATE_PER_SECOND);

static bool isBleConnected() {
  return g_bleConnected && g_bleServer != nullptr &&
         g_bleServer->getConnectedCount() > 0;
}

static bool takeBleToken(uint32_t now, uint16_t reserve = 0) {
  if (g_bleBudget.take(now, reserve)) return true;
  ++g_bleNotifyRateDrops;
  return false;
}

static bool notifyCharacteristic(NimBLECharacteristic* characteristic,
                                 bool subscribed,
                                 const uint8_t* data,
                                 size_t length,
                                 uint32_t now) {
  if (!isBleConnected() || characteristic == nullptr) return false;

  if (!subscribed) {
    ++g_bleNotifyUnsubscribedDrops;
    return false;
  }

  if (g_bleNotifyBackoff.pending(now)) return false;
  const bool oilPacket = characteristic == g_canCharacteristic && length >= 4u &&
      data[0] == 0x10u && data[1] == 0x07u && data[2] == 0 && data[3] == 0;
  // Keep room for a coincident GPS solution, GPS time and oil update. This
  // reserves burst capacity; the total120 notifications/s cap is unchanged.
  const uint16_t reserve = characteristic == g_canCharacteristic && !oilPacket ? 3u : 0u;
  if (!takeBleToken(now, reserve)) return false;

  characteristic->setValue(data, length);
  if (characteristic->notify()) {
    ++g_bleNotifySuccesses;
    return true;
  }

  ++g_bleNotifyFailures;
  g_lastBleNotifyFailureMs = now;
  // A short quiet period is enough to let NimBLE drain. Do not restart BLE.
  g_bleNotifyBackoff.start(now, 100);
  return false;
}

static void startAdvertisingIfNeeded(uint32_t now, bool forceCheck = false) {
  if (g_bleAdvertising == nullptr || isBleConnected()) return;
  if (!forceCheck && !timeReached(now, g_nextAdvertisingCheckMs)) return;

  g_nextAdvertisingCheckMs = now + 3000;
  if (g_bleAdvertising->isAdvertising()) return;

  if (g_bleAdvertising->start()) {
    ++g_advertisingStartCount;
    g_log.println("BLE: advertising");
  } else {
    ++g_advertisingStartFailures;
    g_log.println("WARN: BLE advertising start rejected; retry scheduled");
  }
}

enum class BleEventType : uint8_t {
  Connect, Disconnect, Mtu, CanSubscription, GpsSubscription,
  TimeSubscription, NotifyFailure, Filter
};
struct BleEvent {
  BleEventType type;
  uint16_t value;
  int code;
  uint8_t length;
  uint8_t data[7];
};
static QueueHandle_t g_bleEvents = nullptr;
static std::atomic<uint32_t> g_bleEventDrops{0};
static std::atomic<bool> g_bleEventOverflow{false};

static void queueBleEvent(const BleEvent& event) {
  if (!g_bleEvents || xQueueSend(g_bleEvents, &event, 0) != pdTRUE) {
    ++g_bleEventDrops;
    g_bleEventOverflow.store(true);
  }
}

class ServerCallbacks final : public NimBLEServerCallbacks {
 public:
  void onConnect(NimBLEServer*, NimBLEConnInfo& info) override {
    queueBleEvent({BleEventType::Connect, info.getMTU(), 0, 0, {}});
  }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int reason) override {
    queueBleEvent({BleEventType::Disconnect, 0, reason, 0, {}});
  }
  void onMTUChange(uint16_t mtu, NimBLEConnInfo&) override {
    queueBleEvent({BleEventType::Mtu, mtu, 0, 0, {}});
  }
};
static ServerCallbacks g_serverCallbacks;

class SubscriptionCallbacks final : public NimBLECharacteristicCallbacks {
 public:
  explicit SubscriptionCallbacks(BleEventType type) : type_(type) {}
  void onSubscribe(NimBLECharacteristic*, NimBLEConnInfo&,
                   uint16_t value) override {
    queueBleEvent({type_, value, 0, 0, {}});
  }
  void onStatus(NimBLECharacteristic*, int code) override {
    if (code) queueBleEvent({BleEventType::NotifyFailure, 0, code, 0, {}});
  }
 private:
  BleEventType type_;
};
static SubscriptionCallbacks g_canSubscriptionCallbacks(BleEventType::CanSubscription);
static SubscriptionCallbacks g_gpsSubscriptionCallbacks(BleEventType::GpsSubscription);
static SubscriptionCallbacks g_gpsTimeSubscriptionCallbacks(BleEventType::TimeSubscription);

static void handleRaceChronoFilterWrite(const uint8_t* data, size_t length) {
  if (data == nullptr || length < 1) return;

  ++g_filterCommandCount;

  switch (data[0]) {
    case 0:  // Deny all.
      if (length != 1) return;
      memset(g_requestedPids, 0, sizeof(g_requestedPids));
      g_raceChronoFilterActive = true;
      g_raceChronoAllowAll = false;
      g_raceChronoAllowAllIntervalMs = 0;
      g_log.println("FIL: deny all");
      return;

    case 1:  // Allow all, interval is big-endian.
      if (length != 3) return;
      memset(g_requestedPids, 0, sizeof(g_requestedPids));
      g_raceChronoFilterActive = true;
      g_raceChronoAllowAll = true;
      g_raceChronoAllowAllIntervalMs =
          (static_cast<uint16_t>(data[1]) << 8) |
          static_cast<uint16_t>(data[2]);
      g_log.printf("FIL: allow all interval=%ums\n",
                    static_cast<unsigned>(g_raceChronoAllowAllIntervalMs));
      return;

    case 2: {  // Allow one PID; interval and PID are big-endian.
      if (length != 7) return;
      const uint16_t interval =
          (static_cast<uint16_t>(data[1]) << 8) |
          static_cast<uint16_t>(data[2]);
      const uint32_t pid =
          (static_cast<uint32_t>(data[3]) << 24) |
          (static_cast<uint32_t>(data[4]) << 16) |
          (static_cast<uint32_t>(data[5]) << 8) |
          static_cast<uint32_t>(data[6]);

      g_raceChronoFilterActive = true;
      g_raceChronoAllowAll = false;
      if (!requestPid(pid, interval)) {
        g_log.printf("WARN: FIL PID table full; dropped 0x%03lX\n",
                      static_cast<unsigned long>(pid));
      }
      return;
    }

    default:
      return;
  }
}

class FilterCallbacks final : public NimBLECharacteristicCallbacks {
 public:
  void onWrite(NimBLECharacteristic* characteristic,
               NimBLEConnInfo&) override {
    if (characteristic == nullptr) return;
    const std::string value = characteristic->getValue();
    if (value.size() != 1 && value.size() != 3 && value.size() != 7) return;
    BleEvent event{};
    event.type = BleEventType::Filter;
    event.length = static_cast<uint8_t>(value.size());
    memcpy(event.data, value.data(), value.size());
    queueBleEvent(event);
  }
};

static FilterCallbacks g_filterCallbacks;

static void discardCanCache();
// Only loop() owns routing, tokens and connection state. NimBLE callbacks enqueue.
static void serviceBleEvents(uint32_t now) {
  BleEvent event{};
  // A concurrent producer must not extend this loop indefinitely. Leave any
  // remainder queued for the next pass; overflow still forces reconnection.
  for (unsigned processed = 0; processed < 32u && g_bleEvents &&
       xQueueReceive(g_bleEvents, &event, 0) == pdTRUE; ++processed) {
    switch (event.type) {
      case BleEventType::Connect:
        discardCanCache(); // Only new acquisitions may enter a new BLE session.
        g_bleConnected = true;
        g_canSubscribed = g_gpsSubscribed = g_gpsTimeSubscribed = false;
        g_bleMtu = event.value;
        g_lastBleConnectMs = now;
        ++g_bleConnectCount;
        g_bleNotifyBackoff.clear();
        g_bleBudget.reset(now);
        clearRaceChronoFilter();
        g_log.println("BLE: connected; awaiting subscriptions/filters");
        break;
      case BleEventType::Disconnect:
        discardCanCache();
        g_bleConnected = false;
        g_canSubscribed = g_gpsSubscribed = g_gpsTimeSubscribed = false;
        g_lastBleDisconnectReason = event.code;
        g_lastBleDisconnectMs = now;
        ++g_bleDisconnectCount;
        g_nextAdvertisingCheckMs = now + 1000;
        g_log.printf("BLE: disconnected reason=%d\n", event.code);
        break;
      case BleEventType::Mtu: g_bleMtu = event.value; break;
      case BleEventType::CanSubscription: g_canSubscribed = event.value & 1; break;
      case BleEventType::GpsSubscription: g_gpsSubscribed = event.value & 1; break;
      case BleEventType::TimeSubscription: g_gpsTimeSubscribed = event.value & 1; break;
      case BleEventType::NotifyFailure:
        ++g_bleNotifyFailures;
        g_lastBleNotifyFailureMs = now;
        g_bleNotifyBackoff.start(now, 100);
        break;
      case BleEventType::Filter:
        handleRaceChronoFilterWrite(event.data, event.length);
        break;
    }
  }
  if (g_bleEventOverflow.exchange(false)) {
    // A partial filter sequence must not be silently accepted. Force reconnect.
    g_bleConnected = false;
    g_canSubscribed = g_gpsSubscribed = g_gpsTimeSubscribed = false;
    clearRaceChronoFilter();
    discardCanCache();
    if (g_bleServer) {
      for (const auto handle : g_bleServer->getPeerDevices())
        g_bleServer->disconnect(handle);
    }
    g_log.println("BLE: event queue overflow; reconnect required");
  }
}

static void initBle() {
  g_log.println("BLE: initializing stability profile");

  g_bleEvents = xQueueCreate(160, sizeof(BleEvent));
  if (!g_bleEvents) {
    g_log.println("FATAL: BLE event queue allocation failed");
    ESP.restart();
  }
  NimBLEDevice::init(DEVICE_NAME);

  // 3 dBm is ample inside a vehicle and materially reduces radio current peaks
  // compared with the previous maximum-power setting.
  NimBLEDevice::setPower(static_cast<int8_t>(3));

  // RaceChrono's documented DIY service does not require pairing. The previous
  // passkey/encrypted-filter layer caused iOS pairing churn and rejected the
  // standard 1/3/7-byte filter packets.
  NimBLEDevice::setSecurityAuth(false, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  if (NimBLEDevice::getNumBonds() > 0) {
    NimBLEDevice::deleteAllBonds();
    g_log.println("BLE: cleared obsolete bonds");
  }

  g_bleServer = NimBLEDevice::createServer();
  g_bleServer->setCallbacks(&g_serverCallbacks, false);
  g_bleServer->advertiseOnDisconnect(true);

  g_bleService = g_bleServer->createService(NimBLEUUID(RC_SERVICE_UUID));

  g_canCharacteristic = g_bleService->createCharacteristic(
      NimBLEUUID(RC_CHAR_CAN_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  g_filterCharacteristic = g_bleService->createCharacteristic(
      NimBLEUUID(RC_CHAR_FILTER_UUID), NIMBLE_PROPERTY::WRITE);
  g_gpsCharacteristic = g_bleService->createCharacteristic(
      NimBLEUUID(RC_CHAR_GPS_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  g_gpsTimeCharacteristic = g_bleService->createCharacteristic(
      NimBLEUUID(RC_CHAR_GPS_TIME_UUID),
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  g_canCharacteristic->setCallbacks(&g_canSubscriptionCallbacks);
  g_filterCharacteristic->setCallbacks(&g_filterCallbacks);
  g_gpsCharacteristic->setCallbacks(&g_gpsSubscriptionCallbacks);
  g_gpsTimeCharacteristic->setCallbacks(
      &g_gpsTimeSubscriptionCallbacks);

  uint8_t initialCan[4] = {0, 0, 0, 0};
  uint8_t initialGps[20];
  memset(initialGps, 0xFF, sizeof(initialGps));
  uint8_t initialGpsTime[3] = {0, 0, 0};

  g_canCharacteristic->setValue(initialCan, sizeof(initialCan));
  g_gpsCharacteristic->setValue(initialGps, sizeof(initialGps));
  g_gpsTimeCharacteristic->setValue(initialGpsTime,
                                    sizeof(initialGpsTime));

  g_bleService->start();

  g_bleAdvertising = NimBLEDevice::getAdvertising();
  g_bleAdvertising->setName(DEVICE_NAME);
  g_bleAdvertising->setMinInterval(32);   // 20 ms
  g_bleAdvertising->setMaxInterval(160);  // 100 ms
  g_bleAdvertising->enableScanResponse(false);
  g_bleAdvertising->addServiceUUID(NimBLEUUID(RC_SERVICE_UUID));

  g_nextAdvertisingCheckMs = 0;
  startAdvertisingIfNeeded(millis(), true);
}

// -----------------------------------------------------------------------------
// TWAI/CAN
// -----------------------------------------------------------------------------

static bool g_canDriverInstalled = false;
static bool g_canRunning = false;
static uint32_t g_canNextStartMs = 0;
static uint32_t g_canStartBackoffMs = 500;
static uint32_t g_canStartAttempts = 0;
static uint32_t g_canStartFailures = 0;
static uint32_t g_canRestartCount = 0;
static uint32_t g_canBusOffCount = 0;
static uint32_t g_canFaultCount = 0;
static uint32_t g_canRxCount = 0;
static uint32_t g_canExtendedCount = 0;
static uint32_t g_canRtrCount = 0;
static uint32_t g_canInvalidDlcCount = 0;
static uint32_t g_canRxQueueFullCount = 0;
// Boot-lifetime totals preserve loss evidence across TWAI reinstallations.
static diagnostics::DriverLossCounter g_canRxMissedCount, g_canRxOverrunCount;
static uint64_t g_canRxDiscardedCount = 0;
static uint32_t g_canStatusReadFailures = 0;
static uint32_t g_canSlotEvictions = 0;
static uint32_t g_canStaleDropCount = 0;
static constexpr uint32_t CAN_MAX_CACHE_AGE_MS = 500;
static uint32_t g_virtualIdCollisionCount = 0;
static uint32_t g_lastCanFrameMs = 0;
static uint32_t g_lastCanHealthMs = 0;
static twai_status_info_t g_lastTwaiStatus = {};

static constexpr uint32_t TWAI_ALERTS =
    TWAI_ALERT_ERR_PASS |
    TWAI_ALERT_BUS_OFF |
    TWAI_ALERT_BUS_RECOVERED |
    TWAI_ALERT_RECOVERY_IN_PROGRESS |
    TWAI_ALERT_ABOVE_ERR_WARN |
    TWAI_ALERT_RX_QUEUE_FULL |
    TWAI_ALERT_TX_FAILED |
    TWAI_ALERT_ARB_LOST;

struct CanSlot {
  bool used;
  bool dirty;
  bool extended;
  uint32_t pid;
  uint8_t length;
  uint8_t data[8];
  uint8_t lastSentLength;
  uint8_t lastSentData[8];
  bool hasLastSent;
  uint32_t lastRxMs;
  uint32_t lastSentMs;
  uint32_t receiveCount;
  uint32_t coalescedCount;
};

static constexpr size_t CAN_SLOT_COUNT = 96;
static CanSlot g_canSlots[CAN_SLOT_COUNT] = {};
static size_t g_canForwardCursor = 0;

static void discardCanCache() {
  memset(g_canSlots, 0, sizeof(g_canSlots));
  g_canForwardCursor = 0;
}

static CanSlot* findCanSlot(uint32_t pid, bool extended) {
  for (size_t i = 0; i < CAN_SLOT_COUNT; ++i) {
    if (g_canSlots[i].used && g_canSlots[i].pid == pid &&
        g_canSlots[i].extended == extended) {
      return &g_canSlots[i];
    }
  }
  return nullptr;
}

static CanSlot* acquireCanSlot(uint32_t pid,
                               bool extended,
                               uint32_t now) {
  if (CanSlot* existing = findCanSlot(pid, extended)) return existing;

  for (size_t i = 0; i < CAN_SLOT_COUNT; ++i) {
    if (g_canSlots[i].used) continue;
    memset(&g_canSlots[i], 0, sizeof(g_canSlots[i]));
    g_canSlots[i].used = true;
    g_canSlots[i].pid = pid;
    g_canSlots[i].extended = extended;
    g_canSlots[i].lastRxMs = now;
    return &g_canSlots[i];
  }

  // Local oil/diagnostic state must survive physical-bus cache saturation.
  // serviceCanReceive rejects physical standard IDs 0x710 and 0x777.
  size_t oldestIndex = CAN_SLOT_COUNT;
  uint32_t oldestAge = 0;
  for (size_t i = 0; i < CAN_SLOT_COUNT; ++i) {
    if (!g_canSlots[i].extended &&
        (g_canSlots[i].pid == 0x710u || g_canSlots[i].pid == 0x777u)) continue;
    const uint32_t age = now - g_canSlots[i].lastRxMs;
    if (oldestIndex == CAN_SLOT_COUNT || age >= oldestAge) {
      oldestAge = age;
      oldestIndex = i;
    }
  }

  if (oldestIndex == CAN_SLOT_COUNT) return nullptr;
  ++g_canSlotEvictions;
  memset(&g_canSlots[oldestIndex], 0, sizeof(g_canSlots[oldestIndex]));
  g_canSlots[oldestIndex].used = true;
  g_canSlots[oldestIndex].pid = pid;
  g_canSlots[oldestIndex].extended = extended;
  g_canSlots[oldestIndex].lastRxMs = now;
  return &g_canSlots[oldestIndex];
}

static void cacheCanFrame(uint32_t pid,
                          bool extended,
                          const uint8_t* data,
                          uint8_t length,
                          uint32_t now) {
  CanSlot* slot = acquireCanSlot(pid, extended, now);
  if (slot == nullptr) return;

  if (slot->dirty) {
    if (slot->coalescedCount != UINT32_MAX) ++slot->coalescedCount;
    if (g_canCacheCoalesced != UINT32_MAX) ++g_canCacheCoalesced;
  }
  slot->length = length > 8 ? 8 : length;
  if (slot->length > 0 && data != nullptr) {
    memcpy(slot->data, data, slot->length);
  }
  if (slot->length < sizeof(slot->data)) {
    memset(slot->data + slot->length, 0,
           sizeof(slot->data) - slot->length);
  }

  slot->lastRxMs = now;
  slot->dirty = true;
  if (slot->receiveCount != 0xFFFFFFFFu) ++slot->receiveCount;
}

static void publishVirtualCan(uint32_t pid,
                              const uint8_t* data,
                              uint8_t length,
                              uint32_t now) {
  cacheCanFrame(pid, false, data, length, now);
}

static void updateCanStatus(const twai_status_info_t& status) {
  g_lastTwaiStatus = status;
  g_canRxMissedCount.observe(status.rx_missed_count);
  g_canRxOverrunCount.observe(status.rx_overrun_count);
  // This is the largest observed queue depth, not an ISR-level absolute peak.
  g_canQueueHighWater = std::max<uint32_t>(g_canQueueHighWater, status.msgs_to_rx);
}

static bool sampleCanStatus() {
  twai_status_info_t status = {};
  if (twai_get_status_info(&status) != ESP_OK) {
    if (g_canStatusReadFailures != UINT32_MAX) ++g_canStatusReadFailures;
    return false;
  }
  updateCanStatus(status);
  return true;
}

static void stopCanDriver() {
  if (g_canDriverInstalled) {
    // Stop acquisitions before the final snapshot so uninstall cannot silently
    // erase readable loss counters or already queued, unprocessed frames.
    (void)twai_stop();
    if (sampleCanStatus()) {
      diagnostics::saturatingAdd(g_canRxDiscardedCount, g_lastTwaiStatus.msgs_to_rx);
    }
    (void)twai_driver_uninstall();
  }

  g_canDriverInstalled = false;
  g_canRunning = false;
  memset(&g_lastTwaiStatus, 0, sizeof(g_lastTwaiStatus));
  g_lastTwaiStatus.state = TWAI_STATE_STOPPED;
}

static void scheduleCanRestart(uint32_t now, const char* reason) {
  if (reason != nullptr) {
    g_log.printf("CAN: recovery scheduled (%s)\n", reason);
  }

  stopCanDriver();
  ++g_canFaultCount;
  g_canNextStartMs = now + g_canStartBackoffMs;

  if (g_canStartBackoffMs < 10000u) {
    g_canStartBackoffMs *= 2u;
    if (g_canStartBackoffMs > 10000u) g_canStartBackoffMs = 10000u;
  }
}

static bool startCanDriver(uint32_t now) {
  ++g_canStartAttempts;

  twai_general_config_t general = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(CAN_TX_GPIO),
      static_cast<gpio_num_t>(CAN_RX_GPIO),
      TWAI_MODE_LISTEN_ONLY);
  general.tx_queue_len = 0;  // Receive-only build has no transmit queue.
  general.rx_queue_len = 128;
  general.alerts_enabled = TWAI_ALERTS;

  twai_timing_config_t timing = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t installResult =
      twai_driver_install(&general, &timing, &filter);

  if (installResult == ESP_ERR_INVALID_STATE) {
    // Clean up any stale driver state left by an interrupted recovery.
    (void)twai_stop();
    (void)twai_driver_uninstall();
    installResult = twai_driver_install(&general, &timing, &filter);
  }

  if (installResult != ESP_OK) {
    ++g_canStartFailures;
    g_canRunning = false;
    g_canDriverInstalled = false;
    g_canNextStartMs = now + g_canStartBackoffMs;
    g_log.printf("WARN: TWAI install failed: %d; retry in %lums\n",
                  static_cast<int>(installResult),
                  static_cast<unsigned long>(g_canStartBackoffMs));
    if (g_canStartBackoffMs < 10000u) {
      g_canStartBackoffMs *= 2u;
      if (g_canStartBackoffMs > 10000u) g_canStartBackoffMs = 10000u;
    }
    return false;
  }

  g_canDriverInstalled = true;
  // The newly installed driver's counters begin at zero; accumulated totals
  // remain valid until MCU reset and must not be reset by CAN RESTART.
  g_canRxMissedCount.beginDriver();
  g_canRxOverrunCount.beginDriver();

  const esp_err_t startResult = twai_start();
  if (startResult != ESP_OK) {
    ++g_canStartFailures;
    g_log.printf("WARN: TWAI start failed: %d\n",
                  static_cast<int>(startResult));
    stopCanDriver();
    g_canNextStartMs = now + g_canStartBackoffMs;
    return false;
  }

  g_canRunning = true;
  g_canStartBackoffMs = 500;
  g_canNextStartMs = 0;
  ++g_canRestartCount;
  g_lastCanHealthMs = now;
  memset(&g_lastTwaiStatus, 0, sizeof(g_lastTwaiStatus));
  g_lastTwaiStatus.state = TWAI_STATE_RUNNING;

  g_log.println("CAN: TWAI running, 500 kbit/s, accept-all");
  return true;
}

static void serviceCanStart(uint32_t now) {
  if (g_canRunning) return;
  if (!timeReached(now, g_canNextStartMs)) return;
  startCanDriver(now);
}

static void serviceCanHealth(uint32_t now) {
  if (!g_canRunning || !g_canDriverInstalled) return;
  if (now - g_lastCanHealthMs < 1000u) return;
  g_lastCanHealthMs = now;

  uint32_t alerts = 0;
  const esp_err_t alertResult =
      twai_read_alerts(&alerts, pdMS_TO_TICKS(0));
  if (alertResult != ESP_OK && alertResult != ESP_ERR_TIMEOUT) {
    g_log.printf("WARN: TWAI alert read failed: %d\n",
                  static_cast<int>(alertResult));
  }

  twai_status_info_t status = {};
  const esp_err_t statusResult = twai_get_status_info(&status);
  if (statusResult != ESP_OK) {
    if (g_canStatusReadFailures != UINT32_MAX) ++g_canStatusReadFailures;
    scheduleCanRestart(now, "status read failed");
    return;
  }

  updateCanStatus(status);

  if ((alerts & TWAI_ALERT_RX_QUEUE_FULL) != 0) {
    ++g_canRxQueueFullCount;
  }

  if (status.state == TWAI_STATE_BUS_OFF ||
      (alerts & TWAI_ALERT_BUS_OFF) != 0) {
    ++g_canBusOffCount;
    scheduleCanRestart(now, "bus off");
    return;
  }

  if (status.state == TWAI_STATE_STOPPED) {
    scheduleCanRestart(now, "controller stopped");
    return;
  }

  // CAN silence is not a fault. The bench generator may stop and the vehicle
  // may enter quiet states; no timeout-based controller reset is performed.
}

static void serviceCanReceive(uint32_t now) {
  if (!g_canRunning) return;

  // Sample on every receive pass, before draining, so sub-second congestion
  // remains visible. This read is bounded driver work; target timing is tested
  // separately from the host contract tests.
  (void)sampleCanStatus();
  const uint32_t startUs = micros();
  uint16_t framesThisPass = 0;

  while (framesThisPass < 96 &&
         static_cast<uint32_t>(micros() - startUs) < 2000u) {
    twai_message_t frame = {};
    if (twai_receive(&frame, pdMS_TO_TICKS(0)) != ESP_OK) break;

    ++framesThisPass;
    if (frame.rtr) {
      ++g_canRtrCount;
      continue;
    }

    if (frame.data_length_code > 8) {
      ++g_canInvalidDlcCount;
      continue;
    }

    ++g_canRxCount;
    if (frame.extd) ++g_canExtendedCount;
    g_lastCanFrameMs = now;

    // These BLE packet IDs belong to local oil/status. Physical traffic must
    // never replace a calibrated oil reading with unrelated vehicle bytes.
    if (frame.identifier == 0x710u || frame.identifier == 0x777u) {
      ++g_virtualIdCollisionCount;
      continue;
    }

    // Count all valid acquisition, but do not let unrequested traffic evict
    // requested telemetry from the bounded latest-value cache.
    if (!routeDecision(frame.identifier).allowed) {
      ++g_canFilteredCount;
      continue;
    }

    cacheCanFrame(frame.identifier, frame.extd != 0, frame.data,
                  frame.data_length_code, now);
  }
}

static bool sendCanSlot(CanSlot& slot, uint32_t now) {
  uint8_t packet[12] = {};
  packet[0] = static_cast<uint8_t>(slot.pid & 0xFFu);
  packet[1] = static_cast<uint8_t>((slot.pid >> 8) & 0xFFu);
  packet[2] = static_cast<uint8_t>((slot.pid >> 16) & 0xFFu);
  packet[3] = static_cast<uint8_t>((slot.pid >> 24) & 0xFFu);
  if (slot.length > 0) {
    memcpy(packet + 4, slot.data, slot.length);
  }

  if (!notifyCharacteristic(g_canCharacteristic, g_canSubscribed,
                            packet, 4u + slot.length, now)) {
    return false;
  }

  slot.lastSentLength = slot.length;
  memcpy(slot.lastSentData, slot.data, sizeof(slot.lastSentData));
  slot.hasLastSent = true;
  slot.lastSentMs = now;
  slot.dirty = false;
  slot.receiveCount = 0;
  return true;
}

static void serviceCanForwarding(uint32_t now) {
  if (!isBleConnected() || !g_canSubscribed) return;
  if (g_bleNotifyBackoff.pending(now)) return;

  uint8_t sentThisPass = 0;

  // Oil is locally sampled telemetry with a defined response time. Give its
  // current value first access after GPS; other CAN IDs remain round-robin.
  // RaceChrono's deny/request and minimum-interval rules still apply.
  for (size_t i = 0; i < CAN_SLOT_COUNT; ++i) {
    CanSlot& slot = g_canSlots[i];
    if (!slot.used || !slot.dirty || slot.pid != 0x710u) continue;
    if (now - slot.lastRxMs > CAN_MAX_CACHE_AGE_MS) {
      slot.dirty = false;
      ++g_canStaleDropCount;
      continue;
    }
    const RouteDecision decision = routeDecision(slot.pid);
    if (!decision.allowed || (slot.hasLastSent &&
        now - slot.lastSentMs < decision.minimumIntervalMs)) continue;
    if (!sendCanSlot(slot, now)) return;
    ++sentThisPass;
  }

  for (size_t checked = 0;
       checked < CAN_SLOT_COUNT && sentThisPass < 4;
       ++checked) {
    const size_t index = g_canForwardCursor % CAN_SLOT_COUNT;
    g_canForwardCursor = (g_canForwardCursor + 1) % CAN_SLOT_COUNT;

    CanSlot& slot = g_canSlots[index];
    if (!slot.used || !slot.dirty) continue;
    if (now - slot.lastRxMs > CAN_MAX_CACHE_AGE_MS) {
      slot.dirty = false;
      ++g_canStaleDropCount;
      continue;
    }

    const RouteDecision decision = routeDecision(slot.pid);
    if (!decision.allowed) {
      slot.dirty = false;
      slot.receiveCount = 0;
      continue;
    }

    if (slot.hasLastSent &&
        now - slot.lastSentMs < decision.minimumIntervalMs) {
      continue;
    }

    if (sendCanSlot(slot, now)) {
      ++sentThisPass;
    } else {
      // Keep the latest value dirty for a later pass; do not spin.
      break;
    }
  }
}

// -----------------------------------------------------------------------------
// GPS
// -----------------------------------------------------------------------------

static HardwareSerial g_gpsSerial(1);
static std::atomic<uint32_t> g_gpsFifoOverflows{0}, g_gpsRingOverflows{0};
static std::atomic<uint32_t> g_gpsUartErrors{0};
static std::atomic<bool> g_gpsStreamDiscontinuity{false};
static uint32_t g_gpsLineOverflows = 0, g_gpsResyncs = 0, g_gpsRxHighWater = 0;
static uint32_t g_gpsInvalidCharacters = 0;
static bool g_gpsDroppingLine = false, g_gpsRxBufferConfigured = false;
static void onGpsReceiveError(hardwareSerial_error_t error) {
  if (error == UART_FIFO_OVF_ERROR) ++g_gpsFifoOverflows;
  else if (error == UART_BUFFER_FULL_ERROR) ++g_gpsRingOverflows;
  else ++g_gpsUartErrors;
  g_gpsStreamDiscontinuity.store(true);
}

static constexpr uint32_t MILLIS_PER_SECOND = 1000u;
static constexpr uint32_t MILLIS_PER_MINUTE = 60u * MILLIS_PER_SECOND;
static constexpr uint32_t MILLIS_PER_HOUR = 60u * MILLIS_PER_MINUTE;
static constexpr uint32_t MILLIS_PER_DAY = 24u * MILLIS_PER_HOUR;

static constexpr const char* PMTK_RMC_GGA_ONLY =
    "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";
static constexpr const char* PMTK_10HZ =
    "$PMTK220,100*2F\r\n";
static constexpr const char* PMTK_115200 =
    "$PMTK251,115200*1F\r\n";

enum class GpsState : uint8_t {
  Probe,
  Configure,
  SwitchTo115200,
  Active,
  RetryWait,
};

static constexpr uint32_t GPS_PROBE_BAUDS[] = {
    115200, 9600, 38400, 57600
};
static constexpr size_t GPS_PROBE_BAUD_COUNT =
    sizeof(GPS_PROBE_BAUDS) / sizeof(GPS_PROBE_BAUDS[0]);

static GpsState g_gpsState = GpsState::Probe;
static size_t g_gpsProbeIndex = 0;
static uint32_t g_gpsCurrentBaud = 115200;
static uint32_t g_gpsStateDeadlineMs = 0;
static uint8_t g_gpsConfigCommandIndex = 0;
static bool g_gpsSentenceSeenAtCurrentBaud = false;
static bool g_gpsConfigured = false;
static bool g_gpsValidFix = false;
static uint32_t g_gpsSentenceCount = 0;
static uint32_t g_gpsParseFailureCount = 0;
static uint32_t g_gpsLastSentenceMs = 0;
static uint32_t g_gpsLastValidFixMs = 0;
static uint32_t g_gpsLastNotifyMs = 0;
static uint32_t g_gpsLastTimeNotifyMs = 0;
static uint8_t g_gpsAntennaStatus = 0; // Unknown until checksum-valid $PCD.
static uint32_t g_gpsAntennaStatusMs = 0;
static uint8_t g_gpsRateAck = 0xFF;
static gps::ModeEvidence g_gpsModeEvidence{};
static uint64_t g_gpsTxDeferred = 0, g_gpsTxShortWrites = 0;

// The loop is the only GPS TX writer. Never submit a partial command merely
// because the UART buffer has some space; defer the complete frame instead.
static bool queueGpsCommand(const char* command) {
  if (!command) return false;
  const size_t count = strlen(command);
  const int available = g_gpsSerial.availableForWrite();
  if (available < 0 || static_cast<size_t>(available) < count) {
    diagnostics::saturatingAdd(g_gpsTxDeferred, 1);
    return false;
  }
  if (g_gpsSerial.write(reinterpret_cast<const uint8_t*>(command), count) != count) {
    diagnostics::saturatingAdd(g_gpsTxShortWrites, 1);
    return false; // Driver contract failure: do not advance configuration.
  }
  return true;
}
static uint8_t g_gps10HzIntervals = 0;
static uint32_t g_gpsLastRmcUtc = 0;
static bool g_gpsHaveRmcCadence = false;
static uint32_t g_gpsRmcSequence = 0;
static uint32_t g_gpsLastForwardedRmcSequence = 0;
static bool g_gpsLastForwardedFixValid = false;
static gps::SolutionEpoch g_gpsSolutionEpoch{};
static uint64_t g_gpsDuplicateRmcEpochs = 0, g_gpsBackwardRmcEpochs = 0;
static uint32_t g_gpsLastGgaMs = 0;
static bool g_gpsDateAvailable = false;
static uint8_t g_gpsConfigAttempts = 0;


static char g_gpsLine[160] = {};
static size_t g_gpsLineLength = 0;

static int g_rmcHour = 0;
static int g_rmcMinute = 0;
static int g_rmcSecond = 0;
static int g_rmcMillis = 0;
static double g_rmcLatitudeDeg = 0.0;
static double g_rmcLongitudeDeg = 0.0;
static double g_rmcSpeedKmh = 0.0;
static double g_rmcCourseDeg = 0.0;

static int g_gpsYear = 2000;
static int g_gpsMonth = 1;
static int g_gpsDay = 1;
static int g_ggaSatellites = 0;
static double g_ggaHdop = 99.9;
static double g_ggaAltitudeMeters = 0.0;

static bool g_rmcTimeAvailable = false;
static uint32_t g_rmcMillisSinceMidnight = 0;
static uint32_t g_rmcCaptureMillis = 0;
static uint32_t g_rmcCaptureMicros = 0;
static uint8_t g_gpsSyncBits = 0;
static int g_lastDateHourPacked = -1;

#if GPS_PPS_GPIO >= 0
static volatile uint32_t g_ppsEventMicros = 0;
static volatile uint32_t g_ppsLastIsrMicros = 0;
static volatile uint32_t g_ppsIntervalMicros = 0;
static volatile uint32_t g_ppsPendingCount = 0;
static uint32_t g_ppsProcessedCount = 0;
static uint32_t g_ppsLastProcessedMs = 0;
static bool g_ppsLocked = false;

static void IRAM_ATTR onGpsPps() {
  const uint32_t now = micros();
  const uint32_t last = g_ppsLastIsrMicros;
  if (last != 0 && static_cast<uint32_t>(now - last) < 200000u) return;

  g_ppsLastIsrMicros = now;
  g_ppsEventMicros = now;
  g_ppsIntervalMicros = last == 0 ? 1000000u
                                  : static_cast<uint32_t>(now - last);
  if (g_ppsPendingCount != 0xFFFFFFFFu) g_ppsPendingCount = g_ppsPendingCount + 1;
}
#endif

static uint32_t millisSinceMidnight(int hour,
                                    int minute,
                                    int second,
                                    int millisPart) {
  uint64_t total =
      static_cast<uint64_t>(hour < 0 ? 0 : hour) * MILLIS_PER_HOUR +
      static_cast<uint64_t>(minute < 0 ? 0 : minute) *
          MILLIS_PER_MINUTE +
      static_cast<uint64_t>(second < 0 ? 0 : second) *
          MILLIS_PER_SECOND +
      static_cast<uint64_t>(millisPart < 0 ? 0 : millisPart);
  return static_cast<uint32_t>(total % MILLIS_PER_DAY);
}

static void servicePps(uint32_t now) {
#if GPS_PPS_GPIO >= 0
  uint32_t pending = 0, interval = 0;
  static uint8_t goodIntervals = 0;
  noInterrupts();
  pending = g_ppsPendingCount;
  interval = g_ppsIntervalMicros;
  g_ppsPendingCount = 0;
  interrupts();
  if (pending) {
    g_ppsProcessedCount += pending;
    g_ppsLastProcessedMs = now;
    if (pending == 1 && interval >= 900000u && interval <= 1100000u) {
      if (goodIntervals < 3) ++goodIntervals;
    } else {
      goodIntervals = 0;
    }
    // This means a fresh plausible 1 Hz pulse stream, not sub-millisecond
    // UTC discipline. Position carries its authoritative RMC solution epoch.
    g_ppsLocked = goodIntervals >= 3 && g_rmcTimeAvailable;
  }
  if (now - g_ppsLastProcessedMs > 2500u) {
    g_ppsLocked = false;
    goodIntervals = 0;
  }
#else
  (void)now;
#endif
}

static void startGpsProbe(size_t index, uint32_t now) {
  if (index >= GPS_PROBE_BAUD_COUNT) index = 0;

  g_gpsConfigured = false;
  g_gpsRateAck = 0xFF;
  g_gpsModeEvidence = gps::ModeEvidence{};
  g_gps10HzIntervals = 0;
  g_gpsHaveRmcCadence = false;
  g_gpsConfigAttempts = 0;
  g_gpsAntennaStatus = 0;
  g_gpsProbeIndex = index;
  g_gpsCurrentBaud = GPS_PROBE_BAUDS[index];
  g_gpsSentenceSeenAtCurrentBaud = false;
  g_gpsLineLength = 0;
  g_gpsSerial.end();
  g_gpsDroppingLine = false;
  g_gpsRxBufferConfigured = g_gpsSerial.setRxBufferSize(2048u) == 2048u;
  g_gpsSerial.setTxBufferSize(256u);
  g_gpsSerial.onReceiveError(onGpsReceiveError);
  g_gpsSerial.begin(g_gpsCurrentBaud, SERIAL_8N1,
                    GPS_RX_GPIO, GPS_TX_GPIO);
  g_gpsState = GpsState::Probe;
  g_gpsStateDeadlineMs = now + 1500u;

  g_log.printf("GPS: probing %lu baud\n",
                static_cast<unsigned long>(g_gpsCurrentBaud));
}

static void enterGpsRetryWait(uint32_t now) {
  g_gpsValidFix = false;
  g_gpsState = GpsState::RetryWait;
  g_gpsStateDeadlineMs = now + 60000u;
  g_gpsConfigured = false;
  g_log.println("GPS: no stream; next probe in 60s");
}

static void parseGpsSentence(const char* line, uint32_t now) {
  if (line == nullptr || line[0] != '$') return;
  uint8_t antenna = 0;
  if (gps::parseAntennaStatus(line, antenna)) {
    g_gpsAntennaStatus = antenna;
    g_gpsAntennaStatusMs = now;
    return;
  }
  uint16_t ackCommand = 0;
  uint8_t ackResult = 0;
  if (gps::parsePmtkAck(line, ackCommand, ackResult)) {
    if (ackCommand == 220) g_gpsRateAck = ackResult;
    g_gpsModeEvidence.observeAck(ackCommand, ackResult);
    if (g_gpsRateAck != 3 || !g_gpsModeEvidence.confirmed()) g_gpsConfigured = false;
    g_log.printf("GPS: PMTK ack command=%u result=%u\n", ackCommand, ackResult);
    return;
  }
  uint8_t sbas = 0;
  if (gps::parseSbasReadback(line, sbas)) {
    g_gpsModeEvidence.sbas = sbas;
    if (!g_gpsModeEvidence.confirmed()) g_gpsConfigured = false;
    return;
  }
  // Ignore other checksum-valid startup/ack/NMEA traffic, count malformed lines.
  if (!gps::checksumOk(line, strlen(line))) {
    ++g_gpsParseFailureCount;
    return;
  }

  char work[sizeof(g_gpsLine)];
  strncpy(work, line, sizeof(work) - 1);
  work[sizeof(work) - 1] = '\0';

  bool parsed = false;

  if (strstr(work, "GPRMC") != nullptr ||
      strstr(work, "GNRMC") != nullptr) {
    gps::RmcData rmc;
    if (gps::parseRmcSentence(work, rmc)) {
      parsed = true;

      const auto epoch = g_gpsSolutionEpoch.observe(rmc);
      if (epoch == gps::EpochResult::Duplicate) {
        diagnostics::saturatingAdd(g_gpsDuplicateRmcEpochs, 1);
        if (!rmc.valid) g_gpsValidFix = false;
        // A duplicate talker/sentence is not a new solution. In particular it
        // cannot renew the 500 ms age of an otherwise stale position.
      } else if (epoch == gps::EpochResult::Backward || epoch == gps::EpochResult::Missing) {
        if (epoch == gps::EpochResult::Backward)
          diagnostics::saturatingAdd(g_gpsBackwardRmcEpochs, 1);
        g_gpsValidFix = false;
        g_gpsConfigured = false;
        g_gps10HzIntervals = 0;
      } else {
        ++g_gpsRmcSequence;
        g_rmcCaptureMillis = now;
        if (rmc.has_time) {
          const uint32_t utc = millisSinceMidnight(rmc.hour, rmc.minute,
                                                  rmc.second, rmc.millis);
          if (g_gpsHaveRmcCadence) {
            const uint32_t step = (utc + MILLIS_PER_DAY - g_gpsLastRmcUtc) % MILLIS_PER_DAY;
            if (step == 100u) {
              if (g_gps10HzIntervals < 10) ++g_gps10HzIntervals;
            } else {
              g_gps10HzIntervals = 0;
            }
            g_gpsConfigured = g_gpsCurrentBaud == 115200u &&
                              g_gps10HzIntervals >= 10 && g_gpsRateAck == 3 &&
                              g_gpsModeEvidence.confirmed();
          }
          g_gpsLastRmcUtc = utc;
          g_gpsHaveRmcCadence = true;
          g_rmcHour = rmc.hour;
          g_rmcMinute = rmc.minute;
          g_rmcSecond = rmc.second;
          g_rmcMillis = rmc.millis;
          g_rmcMillisSinceMidnight = millisSinceMidnight(
              g_rmcHour, g_rmcMinute, g_rmcSecond, g_rmcMillis);
          g_rmcCaptureMillis = now;
          g_rmcCaptureMicros = micros();
          g_rmcTimeAvailable = true;
        }

        g_gpsValidFix = rmc.valid;
        if (rmc.valid) g_gpsLastValidFixMs = now;
        if (rmc.has_latitude) g_rmcLatitudeDeg = rmc.latitude_deg;
        if (rmc.has_longitude) g_rmcLongitudeDeg = rmc.longitude_deg;
        g_rmcSpeedKmh = rmc.speed_kmh;
        g_rmcCourseDeg = rmc.course_deg;

        g_gpsDateAvailable = rmc.has_date;
        if (rmc.has_date) {
          g_gpsDay = rmc.day;
          g_gpsMonth = rmc.month;
          g_gpsYear = rmc.year;
        }
      }
    }
  } else if (strstr(work, "GPGGA") != nullptr ||
             strstr(work, "GNGGA") != nullptr) {
    gps::GgaData gga;
    if (gps::parseGgaSentence(work, gga)) {
      parsed = true;
      g_gpsLastGgaMs = now;
      g_ggaSatellites = gga.has_sats ? gga.sats : 0;
      g_ggaHdop = gga.has_hdop ? gga.hdop : 99.9;
      g_ggaAltitudeMeters =
          gga.has_altitude ? gga.altitude_m : NAN;
    }
  }

  if (!parsed) return;

  ++g_gpsSentenceCount;
  g_gpsLastSentenceMs = now;
  g_gpsSentenceSeenAtCurrentBaud = true;

  // State transitions and whole-command TX occur in the loop-owned state
  // machine. A full TX buffer cannot strand Probe after one valid sentence.

}

static void readGpsBytes(uint32_t now) {
  size_t processed = 0;
  if (g_gpsStreamDiscontinuity.exchange(false)) {
    g_gpsLineLength = 0;
    g_gpsDroppingLine = true;  // Resume only at a fresh '$'.
  }
  const int waiting = g_gpsSerial.available();
  if (waiting > 0) g_gpsRxHighWater = std::max<uint32_t>(g_gpsRxHighWater, waiting);

  while (g_gpsSerial.available() && processed < 512u) {
    ++processed;
    const int raw = g_gpsSerial.read();
    if (raw < 0) break;

    const char c = static_cast<char>(raw);
    if (c == '$') {
      if (g_gpsLineLength != 0) ++g_gpsResyncs;
      g_gpsLineLength = 0;
      g_gpsDroppingLine = false;
    }
    if (g_gpsDroppingLine) continue;
    if (c == '\r') continue;

    if (c == '\n') {
      if (g_gpsLineLength > 0) {
        g_gpsLine[g_gpsLineLength] = '\0';
        parseGpsSentence(g_gpsLine, now);
      }
      g_gpsLineLength = 0;
      continue;
    }

    // NMEA is printable ASCII. Reject a corrupt whole line, including embedded
    // NULs which could otherwise make a valid prefix conceal trailing bytes.
    if (raw < 0x20 || raw > 0x7e) {
      ++g_gpsInvalidCharacters;
      g_gpsLineLength = 0;
      g_gpsDroppingLine = true;
      continue;
    }

    if (g_gpsLineLength < sizeof(g_gpsLine) - 1) {
      g_gpsLine[g_gpsLineLength++] = c;
    } else {
      ++g_gpsLineOverflows;
      g_gpsLineLength = 0;
      g_gpsDroppingLine = true;
    }
  }
}

static void serviceGpsStateMachine(uint32_t now) {
  switch (g_gpsState) {
    case GpsState::Probe:
      if (g_gpsSentenceSeenAtCurrentBaud) {
        if (g_gpsCurrentBaud != 115200u) {
          if (!queueGpsCommand(PMTK_115200)) return;
          g_gpsState = GpsState::SwitchTo115200;
          g_gpsStateDeadlineMs = now + 300u;
        } else {
          g_gpsState = GpsState::Configure;
          g_gpsConfigCommandIndex = 0;
          g_gpsStateDeadlineMs = now;
          g_gpsRateAck = 0xFF;
          g_gpsModeEvidence = gps::ModeEvidence{};
          g_gps10HzIntervals = 0;
          g_gpsHaveRmcCadence = false;
        }
        return;
      }
      if (!timeReached(now, g_gpsStateDeadlineMs)) return;

      if (g_gpsProbeIndex + 1 < GPS_PROBE_BAUD_COUNT) {
        startGpsProbe(g_gpsProbeIndex + 1, now);
      } else {
        enterGpsRetryWait(now);
      }
      return;

    case GpsState::Configure: {
      if (!timeReached(now, g_gpsStateDeadlineMs)) return;

      static constexpr const char* commands[] = {
          gps::kGpsOnly, gps::kSbasOff, gps::kDgpsNone,
          PMTK_RMC_GGA_ONLY, PMTK_10HZ, gps::kSbasQuery, "$CDCMD,9,0*44\r\n"
      };

      if (g_gpsConfigCommandIndex <
          sizeof(commands) / sizeof(commands[0])) {
        if (!queueGpsCommand(commands[g_gpsConfigCommandIndex])) return;
        ++g_gpsConfigCommandIndex;
        g_gpsStateDeadlineMs = now + 150u;
      } else {
        g_gpsState = GpsState::Active;
        g_gpsStateDeadlineMs = now + 10000u;
        ++g_gpsConfigAttempts;
        g_log.println("GPS: configuration sent; configured=yes requires positive mode/rate ACKs, SBAS-off readback and observed 10 Hz UTC");
      }
      return;
    }

    case GpsState::SwitchTo115200:
      if (!timeReached(now, g_gpsStateDeadlineMs)) return;
      startGpsProbe(0, now);
      return;

    case GpsState::Active:
      if (!g_gpsConfigured && timeReached(now, g_gpsStateDeadlineMs) &&
          g_gpsConfigAttempts < 3) {
        g_gpsState = GpsState::Configure;
        g_gpsConfigCommandIndex = 0;
        g_gpsStateDeadlineMs = now;
        g_gpsRateAck = 0xFF;
        g_gpsModeEvidence = gps::ModeEvidence{};
        g_gps10HzIntervals = 0;
        g_gpsHaveRmcCadence = false;
      }
      if (g_gpsLastSentenceMs != 0 &&
          now - g_gpsLastSentenceMs > 5000u) {
        g_gpsConfigured = false;
        g_gpsValidFix = false;
        g_gpsState = GpsState::RetryWait;
        g_gpsStateDeadlineMs = now + 10000u;
        g_log.println("GPS: stream stale; reprobe scheduled");
      }
      return;

    case GpsState::RetryWait:
      if (timeReached(now, g_gpsStateDeadlineMs)) {
        startGpsProbe(0, now);
      }
      return;
  }
}

static int clampInt(int value, int low, int high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

static void serviceGpsNotifications(uint32_t now) {
  if (!g_gpsCharacteristic || !g_gpsTimeCharacteristic) return;
  const bool rmcFresh = g_gpsRmcSequence && now - g_rmcCaptureMillis <= 500u;
  const bool ggaFresh = g_gpsLastGgaMs && now - g_gpsLastGgaMs <= 500u;
  const bool validFix = g_gpsValidFix && rmcFresh;
  // Position is timestamped with its actual NMEA solution epoch. UART receipt
  // latency/PPS extrapolation must not move an old position into the future.
  const uint32_t utcMillis = g_rmcMillisSinceMidnight;
  const uint32_t millisIntoHour = utcMillis % MILLIS_PER_HOUR;
  const int hour = static_cast<int>((utcMillis / MILLIS_PER_HOUR) % 24u);
  const int dateHour = g_gpsDateAvailable ?
      (g_gpsYear - 2000) * 8928 + (g_gpsMonth - 1) * 744 +
      (g_gpsDay - 1) * 24 + hour : 0;
  const bool dateChanged = dateHour != g_lastDateHourPacked;
  if (dateChanged) {
    g_lastDateHourPacked = dateHour;
    g_gpsSyncBits = static_cast<uint8_t>((g_gpsSyncBits + 1) & 7);
  }
  uint8_t timePayload[3] = {
      static_cast<uint8_t>((g_gpsSyncBits << 5) | ((dateHour >> 16) & 0x1F)),
      static_cast<uint8_t>(dateHour >> 8), static_cast<uint8_t>(dateHour)};
  // RaceChrono READ-polls this value; update regardless of subscription.
  if (dateChanged) g_gpsTimeCharacteristic->setValue(timePayload, sizeof(timePayload));
  if (g_gpsTimeSubscribed && now - g_gpsLastTimeNotifyMs >= 1000u) {
    g_gpsLastTimeNotifyMs = now;
    notifyCharacteristic(g_gpsTimeCharacteristic, true, timePayload, 3, now);
  }

  const bool newSolution = g_gpsRmcSequence != g_gpsLastForwardedRmcSequence;
  const bool validityChanged = validFix != g_gpsLastForwardedFixValid;
  if (!newSolution && !validityChanged && now - g_gpsLastNotifyMs < 1000u) return;
  uint8_t payload[20];
  memset(payload, 0xFF, sizeof(payload));
  const uint32_t fineTime = millisIntoHour / 2u;
  payload[0] = static_cast<uint8_t>((g_gpsSyncBits << 5) | ((fineTime >> 16) & 0x1F));
  payload[1] = static_cast<uint8_t>(fineTime >> 8);
  payload[2] = static_cast<uint8_t>(fineTime);
  payload[3] = static_cast<uint8_t>((validFix ? 0x40 : 0) |
      (ggaFresh ? clampInt(g_ggaSatellites, 0, 62) : 0x3F));
  const int32_t lat = validFix ? static_cast<int32_t>(lround(g_rmcLatitudeDeg * 1e7)) : 0x7FFFFFFF;
  const int32_t lon = validFix ? static_cast<int32_t>(lround(g_rmcLongitudeDeg * 1e7)) : 0x7FFFFFFF;
  for (int i = 0; i < 4; ++i) {
    payload[4 + i] = static_cast<uint8_t>(static_cast<uint32_t>(lat) >> (24 - i * 8));
    payload[8 + i] = static_cast<uint8_t>(static_cast<uint32_t>(lon) >> (24 - i * 8));
  }
  const uint16_t altitude = validFix && ggaFresh ? racechrono::altitudeWord(g_ggaAltitudeMeters) : 0xFFFF;
  const uint16_t speed = validFix ? racechrono::speedWord(g_rmcSpeedKmh) : 0xFFFF;
  const uint16_t bearing = validFix ? racechrono::bearingWord(g_rmcCourseDeg) : 0xFFFF;
  payload[12] = altitude >> 8; payload[13] = altitude;
  payload[14] = speed >> 8; payload[15] = speed;
  payload[16] = bearing >> 8; payload[17] = bearing;
  if (ggaFresh && g_ggaHdop >= 0 && g_ggaHdop <= 25.4)
    payload[18] = static_cast<uint8_t>(lround(g_ggaHdop * 10));
  g_gpsCharacteristic->setValue(payload, sizeof(payload));
  if (!g_gpsSubscribed || notifyCharacteristic(g_gpsCharacteristic, true, payload, 20, now)) {
    g_gpsLastNotifyMs = now;
    g_gpsLastForwardedRmcSequence = g_gpsRmcSequence;
    g_gpsLastForwardedFixValid = validFix;
  }
}

static void serviceGps(uint32_t now) {
  readGpsBytes(now);
  serviceGpsStateMachine(now);
  if (!g_gpsRmcSequence || now - g_rmcCaptureMillis > 500u) {
    g_gpsValidFix = false;
    g_gpsConfigured = false;
    g_gps10HzIntervals = 0;
  }
  servicePps(now);
  serviceGpsNotifications(now);
}

// -----------------------------------------------------------------------------
// Oil pressure ADC / virtual CAN 0x710
// -----------------------------------------------------------------------------

// 100 Hz acquisition, alpha 0.5; response/alias models bind this exact source.
static constexpr uint32_t OIL_SAMPLE_PERIOD_MS = 10u;
static constexpr uint32_t OIL_SERVICE_LIMIT_US = 5000u;
static constexpr float ADC_IIR_ALPHA = 0.5f;
static oil::Calibration g_oilCalibration{};
static float g_oilRawAdcVolts = 0;
static float g_oilExcitationAdcVolts = 0;
static float g_oilFilteredVolts = 0; // Latest raw median for diagnostics only.
static float g_oilPsi = NAN;
static uint8_t g_oilFlags = 1u << 5;
static bool g_oilHaveValidSample = false;
static timing::PeriodicSchedule g_oilSampleSchedule, g_oilPublishSchedule;
static uint64_t g_oilSkippedSamples = 0, g_oilSkippedPublishes = 0;
static oil::Reading g_oilReading{};
static oil::FaultRecovery g_oilFaultRecovery{};
static timing::ServiceGapMonitor g_oilServiceGap{OIL_SERVICE_LIMIT_US};

static float medianAdcVolts(int pin) {
  (void)analogReadMilliVolts(pin);
  // A discarded conversion reduces cross-channel history; this delay is a
  // scheduling allowance, NOT a guarantee of the SAR internal acquisition time.
  delayMicroseconds(40);
  float samples[5];
  for (size_t i = 0; i < 5; ++i)
    samples[i] = analogReadMilliVolts(pin) / 1000.0f;
  for (size_t i = 1; i < 5; ++i) {
    float value = samples[i]; int j = static_cast<int>(i) - 1;
    while (j >= 0 && samples[j] > value) { samples[j + 1] = samples[j]; --j; }
    samples[j + 1] = value;
  }
  return samples[2];
}
static void sampleOil() {
  g_oilExcitationAdcVolts = medianAdcVolts(OIL_EXC_ADC_PIN);
  g_oilRawAdcVolts = medianAdcVolts(OIL_ADC_PIN);
  g_oilFilteredVolts = g_oilRawAdcVolts;
  g_oilReading = g_oilFaultRecovery.apply(
      oil::evaluate(g_oilRawAdcVolts, g_oilExcitationAdcVolts, g_oilCalibration));
  g_oilFlags = g_oilReading.flags; // Immediate fault entry, qualified recovery; both bypass the IIR.
}
static void loadOilCalibration() {
  g_oilCalibration = oil::Calibration{};
  if (!oil::load(g_oilCalibration))
    g_log.println("Oil calibration absent/incompatible/CRC-or-device mismatch; commissioning required.");
  g_oilHaveValidSample = false;
}
static void showOilCalibration() {
  const uint64_t device = ESP.getEfuseMac();
  g_log.printf("Device base MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
      static_cast<unsigned>(device & 0xFF), static_cast<unsigned>((device >> 8) & 0xFF),
      static_cast<unsigned>((device >> 16) & 0xFF), static_cast<unsigned>((device >> 24) & 0xFF),
      static_cast<unsigned>((device >> 32) & 0xFF), static_cast<unsigned>((device >> 40) & 0xFF));
  g_log.printf("Oil calibration device binding: %s\n",
                oil::identityMatches(g_oilCalibration, device) ? "matches" : "absent/mismatch");
  g_log.printf("Pressure reference: %s\n", g_oilCalibration.pressurePoints == 0 ?
      "Honeywell nominal0.1/0.9; NOT measured; sensor TEB remains" :
      (g_oilCalibration.pressurePoints == 3 ? "measured endpoint pair" : "incomplete endpoint calibration"));
  g_log.printf("Oil model v4: ready=%s r0=%.7f r150=%.7f sensor_points=%lu\n",
                oil::ready(g_oilCalibration) ? "yes" : "no", g_oilCalibration.ratio0,
                g_oilCalibration.ratio150, static_cast<unsigned long>(g_oilCalibration.pressurePoints));
  for (int i = 0; i < 2; ++i) {
    const auto& c = i ? g_oilCalibration.excitation : g_oilCalibration.signal;
    g_log.printf("  %s points=%lu ADC %.6f/%.6fV -> connector %.6f/%.6fV E=%.6fV verified=%s\n",
                  i ? "EXC" : "SIG", static_cast<unsigned long>(c.points), c.raw[0], c.raw[1],
                  c.volts[0], c.volts[1], c.maxError, oil::verified(c) ? "yes" : "no");
  }
}
static bool oilCalibrationCommand(const char* input) {
  if (!strcmp(input, "CAL SHOW") || !strcmp(input, "CAL ADC SHOW")) {
    showOilCalibration(); return true;
  }
  char channel[4] = {}; unsigned point = 0; float known = 0; int end = 0;
  if (sscanf(input, "CAL ADC %3s %u %f%n", channel, &point, &known, &end) == 3 && input[end] == 0) {
    if ((strcmp(channel, "SIG") && strcmp(channel, "EXC")) || point > 1 ||
        !isfinite(known) || known < 0 || known > 5.5f) {
      g_log.println("Usage: CAL ADC SIG|EXC 0|1 <measured connector volts>"); return true;
    }
    auto& c = !strcmp(channel, "SIG") ? g_oilCalibration.signal : g_oilCalibration.excitation;
    c.raw[point] = medianAdcVolts(!strcmp(channel, "SIG") ? OIL_ADC_PIN : OIL_EXC_ADC_PIN);
    c.volts[point] = known;
    c.points |= 1u << point;
    c.maxError = -1; // Refit requires an independent sweep before verification.
    g_oilCalibration.pressurePoints = 0;
    g_oilCalibration.ratio0 = 0.1f;
    g_oilCalibration.ratio150 = 0.9f;
    g_log.println(oil::save(g_oilCalibration) ? "ADC point saved; sweep and CAL VERIFY required" : "ADC save FAILED");
    return true;
  }
  end = 0;
  if (sscanf(input, "CAL VERIFY %3s %f%n", channel, &known, &end) == 2 && input[end] == 0) {
    if ((strcmp(channel, "SIG") && strcmp(channel, "EXC")) || !isfinite(known) || known < 0.001f || known > 0.025f) {
      g_log.println("CAL VERIFY SIG|EXC <measured worst-case connector error 0.001..0.025V>"); return true;
    }
    auto& c = !strcmp(channel, "SIG") ? g_oilCalibration.signal : g_oilCalibration.excitation;
    if (!oil::fitted(c)) { g_log.println("Refused: two valid ADC points required"); return true; }
    c.maxError = known;
    g_log.println(oil::save(g_oilCalibration) ? "ADC measured error bound saved" : "ADC save FAILED");
    return true;
  }
  if (!strcmp(input, "CAL 0") || !strcmp(input, "CAL 1")) {
    if (!oil::verified(g_oilCalibration.signal) || !oil::verified(g_oilCalibration.excitation)) {
      g_log.println("Refused: independently calibrate and verify both ADC channels first"); return true;
    }
    sampleOil();
    if (g_oilReading.flags & ~(1u << 5)) { g_log.println("Refused: raw signal/excitation fault"); return true; }
    const bool zero = !strcmp(input, "CAL 0");
    const float ratio = g_oilReading.ratio;
    if (!isfinite(ratio) || (zero ? (ratio < oil::kCalibrationZeroMin || ratio > oil::kCalibrationZeroMax) : (ratio < oil::kCalibrationFullMin || ratio > oil::kCalibrationFullMax))) {
      g_log.println("Refused: apply known sealed-gauge 0/150 psi (14.7/164.7 psia), never auto-zero at key-on"); return true;
    }
    if (zero) g_oilCalibration.ratio0 = ratio; else g_oilCalibration.ratio150 = ratio;
    g_oilCalibration.pressurePoints |= zero ? 1u : 2u;
    g_oilHaveValidSample = false;
    g_log.println(oil::save(g_oilCalibration) ? "Pressure ratio endpoint saved" : "Pressure save FAILED");
    return true;
  }
  return false;
}
static void invalidateOilTiming() {
  oil::Reading fault = g_oilReading;
  fault.flags |= 1u << 7;
  fault.psi = NAN;
  g_oilReading = g_oilFaultRecovery.apply(fault);
  g_oilFlags = g_oilReading.flags;
  g_oilPsi = NAN;
  g_oilHaveValidSample = false;
}

static void serviceOil(uint32_t now) {
  const uint32_t entryUs = micros();
  now = millis(); // Callers must not publish an old pre-service timestamp.
  bool timingFault = g_oilServiceGap.observe(entryUs);
  if (timingFault) invalidateOilTiming();
  uint32_t skipped = 0;
  if (g_oilSampleSchedule.due(now, OIL_SAMPLE_PERIOD_MS, skipped)) {
    diagnostics::saturatingAdd(g_oilSkippedSamples, skipped);
    sampleOil();
    // Detect a slow acquisition before filtering or forwarding its value.
    if (g_oilServiceGap.recordDuration(micros() - entryUs)) {
      timingFault = true;
      invalidateOilTiming();
    }
    now = millis();
    if (g_oilFlags) {
      g_oilPsi = NAN;
      g_oilHaveValidSample = false;
    } else {
      g_oilPsi = g_oilHaveValidSample ?
          g_oilPsi + ADC_IIR_ALPHA * (g_oilReading.psi - g_oilPsi) : g_oilReading.psi;
      g_oilHaveValidSample = true;
    }
  }
  const bool due = g_oilPublishSchedule.due(now, g_oilPublishPeriodMs, skipped);
  diagnostics::saturatingAdd(g_oilSkippedPublishes, skipped);
  if (due || timingFault) {
    const uint16_t pressureTenths = g_oilFlags || !isfinite(g_oilPsi) ? 0xFFFF :
        static_cast<uint16_t>(clampFloat(g_oilPsi * 10, 0, 1500) + 0.5f);
    uint8_t payload[8] = {static_cast<uint8_t>(pressureTenths >> 8),
                         static_cast<uint8_t>(pressureTenths), g_oilFlags, 0, 0, 0, 0, 0};
    publishVirtualCan(0x710u, payload, sizeof(payload), now);
  }
}

// -----------------------------------------------------------------------------
// Low-rate diagnostics virtual CAN frame
// -----------------------------------------------------------------------------

static uint32_t g_lastDiagnosticPublishMs = 0;

static void publishDiagnostics(uint32_t now) {
  if (now - g_lastDiagnosticPublishMs < 2000u) return;
  g_lastDiagnosticPublishMs = now;

  twai_status_info_t status = g_lastTwaiStatus;
  if (g_canRunning) {
    twai_status_info_t fresh = {};
    if (twai_get_status_info(&fresh) == ESP_OK) {
      status = fresh;
      updateCanStatus(fresh);
    }
  }

  const uint32_t freeHeap =
      heap_caps_get_free_size(MALLOC_CAP_8BIT);
  uint8_t heapKiB =
      static_cast<uint8_t>((freeHeap / 1024u) > 255u
                               ? 255u
                               : (freeHeap / 1024u));

  uint8_t payload[8] = {
      static_cast<uint8_t>(
          (2u << 4) |
          (g_canRunning
               ? (static_cast<uint8_t>(status.state) & 0x0Fu)
               : 0u)),
      static_cast<uint8_t>(status.tx_error_counter & 0xFFu),
      static_cast<uint8_t>(status.rx_error_counter & 0xFFu),
      static_cast<uint8_t>(status.rx_missed_count & 0xFFu),
      static_cast<uint8_t>(g_bleNotifyFailures & 0xFFu),
      static_cast<uint8_t>(g_canRestartCount & 0xFFu),
      heapKiB,
      static_cast<uint8_t>(g_bootResetReason)
  };

  publishVirtualCan(0x777u, payload, sizeof(payload), now);
}

// -----------------------------------------------------------------------------
// LED status
// -----------------------------------------------------------------------------

static uint32_t g_lastLedServiceMs = 0;

static void serviceStatusLeds(uint32_t now) {
  if (now - g_lastLedServiceMs < 20u) return;
  g_lastLedServiceMs = now;

  // Power is deliberately never changed after setup. If this LED physically
  // dims while Board3v3 and EN also droop, that is an electrical problem.
  led_set_power(LedPattern::Solid);

  led_set_ble(isBleConnected() ? LedPattern::Solid
                               : LedPattern::BlinkFast);

  if (!g_canRunning) {
    led_set_can(LedPattern::BlinkFast);
  } else if (g_lastCanFrameMs != 0 &&
             now - g_lastCanFrameMs <= 1000u) {
    led_set_can(LedPattern::Pulse2Every2s);
  } else {
    led_set_can(LedPattern::BlinkSlow);
  }

  if (g_gpsLastSentenceMs != 0 &&
      now - g_gpsLastSentenceMs <= 1500u) {
    led_set_gps(g_gpsValidFix ? LedPattern::Pulse3Every2s
                              : LedPattern::Solid);
  } else {
    led_set_gps(LedPattern::BlinkSlow);
  }

  const bool systemFault =
      (!g_canRunning && g_canStartFailures > 0) ||
      (g_lastBleNotifyFailureMs != 0 &&
       now - g_lastBleNotifyFailureMs < 2000u);
  led_set_sys(systemFault ? LedPattern::BlinkSlow
                          : LedPattern::Off);

  if (g_oilFlags == 0) {
    led_set_oil(LedPattern::Solid);
  } else if ((g_oilFlags & (1u << 1)) != 0) {
    led_set_oil(LedPattern::BlinkFast);
  } else if ((g_oilFlags & (1u << 0)) != 0) {
    led_set_oil(LedPattern::Pulse2Every2s);
  } else {
    led_set_oil(LedPattern::BlinkSlow);
  }

  led_service(now);
}

// -----------------------------------------------------------------------------
// Serial CLI
// -----------------------------------------------------------------------------

static void showConfig() {
  g_log.println("=== CCA Config ===");
  g_log.println("Rev B: listen-only CAN; GPIO15 PWR /GPIO9 OIL; UART0 debug.");
  g_log.printf("Excitation ADC: %.4f V (nominal2.50244V; calibrated ratio only when verified)\n", g_oilExcitationAdcVolts);
  g_log.printf("Build:          %s\n", BUILD_ID);
  g_log.printf("Device:         %s\n", DEVICE_NAME);
  g_log.printf("Profile:        %s\n",
                g_profileEnabled ? "GR86 allow-list" : "sniff-all");
  showOilCalibration();
  g_log.printf("Oil rate:       %u ms\n",
                static_cast<unsigned>(g_oilPublishPeriodMs));
  g_log.printf("Custom divs:    %u\n",
                static_cast<unsigned>(g_customDividerCount));
  g_log.println("==================");
}

static void showStats() {
  const uint32_t now = millis();
  g_log.println("=== CCA Stats ===");
  g_log.printf("Oil timing: acquisition=%lums publish=%ums max_service_or_acquisition_us=%lu violations=%llu (fault bit7)\n",
      static_cast<unsigned long>(OIL_SAMPLE_PERIOD_MS), g_oilPublishPeriodMs,
      static_cast<unsigned long>(g_oilServiceGap.maximumUs()),
      static_cast<unsigned long long>(g_oilServiceGap.violations()));
  g_log.println("Oil 50ms-event model requires RATE <=20ms, client interval <=20ms, accepting transport and service <=5ms; saved slower RATE is retained.");
  g_log.printf("GPS commands: deferred=%llu short_writes=%llu GPS-only_ACK=%u SBAS-off_ACK=%u DGPS-none_ACK=%u SBAS_readback=%u\n",
      static_cast<unsigned long long>(g_gpsTxDeferred), static_cast<unsigned long long>(g_gpsTxShortWrites),
      g_gpsModeEvidence.ack353, g_gpsModeEvidence.ack313, g_gpsModeEvidence.ack301, g_gpsModeEvidence.sbas);
  g_log.printf("IO budget: log_queued=%u log_high=%u log_dropped_bytes=%llu max_loop_gap_us=%lu cli_commands=%lu\n",
      static_cast<unsigned>(g_log.queued()), static_cast<unsigned>(g_log.highWater()),
      static_cast<unsigned long long>(g_log.dropped()), static_cast<unsigned long>(g_maxLoopGapUs),
      static_cast<unsigned long>(g_cliCommands));
  g_log.printf("Acquisition: can_queue_observed_high=%lu cache_coalesced=%lu gps_rx_high=%lu gps_rx2048=%s\n",
      static_cast<unsigned long>(g_canQueueHighWater), static_cast<unsigned long>(g_canCacheCoalesced),
      static_cast<unsigned long>(g_gpsRxHighWater), g_gpsRxBufferConfigured ? "yes" : "NO");
  g_log.printf("CAN routing: filtered_before_cache=%lu\n", static_cast<unsigned long>(g_canFilteredCount));
  g_log.printf("GPS loss: fifo=%lu ring=%lu uart=%lu line=%lu resync=%lu invalid_char=%lu\n",
      static_cast<unsigned long>(g_gpsFifoOverflows.load()), static_cast<unsigned long>(g_gpsRingOverflows.load()),
      static_cast<unsigned long>(g_gpsUartErrors.load()), static_cast<unsigned long>(g_gpsLineOverflows),
      static_cast<unsigned long>(g_gpsResyncs), static_cast<unsigned long>(g_gpsInvalidCharacters));

  g_log.printf("Uptime:         %lu ms\n",
                static_cast<unsigned long>(now));
  g_log.printf("Boot reason:    %s (%d)\n",
                resetReasonName(g_bootResetReason),
                static_cast<int>(g_bootResetReason));
  g_log.printf("RTC boot sequence: %lu (resets with power-on or lost RTC record)\n",
                static_cast<unsigned long>(g_retainedBootRecord.count));

  g_log.printf(
      "BLE: connected=%s can_sub=%s gps_sub=%s time_sub=%s mtu=%u\n",
      isBleConnected() ? "yes" : "no",
      g_canSubscribed ? "yes" : "no",
      g_gpsSubscribed ? "yes" : "no",
      g_gpsTimeSubscribed ? "yes" : "no",
      static_cast<unsigned>(g_bleMtu));
  g_log.printf(
      "BLE events: connect=%lu disconnect=%lu last_reason=%d adv=%lu/%lu\n",
      static_cast<unsigned long>(g_bleConnectCount),
      static_cast<unsigned long>(g_bleDisconnectCount),
      g_lastBleDisconnectReason,
      static_cast<unsigned long>(g_advertisingStartCount),
      static_cast<unsigned long>(g_advertisingStartFailures));
  g_log.printf(
      "BLE notify: ok=%lu fail=%lu rate_drop=%lu unsub_drop=%lu\n",
      static_cast<unsigned long>(g_bleNotifySuccesses),
      static_cast<unsigned long>(g_bleNotifyFailures),
      static_cast<unsigned long>(g_bleNotifyRateDrops),
      static_cast<unsigned long>(g_bleNotifyUnsubscribedDrops));
  g_log.printf(
      "FIL: active=%s allow_all=%s requested=%u commands=%lu\n",
      g_raceChronoFilterActive ? "yes" : "no",
      g_raceChronoAllowAll ? "yes" : "no",
      static_cast<unsigned>(requestedPidCount()),
      static_cast<unsigned long>(g_filterCommandCount));

  g_log.printf(
      "CAN: running=%s rx=%lu ext=%lu rtr=%lu last_age=%lums\n",
      g_canRunning ? "yes" : "no",
      static_cast<unsigned long>(g_canRxCount),
      static_cast<unsigned long>(g_canExtendedCount),
      static_cast<unsigned long>(g_canRtrCount),
      static_cast<unsigned long>(
          g_lastCanFrameMs == 0 ? 0 : now - g_lastCanFrameMs));
  g_log.printf(
      "CAN recovery: starts=%lu failures=%lu restarts=%lu busoff=%lu faults=%lu\n",
      static_cast<unsigned long>(g_canStartAttempts),
      static_cast<unsigned long>(g_canStartFailures),
      static_cast<unsigned long>(g_canRestartCount),
      static_cast<unsigned long>(g_canBusOffCount),
      static_cast<unsigned long>(g_canFaultCount));
  g_log.printf(
      "CAN loss since boot: invalid_dlc=%lu queue_full_alerts=%lu missed=%llu fifo_overrun=%llu restart_discarded=%llu slot_evict=%lu status_read_fail=%lu\n",
      static_cast<unsigned long>(g_canInvalidDlcCount),
      static_cast<unsigned long>(g_canRxQueueFullCount),
      static_cast<unsigned long long>(g_canRxMissedCount.total()),
      static_cast<unsigned long long>(g_canRxOverrunCount.total()),
      static_cast<unsigned long long>(g_canRxDiscardedCount),
      static_cast<unsigned long>(g_canSlotEvictions),
      static_cast<unsigned long>(g_canStatusReadFailures));
  g_log.printf(
      "TWAI: state=%u TEC=%u REC=%u rx_q=%u tx_q=%u\n",
      static_cast<unsigned>(g_lastTwaiStatus.state),
      static_cast<unsigned>(g_lastTwaiStatus.tx_error_counter),
      static_cast<unsigned>(g_lastTwaiStatus.rx_error_counter),
      static_cast<unsigned>(g_lastTwaiStatus.msgs_to_rx),
      static_cast<unsigned>(g_lastTwaiStatus.msgs_to_tx));

  g_log.printf(
      "GPS: configured=%s fix=%s baud=%lu sentences=%lu parse_fail=%lu age=%lums\n",
      g_gpsConfigured ? "yes" : "no",
      g_gpsValidFix ? "yes" : "no",
      static_cast<unsigned long>(g_gpsCurrentBaud),
      static_cast<unsigned long>(g_gpsSentenceCount),
      static_cast<unsigned long>(g_gpsParseFailureCount),
      static_cast<unsigned long>(
          g_gpsLastSentenceMs == 0 ? 0 : now - g_gpsLastSentenceMs));
  g_log.printf("GPS rate: ack220=%u (3=accepted/255=unseen) consecutive100ms=%u\n",
                g_gpsRateAck, g_gps10HzIntervals);
  g_log.printf("GPS epoch: duplicate=%llu backward=%llu accepted_solution_age=%lums\n",
                static_cast<unsigned long long>(g_gpsDuplicateRmcEpochs),
                static_cast<unsigned long long>(g_gpsBackwardRmcEpochs),
                static_cast<unsigned long>(g_gpsRmcSequence ? now - g_rmcCaptureMillis : 0u));
  g_log.printf("GPS antenna: status=%u (0=unknown/1=internal/2=external/3=short) age=%lums\n",
                g_gpsAntennaStatus,
                static_cast<unsigned long>(g_gpsAntennaStatus ? now - g_gpsAntennaStatusMs : 0));
  g_log.printf("GPS solution:   sats=%d hdop=%.1f lat=%.7f lon=%.7f\n",
                g_ggaSatellites, g_ggaHdop,
                g_rmcLatitudeDeg, g_rmcLongitudeDeg);
#if GPS_PPS_GPIO >= 0
  g_log.printf("PPS:            count=%lu pulse_valid=%s age=%lums\n",
                static_cast<unsigned long>(g_ppsProcessedCount),
                g_ppsLocked ? "yes" : "no",
                static_cast<unsigned long>(
                    g_ppsLastProcessedMs == 0
                        ? 0
                        : now - g_ppsLastProcessedMs));
#else
  g_log.println("PPS:            disabled");
#endif

  g_log.printf("Oil deadlines since boot: skipped_samples=%llu skipped_publishes=%llu\n",
                static_cast<unsigned long long>(g_oilSkippedSamples),
                static_cast<unsigned long long>(g_oilSkippedPublishes));
  g_log.printf("Oil:            %.4f V %.1f psi flags=0x%02X\n",
                g_oilFilteredVolts, g_oilPsi,
                static_cast<unsigned>(g_oilFlags));
  g_log.printf("BLE event drops: %lu\n", static_cast<unsigned long>(g_bleEventDrops.load()));
  g_log.printf("Reserved PID collisions: %lu\n", static_cast<unsigned long>(g_virtualIdCollisionCount));
  g_log.printf("CAN stale cache drops: %lu (max age %lums)\n",
                static_cast<unsigned long>(g_canStaleDropCount),
                static_cast<unsigned long>(CAN_MAX_CACHE_AGE_MS));
  g_log.printf("Oil connector: signal=%.6fV excitation=%.6fV ratio=%.7f\n",
                g_oilReading.signal, g_oilReading.excitation, g_oilReading.ratio);
  g_log.printf("Heap:           free=%u min=%u bytes\n",
                static_cast<unsigned>(
                    heap_caps_get_free_size(MALLOC_CAP_8BIT)),
                static_cast<unsigned>(
                    heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT)));
  g_log.println("=================");
}

static void showMap() {
  g_log.println("=== Routing ===");
  g_log.printf("Profile mode: %s\n",
                g_profileEnabled ? "ON" : "OFF");
  g_log.printf("RaceChrono filter: active=%s allow_all=%s requested=%u\n",
                g_raceChronoFilterActive ? "yes" : "no",
                g_raceChronoAllowAll ? "yes" : "no",
                static_cast<unsigned>(requestedPidCount()));

  if (g_raceChronoFilterActive && !g_raceChronoAllowAll) {
    size_t printed = 0;
    for (size_t i = 0;
         i < MAX_REQUESTED_PIDS && printed < 64;
         ++i) {
      if (!g_requestedPids[i].used) continue;
      g_log.printf("  RC 0x%03lX @ %u ms\n",
                    static_cast<unsigned long>(
                        g_requestedPids[i].pid),
                    static_cast<unsigned>(
                        g_requestedPids[i].intervalMs));
      ++printed;
    }
  }

  for (uint16_t i = 0; i < g_customDividerCount; ++i) {
    g_log.printf("  CLI 0x%03X div=%u\n",
                  g_customDividers[i].pid,
                  static_cast<unsigned>(
                      g_customDividers[i].divider));
  }

  for (size_t i = 0; i < g_deniedPidCount; ++i) {
    g_log.printf("  DENY 0x%03lX\n",
                  static_cast<unsigned long>(g_deniedPids[i]));
  }
  g_log.println("===============");
}

static bool parsePidAndNumber(const char* input,
                              uint32_t* pid,
                              uint32_t* number) {
  if (input == nullptr || pid == nullptr || number == nullptr) {
    return false;
  }

  char* end = nullptr;
  const unsigned long parsedPid = strtoul(input, &end, 0);
  if (end == input) return false;
  while (*end == ' ' || *end == '\t') ++end;
  if (*end == '\0') return false;

  char* numberEnd = nullptr;
  const unsigned long parsedNumber = strtoul(end, &numberEnd, 0);
  if (numberEnd == end) return false;
  while (*numberEnd == ' ' || *numberEnd == '\t') ++numberEnd;
  if (*numberEnd != '\0') return false;

  *pid = static_cast<uint32_t>(parsedPid);
  *number = static_cast<uint32_t>(parsedNumber);
  return true;
}

static void processCliLine(char* line) {
  if (line == nullptr) return;

  while (*line == ' ' || *line == '\t') ++line;
  size_t length = strlen(line);
  while (length > 0 &&
         (line[length - 1] == ' ' || line[length - 1] == '\t')) {
    line[--length] = '\0';
  }
  if (length == 0) return;

  String command(line);
  command.toUpperCase();

  if (command == "SHOW" || command == "SHOW CFG") {
    showConfig();
    return;
  }
  if (command == "SHOW STATS") {
    showStats();
    return;
  }
  if (command == "SHOW MAP" || command == "SHOW DENY") {
    showMap();
    return;
  }
  if (command == "BLE STATUS") {
    g_log.printf(
        "BLE connected=%s advertising=%s CAN_sub=%s GPS_sub=%s TIME_sub=%s\n",
        isBleConnected() ? "yes" : "no",
        (g_bleAdvertising != nullptr &&
         g_bleAdvertising->isAdvertising())
            ? "yes"
            : "no",
        g_canSubscribed ? "yes" : "no",
        g_gpsSubscribed ? "yes" : "no",
        g_gpsTimeSubscribed ? "yes" : "no");
    return;
  }
  if (command == "CAN RESTART") {
    scheduleCanRestart(millis(), "operator command");
    g_canNextStartMs = millis() + 50u;
    return;
  }
  if (command == "PROFILE ON") {
    g_profileEnabled = true;
    g_log.println("Profile=GR86 allow-list (not saved)");
    return;
  }
  if (command == "PROFILE OFF") {
    g_profileEnabled = false;
    g_log.println("Profile=sniff-all (not saved)");
    return;
  }
  if (command == "CLEAR FILTERS") {
    clearRaceChronoFilter();
    clearDeniedPids();
    g_log.println("Runtime RaceChrono/deny filters cleared");
    return;
  }

  if (command.startsWith("RATE ")) {
    const uint32_t rate = static_cast<uint32_t>(
        strtoul(line + 5, nullptr, 0));
    if (rate < 10u || rate > 2000u) {
      g_log.println("RATE range is 10..2000 ms");
    } else {
      g_oilPublishPeriodMs = static_cast<uint16_t>(rate);
      if (rate > 20u) g_log.println("RATE >20ms is outside the 50ms oil-event model allocation.");
      g_log.printf("Oil rate=%u ms (not saved)\n",
                    static_cast<unsigned>(g_oilPublishPeriodMs));
    }
    return;
  }

  if (command.startsWith("ALLOW ")) {
    uint32_t pid = 0;
    uint32_t divider = 0;
    if (!parsePidAndNumber(line + 6, &pid, &divider) ||
        pid > 0x7FFu || divider < 1u || divider > 255u) {
      g_log.println("Usage: ALLOW <0x000..0x7FF> <1..255>");
      return;
    }

    undenyPid(pid);
    if (!setCustomDivider(pid, static_cast<uint8_t>(divider))) {
      g_log.println("ALLOW failed: custom divider table full");
    } else {
      g_log.printf("ALLOW 0x%03lX div=%lu (not saved)\n",
                    static_cast<unsigned long>(pid),
                    static_cast<unsigned long>(divider));
    }
    return;
  }

  if (command.startsWith("DENY ")) {
    char* end = nullptr;
    const uint32_t pid =
        static_cast<uint32_t>(strtoul(line + 5, &end, 0));
    while (end != nullptr && (*end == ' ' || *end == '\t')) ++end;
    if (end == line + 5 || (end != nullptr && *end != '\0') ||
        pid > 0x1FFFFFFFu) {
      g_log.println("Usage: DENY <pid>");
      return;
    }

    denyPid(pid);
    g_log.printf("DENY 0x%03lX (runtime only)\n",
                  static_cast<unsigned long>(pid));
    return;
  }

  if (oilCalibrationCommand(command.c_str())) return;

  if (command == "SAVE") {
    g_log.println(saveRuntimeConfig()
                       ? "Saved profile/rate/dividers."
                       : "SAVE failed.");
    showConfig();
    return;
  }

  if (command == "LOAD") {
    loadRuntimeConfig();
    loadOilCalibration();
    g_log.println("Reloaded persistent configuration.");
    showConfig();
    return;
  }

  if (command == "RESETCFG") {
    if (g_prefs.begin(CFG_NAMESPACE, false)) {
      ScopedNvsWrite writeGuard;
      g_prefs.clear();
      g_prefs.end();
    }

    g_profileEnabled = true;
    g_oilPublishPeriodMs = 20;
    clearCustomDividers();
    clearDeniedPids();
    clearRaceChronoFilter();
    g_oilCalibration = oil::Calibration{};
    oil::save(g_oilCalibration);
    g_oilHaveValidSample = false;
    saveRuntimeConfig();
    g_log.println("Configuration reset to stability defaults.");
    showConfig();
    return;
  }

  g_log.println(
      "Commands: SHOW [CFG|STATS|MAP] | BLE STATUS | CAN RESTART | "
      "PROFILE ON|OFF | CLEAR FILTERS | RATE <ms> | "
      "ALLOW <pid> <div> | DENY <pid> | "
      "CAL ADC SIG|EXC 0|1 <V> | CAL VERIFY SIG|EXC <errorV> | CAL 0|1|SHOW | SAVE | LOAD | RESETCFG");
}

static void serviceSerialCli() {
  static char buffer[192] = {};
  static size_t length = 0;
  static bool overflow = false;

  size_t processed = 0;
  while (Serial.available() && processed++ < 64u) {
    const int raw = Serial.read();
    if (raw < 0) break;

    if (raw == '\r' || raw == '\n') {
      if (overflow) {
        g_log.println("CLI line too long; ignored.");
      } else if (length > 0) {
        buffer[length] = '\0';
        ++g_cliCommands;
        processCliLine(buffer);
      }
      length = 0;
      overflow = false;
      return;  // At most one complete command per loop pass.
    }

    if (length < sizeof(buffer) - 1) {
      buffer[length++] = static_cast<char>(raw);
    } else {
      overflow = true;
    }
  }
}

// -----------------------------------------------------------------------------
// Arduino entry points
// -----------------------------------------------------------------------------

static void setupImpl() {
  Serial.setTxBufferSize(1024u);
  Serial.begin(115200);
  const uint32_t serialStart = millis();
  while (!Serial && millis() - serialStart < 1000u) {
    delay(10);
  }

  g_bootResetReason = esp_reset_reason();
  g_retainedBootRecord = diagnostics::nextBoot(g_retainedBootRecord,
                                              g_bootResetReason == ESP_RST_POWERON);

  g_log.println();
  g_log.println("==============================================");
  g_log.println("GR86 CCA telemetry stability firmware");
  g_log.printf("Build: %s\n", BUILD_ID);
  g_log.printf("Boot reason: %s (%d)\n",
                resetReasonName(g_bootResetReason),
                static_cast<int>(g_bootResetReason));
  g_log.printf("RTC boot sequence: %lu (resets with power-on or lost RTC record)\n",
                static_cast<unsigned long>(g_retainedBootRecord.count));
  g_log.println("==============================================");

  initWatchdog();

  led_init();
  led_set_power(LedPattern::Solid);
  led_set_ble(LedPattern::BlinkFast);
  led_set_can(LedPattern::BlinkSlow);
  led_set_gps(LedPattern::BlinkSlow);
  led_set_sys(LedPattern::Off);
  led_set_oil(LedPattern::Off);
  led_service(millis());

  analogReadResolution(12);
  analogSetPinAttenuation(OIL_ADC_PIN, ADC_11db);
  analogSetPinAttenuation(OIL_EXC_ADC_PIN, ADC_11db);
  pinMode(OIL_EXC_ADC_PIN, INPUT);
  pinMode(OIL_ADC_PIN, INPUT);

  loadRuntimeConfig();
  loadOilCalibration();

#if GPS_PPS_GPIO >= 0
  pinMode(GPS_PPS_GPIO, INPUT_PULLDOWN);
  attachInterrupt(GPS_PPS_GPIO, onGpsPps, RISING);
  g_log.printf("GPS: PPS on GPIO %d\n", GPS_PPS_GPIO);
#endif

  initBle();

  startGpsProbe(0, millis());

  g_canNextStartMs = 0;
  serviceCanStart(millis());

  showConfig();
}

static void loopImpl() {
  const uint32_t loopUs = micros();
  if (g_previousLoopUs != 0) g_maxLoopGapUs = std::max<uint32_t>(
      g_maxLoopGapUs, loopUs - g_previousLoopUs);
  g_previousLoopUs = loopUs;
  esp_task_wdt_reset();
  serviceOil(millis());
  g_log.service();
  serviceOil(millis());
  serviceBleEvents(millis());
  serviceOil(millis());
  startAdvertisingIfNeeded(millis());
  serviceOil(millis());
  serviceCanStart(millis());
  serviceOil(millis());
  serviceCanHealth(millis());
  serviceOil(millis());
  serviceCanReceive(millis());
  serviceOil(millis());
  publishDiagnostics(millis());
  serviceOil(millis());
  // GPS retains priority in the same shared 120/s BLE budget.
  serviceGps(millis());
  serviceOil(millis());
  serviceCanForwarding(millis());
  serviceOil(millis());
  serviceStatusLeds(millis());
  serviceSerialCli();
  serviceOil(millis());
  g_log.service();
  serviceOil(millis());
  delay(1);
}

}  // namespace cca

void setup() {
  cca::setupImpl();
}

void loop() {
  cca::loopImpl();
}
