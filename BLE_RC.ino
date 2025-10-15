// ===== CCA Telemetry: ESP32-S3 + TWAI + RaceChrono DIY BLE (CAN) =====
// - Robust BLE reconnect
// - TWAI CAN RX burst buffering + per-PID throttle
// - Profile allow-list (car mode) vs sniff-all (bench mode)
// - Oil pressure analog on GPIO1 -> CAN 0x710 (0.1 psi/bit, big-endian)
// - Flash config (Preferences): V0_ADC, V1_ADC, PROFILE, custom dividers
// - Serial CLI (CR, LF, CRLF, or none):
//     CAL 0              -> capture V0_ADC at current input (not saved until SAVE)
//     CAL 1              -> capture V1_ADC at current input (not saved until SAVE)
//     RATE <ms>          -> set oil CAN TX period (not saved until SAVE)
//     PROFILE ON|OFF     -> enable allow-list mode or sniff-all (not saved until SAVE)
//     SHOW               -> list SHOW subcommands
//     SHOW CFG           -> show concise config (no spam)
//     SHOW STATS         -> show runtime counters (no spam)
//     SHOW MAP           -> dump pidMap summary
//     SHOW DENY          -> dump deny list
//     ALLOW <pid> <div>  -> add/update a PID with divider (also removes from deny)
//     DENY  <pid>        -> add PID to deny list
//     SAVE               -> persist V0/V1/profile/custom dividers to flash
//     LOAD               -> reload saved config (or defaults)
//     RESETCFG           -> clear saved config + revert to defaults
//
// Libs:
//   ESP32-TWAI-CAN    (for S3 TWAI)
//   arduino-RaceChrono (NimBLE backend)
//   NimBLE-Arduino
//   Preferences        (built-in)
//
// Board: ESP32S3 Dev Module
// Partition: Default
// PSRAM: Disabled

#include <ESP32-TWAI-CAN.hpp>
#include <RaceChrono.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <cstring>
#include "config.h"   // must define DEFAULT_UPDATE_RATE_DIVIDER

#ifndef FW_VERSION
#define FW_VERSION "unknown"
#endif

#ifdef FW_GITHASH
static const char FW_VERSION_STRING[] = FW_VERSION " (" FW_GITHASH ")";
#else
static const char FW_VERSION_STRING[] = FW_VERSION;
#endif

// ===== Pins (S3 <-> SN65HVD230) =====
#define CAN_TX 5
#define CAN_RX 4

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
static const uint32_t DIAG_PID = 0x77E; // BLE diagnostics frame
static const uint32_t DIAG_PERIOD_MS = 5000;
static uint32_t lastDiagMs = 0;

static uint32_t rxCount = 0;
static uint32_t lastRxPrintMs = 0;

static const char FW_VERSION[] = "CCA Telemetry " __DATE__ " " __TIME__;
static constexpr uint8_t FW_VERSION_MAJOR = 1;
static constexpr uint8_t FW_VERSION_MINOR = 0;
static constexpr uint8_t FW_VERSION_PATCH = 0;
static const char *lastReconnectCause = "boot";

// ===== RaceChrono pidMap extra (per-PID throttle) =====
struct PidExtra {
  uint8_t updateRateDivider = DEFAULT_UPDATE_RATE_DIVIDER;
  uint8_t skippedUpdates = 0;
};
RaceChronoPidMap<PidExtra> pidMap;

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
// (persisted in flash)
static constexpr bool DEFAULT_USE_PROFILE_ALLOWLIST = true;
static bool USE_PROFILE_ALLOWLIST = DEFAULT_USE_PROFILE_ALLOWLIST;

// ======== Oil pressure (analog) ========
// ADC pin/wiring
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

// Publish to RaceChrono as if CAN 0x710 was received
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

static void apply_divider_to_pidmap(uint16_t pid, uint8_t div) {
  void *entry = pidMap.getEntryId(pid);
  if (!entry) return;
  PidExtra *extra = pidMap.getExtra(entry);
  extra->updateRateDivider = div == 0 ? 1 : div;
  extra->skippedUpdates = 0;
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
  apply_divider_to_pidmap(pid, div);
  return true;
}

bool piddiv_clear(uint16_t pid) {
  int idx = find_custom_divider_index(pid);
  if (idx < 0) return false;
  for (uint16_t i = idx + 1; i < customDividerCount; ++i) {
    customDividers[i - 1] = customDividers[i];
  }
  customDividerCount--;
  apply_divider_to_pidmap(pid, compute_default_divider_for_pid(pid));
  return true;
}

void piddiv_apply_all() {
  for (uint16_t i = 0; i < customDividerCount; ++i) {
    apply_divider_to_pidmap(customDividers[i].pid, customDividers[i].div);
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

  bool ok = true;
  cfgPrefs.putFloat("v0_adc", V0_ADC);
  cfgPrefs.putFloat("v1_adc", V1_ADC);
  cfgPrefs.putUChar("profile", USE_PROFILE_ALLOWLIST ? 1 : 0);

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
  size_t expected = sizeof(uint16_t) + sizeof(CfgPidDivider) * customDividerCount;
  size_t written = cfgPrefs.putBytes("dividers", &blob, expected);
  if (written != expected) ok = false;

  cfgPrefs.end();
  return ok;
}

bool cfg_reset_to_defaults() {
  cfg_apply_runtime_defaults();

  if (!cfgPrefs.begin(CFG_NAMESPACE, false)) return false;
  cfgPrefs.clear();
  cfgPrefs.end();
  return true;
}

void cfg_boot_load_and_apply() {
  bool loaded = cfg_load_from_nvs();
  if (!loaded) {
    cfg_apply_runtime_defaults();
  }

  if (USE_PROFILE_ALLOWLIST) apply_allow_list_profile();
  else                       apply_allow_all();

  piddiv_apply_all();
}

// ===== Forward decls =====
static void waitForConnection();
static bool startCanBusReader();
static void stopCanBusReader();

static void bufferNewPacket(uint32_t pid, const uint8_t *data, uint8_t len);
static void handleBufferedPacketsBurst(uint32_t budget_us = 2000);
static void flushBufferedPackets();
static void sendNumCanBusTimeouts();
static void resetSkippedUpdatesCounters();

static void apply_allow_list_profile();
static void apply_allow_all();

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

// ===== BLE command handler (RaceChrono) =====
class UpdateMapOnRaceChronoCommands : public RaceChronoBleCanHandler {
 public:
  void allowAllPids(uint16_t updateIntervalMs) override {
    // Keep protocol happy; we still gate by our mode and deny list.
    pidMap.allowAllPids(updateIntervalMs);
  }
  void denyAllPids() override {
    pidMap.reset();
  }
  void allowPid(uint32_t pid, uint16_t updateIntervalMs) override {
    if (!pidMap.allowOnePid(pid, updateIntervalMs)) return;
    void *entry = pidMap.getEntryId(pid);
    if (entry) {
      PidExtra *e = pidMap.getExtra(entry);
      e->skippedUpdates = 0;
      uint8_t div = compute_default_divider_for_pid(pid);
      int idx = find_custom_divider_index(pid);
      if (idx >= 0) div = customDividers[idx].div;
      e->updateRateDivider = div;
    }
  }
} raceChronoHandler;

// ===== Clean BLE restart =====
static void restartBle(const char *reason) {
  if (reason && reason[0]) {
    lastReconnectCause = reason;
  }
  Serial.println("BLE: restart...");
  if (NimBLEDevice::getAdvertising()) NimBLEDevice::getAdvertising()->stop();
  NimBLEDevice::deinit(true);
  delay(100);
  RaceChronoBle.setUp(DEVICE_NAME, &raceChronoHandler);
  RaceChronoBle.startAdvertising();
  Serial.println("BLE: advertising");
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis(); while (!Serial && millis() - t0 < 3000) {}

  Serial.printf("FW: %s\n", FW_VERSION_STRING);

  // ADC
  analogReadResolution(12);
  analogSetPinAttenuation(OIL_ADC_PIN, ADC_11db);
  pinMode(OIL_ADC_PIN, INPUT);

  // BLE
  Serial.println("BLE setup...");
  RaceChronoBle.setUp(DEVICE_NAME, &raceChronoHandler);
  RaceChronoBle.startAdvertising();
  Serial.println("Waiting for RaceChrono...");
  waitForConnection();

  lastReconnectCause = "boot";

  // Load persisted config + apply profile/dividers
  cfg_boot_load_and_apply();

  show_config(); // one-time concise boot print
}

static void waitForConnection() {
  uint32_t i=0; bool nl=true;
  while (!RaceChronoBle.waitForConnection(1000)) {
    Serial.print("."); nl=false; if ((++i % 10)==0) { Serial.println(); nl=true; }
  }
  if (!nl) Serial.println();
  Serial.println("RaceChrono connected.");
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

    // Respect mode + deny list:
    // - PROFILE ON: send only if pidMap has entry AND not denied
    // - PROFILE OFF: send everything EXCEPT denied
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
        if (extra->skippedUpdates == 0) {
          RaceChronoBle.sendCanData(m->pid, m->data, m->length);
        }
        extra->skippedUpdates++;
        if (extra->skippedUpdates >= extra->updateRateDivider) {
          extra->skippedUpdates = 0;
        }
      } else {
        RaceChronoBle.sendCanData(m->pid, m->data, m->length);
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
  RaceChronoBle.sendCanData(0x777, d, 2);
  last_time_num_can_bus_timeouts_sent_ms = millis();
}

static void resetSkippedUpdatesCounters() {
  struct { void operator()(void *entry) {
    pidMap.getExtra(entry)->skippedUpdates = 0;
  }} fun;
  pidMap.forEach(fun);
}

// ===== Profile application =====
static void apply_allow_list_profile() {
  pidMap.reset();
  clearDeny(); // optional fresh start
  for (size_t i = 0; i < CAR_PROFILE_LEN; ++i) {
    const auto &r = CAR_PROFILE[i];
    pidMap.allowOnePid(r.pid, /*ms*/40);
    void *entry = pidMap.getEntryId(r.pid);
    if (entry) {
      PidExtra *e = pidMap.getExtra(entry);
      e->skippedUpdates = 0;
      e->updateRateDivider = (r.divider == 0 ? 1 : r.divider);
    }
  }
}

static void apply_allow_all() {
  pidMap.reset();
  clearDeny();
  pidMap.allowAllPids(/*ms*/50);
}

// ===== Oil publish =====
static void oil_update_and_publish_if_due() {
  const uint32_t now = millis();
  float v = filtered_adc_volts();
  oil_flags = oil_health_flags_from_volts(v);
  oil_psi_f = volts_to_psi_exact(v);

  if (RaceChronoBle.isConnected() && (now - lastOilTxMs) >= OIL_TX_RATE_MS) {
    lastOilTxMs = now;
    uint16_t psi01 = (uint16_t)(oil_psi_f * OIL_SCALE_01PSI + 0.5f);
    uint8_t d[8] = {
      (uint8_t)(psi01 >> 8), (uint8_t)(psi01 & 0xFF),
      oil_flags, 0, 0, 0, 0, 0
    };
    RaceChronoBle.sendCanData(OIL_CAN_ID, d, 8);
  }
}

static void publish_diag_frame_if_due(uint32_t now) {
  if (!RaceChronoBle.isConnected()) return;
  if ((now - lastDiagMs) < DIAG_PERIOD_MS) return;

  lastDiagMs = now;

  uint32_t uptimeSeconds = now / 1000U;
  uint8_t payload[8] = {
    static_cast<uint8_t>((uptimeSeconds >> 24) & 0xFF),
    static_cast<uint8_t>((uptimeSeconds >> 16) & 0xFF),
    static_cast<uint8_t>((uptimeSeconds >> 8) & 0xFF),
    static_cast<uint8_t>(uptimeSeconds & 0xFF),
    FW_VERSION_MAJOR,
    FW_VERSION_MINOR,
    FW_VERSION_PATCH,
    0x00
  };
  RaceChronoBle.sendCanData(DIAG_PID, payload, sizeof(payload));
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
  Serial.printf("Last reconnect:  %s\n", lastReconnectCause ? lastReconnectCause : "unknown");
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
    uint32_t pid = pidMap.getPid(entry);
    const PidExtra *e = pidMap.getExtra(entry);
    Serial.printf("  %03lX: div=%u\n", (unsigned long)pid, e->updateRateDivider);
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
    apply_allow_list_profile();
    piddiv_apply_all();
    Serial.println("Profile=ALLOW-LIST (not saved)");
    return;
  }
  if (up == "PROFILE OFF") {
    USE_PROFILE_ALLOWLIST = false;
    apply_allow_all();
    piddiv_apply_all();
    Serial.println("Profile=SNIFF-ALL (not saved)");
    return;
  }

  if (up.startsWith("RATE ")) {
    char buf[64]; strncpy(buf, line, sizeof(buf)); buf[sizeof(buf)-1]=0;
    char *tok = strtok(buf + 5, " \t");
    if (tok) {
      unsigned ms = strtoul(tok, nullptr, 0);
      if (ms < 10 || ms > 2000) { Serial.println("RATE out of range (10..2000 ms)"); }
      else { OIL_TX_RATE_MS = (uint16_t)ms; Serial.printf("RATE=%u ms (not saved)\n", ms); }
    } else Serial.println("Usage: RATE <ms>");
    return;
  }

  if (up.startsWith("ALLOW ")) {
    char buf[96]; strncpy(buf, line, sizeof(buf)); buf[sizeof(buf)-1]=0;
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
        void *entry = pidMap.getEntryId(pid);
        if (entry) {
          PidExtra *e = pidMap.getExtra(entry);
          e->skippedUpdates = 0;
          e->updateRateDivider = (uint8_t)n;
        }
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
    char buf[96]; strncpy(buf, line, sizeof(buf)); buf[sizeof(buf)-1]=0;
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
    if (USE_PROFILE_ALLOWLIST) apply_allow_list_profile();
    else                       apply_allow_all();
    piddiv_apply_all();
    show_config();
    return;
  }

  if (up == "RESETCFG") {
    if (cfg_reset_to_defaults()) Serial.println("Config reset to defaults.");
    else                         Serial.println("Failed to reset config.");
    if (USE_PROFILE_ALLOWLIST) apply_allow_list_profile();
    else                       apply_allow_all();
    piddiv_apply_all();
    show_config();
    return;
  }

  Serial.println("Unknown cmd. Try: SHOW | SHOW CFG | SHOW STATS | SHOW MAP | SHOW DENY | CAL 0 | CAL 1 | RATE <ms> | PROFILE ON|OFF | ALLOW <pid> <div> | DENY <pid> | SAVE | LOAD | RESETCFG");
}

// ===== Main loop =====
void loop() {
  loop_iteration++;

  // Defensive re-assert (lightweight)
  if ((loop_iteration % 200) == 0 && RaceChronoBle.isConnected()) {
    if (USE_PROFILE_ALLOWLIST) apply_allow_list_profile();
    else                       apply_allow_all();
    piddiv_apply_all();
  }

  // Handle disconnect / reconnect
  if ((loop_iteration % 100) == 0 && !RaceChronoBle.isConnected()) {
    Serial.println("RC disconnected -> reset map + CAN + BLE.");
    pidMap.reset();
    stopCanBusReader();

    restartBle("RC disconnect");
    Serial.println("Waiting for new RC connection...");
    waitForConnection();

    if (USE_PROFILE_ALLOWLIST) apply_allow_list_profile();
    else                       apply_allow_all();
    piddiv_apply_all();

    sendNumCanBusTimeouts();
  }

  // Ensure CAN is up
  while (!isCanBusReaderActive) {
    if (startCanBusReader()) {
      flushBufferedPackets();
      resetSkippedUpdatesCounters();
      lastCanMessageReceivedMs = millis();
      break;
    }
    delay(500);
  }

  const uint32_t now = millis();

  // Watchdog after first frame
  if (have_seen_any_can && (now - lastCanMessageReceivedMs > 2000)) {
    Serial.println("ERROR: CAN timeout -> restart CAN");
    num_can_bus_timeouts++;
    sendNumCanBusTimeouts();
    stopCanBusReader();
    return;
  }

  // BLE heartbeat (2 Hz)
  if (RaceChronoBle.isConnected() && now - lastHbMs >= 500) {
    lastHbMs = now;
    uint8_t hb[2] = { (uint8_t)(hbCounter & 0xFF), (uint8_t)(hbCounter >> 8) };
    RaceChronoBle.sendCanData(HB_PID, hb, 2);
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

  publish_diag_frame_if_due(now);

  // Periodic timeout counter
  if (now - last_time_num_can_bus_timeouts_sent_ms > 2000) {
    sendNumCanBusTimeouts();
  }

  // ---- Robust Serial CLI: accept CR, LF, CRLF, or no line ending ----
  static char cliBuf[96];
  static uint8_t cliLen = 0;
  while (Serial.available()) {
    int c = Serial.read();
    if (c < 0) break;
    if (c == '\r' || c == '\n') {
      cliBuf[cliLen] = 0;
      if (cliLen) process_cli_line(cliBuf);
      cliLen = 0;
      if (Serial.peek() == '\n' || Serial.peek() == '\r') Serial.read();
    } else if (cliLen < sizeof(cliBuf) - 1) {
      cliBuf[cliLen++] = (char)c;
    }
  }
}
