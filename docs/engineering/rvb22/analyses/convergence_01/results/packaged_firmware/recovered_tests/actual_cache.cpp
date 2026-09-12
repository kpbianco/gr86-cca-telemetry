#include <algorithm>
#include <atomic>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>
#include <map>
#include <limits>
#include "runtime_timing.h"
#include "driver_loss.h"
#include "oil_model.h"
#include "oil_fault_recovery.h"
#include "gps_nmea.h"
#include "gps_solution_epoch.h"
#include "gps_mode_config.h"
#include "io_budget.h"
namespace io=cca::io;
using std::isfinite;
static uint64_t clockUs=0, checks=0;
uint32_t millis(){return static_cast<uint32_t>(clockUs/1000u);}
uint32_t micros(){return static_cast<uint32_t>(clockUs);}
void check(bool ok){if(!ok){std::fprintf(stderr,"FAILED check %llu\n",(unsigned long long)checks);std::abort();}++checks;}
struct Log {template<class...T>void printf(const char*,T...){};void println(const char*){};}g_log;

struct NimBLECharacteristic{std::vector<std::pair<uint32_t,std::vector<uint8_t>>> sent;std::vector<uint8_t>pending;bool accept=true;void setValue(const uint8_t*p,size_t n){pending.assign(p,p+n);}bool notify(){if(!accept)return false;sent.push_back({millis(),pending});return true;}};
static NimBLECharacteristic canObject,gpsObject,timeObject;
static NimBLECharacteristic* g_canCharacteristic=&canObject;
static bool g_canSubscribed=true;
static bool isBleConnected(){return true;}
static uint32_t g_canSlotEvictions=0,g_canCacheCoalesced=0,g_canStaleDropCount=0;
static constexpr uint32_t CAN_MAX_CACHE_AGE_MS=500;
struct RouteDecision{bool allowed;uint32_t minimumIntervalMs;};
static RouteDecision routeDecision(uint32_t pid){return {true,pid==0x710?20u:10u};}
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

void reset(){discardCanCache();g_bleBudget=io::TokenBucket(BLE_TOKEN_CAPACITY,BLE_TOKEN_RATE_PER_SECOND);g_bleNotifyBackoff={};g_bleNotifySuccesses=g_bleNotifyFailures=g_bleNotifyRateDrops=0;canObject.sent.clear();gpsObject.sent.clear();timeObject.sent.clear();canObject.accept=true;}
int main(){
 reset();uint8_t data[8]={};cacheCanFrame(0x710,false,data,8,1);cacheCanFrame(0x777,false,data,8,1);
 for(unsigned i=0;i<5000;++i)cacheCanFrame(0x100+i%180,false,data,8,i+2);
 check(findCanSlot(0x710,false)&&findCanSlot(0x777,false));check(g_canSlotEvictions>0);
 reset();unsigned arrivals=0;uint8_t gpsData[20]={},timeData[3]={};
 for(unsigned t=0;t<120000;++t){clockUs=t*1000ull;
  if(t%100==0)check(notifyCharacteristic(&gpsObject,true,gpsData,20,t));
  if(t%1000==0)check(notifyCharacteristic(&timeObject,true,timeData,3,t));
  if(t%20==0)publishVirtualCan(0x710,data,8,t);
  if(t%2000==0)publishVirtualCan(0x777,data,8,t);
  const unsigned target=(t+1)*1800/1000;
  while(arrivals<target){data[0]=arrivals;cacheCanFrame(0x100+arrivals%180,false,data,8,t);++arrivals;}
  serviceCanForwarding(t);
 }
 unsigned oilCount=0,lastOil=0,maxOilGap=0;for(const auto&e:canObject.sent){const auto&v=e.second;if(v.size()>=4&&v[0]==0x10&&v[1]==7){if(oilCount)maxOilGap=std::max(maxOilGap,e.first-lastOil);lastOil=e.first;++oilCount;}}
 check(oilCount==6000&&maxOilGap==20);check(gpsObject.sent.size()==1200&&timeObject.sent.size()==120);
 check(g_bleNotifySuccesses<=120u*120u+24u);check(arrivals==216000);
 std::printf("TRAFFIC oil=%u gps=%zu time=%zu total=%u oil_max_gap=%u physical_arrivals=%u\n",oilCount,gpsObject.sent.size(),timeObject.sent.size(),g_bleNotifySuccesses,maxOilGap,arrivals);
 reset();clockUs=1000;data[0]=1;publishVirtualCan(0x710,data,8,1);canObject.accept=false;serviceCanForwarding(1);check(findCanSlot(0x710,false)->dirty);
 data[0]=2;publishVirtualCan(0x710,data,8,2);clockUs=100000;canObject.accept=true;serviceCanForwarding(100);check(canObject.sent.empty());
 clockUs=101000;serviceCanForwarding(101);check(canObject.sent.size()==1&&canObject.sent.back().second[4]==2);
 std::printf("PASS actual cache/notification/backpressure %llu checks\n",(unsigned long long)checks);
}
