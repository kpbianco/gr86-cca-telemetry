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

#define GPS_PPS_GPIO -1
enum hardwareSerial_error_t{UART_FIFO_OVF_ERROR,UART_BUFFER_FULL_ERROR};
static constexpr int SERIAL_8N1=0,GPS_RX_GPIO=18,GPS_TX_GPIO=17;
struct HardwareSerial{
 int space=256;size_t calls=0;bool shortWrite=false;std::vector<std::string> writes;std::string input;size_t cursor=0;
 HardwareSerial(int){} int availableForWrite(){return space;}
 size_t write(const uint8_t*p,size_t n){++calls;size_t accepted=shortWrite&&n?n-1:n;writes.emplace_back((const char*)p,accepted);return accepted;}
 void end(){}size_t setRxBufferSize(size_t n){return n;}void setTxBufferSize(size_t){}
 void onReceiveError(void(*)(hardwareSerial_error_t)){}void begin(uint32_t,int,int,int){}
 int available(){return static_cast<int>(input.size()-cursor);}int read(){return cursor<input.size()?static_cast<unsigned char>(input[cursor++]):-1;}
};
static inline bool timeReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}
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

std::string sentence(const std::string& body){unsigned c=0;for(char x:body)c^=x;char tail[8];std::snprintf(tail,sizeof(tail),"*%02X",c);return "$"+body+tail;}
void feed(const std::string& body,uint32_t now){auto line=sentence(body);clockUs=now*1000ull;parseGpsSentence(line.c_str(),now);}
void reset(){g_gpsSolutionEpoch={};g_gpsRmcSequence=0;g_gpsDuplicateRmcEpochs=g_gpsBackwardRmcEpochs=0;g_gpsSerial.input.clear();g_gpsSerial.cursor=0;g_gpsSerial.space=256;g_gpsSerial.shortWrite=false;g_gpsSerial.calls=0;g_gpsSerial.writes.clear();g_gpsValidFix=false;g_gpsTxDeferred=g_gpsTxShortWrites=0;startGpsProbe(0,0);}
void rmc(unsigned t,uint32_t now,const char* talker="GPRMC",const char* validity="A"){
 char b[180];std::snprintf(b,sizeof(b),"%s,1200%02u.%03u,%s,3745.000,N,12227.000,W,0.0,0.0,090926,,,A",talker,(t/1000)%60,t%1000,validity);feed(b,now);
}
void mode(){feed("PMTK001,353,3",0);feed("PMTK001,313,3",0);feed("PMTK001,301,3",0);feed("PMTK001,220,3",0);feed("PMTK513,0",0);}
int main(){
 reset();const char*cmd=gps::kGpsOnly;const size_t n=std::strlen(cmd);
 for(int room=-1;room<static_cast<int>(n);++room){g_gpsSerial.space=room;check(!queueGpsCommand(cmd));check(g_gpsSerial.calls==0);}
 g_gpsSerial.space=n;check(queueGpsCommand(cmd));check(g_gpsSerial.writes.back()==cmd);
 g_gpsSerial.shortWrite=true;check(!queueGpsCommand(cmd)&&g_gpsTxShortWrites==1);g_gpsSerial.shortWrite=false;
 reset();g_gpsCurrentBaud=9600;g_gpsSentenceSeenAtCurrentBaud=true;g_gpsSerial.space=0;
 serviceGpsStateMachine(2000);check(g_gpsState==GpsState::Probe&&g_gpsSerial.calls==0);
 g_gpsSerial.space=256;serviceGpsStateMachine(2001);check(g_gpsState==GpsState::SwitchTo115200);check(g_gpsSerial.writes.back()==PMTK_115200);
 reset();g_gpsSentenceSeenAtCurrentBaud=true;serviceGpsStateMachine(0);check(g_gpsState==GpsState::Configure);
 g_gpsSerial.space=0;serviceGpsStateMachine(0);check(g_gpsConfigCommandIndex==0);
 g_gpsSerial.space=256;for(unsigned i=0;i<8;++i)serviceGpsStateMachine(i*150);check(g_gpsState==GpsState::Active);check(g_gpsSerial.writes.size()==7);
 for(const auto&w:g_gpsSerial.writes){auto x=w.substr(0,w.size()-2);check(gps::checksumOk(x.c_str(),x.size()));}
 reset();for(unsigned i=0;i<=10;++i)rmc(i*100,i*100);check(!g_gpsConfigured);check(g_gpsRmcSequence==11);
 reset();mode();for(unsigned i=0;i<=10;++i)rmc(i*100,i*100);check(g_gpsConfigured&&g_gpsValidFix);
 const auto seq=g_gpsRmcSequence,capture=g_rmcCaptureMillis;rmc(1000,1800,"GNRMC");check(g_gpsRmcSequence==seq&&g_rmcCaptureMillis==capture&&g_gpsDuplicateRmcEpochs==1);
 rmc(900,1900);check(!g_gpsValidFix&&!g_gpsConfigured&&g_gpsBackwardRmcEpochs==1);
 feed("PMTK513,1",2000);check(!g_gpsModeEvidence.confirmed());
 for(unsigned missing=0;missing<4;++missing){reset();mode();if(missing==0)g_gpsModeEvidence.ack353=255;if(missing==1)g_gpsModeEvidence.ack313=2;if(missing==2)g_gpsModeEvidence.ack301=255;if(missing==3)g_gpsRateAck=255;for(unsigned i=0;i<=10;++i)rmc(i*100,i*100);check(!g_gpsConfigured);}
 uint8_t v=255;
 for(const auto&body:{"PMTK513,0","PMTK513,1"}){auto x=sentence(body);check(gps::parseSbasReadback(x.c_str(),v));}
 for(const auto&body:{"PMTK513,2","PMTK513,00","PMTK513,0,0","PMTK513,","PMTK513,-1"}){auto x=sentence(body);check(!gps::parseSbasReadback(x.c_str(),v));}
 auto bad=sentence("PMTK513,0");bad.back()=bad.back()=='0'?'1':'0';check(!gps::parseSbasReadback(bad.c_str(),v));
 reset();std::string x=sentence("GPRMC,120000.000,A,3745.000,N,12227.000,W,0.0,0.0,090926,,,A");
 g_gpsSerial.input=x+std::string(1,'\0')+"hidden\n"+x+"\r\n";readGpsBytes(10);check(g_gpsRmcSequence==1&&g_gpsInvalidCharacters==1);
 reset();uint32_t random=1234;
 for(unsigned batch=0;batch<2000;++batch){g_gpsSerial.input.clear();g_gpsSerial.cursor=0;for(unsigned i=0;i<512;++i){random=random*1664525u+1013904223u;g_gpsSerial.input+=static_cast<char>(random>>24);}readGpsBytes(batch);check(g_gpsLineLength<sizeof(g_gpsLine));}
 std::printf("PASS actual GPS state/command/epoch/parser %llu checks\n",(unsigned long long)checks);
}
