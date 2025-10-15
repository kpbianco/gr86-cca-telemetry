#pragma once

// Customize your BLE device name here
#define DEVICE_NAME "CCA Telemetry"

// Default divider for RaceChronoPidMap extra (only used as init)
#define DEFAULT_UPDATE_RATE_DIVIDER 10

// Enable NVS-backed configuration persistence helpers
#define ENABLE_CCA_CFG_NVS 1

// CAN bus baud rate (500 kbps)
static const long BAUD_RATE = 500 * 1000L;
