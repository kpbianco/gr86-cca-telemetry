#pragma once

#include <stdint.h>

#ifndef LED_PWR_GPIO
#define LED_PWR_GPIO 46
#endif

#ifndef LED_PWR_ACTIVE_LOW
#define LED_PWR_ACTIVE_LOW 0
#endif

#ifndef LED_BLE_GPIO
#define LED_BLE_GPIO 6
#endif

#ifndef LED_BLE_ACTIVE_LOW
#define LED_BLE_ACTIVE_LOW 0
#endif

#ifndef LED_CAN_GPIO
#define LED_CAN_GPIO 7
#endif

#ifndef LED_CAN_ACTIVE_LOW
#define LED_CAN_ACTIVE_LOW 0
#endif

#ifndef LED_GPS_GPIO
#define LED_GPS_GPIO 8
#endif

#ifndef LED_GPS_ACTIVE_LOW
#define LED_GPS_ACTIVE_LOW 0
#endif

#ifndef LED_SYS_GPIO
#define LED_SYS_GPIO 10
#endif

#ifndef LED_SYS_ACTIVE_LOW
#define LED_SYS_ACTIVE_LOW 0
#endif

#ifndef LED_OIL_GPIO
#define LED_OIL_GPIO 3
#endif

#ifndef LED_OIL_ACTIVE_LOW
#define LED_OIL_ACTIVE_LOW 0
#endif

enum class LedPattern : uint8_t {
  Off = 0,
  Solid,
  BlinkFast,
  BlinkSlow,
  Pulse2Every2s,
  Pulse3Every2s,
};

void led_init();

void led_set_power(LedPattern pattern);
void led_set_ble(LedPattern pattern);
void led_set_can(LedPattern pattern);
void led_set_gps(LedPattern pattern);
void led_set_sys(LedPattern pattern);
void led_set_oil(LedPattern pattern);

void led_service(uint32_t now_ms);
