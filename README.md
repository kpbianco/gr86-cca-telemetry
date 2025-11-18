# CCA Telemetry (GR86 / BRZ Gen2)

CCA Telemetry is a RaceChrono-compatible telemetry stack for the Toyota GR86 / Subaru BRZ (ZN8/ZD8) platform. An ESP32-S3 collects CAN, GPS, and auxiliary analog data, then streams it over Bluetooth Low Energy using RaceChrono's DIY device profile. The codebase is designed for track reliability: aggressive BLE reconnects, TWAI burst handling, persistent configuration, and configurable CAN filtering are all built in.

Hardware bring-up (CAN transceiver wiring, GPS, OLEDs, etc.) is intentionally out of scope here—assume those pieces are already handled. This repository focuses on the firmware, documentation, and validation artifacts you need to build, modify, and deploy the project.

## Repository layout

```
.
├── README.md                ← You're here
├── docs/                    ← RaceChrono presets and user-facing docs
├── firmware/
│   ├── cca_telemetry.ino    ← Main Arduino sketch for the ESP32-S3
│   ├── config.h             ← User-editable build-time configuration
│   ├── pidmaps/             ← CAN whitelist & scaling definitions
│   └── src/                 ← Reusable C++ helpers (GPS parser, NVS config, LEDs, scheduler)
├── libraries/               ← Vendored Arduino dependencies
```

## Firmware architecture

The sketch in `firmware/cca_telemetry.ino` orchestrates several subsystems:

* **CAN ingestion** – `ESP32-TWAI-CAN` runs the ESP32's native TWAI controller in RX burst mode. PID throttling keeps BLE bandwidth under control while the `pidmaps` whitelist ensures only RaceChrono-ready signals are forwarded.
* **BLE transport** – `NimBLE-Arduino` implements the RaceChrono DIY profile (service `0x1FF8`). Notifications are rate-limited with an adaptive governor, reconnection restarts advertisers instead of destroying the controller, and diagnostic counters identify congestion/backpressure.
* **GPS & PPS discipline** – `firmware/src/gps_nmea.*` parses RMC/GGA sentences and disciplines system time to an external PPS input with slew limiting.
* **Persistent configuration** – `firmware/src/nvs_cfg.*` stores calibration data (analog dividers, device profile, etc.) in NVS so tweaks survive power cycles.
* **Status LED + scheduler** – `firmware/src/led_status.*` and `firmware/src/sched.hpp` provide non-blocking indicators and lightweight cooperative task scheduling for housekeeping loops.
* **RaceChrono mapping** – `firmware/pidmaps/` encapsulates CAN scaling/units per signal. `config.h` selects the active table and exposes knobs such as BLE device name, update dividers, and analog calibration constants.

## Building and flashing

### Arduino IDE
1. Install the **ESP32** Arduino core (ESP32-S3 dev module).
2. Drop the `libraries/` contents into your Arduino sketchbook's `libraries` folder or add this repo as an Arduino CLI workspace so the local `libraries/` directory is used.
3. Open `firmware/cca_telemetry.ino` in the IDE. Ensure the board is set to *ESP32S3 Dev Module* and the partition scheme leaves enough app/OTA space.
4. Update `firmware/config.h` with your device name, CAN divisor, analog calibration, and the desired PID whitelist.
5. Compile and upload.

### Arduino CLI (headless)

From the repo root:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 firmware
arduino-cli upload --fqbn esp32:esp32:esp32s3 -p /dev/ttyACM0 firmware
```

The local `libraries/` directory contains every dependency the sketch includes, so no additional `lib install` calls are required when compiling from this workspace.

## Configuration workflow

1. **Select BLE identity & CAN rate** – edit `firmware/config.h` to set `DEVICE_NAME` and `DEFAULT_UPDATE_RATE_DIVIDER`. The divider throttles CAN updates to match BLE throughput.
2. **Choose PID map** – point `ACTIVE_PID_MAP` at one of the structs in `firmware/pidmaps/*.h`. Each entry describes CAN ID, byte ranges, scaling, BLE slot priority, and minimum update period.
3. **Calibrate analog inputs** – update the `V0_ADC` / `V1_ADC` defaults and oil-pressure conversion constants in `config.h` for your sensors.
4. **Persist overrides at runtime** – use the serial CLI exposed by the firmware to tweak dividers or profile settings, then commit them via NVS (`save` command). See the inline comments in `nvs_cfg.*` for the stored keys.

For new cars (non GR86 2nd Gen) create a .h file and place it in the `firmware/pidmaps/*.h` directory. Follow the existing GR86 one as a template. Then within the  `firmware/config.h` change the [`static constexpr const pidmaps::PidMapDefinition *ACTIVE_PID_MAP = &pidmaps::GR86_2022;`] to the name of the namespace defined in your .h file.

## RaceChrono integration

The BLE characteristic layout follows the RaceChrono DIY specification: CAN notify, CAN filter write, GPS notify, GPS time notify, and a diagnostics notify channel. Use the ready-made CAN preset in `docs/racechrono_preset.md` to create RaceChrono channels for the GR86/BRZ platform, including the virtual oil-pressure frame published on CAN ID `0x710` and the curated list of factory CAN IDs.

## Third-party dependencies

All firmware dependencies are vendored inside `libraries/` for reproducibility:

| Library | Purpose |
|---------|---------|
| [`ESP32-TWAI-CAN`](libraries/ESP32-TWAI-CAN) | Thin wrapper over the ESP-IDF TWAI driver for high-volume CAN RX/TX. |
| [`NimBLE-Arduino`](libraries/NimBLE-Arduino) | Efficient BLE stack used for the RaceChrono DIY service. |
| [`RaceChronoDiyBleDevice`](libraries/RaceChronoDiyBleDevice) | Reference BLE UUIDs and helpers from the RaceChrono project. |
| [`arduino-CAN`](libraries/arduino-CAN) | Stock Arduino CAN helpers (useful for experimentation/benching). |
| [`GpsNmea`](libraries/GpsNmea) | Reference parsing helpers (superseded by `firmware/src/gps_nmea.*` but kept for comparison). |
| [`LovyanGFX`, `Joystick`, `Keypad`, `RC-BLE`, `arduino-RaceChrono`] | Optional helpers for advanced UI or HID experiments; not required for the core telemetry flow but retained for completeness. |

If you upgrade any of these libraries, ensure the same versions are available to both the Arduino build and the host-side tests (for headers).

## Maintenance checklist

* Keep `firmware/pidmaps` synchronized with any new CAN reverse-engineering work so BLE filtering stays accurate.
* Monitor BLE notify statistics via the diagnostics characteristic; sustained congestion usually means the CAN divisor should be increased.
* Regenerate RaceChrono presets whenever new signals or virtual channels are exposed.
* Re-run the GPS parser tests (`tests/test_nmea.cc`) after touching `firmware/src/gps_nmea.*` or changing compiler flags.

## License & acknowledgements

This project builds on the open-source work of the ESP32, NimBLE, and RaceChrono communities. Please review the individual licenses inside `libraries/` before redistributing binaries.
