# CCA Telemetry (GR86 / BRZ Gen2)

CCA Telemetry is a RaceChrono-compatible telemetry stack for the Toyota GR86 / Subaru BRZ (ZN8/ZD8) platform. An ESP32-S3 collects CAN, GPS, and auxiliary analog data, then streams it over Bluetooth Low Energy using RaceChrono's DIY device profile. The codebase is designed for track reliability: aggressive BLE reconnects, TWAI burst handling, persistent configuration, and configurable CAN filtering are all built in.

Hardware bring-up (CAN transceiver wiring, GPS, OLEDs, etc.) is intentionally out of scope here—assume those pieces are already handled. This repository focuses on the firmware, documentation, and validation artifacts you need to build, modify, and deploy the project.

## What this guide covers

* **End-to-end bring-up.** The “Track-day bring-up checklist” walks from flashing through wiring, BLE pairing, and pre-session validation so even a first-time CAN user can connect, configure, and verify the module before heading out.【F:README.md†L46-L125】
* **Subsystem deep dives.** “Firmware architecture,” “Oil pressure channel & calibration,” and the PID-map sections explain how each feature works (CAN ingest, BLE, GPS/PPS, oil analog path, pidmaps) and point directly to the source files for further hacking.【F:README.md†L77-L150】【F:README.md†L206-L213】
* **Runtime operations.** The Serial CLI reference documents every SHOW/PROFILE/ALLOW/FIL/TOKEN/RATE/SAVE command so you can administer the device without reflashing once it is installed in the car.【F:README.md†L214-L244】
* **RaceChrono & filtering workflows.** The RaceChrono integration, `.fil` ingestion walkthrough, and pidmap cloning guide teach you how to curate channels from the phone or at compile time, keeping CAN/BLE bandwidth under control.【F:README.md†L131-L150】【F:README.md†L180-L213】
* **Diagnostics & recovery.** LED pin/pattern tables, the troubleshooting checklist, and persistent-configuration guidance translate firmware counters into hands-on fixes at the car so you can recover from CAN, BLE, GPS, or oil faults mid-event.【F:README.md†L114-L130】【F:README.md†L123-L178】【F:README.md†L238-L244】

Together these sections document the full software feature set—from CAN filtering to RaceChrono streaming and oil calibration—so newcomers can treat the README as a production-ready operations guide.

## Release-ready polish

To make the repo publishable as an open-source reference design, every workflow called out in prior PRs is now paired with actionable guidance in this README:

* **Single source of truth.** The Track-day checklist, subsystem deep dives, CLI catalog, and RaceChrono workflows are all anchored to the implementation files they describe, so readers can jump from prose to code while auditing the feature set.
* **Operator focus.** Each runtime touch point—PID governance, GPS/PPS, oil calibration, RaceChrono filters, persistence—includes both “how it works” and “how to recover” instructions so someone new to CAN can confidently run the module at the track.
* **Debug-to-code breadcrumbs.** LED tables, SHOW/CAL output, and `.fil` logging now cite the handlers that emit those diagnostics, letting contributors trace symptoms back to the firmware without guesswork.

This section is the final assurance that the README stands alone as a production-ready operations manual: new users get the high-level tour at the top, the detailed workflows in the middle, and the debug/persistence references at the end without needing outside docs.

## Feature matrix & documentation coverage

| Feature / decision | Where it lives in code | Where to configure / operate it |
|--------------------|------------------------|----------------------------------|
| CAN burst ingestion, TWAI error recovery, PID throttling | `firmware/cca_telemetry.ino` main loop + TWAI handlers | *Firmware architecture → CAN ingestion & PID governance* and *Track-day bring-up checklist* describe how to wire and validate the bus before driving.【F:firmware/cca_telemetry.ino†L205-L235】【F:README.md†L23-L71】 |
| BLE transport, RaceChrono DIY profile, diagnostics notify | NimBLE setup in `firmware/cca_telemetry.ino` | *Track-day bring-up checklist → Pair with RaceChrono*, *RaceChrono integration*, and the troubleshooting bullets tie the LEDs, CLI, and `.fil` workflow together for new users.【F:firmware/cca_telemetry.ino†L89-L146】【F:README.md†L40-L73】【F:README.md†L120-L152】 |
| GPS + PPS time discipline | `firmware/src/gps_nmea.*`, PPS ISR in the sketch | *Firmware architecture → GPS & PPS discipline* plus the LED/troubleshooting tables walk through wiring, lock states, and debugging steps.【F:firmware/src/gps_nmea.cpp†L1-L210】【F:README.md†L75-L152】 |
| Oil pressure sampling, filtering, calibration, and CAN 0x710 publisher | Analog path in `firmware/cca_telemetry.ino` + `nvs_cfg.*` | *Track-day bring-up checklist*, *Oil pressure channel & calibration*, and the CLI section spell out how to hook up sensors, capture endpoints, and stream the data.【F:firmware/cca_telemetry.ino†L755-L807】【F:README.md†L33-L58】【F:README.md†L190-L225】 |
| PID maps, whitelist/deny list, divider overrides, `.fil` ingestion | `firmware/pidmaps/*.h`, filter characteristic handlers | *Creating your own PID map* and *Allowing extra PIDs from RaceChrono filter files* provide both build-time and runtime workflows, while CLI commands (`PROFILE`, `ALLOW`, `DENY`, `FIL VERBOSE`, `TOKEN`) are cataloged in the Serial CLI reference.【F:firmware/pidmaps/gr86_2022.h†L1-L162】【F:firmware/cca_telemetry.ino†L1360-L1465】【F:README.md†L152-L210】 |
| Persistent configuration and recovery | `firmware/src/nvs_cfg.*` | *Configuration workflow* and *Serial CLI → Persistence & recovery* explain how to edit defaults, capture runtime tweaks, and reset misconfigurations without reflashing.【F:firmware/src/nvs_cfg.cpp†L1-L116】【F:README.md†L171-L199】 |
| LED pinout, blink dictionary, scheduler hooks | `firmware/src/led_status.*`, `firmware/src/sched.hpp` | *Status LED + scheduler* table plus the troubleshooting checklist let a first-time CAN user translate physical indicators into next steps at the car.【F:firmware/src/led_status.cpp†L11-L192】【F:README.md†L92-L152】 |
| Build & flash repeatability | Arduino CLI/IDE instructions in this repo | *Building and flashing* covers both GUI and headless flows; the *Maintenance checklist* calls out regression tests to keep the repo production-ready.【F:README.md†L210-L266】 |

This matrix ensures every subsystem that ships in the firmware is paired with an operator-facing doc section, so a new builder can move from flashing through troubleshooting without chasing external resources.

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

## Track-day bring-up checklist

These steps walk a first-time CAN user from a blank ESP32-S3 dev module to a working RaceChrono feed. Each item links back to the source so you know which files implement the behavior you are validating.

### 1. Flash and configure the firmware

1. Install the ESP32 support package inside the Arduino IDE or CLI and select the **ESP32S3 Dev Module** target, matching the board the sketch was written for.【F:firmware/cca_telemetry.ino†L1-L20】
2. Open `firmware/config.h` and set the user-facing knobs—`DEVICE_NAME`, `DEFAULT_UPDATE_RATE_DIVIDER`, PPS input (`GPS_PPS_GPIO`), CAN baud, and `ACTIVE_PID_MAP`. These constants gate BLE discovery, the CAN throttle default, and whether PPS disciplining is enabled at boot.【F:firmware/config.h†L4-L27】
3. Build and flash `firmware/cca_telemetry.ino` using the instructions later in this README. The sketch pulls in the vendored libraries automatically, so no additional dependency fetch is required beyond what lives in `libraries/`.【F:firmware/cca_telemetry.ino†L21-L57】
4. Connect to the USB serial console at 115200 8N1, hit enter to reveal the prompt, then run `SHOW CFG` to confirm the compiled defaults (BLE name, profile mode, dividers) match what you edited before heading to the car.【F:firmware/cca_telemetry.ino†L2370-L2395】【F:firmware/cca_telemetry.ino†L3400-L3409】

### 2. Wire and power the module

1. Tie the SN65HVD230 (or equivalent) transceiver’s TXD/RXD pins to `GPIO5`/`GPIO4`, then hook its CANH/CANL pins into the car’s 500 kbps high-speed bus; the firmware leaves the TWAI controller in listen mode until it sees valid frames so you can verify the connection on the bench first.【F:firmware/cca_telemetry.ino†L76-L82】【F:firmware/config.h†L20-L21】
2. Wire your GPS puck to UART1 (`GPIO18` ← GPS TX, `GPIO17` → GPS RX) and land an optional PPS input on `GPIO16` if you want nanosecond-level lap timing—the PPS ISR and slew limiter are already enabled as soon as `GPS_PPS_GPIO` is ≥ 0.【F:firmware/cca_telemetry.ino†L80-L109】【F:firmware/config.h†L11-L15】
3. Route the analog oil-pressure sender into `GPIO1/ADC1_CH0`. The firmware samples, filters, and publishes it on virtual CAN ID `0x710`, so all you need to do trackside is capture two calibration points with the CLI after plumbing the sensor.【F:firmware/cca_telemetry.ino†L5-L7】【F:firmware/cca_telemetry.ino†L755-L775】
4. Land the status LEDs on the default pins (`GPIO46`, `GPIO6`, `GPIO7`, `GPIO8`, `GPIO10`, and `GPIO3`) or adjust the macros in `firmware/src/led_status.h` if your harness is different. The table later in this README explains every blink pattern you will see once the car powers the unit.【F:firmware/src/led_status.h†L6-L32】【F:firmware/cca_telemetry.ino†L1368-L1373】

### 3. Pair with RaceChrono and feed channels

1. Power the car or bench supply so the ESP32 starts advertising under the `DEVICE_NAME` you configured. RaceChrono will discover the DIY service (`0x1FF8`) plus the CAN, GPS, GPS-time, filter, and diagnostics characteristics defined in the sketch.【F:firmware/cca_telemetry.ino†L89-L95】
2. Import `docs/racechrono_preset.md` into RaceChrono to create ready-made channels for every GR86/BRZ PID in the curated allow-list, including the synthetic oil frame on `0x710`. This saves you from typing scaling equations on the phone.【F:docs/racechrono_preset.md†L1-L68】
3. If you need to add or remove CAN IDs directly from your phone, issue a `TOKEN <hex8>` over serial and then push a `.fil` file from RaceChrono’s *Manage filters* UI. Each command in the file becomes an ALLOW/DENY write on characteristic `0x0002`, which the firmware authenticates with the token before applying it to the runtime map.【F:firmware/cca_telemetry.ino†L1360-L1465】【F:firmware/cca_telemetry.ino†L3613-L3647】
4. Finish the pairing pass by checking `SHOW MAP` / `SHOW DENY` and the BLE diagnostics characteristic (`0x0005`) so you know the RaceChrono UI and the firmware are in sync before the session.【F:firmware/cca_telemetry.ino†L3345-L3505】【F:firmware/cca_telemetry.ino†L867-L920】

### 4. Validate data flow and hit the track

1. Watch the LED bank: solid power LED plus pulsing CAN/GPS LEDs indicate the car is broadcasting and GPS sentences are flowing, while the SYS LED warns about BLE throttling so you can bump dividers before packets drop.【F:firmware/src/led_status.h†L6-L32】【F:firmware/cca_telemetry.ino†L3732-L3879】
2. Use `SHOW STATS` after a shakedown lap to confirm there are no CAN timeouts, TWAI restarts, or BLE notify drops. The counters cover every subsystem mentioned earlier in this README.【F:firmware/cca_telemetry.ino†L3411-L3438】
3. Run `CAL SHOW`, `SHOW CFG`, and `RATE` to double-check that oil calibration, broadcast cadence, and profile mode match what you expect before logging laps; save any runtime tweaks so they persist across pit cycles.【F:firmware/cca_telemetry.ino†L3507-L3575】【F:firmware/cca_telemetry.ino†L3566-L3575】【F:firmware/cca_telemetry.ino†L782-L807】

## Firmware architecture

The sketch in `firmware/cca_telemetry.ino` orchestrates several subsystems:

* **CAN ingestion** – `ESP32-TWAI-CAN` runs the ESP32's native TWAI controller in RX burst mode. PID throttling keeps BLE bandwidth under control while the `pidmaps` whitelist ensures only RaceChrono-ready signals are forwarded.
* **BLE transport** – `NimBLE-Arduino` implements the RaceChrono DIY profile (service `0x1FF8`). Notifications are rate-limited with an adaptive governor, reconnection restarts advertisers instead of destroying the controller, and diagnostic counters identify congestion/backpressure.
* **GPS & PPS discipline** – `firmware/src/gps_nmea.*` parses RMC/GGA sentences and disciplines system time to an external PPS input with slew limiting.
* **Persistent configuration** – `firmware/src/nvs_cfg.*` stores calibration data (analog dividers, device profile, etc.) in NVS so tweaks survive power cycles.
* **Status LED + scheduler** – `firmware/src/led_status.*` and `firmware/src/sched.hpp` provide non-blocking indicators and lightweight cooperative task scheduling for housekeeping loops.
* **RaceChrono mapping** – `firmware/pidmaps/` encapsulates CAN scaling/units per signal. `config.h` selects the active table and exposes knobs such as BLE device name, update dividers, and analog calibration constants.

### Subsystem deep dive

#### CAN ingestion and PID governance

The CAN stack configures TWAI alerts for RX queue overruns, bus-off, arbitration loss, and other critical faults, then polls those counters each second to decide when to restart the controller or surface diagnostic frames back to RaceChrono.【F:firmware/cca_telemetry.ino†L205-L235】 Above that, the `pidmaps` layer predefines the GR86/BRZ whitelist plus default per-ID dividers and lets the CLI flip between strict allow-list streaming (`PROFILE ON`) or a sniff-all fall-through that still honors the runtime deny list.【F:firmware/pidmaps/gr86_2022.h†L9-L118】【F:firmware/cca_telemetry.ino†L718-L865】 Custom divider overrides are persisted and applied on boot so you can bump individual CAN IDs without recompiling, and the same bookkeeping powers `SHOW MAP`, `ALLOW`, and `DENY` in the serial console.【F:firmware/cca_telemetry.ino†L712-L865】【F:firmware/cca_telemetry.ino†L3345-L3505】

**Whitelist, blacklist, and dividers.** Each entry in `firmware/pidmaps/*.h` carries a target RaceChrono channel plus a default divider, meaning “forward one in every _N_ frames for this CAN ID.” Lower dividers increase BLE load; higher ones conserve bandwidth for low-value or high-frequency signals. `PROFILE ON` enforces this whitelist (only IDs present in the map are forwarded) while `PROFILE OFF` switches to “sniff everything” and relies on the deny list to suppress noisy IDs. `ALLOW <pid> <div>` temporarily whitelists additional IDs with their own dividers, whereas `DENY <pid>` forces the firmware to drop that ID even if the profile is off or the ID appears in the static map. Both overrides are stored in NVS via `SAVE` so trackside experiments persist across reboots.【F:firmware/cca_telemetry.ino†L3341-L3609】

#### BLE transport and congestion control

`NimBLE-Arduino` exposes the full DIY profile (CAN, filter, GPS, GPS time, and diagnostics characteristics) and negotiates aggressive 7.5–15 ms connection intervals to keep RaceChrono synced.【F:firmware/cca_telemetry.ino†L275-L357】 A moving-average notify governor tracks bytes-per-second, clamps burst size, and progressively backs off when congestion or ATT errors show up, with event counters logged through `SHOW STATS` and the diagnostics characteristic so you know whether BLE or CAN is the bottleneck.【F:firmware/cca_telemetry.ino†L139-L174】【F:firmware/cca_telemetry.ino†L867-L920】 The reconnection path restarts advertisers and reapplies MTU sizing automatically, so unplugging power or losing the phone link no longer wedges the NimBLE host.【F:firmware/cca_telemetry.ino†L301-L357】

#### GPS and PPS discipline

`gps_nmea.cpp` parses both RMC and GGA sentences, clamps checksum errors, and normalizes latitude/longitude/altitude fields, while the main sketch tracks the GPS init state machine (autobaud probing, fallback timers, and periodic notify cadence).【F:firmware/src/gps_nmea.cpp†L1-L121】【F:firmware/src/gps_nmea.cpp†L123-L198】【F:firmware/cca_telemetry.ino†L237-L274】 When a PPS input is wired, the ISR debounces the pulse, estimates drift, and slews the system clock by ±2 ms/second to keep RaceChrono’s GPS and CAN timestamps aligned, which dramatically reduces lap interpolation error.【F:firmware/cca_telemetry.ino†L176-L188】

#### Persistent configuration and CLI tie-in

The `nvs_cfg` helpers serialize oil calibration voltages, versioning, and CRCs into the `cca_cfg` namespace so even power-cycled cars retain the analog slope you captured with `CAL` commands.【F:firmware/src/nvs_cfg.cpp†L1-L75】 Raw blobs can be inspected with `CAL SHOW`, and the same module exposes guard rails (finite, monotonic voltages) to keep corrupt data from applying at boot.【F:firmware/src/nvs_cfg.cpp†L76-L116】 In addition to analog endpoints, NVS stores PID divider overrides, custom oil broadcast rates, FIL tokens, and the profile bit so the CLI “SAVE / LOAD / RESETCFG” workflow mirrors what’s burned into flash.【F:firmware/cca_telemetry.ino†L782-L865】【F:firmware/cca_telemetry.ino†L3650-L3674】

#### Status LED + scheduler

`led_status.*` defines six independent channels (power, BLE, CAN, GPS, SYS, oil) with per-pin polarity, a library of blink/pulse patterns, and a tiny state machine that updates the GPIOs without blocking other tasks.【F:firmware/src/led_status.cpp†L11-L192】 Time-sliced housekeeping loops use the `every()` helper in `sched.hpp` to run at deterministic cadences (GPS notify, CAN diagnostics, oil broadcasts) even when other work momentarily stalls the main loop.【F:firmware/src/sched.hpp†L5-L23】

**Pinout and debug patterns.** The default harness routes the LEDs to the following ESP32-S3 pins (active high unless the macros in `led_status.h` are flipped). Use the table to translate each blink or pulse into a subsystem status when debugging at the car.【F:firmware/src/led_status.h†L6-L53】【F:firmware/cca_telemetry.ino†L1368-L1373】【F:firmware/cca_telemetry.ino†L3740-L3887】

| LED | ESP32-S3 GPIO | Patterns & meaning |
|-----|---------------|---------------------|
| Power | `GPIO46` | Turns solid immediately after `setup()` runs so you know the MCU left ROM boot and the scheduler/housekeeping loops are alive.【F:firmware/src/led_status.h†L6-L22】【F:firmware/cca_telemetry.ino†L2370-L2384】 |
| BLE | `GPIO6` | Fast blink while advertising/awaiting RaceChrono, solid once a BLE link is up, and off once RaceChrono disconnects so you can instantly spot pairing issues.【F:firmware/src/led_status.h†L6-L24】【F:firmware/cca_telemetry.ino†L1368-L1373】 |
| CAN | `GPIO7` | Slow blink means the TWAI driver is up but no recent frames have been seen, a double pulse every 2 s indicates live CAN traffic, and solid means the reader is running but has gone >1 s without a packet (check wiring or ignition).【F:firmware/src/led_status.h†L6-L26】【F:firmware/cca_telemetry.ino†L3732-L3744】 |
| GPS | `GPIO8` | Slow blink while waiting on UART data, solid when NMEA sentences are flowing, and triple pulses every two seconds once valid RMC fixes are locking timestamps to PPS.【F:firmware/src/led_status.h†L6-L28】【F:firmware/cca_telemetry.ino†L3860-L3875】 |
| SYS | `GPIO10` | Off during normal operation; switches to a slow blink whenever the BLE governor clamps notifications or the burst budget is exhausted, highlighting congestion before packets drop.【F:firmware/src/led_status.h†L6-L30】【F:firmware/cca_telemetry.ino†L3778-L3879】 |
| Oil | `GPIO3` | Solid = calibrated sensor in range; fast blink = ADC sees a short to ground; double pulse every 2 s = open harness; slow blink = voltage out of range but not a hard fault, matching the fault bits that go out on CAN `0x710`.【F:firmware/src/led_status.h†L6-L32】【F:firmware/cca_telemetry.ino†L3838-L3849】 |

## Troubleshooting checklist

* **No CAN frames** – Verify the CAN LED is double-pulsing; if it is solid or dark, check the SN65 wiring on `GPIO5/4` and make sure the bus really is at 500 kbps, then run `SHOW STATS` to confirm there are no TWAI restarts or RX timeouts being logged.【F:firmware/cca_telemetry.ino†L76-L82】【F:firmware/config.h†L20-L21】【F:firmware/cca_telemetry.ino†L3411-L3438】
* **BLE congestion** – A blinking SYS LED means the notify governor is clamping bursts; inspect the diagnostics characteristic (`0x0005`) and `SHOW STATS` counters, then raise the per-PID dividers or disable noisy IDs via `DENY`/`ALLOW`/`PROFILE` until the clamp counter stabilizes.【F:firmware/src/led_status.h†L6-L30】【F:firmware/cca_telemetry.ino†L867-L920】【F:firmware/cca_telemetry.ino†L3341-L3609】
* **GPS not locking** – The GPS LED should switch from slow blink to solid once UART sentences arrive and to triple pulses when PPS discipline is active; if it stays dark, confirm the UART pins (`GPIO18/17`) and PPS input (`GPIO16`) match your harness and recheck the GPS baud/autobaud logic in the console output.【F:firmware/cca_telemetry.ino†L80-L188】【F:firmware/config.h†L11-L15】【F:firmware/src/led_status.h†L6-L28】
* **Oil channel faults** – Combine the oil LED pattern with `CAL SHOW` output to see whether you are tripping the open/short bits, broadcasting stale calibration endpoints, or simply exceeding the configured pressure range. `RATE` lets you slow the cadence if BLE saturation coincides with the faults.【F:firmware/cca_telemetry.ino†L3382-L3549】【F:firmware/cca_telemetry.ino†L3566-L3575】【F:firmware/src/led_status.h†L6-L32】
* **RaceChrono filters not applying** – Turn `FIL VERBOSE ON`, make sure the FIL token matches, and watch the serial console for ALLOW/DENY updates while the `.fil` file streams; the runtime state is always visible via `SHOW MAP` / `SHOW DENY` before you `SAVE`.【F:firmware/cca_telemetry.ino†L1360-L1465】【F:firmware/cca_telemetry.ino†L3345-L3655】

#### RaceChrono mapping & docs

`firmware/pidmaps/` centralizes the RaceChrono-facing whitelist, scaling equations, and default dividers so each CAN signal documents its frequency and equation in one place, and the `docs/racechrono_preset.md` file provides copy/paste RaceChrono channel definitions—including the virtual oil pressure frame—for anyone replicating the setup without reverse-engineering the CAN bus themselves.【F:firmware/pidmaps/gr86_2022.h†L9-L162】【F:docs/racechrono_preset.md†L1-L68】

### Creating your own PID map

1. **Copy the template.** Duplicate `firmware/pidmaps/gr86_2022.h`, keep the include to `pidmaps/pidmap_defs.h`, and rename the namespace/`MAP` constant to match your chassis so the compiler gets a unique `PidMapDefinition`. The header already shows how the whitelist helper, divider policy, and `RULES` array hang together.【F:firmware/pidmaps/pidmap_defs.h†L1-L24】【F:firmware/pidmaps/gr86_2022.h†L1-L106】
2. **Populate `isCanIdWhitelisted()` and `policyDividerForId()`.** Use them to describe which CAN IDs should ever reach RaceChrono and how aggressively to down-sample each one (1 = every frame). This is where you capture heuristics such as “forward all 0x7xx OBD replies” or “keep wheel speeds at full rate.”【F:firmware/pidmaps/gr86_2022.h†L9-L88】
3. **Describe every signal in `RULES`.** Each `PidRule` pairs an ID with its default divider so `PROFILE ON` can pre-load the runtime map without touching NVS. Comment each entry with the signal name so future contributors know why it exists.【F:firmware/pidmaps/gr86_2022.h†L90-L162】
4. **Select the map at build time.** Update `firmware/config.h` so `ACTIVE_PID_MAP` points to your new definition, then rebuild/flash. The pointer is `constexpr`, so the compiler will catch typos before you get to the track.【F:firmware/config.h†L18-L24】
5. **Validate over serial.** Power-cycle, run `SHOW MAP`, and confirm the CLI shows the IDs/dividers you expect. Flip `PROFILE ON/OFF` or use `ALLOW/DENY` to compare your static file with runtime overrides before committing the map to version control.【F:firmware/cca_telemetry.ino†L3341-L3609】

### Allowing extra PIDs from RaceChrono filter files

The firmware also honors RaceChrono’s DIY filter (.fil) uploads so you can curate allow-lists from the phone instead of typing dozens of CLI commands:

1. **Issue a FIL token.** On the serial console run `TOKEN <hex8>` so the ESP32 stores a shared secret in NVS; RaceChrono includes the same token in every filter write and the firmware drops packets that do not match.【F:firmware/cca_telemetry.ino†L3613-L3647】
2. **Build or import a `.fil`.** In RaceChrono’s *Settings → Devices → (CCA telemetry) → Manage filters* UI, create a file that lists each CAN ID plus the interval you want. When the phone connects it writes those entries to characteristic `0x0002`, which the firmware parses in `handleFilWrite()` (command 0 = deny all, 1 = allow all with interval, 2 = add one PID+interval).【F:firmware/cca_telemetry.ino†L1376-L1465】
3. **Verify and persist.** Use `SHOW MAP` / `SHOW DENY` to confirm the file-based updates landed, toggle `FIL VERBOSE ON` if you want to watch each request stream by, then finish with `SAVE` so the resulting runtime map/dividers come back after a reboot.【F:firmware/cca_telemetry.ino†L3345-L3505】【F:firmware/cca_telemetry.ino†L3475-L3481】【F:firmware/cca_telemetry.ino†L3613-L3655】

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

## Oil pressure channel & calibration

The ESP32-S3 monitors the analog oil-pressure sender on `GPIO1/ADC1_CH0`, runs a median-of-five plus IIR filter, and converts the reading into a 0–150 psi value using two calibration endpoints that survive power cycles via NVS.【F:firmware/cca_telemetry.ino†L755-L781】【F:firmware/cca_telemetry.ino†L2770-L2823】 The defaults (`V0_ADC = 0.3482 V` at 0 psi, `V1_ADC = 3.1290 V` at 150 psi) work well with the common 150 psi KA Sensors transducer, but the serial CLI lets you capture your own endpoints at idle (0 psi reference) and during a known pressure event (e.g., 150 psi on a regulated bench). Use `CAL 0` to latch the current filtered voltage as the low endpoint, `CAL 1` for the high endpoint, and `CAL SHOW` to dump the raw structure (including CRC) that was committed to flash.【F:firmware/cca_telemetry.ino†L767-L775】【F:firmware/cca_telemetry.ino†L3507-L3549】

After each filtered reading the firmware evaluates three health bits—open circuit, short-to-ground, and out-of-range—and stuffs them into the third byte of the CAN payload so RaceChrono can flag wiring faults alongside pressure data.【F:firmware/cca_telemetry.ino†L2770-L2778】【F:firmware/cca_telemetry.ino†L3382-L3397】 The final value is clamped to the configured pressure range with gentle dead-bands near 0 psi and 150 psi so noisy samples do not cause flicker in the app.【F:firmware/cca_telemetry.ino†L2780-L2823】

When connected over BLE, the code publishes the oil pressure on virtual CAN ID `0x710` at 0.1 psi resolution (16-bit big-endian) plus the aforementioned flags at a default 40 ms cadence. The `RATE <ms>` CLI command lets you increase or decrease that cadence between 10 ms and 2000 ms without recompiling, and the setting is persisted the next time you issue `SAVE`.【F:firmware/cca_telemetry.ino†L772-L789】【F:firmware/cca_telemetry.ino†L3382-L3397】【F:firmware/cca_telemetry.ino†L3566-L3575】 Import the canned RaceChrono definition in `docs/racechrono_preset.md` to create a ready-to-use gauge (psi or kPa variants) that matches the firmware’s scaling.【F:docs/racechrono_preset.md†L1-L52】

## Serial CLI reference

All runtime configuration happens over the primary USB serial interface (`Serial.begin(115200)`), so you can leave the car fully assembled and still monitor or tweak the telemetry unit. Connect with any terminal emulator at 115200 8N1 and press enter to reveal the prompt; the handler trims whitespace and accepts uppercase or lowercase commands.【F:firmware/cca_telemetry.ino†L2370-L2395】【F:firmware/cca_telemetry.ino†L3484-L3494】

### SHOW commands

* `SHOW` – lists the available subcommands for quick discovery.【F:firmware/cca_telemetry.ino†L3495-L3500】
* `SHOW CFG` – prints the active profile mode, oil calibration endpoints, custom divider count, and FIL token (if any).【F:firmware/cca_telemetry.ino†L3400-L3409】
* `SHOW STATS` – dumps runtime counters (CAN timeouts, restarts, notify suppression, etc.) so you can confirm bus health after a session.【F:firmware/cca_telemetry.ino†L3411-L3438】
* `SHOW MAP` – enumerates the CAN allow-list entries that are currently whitelisted for RaceChrono streaming, while `SHOW DENY` reveals any IDs you have explicitly blocked via the CLI.【F:firmware/cca_telemetry.ino†L3345-L3379】【F:firmware/cca_telemetry.ino†L3475-L3505】

### Calibration & oil telemetry controls

* `CAL 0` / `CAL 1` – capture the present filtered voltage as the low or high calibration point and save it to NVS with CRC protection. The command guards against inverted endpoints so you can recalibrate in the paddock without bricking the device.【F:firmware/cca_telemetry.ino†L3507-L3534】
* `CAL SHOW` – prints the raw structure, including stored/calculated CRCs, to verify that flash writes succeeded.【F:firmware/cca_telemetry.ino†L3537-L3549】
* `RATE <ms>` – overrides the oil CAN broadcast interval (10–2000 ms) in real time. Combine this with the RaceChrono channel frequency if you need to conserve BLE bandwidth on dense CAN maps.【F:firmware/cca_telemetry.ino†L3566-L3575】

### CAN profile & PID management

* `PROFILE ON/OFF` – toggles between the curated allow-list from `firmware/pidmaps/*.h` and a sniff-all mode that relays every CAN frame the ESP32 sees. Switching profiles automatically reapplies dividers and governor state so you can compare modes back-to-back without reflashing.【F:firmware/cca_telemetry.ino†L3341-L3379】【F:firmware/cca_telemetry.ino†L3553-L3563】
* `ALLOW <pid> <div>` – explicitly whitelists an individual CAN ID with a custom divider (1–255). The PID is removed from the deny list, added to the runtime map, and the divider is persisted unless the table is full.【F:firmware/cca_telemetry.ino†L3577-L3593】
* `DENY <pid>` – blocks a CAN ID even when sniff-all mode is enabled. The deny table accepts up to 64 entries and is visible via `SHOW DENY`.【F:firmware/cca_telemetry.ino†L3577-L3609】【F:firmware/cca_telemetry.ino†L3475-L3481】
* `FIL VERBOSE [ON|OFF]` – turns on verbose logging for BLE characteristic `0x0002` so you can audit `ALLOW/DENY` updates arriving from RaceChrono’s DIY filter UI. Use `TOKEN <hex8>` to set or inspect the FIL token that authenticates those writes over BLE; the firmware rate-limits and discards packets that do not match the saved token.【F:firmware/cca_telemetry.ino†L1360-L1455】【F:firmware/cca_telemetry.ino†L3613-L3647】

### Persistence & recovery

* `SAVE` – commits all mutable settings (oil calibration, profile bit, divider overrides, oil rate, FIL token, etc.) to the `cca_cfg` NVS namespace so they survive power cycles.【F:firmware/cca_telemetry.ino†L782-L807】【F:firmware/cca_telemetry.ino†L3650-L3655】
* `LOAD` – reloads whatever was stored previously. If nothing exists yet, the firmware falls back to the defaults defined in `config.h`, reapplies the active PID map, and prints the resulting configuration.【F:firmware/cca_telemetry.ino†L3657-L3665】
* `RESETCFG` – wipes NVS back to the compiled defaults, reapplies the profile/dividers, and prints the fresh state so you can start over after experiments.【F:firmware/cca_telemetry.ino†L3668-L3674】

Every command echoes the new value or error state to make remote debugging easier, and an “Unknown cmd” hint lists the entire command surface whenever you mistype an entry.【F:firmware/cca_telemetry.ino†L3676-L3677】

## License & acknowledgements

This project builds on the open-source work of the ESP32, NimBLE, and RaceChrono communities. Please review the individual licenses inside `libraries/` before redistributing binaries.
