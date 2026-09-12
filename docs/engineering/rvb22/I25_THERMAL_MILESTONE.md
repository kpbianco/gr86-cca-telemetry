# I25 — Thermal design milestone closure

## Disposition

Desktop thermal **design work is complete for the next milestone**. The production design basis is the existing PCB/cooling construction plus the bounded release firmware/power profile defined by C04. No C02 contact shoe, bridge, heatsink, preload mechanism, mask opening, or other cooler hardware is adopted.

This is a design disposition, not an invented physical qualification result. The remaining thermal work is verification on hardware against explicit acceptance limits below; failure of one of those limits reopens thermal design and invokes the preserved C02 contingency or a firmware derating.

## Release design basis

Source commit: `0187cafd8610e11f3faf2a8ef35b39863bd40b40`  
Hosted verification: workflow run `34620964062`  
0.5 mm artifact: `10272517101`  
0.25 mm artifact: `10273346331`

The release profile is frozen at:

- ESP32-S3 production build: 160 MHz;
- BLE transmit power: +3 dBm;
- Wi-Fi: prohibited by checked production source;
- global BLE notification budget: 120/s;
- sustained release main-profile allocation: <= 0.45 A at the retained 3.443 V bound;
- converter efficiency design floor: >= 80% in the release operating region;
- coupled heat allocation: 2.940859375 W.

The historical approximately 4.79 W model / 4.815 W envelope remains a stress sensitivity. It is not the release steady-state thermal source.

## Hosted native-copper results

| Case | Mesh | Max board region | U201 mean | U151 mean | U121 mean | Sink |
|---|---:|---:|---:|---:|---:|---:|
| Release, existing cooling | 0.50 mm | 101.6503 C | 100.3292 C | 82.3078 C | 88.7849 C | 67.3017 C |
| Release, existing cooling | 0.25 mm | 107.2667 C | 105.6558 C | 83.4149 C | 91.9132 C | 67.2629 C |
| Release, C02 contingency | 0.50 mm | 96.3635 C | 95.0379 C | 79.6475 C | 83.3018 C | 67.4747 C |
| Release, C02 contingency | 0.25 mm | 102.3512 C | 100.7157 C | 80.7043 C | 85.6874 C | 67.4359 C |
| Historical full stress, C02 | 0.25 mm | 126.4143 C | 123.6950 C | 89.9959 C | 102.2541 C | 68.9192 C |

The 0.50 -> 0.25 mm rise is 5.6163 C for the selected existing-cooling release case and 5.9877 C for the unadopted C02 contingency. Therefore C04 does **not** claim mesh convergence. For milestone screening, retain a deliberately conservative one-step discretization allowance equal to the observed 0.50 -> 0.25 mm rise. This gives a board-region screening bound of **112.8830 C** for the selected release design. This number is not a semiconductor junction temperature or module ambient temperature.

## Why C02 is not adopted

At the fine mesh C02 reduces the release maximum by only 4.9154 C. That improvement does not justify introducing a new mechanically loaded thermal assembly with preload, tolerance, service-clearance, insulation and RF-proximity qualification work. C02 remains preserved as a contingency if physical release-profile measurements fail.

## Component-temperature interpretation

The thermal solver predicts PCB regions and a sink node. It does not prove ESP32 package/junction temperature or the air temperature immediately outside the module. The fitted `ESP32-S3-WROOM-1-N8R2` is an 85 C ambient-rated module; Espressif defines that ambient as the environment immediately outside the module. The dashboard bulk-air design case remains 65 C, leaving 20 C of local-air rise margin that must be verified on hardware rather than inferred from PCB temperature.

The LM5164 and TPS62172 regulator operating ranges are not the controlling desktop concern at the modeled source-region temperatures. Their final junction/case temperatures are nevertheless verified during bring-up rather than inferred from board-region mean values.

## Thermal acceptance gates at bring-up

Thermal design stays closed unless a gate fails. With the final enclosure/installation representative and the release firmware loaded:

1. At the worst intended dashboard thermal condition, demonstrate sustained release-profile current <= 0.45 A on the main profile. Capture steady state and representative BLE burst behavior.
2. Demonstrate the implemented converter chain is no worse than the 80% efficiency allocation in the release operating region, or directly show actual regulator losses are no greater than the C04 allocations.
3. Measure air immediately outside U201 and require <= 85 C for the fitted N8R2 module.
4. Record U201 shield/case, U121 package, U151 package, PCB hotspot-area and enclosure/landing temperatures to establish correlation to C04. Any manufacturer operating limit or measured abnormal thermal behavior is a failure even if the current/air gates pass.
5. Repeat at the intended upper input-voltage condition and worst credible telemetry workload with CAN receive, 10 Hz GPS, oil sensing and BLE streaming active.

If gates 1-5 pass, no additional cooler is required and thermal verification closes without a PCB respin. If any gate fails, first correct firmware/profile if the release contract was violated; otherwise activate the C02 contingency as the starting point for a mechanically complete cooler design rather than improvising a board respin.

## Freeze / invalidation rules

Thermal design must be reopened before release if any of the following changes: CPU above 160 MHz, BLE power above +3 dBm, Wi-Fi enabled, materially higher radio duty or notification budget, steady main-profile current above 0.45 A, converter efficiency below 80%, fitted ESP32 module temperature grade, board copper/stackup around the modeled heat paths, enclosure airflow/landing assumptions, or intended maximum dashboard ambient above 65 C.

No physical test was fabricated by this record. No package junction temperature is claimed. No accessory whose own published temperature limit is below the installation requirement is silently qualified by this board-level model.
