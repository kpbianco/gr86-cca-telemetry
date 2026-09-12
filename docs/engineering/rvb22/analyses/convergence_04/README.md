# Convergence04 — release power profile before adding thermal hardware

The previous U201 thermal source was deliberately over-conservative: the entire 0.75 A main-rail capacity at 3.443 V was deposited at the ESP32 module, producing 2.58225 W at U201. That remains a useful electrical/transient stress case, but it is not the intended steady operating state of this BLE-only telemetry product.

C04 makes the operating profile explicit in production firmware: 160 MHz CPU build, +3 dBm BLE transmit power, no Wi-Fi API/initialization, and the existing 120 notifications/s global BLE token budget. The pinned FQBN compiles the production image for 160 MHz. Raising CPU frequency or BLE power, enabling Wi-Fi, or materially increasing radio duty invalidates this thermal allocation.

For desktop release analysis, U201 receives a conservative 0.45 A allocation at the retained 3.443 V upper source bound = 1.54935 W. Espressif publishes ESP32-S3 BLE active-TX peaks of 176 mA at 0 dBm and 193 mA at 9 dBm; the +3 dBm setting lies between those tested points, while the 450 mA engineering allocation leaves substantial CPU/PSRAM/peripheral margin rather than treating that table value as a guaranteed whole-module maximum. The 0.75 A board rail capability is retained unchanged.

This is intentionally simpler than forcing a large metal thermal shoe into the ESP32 antenna keepout. C02's RF-clear bottom contacts remain an optional contingency and are screened in parallel. The release profile is not a hardware current limiter: bring-up must confirm that the sustained release workload stays within the 0.45 A allocation. A failure there calls for firmware/profile correction or the already-modeled external cooler—not a blind claim of thermal closure.

The hosted screen runs both 0.5 mm and 0.25 mm meshes against fresh native copper, compares the release allocation with and without the U121 contingency face, and retains the old full-U201 source as a stress sensitivity. Board temperatures are not semiconductor junction temperatures.
