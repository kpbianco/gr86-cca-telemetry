#!/usr/bin/env python3
import json,re,sys
from pathlib import Path
D=Path(__file__).resolve().parent
ROOT=D.parents[1]
ino=ROOT/'candidate/firmware/cca_telemetry/cca_telemetry.ino'
build=ROOT/'candidate/firmware/build.sh'
text=ino.read_text(); build_text=build.read_text()
cfg=json.loads((D/'RELEASE_POWER_PROFILE.json').read_text())
checks={
 'cpu_build_profile': f'CPUFreq={cfg["cpu_mhz"]}' in build_text,
 'no_240mhz_build_profile': 'CPUFreq=240' not in build_text,
 'ble_power_profile': f'NimBLEDevice::setPower(static_cast<int8_t>({cfg["ble_tx_dbm"]}))' in text,
 'no_wifi_api': not re.search(r'\b(WiFi\.|esp_wifi_|WiFiClass|#include\s*[<"]WiFi)',text),
 'notification_budget': 'BLE_TOKEN_RATE_PER_SECOND = 120' in text,
 'allocation_math': abs(cfg['u201_current_allocation_A']*cfg['u201_voltage_bound_V']-cfg['u201_heat_allocation_W'])<1e-12,
 'allocation_reduced': cfg['u201_heat_allocation_W'] < cfg['previous_u201_heat_stress_W']
}
print(json.dumps(checks,indent=2))
if not all(checks.values()): sys.exit(1)
