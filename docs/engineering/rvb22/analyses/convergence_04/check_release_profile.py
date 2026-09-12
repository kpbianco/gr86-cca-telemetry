#!/usr/bin/env python3
import json,re,sys
from pathlib import Path
D=Path(__file__).resolve().parent
ROOT=D.parents[1]
ino=ROOT/'candidate/firmware/cca_telemetry/cca_telemetry.ino'
build=ROOT/'candidate/firmware/build.sh'
text=ino.read_text(); build_text=build.read_text()
cfg=json.loads((D/'RELEASE_POWER_PROFILE.json').read_text())
eff=cfg['converter_efficiency_floor']
u151=cfg['u201_heat_allocation_W']*(1/eff-1)
u121_out=cfg['u201_heat_allocation_W']/eff+cfg['gps_5v_allocation_W']+cfg['oil_5v_allocation_W']+cfg['other_5v_allocation_W']
u121=u121_out*(1/eff-1)+cfg['u121_magnetic_headroom_W']
total=cfg['u201_heat_allocation_W']+u151+u121+cfg['gps_5v_allocation_W']+cfg['oil_5v_allocation_W']+cfg['other_5v_allocation_W']
checks={
 'cpu_build_profile': f'CPUFreq={cfg["cpu_mhz"]}' in build_text,
 'no_240mhz_build_profile': 'CPUFreq=240' not in build_text,
 'ble_power_profile': f'NimBLEDevice::setPower(static_cast<int8_t>({cfg["ble_tx_dbm"]}))' in text,
 'no_wifi_api': not re.search(r'\b(WiFi\.|esp_wifi_|WiFiClass|#include\s*[<"]WiFi)',text),
 'notification_budget': 'BLE_TOKEN_RATE_PER_SECOND = 120' in text,
 'u201_allocation_math': abs(cfg['u201_current_allocation_A']*cfg['u201_voltage_bound_V']-cfg['u201_heat_allocation_W'])<1e-12,
 'u151_coupled_math': abs(u151-cfg['u151_loss_allocation_W'])<1e-12,
 'u121_output_coupled_math': abs(u121_out-cfg['u121_output_burden_W'])<1e-12,
 'u121_coupled_math': abs(u121-cfg['u121_heat_allocation_W'])<1e-12,
 'release_total_math': abs(total-cfg['release_total_heat_allocation_W'])<1e-12,
 'release_below_legacy_stress': total < cfg['previous_total_heat_stress_W'],
 'efficiency_is_conservative_floor': 0 < eff <= 0.80
}
print(json.dumps(checks,indent=2))
if not all(checks.values()): sys.exit(1)
