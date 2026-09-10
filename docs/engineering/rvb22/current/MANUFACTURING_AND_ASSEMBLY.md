# I21 manufacturing and assembly effectivity

The controlling electrical source is I20, PCB SHA256 `b6c704c0c97685a68ba708d320347148e5136c182c221d86627ff5b319954b3c`. The filled PCB is `62c08e7050fe95df0a576ece85a051b4049e9684bf8ea5f73e8949657363276b`. The current source is in `iterations/I20_controlled_handoff/candidate_kicad`; matching native exports are in `runtime/hosted/run22/extracted/native_I06_hosted`. The legacy folder name in the native artifact does not change its I20 hash identity.

Native run 34420381540 has zero ERC, DRC, warning, unconnected or parity findings. All 774 archived outputs and all 101 CAD/32 firmware inputs passed fresh digest verification. Independent copper reconstruction agrees with all four copper Gerbers, 338 plated and ten NPTH drills, and the expanded schematic netlist. This is a review candidate. Thermal engineering and supplier/first-article acceptance remain open.

| Item | Current instruction |
|---|---|
| Assembly | 153 fitted references. The 151 automated placement rows match the native board; F101 and U401 are manual exceptions. Keep both in the fitted BOM. |
| Placement datum | X equals PCB X. Placement Y equals 41.288863 mm minus PCB Y. Side and rotation are checked explicitly. |
| Filled/capped vias | 49 total: 48 at U201 exposed pad 41, plus one at R153. Use the I20 source coordinates and the current via-in-pad list. Do not use the historical 13-via list. Fill, cap and planarize for soldering. |
| Plated wall | 15 µm minimum finished wall is a model/construction requirement. An average plating specification does not establish the minimum. No resin conductivity credit is used. |
| PCB outline | 88.254766 × 61.275846 mm overall board bounds, including the C206 tab. ESP32 module overhang and mated harness/service envelopes are additional. |
| C206 | T598X477M006ATE025, F.Cu at (82, −4) mm on the grounded FR4 tab. Preserve clearance, polarity and the new long branch. The tab's 30 g static case passes its declared hot-FR4 bound; this is not a fatigue result. |
| Precision feedback | R155 TNPU060311K8HWEA00, 11.8 kΩ; R156 TNPU06034K99HWEA00, 4.99 kΩ. The current analysis includes 0.02% tolerance, 2 ppm/K and independent 0.1% drift. |
| Bulk damping | R158 WSLP0603R0820FEA, 82 mΩ. Historical 47 mΩ instructions are superseded. |
| F101 process | Littelfuse 0885001.DR. Exclude from the global paste/PnP process. Its recommended 255–260°C process conflicts with the ESP32 module's 235–250°C range; use a qualified local solder process after global reflow. |
| Models | Every fitted reference has a resolved model. Many are conservative manufacturer-height/allocated-XY envelopes; they are not asserted to be detailed supplier solids. |
| Branding | Approved Compact TW artwork, one-color F.SilkS, 8 mm. Current native polygon comparison agrees with the original SVG within the recorded geometric tolerance. |

The controlling carrier is C05 with W02 wing flexures and T03 central flexure, in `iterations/I15_service_mechanics/candidate_mechanics`. Each flex member uses 200 annealed, unbonded C110 laminae, 0.010–0.011 mm each, with bonded terminals only. Total thickness is 2.0–2.2 mm. Do not substitute a solid bonded stack or the older 40-foil construction: their reaction forces differ. Support the cold terminals as specified. Material conductivity, hot strength, friction, retained clamp tension, creep/fatigue and the maximum 220 g carrier mass remain explicit construction conditions.

The C05 support relief and 150 mm coax S-route are required. Preserve the allocated cable OD ≤1.8 mm, 15 mm bend radius, 3.4 ×5.3 ×3.25 mm mated U.FL prism and negative-Y cable exit. The exact purchased accessory must satisfy these dimensions and temperature limits. The Amphenol assembly studied as a candidate is not silently substituted for the user's Adafruit cable.

All 776 declared mated-body checks and 37 probe approaches pass. The probe contract is a 0.30 mm needle, 0.10 mm position allowance and at least 5 mm slender exposed shaft. Remove the carrier for bottom access. The listed nearest ground contacts are suitable for DC access; use differential or short-return probes for fast signals. Keep the connector's latch and programming-tool service volumes clear.

Harness effectivity remains the GR86 ASC mapping per Timurrr and the single full-current F1.1 return. Leave F1.2 unpopulated in the harness. Use switched, fused accessory power and retain the relaxed free exit, strain relief and controlled termination/profile conditions. The named Honeywell sensor is a 150 psi sealed-gauge, 5 V ratiometric device.

Historical fabrication, mechanical and assembly records remain evidence of earlier iterations. This document and the exact I20 manifests govern the current candidate. Any geometry, material, stackup, part, cable, solder process or firmware change requires impact review and regenerated affected evidence.
