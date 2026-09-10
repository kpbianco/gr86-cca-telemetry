# I22 manufacturing and assembly effectivity

The controlling electrical source is I20, PCB SHA256 `b6c704c0c97685a68ba708d320347148e5136c182c221d86627ff5b319954b3c`. The filled PCB is `62c08e7050fe95df0a576ece85a051b4049e9684bf8ea5f73e8949657363276b`. The current source is in `iterations/I20_controlled_handoff/candidate_kicad`; matching native exports are in `runtime/hosted/run22/extracted/native_I06_hosted`. The legacy folder name in the native artifact does not change its I20 hash identity.

Native run 34420381540 has zero ERC, DRC, warning, unconnected or parity findings. I21 verified all 774 archived outputs and all 101 CAD/32 firmware inputs. I22 separately verifies unchanged source identities and its new engineering evidence; it does not claim a new native build. Independent copper reconstruction agrees with all four copper Gerbers, 338 plated and ten NPTH drills, and the expanded schematic netlist. This is a review candidate. Thermal engineering and supplier/first-article acceptance remain open.

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

The C05 support relief and 150 mm coax S-route are required. The selected assembly is now explicitly Adafruit 851 with the Adafruit 960 puck. Use 150 ±3 mm to the SMA shoulder, nominal cable OD 1.8 mm, receiving-envelope maximum OD 2.0 mm, 0.5 mm lateral route uncertainty and 15 mm bend radius. The miniature mated plug allocation is 3.4 ×5.3 ×3.25 mm. Preserve its negative-Y exit and independent carrier restraint. The supplied plug dimensions and retention remain acceptance conditions. The published 851 operating ceiling is 60°C, below the 65°C modeled air case; see `ACCESSORY_REVIEW.md`.

The populated clearance model adds the complete 1.71 mm stack to each maximum component height: 0.25 mm solder, 0.51 mm capture freedom, 0.75 mm warp and 0.20 mm local deflection. This corrects the previous 1.20 mm code allowance. Component XY envelopes include 0.30 mm; flexure geometry includes the full 2.2 mm packet, 0.50 mm lateral bow and 0.30 mm position allowance. Carrier base/support XY adds 0.15 mm. Fastener/landing boxes are included. All 1,154 populated-board and 776 mated checks pass; the smallest modeled separation is 0.120 mm from R408 to the T03 flexure, followed by 0.230 mm at L406. These bounds are construction requirements. Exceeding any of them requires a new clearance check before assembly.

All 12 IC/module top/bottom orientations and 151 numbered signal lands now have independently transcribed manufacturer drawing overlays against native placement output. U401 is a manual placement: PCB centre (54.5, 9.0) mm, F.Cu, rotation 0°; its corresponding placement Y is 32.288863 mm. Pin 1 is at PCB (46.7, 2.25) mm in the top view. The legacy PA1616D footprint suffix `v06` does not identify the revision of the current Adafruit-linked V.05 PDF; I22 verifies geometry and pin numbering directly. Do not infer a manufacturer revision from that library filename.

| Exposed pad | Required net | Current mask/paste interpretation |
|---|---|---|
| U121 pad 9 | GND | Manufacturer mask window; paste is 58.85% of exposed mask area. Four nearby pad vias lie outside the paste apertures. |
| U151 pad 17 | GND | Manufacturer mask window; paste is 63.17% of exposed mask area. Six pad vias lie outside the paste apertures. |
| U201 pad 41 | GND | Nine 0.9 mm mask/paste windows within 3.9 mm buried copper. The 48 affected vias require fill, cap and planarization. |
| U301 pad 9 | GND | Multiple same-net returns, including two vias 0.4 mm outside the pad edge. The drawing does not mandate via-in-pad. |
| U501 pad 17 | OIL_EFUSE_RTN | Preserve the TPS2660 RTN topology. Paste is 61.54% of exposed mask area; it is not an instruction to short RTN to system ground. |

The package audit records compatible alternative lead-land dimensions where KiCad IPC lands differ from the manufacturer's example. These orientation and copper/mask checks do not substitute for stencil/reflow or independent human assembly review. Keep DRW-03 and the process gates open.

The current drill reconciliation contains 338 plated holes and ten NPTH holes: 63 ×0.2 mm and 259 ×0.3 mm vias, 12 ×1.02 mm connector holes, and four ×1.1 mm header holes. Nominal via annular ring is at least 0.15 mm. The 1.6 mm board acceptance interval is 1.44–1.76 mm. JLCPCB's published through-hole tolerance is +0.13/−0.08 mm; a public capability is not acceptance of this exact order, and no unsupported NPTH tolerance is assigned. The maximum nominal finished-hole aspect ratio is 8.8; allowing the full negative diameter tolerance raises that geometric ratio to 14.67. Supplier drill-tool/preplate construction and exact connector/hardware fit remain DFM-02 work. The published average wall does not satisfy a 15 µm minimum-wall requirement.

All 322 vias pass the declared normal-load self-heating screen at 2 A in each individual via, with no sharing credit, 1.76 mm board thickness, 15 µm minimum wall and −0.08 mm finished-hole allowance. Maximum calculated via self-rise is 5.103 K against a 10 K budget. Absolute board/package temperature, fault survival and supplier plating acceptance remain separate requirements.

All 776 declared mated-body checks and 37 probe approaches pass. The probe contract is a 0.30 mm needle, 0.10 mm position allowance and at least 5 mm slender exposed shaft. Remove the carrier for bottom access. The listed nearest ground contacts are suitable for DC access; use differential or short-return probes for fast signals. Keep the connector's latch and programming-tool service volumes clear.

Harness effectivity remains the GR86 ASC mapping per Timurrr and the single full-current F1.1 return. Leave F1.2 unpopulated in the harness. Use switched, fused accessory power and retain the relaxed free exit, strain relief and controlled termination/profile conditions. The named Honeywell sensor is a 150 psi sealed-gauge, 5 V ratiometric device.

Historical fabrication, mechanical and assembly records remain evidence of earlier iterations. This document and the exact I20 manifests govern the current candidate. Any geometry, material, stackup, part, cable, solder process or firmware change requires impact review and regenerated affected evidence.
