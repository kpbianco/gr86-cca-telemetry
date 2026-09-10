#!/usr/bin/env python3
"""Author the I22 engineering handoff from the preserved I21 checkpoint."""
from pathlib import Path
import json, shutil

W=Path(__file__).resolve().parents[1]
D=W/'analyses/review_i22'
H=W/'control/review_history/I21'
H.mkdir(parents=True,exist_ok=True)
def save(name,text):
    p=W/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(text.strip()+'\n')
for name in ['current/EXECUTION_PLAN.md','current/MANUFACTURING_AND_ASSEMBLY.md','current/ACCESSORY_REVIEW.md','current/THERMAL_REVIEW.md','README_CURRENT.md','GR86_CCA_RevB_Board_Vetting_Summary.md','control/CURRENT_CONTINUATION.md','control/RECOVERY_STATE.json','control/FINAL_HANDOFF_STATE.json','control/DESKTOP_ACTION_DISPOSITIONS_CURRENT.json']:
    dst=H/name
    if not dst.exists():dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(W/name,dst)

save('current/ACCESSORY_REVIEW.md', '''# I22 confirmed Adafruit accessories

The user supplied the exact product links on 2026-09-10: [Adafruit 851 adapter](https://www.adafruit.com/product/851) and [Adafruit 960 active GPS antenna](https://www.adafruit.com/product/960). Their identities are confirmed. PID 960 explicitly recommends PID 851. Current instructions and the mated model use this pair; the earlier Amphenol study is historical.

| Item | Documented or allocated value | Applicability |
|---|---|---|
| 851 cable | RG178, 150 ±3 mm to the SMA shoulder; 1.8 mm nominal OD | The SMA connector length is additional. I22 allocates 2.0 mm maximum OD plus 0.5 mm route uncertainty. This maximum is a receiving requirement, not a published cable tolerance. |
| 851 connectors | Product page specifies standard SMA; the linked C934-001 drawing labels RP-SMA | Product-level pairing is documented. Inspect the supplied center contact against the standard-SMA antenna before mating. The contradictory drawing is retained. |
| 851 temperature | −10 to +60°C in its linked specification | Does not cover 65°C modeled bulk air or any required exposure below −10°C. |
| 960 antenna | GPS L1, five-metre cable, SMA male; 2.3–5.5 V and −30 to +85°C | No GLONASS gain or all-band noise claim is made. |
| PA1616D bias | External-antenna detection 4 mA; documented current limit 25/28/31 mA at 3.0/3.3/3.6 V | Limits are taken from the exact module document. Actual delivered bias voltage and fault timing remain unmeasured. |
| Typical antenna current | Linear interpolation gives 9.8 mA at 3.3 V | Typical headroom is 5.8 mA above detection and 18.2 mA below the 28 mA limit. This does not supply guaranteed hot/cold current extrema. |

At the 3.0 V GPS supply boundary, the entire internal bias feed, cable and contact path has 0.7 V available before reaching the antenna's 2.3 V minimum. At 25 mA this is a 28 Ω total series-resistance budget. It is an acceptance budget, not a measured resistance or proof that the module delivers 3.0 V at its antenna terminal. GPS-09 retains exact bias-drop, hot/cold-current and open/short/hot-plug acceptance.

The revised 150 mm route, 2.0 mm OD allocation and full populated height stack pass all 776 mated checks. RF cable/plug metal remains at least 16.421 mm from the ESP32 antenna region under the declared model. Preserve the negative-Y U.FL cable exit, 15 mm bend radius, independent carrier strain restraint, relaxed free lead and support of the panel SMA. Unmate the U.FL vertically with the intended tool; never use the PCB socket as the harness strain anchor. The 3.4 ×5.3 ×3.25 mm mated U.FL prism is still an allocated envelope because the linked 851 drawing does not specify all miniature-plug dimensions. GPS-14 stays open for exact supplied plug dimensions and retention acceptance.

With 65°C air and 70°C passive landings, improved passive cooling alone cannot keep a 60°C-rated adapter within its limit. The minimum steady-state shortfall is 5 K even at zero dissipation. This is a boundary-condition result, not a measurement of dashboard air. The retained fine board model gives 92.438–94.443°C in the J401 board region; these are board-region estimates, not cable or connector temperatures. Keep the adapter selected, with temperature acceptance unresolved until its applicable assembly rating or the actual verified installation environment covers the requirement. No lower temperature requirement or replacement part is silently adopted.

Primary documents are the [851 C934-001 drawing](https://cdn-shop.adafruit.com/product-files/851/C934-001_datasheet.pdf), [960 GPS-01 specification](https://cdn-shop.adafruit.com/datasheets/GPS-01.pdf), and [PA1616D document](https://cdn-shop.adafruit.com/product-files/5186/5186_PA1616D_Datasheet.pdf). Retrieved PDF hashes, the concise product-page observations and source dates are retained under `analyses/review_i22/sources`. Numerical results are in `ACCESSORY_APPLICABILITY.json`, `THERMAL_ACCESSORY_BOUND.json` and `analyses/mated_i22/RESULTS.json`.
''')

assembly=(H/'current/MANUFACTURING_AND_ASSEMBLY.md').read_text().replace('# I21 manufacturing and assembly effectivity','# I22 manufacturing and assembly effectivity')
assembly=assembly.replace('All 774 archived outputs and all 101 CAD/32 firmware inputs passed fresh digest verification.','I21 verified all 774 archived outputs and all 101 CAD/32 firmware inputs. I22 separately verifies unchanged source identities and its new engineering evidence; it does not claim a new native build.')
start=assembly.index('The C05 support relief and 150 mm coax S-route are required.')
end=assembly.index('\nAll 776 declared mated-body checks',start)
assembly=assembly[:start]+'''The C05 support relief and 150 mm coax S-route are required. The selected assembly is now explicitly Adafruit 851 with the Adafruit 960 puck. Use 150 ±3 mm to the SMA shoulder, nominal cable OD 1.8 mm, receiving-envelope maximum OD 2.0 mm, 0.5 mm lateral route uncertainty and 15 mm bend radius. The miniature mated plug allocation is 3.4 ×5.3 ×3.25 mm. Preserve its negative-Y exit and independent carrier restraint. The supplied plug dimensions and retention remain acceptance conditions. The published 851 operating ceiling is 60°C, below the 65°C modeled air case; see `ACCESSORY_REVIEW.md`.

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
''' +assembly[end:]
save('current/MANUFACTURING_AND_ASSEMBLY.md',assembly)

thermal=(H/'current/THERMAL_REVIEW.md').read_text().replace('# I21 thermal refinement','# I22 thermal applicability and retained I21 refinement')
thermal=thermal.replace('The previously interrupted 0.125 mm','The 0.125 mm').replace('is now complete.','was completed in I21. I22 retains that result. ',1)
thermal=thermal.replace('is therefore reopened.','remains open after its I21 reopening.')
thermal+='''
I22 confirms the selected adapter as Adafruit 851. Its linked specification limits operation to 60°C. For the declared passive steady-state system, all thermal boundaries are at least 65°C and heat generation is nonnegative. A global temperature below every boundary would require outward heat flow of the wrong sign. Therefore passive contact/spreader improvements alone cannot produce a ≤60°C adapter at this design case, even with zero board power. The minimum boundary conflict is 5 K. This does not establish actual installed air temperature and does not authorize reducing the environment requirement.

The exact J401 footprint region was sampled from the preserved 0.125 mm map: 92.438–94.443°C on the bottom board layer. Those values are not cable, plug or socket temperatures. `analyses/review_i22/THERMAL_ACCESSORY_BOUND.json` binds the extraction to the map hash and records its scope. A verified compatible accessory rating or installation temperature is needed in addition to board/package cooling work. No new thermal mesh or physical cooling revision is claimed in I22.
'''
save('current/THERMAL_REVIEW.md',thermal)

save('current/EXECUTION_PLAN.md','''# I22 closure plan and current source

This iteration starts at I21 commit `550d87fea69147e131fd71603d02204a44c903f1` on PR #45. The user confirmed Adafruit 851 and 960 and requested another iteration. The complete I21 redline/criterion state was frozen before the checks. I22 corrects the accessory model, adds the omitted mechanical height allowance, completes independent package overlays and bounded via/RF reviews, and reevaluates all 290 original criteria. It closes six: LIB-02, LIB-08, VIA-06, SI-12, GPS-01 and MECH-03. Counts become 140 closed, 146 open and four not applicable. All 357 recorded redline identities remain visible; that history count is not the count of open criteria.

Confirmed inputs are Honeywell MIPAN2XX150PSAAX, the GR86 ASC harness per Timurrr, Adafruit 851/960, JLCPCB fabrication and dashboard use. Equivalent numerical power/RF/thermal models and circuit, firmware, mechanical and assembly corrections are authorized. The unchanged design case is 4.815 W, 65°C bulk air, 70°C landings and 70 kPa. No engine-bay environment or reduced dashboard requirement is introduced.

| Workstream | I22 result | Next closure action |
|---|---|---|
| Native CAD and target build | Verified I20 source remains current, with zero native findings and 153 resolved models. I22 does not change CAD or firmware. | Rerun the pinned native workflow after any CAD/firmware change and bind the resulting exports. |
| Package orientation and exposed pads | 12 manufacturer top/bottom overlays, 151 signal lands, all 20 GPS pad functions and 12 wrong-orientation negative controls pass. Exact exposed-pad net/mask/thermal construction reviewed. | Retain alternative-land and stencil conditions; obtain the original independent human drawing review under DRW-03. |
| Mechanical model and assembly | Full 1.71 mm height stack replaces 1.20 mm; 1,154 populated and 776 mated checks pass. Smallest separation is 0.120 mm at R408/T03. Cable uses 150 ±3 mm and a 2.0 mm OD receiving envelope. | Verify exact supplied plug dimensions, carrier/foil materials, retention and installed construction. Recheck every cooling revision against these bounds. |
| Via current and drilling | All 322 vias pass the conditional 2 A self-heating screen, maximum 5.103 K against 10 K. Nominal drills agree with native exports. | Resolve exact finished-hole/pin/hardware fit, drawing/DFM agreement and supplier minimum wall/fill/cap conditions. |
| Board RF and digital returns | Analysis-only SI-12 scope is justified by the existing 3,981,312-case board network, minimum 10.219 dB return loss and maximum 0.867 dB insertion loss. All 12 RF segments retain adjacent ground. | Keep actual stack/device allocations, antenna/bias and installed GPS criteria separate. Bound or reroute the 54 digital adjacent-plane interruptions; alternate ground presence alone is insufficient. |
| Accessory interface | User-confirmed 851/960 pair; typical 3.3 V current is 9.8 mA. Exact module detection and current-limit applicability reviewed. | Verify delivered bias voltage, hot/cold current, faults, miniature-plug dimensions and standard-SMA center contacts. |
| Thermal | Fine model remains 143.800°C maximum board region and mesh-sensitive. A 60°C-rated 851 cannot meet 65°C passive boundaries. | Resolve package/local-air data, mesh/topology bounds and a realizable cooling path. Settle accessory rating or verified installation environment; passive contact improvements cannot close that 5 K rating conflict. |
| Remaining original review | All criteria retain their original questions and evidence requirements. WCA-07 still lacks a guaranteed hot/low-current optical bound. | Advance the remaining design calculations, then supplier, independent-review and physical acceptance items according to their actual scope. |

For every subsequent iteration, freeze all open criteria and every newly observed redline. Record a disposition for each finding, including the correction, evidence or exact unmet condition. Select one coordinated revision only after reviewing its power, RF, thermal, populated clearance, materials and service effects together. Regenerate all affected native/model outputs, compare every original criterion with the exact candidate, preserve failed trials and repeat. A combined finding remains open when one of its constituent requirements remains unresolved.

The next board-changing iteration should address thermal paths and digital returns together. Preserve full load/environment bounds until an enforceable alternative is designed and verified. Evaluate candidate heat paths against the ESP32 antenna region, J401 cable, C206 tab, insulation and full 1.71 mm height stack before promoting the geometry. Reconcile current package boundaries before presenting board-region temperatures as device margins.

Physical unit, supplier-process, installed harness/ground, phone/vehicle, material lifetime and environmental requirements remain explicit in the register. PR #45 is the authorized forwarding destination. Fabrication, merge, flashing and live vehicle actions are outside this iteration.
''')

report='''# I22 engineering review

Open original criteria decrease from **152 to 146**. The register now has **140 closed / 146 open / 4 not applicable**, preserving all 290 original questions, required-evidence cells and user inputs. The original user baseline remains 73/213/4. All 351 prior redlines plus six I22 findings are retained.

| Criterion closed | Evidence completing its original scope |
|---|---|
| LIB-02 | Exact manufacturer drawing-to-footprint overlays for all 12 IC/module references, independently transformed from native placement. All 151 signal lands and 12 negative controls pass. |
| LIB-08 | Exact exposed-pad net, copper, thermal-return and mask/paste review for U121/U151/U201/U301/U501, including the isolated TPS2660 RTN connection. |
| VIA-06 | Fabrication-tolerance via-current budget covers all 322 vias: 2 A per individual via, 15 µm minimum wall, 1.76 mm thickness and −0.08 mm finished-hole allowance; maximum 5.103 K self-rise against 10 K. |
| SI-12 | The original criterion permits justified analysis-only acceptance. Native RF ground-reference continuity and the retained 3,981,312-case board network support that limited disposition; external-system and supplier conditions remain separate. |
| GPS-01 | PA1616D top-view numbering, all 20 pad functions/nets and manual placement/pin-1 coordinates independently reconciled. |
| MECH-03 | Complete declared height/XY/fastener/foil tolerance stack: 1,154 populated and 776 mated checks, no intersections, 0.120 mm minimum modeled clearance. |

The exact Adafruit 851 adapter and 960 puck are now controlled inputs. The mated model no longer names the unadopted Amphenol candidate. Cable length is 150 ±3 mm to the SMA shoulder; maximum allocated OD is 2.0 mm, with separate 0.5 mm route uncertainty. The cable and plug metal remain 16.421 mm from the ESP32 antenna region. The height model now includes the previously omitted 0.51 mm capture freedom, bringing the added height stack to 1.71 mm. These are model and assembly-instruction corrections; no CAD or firmware revision is claimed.

The I20 CAD/target source remains exact: zero ERC, DRC, warnings, unconnected or parity findings in native run 34420381540; 153 fitted models; 151 automatic placements and F101/U401 manual exceptions. The 8 mm Compact TW logo from the supplied brand pack remains on F.SilkS. Existing power and RF finite bounds remain applicable. I21's 1,042 digest checks and full independent copper/export reconstruction are retained, with fresh I22 source/evidence identity checks recorded separately.

Remaining engineering work is material. The 0.125 mm thermal estimate is still 143.800°C maximum board region, with an 8.376 K last-refinement change. Adafruit 851's published 60°C ceiling does not cover the declared 65°C air case; passive cooling cannot remove that boundary conflict. Its J401 board region is estimated at 92.438–94.443°C, without asserting cable/connector temperature. All 54 digital adjacent-plane gaps remain explicit under GND-02. DFM-02 still requires exact hole/pin/hardware fit and drawing/DFM agreement; GPS-09/14 retain bias, plug and retention conditions; WCA-07 retains guaranteed optical performance; DRW-03 retains independent human review. Supplier and actual-unit/installation qualification are not replaced by calculation.

Primary package PDFs, source hashes, all twelve SVG/PNG overlays, revised mated STEP/route output, calculations, failure history and the current register are included in the recovery archive. The mated STEP contains connector/service envelope solids; the complete coax path and its tolerance envelope are documented in the route PNG/JSON. The plan and selected evidence are forwarded through PR #45. This remains a draft engineering candidate with thermal work and qualification open.
'''
save('analyses/review_i22/REVIEW_REPORT.md',report)
save('GR86_CCA_RevB_Board_Vetting_Summary.md',report)
save('README_CURRENT.md','''# GR86 CCA RVB22 I22 checkpoint

I22 closes six original criteria: **140 closed, 146 open, four not applicable**. All 290 original criteria and 357 recorded redlines remain traceable. See [the I22 review](analyses/review_i22/REVIEW_REPORT.md), [current plan](current/EXECUTION_PLAN.md), [assembly instructions](current/MANUFACTURING_AND_ASSEMBLY.md), [accessory review](current/ACCESSORY_REVIEW.md) and [thermal review](current/THERMAL_REVIEW.md).

The user has confirmed **Adafruit 851 and 960**. The revised model uses those parts, a 150 ±3 mm cable, 2.0 mm OD receiving envelope and the complete 1.71 mm populated height allowance. All 1,154 populated and 776 mated checks pass. All 12 IC/module orientation overlays and 151 signal lands pass. The exposed-pad review, conditional via heating budget and original analysis-only RF scope are complete. The six newly closed criteria are LIB-02, LIB-08, VIA-06, SI-12, GPS-01 and MECH-03.

The controlling electrical source remains **I20**, source commit `e36523bb1bd374a58b0071e761b7a46c8cc4385e`, PCB SHA256 `b6c704c0c97685a68ba708d320347148e5136c182c221d86627ff5b319954b3c`. I22 changes models, calculations and controlled instructions. Native run [34420381540](https://github.com/tranquilWorks/gr86-cca-telemetry/actions/runs/34420381540) has zero ERC/DRC/warning/unconnected/parity findings and a passing pinned target firmware build. All 153 fitted model references resolve. The original 8 mm Compact TW logo remains on F.SilkS. A future CAD/firmware change must rerun the pinned workflow.

Thermal closure remains open: the retained fine model reaches 143.800°C maximum board region and has material mesh dependence. The selected 851 adapter's documented 60°C ceiling conflicts with the 65°C modeled air case. Digital return, exact accessory bias/plug/retention, supplier construction and original physical/human qualification conditions remain visible. The dashboard load/environment requirement has not been reduced.

Use `iterations/I20_controlled_handoff/candidate_kicad`, `lanes/firmware_oil/candidate`, and `iterations/I15_service_mechanics/candidate_mechanics` for the current hardware, firmware and C05/W02/T03 source. Native exports are in `runtime/hosted/run22/extracted/native_I06_hosted`; its legacy folder name does not change its hash identity. Current I22 evidence is under `analyses/review_i22` and `analyses/mated_i22`. Evidence paths in JSON/workbook cells are relative to this recovery root.

The updated workbook preserves the original A–F questions/evidence requirements and K–M user fields. Earlier records, including the full thermal maps and native archives, remain historical evidence. PR [#45](https://github.com/tranquilWorks/gr86-cca-telemetry/pull/45) remains the draft engineering handoff; fabrication and physical qualification have not been performed.
''')
save('control/CURRENT_CONTINUATION.md','''# I22 continuation state

Resume from PR #45, branch codex/rvb22-corrective-engineering. I22 starts at I21 commit 550d87fea69147e131fd71603d02204a44c903f1. Check the forwarding receipt/current remote head before editing. I20 remains the electrical source; CAD and firmware are unchanged in I22. Native run 34420381540 passes with zero findings and all 153 fitted models. No missing native runner needs to be set up.

The user confirmed Adafruit 851 and 960 with exact links. Do not ask for those identities again. I22 replaced the stale Amphenol candidate in the mated model, adopted150±3mm length and2.0mm allocated maximumOD, and corrected1.20mm added body height to the full1.71mm stack (.25solder+.51capture+.75warp+.20deflection). Results:1154 populated checks and776 mated checks pass; minimumR408/T03clearance.120mm; RFmetalsetback16.421mm. Material, supplied plug dimensions, retention and actual installation remain conditional.

All12IC/module orientation overlays,151signal lands,20GPSfunctions and12wrong-orientation negative controls pass. ExactEPnet/mask/return review is complete. All322vias pass the2Anormal-load conditional self-rise budget(max5.103K). SI12has justified board-only analysis acceptance using the retained3,981,312RFcases. Sixcriteria close:LIB02,LIB08,VIA06,SI12,GPS01,MECH03. Newcounts140closed/146open/4NA;357redlines retained. Original userbaseline73/213/4 remains. No original criterion wording or physical/human requirement is changed.

The0.125mm thermal solve was completed inI21; do not restart it merely because historical notes say it is interrupted. It reaches143.799768Cmaximum board region and105.171457CnearC206 at4.815W,65Cair,70Clandings,70kPaand15umminimumwall. Last-refinement hotspot increase8.375893K. Mesh/topology, package and local-air closure remain open. The nearly ideal-contact comparison is unadopted. I22 adds a passive maximum-principle bound:60C-rated851 cannot meet boundaries>=65C. Its sampledJ401boardregion92.438..94.443C is not a cabletemperature. Do not silently substitute accessories or lower requirements.

All54digital adjacent-plane gaps remain; alternate plane presence does not prove acceptable return transfer. DFM02exactfinished-hole fit/drawing agreement, GPS09bias, GPS14plug/retention, WCA07guaranteedbrightness and DRW03independenthumanreview remain open. Use the complete register for all146open dispositions, not only these examples.

Next: freeze every current redline, evaluate coordinated thermal/return corrections against package boundaries, RF clearance, full populated tolerance stack and assembly/service requirements, then revise and regenerate affected native/model outputs. Preserve failed trials. No subagents, merge, fabrication, flashing, supplier messages or live vehicle work were performed inI22. Current plan and selected evidence are forwarded toPR45; full source PDFs and numerical history are in the recovery archive.
''')
for name in ['control/EXECUTION_PLAN_CURRENT.md','control/README.md']:
    save(name,(W/'current/EXECUTION_PLAN.md').read_text() if 'PLAN' in name else '# Current I22 control records\n\nRead `CURRENT_CONTINUATION.md`, `FINAL_REVIEW_REGISTER.json`, `FINAL_GATES.json` and `../current/EXECUTION_PLAN.md`. Historical recovery/runtime blockers are superseded. The source is I20; the current review/model correction is I22.\n')

observations={
 'accessed':'2026-09-10',
 'method':'Public product/capability pages read through web retrieval. Direct HTML download returned403; no raw HTML snapshot is claimed.',
 'sources':[
  {'url':'https://www.adafruit.com/product/851','observation':'PID851 is described as a150mmRG178 cable with a panel SMA connector; the page distinguishes SMA fromRP-SMA. Its datasheet link is C934-001_datasheet.pdf.'},
  {'url':'https://www.adafruit.com/product/960','observation':'PID960 is a GPS active antenna with5m cable and standardSMA. Its adapter recommendation links directly toPID851. Technical specifications link GPS-01.pdf.'},
  {'url':'https://jlcpcb.com/capabilities/pcb-capabilities','observation':'Published through-hole diameter tolerance is+0.13/-0.08mm; holeposition±0.05mm; board thickness>=1mm has±10%tolerance. Through-hole copper is stated as18um average. These public capabilities are not approval of this design or a15um guaranteed minimum wall.'}
 ]}
save('analyses/review_i22/sources/WEB_OBSERVATIONS.json',json.dumps(observations,indent=2))
print('I22 current reports, assembly instructions and continuation authored.')
