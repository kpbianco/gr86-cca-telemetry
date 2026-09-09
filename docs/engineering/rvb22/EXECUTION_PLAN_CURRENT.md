# GR86 Rev B corrective engineering — current execution plan

The active candidate is I11 on draft PR45, following verified I10 commit4ce50c91be6f54ff3e3b17194978f13fbb82cfa9. I10 returned12 library warnings and no other native findings; I11 adopts the reviewed variants and awaits native reevaluation. The objective is zero unaddressed engineering redlines, with each original review criterion retaining its actual evidence and any model, supplier or installed-unit condition. No criterion is closed merely to change the count.

## Fixed design inputs

- Honeywell MIPAN2XX150PSAAX, distributor 480-MIPAN2XX150PSAAX-ND: 150 psi sealed-gauge, 5 V ratiometric pressure sensor.
- GR86 ASC connection per Timurrr, using the controlled single full-current return and fused switched power requirements.
- Adafruit U.FL–SMA adapter and external GPS puck. Exact accessory PIDs remain an identity condition; 851 and 960 are inferred.
- JLCPCB construction and dashboard installation. The board is not assigned an engine-bay environment.
- Firmware, CAD, mechanics and assembly may change. Power analysis uses executable circuit and numerical models because ADS is unavailable; private device behavior retains explicit bounds.

## Iteration rule

For each batch: record all known redlines and source hashes; resolve interacting items in one coordinated candidate; run native and affected analytical checks; retain failures; freeze the resulting evidence and repeat. Intermediate source compatibility fixes are permitted when needed to expose the complete native findings. They do not imply that the rest of the board has passed.

## Completed corrective scope

The current source incorporates the oil clamp/OUT capacitor and return corrections; cooperative GPS diagnostics and firmware arithmetic fixes; centered straight 0.7 mm U.FL entry and revised RF matching; regulator, storage, damping and overvoltage changes; compact current shunt; captured mounting ears and thermal-contact/carrier changes; exact harness return and relaxed connector exit; and assembly/part reconciliation. I03's conflicting copper move was rejected and retained; I04 corrected it. I06 corrected the harness note without changing electrical objects.

Actual target compilation now passes with Arduino CLI 1.3.1, ESP32 core 3.3.6 and NimBLE 2.3.6. Returned ELF/BIN and input hashes are verified in runtime/hosted/TARGET_BUILD_VERIFICATION.json. This is build evidence, not target execution.

I07 repaired source parser compatibility and oil label warnings. I08 reduced native DRC from181 findings to13. I09 has zero ERC/DRC errors, zero unconnected items, zero schematic-parity findings and unchanged pad/net identities. Its13 warnings are12 library-copy mismatches and one obsolete rail branch. I10 removes the duplicate/backtracking rail geometry and exports exact native library variants for review; it does not suppress checks.

## Remaining execution sequence

1. **Native source and manufacturing checks.** The hosted review route is working with KiCad/pcbnew 9.0.9; no further user setup is needed. Inspect I10 results, adopt reviewed native footprint variants with matching schematic IDs, and recheck all native findings. Regenerate matching Gerbers, drills, placement/BOM, drawings and populated STEP. Never substitute old exports.
2. **Thermal engineering.** The actual I09 native filled copper has now been extracted with fractured holes preserved, explicit copper included, and all NPTH/plated holes removed from material. Completed1.0/0.5mm solutions give C206 board-region maxima110.188/111.400C and hottest board regions129.074/130.559C under4.815W,65C air,70C landing. The0.25mm process ended without a captured completion; no result is claimed. Resolve mesh sensitivity, package heat paths and full-load margins; revise cooling or an enforceable operating condition when necessary. Do not quietly reduce the requested load. Exact fabrication and installed landing/air conditions remain explicit.
3. **Populated mechanical clearance.** Use native component placement plus primary height bounds for all 153 fitted references. Complete carrier, contact, mounting, mated connector, wire-exit and service envelopes. Missing vendor models can be replaced by conservative dimensioned envelopes with documented uncertainty; any unresolved collision is a redline.
4. **Combined regression.** Recheck changed copper connectivity/isolation, rail and oil models, RF launch and nearby thermal metal, return paths, assembly outputs and firmware only where source or assumptions changed. Preserve failed cases and bind every result to the candidate.
5. **Release records and final review.** Reconcile all original criteria, redlines, evidence hashes, current plan and source/export manifests. Report design closures, conditional model conclusions and actual supplier/installed-unit acceptance separately. Review remaining conditions with the user; do not claim a fabricated board's plating, solder, crimps or vehicle behavior was measured.

## Inputs that can narrow the remaining conditions

| Input | Work it enables |
|---|---|
| Actual Adafruit order links or labels | Exact RF bias, connector, cable and puck applicability |
| Actual fused switched power endpoint and single return path | Installed power/ground and unintended-bond verification |
| JLC finished stackup, 13 filled/capped via capability and process acceptance | Bind geometry, thermal and assembly assumptions to the delivered construction |
| Dashboard air/landing bounds and installation envelope | Bind full-load temperature and retention/clearance models |
| First-article or supplier measurements when available | Correlate private device behavior, thermal/RF estimates and as-built joint integrity |

These inputs do not stop the available CAD, firmware and numerical work. Draft forwarding and review jobs are authorized; merging, manufacturing, flashing and vehicle fault injection are not part of this execution.

## Current interacting mechanical redlines

The153-fitted-part prism screen executed465 checks using primary maximum heights and current courtyard-plus0.3mm XY envelopes. Four possible T01 foil interferences remain under maximum bow: R408, C524, L406 and J401. Manufacturer XY coverage, mated connectors and installed fit remain conditions.

The actual T01 bend reachesX73.1mm while its RF calculation assumedX72mm. Including0.3mm placement and0.5mm bow gives14.354766mm antenna setback against15mm required. This overclaimed geometric closure is reopened.

A T02 geometric trial removes all modeled foil/component intersections and restores15.054766mm antenna setback with the full0.8mm foil allowance. However, reduced contact area increases allocated path resistance from10 to13.766K/W. The trial is not adopted until the coupled thermal design is satisfactory. See analyses/populated_envelopes/T02_GEOMETRIC_TRIAL.json and FOIL_CORRECTION_STUDY.png.

## Evidence and zero-open accounting

Native rule findings are not the review-criterion count. The formal290-criterion register remains the I06 snapshot (110 closed,176 open,4 not applicable) pending adjudication of later evidence. Do not report the zero native-error count as zero overall redlines. Each criterion must either have source/calculation evidence sufficient for its stated claim or retain an explicit external acceptance condition. A simulated continuity/isolation conclusion applies to the CAD and modeled construction; solder, plating, crimps and installed behavior are not observed by it.
