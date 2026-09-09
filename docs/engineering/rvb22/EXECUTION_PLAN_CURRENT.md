# GR86 Rev B corrective engineering — current execution plan

The active working candidate is I08, following I07 native evaluation on draft PR45 at commit ab52ea36230885688f92c235ea1d5289245703c7. The objective is zero unaddressed engineering redlines, with each original review criterion retaining its actual evidence and any model, supplier or installed-unit condition. No criterion is closed merely to change the count.

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

I07 reflows the original one-line PCB below KiCad's line-reader limit without changing its parsed tree, and promotes two oil local labels to the already-used global names. Native netlist parity, refill, ERC/DRC and matched exports are being evaluated by workflow run 34381221292. See iterations/I07_source_compatibility/FORWARDED.json and SOURCE_DELTA.json.

## Remaining execution sequence

1. **Native source and manufacturing checks.** The hosted review route is working with KiCad/pcbnew 9.0.9; no further user setup is needed. Inspect the I07 results, compare native schematic netlists, inventory every real ERC/DRC finding, and correct the complete batch. Regenerate matching Gerbers, drills, placement/BOM, drawings and populated STEP. Never substitute old exports.
2. **Thermal engineering.** Replace provisional pours with native filled copper. T02 already models four source copper layers, three dielectrics, 299 plated barrels and side-specific heat allocation. Its 0.25 mm estimate is 111.877 C at the C206 board region and 130.848 C at the hottest board cell; increasing mesh differences prevent closure. Resolve local mesh sensitivity, package heat paths and full-load margins; revise cooling or an enforceable operating condition when necessary. Do not quietly reduce the requested load. Exact fabrication and installed landing/air conditions remain explicit.
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

## I08 corrective batch underway

I07 native evaluation loaded/refilled the board, returned zero ERC findings and an identical schematic netlist, but exposed 181 DRC findings. All are frozen in iterations/I08_native_corrections/REDLINE_FREEZE.json. Seven pad-net losses arose from late net declarations. I08 orders the declarations first and gives the four mechanical mounting pads stable UUIDs; the runner now compares all 566 pad/net identities before and after native processing.

I08 repairs the crowded power-area clearance routes and the inner rail crossing the U.FL keepout, corrects zone priorities and the oil via diameter, trims open stubs, corrects silk clipping and distinguishes mechanical NPTH holes from copper pads in the ear rule. The full source custom-clearance screen now has zero findings without reducing the clearance rules. The explicit-copper connectivity screen retains one changed GND group after replacement of a narrow branch by native planes; native plane connectivity and return-path regression are mandatory. This is not a continuity closure yet.

Exact footprint-library reconciliation, populated models and analytical effectivity remain part of the current batch. The I07 STEP was a valid file with missing component models; model aliases are being bound to the installed package and missing-model logs now fail the corresponding runner postcondition. The original criterion counts remain unchanged until this candidate's evidence is adjudicated.
