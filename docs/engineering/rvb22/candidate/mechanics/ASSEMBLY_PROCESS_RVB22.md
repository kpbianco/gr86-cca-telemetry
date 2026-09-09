# RVB22 assembly and fabrication correction

This source-controlled instruction supersedes RVB21 process steps 6–9 only where explicitly changed below. Receiving, exact-part moisture clocks, cleaning compatibility, inspection and executed traveler records remain applicable. The actual orderable BOM, board hash and regenerated manufacturing files must accompany the job. This is a specified process, not a completed factory traveler.

**F101 remains Littelfuse 0885001.DR, LCSC C2879865, factory fitted quantity one.** Do not substitute a different fuse to resolve soldering. The 885 primary datasheet recommends 255–260°C reflow; ESP32-S3-WROOM-1 permits a 235–250°C whole-board peak, so no common recommended interval exists. RVB22 adopts a later factory local operation instead of claiming the old profile compatible.

1. Bottom SMT first. Top SMT second, with U201 installed only on the second full-board pass. Use a developed profile with component-body peak target 243–245°C, subject to every selected part's full ramp, soak, liquidus duration, cooling and cycle limits. L406 and ESP must remain within their own published limits. A target is not an executed thermocouple record.
2. Omit F101 from both global paste layers and automatic placement/CPL; preserve it in the fitted factory BOM and add it to the secondary operation list. `rebuild_mechanics.py` makes these source edits and provides the matching `RevB:Fuse_Littelfuse-NANO2-885` library footprint so a library update cannot restore the old paste accidentally.
3. After both whole-board passes, factory fit F101 with a masked local mini-wave/selective operation conforming to Littelfuse's published **260°C peak and at most 3 seconds** wave exposure, or a documented manufacturer-supported equivalent. Shield/support the board and verify adjacent components, module, plastics and solder joints stay within cumulative limits. Tool setpoint and hot-air temperature are not component-body temperatures. This instruction does not authorize an arbitrary 350°C iron process. Factory must accept the actual local SMD geometry/process; generic availability of manual assembly does not establish that acceptance.
4. F1/J202/J203 remain factory secondary solder operations under their exact component requirements. U401 remains the sole owner-installed component. No later whole-board oven pass is introduced.
5. F101 MSL1 is supported by the exact 885 datasheet; it is not evidence that the full assembly is MSL1. The new KEMET T598X477M006ATE025 C206 is MSL3, X/7343-43, maximum body 4.3mm. Preserve its actual dry-pack label and cumulative floor-life/bake rules through both passes and any local rework.

## Fabrication correction

Use the authorized four-layer 1.6mm JLC041621-7628 stack, FR4 Tg170, nominal copper 70/30/30/70µm and dielectric 0.203/1.030/0.203mm, ENIG, green LDI mask and white legend. Manufacturing thermal analyses state their minimum-copper assumptions separately; nominal stack values do not guarantee minimum plated copper.

The **12 U201 pad41 vias require filled, capped and planarized via-in-pad fabrication**. The prior drawing note calling for tenting/no-fill is superseded. Preserve the existing nine U201 paste windows; stencil 0.10–0.12mm remains a process choice to be checked with the fine-pitch/large-pad deposits. No F101 global paste is permitted. The updated PCB is 88.254766×61.275846mm; its bounding box fits a120×75mm one-up panel with15.872617mm horizontal and6.862077mm vertical half-margins. Tabs, tooling and fiducials must remain outside four captured ears, the thermal wings and antenna projection. This bounding-box calculation does not itself design or approve panel tabs.

## Source evidence

- `baseline/GR86_CCA_RevB/evidence/manufacturer/littelfuse_885.pdf`, pages2–3: reflow, wave process, package, MSL1.
- `baseline/GR86_CCA_RevB/evidence/manufacturer/espressif_esp32_s3_wroom1.pdf`: module process and mechanical limits.
- `sources/KEM_T2073_T59X.pdf`: exact family orderable X-case, 4.3mm, MSL3 and thermal/ripple scope.
- RVB21 per-MPN process/lifecycle matrix remains a source list, not evidence of completed incoming or reflow acceptance.

## Final I04 via-in-pad addition and R153

The final compact R153 is **Vishay WSLP0603R0100FEA**, maximum body1.774×1.014×0.533mm, with the exact manufacturer1.02mm square lands at±0.76mm centres. It supersedes WSL1206R0100FEA for this reference only. Exact-part process/MSL and BOM source are governed by the final receiving matrix.

The filled-and-capped schedule is now **13 vias total**: the existing12 U201 pad41 vias, plus a thirteenth source-rail via at **X41.4,Y21.0mm**, net **+3V3**, copper diameter0.6mm and drill0.3mm, associated with R153 pad2. This is not a GND via. Record its final UUID from the root rail delta in the manufacturing schedule.

This thirteenth via partly overlaps the R153 copper land. Fill its complete hole, copper-cap it and planarize it with the surrounding land before final finish. Preserve the intended land soldermask/paste opening over the planar cap; a tented via beneath an open pad aperture is not the specified construction. Any annulus outside the pad may remain mask covered. Require no recess, unfilled hole or raised cap that prevents the small resistor from seating or causes solder loss. Native mask/paste and final fabrication DFM must verify the exact land/via registration. The existing12 U201 thermal vias and nine paste windows remain their own unchanged subgroup.

Final I04 R153 centroid isX41.0,Y21.0mm; pad2 centroid isX41.76,Y21.0mm. C157 remainsX36.2,Y22.0mm. The resulting nearest copper-land gap is1.48mm, replacing the rejected0.20mm compact-courtyard trial.
