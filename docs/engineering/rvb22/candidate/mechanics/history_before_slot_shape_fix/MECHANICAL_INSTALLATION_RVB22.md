# RVB22 C03/T01 installation instruction

This instruction controls the final C03 thermal carrier. Earlier C02/C03 coordinates remain historical in the redline history and comparison models. Use `GR86_RVB_CARRIER_C03_THERMAL.scad`, `MECH_GEOMETRY.scad`, `THERMAL_CONTACT_T01.scad` and `HARNESS_CORRIDORS_C03.scad` together.

## Board capture and structure

Four new outboard ears and4.4mm NPTH holes positively restrain the board inX/Y with shoulder sleeves; washers captureZ. Sleeves are4.00±0.05mm OD,2.2mm ID and1.90±0.05mm long; washer OD≤6mm. PCB thickness1.44–1.76mm gives0.09–0.51mm Z freedom. Do not replace a capture with a friction pad. The1mm total motion/tolerance allocation leaves0.75mm to the nearest original courtyard. The thermal contact has its separately controlled antenna clearance.

All four M3 carrier mounts are required: (−9,−6), (−9,47.5), (57,−11.5), (57,54.5)mm. The3mm insulating FR4 carrier has main boundsX−7..66.5,Y−15..58mm, with left mounting wings extending toX−13. Its central and thermal-wing windows remain open. The right mounting-hole minimum edge ligament is1.65mm; the minimum window ligament is1.35156mm after the stated0.15mm tolerance allocation.

The structural screen uses E≥10GPa, flexural allowable≥150MPa, bearing allowable≥50MPa and local shear allowable≥15MPa as engineering acceptance requirements. These are not invented supplier certificate values. The100g base assembly/20g static case produces97.29MPa worst single-ear bending stress; a125g total including thermal hardware produces121.61MPa. The outboard beam and kinematic CTE calculations do not establish vibration modes or solder-fatigue life. The maximum modeled C206 far-edge deflection is0.1601mm within a0.20mm allocation.

## Envelope and service

The final thermal assembly's basic bounding box is**108.454766×73×36.50mm**, including carrier, thermal landing and its fastener allowances. It includes the ESP's6mm PCB-antenna overhang. Actual dashboard cavity, mating plugs, extraction tools, cable bends, antenna exclusion and cooling airspace require additional clearance; no actual vehicle fit is asserted.

Reserve at least7mm clear depth beneath the module region. C206's maximum4.3mm body plus0.25mm solder,0.51mm capture freedom,0.75mm warp and0.20mm local deflection leave0.99mm. C206 lies beyond the carrier's X66.5 edge. Other populated package/lead/solder heights and model-path gaps are enumerated in `FINAL_COMPONENT_ENVELOPES.json`; a full native populated-solid check remains an execution gate.

For U.FL service, disconnect power and F1, release the vehicle mounts and thermal-landing bolts, then remove the carrier captures on the bench while supporting the PCB and cold terminal together. Provide≥20mm tool access below J401 after carrier removal. Do not pull the cable or peel the hot terminal to remove the carrier. Replacing the thermal adhesive requires its controlled preparation and contact acceptance again.

## Harness restraint

Primary F1 harness slots are centred at **X−3.5,Y31/37mm** in the widened7mm left spine. Auxiliary coax service slots are **X63,Y33/39mm**. The old right slots atY16/22 are removed to clear the thermal packet. Nominal slots are3×1.5mm rounded: require finished width2.9–3.1mm, height1.4–1.6mm and no burrs. The primary slots retain at least1.95mm side webs. At an allocated30N local lip load, nominal shear is5.128MPa in a3mm×1.95mm web.

Select **HellermannTyton T18R0HSC2 /111-01905**, with the conservative105°C continuous limit. The maximum strap is2.7mm wide and1.3mm thick; the maximum head requires a5.2×5.2×3.9mm reserved volume. Its catalogue80N tensile strength does not prove installed hot grip.

The primary round bundle must be3–6mm loaded diameter for the evaluated retention case. ReserveX−7..0,Y25..42,Z−16.3..−9mm for the6mm maximum bundle plus1.3mm strap. Park the heads beside the bundle atX−12..−6.8, centred onY31/37, Z−12.9..−9mm; do not stack a3.7mm head underneath the bundle. Preserve the CAN pair's15–25mm twist and30mm minimum bend radius; the bend sweep needs additional external space. The AVSS CAN jacket must remain within its80°C wire heat-class limit, including heat received from the board.

The independent retention report uses the actual wrap: θ=4atan(D/s), with exit spacing≤7.6mm andµTθ≥10N. For loadedD≥3mm and retainedT=20N, requireµ≥0.333 and hot loop strength≥60N. The corresponding jacket pressure must also be acceptable; the report provides the inverse table. These are controlled material/assembly conditions, not measured friction or creep values. A loose outer sleeve that permits individual-wire slip is insufficient. A qualified positive harness shoulder may replace the friction requirement, but none is represented as fitted.

At the right anchors, retain≤2.8mm jacket/strap build with localized3.9mm head volumes. The right path is for the U.FL service loop; external coax pull is carried by the captured insulating SMA bulkhead. Any accessory cable routed near the approximately90°C thermal spine must have an applicable temperature rating; the tie rating does not supply the cable's rating. Exact Adafruit adapter applicability remains controlled by the interface report.

## Thermal and RF conditions

T01 contacts the board underside atX66.6..72,Y11.8..28.8mm through insulating3M8805 and soldermask. Its specified tape/mask/40-foil packet/landing conditions budget9.967K/W, modeled as10K/W to a landing≤70°C. The actual antenna startsX88.254766; after0.3mm positioning and0.5mm foil-bow allocation, metal clearance is15.454766mm. This is a documented revision of the old broader external-metal exclusion. No direct shield-can contact or galvanic PCB-to-chassis bond is introduced.

The controlling environment is a dashboard installation with65°C local bulk-air/radiant screen. The user's120°F estimate is not a hotsoak measurement. No engine-compartment exposure is assumed. The full-load thermal sensitivities retain the failing low-conductivity cases. Actual native copper/interlayer/package heat flow must meet the T01 inverse requirements before claiming full-load thermal safety. The local ESP ambient limit, capacitor hot-ripple conditions and installed cable temperature limits remain applicable.

Keep water paths away from the open electronics and preserve the windows, antenna clearance and removable service architecture. The separate SMA and puck isolators prevent exposed coax metal from becoming an unintended chassis return. Do not isolate the Honeywell threaded housing from its required ground. Exact puck shape, rigid capture features, cable exit and vehicle cavity remain applicability conditions.
