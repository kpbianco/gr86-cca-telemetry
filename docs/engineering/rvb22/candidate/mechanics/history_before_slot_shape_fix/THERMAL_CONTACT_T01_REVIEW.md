# T01 thermal correction, bound to final I03

**A concrete insulated thermal strap is staged, with a10K/W design budget from PCB-back contact to the mounting landing. Actual-source full-load thermal safety remains unpassed.** The required native copper/interlayer/package calculation is an explicit execution gate. This correction does not quietly assume an isothermal PCB or reduce the allocated load.

## Geometry and construction

The contact isX66.6..72.0,Y11.8..28.8mm, area91.8mm², on the PCB underside. The initial enlargedY11.0 candidate intersected J401's final courtyard; it was rejected and moved before adoption. Final-source courtyard clearance is0.32mm toR408,0.50mm toJ401 and0.50mm toL406. Maintain contact cut/placement error within0.20mm relative to the PCB. These are courtyard bounds; populated connector/cable/solder solids still require native execution.

Use3M8805 nominal0.127mm electrically insulating adhesive film against the existing soldermask. The selected film's published thermal impedance is typical, and applies to its stated test surfaces; it is not a guaranteed installed value. Forty annealedC110 copper laminae, each0.050–0.055mm thick and17mm wide, form an unbonded flexible packet with bonded terminals only. The specified maximum developed heat path is34mm; the drawn centreline is27.824mm. Weld/braze the terminal packets before attaching the film to the assembled board. Do not bond the free foils into a rigid laminated beam.

The formed packet descends outside the carrier's right edge and turns beneath it. `THERMAL_CONTACT_T01.scad` is its volume envelope, not a claim of a manufactured laminated solid. The landing is a2mm aluminium plate atX51..66.5,Y4..35.8,Z−15.327..−13.327mm, withM3 holes at(54,7.5)/(54,32.3). The heads are away from the foil and cable anchors. The bolt/washer/nut allowance increases the basic thermal assembly envelope to**108.454766×73×36.50mm**, before vehicle mating/tool/cable/airspace volumes. All original board captures remain required. The100g base assembly allowance must include the strap or be raised to125g; the125g/20g one-ear flexural screen is121.61MPa versus the declared150MPa material requirement.

`GR86_RVB_CARRIER_C03_THERMAL.scad` removes only the auxiliary right slots atY16/22, which would otherwise route cable ties through the thermal packet. It keeps the primary left pair (finalX−3.5,Y31/37) and moves the right pair toY33/39 to clear the selected tie heads. Jacket loops remain≤2.8mm, with localized5.2×5.2×3.9mm head volumes. Route the U.FL cable away from the strip toward−Y. The final physical bundle may not cross the foil, sharp metal edges or ventilation windows.

The contact does not provide primary PCB retention. Adhesion, peel, hot creep and packet spring force require the actual surface/process specification. 3M's published70°C static and dynamic shear data and heat-aged room-temperature tests do not establish a90°C installed creep guarantee. This material/process acceptance condition is retained rather than inferred from room-temperature tensile strength.

## Finite thermal budget

| Segment | Allocated conditions | Rθ, K/W |
|---|---|---:|
| Existing soldermask | thickness≤30µm, k≥0.2W/mK, full91.8mm² contact |1.634|
| 3M8805 installed film/contact | published typical3.486K/W; installed requirement≤4 |4.000|
| Copper packet | length≤34mm, width17mm, total thickness≥2mm, k≥300W/mK |3.333|
| Bonded terminals and cold landing | combined installed requirement |1.000|
| Total | round up for model |**10.000**|

Film/contact, mask conductivity and cold-landing values are explicit engineering acceptance allocations, not manufacturer maxima. Copper Development Association reportsC11000 approximately391W/mK at20°C; the300W/mK allocation retains substantial margin but is still a specified hot-material minimum. The landing is constrained to≤70°C under load. At65°C surrounding structure, a≤1K/W landing-to-structure path could carry the entire4.815W allocation with≤4.815K rise. Its actual metal/contact geometry must provide that path; a small isolated bracket in65°C air is not assumed to remain70°C.

The original4.790W heat corner mixed minimum5V for conversion current and maximum5V for input power. Power analysis also provides the physically coupled4.5321W case for0.75A main/50mA GPS/7.5mA oil and allocated80% converter efficiencies. This iteration adds25mW magnetic-loss headroom, retaining both4.5571W coupled and4.815W conservative screens. No published average-current number is turned into an enforced firmware limit.

`THERMAL_CONTACT_T01_MODEL.json` evaluates4.815W with actual source-region locations,0.5mm finite-volume mesh,70kPa,65°C bulk air/radiant surroundings,ε0.3,70°C landing and10K/W contact. It sweeps equivalent continuous copper96/120/140/160µm. These values are conductivity sensitivities, **not native filled-copper measurements**. The model idealizes perfect interlayer coupling and deposits all main-rail heat at the U201 exposed-pad region. Unspecified module-to-shield conductance is never credited. The previous models' carrier/shadow geometry remains conservative C02 geometry; no extra C03 carrier area is credited.

A source-position result below125°C at C206 is insufficient alone: add capacitor self-heating and any residual package/interlayer thermal rise. KEMET'sX-case ripple table implies60K/W only under its reference mounting condition; it is not a universal PCB mounting resistance. Above105°C, its31.25mW hot ripple-power limit must be used with the actual hot ESR. The power lane owns that electrical RMS bound.

The manufacturer85°C ESP32-S3-WROOM-1-N8R2 limit refers to ambient immediately outside the module, not shield/case temperature. Installation must maintain the local module-air boundary within its rating; the selected70kPa/65°C bulk-air model is an installation condition, not a measured dashboard result. Current hardware does not expose software control of all CAN/GPS/oil loads, so ESP deep sleep alone cannot be credited as an enforced whole-board thermal limit.

## RF, isolation and service

Actual antenna geometry begins atX88.254766. MetalX≤72 plus0.3mm module/cut/placement and0.5mm permitted flex bow retains**15.454766mm**, meeting the official Espressif≥15mm housing-clearance recommendation. This is a controlled revision of the old broaderX68.029766 external-metal exclusion. It does not prove unchanged antenna match, throughput or range. The strap is chassis bonded at the cold end and electrically insulated from PCB copper; it makes no shield-can contact. Filmεr and trace-to-metal loading remain bounded by the RF lane.

For U.FL service, disconnect power/F1, release the vehicle mounts and thermal-landing bolts, then remove the carrier captures on the bench while supporting the board and cold terminal together. Provide≥20mm free tool access below J401 after carrier removal. Do not peel the hot terminal or pull the cable to remove the carrier. Replacing the adhesive requires its controlled surface preparation and thermal-contact acceptance again.

Primary references: [3M8800-series technical data](https://multimedia.3m.com/mws/media/122119O/3m-thermally-conductive-adhesive-transfer-tapes-8800-series.pdf), [C11000 material data](https://alloys.copper.org/alloy/C11000), [Espressif module placement guidance](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32s3/pcb-layout-design.html#general-principles-of-pcb-layout-for-modules-positioning-a-module-on-a-base-board), [KEMETT59X family data](https://content.kemet.com/datasheets/KEM_T2073_T59X.pdf).

## Executed inverse result

At4.815W, the final strip produces C206-region maxima131.585/124.093/119.702/116.349°C for96/120/140/160µm ideal continuous copper respectively. Thus the first sampled sheet condition leaving at least an allocated3K for capacitor/body rise is140µm, equivalent0.04236W/K per square at the assumed300W/mK copper conductivity. Its remaining2.298K corresponds to only0.885K/W additional local module-path resistance;160µm leaves5.651K or2.177K/W. These tight budgets expose why unmodeled interlayer/package resistance cannot be ignored. No guaranteed temperature pass is declared.

The final left-spine widening to7mm receives no extra cooling credit in the retained thermal models. It only increases the tie-slot load-bearing webs.
