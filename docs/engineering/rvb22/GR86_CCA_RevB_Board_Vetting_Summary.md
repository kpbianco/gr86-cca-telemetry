# GR86 CCA RVB22 — corrected source handoff

Current handoff: **I06**, containing the unchanged **I04 electrical revision**. PCB SHA256: `c2297b8c857d54f525667e9b2dace5f5f538b7c8d651370cc1b40fa2bdfb39c1`. Firmware: **2.1.4-revb-recovered-20260909**, source-manifest SHA256 `3cc361dbe9ed9dd112bf889d5a3bece277ac45287a5348ebc2ab4255bf74a90f`. The source files and executed analysis are included; no current target binary or fabrication release is claimed.

The work progressed beyond auditing: CAD, firmware, calculations, harness and mechanical/assembly instructions were revised and checked together. The final fixed-copper screen reports **zero cross-net gaps below 0.15 mm, zero lost previously connected pad groups and zero new courtyard overlaps**. It covers 4,204 explicit copper objects and 158 courtyards. Fresh native zone refill and ERC/DRC have not run.

The original review criteria remain **110 closed / 176 open / 4 not applicable**. The recovered newer RVB21 checkpoint began at 114/172/4, superseding the earlier quoted 213-open snapshot. This pass closes 21 previously open criteria and reopens 25 previously overclaimed closures. These are evidence criteria, not separate board defects. Completed bounded calculations are identified independently from a required measurement or actual supplier record.

## Implemented corrections and verification

| Area | Current correction | Executed evidence and practical limit |
|---|---|---|
| RF | Centered 0.7 mm straight entry into the U.FL signal land; exact Hirose land/keepout retained; 1.6 nH matching part and continuous relief. | Final added 5,578,752 finite cases pass: return loss≥10.281dB, insertion loss≤0.861dB. Includes zero-resistance and thermal-strip loading sensitivity. Exact Adafruit identities and installed RF behavior remain conditions. |
| Power | 3.37 V regulator setpoint; 15 µH inductor; 470 µF/6.3 V bulk capacitor with 82 mΩ damping; coordinated OV thresholds; wide parallel 3V3 rail; exact compact 10 mΩ R153. | Final 3,456 actual-tap cases pass voltage and the conditional controller comparison: 3.122326–3.594481 V; minimum phase 45.638°. Unpublished silicon behavior is not proven by the generic controller family. |
| Startup | Explicit upstream/downstream state, regulator current bounds, input filter and actual R158/polymer charging loss. | 4,096 cases plus three timestep refinements, zero modeled recovery failures; maximum 5 V rail 5.211487 V. R158 short-pulse supplier/thermal correlation remains documented. |
| Oil input | C532 10 nF OUT-to-GND; model includes actual distributed C504-land and In1 contact paths; no invented private-return short. | 18,432 cases pass the declared fault network; certified OUT lower bound −0.10768 V before the separate private-return offset condition. Independent nodal/numerical checks included. |
| Firmware | 100 Hz acquisition/50 Hz publication; gap invalidation; nonblocking framed GPS writes; protected virtual channels; corrected rate and calibration arithmetic; consistent identity. | Fresh host/source tests and actual raw-CAN encoder checks pass. Target compiler, opaque drivers, phone delivery and actual timing remain unverified. |
| CAN / harness | ASC mapping; exact selected AVSS wire, connector exit and tooling rules; one full-current F1 power return; complete branch loading; tap-aware phone profile. | 14,400 branch cases settle within 1% by 137.6 ns in the declared bus envelope. Early edge disturbances are retained. Continuity, isolation, crimps and actual vehicle semantics remain physical conditions. |
| Mechanics / assembly | Captured ears, reinforced carrier, exact tie/slot clearance, insulated RF mounting, thermal wings and T01 strap; F101 local after reflow; 13 specified filled/capped vias. | Geometry, source preservation and finite structural/retention models included. Primary maximum-height bounds cover all 153 fitted references. Complete native populated solids and full-load thermal heat paths remain unresolved. |
| Handling | Exact current 74 MPN / 153 populated-reference inventory and primary handling research. | Primary moisture classifications available for 58 MPNs; remaining 16 have explicit profile/source/receiving conditions, not invented MSL 1. Actual lots/profiles are not supplied. |

## Material results retained for end review

**Full-load thermal safety remains an engineering gate.** The dimensioned T01 path is allocated 10 K/W to a 70°C landing with 65°C bulk air. The source-position screen gives a C206-region temperature of 131.595°C at ideal 96 µm continuous copper, above 125°C; ideal 160 µm gives 116.359°C. Actual filled-copper coverage, interlayer vias and package heat transfer must determine whether the selected construction meets the required path. The average board temperature does not establish a pass. The final mechanical instructions control the actual envelope and cable and service clearances.

**EMI/ESD models are estimates with exposed limits.** The representative conducted model reaches about 95 dBµV RMS harmonic lines at a stated fixture corner; it is not compared to an invented compliance limit. Port protection is geometrically intercepted in five checked paths, but fast clamp/return inductance and trigger-spike cases cross static stress screens. Those cases, inverse impedance/current limits and susceptibility gaps are retained in `lanes/emc/EMC_DESKTOP_REPORT.md`.

**The CAN model retains a fast-edge finding.** In the updated connector-section model, 196 cases with the deliberately fast 2 ns source edge cross the 0.9 V screen repeatedly. Refined cases reach a post-crossing valley of about 0.737 V for a below-0.9 V interval up to 5.263 ns; none recrosses 0.5 V. The refined receiver peak is about 2.318 V. This does not establish an RXD glitch or vehicle error: actual receiver hysteresis, filtering and installed edge timing are not included. Those inputs must bound the event, or branch damping/topology needs another revision. Favorable settling by the late sample does not erase this result.

**Native execution is a real current roadblock.** Actual preflight returned `BLOCKED_RUNTIME`: no callable pinned KiCad/pcbnew/Arduino environment. The bundled runner rejects fake zero-exit/no-output programs. Older Gerbers, compiled images and native passes remain historical and do not certify this revision. Run `control/EXECUTION_PLAN_CURRENT.md` instructions to produce the next concrete native redline batch.

The remaining actual-unit and supplier conditions include exact Adafruit accessory IDs; fused switched power/return endpoints; unintended sensor/coax/chassis/programmer bonds; populated continuity/crimp/retention; finished JLC construction and 13 filled/capped vias; assembly profiles/lot handling; actual RF/vehicle/phone/thermal behavior. Models cannot inspect an unmade solder joint or assert a supplier's accepted process.

## Review package and next iteration

- Current CAD: `iterations/I06_harness_handoff/candidate_kicad`.
- Matched firmware: `lanes/firmware_oil/candidate`.
- Current plan and remaining actions: `control/EXECUTION_PLAN_CURRENT.md` and `control/FINAL_GATES.json`.
- Full original-criterion / redline evidence: `control/FINAL_REVIEW_REGISTER.json` and the accompanying workbook.
- Native runner: `runtime/run_native_candidate.py`.
- Current source-derived review BOM / placement tables: `verification/independent/I04_review_tables`.
- The recovered baseline, available failed iterations, reconstruction notes, source files and model scripts are included. Reconstructed bytes have new hashes; lost historical artifacts are not represented as newly reproduced passes.

[Draft engineering PR 45](https://github.com/tranquilWorks/gr86-cca-telemetry/pull/45) carries the current forwardable work. It remains a draft; no merge, order, fabrication or live flashing was performed. Record the next observed redlines, correct them in one coordinated revision, and rerun the combined checks. The package does **not** claim zero total open criteria or completed physical qualification.
