# RVB22 corrective engineering plan and current handoff

The current handoff is I06, carrying the electrically unchanged I04 PCB. The goal remains zero unaddressed engineering work. Completed source checks, conditional models, native execution and actual acceptance retain separate evidence states; the original 290 criteria are preserved.

## Inputs now incorporated

Honeywell MIPAN2XX150PSAAX 150 psi sealed-gauge sensor; GR86 ASC tap per Timurrr; Adafruit U.FL–SMA cable and external GPS puck; JLCPCB fabrication; dashboard PCB environment. ADS is unavailable in this runtime, so executable nodal, differential-equation and field models are used. Exact Adafruit product IDs are still an explicit input condition.

## Iteration record

| Iteration | Recorded problem / corrective scope | Disposition |
|---|---|---|
| Baseline RVB21 | 172 of 290 original criteria open; CAD and firmware candidates differed from older native/compiled manufacturing package. | Immutable baseline retained. |
| RVB22 lane revisions | Oil response/clamp topology, firmware blocking/rate arithmetic, RF launch/matching, power/setpoint/storage/OV, return model, carrier and assembly corrections. | Source and executable evidence reconstructed after unexpected working-file loss; historical lost hashes not reused as current passes. |
| I03 combined | C157 move cleared a footprint courtyard in isolation but shorted six new power-route objects in the merged board. | Rejected; failed report retained. |
| I04 electrical | Compact adequately rated 10 mΩ R153, corrected position, original C157 restored; all lane corrections combined; no GPS status reroute adopted. | Fixed-copper and courtyard screens pass. Final rail model rerun includes new resistance bounds and bypassed via/pad branch. |
| I05 historical handoff | Old active RVB21 candidate instruction had obsolete scope and broken relative paths. | Replaced instruction only; PCB and all common source files unchanged from I04. |
| I06 current handoff | New Molex primary requirements exposed the old two-return schematic note and missing relaxed wire exit. | One guarded schematic-note correction; single-return harness and separate connector-exit CAN model updated. All electrical objects and PCB remain unchanged. |

## Work completed in this pass

- CAD: centered straight U.FL entry and matching correction; explicit oil OUT capacitance/return network; regulator and OV corrections; bulk storage/damping; wider parallel rail; exact compact shunt lands; captured board mounts and thermal wings.
- Firmware: bounded/cooperative GPS writes and diagnostics, protected virtual channels, rate arithmetic, oil acquisition/publication/gap handling, calibration error bound, GPS configuration/readback and consistent source identity.
- Analysis: actual-tap rail model, coupled startup and charging losses, thermal and fault bounds, oil distributed contact model, ground-loss paths, complete CAN branch, RF field/refinement/accessory loading, conducted/radiated/ESD sensitivities and mechanical/retention calculations.
- Documentation: current ASC harness/source-derived maps, one full-current F1 power return and the controlled relaxed connector exit, part/land/handling reconciliation, factory-local fuse process, 13 specifically located filled/capped vias, dashboard exposure/service contract, full criterion/redline traceability and a fail-closed native runner.

The disposition of every recovery action is in `control/DESKTOP_ACTION_DISPOSITIONS_CURRENT.json`. The earlier `REMAINING_DESKTOP_ACTIONS.json` is a historical snapshot. Complete populated geometry and the actual full-load thermal network remain engineering work, alongside the native and external acceptance gates.

## What removes the remaining roadblocks

| Work | What I can finish with the needed inputs/runtime | What you or the supplier supplies |
|---|---|---|
| Native CAD / target compilation | Execute the supplied job, repair real ERC/DRC/build findings, regenerate matched outputs and rerun effectivity. | A working pinned KiCad/pcbnew/Arduino environment, or its runner output. This session's actual preflight found those programs unavailable. |
| Full-load heat safety | Build the actual copper/interlayer/package thermal network; iterate the heat path or operating envelope. | Finished stackup/metal properties and a credible dashboard air/chassis landing bound; subsequent temperature correlation. The dimensioned T01 path alone is insufficient. |
| Regulator / clamp correlation | Compare exact manufacturer models or measured transfer/transient data against the current finite model and revise margins. | Unpublished control-loop/private-RTN information or bounded measurements; resistor pulse/process acceptance where no published guarantee exists. |
| Accessory / vehicle applicability | Bind supplied part identities and traces to the controlled RF/harness/decoder model; check every mismatch. | Actual Adafruit labels/order links, power/return endpoints and representative vehicle/phone profile evidence. |
| Manufacturing / assembly | Reconcile native fabrication data, exact BOM/CPL, limits and incoming records; correct discrepancies. | JLC final construction and 13-via capability, actual compatible profiles, exact handling/lot declarations, crimp/process and first-article evidence. |
| Installed physical behavior | Analyze measurement data and update the source/model evidence; revise if results violate bounds. | Continuity/isolation, thermal/retention, RF/ESD/vehicle non-interference and unit identity data. Source copper cannot reveal an unmade solder or crimp joint. |

Run the native job from the unpacked checkpoint root using an already provisioned environment. It copies inputs, uses a new output directory and does not flash hardware:

```bash
python runtime/run_native_candidate.py \
  --cad-dir iterations/I06_harness_handoff/candidate_kicad \
  --firmware-dir lanes/firmware_oil/candidate \
  --output native_I06_run \
  --kicad-cli kicad-cli \
  --pcbnew-python python3 \
  --arduino-cli arduino-cli \
  --nimble-dir /absolute/path/to/NimBLE-Arduino \
  --component-step
```

Required pinned versions are KiCad 9.0.9, Arduino CLI 1.3.1, ESP32 core 3.3.6 and NimBLE-Arduino 2.3.6. The runner validates actual nonempty outputs; a zero exit code without outputs fails. Resolve its findings as the next recorded redline batch, produce a coordinated revision, then repeat the combined checks. No ordering, manufacturing release, PR merge or vehicle fault testing is authorized by this handoff.
