# GR86 Rev B — convergence 01

## Product objective and exact state

The objective is to avoid a second PCB order after the eventual fabrication candidate is ordered. Changes to the current draft are permitted and are not themselves a failure. A pre-fabrication revision must be internally consistent across electrical design, firmware, thermal construction, mechanics and assembly. Physical-only acceptance stays outside the active desktop backlog; no such result is invented.

This package starts from PR #46, branch `codex/rvb24-coordinated-closure`, remote commit `5d772f12cb7b7c3fdf40c792726c2e01c8025a73`. It contains **local source/evidence changes only**. The current session exposed GitHub read functions but no write functions; nothing in this package has been pushed, merged, flashed or fabricated.

**Status: useful desktop checks executed; coordinated thermal/mechanical design is not finished.** There is no claim that all 290 criteria have newly passed, that all desktop risks are resolved, or that this candidate is ready to order.

## Actual changes and checks

### Receive-only CAN configuration

The six surviving nearest-In1 reference interruptions are real geometry findings, but occur on `CAN_TX_MCU`, separated from the transceiver by **R301 DNP**. R303 is a fitted 10 kΩ pull-up from +3V3 to transceiver TXD. R306 bus termination remains DNP. The exact firmware uses listen-only mode, disables the transmit queue, has no transmit calls, and retains GPIO5/GPIO4 mapping.

The actual receive path is not the isolated TX stub. Rerouting a functioning bulk rail just to remove these six report flags is not justified for this population. The source-level disposition is conditional on the receive-only configuration; a transmit-enabled variant invalidates it. Power-off faults, internal transceiver failures, a short across the DNP link and actual bus loading are not proved by this source review.

`results/review/RECEIVE_ONLY_DISPOSITION.json` and negative-path tests prevent population/mode/queue/transmit changes from silently retaining this conclusion. GND-02's formal full-scope closure is not claimed: broader neck-down/return applicability remains explicit.

### LED current paths

All six fitted LED/resistor chains, cathode/anode routes, module pads and firmware GPIO/polarity definitions are checked. Each selected 1 kΩ resistor is bounded at 970.2 Ω minimum using 1% initial tolerance and an explicit additional 2% thermal resistance allowance. At 3.6 V and conservatively zero LED forward drop, current is at most 3.711 mA per channel or 22.263 mA for all six; each resistor dissipates at most 13.36 mW in that screen.

No positive low-current optical minimum or loaded GPIO output-high minimum is invented. WCA-07 stays formally open, but the existing standard resistor/LED footprints allow assembly-value or optical-bin tuning without a new layout. This is not a safety-indicator visibility guarantee.

### Current assembly inventory

The current fitted PCB uses **R155 TNPU060311K8HWEA00** and **R156 TNPU06034K99HWEA00**, not the older TNPW entries. The canonical handling JSON and a new CSV now match all **153 fitted references / 74 exact MPNs**. The old table is preserved under `support/HANDLING_I22.json`.

The reviewed Vishay TNPU e3 family sheet permits the listed automated soldering and normal electronics cleaning processes, subject to whole-assembly compatibility. No numeric TNPU moisture sensitivity level or production reflow peak was inferred where the reviewed document did not provide it. This does not qualify the assembler's actual process. Sources are recorded in `SOURCES.json`.

### Executed firmware and native-export verification

The packaged host regression runner was executed successfully on the current candidate. All **nine suites pass**, exercising selected actual firmware functions with explicit driver/time/ADC stubs and ASan/UBSan. The checked firmware binding covers 22 source files. These are not real-time target, phone, radio or vehicle measurements. Nonfatal compiler warnings in the host stubs remain visible in the logs; warning-free compilation is not claimed.

The packaged independent native exporter/checker was also executed. It reconstructs **133 intended connected nets**, checks **141 two-terminal components**, compares schematic pads, all four copper Gerbers, and all **348 drill hits**. The 313 critical outer segments and 12 RF segments retain the reported geometry. No disconnected intended net, same-node component error, netlist mismatch or manufacturing-export mismatch was found.

These checks consume the real verified I24 KiCad artifacts from native run **34499455561**, which passed 14 postconditions. They do not constitute a fresh KiCad invocation in this session. Exact source and filled-board hashes are recorded in the JSON outputs.

All **22 product/configuration negative-path tests pass**. Tests deliberately reject a populated TX isolation link, wrong termination/pull-up/LED wiring, transmit-enabled firmware, unreviewed new MPN, changed PCB evidence and unverified native input. A test count is not a count of closed product criteria.

## Thermal correction and new numerical evidence

The former I25 6.5–7 mm exposed-ground shoe selection was based on a **partial interface resistance**, not the complete board-to-landing path. It omitted the axial bridge, cold joint, spreading allowance and a second interface. In addition, the centered bare-contact geometry intersects nearby component or non-ground routing constraints. Those selection/whole-path acceptance claims have been withdrawn in both the script and current handoff; the previous JSON remains available as explicitly superseded history.

A new standalone native-copper solver removes the previous recovery-directory dependency and makes the full path explicit. It uses the same 4.815 W load, 65 °C bulk air, 70 °C landing boundary, 15 µm plated walls, 5 K/W wing and 15 K/W center path as this review. It parses actual filled zones; it does not silently substitute a guessed fill.

The **unadopted insulated contact trial** has 81.6497 mm² after native bottom courtyards plus 0.5 mm exclusions, including notches for R158, R525 and C531. It retains solder mask and adds an independent insulating TIM. It is not a new bare-copper land.

The trial's full allocated resistance is **8.8975 K/W**: solder mask 1.5309, TIM bulk 1.0206, two interfaces 2.4495, copper thickness 0.0816, 25 × 12 × 3 mm axial copper bridge 2.3148, cold joint 0.5 and spreading allowance 1.0 K/W. These are explicit engineering allocations, not measured or supplier-guaranteed selected-interface performance.

| Quantity | Baseline 0.5 mm | Insulated trial 0.5 mm | Insulated trial 0.25 mm |
|---|---:|---:|---:|
| Maximum board region, °C | 126.227 | 105.258 | 111.533 |
| U201 source-region mean, °C | 124.034 | 102.585 | 105.666 |
| U121 source-region mean, °C | 107.850 | 102.901 | 108.497 |
| C206 board-region maximum, °C | 100.224 | 85.162 | 85.679 |

At the same coarse mesh, the proposed local heat path reduces the board maximum by 20.969 K. Refinement raises the trial maximum by **6.275 K**, with U121 now the hotter source region. **The 105.3 °C coarse result is not accepted as a converged thermal pass.** The refined case has 302,864 nodes, all 348 real barrels and a 5.84e-11 W absolute energy-balance residual. A small algebraic residual proves the discrete energy solve, not mesh/topology accuracy or physical correctness.

The model still homogenizes separated copper pieces within some mesh cells and does not contain a manufacturer-qualified module-internal junction model. The package and local-air margins, finite-mesh sensitivity and actual contact assembly remain unresolved. Neither 105.3 °C nor 111.5 °C is a measured temperature or a semiconductor junction prediction.

### Coupled design constraints that remain

The new contact must be designed together with a real carrier window, support/preload path, insulating-interface compression range, bridge and cold landing. The BLE antenna housing clearance cannot be ignored: Espressif recommends at least 15 mm in all directions in the end product. The local contact approaches the antenna region and therefore has **no antenna-clearance acceptance**. Do not order or install this contact based on the thermal screen alone.

The next corrective revision should address U201 and U121 together, preserve signal isolation and antenna/service space, and then run the complete source-bound regression. Free pre-fabrication relocation/routing/material changes are allowed when justified. This is not a directive to preserve an ineffective prior carrier or minimize a change count.

## Whole-register reconciliation

The full 290-row register is mapped in `results/review/ALL_CRITERIA_COVERAGE.json`. The original 140 closed / 146 open / 4 not-applicable accounting and all 357 historical redline identities are preserved. Only four original rows carried the `DESKTOP_WORK_REMAINING` label: WCA-07, REG-02, GND-02 and THERM-02. The classifier also retains coupled dependencies; the four-row label is not an assertion that only four engineering risks exist.

The review recovered and checked all **106 unique linked historical evidence files**; their recorded digests match. That establishes file identity, not fresh re-execution or an automatic new pass. Twenty-six rows retain explicitly bounded analyses and 109 are routed to the external-acceptance workstream. Classification is a work map, not deletion or waiver of a requirement.

After the coordinated redesign, re-evaluate **all applicable criteria**, including prior closures. Source changes must invalidate affected CAD, export, firmware, power, return, thermal, material, mechanical and antenna evidence. Reuse an unchanged result only when its inputs and applicability still match.

## Reproduction and recovery

See the root package README for restoring the exact baseline plus patch. With the repository working directory as the current directory, set `D=docs/engineering/rvb22/analyses/convergence_01`. Install the pinned dependencies in a venv and use a C++17 compiler for host tests. The code was executed with Python 3.13, NumPy 2.3.5, SciPy 1.17.0, Shapely 2.1.2 and Gerbonara 1.6.3.

```sh
python "$D/firmware_regression.py" --firmware-dir docs/engineering/rvb22/candidate/firmware --out /tmp/gr86-firmware-check
python -m unittest discover -s "$D/tests" -v
python "$D/run_review.py" --out /tmp/gr86-review --evidence-root /path/to/package/historical_evidence
python "$D/thermal_native.py" --native-dir /path/to/package/native_I24 --mesh 0.5 --case both --out /tmp/gr86-thermal-05
python "$D/thermal_native.py" --native-dir /path/to/package/native_I24 --mesh 0.25 --case trial --out /tmp/gr86-thermal-025
```

The workflow patches now install isolated analysis dependencies and call real entry points; a missing review runner is no longer treated as an archival-only successful review. The old SciPy-missing thermal step is replaced with the native solver. **Only local parser/entry-point execution has been checked; these changed workflows have not run remotely.**

The full-path model, contact WKT, raw numerical outputs, maps, original failed-route trial, host regression logs and negative tests are included. No new PCB or mechanical geometry is adopted by this checkpoint. The numerical work must not be represented as completed product convergence.
