# I26 pre-hardware closure contract

Date: 2026-09-11

## Objective

Drive Rev B to zero unresolved **pre-hardware actionable engineering work** without converting supplier, as-built, installed-vehicle or environmental observations into fictional desktop evidence. A physical-only acceptance criterion may remain formally unverified, but it is not an open design task once (1) its design antecedents are complete, (2) a quantitative predicted outcome exists where modeling is meaningful, (3) a numerical acceptance/reopen threshold is fixed, and (4) no source change is recommended absent a failed physical gate.

This milestone supersedes the stale I22 thermal/mechanical execution ordering. I25 is the controlling thermal design disposition.

## Disposition classes

- **DESKTOP_CLOSED** — source, CAD, firmware, analytic/simulation or document evidence is sufficient for the design requirement. No further pre-fabrication work is required unless an invalidator changes.
- **BOUNDED_INFERENCE_CLOSED** — inaccessible internal or installed behavior is conservatively bounded well enough that no design change is justified. Hardware correlation remains mandatory but is verification, not engineering debt.
- **QUALIFICATION_ONLY** — the criterion fundamentally concerns a fabricated unit, supplier lot, installed harness/vehicle, phone/OS, RF chamber or environmental observation. Desktop work is exhausted. A predicted outcome and explicit reopen trigger are recorded where useful.
- **CONTINGENCY_ONLY** — analysis belongs to a configuration that is not adopted. It cannot block the release design.

## Current design closure

### CAD and firmware — DESKTOP_CLOSED

The exact candidate has already passed the pinned KiCad 9.0.9 native refill/ERC/DRC/export flow with zero findings and the target Arduino/core/NimBLE build. Any source mutation invalidates this closure and requires the affected native gates again. No additional pre-hardware CAD or firmware correction is indicated by the current evidence.

### Thermal — DESKTOP_CLOSED by I25

The release thermal source is the coupled 2.940859375 W profile, not the historical residual-filled ~4.8 W stress vector. The selected existing-cooling model gives 101.6503 C at 0.50 mm and 107.2667 C at 0.25 mm. Mesh convergence is not claimed; the I25 screening bound is 112.8830 C after carrying the observed 5.6163 C refinement increase once.

Release invariants: ESP32-S3 160 MHz, BLE <=+3 dBm, no Wi-Fi, <=120 notifications/s, sustained main-profile current <=0.45 A, conversion efficiency >=80% or measured losses no worse than the release model, intended bulk dashboard air <=65 C, and immediate air outside U201 <=85 C. Violation of any invariant reopens thermal.

C02 contact/leaf/bridge cooling is **not adopted**. All C110 leaf yield, joint moment, preload and bridge mechanics specific to that cooler are therefore CONTINGENCY_ONLY and are not release blockers. C02 is retained only as a fallback if I25 physical qualification fails.

### Mechanical/package geometry — DESKTOP_CLOSED for the adopted release configuration

The current release design does not include the C02 cooler. Existing declared package/cable geometry has already passed 1,154 populated stack checks, 776 revised Adafruit-851 mated checks and 37 probe approaches. The retained minimum modeled R408/T03 clearance is 0.120 mm. These establish nominal/tolerance-envelope design fit, not as-built workmanship.

Physical checks remain for actual component/cable dimensional conformance, enclosure fit, connector retention and strain relief. A mismatch is a supplier/as-built/install failure unless it exceeds the controlled receiving envelope, in which case mechanical design reopens.

### Adafruit 851/960 accessory path — BOUNDED_INFERENCE_CLOSED electrically; QUALIFICATION_ONLY thermally/mechanically

Exact products are bound: Adafruit 851 U.FL-to-SMA adapter and Adafruit 960 GPS antenna. The routed/mated geometry passes all 776 checks and keeps RF cable/plug metal at least 16.421 mm from the ESP32 antenna region under the model. At the 3.0 V GPS supply boundary, the 2.3 V antenna minimum leaves 0.7 V for the complete internal bias/cable/contact path; at 25 mA this is a 28 ohm total series-resistance acceptance budget.

The 851 published +60 C limit conflicts with the board-level <=65 C bulk design envelope. This cannot be solved by PCB analysis. The installation contract therefore places the 851 cable/adapter in a **<=60 C local harness zone**. If measured local adapter environment exceeds 60 C, relocate/protect the harness or qualify a different accessory; do not respin the PCB unless the replacement changes the board interface.

Receiving/bring-up checks: verify standard-SMA center contact before mating because the linked drawing carries contradictory RP-SMA labeling; verify supplied miniature plug envelope/retention; verify GPS bias remains >=2.3 V at the antenna under representative load and <=28 mA at 3.3 V module limit.

### CAN receive branch — BOUNDED_INFERENCE_CLOSED; vehicle correlation QUALIFICATION_ONLY

The complete branch model executes 14,400 finite cases with separate ASC free exit, twisted AVSS pair, Molex relaxed exit, PCB line, protection load and TCAN receiver. Every case settles to within 1% of the ideal final level by 137.600 ns versus the representative 1.600 us sample; minimum receiver voltage at that sample is 1.495510 V for a 1.5 V source and the worst DC ratio is 0.997006499.

There are 196 repeated 0.9 V crossings, all in the deliberately harsh 2 ns source-edge cases. Their lowest post-crossing valley is 0.736652 V and time below 0.9 V is <=5.2625 ns. There are zero repeated 0.5 V crossings. These are not modeled RXD errors because the receiver nonlinear threshold/hysteresis/propagation path is not represented. The result does not justify a PCB change before hardware evidence.

Predicted physical outcome: no receive errors attributable to this stub under the controlled <=0.30 m harness allocation and normal 500 kbit/s operation. Reopen CAN design only if an actual installation produces repeatable error-count/frame corruption correlated with this branch, violates the controlled exit/length geometry, or a future release enables transmission. R301 remains DNP and firmware transmit-disabled for the present release.

### Power/reset/analog models — BOUNDED_INFERENCE_CLOSED pending correlation

Finite regulator, rail, startup, transient and oil-signal models remain source-bound engineering estimates. Unpublished regulator internals, exact installed source impedance/reference behavior and private die protection are not inferable exactly. Desktop closure therefore rests on retaining conservative modeled envelopes rather than asserting those internals.

Hardware correlation shall measure input/current/rails during startup and representative worst workload, verify no reset or rail collapse outside the accepted crank-gap behavior, and compare oil transfer endpoints against the known 0.5-4.5 V/5 V ratiometric sensor model. A measured result outside the existing model envelope reopens only the affected power/analog design lane.

### RF/EMC/ESD — BOUNDED_INFERENCE_CLOSED for geometry; chamber/install QUALIFICATION_ONLY

Current antenna/launch/return/keepout geometry and bounded coupling analyses are sufficient for pre-hardware release. They do not establish radiated performance, nonlinear ESD clamp behavior or installed coexistence. Predicted outcome is functional GPS/BLE with no material ADC/CAN interference when the controlled cable routes and antenna clearances are honored.

Reopen RF design for repeatable installed C/N0 loss, BLE link failure, ADC/CAN coupling, or ESD upset that exceeds the controlled acceptance plan. Do not create a PCB rework solely because chamber evidence does not yet exist.

### Supplier fabrication/process — QUALIFICATION_ONLY

The controlled JLC construction, filled/capped via set, stackup and local F101 process are source-defined. Supplier acceptance of the exact lot/process cannot be simulated into existence. The fabrication package must preserve the current manufacturing notes and receive supplier confirmation; any proposed fab deviation that changes copper, dielectric, via fill/cap, mask or assembly process reopens the affected native/thermal/SI evidence before approval.

### As-built and installed integration — QUALIFICATION_ONLY

Continuity/isolation, population identity, solder/crimp/contact quality, enclosure fit, actual ASC switched power/return topology, actual vehicle traffic, GPS sky performance, phone/OS behavior, sustained operation and environmental exposure are physical observations. Desktop evidence predicts behavior but cannot replace them.

These checks are acceptance gates, not unresolved product design tasks. A failed gate reopens the narrow affected lane; a pass completes the corresponding original criterion without a respin.

## Pre-hardware stop condition

The board is pre-hardware-closed when all remaining register rows map to one of the four classes above and **no row contains a recommended source/CAD/firmware/mechanical change before fabrication**. The formal 290-row review register may still show OPEN for rows whose wording explicitly requires supplier, as-built, installed, chamber, vehicle or environmental evidence; those must be labeled QUALIFICATION_ONLY rather than misreported as desktop-closed.

The next register reconciliation shall therefore report two counts independently:

1. original criterion status (Closed / Open / N/A), preserving the wording of the original acceptance criterion; and
2. pre-hardware engineering status (Actionable / Qualification-only / Closed / Contingency-only).

The engineering target is **0 actionable pre-hardware rows**, not a dishonest 290/290 desktop pass.

## Global invalidators

Any CAD/firmware/BOM change, release power-profile violation, fabrication stack/process deviation, CAN transmit enablement, different pressure sensor, different GPS board interface/accessory family, enclosure/airflow change, installation ambient beyond controlled bounds, or failed physical gate requires impact review and reopens only the affected evidence lanes.
