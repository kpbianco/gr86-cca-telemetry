# I26 current execution plan — pre-hardware closure

Date: 2026-09-11

This plan supersedes the stale I22 current plan. The controlling objective is **zero unresolved actionable engineering work before real hardware/supplier/installed validation**, while retaining explicit physical-only acceptance gates instead of fabricating evidence.

## Controlling state

- Branch: `codex/rvb24-coordinated-closure`.
- I25 thermal milestone is controlling for release thermal design.
- I26 closure contract: `I26_PREHARDWARE_CLOSURE.md` and `.json`.
- Exact native CAD/firmware evidence remains source-bound; any source mutation reruns affected gates.
- C02 cooling hardware is not adopted. C02-specific leaf/joint/preload engineering is contingency-only and cannot block release.

## Closure order

1. **Reconcile the 290 original rows into a parallel pre-hardware status.** Preserve original Closed/Open/N/A semantics. Add/derive `Closed`, `Bounded inference`, `Qualification-only`, `Contingency-only`, or `Actionable` for engineering disposition. Target: `Actionable = 0`.
2. **Return path / SI.** Use existing native copper plus the completed CAN branch family and alternate-reference evidence. Do not reroute solely to remove abstract flags when both ground planes provide continuous reference and the release TX stub is isolated by R301 DNP. A real source change is justified only by a demonstrated unresolved fitted-path violation.
3. **Release mechanics.** Evaluate only adopted hardware. Retire C02 cooler structural questions to contingency status. Preserve enclosure, connector, cable, probe and receiving envelopes as qualification gates.
4. **Accessory path.** Exact Adafruit 851/960 identity is bound. Keep the electrical/RF route desktop-closed. Treat the 851 +60 C rating as a local harness-zone acceptance requirement (`<=60 C`); it does not justify a PCB respin.
5. **Power/reset/analog.** Exhaust source-bound interval/ODE/MNA evidence. Where private regulator/die or vehicle-reference behavior is unknowable, freeze a conservative model envelope and a measured reopen threshold rather than leaving an unbounded TODO.
6. **RF/EMC/ESD.** Close geometry/coupling/keepout work on desktop. Chamber, radiated, nonlinear clamp and installed coexistence become qualification-only with explicit failure/reopen criteria.
7. **Supplier/as-built/integration.** Supplier process confirmation, physical continuity/population/crimp, installed ASC topology/traffic, GPS sky performance, phone/OS behavior and environment are qualification-only. No desktop simulation is represented as physical evidence.
8. **Final artifact reconciliation.** Ensure register, gate summary, PR text, CAD, firmware, manufacturing package and acceptance plan all refer to the same release configuration and invalidate together on source changes.

## Current bounded predictions

### Thermal
Selected existing-cooling release profile: 2.940859375 W. Board-region maximum is 101.6503 C at 0.50 mm and 107.2667 C at 0.25 mm; I25 screening bound 112.8830 C. Release invariants are 160 MHz, BLE <=+3 dBm, no Wi-Fi, <=120 notifications/s, sustained main-profile <=0.45 A, converter efficiency >=80%, bulk dashboard air <=65 C and immediate U201 air <=85 C.

### CAN receive branch
14,400 cases. All reach within 1% by 137.600 ns versus representative 1.600 us sampling. Minimum receiver voltage at that sample is 1.495510 V for a 1.5 V source; DC ratio >=0.997006499. The 196 0.9 V recrossings all occur in 2 ns edge cases, with <=5.2625 ns below 0.9 V; zero repeated 0.5 V crossings. Predicted release outcome is no branch-attributable receive error under controlled harness geometry. Vehicle correlation remains mandatory.

### GPS accessory path
Adafruit 851/960 pairing and route are controlled. Mated route passes 776 modeled checks and retains >=16.421 mm metal-to-ESP32 antenna-region spacing. GPS bias has a 28 ohm complete-path series-resistance budget at 25 mA from the 3.0 V / 2.3 V endpoints. The 851 local environment must remain <=60 C.

## Stop condition

Stop pre-hardware engineering only when every original criterion maps to a non-actionable I26 class and **no remaining row recommends a CAD, schematic, PCB, firmware, mechanical or release-profile change before fabrication**. Physical/supplier/installed rows may remain formally Open if their wording requires observation, but they must have a fixed acceptance test, predicted outcome where meaningful, and a narrow reopen rule.

A future hardware failure reopens the affected lane; it does not retroactively turn every qualification-only row into unfinished design work.
