# I23 active engineering wave

Parent PR45 head: c7c93fe48d1106bededf4c8d7698fe234fdd07e4. The complete I22 state is frozen in the recovery history. There were 140 closed, 146 open and four N/A original criteria; 357 prior redlines are retained. Four I23 findings cover returns, thermal applicability, mechanical margin/manufacturability and complete reconciliation.

Candidate PCB SHA256: 1c3c81e35b4a3bc418e55a8cad55f98c0407dc957a356329c246b53349351e93. Only three LED nets change: 205 segments removed, 294 added. BLE and PWR stay on In1; CAN moves one complete chain to In2 between existing through vias. All nonsegment CAD objects and all unrelated segment objects are exact. Independent copper clearance and explicit connectivity have no regressions. Actual filled-zone/native results are required before promotion.

The rejected oil-signal route would add over 60mm of analog path. It is preserved as a failed trial and not adopted. Fixed-anchor oil excitation and remaining bulk-branch return intrusions remain for diagnosis after native refill. This iteration does not lower full 4.815W, 65C air or 70C landings. Adafruit851/960 and the original TW logo remain selected. Current mechanics remain C05/W02/T03 with the complete1.71mm height stack; thermal/material/physical qualification is still open.

Next: retrieve native artifacts, compare every actual reference gap, rerun affected ground/power/thermal and mechanical geometry checks, then update the full criterion register, workbook, controlled instructions and recovery archive. PR45 is a draft engineering handoff, without merge or fabrication authorization.
