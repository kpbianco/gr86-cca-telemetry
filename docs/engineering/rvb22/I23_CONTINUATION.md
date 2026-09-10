# I23 active engineering wave

Parent PR45 head: c7c93fe48d1106bededf4c8d7698fe234fdd07e4. The complete I22 state is frozen in the recovery history. There were 140 closed, 146 open and four N/A original criteria; 357 prior redlines are retained. Four I23 findings cover returns, thermal applicability, mechanical margin/manufacturability and complete reconciliation.

Candidate PCB SHA256: 44121c54ce49ace9c82404726706ef8a8f582ad63a9dad0a29bdd99aa0cad5db. Only three LED nets change: 205 segments removed, 46 added. PWR stays on In1; BLE and CAN move complete chains to In2, joining existing through-via lands. All nonsegment CAD objects and all unrelated segment objects are exact. Independent copper clearance and explicit connectivity have no regressions. Actual filled-zone/native results are required before promotion.

The rejected oil-signal route would add over 60mm of analog path. It is preserved as a failed trial and not adopted. Fixed-anchor oil excitation and remaining bulk-branch return intrusions remain for diagnosis after native refill. This iteration does not lower full 4.815W, 65C air or 70C landings. Adafruit851/960 and the original TW logo remain selected. Current mechanics remain C05/W02/T03 with the complete1.71mm height stack; thermal/material/physical qualification is still open.

Next: retrieve native artifacts, compare every actual reference gap, rerun affected ground/power/thermal and mechanical geometry checks, then update the full criterion register, workbook, controlled instructions and recovery archive. PR45 is a draft engineering handoff, without merge or fabrication authorization.

First native run34444157108 reported three DRC errors: a GPS track keepout crossing and two0.25mm protected-net clearance violations. These are retained in run25 artifacts. The corrected preflight includes all seven track keepouts and raw/protected net rules. Current minimum protected clearance is0.300mm; raw clearance is0.661mm. Exact continuous-line checks simplify the candidate from294 to46 new segments. Re-run native checks before promotion.
