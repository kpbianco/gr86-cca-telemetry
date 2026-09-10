# I21 closure plan and current effectivity

Continue from I20 commit `e36523bb1bd374a58b0071e761b7a46c8cc4385e` on PR #45. The current PCB and firmware are ahead of the quoted I11 status. I21 reconciles all original review criteria, reruns independent source/export checks and completes the interrupted thermal refinement. It preserves the original 73 closed / 213 open / 4 not-applicable user baseline.

Confirmed inputs are Honeywell MIPAN2XX150PSAAX, GR86 ASC harness per Timurrr, Adafruit U.FL-to-SMA adapter and GPS puck by product type, JLCPCB fabrication and dashboard use. Exact Adafruit item numbers remain unconfirmed. The user authorizes circuit, firmware, RF, mechanical and assembly corrections and equivalent numerical simulations. The operating model remains 4.815 W, 65°C bulk air, 70°C cold landings and 70 kPa. Do not replace dashboard bounds with an engine-bay profile.

| Workstream | Current result | Next closure action |
|---|---|---|
| Native CAD and target build | Complete for I20. Zero findings; all 153 fitted models resolve. | Rerun the pinned workflow when source changes. |
| Export/source identity | Fresh 1,042 digest checks pass. Independent four-layer copper, Gerber, drill and netlist reconstruction passes. | Bind any next revision to its own native outputs. |
| Power | 3,456 finite rail cases and 331,488 eligible controller cases pass their defined boxes. A separate numerical method confirms limiting voltages within 4.5 µV. | Retain controller, return, pulse and component bounds. Do not describe a controller-family sweep as a private silicon model. |
| RF | Current GPS reference planes and finite network checks pass their stated bounds; straight centered U.FL entry remains. | Settle exact Adafruit polarity/temperature/dimensions; retain installed RF/EMC correlation conditions. |
| Mechanics and assembly | C05/W02/T03, C206 tab, 49 via-in-pad locations, 776 mated checks and 37 probe approaches are documented. | Confirm exact materials and construction conditions; recheck any cooling change against the same populated/mated solids. |
| Thermal | Completed 0.125 mm solve. Maximum board region 143.800°C, C206 region 105.171°C. The hotspot changes 8.376°C from the 0.25 mm mesh. | Resolve numerical mesh dependence, package paths and local module ambient. Test how much existing contacts can help before selecting the cooling revision. |
| Review register | Preserve all 290 original questions and required-evidence cells. Replace stale source/build/model dispositions with current evidence. | Keep original physical/supplier requirements visible; do not equate a model or native pass with an unexecuted measurement. |

For each revision: freeze every observed redline; record the intended correction and affected domains; make one coordinated candidate addressing those findings; regenerate native CAD/manufacturing and target outputs; evaluate copper continuity, RF reference, power, thermal, populated/mechanical and assembly effects; reconcile every criterion against the exact candidate. Preserve failed trials and superseded identities. Repeat for newly exposed findings. A finding is closed only within the scope its evidence actually establishes.

The next thermal candidate must retain the full electrical load and environment unless an explicit enforceable operating change is designed and verified. Improving interface resistance alone cannot be assumed to cure the board-spreading bottleneck. A new contact, spreader, module or carrier must also satisfy antenna clearance, insulation, clamp force, foil flexibility, solder service and the declared hot material bounds. No speculative cooling geometry is promoted merely to reduce a reported temperature.

Actual serial/flash records, purchased accessory identity, supplier process acceptance, installed harness/ground behavior, phone/vehicle observations, and lifetime/environmental qualification require evidence beyond the CAD package. These are named remaining conditions, not missing native tools. Current source, evidence and the review register are forwarded in the existing draft PR. No fabrication order, firmware flash, merge or vehicle action is part of this checkpoint.
