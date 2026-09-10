# GR86 CCA RVB22 — I21 review checkpoint

The controlling hardware source is **I20** at commit `e36523bb1bd374a58b0071e761b7a46c8cc4385e`. I21 reconciles its evidence, completes the interrupted thermal refinement and corrects the stale I06/I11 status. CAD and firmware are unchanged in this checkpoint.

| Result | Current evidence |
|---|---|
| Native CAD | Zero ERC, DRC, warnings, unconnected and parity findings. |
| Target firmware | Pinned Arduino CLI 1.3.1 / ESP32 3.3.6 / NimBLE 2.3.6 build passes; ELF/BIN hashes verified. |
| Recovery integrity | 1,042 fresh digest checks pass across source and 774 archived native outputs. |
| Manufacturing | Separate copper/Gerber/drill/netlist reconstruction passes; 153 fitted BOM references, 151 automatic placements and two manual exceptions agree. |
| Models and branding | All 153 fitted references resolve. The original 8 mm Compact TW logo is present on F.SilkS and matches its vector source. |
| Power and RF | Finite source-bound sweeps pass their declared component/controller/return and RF allocations. |
| Mechanics | C05/W02/T03; 49 filled/capped vias; 776 declared mated checks and 37 probe approaches pass. |
| Review register | 134 closed, 152 open, four not applicable. All 290 original criteria and 351 recorded redlines are retained. User baseline remains 73/213/4. |
| Thermal | Still open. The completed 0.125 mm model reaches 143.800°C maximum board region and 105.171°C near C206 at 4.815 W, 65°C bulk air and 70°C landings. Mesh and package/local-air closure are unresolved. |

Start with [the current execution plan](current/EXECUTION_PLAN.md), [thermal review](current/THERMAL_REVIEW.md), and [manufacturing/assembly instructions](current/MANUFACTURING_AND_ASSEMBLY.md). The updated review workbook and complete recovery archive carry the full evidence history. JSON evidence paths are relative to the recovery archive root; selected current results are mirrored in this repository. Historical summaries and candidate-directory notes are superseded by this current handoff.

The source candidate remains `candidate/cad` and `candidate/firmware`. The pinned [native workflow](../../../.github/workflows/rvb22-native.yml) has already produced [I20 run 34420381540](https://github.com/tranquilWorks/gr86-cca-telemetry/actions/runs/34420381540). There is no missing native-runtime gate. A future source change must rerun the workflow and regenerate matched outputs.

Confirmed inputs are the Honeywell MIPAN2XX150PSAAX sensor, GR86 ASC harness per Timurrr, Adafruit adapter/GPS puck by product type, JLCPCB and dashboard use. Exact accessory PIDs/temperature/polarity, supplier construction/material acceptance and actual-unit/installation qualification remain explicit conditions. The nearly ideal contact sensitivity is an unadopted analysis, not a physical cooling correction.

The full thermal and supplier/physical evidence required for overall zero is not yet present. PR #45 remains a draft engineering candidate.
