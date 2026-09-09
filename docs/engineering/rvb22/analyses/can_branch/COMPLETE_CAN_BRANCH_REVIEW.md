# Complete ASC CAN branch loading — separate connector exits

CAN22-02 corrects the previous uniform-cable approximation. The model now includes a relaxed untwisted section at each connector, the remaining twisted AVSS pair, the PCB line, protection/connector allowance and receiver. **14,400 finite cases** have been executed. The earlier 180-case model is retained in `history/CAN22-01_180cases` and is superseded for current harness effectivity.

Molex requires a relaxed free exit of at least 25.4 mm, so a 25.4–30 mm F1 exit is represented separately. The ASC exit uses a 0–15 mm sensitivity; zero is a mathematical limiting endpoint, not a manufacturing instruction. Both exits have independent 3–4 mm centre spacing and effective-permittivity endpoints 1 and 8. The 3 mm starting spacing follows the connector geometry; 4 mm and effective permittivity are installation/model allocations. The actual AVSS conductor-envelope endpoints are 0.78 and 0.80 mm. The resulting maximum local impedance is **277.999 Ω**, exceeding the old uniform 200 Ω endpoint.

| Model section/input | Current value and scope |
|---|---|
| Source-side ASC free section | 0 or 15 mm, separate geometry-derived differential RLGC |
| Remaining twisted AVSS | Total 0.30 m conductor path minus both exits; retained independent 25/55/100/160/200 Ω and full-length 1/3.1 ns sensitivities scaled to the remaining path |
| Board-side Molex exit | 25.4 or 30 mm; no twist, tie or forced bend in the required relaxed exit |
| Entire wire resistance | **0.03723 Ω for the complete two-conductor loop** at 80 °C, partitioned by section length; exact source estimate 0.037222296 Ω |
| PCB/contact resistance | Separate 0.025 Ω allocation, unchanged |
| PCB line | 80 Ω, 0.1 ns, with distinct cells |
| TCAN3403 receiver | 20 pF complete differential capacitance maximum and 25 kΩ minimum differential resistance under supplied normal operation |
| D301 and additional load | 2.5 pF differential from 5 pF/channel ESD capacitance at 25 °C; total board-entry extra 7.5 or 32.5 pF includes connector/temperature/PCB allowance |
| Main-bus source | 45/60/75 Ω Thevenin source, 1.5 V final dominant level, 2/20/100 ns edge ramps |
| Timing example | 500 kbit/s, 2 µs bit period, 1.6 µs sampling instant |

The maximum conductor cut path remains **0.30 m**. No additional helix length is added on top of that wire limit. The free-exit calculations use L′ = μ₀ acosh(D/d)/π and C′ = π ε₀ ε_eff/acosh(D/d), with Z = √(L′/C′) and delay = length × √(L′C′). The 80 independent exit geometries combine with the retained twisted/source/load/edge families. Each section has its own R, L and distributed capacitance; the extra board-entry load is placed after the Molex exit, and the receiver capacitance/resistance remains at the far end of the PCB section.

All finite cases reach within 1% of the ideal final source level by **137.600 ns**, compared with the example 1,600 ns sample. The minimum receiver voltage at that sample is **1.495510 V** for a 1.5 V source. The smallest DC ratio is 0.997006499. The rising-transition sweep has **196 cases with multiple/non-single 0.9 V crossings** and **0 with multiple/non-single 0.5 V crossings**.

All 196 recrossing candidates remain after doubled spatial resolution and finer time sampling, and all use the 2 ns source edge. Their lowest post-crossing valley is **0.736652 V**, with at most **5.2625 ns** below 0.9 V. These are repeated crossings of a fixed voltage boundary, not simulated RXD logic errors: receiver trip points, hysteresis and propagation were not modeled. The data sheet's 50 mV hysteresis value is typical, so it cannot be used as a guaranteed filter to erase this finding. There are no repeated 0.5 V crossings in the main grid or these targeted refinements.

The early waveform remains material: the largest temporary tap error relative to the ideal source is **80.145%**, and the largest receiver crest is **2.317994 V**. These published extrema enclose the main grid and the executed selected refinements. The main-grid crest was 2.315975 V and its tap error 79.971%; refinement increases both as shown. They are finite estimated transient excursions, not observed bus voltages or bounds over every intermediate parameter value. A foreign ECU sampling during such a disturbance is outside the favourable late-sample comparison. This model does not establish full-vehicle non-interference.

## Numerical checks

The worst settling, tap-disturbance and receiver-crest cases were each rerun with doubled twisted-line resolution and finer exit/board discretization. The largest waveform difference at a 1.5 V drive is **12.735 mV**. Further time-grid refinement changes the compared peak metrics by at most **0.005 mV**. Refined settling times and exact per-section cell counts are recorded in the results. These differences indicate the precision warranted for the early peak; they are neither a formal numerical error bound nor hardware measurement errors.

The largest DC nodal error against the independent 25 kΩ divider is 1.78e-15. Total series resistance partition error is 6.94e-18 Ω. Setting both new exit lengths to zero reproduces the previous model waveform exactly in two independent limiting cases. A separately rebuilt current-state matrix, using unscaled amperes and direct matrix exponentials, agrees with the modal ramp response within 1.8 × 10⁻¹⁴. All 8 saved independent numerical checks pass. An independent reviewer also checked section order, RLGC equations, capacitor placement and resistance partition.

![CAN branch waveforms](CAN22-02_WAVEFORMS.png)

## Source effectivity and remaining conditions

The final board is `c2297b8c857d54f525667e9b2dace5f5f538b7c8d651370cc1b40fa2bdfb39c1`. Its **349 CAN source objects remain exactly equal** to the previously verified source. This correction changes the wire model, not PCB copper. `FINAL_CAN_SOURCE_BINDING.json` binds those preserved objects to the new result hash; it does not claim native fill, fabrication or a newly measured vehicle interface.

Primary assembly/geometry references are [Molex 43045 application specification](https://www.molex.com/content/dam/molex/molex-dot-com/products/automated/en-us/applicationspecificationspdf/430/43045/430450001-AS-000.pdf) and [TE 1376106-1 connector dimensions](https://www.te.com/en/product-1376106-1.html). Their archived observations and AVSS source data are controlled in `analyses/harness_wire`. Receiver/protector inputs use the archived exact [TCAN340x-Q1](https://www.ti.com/lit/ds/symlink/tcan3404-q1.pdf) and [ESD2CAN24-Q1](https://www.ti.com/lit/ds/symlink/esd2can24-q1.pdf) data sheets.

The result is a **conditional normal differential estimate**. Actual exit spacing, effective permittivity, wire length and temperature must match the installation allocation. Exact vehicle termination, main-bus topology and sampling phase remain external inputs. Common-mode conversion, powered-off behaviour, nonlinear die protection, arbitrary shorter harnesses, multiple traffic edges and actual CAN error counts are outside this finite linear family. Zero dielectric conductance and excluded skin/proximity loss make this an explicitly simplified line model. No 120 Ω supplier guarantee, IBIS result, as-built continuity or vehicle qualification is claimed.
