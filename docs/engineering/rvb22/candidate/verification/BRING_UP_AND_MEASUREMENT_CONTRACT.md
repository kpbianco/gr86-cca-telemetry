# RVB22 bring-up, current limits and measurement contract

Status: controlled engineering procedure for the revised source. This document specifies checks; it does not record hardware results. Native CAD/export and target-build gates remain separate. Source-derived pad coordinates and hashes are in BRINGUP_ACTUAL_NODE_INDEX.json. The annotated schematic text delta must be applied after the power source delta.

| Controlled condition | Bound and measurement location |
|---|---|
| Normal supply | 9.5–16V at board input; 24V is a disconnect/survival condition, not normal regulation |
| Input UV falling | 7.369491–7.709308V at REV_BLOCKED_12V, R102.1 after input diode |
| Input UV rising | 7.669066–8.194934V at the same sense node |
| Input OV trip / recovery | 19.554833–20.450296V / 18.283539–19.644817V at the same sense node;32ms recovery is typical, not a guaranteed maximum |
| Main source rail | Adjustable3.37V nominal; calculated source interval3.297890–3.442858V before distribution drop. R15523.7k/R15610k0.1%,25ppm/K,125K allocation and100nA FB; R15710k selects adjustable operation |
| Main load / distribution | ≤0.75A main allocation. Evaluate actual branched source geometry;0.05Ω distribution is the current model target, not a measured board value. The target implies local DC minimum3.260390V at0.75A |
| Normal main transient | Local rail above3.116050V and below3.6V, without unintended supervisor reset. A3.0V module limit alone is insufficient to establish reset margin |
| Supervisor | Falling threshold3.023950–3.116050V; conservative rising maximum3.193951V; CT-open delay12–28ms. External EN reset does not uniquely identify the cause |
| New local storage | C206 KEMET T598X477M006ATE025,470µF6.3V X-case; minimum210.56µF/maximum806.52µF stacked screen; full main bank maximum945.67µF. Case≤125°C and rail≤3.6V. Rated4.2V category/recommended3.78V at125°C does not relax the IC rail limit |
| Damping branch | R158 Vishay WSLP0603R0820FEA82mΩ1%,75ppm/K;80.57115–83.44115mΩ at125°C screen,0.4W rating at70°C. The capacitor25mΩ ESR is a maximum; do not invent a nonzero ESR minimum. Actual tapped-loop model remains the validation authority |
| Main startup | Final explicit R15882mΩ/L15115µH equivalent model:4,096 finite cases,0 recovery failures/UVLO restarts; maximum EN34.8895ms. Startup5V maximum5.211487V; post-EN5V4.922314–5.139656V. Selected50ns refinement gives post-EN main≥3.222890V. These are controller/source hypotheses, not guaranteed silicon behavior; GPS/oil data remain invalid until device health and settling are established |
| GPS rail | Source tolerance3.201–3.399V; module operating3.0–4.3V and ripple≤50mVpp. Probe at module VCC/GND with a short local loop |
| Sensor excitation | Remote Honeywell B-to-A4.75–5.25V, not merely board OIL_5V-to-GND. Normal6.5±1mA is specified at25°C;15mA hot current is an allocated sensitivity |
| Oil turn-off | U501 OUT-to-RTN≥−0.3V. Conditional source model requires OUT-to-GND≥−0.2V with privateRTN motion≤+0.1V; C532 actual network includes its C504 terminal and In1 shared-copper stitches. Use the final nodal model, not the superseded isolated-branch assumption |

## Staged energization

1. Record PCB/assembly/firmware/harness identities and native/export status. Check fitted parts and polarity, empty reserved F1 cavities, correct fixture face and absence of alternate chassis bonds. Disconnect vehicle and sensor. Open R109 to isolate converters; begin protection-only at a20mA current limit. Sweep normal9.5–16V before limited-energy UV/OV characterization. The calculated protection quiescent budget is below0.491mA at16V;0.6mA is an investigation threshold, not a tested unit acceptance result.
2. Restore R109 and open R128 to isolate the downstream5V load. Begin U121 source-only at12V/100mA. Investigate steady raw current above10mA with the downstream branch disconnected. This allowance includes switching/core/ripple-network losses and is not the IC quiescent-current specification.
3. With U121 isolated by R128, inject5.00V into the downstream5V side, initially0.10A, ceiling0.90A. Never parallel an active external source with an enabled converter. Restore R151 as required; open R153 for U151 source-only operation. Investigate source-only injected5V current above25mA. This is an FPWM diagnostic allocation, not the nonswitching40µA IQ value.
4. Restore the intended main bank. Capture source rail, local rail,5V and EN during startup. Allocate pre-reset main load≤0.20A. Record current-limit entry, recovery and any UVLO/EN restarts. Stop and investigate sustained current limiting, wrong rail, repeated hiccup or no recovery after50ms. The50ms stop is a troubleshooting rule, not a guaranteed startup time.
5. Apply0.05/0.15/0.20A initial loads, then the allocated0.5A step with rise≥1µs while total main load stays≤0.75A. Probe U151 source bank, C206/local ESP rail and EN simultaneously. Compare the recorded waveform with the final finite loop model and its assumed controller response. Preserve adverse longer-response cases; do not turn a short-response simulation into a die guarantee.
6. Add GPS and oil branches separately. R401 is the GPS isolation point. Verify remote sensor excitation, transfer and invalid-data behavior before controlled fault injection. Use only the reviewed limited-energy fixture, never arbitrary battery-to-signal shorts. Capture U501 OUT/GND and OUT/RTN differentially; an oscilloscope ground clip must not bond privateRTN to board GND.
7. Integrate on a12V current-limited bench supply, ordinarily≤0.8A input within the declared thermal envelope. Test CAN on a representative three-node terminated bench with the board in passive receive operation. Confirm GNSS supported operating mode/readback and phone profile. A bench pass does not establish the installed vehicle takeoff or traffic semantics.

## Discharge, partial power and consumption

R1541k bleeds the local main rail. After switch-off observe initially10s, then independently verify TP151, TP203 and C206.1 below0.2V before changing connections. Open R153 creates separate source/local islands; one dead test point does not prove the other island is discharged. GPS-only fixtures need their own1k bleed and voltage check. Before oil calibration, verify TP505 and TP510 below0.2V; use a floating SHDN-to-RTN dry contact and isolated4.75–5.25V stimulus limited to2mA. Do not parallel stimulus and live excitation.

Protection-only current is calculated from U10190µA VIN+110µA VOUT maxima, R111 and threshold-divider low resistance, plus100µA explicit leakage/gate allowance. At16V the result is0.490985mA. Source-only U12110mA raw and U15125mA injected5V are investigation allowances reviewed against switching losses, not guaranteed worst-case silicon numbers. At5V and an80% conversion allocation, main input current is0.129107A at0.15A load,0.172143A at0.20A and0.645536A at0.75A using the maximum main source voltage. Add actual GPS/oil/logic currents separately. Do not compare those loaded values with source-only limits.

## Pin-local loops and instrument loading

| Loop or victim | Pin-local access and purpose |
|---|---|
| U121 input commutation | C123 VIN/GND and U121 VIN/PGND; short loop, identify source impedance and switching current |
| U121 output/COT feedback | L121, output bank and actual ripple-injection R/C network; check feedback ripple and5V recovery together |
| U151 input / internal supply | C152 VIN-to-PGND within manufacturer placement constraint; C153 VCC-localGND; differential C154 BST-to-SW. Do not ground the SW node |
| U151 source and feedback | C155/C156/C157 source-to-localGND, R155/R156 quiet feedback return; compare source and local rail to expose distribution drop |
| ESP local storage | C201/C204, C206 and R158 series branch; local Kelvin voltage and case temperature. C2064.3mm body clearance is mechanical, not a thermal measurement |
| Reset | U202 sense, SUP_RESET_RAW and ESP_EN; record actual threshold/delay and any probe loading. Open-drain service reset only |
| Oil output/return | C532 OUT/GND, U501 OUT/RTN, shared star and remote B/A. Actual same-net overlaps are part of the model topology |
| Analog settling | Op-amp output,220Ω/22nF ADC input and100pF feedback; probe input capacitance/resistance must be included in settling interpretation |
| GNSS | Module VCC/GND and RF supply-noise correlation; local ripple≤50mVpp target. RF connector probes require their calibrated loading model |

Use differential probes where grounds differ; state bandwidth, probe capacitance, offset, DMM uncertainty and current measurement burden. Include uncertainty when comparing to a threshold: no numerical result at the nominal limit is an automatic pass. Schematic/source annotations define current loops and access; measured waveforms, thermal acceptance and supplier assembly records remain separate evidence.

Final startup authority: lanes/power/COUPLED_STARTUP_ENVELOPE.json and C206_CURRENT_THERMAL_ENVELOPE.json. The capacitor post-EN RMS in that startup model is≤0.0194354A; separate actual-tap edge/switching sensitivity reaches0.479947A and17.276mW at75mΩ during its modeled50µs response interval. Startup pulse energy is approximately0.396214mJ in C206 and0.440808mJ in R158; inverse transient thermal conditions and supplier pulse capability remain explicit. Do not equate an interval-average loss with a manufacturer pulse rating.
