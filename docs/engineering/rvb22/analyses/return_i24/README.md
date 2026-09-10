# I24 return-routing evidence

The adopted final candidate is `iterations/I24_return_clearance_final/candidate_kicad`.
Its PCB SHA256 is `5b373f6033fdbd18f126f8ca054b619abada2568778c5a8fc303c4e756fb06c8`.
The earlier partial candidate remains preserved under `iterations/I24_return_clearance`.

Final native run28 binds that source to filled PCB SHA256
`11533ea91c3bc4c61dd7066e0001914a72bc90b27afb38d321c43b8feb90e3b1`.
All 14 native postconditions pass with zero ERC/DRC/parity findings. The peer
manufacturing reconstruction verifies all 133 intended nets and matching
exports. The separate per-UUID comparison covers all 313 critical outer
segments, including subthreshold gaps: 15 interruptions / 5.912173 mm become
six / 1.341746 mm, with no reference regressions. All nine CAN_RX gaps and
the 0.057977 mm simultaneous inner-plane gap are removed. All 12 RF segments
retain nearest-plane ground. See `NATIVE_REFERENCE_REVIEW.json` and
`REDLINE_DISPOSITIONS.json`; the six CAN_TX nearest-In1 gaps remain open.

## Actual problem and scope

I23 native filled zones show 15 adjacent-plane interruptions totaling
5.9121730345 mm: nine CAN_RX_MCU segments / 4.5704273981 mm and six CAN_TX_MCU
segments / 1.3417456364 mm. The CAN_RX causes are OIL_5V and OIL_SIG on In1.
The CAN_TX cause is 3V3_BULK_DAMPED beneath the MCU pad escape. One CAN_TX
interval also lacks In2 for 0.0579770992 mm because an I23 LED_CAN route passes
beneath it. Both original and supplemental findings were frozen before edits.

The final candidate removes the two oil intrusions from In1 by moving complete
chains between unchanged through-via anchors to In2. A local LED_CAN detour
avoids placing its In2 clearance beneath any of the six remaining CAN_TX
adjacent-In1 gaps. It does not establish or claim a local return-current
transfer or close the continuous-nearest-reference criterion.

| Changed chain | Previous length (mm) | Final length (mm) | Change |
|---|---:|---:|---|
| OIL_5V | 12.0538000000 | 5.3487978311 | In1 to In2, same 0.4 mm width |
| OIL_SIG | 22.1384776311 | 22.1384776311 | In1 to In2, same 0.2 mm width |
| LED_CAN | 97.2387914372 | 99.2479797464 | Four local In2 segments replaced by six |

The raw oil path does not lengthen. The excitation branch's conservative hot
trace resistance falls from 50.2242 to 22.2867 mOhm at 24 um copper and
4e-8 ohm m resistivity. The LED detour adds 16.7432 mOhm, or 6.6973 uW at
20 mA. These are trace-only bounds: new native fill and affected ground/PDN,
thermal and analog-noise applicability still require review.

Against I23, 15 segments are removed and 12 added. All nonsegment objects,
component positions, vias, pads, outline, logo, RF geometry, and unrelated
segment objects are exactly preserved. Thus the existing straight U.FL entry
and confirmed accessories are unaffected geometrically.

## Reproduction and verification

Run these from the complete engineering recovery root, which supplies the
pinned local Python packages and native I23 artifacts:

```sh
python3 analyses/return_i24/build_candidate.py
python3 analyses/return_i24/study_inner.py
python3 analyses/return_i24/assemble_candidate.py
python3 analyses/return_i24/verify_candidate.py
python3 analyses/return_i24/study_tx_transition.py
python3 analyses/return_i24/study_tx_alternate_reference.py
python3 analyses/return_i24/assemble_final_candidate.py
python3 analyses/return_i24/verify_final_candidate.py
python3 analyses/return_i24/review_native_reference.py
```

`build_candidate.py` reuses only the trusted definition preamble of the I23
corridor solver. It does not execute I23 source-writing code. The source-only
verification separately reparses both saved boards, checks segment and
nonsegment preservation, explicit foreign-copper geometry and pad connectivity,
all seven native track-keepout polygons, protected/raw rules, board/NPTH
clearances, and projections of every existing B/In1 critical signal. The final
check additionally protects full nominal signal width plus 50 um around every
remaining TX gap; the minimum separation to new inner copper is 0.174264 mm,
versus the 0.155 mm allocation. Actual native refill is authoritative.

The required native entry point remains `candidate/run_native_candidate.py` in
the product repository, invoked by `.github/workflows/rvb22-native.yml`. Native
outputs must bind the exact final source hash before using
`control/check_final_native.py --run <run-folder> --iteration I24` and downstream
models. Native ERC/DRC/parity/export success alone cannot establish return
transfer, manufacturing release, physical receiver waveform quality, or
thermal qualification.

## Preserved trials and remaining findings

A complete fixed-anchor CAN_RX route constrained over full-width native ground
was infeasible on either outer layer. Moving the oil chains gives a small,
causal correction instead of lengthening the raw analog signal path.

The bulk rail cannot simply move to In2: existing critical conductors obstruct
several fixed width-transition corridors. No trial bulk segment was adopted.
The nearest feasible 0.60/0.30 mm TX signal via in the studied local grid is
2.393 mm from the launch, after all-layer copper and native via keepouts.
Adding a transfer path solely to reduce the short launch-gap report was not
adopted. The existing vias and all manufacturing geometry are retained.

Six CAN_TX adjacent-In1 gaps remain open. Their alternate-plane geometry must
be checked after the final native refill. GND-02 also requires broader actual
layer-by-layer return and neck-down inspection, including applicable analog,
private-return and power-loop paths; this scoped improvement does not close
that requirement. SI-07 receiver scope and CAN-03 failure-injection acceptance
remain physical evidence requirements. No physical test or fabrication release
is claimed.
