# RVB22 harness and fixture contract

Status: source-defined engineering contract. Actual harness and vehicle acceptance remain separate.

PCB SHA256 `c2297b8c857d54f525667e9b2dace5f5f538b7c8d651370cc1b40fa2bdfb39c1`.

| F1 cavity | Actual PCB net | Function | Harness endpoint |
|---|---|---|---|
| 1 | GND | POWER_GND1 | QUALIFIED_SINGLE_FULL_CURRENT_VEHICLE_RETURN |
| 2 | GND | EMPTY_GND_SPARE_NO_WIRE | EMPTY |
| 3 | RAW_12V | FUSED_SWITCHED_12V | QUALIFIED_FUSED_SWITCHED_SOURCE_OUTPUT |
| 4 | unconnected-(F1-SPARE_NC-Pad4) | NC | EMPTY |
| 5 | CANL | CANL | ASC.12 |
| 6 | CANH | CANH | ASC.4 |
| 7 | OIL_SIG | OIL_SIG | HONEYWELL.C |
| 8 | OIL_5V | OIL_5V_OUT | HONEYWELL.B |
| 9 | GND | OIL_RETURN | HONEYWELL.A |
| 10 | unconnected-(F1-SPARE_NC-Pad10) | NC | EMPTY |
| 11 | unconnected-(F1-SPARE_NC-Pad11) | NC | EMPTY |
| 12 | unconnected-(F1-SPARE_NC-Pad12) | NC | EMPTY |

CAN uses SWS AVSS0.3 white/blue, uninterrupted pair ≤0.30m total wire path,15–25mm twist pitch,25.4–30mm relaxed F1 exit before first bend/tie/twist,≤15mm ASC untwist,≥30mm installed bend radius. No extra termination. The wire is not specified as guaranteed120Ω.

ASC: TE1376106-1 housing,1376109-1 strip terminal with1366787-2 applicator; AVSS0.3 is explicitly listed. F1: tin0430300001 reel terminal with63900-4500 production tool. Incoming AVSS OD must be1.30–1.50mm;1.30mm is our incoming limit, not SWS supplier minimum. Full strip/crimp/pull method and source hashes are in HARNESS_WIRE_DELTA.json.

Test the isolated harness against every intended endpoint and all66 cavity pairs. All77 wrong insertions,16 adjacent bridges and6 sensor permutations are enumerated. Board-connected GND at1/2/9 can mask a broken lead.

Honeywell A→F1.9, B→F1.8, C→F1.7. Excitation is an output. Do not connect it to vehicle battery. The243-case wire screen retains72 low-excitation cases at the4.75V board lower corner; remoteB-to-A headroom needs the power model.

One full-current primary return usesF1.1. F1.2 has no harness contact/wire although its board pad remainsGND; no parallel-contact current sharing or redundant-return survival is claimed. Source fuse is at takeoff. F1.2/4/10/11/12 remain empty. Isolate RF shells/mounts from chassis; do not connect a chassis-bonded programmer during vehicle use. Sensor A has no added case jumper. The one-return case is normal; old two-return case is historical/prohibited. OpeningF1.1 invokes the no-primary-return cases. Six finite lost-ground topologies retain dangerous alternate-return current and sensor error; they are not universal unknown-bond proof.

Use molded cavity identifiers; the ASC contact-face view and actual switched source/returns must be bound to the installed vehicle. CAN and sensor endpoint labels are design assignments, not photographs of a finished harness.

Molex430450001-AS A1 correction: tin0430451200 header mates tin0430300001 contacts only; no double-wire crimp and no energized mating. The25.4mm free-exit recommendation is adopted as a design minimum; actual mated plug exit, terminal float and carrier service envelope still require dimensional/installation acceptance. Do not force wires into a basic carrier fit box. The4mm relaxed pair spacing maximum is an installation/model allocation, not a manufacturer guarantee.

The current sectional CAN model completes the prior wire-lane request to combine both free exits with the twisted cable, PCB, protection and receiver. It retains14,400 finite cases and196 fast-edge0.9V recrossing observations; actual receiver hysteresis and vehicle applicability remain acceptance conditions. See current_CAN_effectivity in the JSON for exact result hash and enclosing refined extrema.

Fresh checks: 26/26. No physical test or native CAD run is claimed.
