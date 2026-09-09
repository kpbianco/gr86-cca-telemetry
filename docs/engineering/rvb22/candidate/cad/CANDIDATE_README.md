# GR86 CCA RVB22 — I06 handoff / I04 electrical revision

This folder is the current source candidate. The PCB remains byte-identical to the checked I04 compact-shunt revision. I06 changes one existing schematic note to match the current single-return F1 harness. Every electrical schematic object is unchanged. I05 and the former two-return instruction are retained as history.

PCB SHA256: c2297b8c857d54f525667e9b2dace5f5f538b7c8d651370cc1b40fa2bdfb39c1

Use the matched firmware 2.1.4-revb-recovered-20260909, C03 thermal carrier/T01 contact and current ASC harness contract from this checkpoint. The selected sensor is Honeywell MIPAN2XX150PSAAX. Exact Adafruit accessory PIDs remain unconfirmed.

Executed source screens report zero foreign-copper gaps below 0.15 mm, zero previous explicit-pad connectivity regressions and zero new courtyard overlaps. These checks cover source pads, tracks and plated vias. They do not supply native zone fill, manufacturing outputs or as-built continuity.

The current KiCad and Arduino native preflight is BLOCKED_RUNTIME. The latest original manufactured/compiled historical package cannot certify this changed candidate. Do not use its Gerbers or binary as RVB22 outputs. Run the supplied native runner against this folder and the matching firmware directory; keep its output in a new directory and retain reports/hashes.

Current manufacturing instructions include 13 specifically located filled-and-capped vias (12 at U201 and one at 41.4, 21.0 on +3V3 near R153), factory-local F101 after global reflow, and the exact current BOM/assembly split. Source-derived CSVs are engineering review tables, not native JLC export files.

Release remains on hold. Full-load thermal transfer through the actual populated copper and package paths is unresolved; the dimensioned T01 path is a conditional model. Regulator/private-die correlation, actual RF/ESD/vehicle behavior, accessory identity, supplier process and physical acceptance conditions remain listed in the checkpoint review register. No blanket zero-issue or physical-pass claim is made.

F1 harness: one full-current power return at cavity 1; cavity 2 has no terminal or wire while its PCB pad remains GND. Cavities 2, 4, 10, 11 and 12 are empty. Use the current seven-lead harness contract, including the 25.4–30 mm relaxed connector exit before bending, tying or twisting.
