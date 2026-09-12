# I24 independent manufacturing reconstruction

Final source PCB SHA256: `5b373f6033fdbd18f126f8ca054b619abada2568778c5a8fc303c4e756fb06c8`.

Native filled PCB SHA256: `11533ea91c3bc4c61dd7066e0001914a72bc90b27afb38d321c43b8feb90e3b1`.

The separate Shapely/Gerbonara analysis passes all 133 net connectivity checks across 4,378 copper objects, with no disconnected pad nets or padless islands. All four native copper Gerbers match the PCB's line, flash and region geometry; each zone-union difference is exactly zero at parser precision. All 338 plated and 10 non-plated drill hits match the board geometry within the unchanged 0.00050001 mm export-rounding tolerance.

Independent schematic comparison matches 502 PCB pad identities to 502 schematic nodes in both directions with no missing nodes or net mismatches. All 141 two-terminal components have distinct node identities. Native outputs retain all 14 passing postconditions and zero ERC, DRC, unconnected or schematic-parity findings.

The unchanged critical-signal screen checks 313 outer signal segments. All 12 RF segments retain zero adjacent-plane gaps. Six CAN_TX_MCU adjacent-plane findings remain; the sum of simultaneous both-ground-plane gaps is zero. Detailed per-UUID results are available under `reference.all_checked_segment_results` in `RESULTS.json`; return-path disposition is owned separately under `analyses/return_i24`.

This verifies intended CAD geometry and matched exports. It does not establish fabricated copper, plating, solder, installed harness contacts, or manufacturing release. Reproduction arguments and file hashes are recorded in `EXECUTION.json`.
