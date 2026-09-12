# Current-source correction: convergence 01

R155 and R156 are TNPU060311K8HWEA00 and TNPU06034K99HWEA00, not the prior TNPW entries. The canonical JSON/CSV have been rebuilt from the actual fitted PCB inventory: 153 fitted references, 74 exact MPNs. No numeric TNPU MSL or production peak was invented. The immutable prior table is preserved in `../convergence_01/support/HANDLING_I22.json`. This is source/process preparation, not an assembler qualification.

The I04 board has 74 fitted manufacturer part numbers and 153 fitted references. All 74 now have a primary-source handling or process-scope entry. Moisture classification is established for 58 part numbers: 28 have numeric MSL values, and 30 have a manufacturer designation of Not Applicable or unlimited floor life. These are component source findings, not 58 assembly acceptance passes.

The remaining 16 numeric classifications are explicitly identified: eight TDK CGA parts, two Panasonic ERA parts, PMEG2010BEA, the GPS module, U.FL, F1, F101 and the local pin headers. Current TDK and Panasonic documents give storage, soldering and cleaning requirements without a numeric MSL. The live Nexperia API advertises an exact PMEG chemical report, but that report could not be retrieved. The other parts retain their specific local or module handling conditions. None is assigned MSL1 by analogy.

The exact-part TDK page review distinguishes six general-termination parts from two soft-termination parts. The two soft parts contain resin and use the linked soft-termination specification; they are not classified as resinless capacitors. Both specifications support the recorded storage and soldering controls. The downloaded midvoltage-derating document applies to a different product and is excluded from the controlling mapping.

The process records now preserve these distinctions:

- MIC5504's exact Microchip material API gives MSL1. Its datasheet lead-local solder temperature is not a whole-body reflow allowance.
- LTC4367 uses the manufacturer's explicit MS8 MSL1 package table; that table does not supply a numeric body peak.
- STD25NF20's exact linked material declaration specifies MSL1, 260°C classification and three cycles.
- The Murata LQ family is MSL1. L406's exact reference specification recommends 245 ±3°C, limits peak to 260°C for 10 seconds, and permits two reflows.
- Vishay exact resistor quality rows retain their manufacturer nonnumeric classifications. The power-shunt profile uses the restrictive 250°C/20-second prose where its table separately allows 260°C.
- The GPS module requires pre-baking before SMT reflow and second-pass placement for double-sided reflow. Its source does not provide a bake time or temperature; no generic bake is invented.
- U.FL retains its 250°C board-surface limit, cycle count, local solder limit, storage and solderability rules.
- Panasonic ERA storage is 5–35°C and 45–85% RH, with one-year stock control. Its flux and cleaning restrictions are carried separately from solder temperature.
- F1's applicable Molex 43045 application specification allows 260°C wave/reflow and recommends 25.4 mm relaxed wire exit before a tie, bend or twist for a 12-circuit housing. Matched contact plating and independent current-path restrictions require comparison with the harness. Root and mechanics were notified. The source gives no local-wave duration; another connector's duration is not substituted.

The existing 243–245°C global peak allocation is below the retrieved component maximum temperatures. This alone is not a complete compatible oven recipe: dwell definitions, ramp, paste wetting, board temperature spread, moisture exposure, cleaning and local-process acceptance remain to be reconciled by the assembler using a measured profile. Nexperia's recommended 20–30 seconds within 5°C of the peak must not be mechanically treated as the same definition as every component's 10-second peak limit. No physical process result is inferred.

Reproduce with `build_primary_expansion.py`, then `build_handling_register.py --pcb <I04 PCB>`, then `verify_handling_expansion.py`. The final verification passes 105 checks, including exact Vishay row extraction, current R153 MPN, all eight TDK source mappings, retained local routing, source hashes and inventory counts. `HANDLING_SOURCE_MANIFEST.json` binds the controlling artifacts and downloaded sources. Web-only observations are labeled factual observations, never recreated manufacturer PDFs.
