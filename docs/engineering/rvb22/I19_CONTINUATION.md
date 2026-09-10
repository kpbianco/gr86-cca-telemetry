# I19 precision feedback correction

I18 native run 34417761748 passed ERC, DRC, board/schematic parity, connectivity, STEP and pinned firmware compilation. Independent manufacturing reconstruction found 133 connected nets, matching copper primitives and zone areas, and matching 338 PTH / 10 NPTH drill hits.

The I18 expanded rail model exposed only 0.278 mV reset margin and a 3.61214 V zero-DC-credit release envelope. I19 replaces R155/R156 with Vishay TNPU 11.8k / 4.99k, 0.02%, 2 ppm/K, preserving all pads, placement, nets, copper, outline and logo geometry. Independent initial, temperature and 0.1% resistor drift allocations give source bounds 3.306564..3.423112 V. Exact ordering codes require supplier acceptance. Current native verification and updated 150°C-copper rail model are required.

I18 capacitor placement is F.Cu at (82,-4), on the added grounded thermal tab. The 0.25 mm face-mesh model predicts the capacitor board region at 102.642°C under the explicit 65°C-air / 70°C-cold-landing cooling contract. This is a conditional board-temperature calculation; it is not package or installed-unit thermal qualification.

Historical I06 review totals and native-blocked records are superseded as evidence where the current native binding applies. The final criterion-by-criterion register is being reconciled; no zero hardware-qualification or manufacturing release is asserted.
