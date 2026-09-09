# I13 thermal-via construction delta

I13 replaces the twelve U201 pad41 heat holes with **48 global GND vias**, each 0.60mm land / 0.20mm drill. The 0.45mm lattice retains 0.25mm nominal hole-edge spacing. The right column, top row and lower-right corner of the rejected64-hole trial are omitted to retain CAN_RX_MCU and C206 clearances. No routing, device MPN, signal-pad definition or rule minimum is relaxed.

The current controlled filled-and-capped total is **49 locations:48 under U201 plus the existing R153 pad2 location**. This supersedes the13-location lists for earlier candidates. The current source-change JSON enumerates all48 new UUIDs and coordinates; R153 retains its prior source identity. Filled/capped construction, planar caps, and the U201 paste/stencil process remain required to prevent solder loss into holes. Unfilled or merely tented holes are not an accepted substitute. No thermal conductivity is credited to fill material in the barrel-only calculation.

[JLCPCB capabilities](https://jlcpcb.com/capabilities/pcb-capabilities) list0.20mm preferred via drills,0.20mm minimum via hole-edge spacing, and filled/capped epoxy or copper-paste options for0.15–0.55mm via diameters. Exact four-layer process applicability, minimum plating, cap planarity and lot acceptance still need supplier evidence. The nominal0.25mm spacing is above the published nominal via-spacing rule; finished tolerances remain explicit construction conditions.

The 48-hole pattern has four times the nominal plated-barrel area of the previous twelve0.20mm holes at equal plating and stack thickness. This ratio alone is **not** a board-temperature result. Native refill/DRC, populated STEP, the full source-geometry thermal network and assembly review must be rerun before thermal adoption.
