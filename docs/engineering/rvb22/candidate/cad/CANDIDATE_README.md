# I07 native source compatibility candidate

This candidate corrects I06 serialization without changing any parsed PCB object and promotes two C532 local labels to existing global net names. Firmware, component values, placement, routing and mechanical geometry are unchanged. Native ERC and full schematic netlist parity against I06 must confirm the label correction. Native refill/DRC/export verification is pending; do not fabricate from this engineering candidate.

The PCB is line-separated because the I06 one-line output exceeded KiCad9.0.9's1,000,000-byte line-reader limit. All original source records are preserved. The parent I06 candidate and the failed native runs remain available. See the adjacent REDLINE_FREEZE.json and SOURCE_DELTA.json for the exact change.
