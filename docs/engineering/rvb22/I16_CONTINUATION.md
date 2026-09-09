# I16 RF reference correction

The independent I15 reference screen exposed 4.679 mm of GPS_EXT_ANT above 3.3 V instead of ground. I16 reroutes the complete In2 power detour and removes its broad overlap beneath UART0_RX. It preserves the U.FL centered straight entry, matching network, all component placement and the TranquilWorks logo. The corrected portion is 26.651 mm long, predominantly 2 mm wide; its conservative 125 C /24 um resistance falls from18.441 to13.533 mOhm.

The recovered source has fresh deterministic UUIDs and a new SHA256; it reproduces the already checked route geometry, followed by a repeated explicit-copper preflight. No old source hash is reused. Native refill/ERC/DRC and exact export/reference checks must run on this source.

Thermal engineering remains open: the last .25 mm I15 full4.815 W model reached124.563 C near C206 before package/ripple allowance. Lower-load estimates do not replace the0.75 A requirement. Formal register reconciliation and current C05/W02/T03 mechanics are ongoing. This is an engineering candidate, not a fabrication release.
