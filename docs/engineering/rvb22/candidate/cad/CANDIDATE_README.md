# I09 drill-aware native correction candidate

I08 reduced native DRC from 181 to 13 findings: one rail-to-Tag-Connect-NPTH conflict and 12 footprint-library differences. ERC, unconnected nets and schematic parity were zero, and all 566 pad/net identities survived native load/refill.

I09 reroutes the 2 mm inner rail with every NPTH hole included as an obstacle. Source custom clearance and hole-to-copper checks must pass, followed by native verification. The export runner resolves actual installed model files in a separate export-only PCB copy and generates native footprint-variant proposals for review. These proposals are not silently adopted and DRC is not suppressed.

Native reevaluation, library-variant adoption, populated model coverage and affected rail/return/RF/thermal regression remain open. No manufacturing release is implied.
