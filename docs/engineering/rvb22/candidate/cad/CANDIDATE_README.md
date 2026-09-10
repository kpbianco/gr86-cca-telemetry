# RVB22 I20 engineering candidate

This folder is the current CAD source. Native KiCad outputs are bound by SHA256 to the exact source and pinned tool versions in the current verification manifest. Engineering iteration I20 corrects annotations; its circuit, pads, placement, copper, outline and TranquilWorks artwork are identical to I19.

The coordinated mechanical candidate is C05 carrier with W02 wing flex links and T03 central contact. The bulk capacitor C206 is on F.Cu at (82,-4) on the added grounded tab. All dimensions are millimetres. The overall PCB envelope remains 88.254766 by61.275846mm; the ESP module overhang is additional.

The 3V3 feedback divider is TNPU060311K8HWEA00 / TNPU06034K99HWEA00,11.8k/4.99k,0.02%,2ppm/K. Do not substitute the superseded23.7k/10k parts. C206 is T598X477M006ATE025 and R158 is WSLP0603R0820FEA. Current land-specific library variants and all153 fitted component envelopes are included.

Use current generated BOM, placement, Gerbers, drills, schematic PDF and STEP together.49 filled/capped/planarized via locations are required:48 at U201 pad41 and one at R153. Plating minimum15um is an engineering condition, not implied by a supplier average.

The native checks and calculations concern intended design. Model applicability, purchased accessories, assembly process, thermal installation and unit qualification are explicitly recorded in CURRENT_ACCEPTANCE_CONDITIONS.md and the review register. This folder does not authorize manufacture, energization or vehicle use. Superseded snapshots remain historical evidence.
