# Populated envelope analysis scope

The I08 source corrections are under native evaluation. Existing primary evidence covers the maximum height of all 153 fitted references. This analysis may use conservative courtyard/Fab prisms to screen contact/carrier/service clashes where exact native models are missing, but must retain the assumption that the maximum component/lead/solder XY envelope lies within the adopted bound. Nominal KiCad generic models alone do not establish manufacturer tolerances.

Predeclared redlines:

1. Bind the complete fitted inventory, side, source XY and primary maximum height to the current source; exclude DNP and bare test/mount features deliberately.
2. Include mounting float, board-thickness bound, solder/stand-off and carrier/contact volume. Treat a failed conservative bound as a review redline, not necessarily a physical collision.
3. Check both carrier and the folded thermal contact, not only its board contact rectangle. Include mated connector/cable/service volumes separately.
4. Native STEP file existence is not populated coverage. Inventory missing or mismatched model bodies explicitly.
5. Preserve the manufacturer-XY, material/retention and installed-dashboard conditions when an estimate substitutes for measured or supplier geometry.

RVB22-MECH-NATIVE-02: Actual T01 foil polygon extends to X73.1mm; T01 RF specification uses X72mm. With0.3mm placement and0.5mm bow, actual geometric antenna clearance is88.254766-73.1-0.8=14.354766mm, below the15mm allocation. Retain as a design correction requirement; do not waive by using contact extent as packet extent.
