# I24 -> I25 thermal closure handoff

This handoff converts the I24 U201 feasibility direction into a bounded I25 design requirement. It does **not** close REG-02 or THERM-02 and it does not authorize fabrication.

## Decision

The solder-mask-covered no-respin shoe is rejected as the primary closure path. The deterministic allocation screen in `../thermal_i25/U201_SHOE_ALLOCATION.json` shows that, with the retained conservative 25 um / 0.2 W/mK solder-mask allocation, 0.30 mm / 3 W/mK TIM, 1 mm copper spreader and a 10 kW/m2K lumped interface-conductance allocation, even an 8 x 8 mm masked contact is 5.130 K/W. That misses the <=5 K/W board-back-to-landing target before tolerance, pressure loss, aging or geometric clearance margin.

The first exposed-land candidate that passes the same allocation screen is 6.5 x 6.5 mm at 4.813 K/W. A 7 x 7 mm exposed land is 4.150 K/W and is preferred if native copper, mask, component-body, courtyard and assembly/service checks permit it. The 10 kW/m2K interface conductance remains an allocation, not a measured property, and must be replaced by selected-material/supplier data or physical correlation before closure.

## I25 CAD redline

1. Add a deliberate grounded backside thermal-contact land centered on the existing U201 48-via field; start with a 7 x 7 mm target and fall back no smaller than 6.5 x 6.5 mm unless the contact stack is reallocated with stronger evidence.
2. Keep the contact electrically on GND and preserve all existing intended net connectivity. Do not convert the U201 shield into a credited thermal path.
3. Define the B.Mask opening explicitly over the selected contact land. Preserve solder-mask dams/clearance to any non-GND exposed copper and verify manufacturer minimums.
4. Add a compliant local shoe/spreader whose nominal footprint does not overhang unverified component bodies. The known nearest bottom-side component center (R158) is about 3.94 mm from the via-field center, so the 7 mm candidate requires an actual body/courtyard/tolerance sweep rather than center-distance acceptance.
5. Bind the shoe to a <=70 C landing path and retain the corrected I24 carrier/wing construction unless the interference sweep forces another mechanical revision.

## Acceptance gates for the next wave

- native KiCad ERC/DRC/unconnected/export/parity: zero findings;
- full 133-net manufacturing connectivity/export reconciliation unchanged;
- U201 exposed-land geometry and B.Mask opening independently reconstructed from the native refill/export;
- no new critical-signal return-reference regression, including the current six CAN_TX nearest-In1 gaps;
- full component/cable/service/hot-shoe tolerance sweep with the local shoe;
- board-back-to-landing allocation <=5 K/W using selected material/contact data, not TIM bulk conductivity alone;
- exact native-copper thermal solve repeated with the local contact, followed by local mesh/topology refinement before thermal closure;
- REG-02 and THERM-02 remain open until package/junction margin and required correlation are demonstrated.

The parallel CAN return task remains separate: the six CAN_TX gaps are real nearest-In1 interruptions caused by the 3V3_BULK_DAMPED region. In2 availability prevents a simultaneous-inner-plane void but is not used to administratively close GND-02. The next routing wave should prefer a causal plane/launch correction that does not introduce a long transfer-via detour or regress the 313-segment critical-net set.
