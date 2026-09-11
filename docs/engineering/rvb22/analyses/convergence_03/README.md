# Convergence03 — refinement disposition and next thermal construction gate

C03 records the decision from the completed hosted 0.25 mm refinement. It does **not** adopt new PCB or cooler geometry.

## Refinement result

The source-pinned C02 hosted run `34567386808` returned both 0.5 and 0.25 mm artifacts. At 0.25 mm the RF-clear C02 layout reaches 127.2730 °C with a fixed 70 °C landing, 126.4143 °C with a 1 K/W finite sink to 65 °C air, and 129.2614 °C with a 2 K/W sink. In the 1 K/W case the modeled sink is 68.9192 °C, U201 source-region mean is 123.6950 °C, and U121 source-region mean is 102.2541 °C. The maximum remains at approximately X=80.25 mm, Y=23.0 mm in the U201 region.

Relative to the 0.5 mm results, the refined maximum increases by 9.8420 °C for fixed landing, 9.8197 °C for the 1 K/W sink, and 9.6580 °C for the 2 K/W sink. This is too large to call the current model mesh-converged. Algebraic heat balance remains excellent, but algebraic convergence is not spatial/model convergence.

## Product decision

C02 remains useful **feasibility evidence**: its contact faces satisfy the scoped XY component exclusions and antenna setback screen, and its shared-shoe/finite-sink network is more physically honest than the earlier idealized cooling boundary. It is not adopted as the finished cooler.

The next construction must preferentially improve the U201 source-to-sink path. U201 is the ESP32-S3-WROOM-1-N8R2 and dominates the refined hotspot; simply improving the external heatsink while leaving the local board/package path unchanged is insufficient evidence. U121 remains secondary but still needs its package/source-to-contact path bounded.

Until a better full construction is demonstrated, retain these hard design gates:

- external sink path conservatively no worse than 1.0384 K/W to 65 °C air if the retained 70 °C landing target is used;
- at least 15 mm metal setback from the retained antenna region;
- at least 0.5 mm populated bottom-courtyard clearance for proposed contact faces;
- complete bridge, support, preload, insulation, fastener, cable and service-clearance model before adoption;
- source/package-to-board thermal bounds for U201 and U121 before any junction-temperature closure;
- repeat source-pinned native CAD, firmware, thermal and affected-criterion regression after the cooler geometry is actually integrated.

## What is explicitly not claimed

No physical thermal test was performed. No package junction temperature is proven. No thermal criterion is newly closed. No C02 contact face, bridge, heatsink, preload mechanism, enclosure interface or PCB change is adopted by this record.
