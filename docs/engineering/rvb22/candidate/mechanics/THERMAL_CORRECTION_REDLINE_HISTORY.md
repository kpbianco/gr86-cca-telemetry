# Thermal iteration redlines recorded before edits

MECH22-23: Wing-only thermal path fails the4.790W continuous allocation under low emissivity/low pressure; thermal contacts through wing necks alone are not sufficient.

MECH22-24: ESP32-S3-WROOM-1-N8R2 85C rating is ambient immediately outside module, not case maximum. Correct the comparison; this does not waive component/junction temperature limits.

MECH22-25: Evaluate a direct, insulated PCB-back thermal contact outside the manufacturer15mm antenna exclusion but inside the previous broader hardware exclusion. Candidate strip X68.3..72.0,Y11.5..28.0; RF review required before adoption. Uniform-sheet model remains a sensitivity, not native copper proof.

MECH22-26: The position model puts all main-rail heat into the U201 exposed-pad footprint and assumes perfect layer coupling. This is an explicit local-deposition sensitivity, not package heat-flow evidence; resolve with bounds or source-verified package model before claiming a temperature pass.

MECH22-27: C02's right chassis holes have only0.65mm to a thermal window. Correct before final carrier: move them to(57,−11.5)/(57,54.5), extend main carrierY−15..58. Record new fit envelope; do not conceal the increase or reuse the old local ligament value.

MECH22-28: Final I03 connector courtyard check found the enlarged thermal strip atY11.0 overlapping J401 by0.294mm². Move the strip toY11.8..28.8 (17mm foil width); constrain coax exit toward−Y and update thermal area/budget/model. This finding came from the final source, not an imagined physical defect.

MECH22-29: Exact heat-stabilized T18R tie has a head up to5×5×3.7mm, exceeding the old2.8mm blanket underside allowance. Allocate local5.2×5.2×3.9mm head volumes and move the thermal-variant right pair fromY30/36 toY33/39 so heads clear the thermal packet. Keep jacket loops within2.8mm; selected tie/installed grip is handled by independent review.

MECH22-30: The independent retained-tension model requires more left-slot web than C03's1mm nominal. Widen the insulating left spine fromX−5..0 toX−7..0 and shift primary slots toX−3.5. With finished slot width≤3.1mm, each minimum web becomes1.95mm. This is a carrier-only pretension-load correction; PCB and overall mounting envelope stay unchanged.
