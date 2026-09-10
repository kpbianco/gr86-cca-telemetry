#!/usr/bin/env python3
"""Conservative allocation screen for the I25 U201 backside thermal shoe.

This is a deterministic sizing calculation, not a package/junction or contact
qualification model.  It intentionally separates the solder-mask-covered
I24 option from a deliberate exposed-ground-land respin candidate.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from pathlib import Path

MASK_T_M = 25e-6
MASK_K_W_MK = 0.20
TIM_T_M = 0.30e-3
TIM_K_W_MK = 3.0
COPPER_T_M = 1.0e-3
COPPER_K_W_MK = 300.0
CONTACT_H_W_M2K = 10_000.0
TARGET_K_W = 5.0


@dataclass(frozen=True)
class Case:
    side_mm: float
    mask_present: bool
    area_mm2: float
    r_mask_k_w: float
    r_tim_bulk_k_w: float
    r_contact_k_w: float
    r_spreader_k_w: float
    total_board_back_to_landing_k_w: float
    meets_5_k_w_allocation: bool


def r_layer(thickness_m: float, conductivity: float, area_m2: float) -> float:
    return thickness_m / (conductivity * area_m2)


def evaluate(side_mm: float, mask_present: bool) -> Case:
    area_mm2 = side_mm * side_mm
    area_m2 = area_mm2 * 1e-6
    r_mask = r_layer(MASK_T_M, MASK_K_W_MK, area_m2) if mask_present else 0.0
    r_tim = r_layer(TIM_T_M, TIM_K_W_MK, area_m2)
    r_contact = 1.0 / (CONTACT_H_W_M2K * area_m2)
    r_spreader = r_layer(COPPER_T_M, COPPER_K_W_MK, area_m2)
    total = r_mask + r_tim + r_contact + r_spreader
    return Case(
        side_mm=side_mm,
        mask_present=mask_present,
        area_mm2=area_mm2,
        r_mask_k_w=r_mask,
        r_tim_bulk_k_w=r_tim,
        r_contact_k_w=r_contact,
        r_spreader_k_w=r_spreader,
        total_board_back_to_landing_k_w=total,
        meets_5_k_w_allocation=total <= TARGET_K_W,
    )


def main() -> None:
    sides = [3.9, 5.0, 5.5, 6.0, 6.5, 7.0, 8.0]
    cases = [evaluate(side, mask) for mask in (True, False) for side in sides]
    output = {
        "status": "I25_CONSERVATIVE_SHOE_ALLOCATION_SCREEN",
        "assumptions": {
            "solder_mask_thickness_um": MASK_T_M * 1e6,
            "solder_mask_k_W_mK": MASK_K_W_MK,
            "TIM_thickness_mm": TIM_T_M * 1e3,
            "TIM_k_W_mK": TIM_K_W_MK,
            "spreader_thickness_mm": COPPER_T_M * 1e3,
            "spreader_k_W_mK": COPPER_K_W_MK,
            "lumped_interface_conductance_W_m2K": CONTACT_H_W_M2K,
            "allocation_K_W": TARGET_K_W,
        },
        "cases": [asdict(c) for c in cases],
        "decision": {
            "masked_i24_style_contact": "REJECT_AS_PRIMARY_CLOSURE_DIRECTION",
            "reason": "At the retained conservative properties, even 8x8 mm remains above 5 K/W with solder mask present unless contact assumptions are improved; the known nearby-component geometry also makes such growth nontrivial.",
            "preferred_candidate": "6.5x6.5 mm or larger exposed grounded backside land plus compliant shoe, subject to native copper/mask/assembly clearance and supplier-qualified interface data.",
            "minimum_screened_exposed_side_mm_at_h_10000": min(c.side_mm for c in cases if not c.mask_present and c.meets_5_k_w_allocation),
            "qualification_warning": "The 10 kW/m2K interface conductance is an allocation, not a measured material/contact value. No closure is allowed until the selected stack and pressure are bounded by supplier data or test correlation.",
        },
    }
    out = Path(__file__).with_name("U201_SHOE_ALLOCATION.json")
    out.write_text(json.dumps(output, indent=2) + "\n")
    print(json.dumps(output["decision"], indent=2))


if __name__ == "__main__":
    main()
