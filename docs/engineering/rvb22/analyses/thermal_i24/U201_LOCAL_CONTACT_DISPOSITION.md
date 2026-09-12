# Convergence 01 superseding disposition

The I25 6.5/7 mm exposed-land selection and whole-path <=5 K/W claim below are withdrawn. The original calculation was a partial interface, not a complete heat path; candidate geometry intersects component/trace constraints. `../convergence_01/README.md` and its native-copper results govern the next work. No mask opening or new PCB revision has been adopted.

## Historical I24/I25 discussion (retained, not current acceptance)

# I24 U201 local thermal extraction disposition

Status: **DESIGN REDLINE + FEASIBILITY DIRECTION — NOT THERMALLY QUALIFIED**

## Why the current I24 cooling construction is not a closure

The mechanically corrected I24 carrier preserves a 5 K/W allocation for each wing and a 15 K/W allocation for the central path. The retained board thermal model used 5 K/W for each wing and approximately 13.766 K/W centrally. The corrected mechanical construction therefore does not provide a meaningful thermal-resistance improvement; centrally it is slightly worse.

More importantly, the retained same-footprint feasibility comparison already drove the three historical contacts to an intentionally unrealistic 0.01 K/W each and still produced a 118.656 °C maximum board region on the 0.25 mm mesh. That isolates **board spreading / source-to-contact distance** as a dominant problem. Spending additional design margin only on the three existing remote contacts is rejected as the primary thermal closure strategy.

The 4.815 W total load, 65 °C bulk air, 70 °C landing boundary, 70 kPa pressure, minimum 15 µm plated wall and conservative material properties remain unchanged.

## U201 is the first local extraction target

The exact current PCB geometry provides a useful no-routing-change feature beneath U201:

- U201 is `ESP32-S3-WROOM-1-N8R2`.
- Its modeled board source allocation is 2.58225 W before the retained 4.815/4.79 global scale, or approximately 2.5957 W after scaling.
- The footprint contains a 3.9 mm x 3.9 mm GND thermal pad.
- A 48-via GND array lies directly beneath that pad.
- The vias are 0.6 mm finished copper land / 0.2 mm drill in the CAD and traverse F.Cu through B.Cu.
- The via-array centers span approximately x=77.469766..80.169766 mm and y=20.375..23.075 mm.
- The nearest bottom-side component center found in the preserved geometry is R158, approximately 3.94 mm from the via-array center. A full component-body/courtyard check is still required for the final thermal shoe.

For a conservative analytic screen using 1.76 mm vertical length, 15 µm plated wall and copper k=300 W/mK, the 48 via barrels alone are about 12.06 K/W in parallel. Direct FR-4 conduction through the full 3.9 mm square at k=0.25 W/mK is roughly 462.9 K/W, giving about 11.76 K/W for those two idealized vertical paths in parallel. At the scaled U201 allocation, that vertical resistance alone corresponds to roughly 30.5 K of rise if it carried all U201 heat. This is why reducing only the external contact resistance below a few K/W has diminishing return.

The preserved native B.Mask export has no solder-mask opening over the U201 via field. Therefore a no-PCB-respin backside shoe contacts solder mask/TIM, not bare copper. No low contact resistance is credited until solder-mask thickness/conductivity, TIM, contact pressure, spreader geometry and the 70 °C sink interface are specified and bounded.

## Coarse sensitivity screen

A local recovery run extended the retained four-layer model with one additional bottom-layer thermal boundary coincident with the U201 source rectangle. This is a **2.0 mm provisional-fill sensitivity screen**, not the final I24 native-fill solve, not continuum convergence and not a package/junction model. It uses 15 µm plating, the full 4.815 W load, 65 °C air, 70 °C landings, I24's 15 K/W central path and 5 K/W wing paths.

| Case | Max board region (°C) | U201 source-region mean (°C) | U121 mean (°C) | U151 mean (°C) |
|---|---:|---:|---:|---:|
| I24 contact allocations, no U201 shunt | 128.122 | 124.797 | 101.499 | 91.034 |
| Add U201 bottom contact, 5 K/W to 70 °C landing | 109.416 | 106.698 | 96.570 | 88.009 |
| Add U201 bottom contact, 2 K/W to 70 °C landing | 106.627 | 103.986 | 95.815 | 87.545 |

The 5 K/W shunt removes about 18.7 °C from the coarse board maximum and about 18.1 °C from the U201 source mean. Tightening the hypothetical external contact from 5 to 2 K/W yields only another ~2.8 °C at the board maximum, consistent with the via-field bottleneck. This supports a **manufacturable <=5 K/W board-back-to-landing target** for the next local thermal shoe; it does not prove that target has been achieved by a real stack.

## I25 mechanical/thermal rework direction

1. Retain the corrected I24 three-contact carrier and its geometry/load-path improvements.
2. Add a local, electrically safe thermal shoe aligned to the U201 backside via-field/source rectangle and tied to a verified <=70 °C landing/spreader path.
3. Design the local shoe around a total **board-back-to-landing thermal resistance target <=5 K/W**, including solder mask, TIM, contact pressure, copper/spreader and terminal interfaces. Do not treat TIM bulk conductivity alone as the contact resistance.
4. Re-run the full component/cable/service interference sweep with the shoe, tolerance stack and compression range.
5. Do not use the ESP32 metal shield as a credited thermal path without a bounded module-internal die-to-shield/case model. A top-side pad to the shield may be retained as an experimental parallel path, but its thermal credit requires correlation.
6. After mechanical acceptance, run the exact current native-copper thermal model with the local contact implemented, then refine mesh/topology around U201 and the other power devices.
7. If the current solder-mask-covered backside cannot meet the <=5 K/W board-back-to-landing allocation with qualified materials and tolerances, the preferred PCB correction is a controlled dedicated backside grounded thermal-contact land over/around the existing filled/capped U201 via field. That would be a deliberate minor respin rather than an uncontrolled field modification.

## Criterion effect

- `REG-02`: remains open. The new path is a feasible correction direction, not a converged package margin.
- `THERM-02`: remains open. Junction/package modeling and measured correlation are still required.
- The current I24 `CANDIDATE_SPEC` cooling path is retained as mechanically useful but is **not accepted as thermal closure**.
- No fabrication release is authorized by this disposition.
