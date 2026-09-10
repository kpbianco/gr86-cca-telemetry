# I21 thermal refinement

The previously interrupted 0.125 mm four-layer finite-volume solve is now complete. It uses actual I18 native copper, with geometric equivalence to I20 established in `SOURCE_EFFECTIVITY.json`. No power, environment, conductivity, contact area or plating assumption was relaxed. Total dissipated power is 4.815 W; bulk air is 65°C at 70 kPa; cold landings are 70°C; minimum plated wall is 15 µm. Copper conductivity is 300 W/m·K and FR4 is 0.25 W/m·K. The declared carrier shadow, contact and natural convection/radiation terms remain active.

| Mesh (mm) | Maximum board region (°C) | C206 region maximum (°C) | U201 source-region mean (°C) | U121 source-region mean (°C) |
|---:|---:|---:|---:|---:|
| 0.500 | 125.820 | 99.190 | 123.659 | 107.360 |
| 0.250 | 135.424 | 102.642 | 132.913 | 113.485 |
| 0.125 | 143.800 | 105.171 | 141.354 | 124.463 |

The last solve has 1,199,572 thermal nodes. It converged in four nonlinear iterations, with all linear solver status codes zero, maximum nodal residual 4.23×10⁻¹⁵ W and whole-board energy residual below 8.0×10⁻¹¹ W. Heat flows are 3.459691 W to the landings and 1.355309 W to air. The via-barrel area capture is 99.98995%. These checks establish numerical solution of the discrete network, not convergence to the physical continuum.

Mesh dependence remains material: refining from 0.25 to 0.125 mm raises the maximum by 8.376°C, U121 mean by 10.977°C and C206 maximum by 2.529°C. The previous C206 relocation improves its region temperature, but its remaining 19.829°C distance to 125°C is not a guaranteed package/self-heating margin. No Richardson extrapolation is credited because asymptotic order is not established and topology changes as narrow copper features resolve. The model still has 3,016 cells containing more than one copper polygon. A near-zero algebraic residual cannot erase that approximation.

The source allocations put all 2.58225 W of the MCU allowance into its board exposed-pad region and scale all heat terms from 4.79 to 4.815 W. This is not a validated internal ESP32 module network. Its board result cannot be called a die, shield or local-air temperature. The exact N8R2 module's 85°C ambient rating refers to air immediately outside the module, not a universal 85°C board or case limit. Conversely, setting bulk air to 65°C does not establish that local ambient stays below 85°C in the installed dashboard.

The [Espressif module datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf) provides the operating ambient range but no application-specific module-to-shield/board network. Typical BLE current measured at 25°C cannot replace the full sustained heat allocation with a guaranteed hot-current bound. The [LM5164 datasheet](https://www.ti.com/lit/ds/symlink/lm5164.pdf) supplies package thermal metrics for the exact non-Q1 U121; those metrics require the appropriate package boundary and are not universal board-level thermal resistances. The original thermal criterion REG-02 is therefore reopened.

An additional optimistic boundary comparison is complete in `analyses/thermal_i21/EXISTING_CONTACT_FEASIBILITY.json`. It uses 0.01 K/W at each existing contact while preserving copper, load, air and landing temperature. On the same 0.25 mm mesh, maximum board temperature falls from 135.424°C to 118.656°C, a 16.768°C reduction. This shows that the interfaces matter, but there is still substantial temperature rise through the board even with nearly ideal contacts. The 0.01 K/W values are not an achievable material specification or a manufacturing change. This comparison has the same mesh/package limits as the main model and is not adopted as cooling credit.

Full thermal closure needs a reliable mesh/topology bound, exact package and local-air applicability, and a realizable cooling path whose material/contact/landing assumptions are met. The current candidate is not represented as thermally qualified or ready for unrestricted manufacture.
