# Historical mechanical mirror

Current C05/W02/T03 source is in `../../current/mechanics`. Read the current assembly instructions and I22 review before using any file in this historical directory.

# RVB22 mechanics lane

The current deliverable is the C03 carrier with the T01 thermal correction candidate. It preserves positive board capture, corrects the fuse process and provides source-bound mechanical, thermal and RF-isolation evidence. Actual-source full-load thermal safety and populated native solid validation remain explicit gates.

Use these source files together:

- `GR86_RVB_CARRIER_C03_THERMAL.scad`, `MECH_GEOMETRY.scad` and `THERMAL_CONTACT_T01.scad`.
- `HARNESS_CORRIDORS_C03.scad`, `SMA_ISOLATOR.scad` and `PUCK_ISOLATOR.scad` for installation/accessory envelopes.
- `ASSEMBLY_PROCESS_RVB22.md` and `MECHANICAL_INSTALLATION_RVB22.md` for the controlled factory and installation instructions.
- `THERMAL_CONTACT_T01_REVIEW.md` and `REDLINE_DISPOSITIONS.json` for the actual result and remaining conditions.

C02 and the earlier thermal sensitivities are retained as history and comparison. Their hashes and geometry must not be mistaken for the final combined PCB. `FINAL_COMPONENT_ENVELOPES.json` names the PCB actually inspected; the root package owns final merged CAD and manufacturing exports.

From this directory, with Python, NumPy, SciPy, Shapely and sexpdata installed:

```sh
python rebuild_mechanics.py --input /path/to/pre_mechanics.kicad_pcb --output /path/to/output.kicad_pcb --stage all
python structural_service_model.py
python check_rf_isolation.py
python build_thermal_contact.py
python final_component_envelopes.py --pcb /path/to/final_combined.kicad_pcb
python apply_height_supersession.py
python thermal_contact_t01_model.py
python finalize_dispositions.py
```

The unified PCB patch must run on a pre-mechanics candidate, not on a board that already contains the ears/wings. The final thermal calculation takes its outline from the supplied geometry JSON; it is an idealized conductivity sensitivity and does not replace native filled-copper/package thermal analysis. Native KiCad/OpenSCAD/STEP execution is reported separately; source-generated SCAD is not itself a successful solid export.
