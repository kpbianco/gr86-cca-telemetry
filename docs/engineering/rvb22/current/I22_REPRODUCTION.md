# Reproduce the I22 review

Run model scripts from the complete recovery root. They use the archived native
I20 filled PCB, placement CSV, mechanical source and `runtime/local` /
`runtime/vendor` dependencies. The repository contains selected inputs/results;
it is not a substitute for the complete numerical recovery archive.

```sh
python3 analyses/review_i22/package_audit.py
python3 analyses/mated_i22/check_mated.py
python3 analyses/review_i22/mechanical_audit.py
python3 analyses/review_i22/electrical_audit.py
python3 control/build_i22_documents.py
python3 control/verify_i22_iteration.py . --repository /path/to/gr86-cca-telemetry/docs/engineering/rvb22
python3 control/build_i22_review.py .
```

The package audit compares independently transcribed manufacturer drawings with
native placement and actual pads. The mated check must run before the populated
mechanical audit, which uses its 153 height rows. The electrical audit reads the
retained finite RF and thermal outputs; it does not rerun those large sweeps or
claim a new thermal mesh solution. Keep all source PDF hashes and model
allocations with any rerun.

The workbook builder `control/build_i22_workbook.mjs` uses Artifact Tool. Set
`RVB22_RECOVERY`, `RVB22_WORKBOOK_INPUT` and `RVB22_OUTPUT_DIR` explicitly.
The input is the preserved I21 workbook version 6, not the earlier user baseline.
It updates the original register's G–J and N columns and the current RVB22 views,
preserving A–F and K–M. It recalculates counts, checks a temporary status change,
restores it, checks formula errors and exports the workbook. Run
`control/verify_i21_workbook.py` with the input workbook, exported workbook,
current JSON register and output audit path to compare the saved XLSX contents.

Regenerating a drawing can change SVG metadata. After any affected evidence is
regenerated, rerun source verification and register/workbook consolidation so
that their hashes describe the delivered bytes. The register builder hashes
dependent current gate/action files after writing them.

I22 leaves CAD, firmware, C05/W02/T03 solids and the TranquilWorks logo unchanged.
An actual CAD/firmware revision requires the pinned native workflow and new
source/output binding before its evidence supersedes I20. Current electrical
source has zero native findings; thermal, supplier, physical and independent
human acceptance remain as recorded in the register.
