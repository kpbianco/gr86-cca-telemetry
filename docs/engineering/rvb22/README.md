# RVB-22 corrective engineering handoff

This is the user's authorized continuation of the GR86 dashboard CCA engineering hold. It begins from the recovered RVB-21 engineering checkpoint, which is ahead of the older firmware in this repository. See `EXECUTION_PLAN.md` for confirmed interfaces, the complete iteration protocol and acceptance boundaries.

The user selected Honeywell MIPAN2XX150PSAAX, the GR86 ASC connector following Timurrr's guide, Adafruit U.FL-to-SMA and external GPS puck accessories, the existing JLCPCB setup, and dashboard installation. Firmware, CAD, mechanics and assembly corrections are authorized. Failed cases and redlines are retained before each coordinated revision.

## Scope and effectivity

This new hardware/firmware corrective batch is distinct from the historical CCA-M0-01 host-only, behavior-preserving contract. That older batch remains historical; its exclusions do not describe the user's new request. This handoff does not replace the repository's older firmware or assert that staged engineering CAD is ready to order.

The downloadable RVB-22 engineering checkpoint carries the full baseline, corrected candidate, redline history and executable model evidence. Candidate source hashes and the final integration manifest identify what to run. Never combine a new candidate with predecessor Gerbers, placement files or compiled binaries.

## Native job

`run_native_candidate.py` runs on the local engineering machine where the required tools are installed. It receives explicit candidate directories, copies them into a new output directory, refills the board, runs native ERC/DRC and schematic parity, exports review artifacts and builds the embedded application. It never modifies the input candidates or flashes hardware.

```bash
python3 docs/engineering/rvb22/run_native_candidate.py \
  --cad-dir /path/to/rvb22/candidate_kicad \
  --firmware-dir /path/to/rvb22/candidate_firmware \
  --pcbnew-python /path/to/python_with_pcbnew \
  --nimble-dir /path/to/NimBLE-Arduino \
  --output /path/to/new_rvb22_native_results
```

Required versions: KiCad CLI and pcbnew 9.0.9, Arduino CLI 1.3.1, ESP32 core 3.3.6, NimBLE-Arduino 2.3.6. The full ESP32-S3 board configuration is pinned in the script. Explicit executable/configuration options are available through `--help`. `--component-step` also requests the available component models; a component inventory must still establish completeness.

The job verifies artifact structure and source identity. A zero process exit without real filled copper, reports, netlists, Gerber layers, drill files or requested ELF/BIN files fails with `INCOMPLETE_NATIVE_OUTPUT`. It does not silently waive native findings. JLC panel generation, supplier BOM/CPL reconciliation, controlled drawings and populated collision checks remain project-specific follow-on gates.

The Work session ran the actual preflight and obtained `BLOCKED_RUNTIME`; no callable KiCad/Arduino service was exposed. The runner's negative controls and archived-format tests passed. These checker tests are distinct from a new native board or embedded build. Return the whole result directory, including `RESULT.json`, `OUTPUT_MANIFEST.json`, source inventories and logs, for the next corrective iteration.

## Acceptance

Review the exact combined candidate after all known redlines have been applied. Any new failure creates a recorded redline and a further revision. Numerical evidence is accepted for the stated modeled design claim; missing device guarantees, installation facts and actual-unit observations remain explicit. Manufacturing adoption requires a matching source, build and export set. This draft does not authorize fabrication, ordering, live vehicle faults or merging.
