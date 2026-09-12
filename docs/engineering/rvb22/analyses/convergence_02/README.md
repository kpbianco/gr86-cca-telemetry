# Convergence02: radio-safe contact and shared-sink feasibility

This is an unadopted mechanical/thermal proposal, not a fabricated or released construction. The unchanged candidate remains the verified I24 board. C01 source publication is complete.

## Corrective finding
C01's insulated contact reaches X=84 mm, only 4.254766 mm from the retained antenna region start X=88.254766 mm. It fails the existing 15 mm metal-setback allocation. Its lower temperatures must not be represented as an acceptable finished cooler. This is a geometric finding, not a measured RF failure.

The two C02 contact faces retain solder mask and a separate insulating TIM. Native-courtyard exclusions are expanded by 0.5 mm. The new central face ends at X=72.7 mm: 15.554766 mm nominal setback, leaving 0.254766 mm after a 0.3 mm lateral allowance. The U121-back face has 33.554766 mm setback. 154 exact bottom-courtyard comparisons pass locally. Neither face overlaps the current carrier base in XY; complete three-dimensional bridges, support, preload, cable and service clearance still require engineering. No new carrier cutout is asserted solely from these face checks.

## Model correction
A thermal contact must not give each board cell its own independent bridge to the cold landing. Each C02 contact has distributed local interface conductance to one common shoe node, one finite bridge to one common sink, and optionally a finite sink-to-air resistance. Carrier/contact occlusion is subtracted once using a geometric union. The original 4.815 W load, 65 C air, 70 kPa pressure, 15 um minimum plating and conservative conductivities are retained.

Contact, mask, TIM, bridge and spreading quantities remain engineering allocations. Finite 1 and 2 K/W external sinks are sensitivity cases, not selected or qualified heatsinks. All outputs are board-region temperatures; they do not establish package junction or immediate module ambient margins. The inherited four-layer face-conductance mesh still lumps disconnected copper islands within a cell. Refinement, topology sensitivity and actual package paths remain relevant. Algebraic energy balance is not mesh convergence.

## Local 0.5 mm screen
Shared baseline at fixed 70 C landing: 126.086961 C maximum board temperature. Two-contact trial at fixed 70 C: 117.430976 C. Trial with 1 K/W finite sink to 65 C air: 116.594674 C board / 68.986963 C sink. Trial with 2 K/W sink: 119.603459 C board / 72.637836 C sink. The 2 K/W case fails the retained 70 C landing target. These figures are preliminary; final bound results require source-pinned refinement.

The first local 0.25 mm direct-LU attempt exceeded the 4 GiB process-group memory allowance and produced no result. Larger systems now use a genuine AMG preconditioner. Resource failure is not a thermal result. Six analytic network tests pass locally; hosted reproduction and refinement are separately recorded.

## Reproduce
From the repository root:
```
python3 -m venv .venv-c02
.venv-c02/bin/python -m pip install -r docs/engineering/rvb22/analyses/convergence_02/requirements.txt
.venv-c02/bin/python -m unittest discover -s docs/engineering/rvb22/analyses/convergence_02/tests -v
.venv-c02/bin/python docs/engineering/rvb22/analyses/convergence_02/check_contacts.py --pcb NATIVE/candidate_kicad/GR86_CCA_RevB.kicad_pcb --out contact-checks.json
.venv-c02/bin/python docs/engineering/rvb22/analyses/convergence_02/run_screen.py --native-dir NATIVE --mesh 0.5 --out c02-0p5
.venv-c02/bin/python docs/engineering/rvb22/analyses/convergence_02/run_screen.py --native-dir NATIVE --mesh 0.25 --out c02-0p25
```
NATIVE must contain freshly filled outputs bound by C01's exact full-content fingerprint. No guessed copper fill is substituted. Larger systems use a real AMG preconditioner; failed memory-limited trials are not thermal results.

## Authority and remaining work
The owner authorized source publication and continued desktop design work, not merge, fabrication, flashing or live-vehicle operation. No 290-criterion pass or thermal closure is asserted by this narrower screen. Resolve the complete cooler/support and package/source-to-contact paths before adopting geometry or declaring fabrication readiness.
