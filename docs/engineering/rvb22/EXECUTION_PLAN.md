# RVB-22 — dashboard CCA redline, revision and verification plan

Date: 2026-09-09. Owner: Kian. This execution plan incorporates the user's new system inputs and authorizes ongoing corrective CAD, firmware, analytical/simulation, mechanical and assembly work. It replaces the previous desktop plan's requests to select a sensor, tap type, antenna family, fabrication service and installation environment.

## Fixed inputs and retained constraints

| Input | Controlled configuration | Evidence/status |
|---|---|---|
| Pressure sensor | DigiKey480-MIPAN2XX150PSAAX-ND; Honeywell MIPAN2XX150PSAAX;150psi sealed-gage,5V ratiometric | Exact user-selected SKU; manufacturer limits and connector mapping recorded in interface evidence. |
| Vehicle tap | GR86 ASC connector following Timurrr's RaceChrono guide | User-selected tap; preserve ASC rather than silently substituting DCM. Derive available channels and cavity functions from the guide, with face orientation and power limitations explicit. |
| RF accessories | Adafruit U.FL-to-SMA adapter into Adafruit external GPS puck | User-confirmed brand/type. PID851+PID960 are matching catalog candidates, not user-confirmed exact IDs; model their stated characteristics conditionally until product identity is bound. |
| Fabrication | JLCPCB Standard PCBA, both sides; JLC041621-7628 four layers,1.60mm FR4Tg170;70/30/30/70µm Cu;0.203/1.030/0.203mm dielectrics;ENIG,greenLDI,whitelegend | Existing controlled JLC order setup/source.120×75mm one-up panel; no purchased controlled-impedance service.12U201pad41vias resin-fill/copper-cap;0.10–0.12mm stencil allocation. Recheck changed outline/panel and process feasibility. |
| Installation | Dashboard; not engine bay | User confirmed. Prior user estimate120°F/48.9°C; retain65°C local thermal screening ambient as an explicit design bound, not a measured hot-soak maximum. No engine-bay exposure profile is imposed on the dashboard CCA. |
| Power behavior | Buck-only; reset and a small RaceChrono gap during crank accepted; F1-3 requires fused switched12V | Existing user decision retained. Normal9.5–16V and previously declared transient cases remain model envelopes, not measured ASC source facts. Sensor/harness portions outside dashboard retain their own applicable environment. |
| Firmware | Changes permitted to achieve required behavior | Preserve physical pins, hardware no-TX design and documented wire protocol unless a separately recorded redline shows a necessary change. The final build identity follows actual source. |
| Iteration | Document all discovered redlines, apply a coordinated revision, then re-evaluate and repeat | Current user instruction. Several revisions are authorized. All failed cases and conditional assumptions remain visible. |

## Starting authority

- Recovered checkpoint: RVB-21, ZIP SHA2568e564b6dd8af42640ac1fa0caa08a5fd5fbb0d403ecc04acd94418b92154b45b.
- Original register:290criteria,114closed/172open/4not applicable; JSON SHA25664566280bcefb55497f5e28f43bf1d70eda4e088ce2a2b9ff00eb65ffd060179.
- Final combined CAD candidate: `audit/online_resolution_rvb21/reset/candidate_kicad/`, PCB SHA256d0b78f5d91297a07ee3783d0d1e3ae3d66c71459b2ed2c9669c8780bac7624cc.
- Final firmware candidate: `audit/online_resolution_rvb21/firmware/candidate/`,2.1.3-revb-online-20260909; sketch SHA256ee3e0305e27e3b543fcb816e4da38f1199ce7f8c878c4bb91a13b19ef995a3fd.
- Existing native-verified CAD/manufacturing and compiled2.1.2 sources remain predecessor evidence. A staged change does not inherit their passes without an impact check.
- Current GitHub repository is `tranquilWorks/gr86-cca-telemetry`; the recovered engineering checkpoint is ahead of its older firmware-only contents. Do not overwrite current repository firmware merely to deliver a local engineering job.

## Per-iteration contract

1. **Inventory before integration.** Each lane records redline ID, source/hash, affected review criteria, observed failure or missing bound, correction, numerical acceptance and dependencies. Preserve the172-row desktop action map and revisit already-closed criteria affected by changes. Investigative trials remain disposable candidates, not a promoted board revision.
2. **Freeze the complete known redline set.** Collect every lane's discoveries in the iteration register. Keep technical defects, model-bound questions, execution gates and external unit/installation facts separately identifiable. A known failing corner cannot be omitted from the frozen set.
3. **Apply the coordinated change.** Integrate lane object/file deltas against the same frozen source, detect overlapping edits, update schematic/PCB/BOM/footprints/firmware/mechanics/process instructions together. Preserve redline-to-change traceability and an exact before/after patch.
4. **Verify the exact result.** Run native refill/ERC/DRC/exports and target compile where an actual runner is available; run independent connectivity/clearance, source-execution, analytical circuit, timing, RF, thermal and mechanical checks. Tie every output to the resulting source hash. Explicitly distinguish new execution from inherited evidence.
5. **Review all results.** A failed or inconclusive criterion creates/reopens a redline with a reproducer. Correct all actionable redlines in the next coordinated change and rerun the affected regression set. Do not rerun unrelated tests just to inflate counts.
6. **Promote only a matched set.** Final product CAD, firmware image, BOM/CPL, Gerbers/drills, drawings, assembly model and evidence must agree. Until native gates and matching exports exist, retain a clearly labeled engineering candidate. No fabrication order, live vehicle fault injection or protected-branch merge is implied.

The stop condition is **zero unresolved actionable engineering redlines within the accepted envelope**, with no failed case hidden by a changed label. Unit-specific observations, supplier guarantees and uncertain model inputs remain listed for the requested final review. Original acceptance rows are not all declared closed merely because every row has a proposed action.

## Engineering lanes and first correction targets

| Lane | Work to execute | Initial targets |
|---|---|---|
| Interfaces/harness | Bind exact sensor characteristics, ASC cavities/availableCANIDs, accessories, connector faces and current paths; update harness contract and error/fault inputs. | Correct ASC channel expectations; retain ratiometric measurement already present; establish sealed-reference semantics; resolve source-feed and accessory-document inconsistencies. |
| RF | Center J401 pad entry; inspect actual copper/return/keepout; rebind matching and distributed RF models to final route and selected GPS band. | Side-fed J401; actual Adafruit GPS-L1 accessory limits; distinguish exact geometry checks, bounded circuit model and unavailable full connector EM characterization. |
| Power/reset | Reconstruct distributed rail resistance/capacitance, guaranteed component limits and regulator/supervisor dynamics; execute correlated circuit/ODE calculations and revise hardware where needed. | Reset headroom and local versus regulator rail; input-filter/sourceL; fault energy/SOA and fuse coordination. Count local capacitors and all distribution impedance. |
| Firmware/oil | Implement corrections in the actual candidate; exercise timing, queues, invalid/fault/stale/epoch/recovery paths and full oil transfer/error models. | Oil short-dip margin; bounded nonblocking service; compile identity drift; ASC comments; sensor calibration semantics. |
| Mechanics/assembly | Add positive mounting restraint, component-inclusive clearance and cable/service provisions; fix incompatible thermal/process allocation. | Missing+Xrestraint; feasible mounting ears/supports within panel constraints; on-board fuse versus reflow profile. New envelope is a requirement for installation, not claimed measured dashboard fit. |
| Integration/verification | Combine all redlines and source deltas, perform independent source/geometry checks and requirement impact review; publish a concrete execution handoff. | Exact source matching, native job, repeated correction of check failures, complete traceability and persistent checkpoint. |

## Numerical evidence and tool choice

ADS is preferred by the user for power simulation, with equivalent methods accepted. ADS is not currently exposed. Use executable MNA/ODE/transmission-line models with explicit equations, source-derived limits, solver convergence and independent correlation when possible. These are the named tools actually used; do not rename them as ADS or claim unavailable device internals are simulated. Analytical interval/monotonic bounds can establish a design limit; random/finite sweeps alone are searches, not universal proofs.

Artwork connectivity is established from real copper objects and plated layer transitions, compared against intended pad endpoints. Net names alone do not establish a connection. Test isolation under documented geometry tolerances and exercise deliberate cut/bridge/wrong-layer negative controls where the checker is new or modified. Continuity does not itself establish current capacity, reference quality or as-built solder/barrel integrity.

For RF, retain manufacturer land/keepout requirements and evaluate signal and return transitions together. Use exact physical geometry in the model and state the connector/cable uncertainty. The prior circuit sweep does not establish the new launch's full3Dperformance.

## Actual execution access

This environment currently lacks native KiCad/pcbnew, ArduinoCLI and ADS. Cached ngspice42 cannot execute because an Xft dependency is absent. The ordinary package route failed under runtime permissions and the official dependency download ended with a canceled network approval; those routes are not bypassed. Local Python/C++ analysis and source tests remain executable.

The established portfolio runner invokes commands on the machine where it is run; no remote KiCad service endpoint is exposed by the current tools. The current product repository does not yet contain the remembered verification scripts. Provide a portable native job and exact candidate package to that established local workflow. A delivered job is not a completed native run. Reconcile the old host-only CCA-M0-01 contract with this newly authorized hardware-corrective scope instead of pretending its exclusions describe the current user request.

## Forwarding and delivery

Prepare the new scope, all-redlines iteration contract and native job as a reviewable handoff in the established product repository. Product source remains owned by that repository; the complete engineering source/evidence checkpoint remains separately identifiable until adopted. Forwarding the handoff does not request or perform supplier acceptance, third-party email, fabrication or live hardware operation.

At completion, report the changed revision/source hashes, exact tests/calculations, all redline dispositions, remaining execution/external conditions and next reproducible action. Preserve the original criteria and their evidence alongside any newly supported closure. Continue engineering work through failures rather than ending with another unexecuted plan.
