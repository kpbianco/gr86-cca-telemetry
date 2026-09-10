#!/usr/bin/env python3
"""Reconcile the unchanged 290 review criteria with source-bound I20 evidence."""
import argparse
import collections
import copy
import hashlib
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('recovery', type=Path)
args = parser.parse_args()
W = args.recovery
C = W / 'control'
H = C / 'review_history/I06'
H.mkdir(parents=True, exist_ok=True)
old_path = H / 'FINAL_REVIEW_REGISTER.json'
if not old_path.exists():
    old_path.write_bytes((C / 'FINAL_REVIEW_REGISTER.json').read_bytes())
old = json.loads(old_path.read_text())
data = copy.deepcopy(old)
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads((W / p).read_text())

native = 'iterations/I20_controlled_handoff/NATIVE_BINDING.json'
verified = 'current/I21_SOURCE_VERIFICATION.json'
effect = 'current/SOURCE_EFFECTIVITY.json'
copper = 'analyses/manufacturing_i20/RESULTS.json'
service = 'analyses/service_i19/RESULTS.json'
mated = 'analyses/mated_i18/RESULTS.json'
power = 'analyses/power_i19/SUMMARY.json'
power_check = 'analyses/power_i19/EXTREMA_CONFIRMATION.json'
thermal = 'analyses/thermal_i18/face_15.0/RESULTS.json'
rf = 'analyses/rf_i18/EXTENDED_NETWORK_RESULTS.json'
binding = read(native)
assert read(verified)['status'] == 'PASS'
assert read(copper)['independent_manufacturing_status'] == 'PASS_INTENDED_COPPER_AND_EXPORTS'
cases = sorted(read(thermal), key=lambda r: r['mesh_mm'], reverse=True)
assert cases[-1]['mesh_mm'] == .125

# Closure changes are explicit. Native output does not satisfy a physical or
# supplier criterion merely because it is newer than the previous review.
closed = {
    'CFG-08': ('Pinned KiCad 9.0.9 and firmware workflow successfully opened and regenerated the I20 candidate. All 101 CAD inputs, 32 firmware inputs and 774 output files were freshly checked against their hashes.', [native, verified]),
    'LIB-04': ('Independent expanded-netlist review checks 141 two-terminal components. No collapsed passive and no schematic-to-PCB pad-net mismatch remains.', [copper, native]),
    'LIB-06': ('Expanded schematic has 195 components and 133 nets. Every PCB pad agrees with the schematic netlist, and intended filled-copper connectivity has no disconnected pad net.', [copper, native]),
    'SEQ-09': ('Current firmware retains the reset reason, bounded reset count and diagnostic recovery design. Host diagnostic evidence is bound to the same source compiled by the pinned target build.', ['lanes/firmware_oil/RECOVERED_FIRMWARE_TEST_RESULTS.json', effect, native]),
    'GND-01': ('Independent reconstruction checks 133 actual filled-copper nets, power and signal returns, all copper Gerbers and every plated/NPTH drill. Return reference exceptions remain explicit under GND-02.', [copper, effect, 'control/REG07_SOURCE_COPPER_ANNOTATIONS.json']),
    'GND-08': ('Native filled copper contains no floating padless islands or disconnected pad nets. DRC and unconnected-item counts are zero.', [copper, native]),
    'VIA-02': ('Current filled-reference inspection and retained transition geometry provide local ground transitions for the critical RF path. Remaining digital alternate-reference conditions are recorded under GND-02.', [copper, rf, effect]),
    'VIA-05': ('U.FL local grounds and both RF layer references are connected. All RF centerline segments have zero adjacent-ground gap after their own via antipads. The centered straight 0.700 mm connector entry is preserved.', [copper, effect, rf]),
    'VIA-10': ('Native clearance and density checks pass. The complete mated-body model and all 37 test-probe envelopes pass with minimum 0.700 mm radial clearance. Filled/capped process requirements are separately controlled.', [native, mated, service]),
    'BLE-01': ('The manufacturer antenna region is preserved across all copper layers and the declared assembled solids. The C206 tab and the qualified cable/metal envelopes remain outside the antenna region.', [mated, service, effect]),
    'BLE-02': ('Filled-copper inspection preserves the prohibited antenna region while retaining intended module/RF ground connections. Native parity and all 153 model placements pass.', [copper, native, effect]),
    'MCU-08': ('Current vehicle profile, no-transmit restrictions and debug defaults are bound to the reviewed host configuration checks and the pinned target image. This closes the configuration criterion, not a vehicle transmission measurement.', ['lanes/firmware_oil/RECOVERED_RELEASE_CHECKS.json', effect, native]),
    'FW-02': ('Arduino CLI 1.3.1, ESP32 core 3.3.6 and NimBLE 2.3.6 produced the archived ELF and application binary. All 32 firmware inputs and every archived output hash match.', [native, verified]),
    'LAY-02': ('Actual filled layers and the source-bound switching/coupling review are reconciled with I20. No new native clearance or pad-net finding remains. Conditional field coupling and thermal qualification remain in their own criteria.', [copper, effect, 'control/SI11_COUPLING_PATH_REVIEW.json']),
    'LAY-08': ('Declared populated, mated and RF-accessory geometry has 776 checks with zero interference. The dashboard installation and RF correlation plan remains attached to the selected locations.', [mated, service, 'analyses/installation/DASHBOARD_EXPOSURE_AND_SERVICE.md']),
    'LAY-09': ('Actual native clearance/legend checks pass, including the compact TW logo and the Tag-Connect 0.508 mm rule. All 37 declared probe approaches pass. Bottom probing requires carrier removal.', [native, service, effect]),
    'LAY-10': ('Current source hashes match the I20 native run. Independent comparison finds exact agreement for all four copper Gerbers, 338 plated drills, ten NPTH drills and every pad net.', [native, verified, copper]),
    'DFM-03': ('Separate Gerber/drill reconstruction distinguishes 338 plated and ten non-plated drills, including the Tag-Connect locating holes. The controlled outline contains the C206 thermal tab.', [copper, 'iterations/I18_capacitor_thermal_tab/CURRENT_OUTLINE_GEOMETRY.json']),
    'DFM-10': ('All 153 fitted BOM references match their exact source properties. The 151 placement rows match position/side/rotation. F101 and U401 are explicit manual-assembly exceptions; DNP/test configuration is retained.', [effect, native]),
    'DRW-01': ('The current manifest binds source CAD, native filled PCB, BOM, placement, Gerber, drill, STEP, schematic PDF and firmware image. C05/W02/T03 and the 49 filled/capped locations are the controlling mechanical/process construction.', [native, verified, effect, 'current/MANUFACTURING_AND_ASSEMBLY.md']),
    'DRW-02': ('BOM/PCB comparison covers all 153 fitted references, exact manufacturer part fields and footprint identity. PnP matches all 151 automated placements with explicit manual exceptions F101 and U401.', [effect, native]),
    'DRW-08': ('I20 native ERC, DRC, warnings, unconnected and schematic/PCB parity findings are all zero. The former ESP32 footprint warning is resolved. No finding waiver is needed.', [native, verified, copper]),
    'DRW-09': ('The rerun uses a separate geometric/Gerber parser, not a native CAD pass alone. All four copper exports and every plated/non-plated drill match intended source geometry.', [copper, verified]),
    'DFT-02': ('The current test-point map checks 37 normal probe approaches and nearest ground contacts. Minimum radial clearance is 0.700 mm with a 0.30 mm needle plus 0.10 mm positioning allowance and at least 5 mm slender shaft. Fast signals require a differential or short return probe, not the distant DC ground contacts.', [service, effect]),
    'CCB-06': ('Clean native regeneration, source inventory and complete output manifest are available for I20. Fresh digest and independent copper/export checks agree. The I21 reconciliation does not alter CAD or firmware.', [native, verified, copper]),
}

lane_evidence = {
    'CFG': [verified, native, effect], 'REQ': ['current/EXECUTION_PLAN.md'],
    'LIB': [native, effect, mated], 'WCA': [power, power_check, thermal],
    'PWR': [power, power_check], 'REG': [power, power_check, thermal],
    'SEQ': [power, native, effect], 'GND': [copper, effect], 'VIA': [copper, service],
    'SI': [copper, 'control/SI11_COUPLING_PATH_REVIEW.json'],
    'CAN': ['analyses/can_branch/FINAL_CAN_SOURCE_BINDING.json', 'interfaces/asc_profile/ASC_PROFILE_REVIEW.json'],
    'ADC': ['lanes/firmware_oil/FINAL_I04_OIL_FIRMWARE_EFFECTIVITY.json', effect],
    'GPS': [rf, mated, native], 'BLE': [mated, service, copper], 'MCU': [native, effect, service],
    'FW': [native, effect], 'HAR': ['interfaces/HARNESS_CHECKS.json', 'interfaces/SINGLE_RETURN_EFFECTIVITY.json'],
    'MECH': [mated, service, 'current/MANUFACTURING_AND_ASSEMBLY.md'],
    'THERM': [thermal, 'current/THERMAL_REVIEW.md'],
    'EMC': [rf, 'control/SI11_COUPLING_PATH_REVIEW.json'], 'LAY': [copper, service, effect],
    'DFM': [native, effect, 'current/MANUFACTURING_AND_ASSEMBLY.md'],
    'DRW': [native, verified, effect, 'current/MANUFACTURING_AND_ASSEMBLY.md'],
    'DFT': [service, effect], 'VNV': ['current/EXECUTION_PLAN.md', verified],
    'FMEA': ['current/EXECUTION_PLAN.md'], 'CCB': [verified, native, 'current/EXECUTION_PLAN.md'],
}
lane_observation = {
    'REG': 'I19 finite rail model passes 3,456 circuit cases. Minimum local rail is 3.121029 V and the conservative zero-DC-credit release envelope is 3.594820 V. These are bounded simulations. Full-load thermal closure remains open.',
    'WCA': 'Current electrical finite bounds and independent numerical extrema are available. Native geometry is I20. The 0.125 mm thermal result remains mesh-sensitive and does not prove package temperatures.',
    'THERM': 'The completed 0.125 mm actual-copper solve gives 143.800 C maximum board region and 105.171 C near C206 at 4.815 W, 65 C bulk air and 70 C landings. Refinement changes the hotspot by 8.376 C. Local ambient and package paths remain unresolved.',
    'MECH': 'Current C05/W02/T03 construction has complete declared mated-envelope and service checks. All 153 fitted references have envelope models. Actual material, retention, installation and thermal conditions remain explicit.',
    'GPS': 'The current board RF reference and finite network pass their declared bounds. Exact Adafruit adapter/puck PIDs, connector polarity and temperature applicability remain unconfirmed.',
    'FW': 'The exact current source has a successful pinned target build with verified ELF/BIN identity. No flash readback, phone or live-vehicle result is claimed.',
    'MCU': 'The current module library, all fitted envelope models, pinned target build and declared programming probe approach are verified. Unit-level behavior remains distinct.',
    'DFM': 'I20 native rules and independently reconstructed manufacturing exports pass. Current construction has 49 filled/capped/planarized vias. Actual JLC process acceptance and lot evidence are not supplied by CAD.',
    'DRW': 'Current I20 source/export identity is checked. The coordinated handoff points to C05/W02/T03 and 49 filled/capped locations. Historical snapshots are retained with their original effectivity.',
    'LAY': 'Current source geometry, filled copper, 153 models, TranquilWorks logo and 37 test-probe approaches are verified within declared envelopes.',
}

for row in data['rows']:
    cid = row['id']
    row['previous_I06_disposition'] = {k: copy.deepcopy(row[k]) for k in ['closure', 'observation', 'remaining', 'desktop_status', 'evidence']}
    row['prior_closure'] = next(r.get('N', r.get('closure', '')) for r in read('control/review_input/USER_BASELINE.json')['rows'] if r.get('A', r.get('id')) == cid).lower().replace('n/a', 'na')
    category = cid.split('-')[0]
    extras = lane_evidence.get(category, [effect])
    row['evidence'] = list(dict.fromkeys([p for p in row['evidence'] if (W / p).is_file()] + extras))
    row['historical_missing_evidence'] = [p for p in row['previous_I06_disposition']['evidence'] if not (W / p).is_file()]
    if cid in closed:
        observation, evidence = closed[cid]
        row.update(closure='closed', observation=observation, remaining='Original documentary, design-inspection or execution scope satisfied. Related supplier and physical criteria retain their own dispositions.', desktop_status='COMPLETED_CURRENT_SOURCE_VERIFICATION', evidence_status='CLOSED_BY_ORIGINAL_SCOPE', evidence=list(dict.fromkeys(evidence + [effect])))
    elif cid == 'REG-02':
        row.update(closure='open', observation=lane_observation['REG'] + ' Converter current/electrical screens do not establish the thermal part of this criterion.', remaining='Establish converged package-temperature margins at the stated full-load/air/landing bounds, or implement and verify a coordinated cooling or enforceable power correction.', desktop_status='DESKTOP_WORK_REMAINING', evidence_status='OPEN_THERMAL_ENGINEERING')
    elif row['closure'] != 'na':
        prefix = lane_observation.get(category, 'I20 native source and matched outputs have been verified. The original evidence scope below remains controlling.')
        row['observation'] = prefix + '\nOriginal criterion disposition remains ' + row['closure'] + '. The prior detailed assessment is retained in previous_I06_disposition and the historical register; current evidence and remaining conditions govern this row.'
        if row['desktop_status'] == 'NATIVE_OR_TARGET_EXECUTION':
            row['desktop_status'] = 'CURRENT_NATIVE_COMPLETE_OTHER_EVIDENCE_REQUIRED'
            row['remaining'] = 'Native source/build gate is now complete. Remaining original evidence: ' + row['F']
    row['native_candidate_pass_claimed'] = True
    row['physical_test_claimed'] = False
    row['current_source_PCB_sha256'] = binding['source_PCB_sha256']
    row['original_criterion_unchanged'] = True

specific = {
    'LIB-02': ('All 153 source-to-placement/model identities match. Native parity alone does not constitute the requested independent package-drawing overlay for every IC/module.', 'Complete an exact manufacturer top/bottom/pin-1 drawing overlay against each IC/module footprint and assembly rotation.'),
    'LIB-08': ('Pad and shield connectivity, native mask/clearance and source parity pass. Full package thermal closure is still unresolved.', 'Finish exact manufacturer exposed-pad/package thermal and solder-mask applicability review.'),
    'GPS-01': ('Current PA1616D footprint, expanded netlist, placement and conservative body envelope are consistent. The requested independent exact top-view pad-number drawing reconciliation remains a distinct check.', 'Complete and retain the manufacturer drawing-to-native-pad overlay for PA1616D.'),
    'GND-02': ('All RF segments retain their adjacent ground reference. Some digital segments have adjacent-plane gaps with the alternate ground plane present. That is not an uninterrupted nearest reference.', 'Retain the actual reference-gap map and bound affected digital transitions; reroute any path whose source/return bound is not met.'),
    'DFM-04': ('The current native JLC-targeted clearance, mask and edge rules report zero findings. This is not an executed supplier DFM review or a supplier guarantee of construction limits.', 'Obtain a supplier result for the exact current files and minimum copper, plating, mask and edge conditions. Resolve any returned findings in a new coordinated iteration.'),
}
for cid, (observation, remaining) in specific.items():
    row = next(r for r in data['rows'] if r['id'] == cid)
    row.update(observation=observation, remaining=remaining)

# Every frozen finding is retained. The disposition uses current evidence, and
# combined findings that still include thermal work remain open as a whole.
redlines = copy.deepcopy(old['redlines'])
current_remaining = {
    'RF22-01': 'Native U.FL geometry, refill/DRC and export parity are complete for I20. Exact accessory and parasitic conditions remain in RF22-03/05.',
    'RF22-06': 'Native source and part identity are verified, including the 1.6 nH component. RF model and supplier part allocations remain conditional.',
    'RF22-07': 'Current declared 3D envelopes and native source pass. Allocated RF resistance/parasitics and actual accessory/installed behavior remain conditional.',
    'PWR22-R01': 'Native source validation is complete. Actual-load/model correlation remains conditional.',
    'PWR22-R10': 'I20 native refill/DRC and final combined filled-copper/export checks are complete.',
    'PL22-01': 'Current native/source/manufacturing checks are complete. The 13 us gate and cold-TVS allocations remain model conditions.',
    'FWO16': 'Native refill/DRC is complete. Minimum copper/plating and private-return model conditions remain explicit.',
    'FWO-R19': 'Exact I20 target ELF/BIN images exist and match their archived hashes. Actual unit flash/readback remains separate.',
    'MECH22-03': 'All 153 fitted model references and manufacturer-height envelopes resolve; 776 declared mated checks pass. Actual vehicle fit and supplied dimensions/materials remain conditions.',
    'MECH22-08': 'Current native STEP output is complete. Actual construction/material acceptance remains separate.',
    'MECH22-12': 'Current C05/W02/T03 cooling replaces T01. The completed actual-copper thermal refinement is still mesh-sensitive and package/local-air closure remains open.',
    'MECH22-28': 'Declared mated plug, full 150 mm cable and service volumes pass. Preserve negative-Y U.FL exit and exact dimensional/material assumptions.',
    'EMC22-03': 'Current native fill is verified. Tape thickness, dielectric constant, fringe field and actual chassis resonance remain explicit model/installation conditions.',
    'INT22-08': 'The controlled I20 notes and native rendering/ERC pass. Historical intermediate source notes are superseded.',
    'DOC22-MOLEX-A1-F1-NOTE': 'Current native schematic rendering/ERC is complete. Installed single-return harness acceptance remains separate.',
}
for cid in ['RAIL22-01', 'INT22-02', 'INT22-04', 'INT22-05', 'INT22-06', 'INT22-07']:
    current_remaining[cid] = 'I20 native refill and matched manufacturing checks are complete. The current I19 finite rail/controller/return bounds remain applicable.'
for r in redlines:
    r['previous_I06_status'] = r['status']
    r['previous_I06_remaining'] = r.get('remaining', '')
    if r['id'] in current_remaining:
        r['remaining'] = current_remaining[r['id']]
        r['evidence'] = list(dict.fromkeys(r['evidence'] + [native, verified, effect]))
    if r['status'] == 'BLOCKED_NATIVE':
        r.update(status='CORRECTED_NATIVE_I20', correction=r['correction'] + ' I20 native and matched target outputs now pass.', remaining='No native blocker remains. Original physical/supplier conditions are retained separately.')
        r['evidence'] = list(dict.fromkeys(r['evidence'] + [native, verified]))
    if r['id'] == 'HGT22-01':
        r.update(remaining='All 153 fitted envelope models and 776 mated checks now pass. Exact installed objects and material conditions remain conditional.', evidence=r['evidence'] + [native, mated, service])

for p in sorted((W / 'iterations').glob('I*/REDLINE_FREEZE.json')):
    d = json.loads(p.read_text())
    items = next((d[k] for k in ['redlines', 'native_findings', 'findings', 'known_redlines', 'items'] if isinstance(d.get(k), list)), [])
    iteration = p.parent.name.split('_')[0]
    for i, item in enumerate(items, 1):
        if isinstance(item, str): item = {'finding': item}
        finding = item.get('finding', item.get('issue', item.get('description', item.get('native_finding', {}).get('description', 'Controlled footprint variant for ' + item.get('reference', 'source')))))
        cid = item.get('id', iteration + '-FROZEN-' + str(i).zfill(3))
        action = item.get('correction', item.get('action', 'Resolved through the subsequent coordinated native candidate and recorded source changes.'))
        status = 'CORRECTED_NATIVE_I20'
        evidence = [str(p.relative_to(W)), native, verified, effect]
        remaining = ''
        text = (finding + ' ' + action).lower()
        if any(s in text for s in ['package temperature', 'package paths', 'thermal margin', 'mesh dependence', 'thermal/package', 'native thermal', 'thermal calculation', 'thermal mesh', 'thermal, return', 'full-load thermal', 'full4.815w', '0.43c']):
            status = 'OPEN_THERMAL_ENGINEERING'; evidence += [thermal, 'current/THERMAL_REVIEW.md']; remaining = 'Current refinement is complete but still mesh-sensitive; package/local-ambient closure remains open.'
        elif '851' in text or 'adapter identity' in text or 'accessory identity' in text:
            status = 'ACCESSORY_SPECIFICATION_REQUIRED'; remaining = 'Exact Adafruit item identity, connector polarity and hot-environment specification required.'
        elif any(s in text for s in ['plating allocation', 'foil', 'mated', 'carrier support', 'body dimensions']):
            status = 'CORRECTED_DECLARED_GEOMETRY'; evidence += [mated, service]; remaining = 'Declared dimensions, materials, min plating and supplier/installed conditions remain applicable.'
        elif any(s in text for s in ['register', 'formal review', 'final review', 'documentation', 'current mechanics', 'current docs']):
            status = 'CORRECTED_CURRENT_REVIEW'; evidence += ['current/EXECUTION_PLAN.md', 'current/MANUFACTURING_AND_ASSEMBLY.md']
        if cid in ['I10-FROZEN-004', 'I10-FROZEN-005']:
            status = 'CORRECTED_DECLARED_GEOMETRY'; evidence += [mated, service]
        if cid in ['I19-01', 'I19-02']:
            status = 'CORRECTED_FINITE_POWER_MODEL'; evidence += [power, power_check]
            remaining = 'Finite component/controller/return box applies. No private silicon compensation or physical transient measurement is claimed.'
        if cid in ['I10-FROZEN-007', 'I12-08', 'I18-02']:
            status = 'OPEN_THERMAL_ENGINEERING'; evidence += [thermal, power, rf, mated, service]
            remaining = 'Electrical, RF, native and declared geometry regression is complete. The combined finding still includes unresolved thermal/package closure.'
        if cid == 'I12-06':
            status = 'SUPPLIER_MINIMUM_REQUIRED'; remaining = 'The 15 um minimum wall is documented and modeled; a published average is not a guaranteed finished minimum.'
        if cid == 'I16-SI-02':
            status = 'REFERENCE_CONDITION_RETAINED'; evidence += [copper]
            remaining = 'UART underlap removed. All 54 remaining digital adjacent-plane gaps have alternate ground beneath them; exact digital source/return bounds remain applicable. GND-02 is still open.'
        redlines.append({'id': cid, 'lane': iteration, 'status': status, 'finding': finding, 'correction': action, 'remaining': remaining, 'evidence': list(dict.fromkeys(evidence)), 'original_frozen_record': item})

freeze = json.loads((Path(__file__).parent / 'I21_REDLINE_FREEZE.json').read_text())
for r in freeze.get('redlines', []):
    cid = r['id']
    redlines.append({'id': cid, 'lane': 'I21', 'status': 'OPEN_THERMAL_ENGINEERING' if cid == 'I21-02' else 'CORRECTED_CURRENT_REVIEW', 'finding': r.get('finding', r.get('issue', str(r))), 'correction': r.get('action', r.get('correction', 'See current reconciliation and fresh source checks.')), 'remaining': 'Converged package/local-air analysis and verified cooling correction remain required.' if cid == 'I21-02' else '', 'evidence': [verified, 'current/EXECUTION_PLAN.md', 'current/MANUFACTURING_AND_ASSEMBLY.md', 'current/THERMAL_REVIEW.md']})
assert len({r['id'] for r in redlines}) == len(redlines), 'Duplicate redline identity'

gates = copy.deepcopy(old['gates'])
updates = {
    'G22-01': ('CLOSED_NATIVE_I20', 'KiCad 9.0.9 native refill/ERC/DRC/parity pass with zero findings. Current Gerber, drill, netlist, BOM, placement and 153-model STEP output match.', 'Complete. Rerun the pinned workflow after any CAD/firmware change.'),
    'G22-02': ('CLOSED_TARGET_BUILD_I20', 'Pinned Arduino/core/NimBLE build produced verified target ELF/BIN outputs.', 'Build scope complete. Flash/readback and actual unit behavior remain under physical acceptance.'),
    'G22-03': ('OPEN_THERMAL_ENGINEERING', lane_observation['THERM'], 'Establish a converged actual-source thermal bound and package/local-ambient margins. Evaluate cooling geometry before changing source. Retain the full 4.815 W, 65 C air and 70 C landing requirements unless a documented enforceable alternative is adopted.'),
    'G22-06': ('ACCESSORY_SPECIFICATION_REQUIRED', 'Adafruit U.FL-to-SMA adapter and external GPS puck are selected by product type. Exact product IDs and hot specification remain unbound. Inferred PID 851 documentation has temperature and connector-polarity conflicts.', 'Use actual labels/order records and an exact supplier drawing/specification to settle adapter/puck identity, SMA/RP-SMA polarity, bias current, temperature and mating dimensions.'),
    'G22-07': ('SUPPLIER_PROCESS_ACCEPTANCE', 'Current construction has 49 filled/capped/planarized vias: 48 under U201 pad 41 and one at R153. Minimum finished plated wall is a 15 um engineering condition. F101 needs a separate local solder process.', 'Confirm these exact conditions with JLC for the current fabrication files, plus stackup, copper/mask bounds and lot/process handling. Do not use the historical 13-via list.'),
    'G22-10': ('DECLARED_GEOMETRY_COMPLETE_MATERIAL_CONDITIONS_OPEN', 'All 153 component models, 776 mated geometry checks and 37 probe approaches pass declared envelopes. C05/W02/T03 has complete coax routing and foil construction. Actual hot material strength, creep/fatigue, retention and landing temperature are not established.', 'Accept exact materials, cold-terminal support, retained tensions, installed geometry and temperature conditions. Recheck any changed cable or bracket against the same solids and structural load cases.'),
}
for r in gates:
    if r['id'] in updates:
        r['status'], r['description'], r['closure_action'] = updates[r['id']]

data.update(checkpoint='RVB22_I21_RECONCILIATION_I20_CAD', date='2026-09-10', status='NATIVE_COMPLETE_THERMAL_ENGINEERING_OPEN', PCB_sha256=binding['source_PCB_sha256'], firmware_manifest_sha256='3cc361dbe9ed9dd112bf889d5a3bece277ac45287a5348ebc2ab4255bf74a90f', redlines=redlines, gates=gates)
data['inputs'] = [
    {'item': 'Oil sensor', 'value': 'Honeywell MIPAN2XX150PSAAX, 150 psi sealed gauge, 5 V ratiometric', 'basis': 'User-confirmed 480-MIPAN2XX150PSAAX-ND'},
    {'item': 'Vehicle connector', 'value': 'GR86 ASC connector per Timurrr; switched fused power and one full-current return', 'basis': 'interfaces/HARNESS_CHECKS.json'},
    {'item': 'RF accessory', 'value': 'Adafruit U.FL/SMA adapter and external GPS puck; exact PIDs not confirmed', 'basis': 'User product-type selection; G22-06'},
    {'item': 'Dashboard thermal load (W)', 'value': 4.815, 'basis': thermal},
    {'item': 'Bulk air (C)', 'value': 65, 'basis': thermal},
    {'item': 'Cold landing (C)', 'value': 70, 'basis': thermal},
    {'item': 'Atmospheric pressure (kPa)', 'value': 70, 'basis': 'analyses/thermal_i18/model_fast.py'},
    {'item': 'Finished minimum via wall (um)', 'value': 15, 'basis': 'Explicit construction condition, not JLC average plating'},
    {'item': 'Filled/capped/planarized vias', 'value': 49, 'basis': '48 U201 pad-41 vias plus one R153 via'},
    {'item': 'Mechanical construction', 'value': 'C05 / W02 / T03; 200 unbonded 0.010–0.011 mm C110 laminae per flex member', 'basis': 'iterations/I15_service_mechanics/candidate_mechanics/FLEX_CONSTRUCTION_W02_T03.json'},
    {'item': 'Current rail source', 'value': 'R155 11.8 kohm / R156 4.99 kohm TNPU, 0.02%, 2 ppm/K with independent 0.1% drift bound', 'basis': power},
    {'item': 'Local rail minimum (V)', 'value': 3.121028568, 'basis': power},
    {'item': 'Zero-DC-credit release maximum (V)', 'value': 3.594820346, 'basis': power},
    {'item': 'Target reset maximum (V)', 'value': 3.11605, 'basis': power},
    {'item': 'Thermal mesh minimum (mm)', 'value': .125, 'basis': thermal},
    {'item': 'Maximum board-region temperature (C)', 'value': cases[-1]['max_board_C'], 'basis': 'Finite mesh estimate; not package proof'},
    {'item': 'C206-region maximum (C)', 'value': cases[-1]['C206_board_region_max_C'], 'basis': 'Finite mesh estimate; self-heating/package margin separate'},
    {'item': 'Native findings', 'value': 0, 'basis': native},
    {'item': 'Fitted model count', 'value': 153, 'basis': native},
    {'item': 'Brand artwork', 'value': 'Compact TW, one-color F.SilkS, 8 mm; original SVG polygon identity verified', 'basis': effect},
]

counts = dict(collections.Counter(r['closure'] for r in data['rows']))
data['summary'] = {
    'criteria': 290, 'user_baseline': {'closed': 73, 'open': 213, 'na': 4}, 'prior_I06': old['summary']['current'], 'current': counts,
    'newly_closed_since_I06': list(closed), 'reopened_since_I06': ['REG-02'],
    'redline_status_counts': dict(collections.Counter(r['status'] for r in redlines)),
    'desktop_status_counts': dict(collections.Counter(r['desktop_status'] for r in data['rows'])),
    'native_candidate_passes': 1, 'target_candidate_builds': 1, 'native_findings': 0, 'physical_tests': 0,
    'actual_source_thermal_engineering_complete': False, 'release_status': 'Native checks complete; thermal engineering and qualification remain open',
    'iterations': [{'id': 'I06', 'status': 'HISTORICAL', 'description': 'Prior consolidated register; 110 closed, 176 open, 4 not applicable.'}, {'id': 'I07–I17', 'status': 'SUPERSEDED SOURCE', 'description': 'Native parser, electrical clearance, footprint variants, complete models, logo, RF reference and Tag-Connect corrections.'}, {'id': 'I18', 'status': 'CURRENT GEOMETRY BASIS', 'description': 'C206 thermal tab, copper/outline and mated/service geometry.'}, {'id': 'I19', 'status': 'CURRENT ELECTRICAL BASIS', 'description': 'Precision feedback divider and current hot-copper power model.'}, {'id': 'I20', 'status': 'CURRENT CAD / TARGET IMAGE', 'description': 'Controlled source notes, zero native findings, matching exports and target image.'}, {'id': 'I21', 'status': 'CURRENT REVIEW', 'description': 'Fresh 1,042 digest checks, independent copper/export recheck, completed 0.125 mm thermal solve and original-criterion reconciliation.'}],
}
data['meaning_of_zero'] = 'Zero native findings applies to the verified I20 CAD and target build. It is not zero overall redlines: thermal mesh/package/local-air engineering remains open, and original unit/supplier/installation criteria retain their own requirements.'
for row in data['rows']:
    missing = [p for p in row['evidence'] if not (W / p).is_file()]
    assert not missing, (row['id'], missing)
    row['evidence_sha256'] = {p: sha(W / p) for p in row['evidence']}
assert all(all(a[k] == b[k] for k in ['row', 'A', 'B', 'C', 'D', 'E', 'F']) for a, b in zip(data['rows'], old['rows']))
assert sum(counts.values()) == 290 and counts['na'] == 4
(C / 'FINAL_REVIEW_REGISTER.json').write_text(json.dumps(data, indent=2) + '\n')
(C / 'FINAL_REVIEW_VERIFICATION.json').write_text(json.dumps({'checkpoint': data['checkpoint'], 'original_criteria_preserved': 290, 'user_baseline': data['summary']['user_baseline'], 'current': counts, 'redlines': len(redlines), 'all_current_row_evidence_present_and_hashed': True, 'physical_tests_claimed': 0, 'overall_zero': False}, indent=2) + '\n')
print(json.dumps(data['summary'], indent=2))
