#!/usr/bin/env python3
"""Reevaluate the immutable 290-criterion I21 register against I22 evidence."""
from pathlib import Path
import argparse,collections,copy,hashlib,json,re

p=argparse.ArgumentParser();p.add_argument('recovery',type=Path);a=p.parse_args();W=a.recovery;C=W/'control'
read=lambda path:json.loads((W/path).read_text())
sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
old=read('control/review_history/I21/FINAL_REVIEW_REGISTER.json');d=copy.deepcopy(old)
assert old['summary']['current']=={'closed':134,'open':152,'na':4}
V='current/I22_SOURCE_VERIFICATION.json';P='analyses/review_i22/';M='analyses/mated_i22/RESULTS.json'
assert read(V)['status']=='PASS'
assert sha(W/'control/review_history/I21/FINAL_REVIEW_REGISTER.json')==read(P+'REDLINE_FREEZE.json')['input_review_sha256']
closed={
 'LIB-02':('All12 IC/module manufacturer drawing orientations were transcribed independently, transformed using native placement output and compared with151 numbered signal lands. All12 deliberate180-degree errors are rejected. The two displayed views distinguish through-board and bottom assembly interpretation.',[P+'PACKAGE_ORIENTATION_RESULTS.json',P+'package_audit.py',P+'sources/MANIFEST.json','current/MANUFACTURING_AND_ASSEMBLY.md']),
 'LIB-08':('All5 exposed-pad packages match exact manufacturer net, copper/mask and thermal-return recommendations. U501 pad17 remains OIL_EFUSE_RTN; U201 retains nine specified mask/paste lands and48 controlled filled/capped vias. Exact paste/nearby-via quantities and compatible alternative lead lands are recorded.',[P+'EXPOSED_PAD_REVIEW.json',P+'PACKAGE_ORIENTATION_RESULTS.json',P+'electrical_audit.py','current/MANUFACTURING_AND_ASSEMBLY.md']),
 'VIA-06':('All322 vias meet a10K normal-load self-rise budget at2A per individual via, without current-sharing credit. The calculation uses1.76mm board thickness,15um minimum wall, -0.08mm finished-hole tolerance, hot-copper resistivity4e-8ohm*m and conductivity300W/m/K. Maximum calculated rise is5.102515K.',[P+'VIA_CURRENT_THERMAL_BUDGET.json',P+'DRILL_RECONCILIATION.json',P+'electrical_audit.py','current/MANUFACTURING_AND_ASSEMBLY.md']),
 'SI-12':('The original criterion explicitly permits justified analysis-only acceptance. For this board RF network, the user-approved model workflow, exact native geometry/adjacent reference and3,981,312 finite cases give minimum10.219007dB two-port return loss and maximum0.867241dB insertion loss against10dB/1dB limits. This is limited board-network acceptance at documented stack/etch/parasitic allocations.',[P+'RF_RETURN_REVIEW.json','analyses/rf_i18/EXTENDED_NETWORK_RESULTS.json','analyses/manufacturing_i20/RESULTS.json',P+'REVIEW_REPORT.md']),
 'GPS-01':('Exact PA1616D top-view numbering, all20 physical pad functions/nets and manual assembly location agree. Pin1 is PCB(46.7,2.25)mm; centre(54.5,9)mm, F.Cu,0degrees. The current Adafruit-linked V.05 drawing is checked directly; a legacy footprint suffix is not used as manufacturer revision evidence.',[P+'PACKAGE_ORIENTATION_RESULTS.json',P+'package_overlays/U401.svg',P+'sources/pa1616d.pdf','current/MANUFACTURING_AND_ASSEMBLY.md']),
 'MECH-03':('The full height allowance is1.71mm:0.25solder+0.51capturefreedom+0.75warp+0.20deflection, replacing the incomplete1.20mm code allowance. Declared manufacturer-height/componentXY, carrier/support/fastener and full-thickness bowed foil solids pass1,154 populated checks;776 revised mated checks also pass. Minimum modeled gap is0.120mm atR408/T03.',[P+'POPULATED_CLEARANCE_RESULTS.json',P+'mechanical_audit.py',M,'analyses/mated_i22/check_mated.py','current/MANUFACTURING_AND_ASSEMBLY.md'])
}
conditions={
 'LIB-02':'Drawing/placement calculation scope complete. Independent human review under DRW-03 and assembler stencil/process acceptance remain separate.',
 'LIB-08':'Exposed-pad design review complete. Absolute junction/module temperatures, filled/capped construction and stencil/reflow acceptance remain separate thermal/process gates.',
 'VIA-06':'Normal-load budget complete within the stated fabrication bounds. Minimum wall, ring integrity, absolute end/board temperature and fault survival remain supplier/thermal/protection conditions.',
 'SI-12':'Board-network analysis scope complete at the stated finite allocations. Supplier stack acceptance, actual cable/receiver/antenna behavior and installed GPS comparison remain DFM-01 and GPS-09/12/13.',
 'GPS-01':'Exact orientation/pad/placement scope complete. Supplied module identity, actual assembly and operating behavior retain their separate criteria.',
 'MECH-03':'Declared populated tolerance-stack scope complete. Exact materials, supplied accessories and installed physical fit remain MECH-01/06/08/09 and GPS-14.'}
specific={
 'CFG-09':('Selected sensor, harness and Adafruit851/960 accessory identities are controlled. Complete installed harness, mounting environment and phone/OS context still need actual-unit records.','Record the actual installed harness/mounting and phone/OS configuration; accessory product IDs no longer require clarification.'),
 'GPS-09':('User-confirmed851/960 pair is manufacturer-recommended. At3.3V, interpolated typical antenna current9.8mA exceeds4mA detection and is below28mA module current limiting. At3.0V supply, the full bias-feed drop allocation is0.7V/28ohm at25mA. Typical current does not establish guaranteed delivered bias or temperature extremes.','Verify exact delivered bias voltage/drop, antenna hot/cold current and open/short/hot-plug response; settle the851 temperature applicability.'),
 'GPS-14':('The mated model now uses Adafruit851,150±3mm cable,2.0mm allocated maximumOD plus separate route uncertainty. All776 mated checks pass and RF metal setback is16.421mm. The miniature3.4x5.3x3.25mm plug envelope is still an engineering allocation.','Obtain exact supplied miniature-plug dimensions and demonstrate carrier/panel restraint and service retention without using the PCB socket as a strain anchor. Product IDs are confirmed.'),
 'GND-02':('All12 RF segments retain adjacent ground after native antipad treatment. The independent reference map still contains54 digital adjacent-plane interruptions, with zero simultaneous gaps across both ground planes. Alternate-plane availability does not prove acceptable return transfer.','Bound affected transition source/return paths and reroute any path whose bound is not met; retain the explicit54-finding map.'),
 'DFM-02':('The current nominal native/export drill table contains338 plated holes and10NPTH. Minimum nominal via ring is0.15mm. At1.76mm maximum board, nominal finished-hole aspect ratio is8.8; applying the published negative0.08mm hole allowance raises the smallest-hole geometric ratio to14.67. No unsupported NPTH tolerance is assigned.','Reconcile exact connector pin/hardware fit, finished hole/registration tolerances and drill-tool/preplate requirements with the fabrication drawing and actual supplier DFM report.'),
 'DRW-03':('All12 IC/module manufacturer pin1/top/bottom overlays and independent placement-transform checks now pass. The original requirement also calls for an independent reviewer.','Complete the independent human review against the retained overlays and current assembly output.'),
 'WCA-07':('Native pin route/polarity and LED current-loading calculations remain available. A guaranteed low-current/hot optical minimum and dashboard visibility are still absent.','Establish a guaranteed worst-case brightness/current bound for the exact LED operating point and required visibility; do not replace the original optical requirement with current limiting alone.'),
 'REG-02':('The existing finite regulator/current screens pass their model boxes. The retained0.125mm thermal model still reaches143.800Cmaximum board region and is mesh-sensitive; package/local-air margins are unresolved.','Establish converged package-temperature margins at full4.815W,65Cair and70Clandings, or design and verify an enforceable coordinated power/cooling correction.'),
}
lane_extra={
 'LIB':[P+'PACKAGE_ORIENTATION_RESULTS.json',P+'EXPOSED_PAD_REVIEW.json'],
 'GPS':[P+'ACCESSORY_APPLICABILITY.json',P+'PACKAGE_ORIENTATION_RESULTS.json',M,'current/ACCESSORY_REVIEW.md'],
 'MECH':[P+'POPULATED_CLEARANCE_RESULTS.json',M,'current/MANUFACTURING_AND_ASSEMBLY.md'],
 'VIA':[P+'VIA_CURRENT_THERMAL_BUDGET.json',P+'DRILL_RECONCILIATION.json'],
 'DFM':[P+'DRILL_RECONCILIATION.json','current/MANUFACTURING_AND_ASSEMBLY.md'],
 'DRW':[P+'PACKAGE_ORIENTATION_RESULTS.json','current/MANUFACTURING_AND_ASSEMBLY.md'],
 'SI':[P+'RF_RETURN_REVIEW.json'], 'GND':[P+'RF_RETURN_REVIEW.json'],
 'THERM':[P+'THERMAL_ACCESSORY_BOUND.json','current/THERMAL_REVIEW.md'],
 'REG':[P+'THERMAL_ACCESSORY_BOUND.json','current/THERMAL_REVIEW.md']}
for r in d['rows']:
    cid=r['id'];cat=cid.split('-')[0]
    r['previous_I21_disposition']={k:copy.deepcopy(r[k])for k in ['closure','observation','remaining','desktop_status','evidence_status','evidence','evidence_sha256']}
    r['evidence']=list(dict.fromkeys(r['evidence']+[V]+lane_extra.get(cat,[])))
    r['I22_reassessment']='Original criterion and evidence scope reviewed against unchanged I20 source plus I22 results. Prior closure retained unless an explicit I22 change is recorded.'
    if cat=='GPS'and cid not in closed:
        r['observation']='I22 confirms Adafruit851/960 and exact PA1616D pad orientation. Board RF bounds are retained; accessory bias/temperature/plug and installed operating conditions remain explicit. '+('Original operating requirement remains open: '+r['F']if r['closure']=='open'else'Existing original design-scope closure remains applicable. The detailed I21 assessment is preserved.')
    if cid in closed:
        assert r['closure']=='open',cid
        observation,evidence=closed[cid]
        r.update(closure='closed',observation=observation,remaining=conditions[cid],desktop_status='COMPLETED_I22_ORIGINAL_DESIGN_SCOPE',evidence_status='CLOSED_BY_ORIGINAL_SCOPE',evidence=list(dict.fromkeys(evidence+[V])))
        r['I22_reassessment']='Closed by new evidence or justified original analysis-only scope; no physical or independent-human result claimed.'
    elif cid in specific:
        r['observation'],r['remaining']=specific[cid]
    if cid in ['GND-02','REG-02','WCA-07']:
        r['desktop_status']='DESKTOP_WORK_REMAINING'
    if cid in ['CFG-09','GPS-09','GPS-14']:
        r['evidence']=list(dict.fromkeys(r['evidence']+[P+'ACCESSORY_APPLICABILITY.json',M,'current/ACCESSORY_REVIEW.md']))
    r['physical_test_claimed']=False;r['native_candidate_pass_claimed']=True;r['fresh_native_execution_claimed']=False
    r['original_criterion_unchanged']=True

redlines=d['redlines']
for r in redlines:
    r['previous_I21_status']=r['status'];r['previous_I21_remaining']=r.get('remaining','')
    r['previous_I21_correction']=r.get('correction','')
    r['I22_reassessment']='Retained against unchanged I20 electrical source. Original frozen finding/history preserved; current criteria and gates control remaining scope.'
    if r['id']in ['MECH22-03','MECH22-14','HGT22-01']:
        r['remaining']='I22 verifies153fitted heights, the full1.71mm stack,1154populated and776mated checks. Adafruit851/960IDs are confirmed. Exact supplied plug/material/retention and installed conditions remain.'
        r['evidence']=list(dict.fromkeys(r['evidence']+[M,P+'POPULATED_CLEARANCE_RESULTS.json','current/ACCESSORY_REVIEW.md']))
    if r['id']=='I14-03':
        r['status']='ACCESSORY_IDENTIFIED_APPLICABILITY_OPEN'
        r['remaining']='Adafruit851/960 are user-confirmed and product-level standardSMA pairing is documented. Resolve linked-drawing contact discrepancy,60Ctemperature limit, actual bias and supplied plug/retention conditions.'
        r['evidence']=list(dict.fromkeys(r['evidence']+[P+'ACCESSORY_APPLICABILITY.json','current/ACCESSORY_REVIEW.md']))
    if r['id']=='RF22-03':
        r.update(status='ACCESSORY_IDENTIFIED_APPLICABILITY_OPEN',correction='The user confirms Adafruit 851 and 960. Their product pages document standard-SMA pairing; the puck is GPS L1. I22 quantifies typical bias/detection compatibility and retains the linked drawing discrepancy.',remaining='Verify applicable contact/temperature specification, actual bias and external-system performance. The finite RF model uses allocated matched 50 ohm ports.')
        r['evidence']=list(dict.fromkeys(r['evidence']+[P+'ACCESSORY_APPLICABILITY.json','current/ACCESSORY_REVIEW.md']))
new={
 'I22-R01':('ACCESSORY_IDENTIFIED_APPLICABILITY_OPEN','Bound the exact851/960links, replaced stale alternative-cable identity and adopted150±3mm/2.0mmOD envelope. Typical bias/detection applicability is quantified.','851temperature limit, contradictory linked contact drawing, actual bias and supplied plug/retention remain.',[P+'ACCESSORY_APPLICABILITY.json',M,'current/ACCESSORY_REVIEW.md']),
 'I22-R02':('CORRECTED_PACKAGE_DRAWING_REVIEW','Completed all12manufacturer overlays,151signal lands,20GPSfunctions,12negativecontrols and5EPnet/mask/return reviews.','Independent human drawing review and assembler stencil/process acceptance retain their own criteria.',[P+'PACKAGE_ORIENTATION_RESULTS.json',P+'EXPOSED_PAD_REVIEW.json']),
 'I22-R03':('OPEN_THERMAL_AND_RETURN_ENGINEERING','Reconfirmed all12RFreturns and54digitalfindings. Derived the passive65Cboundary/60Caccessory conflict and extracted exactJ401board-region temperatures.','Digital return transfer and mesh/package/local-air/cooling closure remain.',[P+'RF_RETURN_REVIEW.json',P+'THERMAL_ACCESSORY_BOUND.json','current/THERMAL_REVIEW.md']),
 'I22-R04':('CORRECTED_MODEL_DFM_AGREEMENT_OPEN','Ran the complete populated stack and tolerance-aware322via heating budget; reconciled nominal338PTH/10NPTHdrills. MECH03andVIA06design scope close.','DFM02exactfinished-hole/pin fit and supplier drawing/DFM agreement remain.',[P+'POPULATED_CLEARANCE_RESULTS.json',P+'VIA_CURRENT_THERMAL_BUDGET.json',P+'DRILL_RECONCILIATION.json']),
 'I22-R05':('RF_ANALYSIS_ACCEPTED_OPTICAL_BOUND_OPEN','SI12analysis-only acceptance is justified within its original explicit option and finiteboard scope. WCA07was assessed against its complete optical/current requirement.','Guaranteed hot/low-current LED optical minimum and visibility remain WCA07.',[P+'RF_RETURN_REVIEW.json',P+'REVIEW_REPORT.md']),
 'I22-R06':('CORRECTED_FULL_HEIGHT_STACK','Added the omitted0.51mmcapturefreedom; totalextraheight1.71mm. Reexecuted1154populated and776matedchecks with zero intersections.','Declared stack/material/installed acceptance conditions remain; smallest modeled gap0.120mm atR408/T03.',[P+'POPULATED_CLEARANCE_RESULTS.json',M,'current/MANUFACTURING_AND_ASSEMBLY.md'])}
freeze=read(P+'REDLINE_FREEZE.json')
for item in freeze['redlines']+freeze['supplemental_findings']:
    status,correction,remaining,evidence=new[item['id']]
    redlines.append(dict(id=item['id'],lane='I22',status=status,finding=item['finding'],criteria=item['criteria'],correction=correction,remaining=remaining,evidence=[P+'REDLINE_FREEZE.json']+evidence,original_frozen_record=item))
assert len(redlines)==357 and len({r['id']for r in redlines})==357

for g in d['gates']:
    if g['id']=='G22-03':
        g['description']+=' I22 also establishes that passive boundaries>=65C cannot meet the selected851adapter60Cceiling.'
        g['closure_action']+=' Resolve exact accessory temperature applicability; passive contact changes alone cannot close its5Kboundary conflict.'
    elif g['id']=='G22-06':
        g.update(status='ACCESSORY_IDENTIFIED_APPLICABILITY_OPEN',description='Adafruit851and960 are now user-confirmed. Product-level standardSMA pairing is documented. The linked851drawing retains anRP-SMA label and60Cceiling; typicalDCcompatibility and full allocated route pass.',closure_action='Resolve applicable contact drawing/rating, guaranteed delivered bias/current/fault behavior and supplied miniature-plug/retention requirements. No further product-ID clarification is required.')
    elif g['id']=='G22-10':
        g['description']='All153models,1154full1.71mmstack populatedchecks,776revised851matedchecks and37retainedprobeapproaches pass declared envelopes. Minimum modeledR408/T03clearance0.120mm. Actual hot material strength, retained clamp load, creep/fatigue, plug dimensions and installation remain conditions.'

for item in d['inputs']:
    if item['item']=='RF accessory':
        item.update(value='Adafruit 851 U.FL/SMA adapter and 960 active GPS L1 puck; exact PIDs user-confirmed',basis='User links 2026-09-10; current/ACCESSORY_REVIEW.md')
d['inputs'] +=[
 dict(item='Adapter cable nominal length (mm)',value=150,basis='851C934-001drawing, ±3mmtoSMAshoulder'),
 dict(item='Cable maximum OD allocation (mm)',value=2.0,basis='Engineering receiving envelope; nominal1.8mm, plus0.5mmrouteuncertainty separately'),
 dict(item='Added populated height stack (mm)',value=1.71,basis=P+'POPULATED_CLEARANCE_RESULTS.json'),
 dict(item='Minimum populated model clearance (mm)',value=.12,basis='R408toT03; fullstackandbow'),
 dict(item='Minimum RF metal setback (mm)',value=read(M)['pigtail']['RF_metal_setback_mm'],basis=M),
 dict(item='851 operating ceiling (C)',value=60,basis='https://cdn-shop.adafruit.com/product-files/851/C934-001_datasheet.pdf'),
 dict(item='Typical antenna current at3.3V (mA)',value=9.8,basis=P+'ACCESSORY_APPLICABILITY.json; interpolation, not guaranteed maximum'),
 dict(item='Maximum via self-rise at2A (K)',value=read(P+'VIA_CURRENT_THERMAL_BUDGET.json')['maximum_self_rise_K'],basis=P+'VIA_CURRENT_THERMAL_BUDGET.json')]
counts=dict(collections.Counter(r['closure']for r in d['rows']))
assert counts=={'closed':140,'open':146,'na':4}
d.update(checkpoint='RVB22_I22_ACCESSORY_PACKAGE_MECHANICAL_REVIEW_I20_CAD',date='2026-09-10',status='I22_DESIGN_SCOPE_PROGRESS_THERMAL_AND_QUALIFICATION_OPEN')
s=d['summary'];s['prior_I21']=old['summary']['current'];s['current']=counts;s['newly_closed_since_I21']=list(closed);s['reopened_since_I21']=[]
s['redline_status_counts']=dict(collections.Counter(r['status']for r in redlines));s['desktop_status_counts']=dict(collections.Counter(r['desktop_status']for r in d['rows']))
s['release_status']='Six more design criteria closed; thermal engineering and qualification remain open'
s['I22_source_digest_checks']=read(V)['checks_count'];s['I22_physical_tests']=0;s['I22_native_executions']=0
for it in s['iterations']:
    if it['id']=='I21':it['status']='PREVIOUS REVIEW / CURRENT THERMAL EVIDENCE'
s['iterations'].append(dict(id='I22',status='CURRENT REVIEW / MODEL CORRECTION',description='User-confirmed851/960; independent12package overlays; full1.71mmstack and revised cable envelope;322via heating budget;6originalcriteria close. I20CAD/firmwareunchanged.',evidence=[P+'REDLINE_FREEZE.json',P+'REVIEW_REPORT.md',V]))
d['meaning_of_zero']='Zero native findings remains true for unchanged I20CAD/target outputs. Overall zero is not reached:146originalcriteria remain open. I22 adds6design-scope closures and preserves every physical, supplier and independent-human requirement.'
# Format newly authored prose for the workbook while preserving original A-F,
# prior dispositions, frozen findings, identifiers and all evidence paths.
def prose(text):
    text=re.sub(r'(?<=[a-z])(?=\d)', ' ', text)
    text=re.sub(r'(?<=\d)(?=[a-z])', ' ', text)
    text=re.sub(r'(?<=\d)(?=(?:IC|RF|GPS|DC|PTH|NPTH|IDs|ID|OD|NA|CAD|EP|C(?:maximum|temperature|ceiling|boundary|air|landings|and)|K(?:normal|boundary)|V(?:supply)|A(?:per|normal)|C\b|K\b|V\b|A\b))', ' ', text)
    for old,new in {'capturefreedom':'capture freedom','signal lands':'signal lands','hot-copper':'hot copper','finiteboard':'finite board','current3':'current 3','user-confirmed':'user-confirmed','fullstackandbow':'full stack and bow','matedchecks':'mated checks','populatedchecks':'populated checks','boardregion':'board region','atR408':'at R408','minimumR408':'minimum R408','RFmetal':'RF metal','via heating':'via heating','wrong-orientation':'wrong orientation','F.Cu,0':'F.Cu, 0'}.items():
        text=text.replace(old,new)
    return text
for row in d['rows']:
    if row['id'] in closed or row['id'] in specific or row['id'].startswith('GPS-'):
        for key in ['observation','remaining']:row[key]=prose(row[key])
for row in redlines:
    if row['id'] in new:
        for key in ['correction','remaining']:row[key]=prose(row[key])
for row in d['gates']:
    if row['id'] in ['G22-03','G22-06','G22-10']:
        for key in ['description','closure_action']:row[key]=prose(row[key])
d['meaning_of_zero']=prose(d['meaning_of_zero'])
for before,after in zip(old['rows'],d['rows']):
    assert all(before[k]==after[k]for k in ['row','A','B','C','D','E','F']),after['id']
    assert all((W/path).is_file()for path in after['evidence']),after['id']
    after['evidence_sha256']={path:sha(W/path)for path in after['evidence']}
assert {r['id']for r in d['rows']if r['closure']!=next(x for x in old['rows']if x['id']==r['id'])['closure']}==set(closed)
def put(name,data):(C/name).write_text(json.dumps(data,indent=2)+'\n')
put('FINAL_REVIEW_REGISTER.json',d)
put('FINAL_REVIEW_VERIFICATION.json',dict(checkpoint=d['checkpoint'],original_criteria_preserved=290,prior_I21=old['summary']['current'],current=counts,newly_closed=list(closed),reopened=[],redlines=357,all_current_row_evidence_present_and_hashed=True,physical_tests_claimed=0,fresh_native_executions_claimed=0,overall_zero=False))
put('FINAL_INPUTS.json',dict(checkpoint=d['checkpoint'],inputs=d['inputs']))
put('FINAL_GATES.json',dict(checkpoint=d['checkpoint'],gates=d['gates']))
state=read('control/review_history/I21/control/RECOVERY_STATE.json')
state.update(checkpoint='I22',status=d['status'],criteria=counts,redlines=357,source_verification=V,confirmed_accessories=[851,960],newly_closed=list(closed),forwarding_status='I22_PREPARED_PENDING_FORWARD',previous_forwarded_commit=state.pop('draft_PR_commit'),previous_forwarded_tree=state.pop('draft_PR_tree'),forwarding_receipt='control/FORWARDED_I22.json')
put('RECOVERY_STATE.json',state);put('FINAL_HANDOFF_STATE.json',state)
actions=read('control/review_history/I21/control/DESKTOP_ACTION_DISPOSITIONS_CURRENT.json');actions['checkpoint']='I22'
for r in actions['rows']:
    if r['id']=='RDA-06':r.update(work='Full1.71mmstack and153fittedenvelopes pass1154populated/776revisedmatedchecks. Exact851/960IDsconfirmed; suppliedplug/material/retention andinstalledfit remain.',evidence=P+'POPULATED_CLEARANCE_RESULTS.json')
    if r['id']=='RDA-I03':r.update(status='Completed I22 register reconciliation',work='All290originalcriteria preserved;140closed/146open/4NA. All357redlineidentitiesretained. Sixexplicitnewdesign-scopeclosures.',evidence='control/FINAL_REVIEW_REGISTER.json')
    if r['id']=='THERMAL':r['work']+=' I22 establishes the85160Cceiling/65Cpassive-boundary conflict.'
actions['rows'].append(dict(id='I22_PACKAGE_VIA_RF',status='Completed original design scope',work='12manufacturer orientation overlays,5EPnet/maskreviews,322viacurrent/thermalbudget andjustifiedboardRFanalysis-onlyacceptance.',evidence=P+'REVIEW_REPORT.md'))
put('DESKTOP_ACTION_DISPOSITIONS_CURRENT.json',actions)
# Inputs, gates and current action files are also row evidence. Hash only their
# final bytes, after those dependent files have been written.
for row in d['rows']:
    assert 'control/FINAL_REVIEW_REGISTER.json' not in row['evidence']
    row['evidence_sha256']={path:sha(W/path)for path in row['evidence']}
put('FINAL_REVIEW_REGISTER.json',d)
print(json.dumps(dict(checkpoint=d['checkpoint'],current=counts,newly_closed=list(closed),redlines=357,evidence_references=sum(len(r['evidence'])for r in d['rows'])),indent=2))
