#!/usr/bin/env python3
"""Verify I22 source reuse, executed model results and the new evidence identity."""
from pathlib import Path
import argparse, hashlib, json

p=argparse.ArgumentParser();p.add_argument('recovery',type=Path);p.add_argument('--repository',type=Path,required=True);a=p.parse_args()
W=a.recovery;D=W/'analyses/review_i22';N=W/'runtime/hosted/run22/extracted/native_I06_hosted'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
read=lambda p:json.loads(p.read_text())
old=read(W/'current/I21_SOURCE_VERIFICATION.json');assert old['status']=='PASS'
native=read(N/'RESULT.json');binding=read(W/'iterations/I20_controlled_handoff/NATIVE_BINDING.json')
checks=[]
def same(label,path,expected):
    digest=sha(path);checks.append(dict(item=label,sha256=digest,expected=expected,passed=digest==expected));assert digest==expected,label
for group in ['cad','firmware']:
    root=W/('iterations/I20_controlled_handoff/candidate_kicad'if group=='cad'else'lanes/firmware_oil/candidate')
    for name,digest in native['inputs'][group].items():
        same('recovery/'+group+'/'+name,root/name,digest)
        same('repository/'+group+'/'+name,a.repository/'candidate'/group/name,digest)
same('native output manifest',N/'OUTPUT_MANIFEST.json',native['output_manifest']['sha256'])
same('native filled PCB',N/'candidate_kicad/GR86_CCA_RevB.kicad_pcb',binding['filled_PCB_sha256'])
freeze=read(D/'REDLINE_FREEZE.json')
same('I21 baseline register',W/'control/review_history/I21/FINAL_REVIEW_REGISTER.json',freeze['input_review_sha256'])
for row in read(D/'sources/MANIFEST.json'):
    same('manufacturer PDF/'+row['path'],D/'sources'/row['path'],row['sha256'])
pkg=read(D/'PACKAGE_ORIENTATION_RESULTS.json');mech=read(D/'POPULATED_CLEARANCE_RESULTS.json');mated=read(W/'analyses/mated_i22/RESULTS.json')
via=read(D/'VIA_CURRENT_THERMAL_BUDGET.json');rf=read(D/'RF_RETURN_REVIEW.json');ep=read(D/'EXPOSED_PAD_REVIEW.json')
assert pkg['status']=='PASS_ALL_12_IC_MODULE_ORIENTATIONS' and pkg['references']==12 and pkg['numbered_signal_lands']==151
assert len(pkg['negative_controls'])==12 and all(x['rejected']for x in pkg['negative_controls'])
assert pkg['source_PCB_sha256']==binding['filled_PCB_sha256']
assert mech['status']=='PASS_DECLARED_FULL_STACK_CLEARANCE' and mech['checks_count']==1154 and not mech['interferences']
assert abs(mech['full_extra_height_mm']-1.71)<1e-12
assert mated['status']=='PASS_DECLARED_ENVELOPES' and mated['checks']==776 and not mated['interferences']
assert via['via_count']==322 and via['maximum_self_rise_K']<via['allocated_self_rise_limit_K']==10
assert rf['RF_segments']==12 and rf['RF_adjacent_reference_gaps']==0 and rf['digital_segment_findings']==54
assert rf['current_network_cases']==3981312 and rf['min_both_port_return_loss_dB']>=10 and rf['max_board_insertion_loss_dB']<=1
assert ep['status']=='PASS_EXACT_EP_NET_AND_LAND_REVIEW' and len(ep['rows'])==5
assert all(x['pass']for x in native['output_postconditions'].values())and native['native_report_finding_count']==0
inventory={str(p.relative_to(W)):dict(sha256=sha(p),bytes=p.stat().st_size)
 for root in [D,W/'analyses/mated_i22']for p in sorted(root.rglob('*'))
 if p.is_file()and '__pycache__'not in p.parts and p.suffix not in ['.pyc','.log']}
out=dict(status='PASS',checkpoint='I22',date='2026-09-10',source_revision='I20',source_PCB_sha256=binding['source_PCB_sha256'],
 filled_PCB_sha256=binding['filled_PCB_sha256'],native_run=binding['run'],native_findings=0,
 checks_count=len(checks),checks=checks,new_evidence_inventory=inventory,
 reused_native_evidence='current/I21_SOURCE_VERIFICATION.json',fresh_native_execution_claimed=False,
 freshly_executed=['12 manufacturer orientation and negative-control checks','1,154 full-stack populated-clearance checks','776 revised mated/service envelope checks','322 via-current/heating calculations','5 exposed-pad net/return reviews','GPS DC applicability and passive thermal boundary bound'],
 reused_model_evidence=['I19 finite power/controller and independent extrema','I18 finite RF model with current I20 route/reference identity','I21 completed fine thermal model; I22 extracts J401 board-region values'],
 package_overlay_visual_QA='All12 final overlays inspected via individual figures and three contact sheets; final U401/U201/U301 figures checked at full view. Views, pin labels and captions are legible.',
 physical_measurements_claimed=0,overall_zero=False)
(W/'current/I22_SOURCE_VERIFICATION.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({k:v for k,v in out.items()if k not in ['checks','new_evidence_inventory']},indent=2))
