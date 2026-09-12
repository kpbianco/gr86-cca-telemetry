"""Adopt only independently verified fixed-via oil-chain corrections."""
from pathlib import Path
import sys,json,uuid,hashlib,shutil,math,collections
sys.path.insert(0,str(Path(__file__).resolve().parent))
import build_candidate as m
c=m.c;D=m.D;W=m.W;P=m.P
trials=json.loads((D/'INNER_TRIALS.json').read_text());chosen=[r for r in trials if r['net']in ['OIL_5V','OIL_SIG']]
assert len(chosen)==2 and all(r['path_mm']for r in chosen)
removed=set();added=[];rows=[]
for r in chosen:
 net=next(k for k,v in m.names.items()if v==r['net']);w=r['width_mm'];pts=r['path_mm']
 # Both anchors are unchanged through-via centres, never naked mid-trace ends.
 for p in [pts[0],pts[-1]]:
  assert any(c.get(v,'net')==[net]and math.dist(c.get(v,'at')[:2],p)<1e-6 for v in c.child(m.b,'via'))
 removed.update(r['original_uuids'])
 for a,z in zip(pts,pts[1:]):
  ident=str(uuid.uuid5(uuid.NAMESPACE_URL,'I24-oil-reference:'+json.dumps([net,a,z])))
  added.append('(segment (start %s %s) (end %s %s) (width %s) (layer "In2.Cu") (net %s) (uuid "%s"))'%(*a,*z,w,net,ident))
 rows.append({**r,'status':'ADOPTED_FIXED_VIA_IN2_CORRECTION','new_layer':'In2.Cu','old_layer':'In1.Cu','trace_resistivity_ohm_m':4e-8,'minimum_copper_um':24,'old_hot_trace_R_ohm':r['old_length_mm']/w*4e-8/24e-6,'new_hot_trace_R_ohm':r['new_length_mm']/w*4e-8/24e-6})
s=P.read_text()
for ident in removed:
 lines=[line for line in s.splitlines()if line.lstrip().startswith('(segment ')and ident in line];assert len(lines)==1;s=s.replace(lines[0],'',1)
s=s.rstrip()[:-1]+'\n'+'\n'.join(added)+'\n)\n'
dst=W/'iterations/I24_return_clearance/candidate_kicad';shutil.copytree(P.parent,dst,dirs_exist_ok=True);(dst/P.name).write_text(s)
j={'status':'SOURCE_CANDIDATE_ASSEMBLED_VALIDATION_REQUIRED','source_before_sha256':hashlib.sha256(P.read_bytes()).hexdigest(),'source_after_sha256':hashlib.sha256(s.encode()).hexdigest(),'removed_uuids':sorted(removed),'added_segments':added,'changed_nets':['OIL_5V','OIL_SIG'],'chains':rows,'physical_tests_claimed':0,'six_CAN_TX_MCU_findings_remain':True,'native_required':True,'affected_PDN_ground_thermal_models_required':True}
(D/'CANDIDATE_MANIFEST.json').write_text(json.dumps(j,indent=2)+'\n');print(json.dumps({k:v for k,v in j.items()if k not in ['added_segments','removed_uuids','chains']},indent=2))
