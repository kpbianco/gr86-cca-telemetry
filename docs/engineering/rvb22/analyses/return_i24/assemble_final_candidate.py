"""Final I24 adds a local LED detour around the remaining TX reference gaps."""
from pathlib import Path
import sys,json,hashlib,datetime,uuid,shutil,math
W=Path(__file__).resolve().parents[2];D=Path(__file__).resolve().parent
sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor'),str(W/'control')]
import check_combined_copper as c
from shapely.geometry import LineString
P=W/'iterations/I24_return_clearance/candidate_kicad/GR86_CCA_RevB.kicad_pcb';b,ii,u=c.collect(P);assert not u
r=json.loads((D/'TX_ALTERNATE_REFERENCE_TRIAL.json').read_text())[0]
assert r['path_mm']
# Preserve the original seventeen segment prefix, including collinear vertices.
anchor=[79.9,16.1];oa=r['original_path_mm'].index(anchor);na=r['path_mm'].index(anchor);newpath=r['original_path_mm'][:oa]+r['path_mm'][na:]
key=lambda a,z:tuple(sorted([tuple(a),tuple(z)]))
newkeys={key(a,z)for a,z in zip(newpath,newpath[1:])};oldkeys={key(a,z)for a,z in zip(r['original_path_mm'],r['original_path_mm'][1:])}
oldsegments={c.get(s,'uuid')[0]:s for s in c.child(b,'segment')};remove=[k for k in r['original_uuids']if key(c.get(oldsegments[k],'start'),c.get(oldsegments[k],'end'))not in newkeys]
net=next(n[1]for n in c.child(b,'net')if n[2]=='LED_CAN');newseg=[]
for a,z in zip(newpath,newpath[1:]):
 if key(a,z)in oldkeys:continue
 ident=str(uuid.uuid5(uuid.NAMESPACE_URL,'I24-LED-CAN-reference:'+json.dumps([a,z])))
 newseg.append('(segment (start %s %s) (end %s %s) (width 0.2) (layer "In2.Cu") (net %s) (uuid "%s"))'%(*a,*z,net,ident))
freeze={'id':'I24-RET-03','frozen_before_candidate_edit':True,'date':datetime.datetime.now(datetime.timezone.utc).isoformat(),'input_sha256':hashlib.sha256(P.read_bytes()).hexdigest(),'finding':'I23 LED_CAN In2 segment02837ab8-c764-5353-a2f8-0c1aa8532b66 causes0.057977mm simultaneous inner-plane gap on CAN_TX_MCU segment871a1d07. Preserve alternate plane beneath all six remaining adjacent-In1 interruptions, without crediting alternate copper as established return transfer.','chosen_correction':'Local LED_CAN In2 detour preserving both original through-via anchors, all components and all earlier oil corrections.','six_CAN_TX_adjacent_findings_remain':True,'signal_via_trial_rejected':'Nearest feasible same-size0.60/0.30mm all-layer signalvia is2.393mm from TX launch; added transfer route is not justified merely to reduce reported reference gaps.'}
(D/'SUPPLEMENTAL_REDLINE_FREEZE.json').write_text(json.dumps(freeze,indent=2)+'\n')
s=P.read_text()
for k in remove:
 lines=[line for line in s.splitlines()if line.lstrip().startswith('(segment ')and k in line];assert len(lines)==1;s=s.replace(lines[0],'',1)
s=s.rstrip()[:-1]+'\n'+'\n'.join(newseg)+'\n)\n';dst=W/'iterations/I24_return_clearance_final/candidate_kicad';shutil.copytree(P.parent,dst,dirs_exist_ok=True);(dst/P.name).write_text(s)
j=json.loads((D/'CANDIDATE_MANIFEST.json').read_text());j['source_after_sha256']=hashlib.sha256(s.encode()).hexdigest();j['removed_uuids']+=remove;j['added_segments']+=newseg;j['changed_nets']+=['LED_CAN'];j['status']='FINAL_SOURCE_CANDIDATE_ASSEMBLED_VALIDATION_REQUIRED';j['chains'].append({'net':'LED_CAN','old_layer':'In2.Cu','new_layer':'In2.Cu','old_length_mm':r['original_length_mm'],'new_length_mm':LineString(newpath).length,'only_local_original_segments_removed':remove,'only_local_new_segments_count':len(newseg),'new_path_mm':newpath,'width_mm':.2,'old_hot_trace_R_ohm':r['original_length_mm']/.2*4e-8/24e-6,'new_hot_trace_R_ohm':LineString(newpath).length/.2*4e-8/24e-6,'allocated_indicator_screen_A':.02,'extra_trace_heat_at20mA_W':(LineString(newpath).length-r['original_length_mm'])/.2*4e-8/24e-6*.02**2});j['native_candidate_directory']=str(dst.relative_to(W));j['promoted_from_passing_partial_candidate_sha256']=freeze['input_sha256'];j['all_nonsegment_objects_preserved']=True
(D/'FINAL_CANDIDATE_MANIFEST.json').write_text(json.dumps(j,indent=2)+'\n');print(j['source_after_sha256'],len(remove),len(newseg))
