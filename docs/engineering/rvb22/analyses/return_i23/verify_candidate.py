from pathlib import Path
import sys,json,hashlib,math,collections
W=Path(__file__).resolve().parents[2];D=Path(__file__).resolve().parent
sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor'),str(W/'control')]
import check_combined_copper as c
import sexpdata as sx
from shapely.geometry import LineString,Point
from shapely.ops import unary_union
old=W/'iterations/I20_controlled_handoff/candidate_kicad/GR86_CCA_RevB.kicad_pcb';new=W/'iterations/I23_return_clearance/candidate_kicad/GR86_CCA_RevB.kicad_pcb'
a,ai,u=c.collect(old);b,bi,v=c.collect(new);assert not u and not v
j=json.loads((D/'INTRUSION_TRIAL.json').read_text());removed=set(j['removed_uuids']);oldseg={c.get(s,'uuid')[0]:s for s in c.child(a,'segment')};newseg={c.get(s,'uuid')[0]:s for s in c.child(b,'segment')}
added=set(newseg)-set(oldseg);assert set(oldseg)-set(newseg)==removed
assert all(oldseg[k]==newseg[k]for k in set(oldseg)&set(newseg))
nonseg=lambda t:[x for x in t if not(isinstance(x,list)and x and str(x[0])=='segment')]
assert nonseg(a)==nonseg(b)
assert {c.get(oldseg[k],'net')[0]for k in removed}=={54,56,62}
assert {c.get(newseg[k],'net')[0]for k in added}=={54,56,62}
ov,oc=c.analyze(ai);nv,nc=c.analyze(bi)
key=lambda r:(r['layer'],*sorted([r['a'],r['b']]))
ov={key(r):r for r in ov}
regress=[r for r in nv if key(r)not in ov or r['gap_mm']<ov[key(r)]['gap_mm']-2e-6]
lost=[dict(net=n,old_group=g)for n,gs in oc.items()for g in gs if len(g)>1 and not any(set(g)<=set(q)for q in nc[n])]
assert not regress and not lost,(regress,lost)
rows=[]
for r in j['chains']:
 if r['status']!='REROUTED_PREFLIGHT_PASS':continue
 oldsq=r['original_length_mm']/r['width_mm'];newsq=r['new_length_mm']/r['width_mm'];rho=4e-8;t=24e-6
 # Deliberately20mA, above the selected indicator operating allocation. This
 # is a trace-only voltage/power screen; optical hot-current guarantees stay open.
 rows.append(dict(net=r['net'],old_layer=r['layer'],new_layer=r['new_layer'],old_length_mm=r['original_length_mm'],new_length_mm=r['new_length_mm'],old_R_hot_ohm=oldsq*rho/t,new_R_hot_ohm=newsq*rho/t,added_drop_at20mA_V=(newsq-oldsq)*rho/t*.02,added_heat_at20mA_W=(newsq-oldsq)*rho/t*.02**2))
report=dict(status='PASS_INDEPENDENT_SOURCE_PREFLIGHT_NATIVE_REQUIRED',source_before_sha256=hashlib.sha256(old.read_bytes()).hexdigest(),source_after_sha256=hashlib.sha256(new.read_bytes()).hexdigest(),removed_segments=len(removed),added_segments=len(added),changed_nets=['LED_BLE','LED_CAN','LED_PWR'],all_nonsegment_objects_exact=True,unchanged_segment_objects_exact=True,new_clearance_regressions=regress,lost_explicit_pad_groups=lost,trace_current_screen_A=.02,trace_copper_min_um=24,trace_resistivity_ohm_m=4e-8,trace_rows=rows,total_added_indicator_trace_heat_W=sum(r['added_heat_at20mA_W']for r in rows),native_required=True,thermal_and_filled_ground_model_required=True,physical_test_claimed=False)
(D/'SOURCE_CHANGE_CHECK.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
