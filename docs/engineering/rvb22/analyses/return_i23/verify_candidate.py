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
# Exact board and footprint track keepouts, previously omitted from the
# independent preflight. PCB-contained zone vertices are board coordinates.
from shapely.geometry import Polygon
keepout_hits=[];keepout_count=0
for parent in [b]+c.child(b,'footprint'):
 for zone in c.child(parent,'zone'):
  ko=c.child(zone,'keepout')
  if not ko or str((c.get(ko[0],'tracks')or [''])[0])!='not_allowed':continue
  layers=c.get(zone,'layers')or c.get(zone,'layer')or []
  for poly in c.child(zone,'polygon'):
   keepout_count+=1;g=Polygon([p[1:]for p in c.child(c.child(poly,'pts')[0],'xy')])
   for i in bi:
    if i['id']in added and i['layer']in layers and i['geometry'].intersects(g):keepout_hits.append(dict(id=i['id'],layer=i['layer'],overlap_mm2=i['geometry'].intersection(g).area))
assert not keepout_hits,keepout_hits
protected_names={'PROTECTED_12V','PROTECT_CTRL_VIN','INPUT_GATE','INPUT_GATE_DRIVE','INPUT_GATE_SLEW','BUCK_FEED','5V_VIN','5V_SW','5V_BST'}
raw_names={'RAW_12V','FUSED_12V','REV_BLOCKED_12V'}
names={x[1]:x[2]for x in c.child(b,'net')};specific_hits=[];specific_min={}
for label,nn,gap in [('protected',protected_names,.25),('raw',raw_names,.6)]:
 for layer in c.L:
  gg=unary_union([i['geometry']for i in bi if i['layer']==layer and names[i['net']]in nn])
  if gg.is_empty:continue
  for i in bi:
   if i['id']not in added or i['layer']!=layer:continue
   distance=i['geometry'].distance(gg);specific_min[label]=min(specific_min.get(label,1e99),distance)
   if distance<gap-2e-6:specific_hits.append(dict(id=i['id'],rule=label,actual_mm=distance,required_mm=gap))
assert not specific_hits,specific_hits
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
report=dict(status='PASS_INDEPENDENT_SOURCE_PREFLIGHT_NATIVE_REQUIRED',source_before_sha256=hashlib.sha256(old.read_bytes()).hexdigest(),source_after_sha256=hashlib.sha256(new.read_bytes()).hexdigest(),removed_segments=len(removed),added_segments=len(added),changed_nets=['LED_BLE','LED_CAN','LED_PWR'],all_nonsegment_objects_exact=True,unchanged_segment_objects_exact=True,new_clearance_regressions=regress,lost_explicit_pad_groups=lost,trace_current_screen_A=.02,trace_copper_min_um=24,trace_resistivity_ohm_m=4e-8,trace_rows=rows,total_added_indicator_trace_heat_W=sum(r['added_heat_at20mA_W']for r in rows),source_DRC_rules_sha256=hashlib.sha256(new.with_suffix('.kicad_dru').read_bytes()).hexdigest(),net_specific_minimum_clearances_mm=specific_min,net_specific_clearance_findings=specific_hits,track_keepout_polygons_checked=keepout_count,new_track_keepout_hits=keepout_hits,native_required=True,thermal_and_filled_ground_model_required=True,physical_test_claimed=False)
(D/'SOURCE_CHANGE_CHECK.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
