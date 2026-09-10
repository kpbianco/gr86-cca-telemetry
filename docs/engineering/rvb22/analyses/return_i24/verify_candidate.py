"""Independent preservation, explicit-copper and projected-reference acceptance."""
from pathlib import Path
import sys,json,hashlib,math
W=Path(__file__).resolve().parents[2];D=Path(__file__).resolve().parent
sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor'),str(W/'control')]
import check_combined_copper as c
from shapely.geometry import LineString,Point,Polygon
from shapely.ops import unary_union as U
old=W/'iterations/I23_return_clearance/candidate_kicad/GR86_CCA_RevB.kicad_pcb';new=W/'iterations/I24_return_clearance/candidate_kicad/GR86_CCA_RevB.kicad_pcb'
a,ai,u=c.collect(old);b,bi,v=c.collect(new);assert not u and not v
j=json.loads((D/'CANDIDATE_MANIFEST.json').read_text());removed=set(j['removed_uuids']);aa={c.get(s,'uuid')[0]:s for s in c.child(a,'segment')};bb={c.get(s,'uuid')[0]:s for s in c.child(b,'segment')};added=set(bb)-set(aa)
assert set(aa)-set(bb)==removed and all(aa[k]==bb[k]for k in set(aa)&set(bb))
nonseg=lambda t:[x for x in t if not(isinstance(x,list)and x and str(x[0])=='segment')]
assert nonseg(a)==nonseg(b)
names={n[1]:n[2]for n in c.child(b,'net')}
assert {names[c.get(aa[k],'net')[0]]for k in removed}=={'OIL_5V','OIL_SIG'}
assert {names[c.get(bb[k],'net')[0]]for k in added}=={'OIL_5V','OIL_SIG'}
ov,oc=c.analyze(ai);nv,nc=c.analyze(bi);key=lambda r:(r['layer'],*sorted([r['a'],r['b']]))
ov={key(r):r for r in ov};regress=[r for r in nv if key(r)not in ov or r['gap_mm']<ov[key(r)]['gap_mm']-2e-6];lost=[dict(net=n,old_group=g)for n,gs in oc.items()for g in gs if len(g)>1 and not any(set(g)<=set(q)for q in nc[n])]
assert not regress and not lost,(regress,lost)
keepout_count=0;hits=[]
for parent in [b]+c.child(b,'footprint'):
 for z in c.child(parent,'zone'):
  ko=c.child(z,'keepout')
  if not ko or str((c.get(ko[0],'tracks')or [''])[0])!='not_allowed':continue
  ls=c.get(z,'layers')or c.get(z,'layer')or []
  for poly in c.child(z,'polygon'):
   keepout_count+=1;g=Polygon([p[1:]for p in c.child(c.child(poly,'pts')[0],'xy')])
   hits.extend(i['id']for i in bi if i['id']in added and i['layer']in ls and i['geometry'].intersects(g))
assert not hits,hits
protected={'PROTECTED_12V','PROTECT_CTRL_VIN','INPUT_GATE','INPUT_GATE_DRIVE','INPUT_GATE_SLEW','BUCK_FEED','5V_VIN','5V_SW','5V_BST'};raw={'RAW_12V','FUSED_12V','REV_BLOCKED_12V'}
mins={};vh=[]
for label,nn,gap in [('protected',protected,.25),('raw',raw,.6)]:
 for l in c.L:
  gg=U([i['geometry']for i in bi if i['layer']==l and names[i['net']]in nn])
  if gg.is_empty:continue
  for i in bi:
   if i['id']not in added or i['layer']!=l:continue
   distance=i['geometry'].distance(gg);mins[label]=min(mins.get(label,1e99),distance)
   if distance<gap-2e-6:vh.append([i['id'],label,distance,gap])
assert not vh,vh
critical={'ADC_NODE','CANH','CANL','CAN_TX_MCU','CAN_RX_MCU','GPS_ANT_RF_BIASED','GPS_EXT_ANT','GPS_TX_RAW','GPS_TX_MCU','GPS_TX_BUFFER','GPS_RX_MODULE','GPS_PPS_RAW','GPS_PPS_BUFFER','GPS_PPS_MCU','OIL_EXCITATION_ADC','UART0_RX','UART0_TX'}
projected=[]
for l in ['B.Cu','In1.Cu']:
 for s in c.child(b,'segment'):
  if c.get(s,'layer')!=[l]or names[c.get(s,'net')[0]]not in critical:continue
  width=c.get(s,'width')[0];name=names[c.get(s,'net')[0]];g=LineString([c.get(s,'start'),c.get(s,'end')]).buffer(width/2+(.95 if name in ['GPS_EXT_ANT','GPS_ANT_RF_BIASED']else .05))
  for i in bi:
   if i['id']in added and i['layer']=='In2.Cu':
    dd=g.distance(i['geometry'])
    if dd<.155-2e-6:projected.append([name,c.get(s,'uuid')[0],i['id'],dd])
assert not projected,projected
outline=Polygon(json.loads((W/'iterations/I18_capacitor_thermal_tab/CURRENT_OUTLINE_GEOMETRY.json').read_text())['outline_mm']);holes=U([c.pad_shape(f,p)for f in c.child(b,'footprint')for p in c.child(f,'pad')if str(p[2])=='np_thru_hole'])
edge=min(outline.boundary.distance(i['geometry'])for i in bi if i['id']in added);hole=min(holes.distance(i['geometry'])for i in bi if i['id']in added);assert edge>=.25 and hole>=.25
report={'status':'PASS_INDEPENDENT_SOURCE_PREFLIGHT_NATIVE_REQUIRED','source_before_sha256':hashlib.sha256(old.read_bytes()).hexdigest(),'source_after_sha256':hashlib.sha256(new.read_bytes()).hexdigest(),'changed_nets':['OIL_5V','OIL_SIG'],'removed_segments':len(removed),'added_segments':len(added),'all_nonsegment_objects_exact':True,'unchanged_segment_objects_exact':True,'new_clearance_regressions':regress,'lost_explicit_pad_groups':lost,'all_track_keepout_polygons_checked':keepout_count,'new_track_keepout_hits':hits,'voltage_specific_minimum_clearance_mm':mins,'voltage_specific_findings':vh,'projected_adjacent_reference_regressions':projected,'minimum_copper_board_edge_mm':edge,'minimum_copper_NPTH_mm':hole,'native_required':True,'affected_PDN_ground_thermal_required':True,'physical_tests_claimed':0}
(D/'SOURCE_CHANGE_CHECK.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))
