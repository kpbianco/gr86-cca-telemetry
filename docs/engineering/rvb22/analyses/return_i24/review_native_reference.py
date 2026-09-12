"""Compare exact I23/I24 filled reference results, retaining subthreshold changes."""
from pathlib import Path
import sys,json,hashlib,collections
W=Path(__file__).resolve().parents[2];D=Path(__file__).resolve().parent
sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor'),str(W/'control')]
import check_combined_copper as c
import shapely as sh
from shapely.geometry import Polygon,LineString
from shapely.ops import unary_union as U
critical={'ADC_NODE','CANH','CANL','CAN_TX_MCU','CAN_RX_MCU','GPS_ANT_RF_BIASED','GPS_EXT_ANT','GPS_TX_RAW','GPS_TX_MCU','GPS_TX_BUFFER','GPS_RX_MODULE','GPS_PPS_RAW','GPS_PPS_BUFFER','GPS_PPS_MCU','OIL_EXCITATION_ADC','UART0_RX','UART0_TX'}
old=W/'runtime/hosted/run26_retry2/extracted/native_I06_hosted/candidate_kicad/GR86_CCA_RevB.kicad_pcb'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def old_reference():
 b,items,unsupported=c.collect(old);assert not unsupported
 names={n[1]:n[2]for n in c.child(b,'net')};gg={l:[i['geometry']for i in items if i['net']==30 and i['layer']==l]for l in c.L}
 for z in c.child(b,'zone'):
  if c.child(z,'keepout')or c.get(z,'net')!=[30]:continue
  for fp in c.child(z,'filled_polygon'):
   gg[c.get(fp,'layer')[0]].append(sh.make_valid(Polygon([p[1:]for p in c.child(c.child(fp,'pts')[0],'xy')])))
 gg={l:U(g).buffer(.000002)for l,g in gg.items()};own={};rows=[]
 for s in c.child(b,'segment'):
  net=c.get(s,'net')[0];name=names[net];layer=c.get(s,'layer')[0]
  if name not in critical or layer not in ['F.Cu','B.Cu']:continue
  ref='In1.Cu'if layer=='F.Cu'else'In2.Cu';other='In2.Cu'if layer=='F.Cu'else'In1.Cu';key=net,ref
  if key not in own:own[key]=U([i['geometry'].buffer(.155)for i in items if i['net']==net and i['layer']==ref and i['type']in ['via','pad']])
  line=LineString([c.get(s,'start'),c.get(s,'end')]);missing=line.difference(gg[ref]).difference(own[key]);both=missing.difference(gg[other])
  rows.append({'uuid':c.get(s,'uuid')[0],'net':name,'layer':layer,'reference':ref,'start':c.get(s,'start'),'end':c.get(s,'end'),'length_mm':line.length,'adjacent_gap_after_own_antipads_mm':missing.length,'both_ground_planes_gap_mm':both.length})
 return {'filled_PCB_sha256':sha(old),'all_checked_segments':len(rows),'all_checked_segment_results':rows,'own_antipad_allowance_mm':.155,'geometry_quantization_tolerance_mm':.000002,'not_a_return_transfer_or_neck_down_proof':True}

def summarize(rows):
 findings=[r for r in rows if r['adjacent_gap_after_own_antipads_mm']>.005]
 return {'critical_segments_checked':len(rows),'reportable_segment_findings':len(findings),'reportable_adjacent_gap_length_mm':sum(r['adjacent_gap_after_own_antipads_mm']for r in findings),'by_net':dict(collections.Counter(r['net']for r in findings)),'all_adjacent_gap_length_mm_including_subthreshold':sum(r['adjacent_gap_after_own_antipads_mm']for r in rows),'simultaneous_inner_plane_gap_length_mm':sum(r['both_ground_planes_gap_mm']for r in rows),'simultaneous_inner_plane_gap_segment_count':sum(r['both_ground_planes_gap_mm']>1e-7 for r in rows)}

if __name__=='__main__':
 cache=D/'I23_ALL_CRITICAL_REFERENCE.json'
 if not cache.exists()or json.loads(cache.read_text())['filled_PCB_sha256']!=sha(old):cache.write_text(json.dumps(old_reference(),indent=2)+'\n')
 before=json.loads(cache.read_text());assert before['filled_PCB_sha256']=='3703e280fdbfab602721db5c41fad83669c1bc936725ecf508a584db61373fce'
 mp=W/'analyses/manufacturing_i24/RESULTS.json';result=json.loads(mp.read_text());after=result['reference']
 assert result['source_PCB_sha256']=='5b373f6033fdbd18f126f8ca054b619abada2568778c5a8fc303c4e756fb06c8'
 assert result['filled_PCB_sha256']=='11533ea91c3bc4c61dd7066e0001914a72bc90b27afb38d321c43b8feb90e3b1'
 assert result['independent_manufacturing_status']=='PASS_INTENDED_COPPER_AND_EXPORTS'
 a={r['uuid']:r for r in before['all_checked_segment_results']};b={r['uuid']:r for r in after['all_checked_segment_results']};assert set(a)==set(b)
 regress=[];improved=[]
 for uid,now in b.items():
  prior=a[uid];assert all(now[k]==prior[k]for k in ['net','layer','reference','start','end','length_mm'])
  for k in ['adjacent_gap_after_own_antipads_mm','both_ground_planes_gap_mm']:
   delta=now[k]-prior[k]
   if delta>2e-6:regress.append({'uuid':uid,'net':now['net'],'metric':k,'before_mm':prior[k],'after_mm':now[k],'delta_mm':delta})
   if delta< -2e-6:improved.append({'uuid':uid,'net':now['net'],'metric':k,'before_mm':prior[k],'after_mm':now[k],'delta_mm':delta})
 assert not regress,regress
 oldrx=[r for r in a.values()if r['net']=='CAN_RX_MCU'and r['adjacent_gap_after_own_antipads_mm']>.005]
 assert len(oldrx)==9 and all(b[r['uuid']]['adjacent_gap_after_own_antipads_mm']<=1e-7 for r in oldrx)
 remain=[r for r in b.values()if r['adjacent_gap_after_own_antipads_mm']>.005];assert len(remain)==6 and {r['net']for r in remain}=={'CAN_TX_MCU'}
 assert all(r['both_ground_planes_gap_mm']<=1e-7 for r in b.values())
 rf=[r for r in b.values()if r['net']in ['GPS_EXT_ANT','GPS_ANT_RF_BIASED']];assert len(rf)==12 and all(r['adjacent_gap_after_own_antipads_mm']<=1e-7 for r in rf)
 report={'status':'PASS_EXACT_NATIVE_REFERENCE_REGRESSION_REVIEW_SIX_CAN_TX_GAPS_OPEN','source_PCB_sha256':result['source_PCB_sha256'],'filled_PCB_sha256':result['filled_PCB_sha256'],'before':summarize(list(a.values())),'after':summarize(list(b.values())),'critical_net_scope':sorted(critical),'source_critical_segment_identity_and_geometry_unchanged':True,'full_per_UUID_comparison_includes_subthreshold_gaps':True,'regressions':regress,'improved_metrics':improved,'all_nine_prior_CAN_RX_gaps_removed':True,'all_simultaneous_inner_plane_gaps_removed':True,'RF_segments_checked':12,'RF_adjacent_reference_gaps':0,'remaining_segments':remain,'native_postconditions_passed':result['native_postconditions_passed'],'native_ERC_DRC_unconnected_parity_findings':[result[k]for k in ['native_ERC','native_DRC','native_unconnected','native_parity']],'manufacturing_report_path':str(mp.relative_to(W)),'manufacturing_report_sha256':sha(mp),'before_reference_cache_sha256':sha(cache),'scope_limits':['Critical outer signal centreline reference, explicit own-pad/via antipad allowance, and geometric same-net connectivity are checked; no local return-current transfer or EM field/neck-down proof is claimed.','GND-02 remains open for the six CAN_TX launch interruptions and its complete layer-by-layer/neck-down requirement; additional analog/private-return/power-loop applicability remains separate.','SI-07 receiver scope waveforms and CAN-03 failure tests remain unexecuted physical requirements.','All 133-net intended connectivity and export agreement are independently supplied by the peer manufacturing report.'],'physical_tests_claimed':0,'manufacturing_release':False}
 (D/'NATIVE_REFERENCE_REVIEW.json').write_text(json.dumps(report,indent=2)+'\n')
 dispositions={'iteration':'I24','source_PCB_sha256':report['source_PCB_sha256'],'filled_PCB_sha256':report['filled_PCB_sha256'],'evidence':'analyses/return_i24/NATIVE_REFERENCE_REVIEW.json','evidence_sha256':sha(D/'NATIVE_REFERENCE_REVIEW.json'),'redlines':[{'id':'I24-RET-01','status':'RESOLVED_SCOPED_ROUTING_FINDING','disposition':'All nine prior CAN_RX adjacent-In1 gaps are zero in actual final native fill. The OIL_5V chain is shorter; raw OIL_SIG length is unchanged. No critical reference regression, native finding, intended copper discontinuity or export mismatch introduced.','broader_criterion':'GND-02 remains open.'},{'id':'I24-RET-02','status':'OPEN','disposition':'Six CAN_TX launch segments still lack nearest In1 reference over1.3417456364mm. Inner-rail and new signal-via alternatives were evaluated but not adopted. Alternate In2 ground is present throughout after the LED correction; presence alone is not proof of return-current transfer or compliance with continuous-nearest-reference wording.','next_action':'Develop and verify a coordinated bulk-rail/MCU launch correction or formally review a quantified local return-transfer treatment while retaining original criterion scope.'},{'id':'I24-RET-03','status':'RESOLVED_SCOPED_SIMULTANEOUS_GAP','disposition':'The previously0.0579770992mm simultaneous In1/In2 reference gap is zero in actual final native fill. A local LED_CAN In2 detour provides alternate ground beneath all remaining CAN_TX nearest-plane interruptions without changing vias or RF geometry.','broader_criterion':'The six In1 interruptions and GND-02 remain open.'}],'all_original_criteria_automatically_closed':False,'physical_tests_claimed':0,'preserved_failed_trials':['ROUTE_TRIALS.json','INNER_TRIALS.json','TX_TRANSITION_VIA_FEASIBILITY.json','TX_ALTERNATE_REFERENCE_TRIAL.json'],'preserved_native_artifacts':['runtime/hosted/run26_retry2','runtime/hosted/run27','runtime/hosted/run28']}
 (D/'REDLINE_DISPOSITIONS.json').write_text(json.dumps(dispositions,indent=2)+'\n');print(json.dumps({k:report[k]for k in ['status','before','after','all_nine_prior_CAN_RX_gaps_removed','all_simultaneous_inner_plane_gaps_removed','RF_adjacent_reference_gaps']},indent=2))
