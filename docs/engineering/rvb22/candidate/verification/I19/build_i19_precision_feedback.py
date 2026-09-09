from pathlib import Path
import json,sys,shutil,hashlib,re
W=Path(__file__).resolve().parents[1];D=W/'iterations/I19_precision_feedback';D.mkdir(exist_ok=True)
src=W/'iterations/I18_capacitor_thermal_tab/candidate_kicad';dst=D/'candidate_kicad'
sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor'),str(W/'control')]
import check_combined_copper as c
import sexpdata as sx
P=src/'GR86_CCA_RevB.kicad_pcb';PCBsha=hashlib.sha256(P.read_bytes()).hexdigest()
freeze={'iteration':'I19','base_source_PCB_sha256':PCBsha,'frozen_before_edit':True,'redlines':[
 {'id':'I19-01','finding':'I18 full rail model leaves0.278mV above maximum falling reset threshold. Its zero-DC-credit release envelope exceeds3.6V, even though realizable modeled releases remain below3.6V.','correction':'Select same0603 land-compatible TNPU060311K8HWEA00 / TNPU06034K99HWEA00:11.8k/4.99k,0.02%,2ppm/K. Include0.1% independent drift and hot copper up to150C in the revised source interval and full rail sweep.'},
 {'id':'I19-02','finding':'Long C206 lead needs updated copper/RL source binding; component and copper temperature scopes differ.','correction':'Retain0..25nH branch sensitivity, expand branch R excludingR158 to0..0.12ohm, extract exact changed rail geometry at150C with15um minimum plated wall, and repeat separate numerical methods at controlling extrema.'},
 {'id':'I19-03','finding':'I18 finer-mesh thermal, return-reference and mechanical closure still need final effectivity.','correction':'Finish them against unchanged I18 copper/placement geometry, then prove I19 changes only these two resistor properties and schematic notes. Re-run native ERC/DRC/BOM/STEP exports.'}],'sources':['https://www.vishay.com/docs/28779/tnpue3.pdf']}
(D/'REDLINE_FREEZE.json').write_text(json.dumps(freeze,indent=2)+'\n')
def objects(t):
 depth=0;quote=False;esc=False;start=0
 for i,ch in enumerate(t):
  if quote:
   if esc:esc=False
   elif ch=='\\':esc=True
   elif ch=='"':quote=False
   continue
  if ch=='"':quote=True
  elif ch=='(':
   if depth==1:start=i
   depth+=1
  elif ch==')':
   depth-=1
   if depth==1:yield t[start:i+1]
parts={'R155':{'old_mpn':'TNPW060323K7BEEA','mpn':'TNPU060311K8HWEA00','old_value':'23.7k /0.1% 25ppm FB TOP','value':'11.8k /0.02% 2ppm FB TOP','ohms':11800},'R156':{'old_mpn':'TNPW060310K0BEEA','mpn':'TNPU06034K99HWEA00','old_value':'10k /0.1% 25ppm FB BOTTOM','value':'4.99k /0.02% 2ppm FB BOTTOM','ohms':4990}}
shutil.copytree(src,dst,dirs_exist_ok=True);files={}
for name,tag in [('GR86_CCA_RevB.kicad_pcb','footprint'),('Power_3V3.kicad_sch','symbol')]:
 s=(src/name).read_text();remove=[];add=[]
 for o in objects(s):
  if not o.startswith('('+tag+' '):continue
  node=sx.loads(o);ref=c.prop(node).get('Reference')
  if ref not in parts:continue
  p=parts[ref];q=o
  for a,z in [(p['old_mpn'],p['mpn']),(p['old_value'],p['value'])]:assert q.count(a)==1,(ref,a,q.count(a));q=q.replace(a,z)
  q=q.replace('https://www.vishay.com/docs/28758/tnpw_e3.pdf','https://www.vishay.com/docs/28779/tnpue3.pdf')
  remove.append(o);add.append(q)
 assert len(remove)==2
 for a,z in zip(remove,add):assert s.count(a)==1;s=s.replace(a,z)
 (dst/name).write_text(s);files[name]={'before_sha256':hashlib.sha256((src/name).read_bytes()).hexdigest(),'after_sha256':hashlib.sha256(s.encode()).hexdigest(),'remove':remove,'add':add}
# Exact geometric/electrical node equivalence outside resistor property values.
bo,io,u=c.collect(P);bn,inn,un=c.collect(dst/P.name);assert not u and not un
assert [(x['id'],x['net'],x['layer'],x['geometry'].wkb)for x in io]==[(x['id'],x['net'],x['layer'],x['geometry'].wkb)for x in inn]
for key in ['segment','via','zone','gr_line','gr_arc','gr_poly']:
 assert c.child(bo,key)==c.child(bn,key),key
for f in c.child(bo,'footprint'):
 q=next(g for g in c.child(bn,'footprint')if c.prop(g)['Reference']==c.prop(f)['Reference'])
 assert c.child(f,'pad')==c.child(q,'pad') and c.get(f,'at')==c.get(q,'at')and c.get(f,'layer')==c.get(q,'layer')
t=.0002+2e-6*100+.001;rt=11800;rb=4990;rhi=rt*(1+t);rlo=rt*(1-t);bhi=rb*(1+t);blo=rb*(1-t)
lo=.985*(1+rlo/bhi)-1e-7*rhi;hi=1.015*(1+rhi/blo)+1e-7*rhi
bounds={'nominal_V':1+rt/rb,'source_min_V':lo,'source_max_V':hi,'resistor_tolerance_fraction':.0002,'resistor_TCR_per_K':2e-6,'maximum_deltaT_K':100,'independent_drift_fraction_each':.001,'reference_interval_V':[.985,1.015],'FB_current_abs_A':1e-7,'resistor_film_max_C':125,'component_internal_thermal_resistance_K_W':63,'R155_power_at_maxrail_W':(hi-1.015)**2/(rt*(1-t)),'R156_power_at_maxreference_W':1.015**2/(rb*(1-t)),'source':'https://www.vishay.com/docs/28779/tnpue3.pdf','qualification_scope':'Part-code construction matches manufacturer size, resistance, H tolerance, W TCR and EA packaging with standard00 suffix. Exact availability/supplier procurement remains a supplier acceptance condition.0.1% drift is a stated allocation corresponding to the8000h rated-power stability row, not a mission-life guarantee or arbitrary combined-stress proof.'}
(D/'SETPOINT_BOUNDS.json').write_text(json.dumps(bounds,indent=2)+'\n');(D/'TRANSPORT.json').write_text(json.dumps(files,indent=2)+'\n')
proof={'iteration':'I19','PCB_before_sha256':PCBsha,'PCB_after_sha256':files[P.name]['after_sha256'],'schematic_after_sha256':files['Power_3V3.kicad_sch']['after_sha256'],'all_explicit_copper_shapes_nets_layers_equal':True,'all_zones_edges_arcs_silkscreen_equal':True,'all_pad_and_component_positions_equal':True,'property_changes':parts,'setpoint':bounds,'native_required':True}
(D/'SOURCE_CHANGE_CHECK.json').write_text(json.dumps(proof,indent=2)+'\n');print(json.dumps(proof,indent=2),flush=True)
