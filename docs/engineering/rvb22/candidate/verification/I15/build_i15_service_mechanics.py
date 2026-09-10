from pathlib import Path
import json,re,hashlib,uuid,shutil,sys
W=Path(__file__).resolve().parents[1];D=W/'iterations/I15_service_mechanics';D.mkdir(exist_ok=True)
sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor'),str(W/'control')]
import check_combined_copper as c
from shapely.geometry import box,Point
import cadquery as cq
src=W/'iterations/I14_service_clearance/candidate_kicad';dst=D/'candidate_kicad';s=(src/'GR86_CCA_RevB.kicad_pcb').read_text()
freeze={'iteration':'I15','base_source_sha256':hashlib.sha256(s.encode()).hexdigest(),'known_redlines':[
 {'id':'I15-01','finding':'I14 native DRC has one unconnected GND pour island after close J201.3 via removal; all other native checks including0.508mm rule pass.','correction':'Restore a GND stitching via at71/7.5mm outside the contact area; move three ESP_EN back traces clear of it.'},
 {'id':'I15-02','finding':'Mated service envelope intersects C201 broad courtyard and J401 broad courtyard.','correction':'Use primary body dimensions and bounded tool positioning instead of treating each courtyard as a solid; retain explicit dimensional acceptance.'},
 {'id':'I15-03','finding':'The full150mm coax route intersects the upper-right carrier support.','correction':'C05 support relief,15mm-radius S bend and complete rerouted150mm loop; recalculate reduced ligament and confirm3D envelopes.'},
 {'id':'I15-04','finding':'Thermal/package margins, exact adapter identity and formal register remain unresolved.','correction':'Carry all forward explicitly; no implication that native zero closes these.'}]}
(D/'REDLINE_FREEZE.json').write_text(json.dumps(freeze,indent=2)+'\n')
shutil.copytree(src,dst,dirs_exist_ok=True)
def children(text):
 depth=0;quoted=False;escape=False;start=0
 for i,ch in enumerate(text):
  if quoted:
   if escape:escape=False
   elif ch=='\\':escape=True
   elif ch=='"':quoted=False
  elif ch=='"':quoted=True
  elif ch=='(':
   if depth==1:start=i
   depth+=1
  elif ch==')':
   depth-=1
   if depth==1:yield start,i+1,text[start:i+1]
 assert not depth and not quoted
remove={'5310347d-239f-5f4d-87dc-98ba5c70e3b1','867107fb-d94b-5c75-9188-b4804768d2c3','b977b17d-0144-5b8a-a54a-901ec3afe7bb'}
edits=[]
for a,b,t in children(s):
 if t.startswith('(segment ') and any(k in t for k in remove):edits.append((a,b,''))
assert len(edits)==3
new=[]
for a,b in zip([(69.5,6.3),(69.5,7.2),(70.6,8.3),(72.2,8.3)],[(69.5,7.2),(70.6,8.3),(72.2,8.3),(73.6,9.7)]):
 ident=str(uuid.uuid5(uuid.NAMESPACE_URL,'I15:ESP_EN:'+str(a)+str(b)))
 new.append('(segment (start %s %s) (end %s %s) (width 0.2) (layer "B.Cu") (net 27) (uuid "%s"))'%(*a,*b,ident))
ident=str(uuid.uuid5(uuid.NAMESPACE_URL,'I15:J201_GND:71:7.5'))
new.append('(via (at 71 7.5) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net 30) (uuid "%s"))'%ident)
for a,b,t in sorted(edits,reverse=True):s=s[:a]+t+s[b:]
s=s.rstrip()[:-1]+'\n'+'\n'.join(new)+'\n)\n';p=dst/'GR86_CCA_RevB.kicad_pcb';p.write_text(s)
_,items,u=c.collect(p);assert not u;changed={re.search(r'\(uuid "([^"]+)"',t).group(1) for t in new};hits=[]
for a in items:
 if a['id'] not in changed:continue
 for b in items:
  if a['layer']!=b['layer'] or a['net']==b['net']:continue
  gap=a['geometry'].distance(b['geometry'])
  if gap<.15-1e-7:hits.append({'a':a['id'],'b':b['id'],'layer':a['layer'],'gap_mm':gap})
proof={'source_before_sha256':freeze['base_source_sha256'],'source_after_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'removed':sorted(remove),'added_objects':new,'explicit_copper_preflight':hits,'native_required':True,'footprints_pads_schematics_RF_and_logo_unchanged':True}
(D/'SOURCE_CHANGE_CHECK.json').write_text(json.dumps(proof,indent=2)+'\n');assert not hits,hits
msrc=W/'iterations/I12_branding_models_thermal/candidate_mechanics';m=D/'candidate_mechanics';shutil.copytree(msrc,m,dirs_exist_ok=True)
g=json.loads((W/'lanes/mechanics/THERMAL_WING_GEOMETRY.json').read_text());posts=[];plans=[]
for x,y in g['mount_centres_mm']:
 right=65 if (x,y)==(63,-4) else x+3.5;shape=box(x-3.5,y-3.5,right,y+3.5).difference(Point(x,y).buffer(1.1,quad_segs=64));plans.append({'center':[x,y],'WKT':shape.wkt})
 wp=cq.Workplane('XY',origin=(0,0,-6)).polyline(list(shape.exterior.coords)[:-1]).close()
 for ring in shape.interiors:wp=wp.polyline(list(ring.coords)[:-1]).close()
 posts.append(wp.extrude(6).val())
cq.exporters.export(cq.Compound.makeCompound(posts),str(m/'C05_SUPPORTS.step'))
scad=(m/'GR86_RVB_CARRIER_C04.scad').read_text();a=scad.index('module supports()');scad=scad[:a]+'''module supports(){for(p='''+json.dumps(g['mount_centres_mm'])+''')translate([p[0],p[1],-6])difference(){translate([-3.5,-3.5,0])cube([p[0]==63 && p[1]==-4 ?5.5:7,7,6]);translate([0,0,-1])cylinder(d=2.2,h=8);}}
color("ivory"){c04_base();supports();}
''';scad=scad.replace('// C04 PCB','// C05 PCB').replace('existing6mm support posts','upper-right support relieved toX65mm;6mm posts')
(m/'GR86_RVB_CARRIER_C05.scad').write_text(scad)
force=29.740623037861003;wall=65-.15-(63+.15)-(2.2+.15)/2;height=6-.15
stress=3*force/(2*wall*height)
spec={'status':'CONDITIONAL_C05_SUPPORT_SELECTED','base_and_thermal_contact_geometry_unchanged':True,'support_plans':plans,'changed_post_center_mm':[63,-4],'post_right_edge_mm':65,'edge_position_tolerance_mm':.15,'hole_center_tolerance_mm':.15,'hole_diameter_mm':[2.05,2.35],'minimum_ligament_mm':wall,'minimum_height_mm':height,'one_ear_load_N':force,'Kt3_shearout_MPa':stress,'required_hot_notched_shear_allowable_MPa':17.544,'pass_allocated_static_bound':stress<=17.544,'requirements':['M2 fastener passes through the spacer into a retained nut or independent structure; no holding strength from printed spacer threads is credited.','Retain C04 base, all four board-mount centers and W02/T03 contacts. Only one support outer edge changes.','Local notch/creep/fatigue and material hot allowables remain supplier/installation conditions; the geometric and static calculations do not prove endurance.']}
(m/'C05_SUPPORT_RELIEF.json').write_text(json.dumps(spec,indent=2)+'\n');print(json.dumps({'source_sha':proof['source_after_sha256'],'preflight':hits,'post_ligament_mm':wall,'post_Kt3_shear_MPa':stress}))
