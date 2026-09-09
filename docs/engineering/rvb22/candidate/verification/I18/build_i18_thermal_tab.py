from pathlib import Path
import json,sys,shutil,hashlib,re,uuid,math,heapq,copy
W=Path(__file__).resolve().parents[1];D=W/'iterations/I18_capacitor_thermal_tab';D.mkdir(exist_ok=True)
sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor'),str(W/'control')]
import check_combined_copper as c
import sexpdata as sx
from shapely.geometry import Point,LineString,Polygon,box
from shapely.ops import unary_union
from shapely import contains_xy,prepare
import numpy as np
src=W/'iterations/I17_RF_hole_clearance/candidate_kicad';P=src/'GR86_CCA_RevB.kicad_pcb';s=P.read_text();b,items,u=c.collect(P);assert not u
freeze={'iteration':'I18','base_source_sha256':hashlib.sha256(s.encode()).hexdigest(),'known_redlines':[
 {'id':'I18-01','finding':'C206 on B.Cu next to the MCU has only0.43C modeled margin to125C at65C air/70C cold landings/full4.815W on0.25mm mesh, without convergence.','correction':'Relocate C206 to F.Cu on added top-right FR4 tab, using same qualified part and footprint. Keep original power load, cooling boundaries and all existing TP/service locations.'},
 {'id':'I18-02','finding':'Relocation affects outline, current path inductance/resistance, thermal field, fabrication and assembly.','correction':'Recompute all affected source-bound analyses, native DRC/ERC and exports, drill/gerber continuity; verify tab mechanics, clearance and revision identity. Native pass is required before any desktop closure.'}],'frozen_before_edit':True}
(D/'REDLINE_FREEZE.json').write_text(json.dumps(freeze,indent=2)+'\n')
# String-preserving top-level editor.
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
obj=list(objects(s));capold=next(x for x in obj if x.startswith('(footprint ') and '(property "Reference" "C206"' in re.sub(r'\s+',' ',x))
cap=next(f for f in c.child(b,'footprint')if c.prop(f)['Reference']=='C206')
lib=sx.loads((src/'libraries/RevB.pretty/KEMET_T598_X_7343_Median_RVB22_C206.kicad_mod').read_text())
# Retain board identifiers/electrical metadata; use front-side library geometry exactly.
capnew=copy.deepcopy(cap);c.child(capnew,'layer')[0][1]='F.Cu';c.child(capnew,'at')[0][1:]=[82.0,-4.0,0]
for p in c.child(capnew,'property'):
 at=c.child(p,'at')[0];at[1:]=[0,1.6,180];c.child(p,'layer')[0][1]='F.Fab'
 for e in c.child(p,'effects'):
  e[:]=[x for x in e if c.tag(x)!='justify']
for tag in ['fp_rect','fp_line','fp_text']:
 oldnodes=c.child(capnew,tag);newnodes=c.child(lib,tag)
 assert len(oldnodes)==len(newnodes),(tag,len(oldnodes),len(newnodes))
 for old,new in zip(oldnodes,newnodes):
  uid=c.child(old,'uuid');new=copy.deepcopy(new);new[:]=[x for x in new if c.tag(x)!='uuid'];new+=uid;capnew[capnew.index(old)]=new
for p in c.child(capnew,'pad'):
 q=next(q for q in c.child(lib,'pad')if q[1]==p[1]);oldmeta=c.child(p,'net')+c.child(p,'uuid');q=copy.deepcopy(q);q[:]=[x for x in q if c.tag(x)not in ['net','uuid']];q+=oldmeta;capnew[capnew.index(p)]=q
newcaptext=sx.dumps(capnew)
# Chamfered8mm tab stays inside previous total board bbox.
edgeold=next(x for x in obj if x.startswith('(gr_line ')and c.get(sx.loads(x),'layer')==['Edge.Cuts']and c.get(sx.loads(x),'start')==[66.5,0])
outlinepts=[[66.5,0],[76.5,0],[77,-.5],[77,-7.5],[77.5,-8],[86.4,-8],[86.9,-7.5],[86.9,-.5],[87.273825,0]]
uid=lambda x:str(uuid.uuid5(uuid.NAMESPACE_URL,'RVB22:I18:'+x))
newedges=['(gr_line (start %s %s) (end %s %s) (stroke (width 0.05) (type default)) (layer "Edge.Cuts") (uuid "%s"))'%(*a,*z,uid('edge'+str(i)))for i,(a,z)in enumerate(zip(outlinepts,outlinepts[1:]))]
geom=json.loads((W/'lanes/mechanics/THERMAL_WING_GEOMETRY.json').read_text());oldoutline=Polygon(geom['outline_mm']);tab=Polygon(outlinepts[1:]+[[76.5,0]])
board=oldoutline.union(tab);assert board.geom_type=='Polygon' and board.is_valid
# Explicit foreign copper / NPTH / board edge obstacles. J201 gets manufacturer.508mm foreign copper.
foreign=[x for x in items if x['layer']=='In1.Cu' and x['net']!=133 and not x['id'].startswith('C206.')]
# Front-side new capacitor pads are not obstacles on B.Cu.
obs=unary_union([x['geometry']for x in foreign]);holes=unary_union([c.pad_shape(f,p)for f in c.child(b,'footprint')for p in c.child(f,'pad')if str(p[2])=='np_thru_hole'])
j201=unary_union([x['geometry']for x in items if x['layer']=='F.Cu'and x['id'].startswith('J201.')])
start=(79.48,18.4);end=(78.88,-6.8);w=.8
ob=unary_union([obs.buffer(w/2+.15002),holes.buffer(w/2+.25002),j201.buffer(w/2+.50802)])
prepare(ob)
print('Routing0.8mm branch with exact obstacle screen',flush=True)
xs=np.round(np.arange(76,86.90001,.02),5);ys=np.round(np.arange(-7,19.00001,.02),5);X,Y=np.meshgrid(xs,ys)
blocked=contains_xy(ob,X,Y)|~contains_xy(board.buffer(-w/2-.25002),X,Y)
def idx(p):return(round((p[0]-xs[0])/.02),round((p[1]-ys[0])/.02))
def pt(q):return(float(xs[q[0]]),float(ys[q[1]]))
a=idx(start);z=idx(end);assert not blocked[a[1],a[0]] and not blocked[z[1],z[0]],'Endpoint blocked'
pq=[(0,a)];cost={a:0};prev={};found=None
while pq:
 _,q=heapq.heappop(pq)
 if q==z:found=q;break
 for dx,dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,-1),(1,-1),(-1,1)]:
  nq=q[0]+dx,q[1]+dy
  if not(0<=nq[0]<len(xs)and 0<=nq[1]<len(ys))or blocked[nq[1],nq[0]]:continue
  if ob.intersects(LineString([pt(q),pt(nq)])):continue
  nc=cost[q]+.02*math.hypot(dx,dy)
  if nc>=cost.get(nq,1e99):continue
  cost[nq]=nc;prev[nq]=q;heapq.heappush(pq,(nc+math.dist(pt(nq),end),nq))
assert found,'No0.8mm inner route';seq=[found]
while seq[-1]!=a:seq.append(prev[seq[-1]])
seq.reverse();pairs=[]
for qa,qb in zip(seq,seq[1:]):
 pa,pb=pt(qa),pt(qb)
 if pairs:
  aa,bb=pairs[-1]
  if abs((bb[0]-aa[0])*(pb[1]-bb[1])-(bb[1]-aa[1])*(pb[0]-bb[0]))<1e-8:pairs[-1]=(aa,pb);continue
 pairs.append((pa,pb))
# Every path shortcut must independently meet the same constraints. Greedy line-of-sight produces fewer arcs in route.
ps=[pairs[0][0]]+[x[1]for x in pairs];simple=[ps[0]];i=0
while i<len(ps)-1:
 j=len(ps)-1
 while j>i+1 and (ob.intersects(LineString([ps[i],ps[j]]))or not board.buffer(-w/2-.25002).covers(LineString([ps[i],ps[j]]))):j-=1
 simple.append(ps[j]);i=j
pairs=list(zip(simple,simple[1:]));wide=[]
width_ob=unary_union([obs,Point(85.12,-6.8).buffer(.3,quad_segs=64),Point(77.7,-5.8).buffer(.3,quad_segs=64)])
for a,z in pairs:
 count=max(1,math.ceil(math.dist(a,z)/.5))
 for k in range(count):
  aa=tuple(round(a[j]+(z[j]-a[j])*k/count,6)for j in [0,1]);zz=tuple(round(a[j]+(z[j]-a[j])*(k+1)/count,6)for j in [0,1]);line=LineString([aa,zz])
  ww=max(v for v in [.8,1.,1.5,2.] if line.distance(width_ob)>=v/2+.150005 and board.buffer(-v/2-.250005).covers(line))
  if wide and wide[-1][2]==ww and abs((wide[-1][1][0]-wide[-1][0][0])*(zz[1]-aa[1])-(wide[-1][1][1]-wide[-1][0][1])*(zz[0]-aa[0]))<1e-5:wide[-1]=(wide[-1][0],zz,ww)
  else:wide.append((aa,zz,ww))
newtracks=['(segment (start %s %s) (end %s %s) (width %s) (layer "In1.Cu") (net 133) (uuid "%s"))'%(*a,*z,ww,uid('power'+str(i)))for i,(a,z,ww)in enumerate(wide)]

newvias=[]
for i,(p,n)in enumerate([([79.48,18.4],133),([78.88,-6.8],133),([85.12,-6.8],30),([77.7,-5.8],30)]):
 newvias.append('(via (at %s %s) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net %s) (uuid "%s"))'%(*p,n,uid('via'+str(i))))
newtracks.append('(segment (start 78.88 -6.8) (end 78.88 -4) (width 0.8) (layer "F.Cu") (net 133) (uuid "%s"))'%uid('cap_positive'))
newtracks.append('(segment (start 85.12 -4) (end 85.12 -6.8) (width 0.8) (layer "F.Cu") (net 30) (uuid "%s"))'%uid('gnd'))
zonepts=[[76.5,.75],[76.5,0],[77,-.5],[77,-7.5],[77.5,-8],[86.4,-8],[86.9,-7.5],[86.9,-.5],[87.27,0],[87.27,.75]]
newzones=['(zone (net 30) (net_name "GND") (layer "%s") (uuid "%s") (name "RVB22_C206_THERMAL_TAB") (hatch edge 0.5) (connect_pads yes (clearance 0.2)) (min_thickness 0.2) (fill yes (thermal_gap 0.25) (thermal_bridge_width 0.4)) (polygon (pts %s)) (priority 1))'%(l,uid('zone'+l),' '.join('(xy %s %s)'%tuple(p)for p in zonepts))for l in c.L]
remove=[capold,edgeold];add=[newcaptext]+newedges+newtracks+newvias+newzones
for x in remove:assert s.count(x)==1;s=s.replace(x,'')
s=s.rstrip()[:-1]+'\n'+'\n'.join(add)+'\n)\n';dst=D/'candidate_kicad';shutil.copytree(src,dst,dirs_exist_ok=True);out=dst/P.name;out.write_text(s)
# All changed positive copper and all new vias: collision checks across the4 layers.
_,newitems,_=c.collect(out);checks=[]
newids={uid('power'+str(i))for i in range(len(wide))}|{uid('gnd'),uid('cap_positive')}|{uid('via'+str(i))for i in range(4)}
for x in newitems:
 if x['id']not in newids and not x['id'].startswith('C206.'):continue
 other=unary_union([y['geometry']for y in newitems if y['layer']==x['layer']and y['net']!=x['net']]);gap=x['geometry'].distance(other)
 assert gap>=.149999,(x['id'],x['layer'],gap)
 assert board.buffer(-.24999).covers(x['geometry']),(x['id'],'edge')
 checks.append({'id':x['id'],'layer':x['layer'],'foreign_copper_gap_mm':gap})
geometry={**geom,'revision':'I18_C206_TAB','parent_geometry_sha256':hashlib.sha256((W/'lanes/mechanics/THERMAL_WING_GEOMETRY.json').read_bytes()).hexdigest(),'outline_mm':list(map(list,board.exterior.coords)),'tab_outline_addition_mm':outlinepts,'tab_area_added_mm2':board.area-oldoutline.area,'C206_front_body_mm':[78.35,-6.15,85.65,-1.85],'C206_height_max_mm':4.3}
(D/'CURRENT_OUTLINE_GEOMETRY.json').write_text(json.dumps(geometry,indent=2)+'\n')
proof={'iteration':'I18','source_before_sha256':freeze['base_source_sha256'],'source_after_sha256':hashlib.sha256(s.encode()).hexdigest(),'C206_before_mm':[82.6,18.4,'B.Cu'],'C206_after_mm':[82.,-4.,'F.Cu'],'positive_route_In1_mm':simple,'positive_route_length_mm':sum(math.dist(a,z)for a,z in pairs),'positive_route_width_range_mm':[.8,2.],'positive_segments':[{'a':a,'b':z,'width_mm':ww,'length_mm':math.dist(a,z)}for a,z,ww in wide],'new_vias':4,'tab_area_added_mm2':board.area-oldoutline.area,'old_outline_bounds':oldoutline.bounds,'new_outline_bounds':board.bounds,'checks':checks,'other_footprints_unchanged':True,'old_ground_zones_and_native_arcs_unchanged':True,'native_required':True}
(D/'SOURCE_CHANGE_CHECK.json').write_text(json.dumps(proof,indent=2)+'\n');(D/'TRANSPORT.json').write_text(json.dumps({'remove':remove,'add':add,'source_sha256':proof['source_after_sha256']},indent=2)+'\n')
print(json.dumps({k:v for k,v in proof.items()if k!='checks'}),flush=True)
