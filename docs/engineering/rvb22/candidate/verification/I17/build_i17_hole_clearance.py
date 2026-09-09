from pathlib import Path
import json,sys,shutil,hashlib,re,uuid,math,heapq
W=Path(__file__).resolve().parents[1];D=W/'iterations/I17_RF_hole_clearance';D.mkdir(exist_ok=True)
sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor'),str(W/'control')]
import check_combined_copper as c
from shapely.geometry import Point,LineString
from shapely.ops import unary_union
from shapely import contains_xy
import numpy as np
src=W/'iterations/I16_RF_reference/candidate_kicad';P=src/'GR86_CCA_RevB.kicad_pcb';s=P.read_text();b,items,u=c.collect(P);assert not u
freeze={'iteration':'I17','base_source_sha256':hashlib.sha256(s.encode()).hexdigest(),'known_redlines':[
 {'id':'I17-01','finding':'I16 has one native hole-clearance error: In2 +3V3 crosses J201 locating NPTH at68.46/6.','correction':'Replace short66.5/5.1 to70.7/9 detour using all NPTH holes as physical routing obstacles; retain .25mm hole clearance and RF reference corridor. Update preflight to include NPTH.'},
 {'id':'I17-02','finding':'RF, digital reference, power/thermal and documentation need final effectivity after corrected routing.','correction':'Run final native checks; recalculate all affected geometric and model bindings. Preserve unresolved thermal and supplier/physical conditions.'}],'frozen_before_edit':True}
(D/'REDLINE_FREEZE.json').write_text(json.dumps(freeze,indent=2)+'\n')
tr=json.loads((W/'iterations/I16_RF_reference/TRANSPORT.json').read_text());remove=[]
for x in tr['add']:
 a=re.search(r'\(start ([\d.-]+) ([\d.-]+)',x);z=re.search(r'\(end ([\d.-]+) ([\d.-]+)',x)
 if float(a[1])>=66.5 and float(z[1])<=70.7:remove.append(x)
assert len(remove)==5
obs=unary_union([i['geometry']for i in items if i['layer']=='In2.Cu' and i['net']!=1])
holes=unary_union([c.pad_shape(f,p) for f in c.child(b,'footprint')for p in c.child(f,'pad')if str(p[2])=='np_thru_hole'])
rf=unary_union([i['geometry']for i in items if i['layer']=='B.Cu' and i['net']in [33,34]])
start=(66.5,5.1);end=(70.7,9.);w=1.
ob=unary_union([obs.buffer(w/2+.15002),holes.buffer(w/2+.25002),rf.buffer(w/2+1.15002)])
xs=np.round(np.arange(65,72.001,.05),5);ys=np.round(np.arange(3.5,10.501,.05),5);X,Y=np.meshgrid(xs,ys);blocked=contains_xy(ob,X,Y)
def idx(p):return(round((p[0]-xs[0])/.05),round((p[1]-ys[0])/.05))
def pt(q):return(float(xs[q[0]]),float(ys[q[1]]))
a=idx(start);z=idx(end);assert not blocked[a[1],a[0]] and not blocked[z[1],z[0]]
pq=[(0,a)];cost={a:0};prev={};found=None
while pq:
 _,q=heapq.heappop(pq)
 if q==z:found=q;break
 for dx,dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,-1),(1,-1),(-1,1)]:
  nq=(q[0]+dx,q[1]+dy)
  if not(0<=nq[0]<len(xs)and 0<=nq[1]<len(ys))or blocked[nq[1],nq[0]]:continue
  if LineString([pt(q),pt(nq)]).intersects(ob):continue
  nc=cost[q]+.05*math.hypot(dx,dy)
  if nc>=cost.get(nq,1e99):continue
  cost[nq]=nc;prev[nq]=q;heapq.heappush(pq,(nc+math.dist(pt(nq),end),nq))
assert found,'No1mm corridor';seq=[found]
while seq[-1]!=a:seq.append(prev[seq[-1]])
seq.reverse();pairs=[]
for qa,qb in zip(seq,seq[1:]):
 pa,pb=pt(qa),pt(qb)
 if pairs:
  aa,bb=pairs[-1]
  if abs((bb[0]-aa[0])*(pb[1]-bb[1])-(bb[1]-aa[1])*(pb[0]-bb[0]))<1e-8:pairs[-1]=(aa,pb);continue
 pairs.append((pa,pb))
new=[]
for a,z in pairs:
 ident=str(uuid.uuid5(uuid.NAMESPACE_URL,'I17:'+json.dumps([a,z])))
 new.append('(segment (start %s %s) (end %s %s) (width 1) (layer "In2.Cu") (net 1) (uuid "%s"))'%(*a,*z,ident))
for x in remove:assert s.count(x)==1;s=s.replace(x,'')
s=s.rstrip()[:-1]+'\n'+'\n'.join(new)+'\n)\n';dst=D/'candidate_kicad';shutil.copytree(src,dst,dirs_exist_ok=True);out=dst/P.name;out.write_text(s)
checks=[]
for a,z in pairs:
 g=LineString([a,z]).buffer(w/2,quad_segs=64);checks.append({'start':a,'end':z,'copper_gap_mm':g.distance(obs),'NPTH_gap_mm':g.distance(holes),'RF_edge_gap_mm':g.distance(rf)})
assert min(x['copper_gap_mm']for x in checks)>=.15 and min(x['NPTH_gap_mm']for x in checks)>=.25
oldlen=sum(LineString([list(map(float,re.search(r'\(start ([\d.-]+) ([\d.-]+)',x).groups())),list(map(float,re.search(r'\(end ([\d.-]+) ([\d.-]+)',x).groups()))]).length for x in remove)
newlen=sum(math.dist(a,z)for a,z in pairs)
proof={'iteration':'I17','source_before_sha256':freeze['base_source_sha256'],'source_after_sha256':hashlib.sha256(s.encode()).hexdigest(),'removed_count':len(remove),'added_count':len(new),'old_length_mm':oldlen,'new_length_mm':newlen,'checks':checks,'NPTH_screen_scope':'All10 NPTH pads in the board, .25mm minimum copper-to-hole boundary. General native DRC remains required.','other_objects_unchanged':True,'native_required':True}
(D/'SOURCE_CHANGE_CHECK.json').write_text(json.dumps(proof,indent=2)+'\n');(D/'TRANSPORT.json').write_text(json.dumps({'remove':remove,'add':new,'source_sha256':proof['source_after_sha256']},indent=2)+'\n')
print(json.dumps({k:v for k,v in proof.items()if k!='checks'}),flush=True)
