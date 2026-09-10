"""Move noncritical inner-plane traces away from critical outer-layer routes."""
from pathlib import Path
import sys,json,math,heapq,uuid,hashlib,collections,shutil,time
W=Path(__file__).resolve().parents[2];D=Path(__file__).resolve().parent
sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor'),str(W/'control')]
import numpy as np
import shapely as sh
from shapely.geometry import Point,Polygon,LineString,box
from shapely.ops import unary_union
import check_combined_copper as c
P=W/'iterations/I20_controlled_handoff/candidate_kicad/GR86_CCA_RevB.kicad_pcb'
b,items,u=c.collect(P);assert not u
names={n[1]:n[2]for n in c.child(b,'net')};segments=c.child(b,'segment');uid=lambda s:c.get(s,'uuid')[0];key=lambda p:tuple(round(float(t),6)for t in p)
critical={'ADC_NODE','CANH','CANL','CAN_TX_MCU','CAN_RX_MCU','GPS_ANT_RF_BIASED','GPS_EXT_ANT','GPS_TX_RAW','GPS_TX_MCU','GPS_TX_BUFFER','GPS_RX_MODULE','GPS_PPS_RAW','GPS_PPS_BUFFER','GPS_PPS_MCU','OIL_EXCITATION_ADC','UART0_RX','UART0_TX'}
# Clearance under the full signal width, plus .05mm positional allocation,
# in addition to the native .15mm non-ground copper clearance.
outer=unary_union([LineString([c.get(s,'start'),c.get(s,'end')]).buffer(c.get(s,'width')[0]/2+.05)for s in segments if c.get(s,'layer')==['F.Cu'] and names[c.get(s,'net')[0]]in critical])
outline=Polygon(json.loads((W/'iterations/I18_capacitor_thermal_tab/CURRENT_OUTLINE_GEOMETRY.json').read_text())['outline_mm'])
holes=unary_union([c.pad_shape(f,p)for f in c.child(b,'footprint')for p in c.child(f,'pad')if str(p[2])=='np_thru_hole'])

# Zones embedded in a .kicad_pcb footprint are expressed in board XY.
# Library-local zones must not be fed into this board-source reader.
keepouts={l:[]for l in c.L}
for parent in [b]+c.child(b,'footprint'):
 for zone in c.child(parent,'zone'):
  ko=c.child(zone,'keepout')
  if not ko or str((c.get(ko[0],'tracks')or [''])[0])!='not_allowed':continue
  for poly in c.child(zone,'polygon'):
   g=Polygon([p[1:]for p in c.child(c.child(poly,'pts')[0],'xy')])
   for layer in (c.get(zone,'layers')or c.get(zone,'layer')or []):
    if layer in keepouts:keepouts[layer].append(g)
keepouts={l:unary_union(gs)for l,gs in keepouts.items()}

# Net-specific source rules apply in addition to the generic .15mm rule.
protected_names={'PROTECTED_12V','PROTECT_CTRL_VIN','INPUT_GATE','INPUT_GATE_DRIVE','INPUT_GATE_SLEW','BUCK_FEED','5V_VIN','5V_SW','5V_BST'}
raw_names={'RAW_12V','FUSED_12V','REV_BLOCKED_12V'}
dru=P.with_suffix('.kicad_dru').read_text()
assert all("A.NetName == '"+n+"'" in dru for n in protected_names|raw_names)
def voltage_obstacles(layer,width):
 return unary_union([unary_union([i['geometry']for i in items if i['layer']==layer and names[i['net']]in nn]).buffer(width/2+gap+.00002)for nn,gap in [(protected_names,.25),(raw_names,.6)]])

def route(a,z,ob,width):
 step=.1;xs=np.round(np.arange(0,89,step),5);ys=np.round(np.arange(-9.5,51.6,step),5);X,Y=np.meshgrid(xs,ys)
 allowed=outline.buffer(-(width/2+.25));free=sh.contains_xy(allowed,X,Y)&~sh.intersects_xy(ob,X,Y)
 def xy(q):return(float(xs[q[0]]),float(ys[q[1]]))
 def near(p):
  i=round((p[0]-xs[0])/step);j=round((p[1]-ys[0])/step);qs=[]
  for dx in range(-4,5):
   for dy in range(-4,5):
    q=i+dx,j+dy
    if 0<=q[0]<len(xs)and 0<=q[1]<len(ys)and free[q[1],q[0]]:
     line=LineString([p,xy(q)])
     if not line.intersects(ob)and allowed.covers(line):qs.append(q)
  return qs
 starts=near(a);ends=set(near(z));blocked_edges=set()
 if not starts or not ends:return None,dict(start_access=len(starts),end_access=len(ends))
 for attempt in range(8):
  cost={q:math.dist(a,xy(q))for q in starts};prev={q:None for q in starts};pq=[(v+math.dist(xy(q),z),q)for q,v in cost.items()];heapq.heapify(pq);found=None
  while pq:
   f,q=heapq.heappop(pq)
   if f>cost[q]+math.dist(xy(q),z)+1e-8:continue
   if q in ends:found=q;break
   for dx,dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,-1),(1,-1),(-1,1)]:
    nq=q[0]+dx,q[1]+dy
    if not(0<=nq[0]<len(xs)and 0<=nq[1]<len(ys))or not free[nq[1],nq[0]]or(q,nq)in blocked_edges:continue
    if dx and dy and(not free[q[1],nq[0]]or not free[nq[1],q[0]]):continue
    nc=cost[q]+step*math.hypot(dx,dy)
    if nc>=cost.get(nq,1e99)-1e-12:continue
    cost[nq]=nc;prev[nq]=q;heapq.heappush(pq,(nc+math.dist(xy(nq),z),nq))
  if found is None:return None,dict(start_access=len(starts),end_access=len(ends),visited=len(cost))
  qs=[found]
  while prev[qs[-1]]is not None:qs.append(prev[qs[-1]])
  qs.reverse();bad=[]
  for qa,qz in zip(qs,qs[1:]):
   line=LineString([xy(qa),xy(qz)])
   if line.intersects(ob)or not allowed.covers(line):bad.append((qa,qz))
  if bad:
   for qa,qz in bad:blocked_edges.update([(qa,qz),(qz,qa)])
   continue
  pts=[a]+[xy(q)for q in qs]+[z];clean=[a]
  for pt in pts[1:]:
   if math.dist(clean[-1],pt)<1e-8:continue
   if len(clean)>=2:
    aa,zz=clean[-2:]
    if abs((zz[0]-aa[0])*(pt[1]-zz[1])-(zz[1]-aa[1])*(pt[0]-zz[0]))<1e-9:clean[-1]=pt;continue
   clean.append(pt)
  # Simplify grid staircases into exact-clearance 45-degree doglegs. Every
  # accepted replacement is checked as a complete continuous line geometry.
  smooth=[clean[0]];i=0
  while i<len(clean)-1:
   accepted=None
   for j in range(len(clean)-1,i,-1):
    aa,zz=clean[i],clean[j];dx=zz[0]-aa[0];dy=zz[1]-aa[1];sx0=1 if dx>=0 else -1;sy0=1 if dy>=0 else -1
    candidates=[]
    if abs(dx)>=abs(dy):
     candidates=[(aa[0]+sx0*(abs(dx)-abs(dy)),aa[1]),(aa[0]+sx0*abs(dy),zz[1])]
    else:
     candidates=[(aa[0],aa[1]+sy0*(abs(dy)-abs(dx))),(zz[0],aa[1]+sy0*abs(dx))]
    for bend in candidates:
     pp=[aa]+([bend]if math.dist(aa,bend)>1e-8 and math.dist(bend,zz)>1e-8 else [])+[zz]
     candidate=LineString(pp)
     if not candidate.intersects(ob)and allowed.covers(candidate):accepted=(j,pp);break
    if accepted:break
   if accepted is None:accepted=(i+1,[clean[i],clean[i+1]])
   i,pp=accepted;smooth.extend(pp[1:])
  clean=smooth
  line=LineString(clean);assert not line.intersects(ob)and allowed.covers(line)
  return [key(p)for p in clean],dict(start_access=len(starts),end_access=len(ends),visited=len(cost),exact_edges_checked=len(qs)-1)
 return None,dict(failure='Exact edge retry budget exhausted')

chains=[]
for name in ['LED_BLE','LED_PWR','LED_CAN','OIL_5V']:
 net=next(n for n,v in names.items()if v==name);ss=[s for s in segments if c.get(s,'net')==[net]and c.get(s,'layer')==['In1.Cu']]
 adj=collections.defaultdict(list)
 for s in ss:
  for p in [c.get(s,'start'),c.get(s,'end')]:adj[key(p)].append(s)
 anchors={p for p,ed in adj.items()if len(ed)!=2 or len({c.get(s,'width')[0]for s in ed})>1 or any(i['net']==net and i['layer']=='In1.Cu'and i['type']in ['pad','via']and i['geometry'].buffer(1e-6).contains(Point(p))for i in items)}
 visited=set()
 for a in sorted(anchors):
  for first in adj[a]:
   if uid(first)in visited:continue
   seq=[];pts=[a];s=first
   while True:
    visited.add(uid(s));seq.append(s);aa,zz=key(c.get(s,'start')),key(c.get(s,'end'));p=zz if aa==pts[-1]else aa;pts.append(p)
    if p in anchors:break
    s=next(v for v in adj[p]if uid(v)!=uid(s))
   if LineString(pts).buffer(c.get(first,'width')[0]/2+.155).intersects(outer):chains.append((net,seq,pts))
print('Intruding chains',len(chains),flush=True)
removed=[];added=[];newitems=[];results=[]
for index,(net,seq,pts)in enumerate(chains):
 width=c.get(seq[0],'width')[0];assert all(c.get(s,'width')==[width]for s in seq)
 foreign=unary_union([i['geometry']for i in items+newitems if i['layer']=='In1.Cu'and i['net']!=net and i.get('id')not in removed])
 ob=unary_union([foreign.buffer(width/2+.15002),holes.buffer(width/2+.25002),outer.buffer(width/2+.155),keepouts['In1.Cu'].buffer(width/2+.01),voltage_obstacles('In1.Cu',width)])
 path,diag=route(pts[0],pts[-1],ob,width)
 target_layer='In1.Cu'
 if not path and names[net].startswith('LED_'):
  # Both retained chain ends are through-via centers. Moving this complete
  # chain to In2 needs no added hole or unreviewed layer transition.
  assert all(any(c.get(v,'net')==[net]and math.dist(c.get(v,'at')[:2],q)<c.get(v,'size')[0]/2-.02 for v in c.child(b,'via'))for q in [pts[0],pts[-1]])
  protected=unary_union([LineString([c.get(s,'start'),c.get(s,'end')]).buffer(c.get(s,'width')[0]/2+(.95 if names[c.get(s,'net')[0]]in ['GPS_EXT_ANT','GPS_ANT_RF_BIASED']else .05))for s in segments if c.get(s,'layer')[0]in ['B.Cu','In1.Cu']and names[c.get(s,'net')[0]]in critical])
  foreign2=unary_union([i['geometry']for i in items+newitems if i['layer']=='In2.Cu'and i['net']!=net and i.get('id')not in removed])
  ob2=unary_union([foreign2.buffer(width/2+.15002),holes.buffer(width/2+.25002),protected.buffer(width/2+.155),keepouts['In2.Cu'].buffer(width/2+.01),voltage_obstacles('In2.Cu',width)])
  path,diag2=route(pts[0],pts[-1],ob2,width);diag={'In1':diag,'In2':diag2}
  if path:target_layer='In2.Cu';foreign=foreign2;outer_check=protected
  else:outer_check=outer
 else:outer_check=outer
 row=dict(net=names[net],layer='In1.Cu',original_uuids=[uid(s)for s in seq],original_path_mm=pts,width_mm=width,diagnostics=diag)
 if path:
  row.update(status='REROUTED_PREFLIGHT_PASS',new_layer=target_layer,path_mm=path,original_length_mm=LineString(pts).length,new_length_mm=LineString(path).length,minimum_projected_reference_clearance_mm=LineString(path).buffer(width/2).distance(outer_check),minimum_foreign_copper_clearance_mm=LineString(path).buffer(width/2).distance(foreign))
  for a,z in zip(path,path[1:]):
   ident=str(uuid.uuid5(uuid.NAMESPACE_URL,'I23-inner:'+json.dumps([net,index,a,z])))
   added.append('(segment (start %s %s) (end %s %s) (width %s) (layer "%s") (net %s) (uuid "%s"))'%(*a,*z,width,target_layer,net,ident))
   newitems.append(dict(geometry=LineString([a,z]).buffer(width/2),net=net,layer=target_layer))
  removed.extend(uid(s)for s in seq)
 else:row['status']='NO_FEASIBLE_FIXED_ANCHOR_CORRIDOR'
 results.append(row);(D/'INTRUSION_TRIAL.json').write_text(json.dumps(dict(chains=results,removed_uuids=removed,new_segments=added),indent=2)+'\n');print(index+1,names[net],row['status'],diag,flush=True)
s=P.read_text()
for ident in removed:
 lines=[line for line in s.splitlines()if line.lstrip().startswith('(segment ')and ident in line];assert len(lines)==1,ident;s=s.replace(lines[0],'',1)
s=s.rstrip()[:-1]+'\n'+'\n'.join(added)+'\n)\n'
dst=W/'iterations/I23_return_clearance/candidate_kicad';shutil.copytree(P.parent,dst,dirs_exist_ok=True);(dst/P.name).write_text(s)
print('Candidate',len(removed),'removed',len(added),'added',hashlib.sha256(s.encode()).hexdigest(),flush=True)
