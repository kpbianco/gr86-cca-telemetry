"""I24 scoped routing study, preserving source objects outside selected chains."""
from pathlib import Path
import sys,json,hashlib,math,collections,uuid,shutil
W=Path(__file__).resolve().parents[2];D=Path(__file__).resolve().parent
preamble=(W/'analyses/return_i23/repair_intrusions.py').read_text().split('\nchains=[]')[0]
preamble=preamble.replace("iterations/I20_controlled_handoff/", "iterations/I23_return_clearance/")
ns={'__file__':str(W/'analyses/return_i23/repair_intrusions.py')};exec(compile(preamble,'I23_exact_corridor_library','exec'),ns)
c=ns['c'];sh=ns['sh'];unary_union=ns['unary_union'];LineString=ns['LineString'];Point=ns['Point'];Polygon=ns['Polygon'];route=ns['route'];b=ns['b'];items=ns['items'];P=ns['P'];names=ns['names'];keepouts=ns['keepouts'];holes=ns['holes'];outline=ns['outline']
N=W/'runtime/hosted/run26_retry2/extracted/native_I06_hosted/candidate_kicad/GR86_CCA_RevB.kicad_pcb'
nb,ni,u=c.collect(N);assert not u
gnd={l:[x['geometry']for x in ni if x['layer']==l and x['net']==30]for l in c.L}
for z in c.child(nb,'zone'):
 if c.child(z,'keepout')or c.get(z,'net')!=[30]:continue
 for f in c.child(z,'filled_polygon'):
  l=c.get(f,'layer')[0];g=sh.make_valid(Polygon([p[1:]for p in c.child(c.child(f,'pts')[0],'xy')]));gnd[l].append(g)
gnd={l:unary_union(v)for l,v in gnd.items()}
uid=lambda s:c.get(s,'uuid')[0]
key=lambda p:tuple(round(float(v),6)for v in p)
segs=c.child(b,'segment');removed=[];added=[];newitems=[];rows=[]
def chains_for(net,layer):
 ss=[s for s in segs if c.get(s,'net')==[net]and c.get(s,'layer')==[layer]];adj=collections.defaultdict(list)
 for s in ss:
  for p in [c.get(s,'start'),c.get(s,'end')]:adj[key(p)].append(s)
 anchors={p for p,ed in adj.items()if len(ed)!=2 or len({c.get(s,'width')[0]for s in ed})>1 or any(i['net']==net and i['layer']==layer and i['type']in ['pad','via']and i['geometry'].buffer(1e-6).contains(Point(p))for i in items)}
 visit=set();ret=[]
 for a in sorted(anchors):
  for first in adj[a]:
   if uid(first)in visit:continue
   seq=[];pts=[a];s=first
   while True:
    visit.add(uid(s));seq.append(s);aa,zz=key(c.get(s,'start')),key(c.get(s,'end'));pt=zz if aa==pts[-1]else aa;pts.append(pt)
    if pt in anchors:break
    s=next(v for v in adj[pt]if uid(v)!=uid(s))
   ret.append((seq,pts))
 return ret

def try_path(net,seq,pts,layer):
 width=c.get(seq[0],'width')[0];ref='In1.Cu'if layer=='F.Cu'else'In2.Cu'
 foreign=unary_union([i['geometry']for i in items+newitems if i['layer']==layer and i['net']!=net and i.get('id')not in removed])
 own=unary_union([i['geometry'].buffer(.155)for i in ni if i['net']==net and i['layer']==ref and i['type']in ['via','pad']])
 # Preserve full nominal trace width plus50um route-placement allocation over
 # known native filled ground, except explicit own via/pad anti-pad envelopes.
 allowed_ground=gnd[ref].buffer(-width/2-.05).union(own.buffer(width/2+.05002))
 missing=outline.difference(allowed_ground)
 ob=unary_union([foreign.buffer(width/2+.15002),holes.buffer(width/2+.25002),keepouts[layer].buffer(width/2+.01),ns['voltage_obstacles'](layer,width),missing])
 path,diag=route(pts[0],pts[-1],ob,width)
 return path,diag

if __name__=='__main__':
 original=json.loads((D/'REDLINE_FREEZE.json').read_text());badids={r['uuid']for r in original['remaining_segments']}
 for name in ['CAN_RX_MCU']:
  net=next(k for k,v in names.items()if v==name)
  for seq,pts in chains_for(net,'F.Cu'):
   if not any(uid(s)in badids for s in seq):continue
   row={'net':name,'original_uuids':[uid(s)for s in seq],'original_path_mm':pts,'original_length_mm':LineString(pts).length,'trials':[]}
   for layer in ['F.Cu','B.Cu']:
    print('Routing',name,pts[0],pts[-1],len(seq),layer,flush=True)
    path,diag=try_path(net,seq,pts,layer);row['trials'].append({'layer':layer,'diagnostics':diag,'path_mm':path})
    if path:
     row.update(status='PREFLIGHT_ROUTE_FOUND',new_layer=layer,path_mm=path,new_length_mm=LineString(path).length,width_mm=c.get(seq[0],'width')[0]);break
   else:row['status']='NO_NATIVE_REFERENCE_CORRIDOR'
   rows.append(row);(D/'ROUTE_TRIALS.json').write_text(json.dumps(rows,indent=2)+'\n')
   print(row['status'],row.get('path_mm'),flush=True)
