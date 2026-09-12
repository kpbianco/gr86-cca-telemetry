"""Conservative subset of actual In1 ground: full square cells only, no cell bridges.

DC resistance of a restricted copper subset, single contact cell per actual via.
The grid is eroded by its half diagonal, retaining only cells wholly in copper.
Omitted copper and parallel layers cannot reduce this calculated DC resistance.
Finite-volume cell-centre spreading is still a numerical approximation; two
meshes and an explicit inflation are reported, not a manufactured measurement.
"""
from pathlib import Path
import sys,json,math,hashlib,argparse
D=Path(__file__).resolve().parent;W=D.parents[1]
sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor'),str(W/'control')]
import numpy as np,shapely as sh,sexpdata as sx,pyamg
from shapely.geometry import Polygon,Point
from shapely.ops import unary_union
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.sparse.linalg import cg
import check_combined_copper as c
ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('--native-dir',type=Path,required=True)
ap.add_argument('--expected-hash',required=True)
ap.add_argument('--expected-source-hash',required=True)
ap.add_argument('--output-dir',type=Path,default=D)
args=ap.parse_args();D=args.output_dir.resolve();D.mkdir(parents=True,exist_ok=True)
P=args.native_dir.resolve()/'candidate_kicad/GR86_CCA_RevB.kicad_pcb'
assert hashlib.sha256(P.read_bytes()).hexdigest()==args.expected_hash,'Filled PCB hash mismatch'
native=json.loads((args.native_dir/'RESULT.json').read_text())
assert native['inputs']['cad']['GR86_CCA_RevB.kicad_pcb']==args.expected_source_hash,'Native candidate source mismatch'
assert len(native['output_postconditions'])==14 and all(x['pass']for x in native['output_postconditions'].values()) and native['native_report_finding_count']==0
baseline=W/'runtime/hosted/run26_retry2/extracted/native_I06_hosted/candidate_kicad/GR86_CCA_RevB.kicad_pcb'
assert hashlib.sha256(baseline.read_bytes()).hexdigest()=='3703e280fdbfab602721db5c41fad83669c1bc936725ecf508a584db61373fce'
b,it,u=c.collect(P);assert not u
def polygon_parts(g):
 if g.geom_type=='Polygon':return [g]
 return [q for a in getattr(g,'geoms',[])for q in polygon_parts(a)]
geoms=[]
for z in c.child(b,'zone'):
 if c.get(z,'net')!=[30] or c.child(z,'keepout'):continue
 for f in c.child(z,'filled_polygon'):
  if c.get(f,'layer')!=['In1.Cu']:continue
  raw=Polygon([p[1:]for p in c.child(c.child(f,'pts')[0],'xy')]);fixed=unary_union(polygon_parts(sh.make_valid(raw)))
  assert abs(raw.area-fixed.area)<1e-5
  geoms.append(fixed)
geoms.extend(x['geometry']for x in it if x['layer']=='In1.Cu' and x['net']==30)
holes=[]
for v in c.child(b,'via'):holes.append(Point(*c.get(v,'at')[:2]).buffer(c.get(v,'drill')[0]/2,quad_segs=32))
# PTH lands are solid in the independent pad parser. Remove every real
# footprint drill too; five GND PTH bores were omitted by the inherited I23
# solver. NPTH and foreign-net PTH exclusions are explicit rather than assumed.
footprint_holes=0
for f in c.child(b,'footprint'):
 for p in c.child(f,'pad'):
  dd=c.get(p,'drill')
  if not dd:continue
  assert len(dd)==1 and isinstance(dd[0],(int,float)),'Unsupported non-round or offset bore'
  assert str(p[2])in ['thru_hole','np_thru_hole']
  center=c.pad_shape(f,p).centroid
  holes.append(center.buffer(dd[0]/2,quad_segs=32));footprint_holes+=1
raw_ground=unary_union(geoms)
g=raw_ground.difference(unary_union(holes))
ports={'source_via':(37.675,24.,.3),'load_EP_via':(78.819766,21.275,.2),'C206_ground_via':(85.12,-6.8,.3)}
# Retained scalar trace/barrel terms require unchanged physical terminals.
bb,_,bu=c.collect(baseline);assert not bu
terminal_tracks={'4c8ad6a2-9b32-4016-9596-d0b0bce231a3','6e1eba61-9b5e-5732-91ec-2bb8003465c9'}
terminal_pads={('C157','2'),('U201','41'),('C206','2')}
def terminal_geometry(board):
 answer={'tracks':{},'pads':{},'vias':{},'stackup':c.child(c.child(board,'setup')[0],'stackup')}
 for seg in c.child(board,'segment'):
  uid=c.get(seg,'uuid')[0]
  if uid in terminal_tracks:answer['tracks'][uid]=seg
 for f in c.child(board,'footprint'):
  ref=c.prop(f)['Reference']
  for pad in c.child(f,'pad'):
   if (ref,str(pad[1]))in terminal_pads:answer['pads'][ref+'.'+str(pad[1])]=[c.get(f,'at'),pad]
 for name,(x,y,dr)in ports.items():
  matching=[v for v in c.child(board,'via')if math.dist(c.get(v,'at')[:2],(x,y))<1e-7]
  assert len(matching)==1,(name,len(matching))
  via=matching[0];assert c.get(via,'net')==[30]and c.get(via,'drill')==[dr]
  answer['vias'][name]=via
 assert len(answer['tracks'])==2 and len(answer['pads'])==3
 return sx.dumps(answer['stackup']),sx.dumps(sorted(answer['tracks'].items())),sx.dumps(sorted(answer['pads'].items())),sx.dumps(list(answer['vias'].items()))
assert terminal_geometry(b)==terminal_geometry(bb),'Terminal or stack geometry changed; regenerate trace/barrel contract'
terminal_signature=hashlib.sha256('\n'.join(terminal_geometry(b)).encode()).hexdigest()
rho=1.724e-8*(1+.00393*130);sheet=rho/24e-6;rows=[]
for h in [.1,.075]:
 core=g.buffer(-math.sqrt(2)*h/2-1e-6);xmin,ymin,xmax,ymax=g.bounds
 xs=np.arange(math.floor(xmin/h)*h+h/2,xmax,h);ys=np.arange(math.floor(ymin/h)*h+h/2,ymax,h)
 X,Y=np.meshgrid(xs,ys);valid=sh.contains_xy(core,X,Y);ij=np.full(valid.shape,-1,dtype=np.int32);ij[valid]=np.arange(valid.sum());n=int(valid.sum())
 aa=[];bb=[]
 for a,z in [(ij[:,:-1],ij[:,1:]),(ij[:-1,:],ij[1:,:])]:
  ok=(a>=0)&(z>=0);aa.append(a[ok]);bb.append(z[ok])
 a=np.concatenate(aa);z=np.concatenate(bb);one=np.ones(len(a))/sheet
 A=coo_matrix((np.r_[one,one,-one,-one],(np.r_[a,z,a,z],np.r_[a,z,z,a])),shape=(n,n)).tocsr()
 nc,lab=connected_components(A);xx=X[valid];yy=Y[valid];pi={}
 for name,(x,y,dr) in ports.items():
  d=np.hypot(xx-x,yy-y);eligible=np.flatnonzero((d>dr/2)&(d<.3))
  if not len(eligible):
   print('No wholly retained land cell',h,name,flush=True);pi={};break
  # Closest retained cell centre on actual via land, favour largest component.
  biggest=max(eligible,key=lambda i:np.count_nonzero(lab==lab[i]));same=eligible[lab[eligible]==lab[biggest]];k=int(same[np.argmin(d[same])]);pi[name]=k
 if not pi:
  rows.append({'mesh_mm':h,'status':'NO_FULL_LAND_CELL','nodes':n});continue
 assert len({lab[i]for i in pi.values()})==1
 keep=np.flatnonzero(lab==lab[next(iter(pi.values()))]);remap=np.full(n,-1);remap[keep]=np.arange(len(keep));B=A[keep][:,keep];pi={k:int(remap[v])for k,v in pi.items()}
 ground=pi['load_EP_via'];free=np.flatnonzero(np.arange(len(keep))!=ground);C=B[free][:,free];ml=pyamg.smoothed_aggregation_solver(C,max_coarse=500);M=ml.aspreconditioner(cycle='V');measures=[]
 for name in ['source_via','C206_ground_via']:
  rhs=np.zeros(len(keep));rhs[pi[name]]=1;rhs[ground]=-1
  v,info=cg(C,rhs[free],M=M,rtol=1e-10,atol=1e-12,maxiter=2500);assert info==0,info
  vv=np.zeros(len(keep));vv[free]=v;r=float(vv[pi[name]]);res=float(np.max(abs(B@vv-rhs)))
  assert res<1e-7
  measures.append({'from':name,'to':'load_EP_via','restricted_plane_R_ohm':r,'max_KCL_residual_A':res})
 out={'mesh_mm':h,'status':'SOLVED_RESTRICTED_GROUND_SUBSET','nodes':len(keep),'retained_area_mm2':len(keep)*h*h,'ports':pi,'measurements':measures};rows.append(out);print(json.dumps(out),flush=True)
(D/'GROUND_PLANE_REFINED.json').write_text(json.dumps({'source_filled_pcb_sha256':hashlib.sha256(P.read_bytes()).hexdigest(),'source_PCB_sha256':args.expected_source_hash,'analyzer_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'terminal_geometry_signature_sha256':terminal_signature,'explicit_footprint_bores_removed':footprint_holes,'total_bores_removed':len(holes),'removed_drill_copper_area_mm2':raw_ground.area-g.area,'layer':'In1.Cu','temperature_C':150,'copper_um':24,'sheet_R_ohm':sheet,'rows':rows,'limits':['Includes only actual In1 ground subset. No removed copper, layer parallel paths or extra contacts are credited.','No disconnected pieces are joined within a cell. All cells fit entirely in source copper by half-diagonal erosion.','Point current injection represents one retained cell of the actual via land. Via barrels and source/terminal tracks are added separately.','Numerical centre-to-centre finite volume is not a strict continuum upper-bound theorem; use two-mesh result and explicit numerical margin.']},indent=2)+'\n')

# Retain the original finite-power model's non-plane terms exactly. For the
# plane term use the larger of the two freshly solved meshes plus the original
# 10% inflation. No manufactured contact or exact continuum theorem is claimed.
old=json.loads((W/'analyses/power_i19/GROUND_PLANE_REFINED.json').read_text())
oldrow=next(r for r in old['rows']if r['mesh_mm']==.075)
original=json.loads((W/'analyses/power_i19/GROUND_RETURN_CONTRACT.json').read_text())
assert len(rows)==2 and all(r['status']=='SOLVED_RESTRICTED_GROUND_SUBSET'for r in rows)
contract={k:v for k,v in original.items()}
terms=[]
for index,(key,limitkey)in enumerate([
 ('source_to_load_ground_upper_ohm','shared_return_allocation_ohm'),
 ('C206_to_load_plane_plus_load_barrel_upper_ohm','C206_ground_spreading_allocation_ohm')]):
 nonplane=original[key]-1.1*oldrow['measurements'][index]['restricted_plane_R_ohm']
 assert nonplane>0
 values=[r['measurements'][index]['restricted_plane_R_ohm']for r in rows]
 plane=max(values);contract[key]=nonplane+1.1*plane
 terms.append(dict(contract_key=key,retained_nonplane_ohm=nonplane,mesh_plane_values_ohm=values,credited_plane_ohm=plane,numerical_multiplier=1.1,margin_ohm=contract[limitkey]-contract[key],pass_allocation=contract[key]<contract[limitkey]))
contract.update(status='PASS_RETAINED_POWER_RETURN_ALLOCATIONS'if all(t['pass_allocation']for t in terms)else'FINDINGS_POWER_RETURN_ALLOCATION',
 original_contract=original,filled_PCB_sha256=args.expected_hash,source_PCB_sha256=args.expected_source_hash,
 method='Recompute actual In1 restricted copper including all via/PTH/NPTH bores at 0.1 and 0.075 mm; retain original trace/barrel terms after exact terminal/stack geometry identity check; use larger mesh resistance and original 10% inflation.',
 terminal_geometry_signature_sha256=terminal_signature,terms=terms,physical_tests_claimed=0)
(D/'GROUND_RETURN_CONTRACT.json').write_text(json.dumps(contract,indent=2)+'\n')
print(json.dumps(contract,indent=2),flush=True)
if not all(t['pass_allocation']for t in terms):raise SystemExit(1)
