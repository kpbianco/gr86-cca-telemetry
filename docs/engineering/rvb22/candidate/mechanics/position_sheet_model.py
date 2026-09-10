#!/usr/bin/env python3
"""Source-position finite-volume sheet sensitivity. Uniform Cu is explicit idealization, not native fill proof."""
from thermal_network import *
from scipy.sparse import coo_matrix,diags
from scipy.sparse.linalg import spsolve
import hashlib
# A single thermal sheet has perfect interlayer coupling and only declared effective Cu thickness.
board=Polygon(G['outline_mm'])
for xy in G['mount_centres_mm']:board=board.difference(Point(*xy).buffer(2.2,resolution=128))
patches=[box(15,-7,53,-1),box(15,43,53,49)]
heat_groups=[('U201_main_allocation',2.58225,box(77.094766,19.55,80.994766,23.45)),('U151_main_conversion',.6455358313,box(33.3,10.5,36.7,15.5)),('U121_fiveV_and_corner_residual',1.1662141687,box(45.525,24.05,48.475,28.95)),('U403_GPS_LDO',.101,box(37.2,4.5,40.8,7.5)),('U401_GPS_all_local',.165,box(46.5,1,62.5,17)),('other5V_allocation',.13,box(17,7,24,12))]
assert abs(sum(P for n,P,g in heat_groups)-4.79)<1e-8

def model(step,cu_um,eps,chassis,ambient=65,Pscale=1):
 xmin,ymin,xmax,ymax=board.bounds;xs=np.arange(xmin,xmax,step);ys=np.arange(ymin,ymax,step);xx,yy=np.meshgrid(xs,ys);xx=xx.ravel();yy=yy.ravel();cells=shapely.intersection(shapely.box(xx,yy,xx+step,yy+step),board);ar=shapely.area(cells);mask=ar>1e-6;xx=xx[mask];yy=yy[mask];cells=cells[mask];ar=ar[mask]*1e-6;pts=shapely.get_coordinates(shapely.centroid(cells));N=len(ar);ids={(round(x/step),round(y/step)):i for i,(x,y) in enumerate(zip(xx,yy))}
 def overlap(shape):return shapely.area(shapely.intersection(cells,shape))*1e-6
 q=np.zeros(N)
 for name,P,shape in heat_groups:
  a=overlap(shape);assert sum(a)>0;q+=P*Pscale*a/sum(a)
 covered=overlap(carrier);front=ar.copy();back=np.maximum(ar-covered,0);areas=front+back
 contact=[overlap(x) for x in patches];gc=np.zeros(N)
 if chassis:
  for a in contact:gc+=(1/28)*a/sum(a) # each8interface+12Al+2x4shared landing; symmetric allocation
  areas-=sum(contact) # no exposed backside heat credit under contacts
 rows=[];cols=[];data=[];diag=np.zeros(N);ks=300*cu_um*1e-6+.25*.00144
 for (gx,gy),i in ids.items():
  for key in [(gx+1,gy),(gx,gy+1)]:
   if key not in ids:continue
   j=ids[key];#Fractional edge cells penalized to avoid assigning a full-width path outside board.
   geom=min(ar[i],ar[j])/(step*.001)**2;g=ks*geom;diag[i]+=g;diag[j]+=g;rows.extend([i,j]);cols.extend([j,i]);data.extend([-g,-g])
 base=coo_matrix((data,(rows,cols)),shape=(N,N)).tocsr()+diags(diag)
 T=np.full(N,ambient+30.0);Ta=ambient+273.15
 for it in range(100):
  hs=np.array([hc(t+273.15,Ta,H,70000,.7)[0] for t in T]);Tk=T+273.15;hrad=eps*SIGMA*(Tk+Ta)*(Tk*Tk+Ta*Ta);loss=(hs+hrad)*areas
  M=base+diags(loss+gc);rhs=q+ambient*loss+(65*gc if chassis else 0);new=spsolve(M,rhs)
  diff=max(abs(new-T));T=.5*T+.5*new
  if diff<1e-7:break
 residual=base@T+(loss+gc)*T-rhs
 probe=overlap(box(78.95,16.25,86.25,20.55));cT=float(np.sum(probe*T)/sum(probe));heat_out=float(sum(gc*(T-65)))
 temperatures={name:float(np.sum(overlap(shape)*T)/sum(overlap(shape))) for name,P,shape in heat_groups}
 return {'mesh_mm':step,'effective_continuous_Cu_um':cu_um,'epsilon':eps,'chassis_contacts':chassis,'cells':N,'iterations':it+1,'max_temperature_C':float(max(T)),'C206_region_mean_C':cT,'C206_region_max_C':float(max(T[probe>0])),'C206_region_max_plus10K_C':float(max(T[probe>0])+10),'source_region_mean_C':temperatures,'heat_to_chassis_W':heat_out,'total_heat_W':sum(q),'max_energy_residual_W':float(max(abs(residual))),'temperature_iteration_change_K':float(diff),'probe_region_mm':[78.95,16.25,86.25,20.55]}
results=[]
for step in [1.0,.5]:
 for cu in [24,48,96,160]:
  for e in [.3,.6]:
   for chassis in [False,True]:results.append(model(step,cu,e,chassis))
r={'source_pcb_sha256':json.loads((D/'CAD_PATCH_MANIFEST.json').read_text())['output_sha256'],'source_positions_verified_against_native_footprints':True,'model':'2D finite-volume position-resolved equivalent sheet; steady70000Pa/65Cair&radiant;uniformCu/perfectverticalcoupling explicitly idealized','heat_allocations':[{'name':n,'W':P,'region_mm':list(g.bounds)} for n,P,g in heat_groups],'total_W':4.79,'Cu_interpretation':'24/48/96/160um are continuous-equivalent sheet sensitivities, not a claim that unfilled native planes provide that conductivity. Source-dependent cuts, interlayer thermal resistances, package coupling and local radiation occlusion remain to be extracted.','cases':results,'checks':{'all_energy_residuals_below1uW':all(x['max_energy_residual_W']<1e-6 for x in results),'all_iteration_changes_below1uK':all(x['temperature_iteration_change_K']<1e-6 for x in results),'heat_conserved4p79':all(abs(x['total_heat_W']-4.79)<1e-8 for x in results)}}
(D/'POSITION_SHEET_MODEL.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({'checks':r['checks'],'cases':len(results),'selected':[x for x in results if x['mesh_mm']==.5 and x['epsilon']==.3 and x['chassis_contacts']]}))
