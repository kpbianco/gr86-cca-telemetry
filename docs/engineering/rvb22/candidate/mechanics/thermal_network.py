"""Source polygon thermal networks. Properties/boundaries are declared model conditions."""
from pathlib import Path
import json,math
import mechanics_runtime
import numpy as np
from scipy.optimize import root
from scipy.spatial.distance import cdist
from shapely.geometry import Polygon,Point,box,LineString
from shapely.ops import unary_union
import shapely
D=Path(__file__).resolve().parent;G=json.loads((D/'THERMAL_WING_GEOMETRY.json').read_text());old=json.loads((D/'EAR_GEOMETRY.json').read_text());pcb=Polygon(old['complete_polygon_for_visualization_mm'])
for xy in old['centres_mm']:pcb=pcb.difference(Point(*xy).buffer(2.2,resolution=128))
wt,wl=[Polygon(p) for p in G['wing_polygons_mm']]
carrier=unary_union([box(-5,-12,66.5,55),box(-13,-12,-5,-1),box(-13,42.5,-5,55)])
windows=[box(2,4,57.5,37).buffer(2,resolution=128),box(14.5,-8.5,53.5,-1.5).buffer(1,resolution=128),box(14.5,42.8,53.5,49.8).buffer(1,resolution=128)]
for w in windows:carrier=carrier.difference(w)
for xy in old['centres_mm']:carrier=carrier.difference(Point(*xy).buffer(1.1,resolution=128))
bracket=[[-9,-6],[-9,47.5],[57,-8],[57,50.5]]
for xy in bracket:carrier=carrier.difference(Point(*xy).buffer(1.7,resolution=128))
anchors=[[-2.5,31],[-2.5,37],[63,16],[63,22],[63,30],[63,36]]
for x,y in anchors:carrier=carrier.difference(LineString([(x-.75,y),(x+.75,y)]).buffer(.75,resolution=128))
shapes=[pcb,wt,wl,carrier];A=np.array([s.area*1e-6 for s in shapes]);overlaps=np.array([s.intersection(carrier).area*1e-6 for s in shapes[:3]])
H=G['board_size_mm'][1]/1000;HC=.067;gap=.006;SIGMA=5.670374419e-8

def mesh(p,step):
 a,b,c,d=p.bounds;x,y=np.meshgrid(np.arange(a,c,step),np.arange(b,d,step));x=x.ravel();y=y.ravel();clipped=shapely.intersection(shapely.box(x,y,x+step,y+step),p);ar=shapely.area(clipped);mask=ar>1e-12
 return shapely.get_coordinates(shapely.centroid(clipped[mask]))*.001,ar[mask]*1e-6
views=[]
for step in [1.2,.6]:
 c,wc=mesh(carrier,step);ex=[]
 for sh in shapes[:3]:
  pts,wb=mesh(sh,step);integ=0
  for st in range(0,len(pts),128):
   rr=cdist(pts[st:st+128],c,'sqeuclidean')+gap*gap;integ+=np.sum(gap*gap/(math.pi*rr*rr)*wb[st:st+128,None]*wc[None,:])
  ex.append(float(integ))
 views.append({'grid_mm':step,'exchange_areas_m2':ex})
F=np.zeros((4,4))
for i,v in enumerate(views[-1]['exchange_areas_m2']):F[i,3]=v/A[i];F[3,i]=v/A[3]
assert np.all(F.sum(axis=1)<1)
def air(T,p):
 mu=1.716e-5*(T/273)**1.5*(273+111)/(T+111);kk=.0241*(T/273)**1.5*(273+194)/(T+194);rho=p/(287.05*T);cp=1007;return kk,mu/rho,kk/(rho*cp),mu*cp/kk,rho

def hc(Ts,Ta,L,p,factor):
 film=(Ts+Ta)/2;k,nu,al,Pr,_=air(film,p);Ra=9.80665/film*abs(Ts-Ta)*L**3/(nu*al);Nu=(.825+.387*max(Ra,1e-8)**(1/6)/(1+(.492/Pr)**(9/16))**(8/27))**2;return factor*Nu*k/L,Ra,Nu
neck=1/(300*.030*.000048/.005+.25*.030*.00144/.005)
def solve(P,eps,p,factor,feed,Ta_C=65,Tr_C=65):
 Ta=273.15+Ta_C;Tr=273.15+Tr_C;Ea=SIGMA*Tr**4
 def calc(T,details=False):
  hs=[hc(t,Ta,H if i<3 else HC,p,factor) for i,t in enumerate(T)]
  J=np.linalg.solve(np.eye(4)-(1-eps)*F,eps*SIGMA*T**4+(1-eps)*(1-F.sum(axis=1))*Ea)
  qrad=A*(J-F@J-(1-F.sum(axis=1))*Ea)+eps*SIGMA*A*(T**4-Tr**4)
  exposed=2*A.copy();exposed[:3]-=overlaps;exposed[3]-=sum(overlaps)
  qconv=np.array([h[0] for h in hs])*exposed*(T-Ta);qh=np.array([(T[0]-T[i])/(feed+neck) for i in [1,2]]);qg=np.array([air((T[i]+T[3])/2,p)[0]*overlaps[i]/gap*(T[i]-T[3]) for i in range(3)])
  res=-qrad-qconv;res[0]+=P-sum(qh);res[1:3]+=qh;res[:3]-=qg;res[3]+=sum(qg)
  if details:return {'board_average_C':float(T[0]-273.15),'upper_wing_C':float(T[1]-273.15),'lower_wing_C':float(T[2]-273.15),'carrier_C':float(T[3]-273.15),'wing_heat_total_W':float(sum(qh)),'h_board_W_m2K':float(hs[0][0]),'Ra_board':float(hs[0][1]),'body_average_plus10K_C':float(T[0]-273.15+10),'body105_engineering_target_pass':bool(T[0]-273.15+10<=105),'C206_Xcase125C_body_screen_pass':bool(T[0]-273.15+10<=125),'energy_balance_residual_W':float(max(abs(res)))}
  return res
 sol=root(calc,[Ta+30,Ta+15,Ta+15,Ta+5]);assert sol.success and max(abs(calc(sol.x)))<1e-6;return calc(sol.x,True)

def solve_chassis(P,eps,p,feed,Rinterface_each=8,Ral_each=12,Rlanding_shared=4,Tlanding_C=65,Ta_C=65,Tr_C=65):
 Ta=273.15+Ta_C;Tr=273.15+Tr_C;Tland=273.15+Tlanding_C;Ea=SIGMA*Tr**4;Fc=F.copy();Fc[1:3,:]=0;Fc[:,1:3]=0
 def calc(T,details=False):
  B=T[:4];Tsink=T[4];hs=[hc(t,Ta,H if i<3 else HC,p,.7) for i,t in enumerate(B)]
  J=np.linalg.solve(np.eye(4)-(1-eps)*Fc,eps*SIGMA*B**4+(1-eps)*(1-Fc.sum(axis=1))*Ea)
  qrad=A*(J-Fc@J-(1-Fc.sum(axis=1))*Ea);qrad[1:3]=0;qrad[:3]+=eps*SIGMA*A[:3]*(B[:3]**4-Tr**4)
  exposed=np.array([2*A[0]-overlaps[0],A[1],A[2],max(A[3]-overlaps[0],0)])
  qconv=np.array([x[0] for x in hs])*exposed*(B-Ta);qn=np.array([(B[0]-B[i])/(feed+neck) for i in [1,2]]);qp=np.array([(B[i]-Tsink)/(Rinterface_each+Ral_each) for i in [1,2]]);qg=air((B[0]+B[3])/2,p)[0]*overlaps[0]/.006*(B[0]-B[3])
  res=np.zeros(5);res[:4]=-qrad-qconv;res[0]+=P-sum(qn)-qg;res[1:3]+=qn-qp;res[3]+=qg;res[4]=sum(qp)-(Tsink-Tland)/Rlanding_shared
  if details:return {'PCB_average_C':float(B[0]-273.15),'C206_case_plus10K_C':float(B[0]-273.15+10),'C206_case125C_pass':bool(B[0]-273.15+10<=125),'wing_C':(B[1:3]-273.15).tolist(),'carrier_C':float(B[3]-273.15),'Al_frame_landing_C':float(Tsink-273.15),'heat_to_chassis_W':float(sum(qp)),'energy_residual_W':float(max(abs(res)))}
  return res
 sol=root(calc,[Ta+40,Ta+15,Ta+15,Ta+10,Tland+2]);assert sol.success and max(abs(calc(sol.x)))<1e-6;return calc(sol.x,True)
