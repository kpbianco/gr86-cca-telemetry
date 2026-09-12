#!/usr/bin/env python3
"""Exploratory only: endpoint, corridor-width and electrical margins remain unverified."""
from pathlib import Path
import heapq,math,time,json,sys
import numpy as np
import argparse
from shapely.geometry import Point
from shapely.ops import unary_union
import shapely as sh
import return_review as rr
c=rr.copper
ap=argparse.ArgumentParser();ap.add_argument('--native',type=Path,required=True);ap.add_argument('--out',type=Path,default=Path(__file__).with_name('TRANSFER_SCREEN_PATHS_EXPLORATORY.json'));ap.add_argument('--start',type=int,default=0);ap.add_argument('--end',type=int,default=60);args=ap.parse_args()
b,it,names,gg=rr.load_native(args.native)
# Model explicit through-hole bores; no ground-copper credit through a drill.
bores=[Point(c.get(v,'at')[:2]).buffer(c.get(v,'drill')[0]/2,quad_segs=24)for v in c.child(b,'via')]
for f in c.child(b,'footprint'):
 for p in c.child(f,'pad'):
  dr=c.get(p,'drill')
  if dr:
   dims=[float(x)for x in dr if isinstance(x,(int,float))]
   if dims:bores.append(c.pad_shape(f,p).centroid.buffer(max(dims)/2,quad_segs=24))
cut=unary_union(bores)
actual={l:g.difference(cut)for l,g in gg.items()}
step=.1;etch=.03;halfwidth=.025
safe={l:g.buffer(-(etch+halfwidth+step/math.sqrt(2)))for l,g in actual.items()}
common=safe['In1.Cu'].intersection(safe['In2.Cu'])
gvias=[v for v in c.child(b,'via')if names[c.get(v,'net')[0]]=='GND']
crit={'ADC_NODE','CANH','CANL','CAN_TX_MCU','CAN_RX_MCU','CAN_RXD','CAN_TXD','GPS_ANT_RF_BIASED','GPS_EXT_ANT','GPS_TX_RAW','GPS_TX_MCU','GPS_TX_BUFFER','GPS_TX_MODULE_BUFFERED','GPS_RX_MODULE','GPS_RX_BUFFER','GPS_PPS_RAW','GPS_PPS_BUFFER','GPS_PPS_MCU','OIL_EXCITATION_ADC','UART0_RX','UART0_TX','ESP_EN','ESP_GPIO0','OIL_FILTERED','OIL_EXC_FILTERED','OIL_SIG','OIL_FB','OIL_EXC_FB','OIL_AMP_OUT','OIL_EXC_AMP_OUT','GPS_FIX_RAW','GPS_SEARCH_BUFFER'}
svias=[v for v in c.child(b,'via')if names[c.get(v,'net')[0]]in crit]

def astar(poly,start,end):
 x0=min(start[0],end[0])-3;y0=min(start[1],end[1])-3
 x1=max(start[0],end[0])+3;y1=max(start[1],end[1])+3
 xx=np.arange(x0,x1+step*.5,step);yy=np.arange(y0,y1+step*.5,step)
 X,Y=np.meshgrid(xx,yy);valid=sh.contains_xy(poly,X,Y)
 def snap(p):
  ds=(X-p[0])**2+(Y-p[1])**2;ds[~valid]=np.inf;k=np.argmin(ds)
  if ds.flat[k]>.25**2:return None
  return tuple(np.unravel_index(k,ds.shape))
 a=snap(start);z=snap(end)
 if a is None or z is None:return None
 H=lambda q:math.hypot(q[0]-z[0],q[1]-z[1])*step
 heap=[(H(a),0.,a)];best={a:0.};prev={};dirs=[(i,j)for i in [-1,0,1]for j in [-1,0,1]if i or j]
 while heap:
  _,dist,q=heapq.heappop(heap)
  if dist>best[q]+1e-9:continue
  if q==z:
   path=[q]
   while q!=a:q=prev[q];path.append(q)
   path.reverse();xy=[(float(xx[c]),float(yy[r]))for r,c in path]
   return {'length_mm':dist,'xy':xy}
  for dr,dc in dirs:
   v=(q[0]+dr,q[1]+dc)
   if not(0<=v[0]<len(yy)and 0<=v[1]<len(xx))or not valid[v]:continue
   # Full-cell erosion prevents crossing a copper void through a diagonal corner.
   nd=dist+step*math.hypot(dr,dc)
   if nd<best.get(v,float('inf')):
    best[v]=nd;prev[v]=q;heapq.heappush(heap,(nd+H(v),nd,v))
 return None

out=args.out;rows=json.loads(out.read_text())if out.exists()else[]
start_index=args.start;end_index=args.end
from shapely.ops import nearest_points
for idx in range(start_index,min(len(svias),end_index)):
 v=svias[idx];p=Point(c.get(v,'at')[:2]);q=nearest_points(common,p)[0];q=q.buffer(.02).intersection(common).representative_point()
 candidates=sorted(gvias,key=lambda g:p.distance(Point(c.get(g,'at')[:2])))[:5];solutions=[]
 for g in candidates:
  gp=Point(c.get(g,'at')[:2]);disk=gp.buffer(c.get(g,'size')[0]/2-etch,quad_segs=24);paths={}
  for l in ['In1.Cu','In2.Cu']:
   path=astar(safe[l].union(disk),[q.x,q.y],[gp.x,gp.y])
   if path is None:break
   paths[l]=path
  if len(paths)==2:
   length=sum(x['length_mm']for x in paths.values())+2*p.distance(q)+2*1.76
   # Conservative deliberately narrow filament screening allocation, not extracted RF impedance.
   # Two-wire long-loop estimate plus factor2; report as an assumed screen, not a theorem.
   L=.4*length*max(1.,math.log(2*max(length,.1)/.025))
   solutions.append({'ground_via_uuid':c.get(g,'uuid')[0],'ground_via_xy_mm':c.get(g,'at')[:2],'ground_via_drill_mm':c.get(g,'drill')[0],'paths':paths,'reference_access_radius_mm':p.distance(q),'screen_loop_length_mm':length,'screen_loop_inductance_nH':L})
 if solutions:
  solution=min(solutions,key=lambda z:z['screen_loop_length_mm']);status='TWO_EXPLICIT_COPPER_CORRIDORS_FOUND'
 else:solution=None;status='NO_CORRIDOR_IN_FINITE_SEARCH'
 row={'index':idx,'signal_via_uuid':c.get(v,'uuid')[0],'net':names[c.get(v,'net')[0]],'signal_via_xy_mm':c.get(v,'at')[:2],'status':status,'solution':solution,'logic_margin_proven':False}
 rows=[r for r in rows if r['index']!=idx]+[row];out.write_text(json.dumps(sorted(rows,key=lambda r:r['index']),indent=2)+'\n')
 print(idx,row['net'],status,round(solution['screen_loop_inductance_nH'],2)if solution else '',flush=True)
