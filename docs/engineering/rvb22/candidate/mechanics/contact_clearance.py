#!/usr/bin/env python3
from mechanics_runtime import *
import sexpdata as sx,math,json,argparse
from pathlib import Path
from shapely.geometry import Polygon,box
D=Path(__file__).resolve().parent
ap=argparse.ArgumentParser();ap.add_argument('--pcb',default=str(D/'candidate_thermal_wings/GR86_CCA_RevB.kicad_pcb'));a=ap.parse_args()
r=sx.loads(Path(a.pcb).read_text());k=lambda n:str(n[0]) if isinstance(n,list) and n else '';kids=lambda n,key:[x for x in n if k(x)==key]
def one(n,key,default=None):return next(iter(kids(n,key)),default)
roi=box(63,11.5,72,28);hits=[]
for fp in kids(r,'footprint'):
 if one(fp,'layer')[1]!='B.Cu':continue
 ref=next((str(x[2]) for x in kids(fp,'property') if x[1]=='Reference'),'?');at=one(fp,'at');ang=math.radians(float(at[3]) if len(at)>3 else 0)
 def tr(p):
  x,y=float(p[1]),float(p[2]);return float(at[1])+x*math.cos(ang)+y*math.sin(ang),float(at[2])-x*math.sin(ang)+y*math.cos(ang)
 pts=[]
 for z in fp:
  if not isinstance(z,list) or not one(z,'layer') or one(z,'layer')[1]!='B.CrtYd':continue
  for typ in ['start','end','mid']:
   if one(z,typ):pts.append(tr(one(z,typ)))
 if not pts:continue
 c=box(min(p[0] for p in pts),min(p[1] for p in pts),max(p[0] for p in pts),max(p[1] for p in pts))
 if c.intersects(roi):hits.append({'ref':ref,'bounds_mm':list(c.bounds),'intersection_mm2':c.intersection(roi).area})
print(json.dumps(hits,indent=2))
