#!/usr/bin/env python3
"""Native footprint envelopes + source-bound tall part heights. Never substitutes Fab graphics for a 3D solid."""
from mechanics_runtime import *
import sexpdata as sx,math,json,hashlib,argparse
from pathlib import Path
from shapely.geometry import box,Point,Polygon
D=Path(__file__).resolve().parent
ap=argparse.ArgumentParser();ap.add_argument('--pcb',required=True);a=ap.parse_args();P=Path(a.pcb);r=sx.loads(P.read_text());k=lambda n:str(n[0]) if isinstance(n,list) and n else '';kids=lambda n,key:[x for x in n if k(x)==key]
def one(n,key,default=None):return next(iter(kids(n,key)),default)
roi=box(66.6,11.8,72,28.8);rows=[]
# Explicit primary-bound dimensions known this iteration; all other missing heights are enumerated.
heights={'U201':(3.25,'Espressif ESP32-S3-WROOM-1 dimensions25.5±.2×18±.2×3.1±.15mm'), 'C206':(4.3,'KEMET T2073 X-case7343-43 maxbody4.3mm'), 'F101':(2.9,'Littelfuse885 package drawing2.8±.1mm'), 'L151':(3.1,'CoilcraftXGL5030 primary mechanical drawing')}
for fp in kids(r,'footprint'):
 props={str(x[1]):str(x[2]) for x in kids(fp,'property')};ref=props.get('Reference','?');layer=str(one(fp,'layer')[1]);at=one(fp,'at');ang=math.radians(float(at[3]) if len(at)>3 else 0)
 if props.get('MPN')=='MCAST32MSB7226KPNA01':heights[ref]=(2.7,'Exact TaiyoYuden TYCOMPAS part page:bodyL3.2±.30,W2.5±.20,T2.5±.20mm')
 if props.get('MPN','').startswith('WSL1206'):heights[ref]=(.889,'Vishay30100 p2:WSL1206 H0.635±.254mm')
 if props.get('MPN','').startswith(('WSL0603','WSLP0603')):heights[ref]=(.533,'Vishay30100/30122 primary family:WSL/WSLP0603 H0.406±.127mm')
 def tr(x,y):return float(at[1])+x*math.cos(ang)+y*math.sin(ang),float(at[2])-x*math.sin(ang)+y*math.cos(ang)
 def bounds(layername):
  pts=[]
  for z in fp:
   if not isinstance(z,list) or not one(z,'layer') or str(one(z,'layer')[1])!=layername:continue
   if k(z)=='fp_rect':
    u=one(z,'start');v=one(z,'end')
    for x in [float(u[1]),float(v[1])]:
     for y in [float(u[2]),float(v[2])]:pts.append(tr(x,y))
   else:
    for typ in ['start','end','mid']:
     q=one(z,typ)
     if q:pts.append(tr(float(q[1]),float(q[2])))
    pp=one(z,'pts')
    if pp:
     for q in pp[1:]:
      if k(q)=='xy':pts.append(tr(float(q[1]),float(q[2])))
  return box(min(p[0] for p in pts),min(p[1] for p in pts),max(p[0] for p in pts),max(p[1] for p in pts)) if pts else None
 c=bounds('B.CrtYd' if layer=='B.Cu' else 'F.CrtYd');f=bounds('B.Fab' if layer=='B.Cu' else 'F.Fab')
 # ESP fab drawings include antenna marks; replace with exact manufacturer body bounds.
 if ref=='U201':f=box(68.754766,14,94.254766,32)
 models=[str(x[1]) for x in kids(fp,'model')]
 rows.append({'ref':ref,'MPN':props.get('MPN'), 'side':layer,'courtyard_bounds_mm':list(c.bounds) if c is not None else None,'Fab_graphic_bounds_mm':list(f.bounds) if f is not None else None,'manufacturer_max_body_height_mm':heights.get(ref,(None,None))[0],'height_evidence':heights.get(ref,(None,None))[1],'model_references':models,'strip_courtyard_overlap_mm2':float(c.intersection(roi).area) if layer=='B.Cu' and c is not None else 0,'strip_courtyard_distance_mm':float(c.distance(roi)) if layer=='B.Cu' and c is not None else None,'scope':'Courtyard/Fab bounds are XY source screens, not validated package, lead, solder or cable solids.'})
physical=[x for x in rows if x['MPN'] and not x['ref'].startswith(('TP','MH','SJ')) and x['ref']!='J201']
r={'source_PCB_sha256':hashlib.sha256(P.read_bytes()).hexdigest(),'source_PCB_path':str(P),'status':'EXECUTED_SOURCE_XY_ENVELOPE_SCREEN_WITH_ENUMERATED_Z_GAPS','contact_mm':[66.6,11.8,72,28.8],'rows':rows,'physical_reference_count':len(physical),'strip_hits':[x for x in physical if x['strip_courtyard_overlap_mm2']>1e-9],'nearest_strip_courtyards':sorted([x for x in physical if x['strip_courtyard_distance_mm'] is not None],key=lambda x:x['strip_courtyard_distance_mm'])[:8],'missing_primary_height_refs':[x['ref'] for x in physical if x['manufacturer_max_body_height_mm'] is None],'missing_model_reference_refs':[x['ref'] for x in physical if not x['model_references']],'missing_courtyard_refs':[x['ref'] for x in physical if x['courtyard_bounds_mm'] is None],'important_unresolved_solids':['F1 exact0430451200 mated plug/latch/wire-exit and extraction sweep','U401 PA1616D owner-solder joint/standoff and cable routing','L121 MSS1246T-473MLC and taller power-package worst-caselead/solder geometry','Every unresolved model path or unbound manufacturer Zheight; numbers enumerated above','Actual dashboard bracket/enclosure and flexible-harness shapes'],'checks':{'strip_no_native_bottom_courtyard_overlap':not any(x['strip_courtyard_overlap_mm2']>1e-9 for x in physical),'new_cap_maxheight_within7mm_allocation':4.3+.25+.51+.75+.20<7},'required_next_execution':'Native STEP/body export with resolved model paths and exact populated/DNP effectivity; 3D collision/clearance under tolerances. Missing heights are desktop evidence gaps, not claimed unavailable physical tests.'}
(D/'FINAL_COMPONENT_ENVELOPES.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({'sha256':r['source_PCB_sha256'],'checks':r['checks'],'missing_heights':len(r['missing_primary_height_refs']),'missing_models':len(r['missing_model_reference_refs']),'nearest':[{'ref':x['ref'],'distance_mm':x['strip_courtyard_distance_mm']} for x in r['nearest_strip_courtyards']]}))
