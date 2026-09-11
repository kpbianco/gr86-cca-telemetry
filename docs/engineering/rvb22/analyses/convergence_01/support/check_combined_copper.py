#!/usr/bin/env python3
"""Reconstructed independent explicit-copper graph/clearance screen; no zone-fill credit."""
import argparse,hashlib,json,math,pathlib,sys,collections
import sexpdata as sx
from shapely.geometry import Point,LineString,box
from shapely.affinity import rotate,translate
from shapely.strtree import STRtree
L=['F.Cu','In1.Cu','In2.Cu','B.Cu']
def tag(x):return str(x[0])if isinstance(x,list)and x else ''
def child(x,k):return [a for a in x[1:]if tag(a)==k]
def get(x,k,default=None):return next((a[1:]for a in x[1:]if tag(a)==k),default)
def prop(x):return {a[1]:a[2]for a in child(x,'property')}
def pad_shape(f,p):
 at=get(f,'at');pa=get(p,'at');ang=math.radians(at[2]if len(at)>2 else 0);w,h=get(p,'size');a=pa[2]if len(pa)>2 else 0
 cx=at[0]+pa[0]*math.cos(ang)+pa[1]*math.sin(ang);cy=at[1]-pa[0]*math.sin(ang)+pa[1]*math.cos(ang)
 shape=str(p[3])
 if shape=='circle':g=Point(0,0).buffer(w/2,quad_segs=64)
 elif shape=='rect':g=box(-w/2,-h/2,w/2,h/2)
 elif shape=='roundrect':
  rr=min(w,h)*get(p,'roundrect_rratio',[0])[0];g=box(-w/2+rr,-h/2+rr,w/2-rr,h/2-rr).buffer(rr,quad_segs=64)if rr else box(-w/2,-h/2,w/2,h/2)
 elif shape=='oval':g=LineString([(-(w-h)/2,0),((w-h)/2,0)]).buffer(h/2,quad_segs=64)if w>=h else LineString([(0,-(h-w)/2),(0,(h-w)/2)]).buffer(w/2,quad_segs=64)
 else:raise ValueError('Unsupported pad shape '+shape)
 return translate(rotate(g,-a,origin=(0,0)),cx,cy)
def collect(path):
 b=sx.loads(path.read_text());items=[];unsupported=[]
 def add(i,n,ls,g,t,bridge=None):
  for l in ls:
   if l in L:items.append({'id':i,'net':n,'layer':l,'geometry':g,'type':t,'bridge':bridge})
 for f in child(b,'footprint'):
  ref=prop(f)['Reference']
  for p in child(f,'pad'):
   if str(p[2])=='np_thru_hole':continue
   ls=get(p,'layers',[]);ls=L if '*.Cu'in ls else [l for l in ls if l in L]
   if not ls:continue
   try:g=pad_shape(f,p)
   except ValueError as e:unsupported.append({'ref':ref,'pad':p[1],'reason':str(e)});continue
   pid=str(get(p,'uuid',[ref+'.'+str(p[1])])[0]);bridge=pid if str(p[2])=='thru_hole'else None
   add(ref+'.'+str(p[1])+'@'+pid,get(p,'net',[0])[0],ls,g,'pad',bridge)
 for s in child(b,'segment'):add(get(s,'uuid')[0],get(s,'net')[0],get(s,'layer'),LineString([get(s,'start'),get(s,'end')]).buffer(get(s,'width')[0]/2,quad_segs=64),'segment')
 for v in child(b,'via'):
  ls=get(v,'layers');a,z=sorted([L.index(l)for l in ls]);uid=get(v,'uuid')[0];add(uid,get(v,'net')[0],L[a:z+1],Point(get(v,'at')[:2]).buffer(get(v,'size')[0]/2,quad_segs=64),'via',uid)
 for a in child(b,'arc'):unsupported.append({'uuid':get(a,'uuid'),'reason':'Copper arc not modeled'})
 return b,items,unsupported
class DSU:
 def __init__(self,n):self.p=list(range(n))
 def find(self,a):
  while self.p[a]!=a:self.p[a]=self.p[self.p[a]];a=self.p[a]
  return a
 def union(self,a,b):self.p[self.find(a)]=self.find(b)
def analyze(items):
 d=DSU(len(items));viol=[];bridges={}
 for i,it in enumerate(items):
  if it['bridge']:
   k=(it['bridge'],it['net'])
   if k in bridges:d.union(i,bridges[k])
   else:bridges[k]=i
 for layer in L:
  ids=[i for i,x in enumerate(items)if x['layer']==layer];gg=[items[i]['geometry']for i in ids];tree=STRtree(gg)
  for jj,g in enumerate(gg):
   i=ids[jj];x=items[i]
   for kk in tree.query(g.buffer(.150002)):
    if kk<=jj:continue
    j=ids[int(kk)];y=items[j];dist=g.distance(y['geometry'])
    if x['net']==y['net']:
     if dist<=.000002:d.union(i,j)
    elif dist<.149998:viol.append({'layer':layer,'a':x['id'],'b':y['id'],'net_a':x['net'],'net_b':y['net'],'gap_mm':dist,'overlap_mm2':g.intersection(y['geometry']).area})
 pads=collections.defaultdict(lambda:collections.defaultdict(set))
 for i,x in enumerate(items):
  if x['type']=='pad'and x['net']!=0:pads[x['net']][d.find(i)].add(x['id'])
 return viol,{str(n):[sorted(v)for v in groups.values()]for n,groups in pads.items()}
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--baseline',type=pathlib.Path,required=True);ap.add_argument('--candidate',type=pathlib.Path,required=True);ap.add_argument('--output',type=pathlib.Path,required=True);a=ap.parse_args()
 b,bi,bu=collect(a.baseline);c,ci,cu=collect(a.candidate);bv,bc=analyze(bi);cv,cc=analyze(ci)
 def vk(v):return(v['layer'],*sorted([v['a'],v['b']]))
 old={vk(v):v for v in bv};new=[v for v in cv if vk(v)not in old or v['gap_mm']<old[vk(v)]['gap_mm']-.000002 or v['overlap_mm2']>old[vk(v)]['overlap_mm2']+.000001]
 names={str(n[1]):n[2]for n in child(c,'net')};regr=[]
 for net,groups in bc.items():
  ng=cc.get(net,[])
  for group in groups:
   present=[p for p in group if any(p in g for g in ng)]
   if len(present)>1 and not any(set(present)<=set(g)for g in ng):regr.append({'net':names.get(net),'prior_pads':present})
 r={'baseline_sha256':hashlib.sha256(a.baseline.read_bytes()).hexdigest(),'candidate_sha256':hashlib.sha256(a.candidate.read_bytes()).hexdigest(),'scope':'Explicit pads/tracks/plated vias;64-quadrant shapes,2nm contact allowance; no unfilled zone or as-built credit','unsupported_baseline':bu,'unsupported_candidate':cu,'baseline_copper_items':len(bi),'candidate_copper_items':len(ci),'baseline_small_gaps':len(bv),'candidate_small_gaps':len(cv),'new_or_worsened_foreign_copper_gaps_below_0p15mm':new,'previous_explicit_pad_connectivity_regressions':regr,'candidate_explicit_pad_components':{names.get(n,n):v for n,v in cc.items()},'status':'PASS_SCOPED_SOURCE_SCREEN'if not(new or regr or cu)else'FINDINGS_REQUIRE_REVIEW','native_zone_refill':'NOT_RUN'}
 a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items()if k!='candidate_explicit_pad_components'}))
if __name__=='__main__':main()
