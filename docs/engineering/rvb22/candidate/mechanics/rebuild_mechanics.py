#!/usr/bin/env python3
"""Recovered source-bound ear/fuse/wing ECO. Native fill and manufacturing export remain required."""
from pathlib import Path
import json,math,hashlib,uuid,argparse,copy
import mechanics_runtime
import sexpdata as sx
from shapely.geometry import Polygon,Point,box
D=Path(__file__).resolve().parent
S=sx.Symbol
k=lambda x:str(x[0]) if isinstance(x,list) and x else ''
def kids(o,key):return [x for x in o if k(x)==key]
def one(o,key):return kids(o,key)[0]
def uid(s):return str(uuid.uuid5(uuid.NAMESPACE_URL,'GR86-RVB22-MECHANICS/'+s))
def bottom(x):return 41.288863+(x-2.557101)*(41.254763-41.288863)/(84.927437-2.557101)
centres=[[6.2,-4],[63,-4],[6.2,bottom(6.2)+4],[63,bottom(63)+4]]
def ear(x,upper):
 y=0 if upper else bottom(x);sgn=-1 if upper else 1
 return [[x-3.5,0 if upper else bottom(x-3.5)],[x-3.5,y+sgn*6.5],[x-3,y+sgn*7],[x+3,y+sgn*7],[x+3.5,y+sgn*6.5],[x+3.5,0 if upper else bottom(x+3.5)]]
def wing(upper):
 y=bottom(34)
 return [[13,0],[13,-9.5],[13.5,-10],[54.5,-10],[55,-9.5],[55,0]] if upper else [[13,bottom(13)],[13,y+9.5],[13.5,y+10],[54.5,y+10],[55,y+9.5],[55,bottom(55)]]
def line(a,b,name):return sx.loads(f'(gr_line (start {a[0]:.9f} {a[1]:.9f}) (end {b[0]:.9f} {b[1]:.9f}) (stroke (width .05) (type default)) (layer "Edge.Cuts") (uuid "{uid(name)}"))')
def edge(o):return k(o) in ['gr_line','gr_arc'] and one(o,'layer')[1]=='Edge.Cuts'
def arc_points(o):
 a,b,c=[one(o,key)[1:] for key in ['start','mid','end']];x1,y1=a;x2,y2=b;x3,y3=c
 den=2*(x1*(y2-y3)+x2*(y3-y1)+x3*(y1-y2));assert abs(den)>1e-15
 ux=((x1*x1+y1*y1)*(y2-y3)+(x2*x2+y2*y2)*(y3-y1)+(x3*x3+y3*y3)*(y1-y2))/den
 uy=((x1*x1+y1*y1)*(x3-x2)+(x2*x2+y2*y2)*(x1-x3)+(x3*x3+y3*y3)*(x2-x1))/den
 aa,bb,cc=[math.atan2(y-uy,x-ux) for x,y in [a,b,c]];ccw=(cc-aa)%(2*math.pi);mid=(bb-aa)%(2*math.pi);delta=ccw if mid<=ccw else ccw-2*math.pi;n=max(2,math.ceil(abs(delta)/.04));r=math.hypot(x1-ux,y1-uy)
 return [a]+[[ux+r*math.cos(aa+delta*i/n),uy+r*math.sin(aa+delta*i/n)] for i in range(1,n)]+[c]
def outline(r):
 paths=[arc_points(o) if k(o)=='gr_arc' else [one(o,'start')[1:],one(o,'end')[1:]] for o in r if edge(o)];ordered=paths.pop(0)
 while paths:
  found=False
  for i,p in enumerate(paths):
   if math.dist(ordered[-1],p[0])<2e-7:ordered.extend(p[1:]);paths.pop(i);found=True;break
   if math.dist(ordered[-1],p[-1])<2e-7:ordered.extend(list(reversed(p))[1:]);paths.pop(i);found=True;break
  assert found,'Edge.Cuts not connected'
 assert math.dist(ordered[0],ordered[-1])<2e-7
 ordered[-1]=ordered[0];p=Polygon(ordered);assert p.is_valid and p.minimum_clearance>1e-6
 return p

def do_ears(r):
 assert not any(k(o)=='footprint' and any(p[1:3]==['Reference','MH901'] for p in kids(o,'property')) for o in r),'Ears already present'
 changes=0
 for upper,key in [(True,'c3b829c0-8648-5bc6-a9ad-133285b5c869'),(False,'680aaebe-65f1-5c73-a026-6259bbc1c0a8')]:
  old=next(o for o in r if edge(o) and one(o,'uuid')[1]==key);st=one(old,'start')[1:];en=one(old,'end')[1:];pts=[st]+(ear(6.2,True)+ear(63,True) if upper else list(reversed(ear(63,False)))+list(reversed(ear(6.2,False))))+[en];i=r.index(old);r[i:i+1]=[line(a,b,f'EAR{upper}{n}') for n,(a,b) in enumerate(zip(pts,pts[1:]))];changes+=1
 for n,((x,y),points) in enumerate(zip(centres,[ear(6.2,True),ear(63,True),ear(6.2,False),ear(63,False)]),901):
  r.append(sx.loads(f'''(footprint "RevB:MountingHole_4.4mm_Outboard" (layer "F.Cu") (uuid "{uid(str(n))}") (at {x:.9f} {y:.9f})
   (property "Reference" "MH{n}" (at 0 0) (layer "F.Fab") (hide yes) (effects (font (size 1 1) (thickness .15))))
   (property "Value" "4.4mm NPTH" (at 0 0) (layer "F.Fab") (hide yes) (effects (font (size 1 1) (thickness .15))))
   (attr through_hole board_only exclude_from_pos_files exclude_from_bom)
   (fp_circle (center 0 0) (end 3 0) (stroke (width .1) (type default)) (fill none) (layer "F.CrtYd"))
   (pad "" np_thru_hole circle (at 0 0) (size 4.4 4.4) (drill 4.4) (layers "*.Cu" "*.Mask")))'''))
  p=Polygon(points);coords=' '.join(f'(xy {a:.9f} {b:.9f})' for a,b in p.exterior.coords)
  r.append(sx.loads(f'''(zone (net 0) (net_name "") (layers "F.Cu" "In1.Cu" "In2.Cu" "B.Cu") (uuid "{uid('KEEP'+str(n))}") (name "RVB22_EAR_NO_COPPER") (hatch edge .5) (connect_pads (clearance 0)) (min_thickness .2) (keepout (tracks not_allowed) (vias not_allowed) (pads not_allowed) (copperpour not_allowed) (footprints allowed)) (polygon (pts {coords})))'''))
 return changes

def do_fuse(r):
 f=next(f for f in kids(r,'footprint') if any(p[1:3]==['Reference','F101'] for p in kids(f,'property')));props={p[1]:p[2] for p in kids(f,'property')};assert props['MPN']=='0885001.DR';assert 'Assembly_Process' not in props,'Fuse already modified'
 for pad in kids(f,'pad'):
  ls=one(pad,'layers');assert 'F.Paste' in ls;ls.remove('F.Paste')
 attr=one(f,'attr');attr.append(S('exclude_from_pos_files'))
 for name,val in [('Assembly_Process','FACTORY_LOCAL_AFTER_REFLOW'),('Factory_Paste','NO')]:f.append(sx.loads(f'(property "{name}" "{val}" (at 0 0) (layer "F.Fab") (hide yes) (effects (font (size 1 1) (thickness .15))))'))
 return f

def do_wings(r):
 count=0
 for upper in [True,False]:
  target=[9.7,0,59.5,0] if upper else [59.5,bottom(59.5),9.7,bottom(9.7)]
  old=next(o for o in r if k(o)=='gr_line' and edge(o) and all(abs(i-j)<2e-7 for i,j in zip(one(o,'start')[1:]+one(o,'end')[1:],target)))
  pts=[one(old,'start')[1:]]+(wing(True) if upper else list(reversed(wing(False))))+[one(old,'end')[1:]];i=r.index(old);r[i:i+1]=[line(a,b,f'WING{upper}{n}') for n,(a,b) in enumerate(zip(pts,pts[1:]))]
  ylo=-9.75 if upper else bottom(54.75)-.75;yhi=.75 if upper else bottom(34)+9.75
  for layer in ['F.Cu','In1.Cu','In2.Cu','B.Cu']:
   r.append(sx.loads(f'''(zone (net 30) (net_name "GND") (layer "{layer}") (uuid "{uid('WING'+str(upper)+layer)}") (name "RVB22_THERMAL_WING_{'TOP' if upper else 'BOTTOM'}") (hatch edge .5) (connect_pads yes (clearance .2)) (min_thickness .2) (fill yes (thermal_gap .25) (thermal_bridge_width .4)) (polygon (pts (xy 13.25 {ylo:.9f}) (xy 54.75 {ylo:.9f}) (xy 54.75 {yhi:.9f}) (xy 13.25 {yhi:.9f}))))'''));count+=1
 return count

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--project-root',type=Path);ap.add_argument('--input',required=True,type=Path);ap.add_argument('--output',required=True,type=Path);ap.add_argument('--stage',choices=['all','ears','fuse','wings'],default='all');a=ap.parse_args();src=a.input.read_text();r=sx.loads(src);before=copy.deepcopy(r);old=outline(r);earshape=None
 if a.stage in ['all','ears']:do_ears(r);earshape=outline(r);(D/'EAR_GEOMETRY.json').write_text(json.dumps({'centres_mm':centres,'complete_polygon_for_visualization_mm':list(earshape.exterior.coords),'ear_polygons_mm':[ear(6.2,True),ear(63,True),ear(6.2,False),ear(63,False)]},indent=2)+'\n')
 if a.stage in ['all','fuse']:do_fuse(r)
 if a.stage in ['all','wings']:do_wings(r)
 for z in kids(r,'zone'):z[:]=[o for o in z if k(o) not in ['filled_polygon','fill_segments']]
 final=outline(r);assert old.difference(final).area<1e-6
 for key in ['segment','arc','via']:assert kids(before,key)==kids(r,key),key
 oldrefs={next(p[2] for p in kids(f,'property') if p[1]=='Reference'):f for f in kids(before,'footprint')};newrefs={next(p[2] for p in kids(f,'property') if p[1]=='Reference'):f for f in kids(r,'footprint')}
 for ref,f in oldrefs.items():
  if ref!='F101':assert f==newrefs[ref],ref
  else:
   for p,q in zip(kids(f,'pad'),kids(newrefs[ref],'pad')):
    pp=copy.deepcopy(p);qq=copy.deepcopy(q)
    for zz in [pp,qq]:one(zz,'layers')[:]=[x for x in one(zz,'layers') if x!='F.Paste']
    assert pp==qq
 out=sx.dumps(r)+'\n';a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(out);sx.loads(out)
 manifest={'reconstructed_after_shared_workspace_loss':True,'stage':a.stage,'input_sha256':hashlib.sha256(src.encode()).hexdigest(),'output_sha256':hashlib.sha256(out.encode()).hexdigest(),'output_file':str(a.output),'all_original_tracks_arcs_vias_preserved':True,'all_original_nonF101_footprints_preserved':True,'F101_only_paste_metadata_changed':True,'native_fill_required':True,'old_area_preserved_mm2_error':old.difference(final).area,'board_bounds_mm':list(final.bounds),'board_size_mm':[final.bounds[2]-final.bounds[0],final.bounds[3]-final.bounds[1]]}
 (D/'CAD_PATCH_MANIFEST.json').write_text(json.dumps(manifest,indent=2)+'\n')
 if a.stage in ['all','wings']:
  G={'effectivity':manifest,'outline_mm':list(final.exterior.coords),'wing_polygons_mm':[wing(True),wing(False)],'mount_centres_mm':centres,'board_bounds_mm':list(final.bounds),'board_size_mm':manifest['board_size_mm'],'wing_area_total_mm2':sum(Polygon(wing(v)).area for v in [True,False]),'panel120x75_halfmargin_mm':[(120-manifest['board_size_mm'][0])/2,(75-manifest['board_size_mm'][1])/2],'checks':{'original_area_preserved':old.difference(final).area<1e-6,'simple_outline':final.is_valid and final.minimum_clearance>1e-6,'wings_outside_ESP_exclusion':55<68.029766,'mounts_outside_wing_copper':all(Point(*p).distance(Polygon(wing(u)))>2.225 for p in centres for u in [True,False]),'original_electrical_geometry_preserved':True,'panel_bbox_fits':all(x<y for x,y in zip(manifest['board_size_mm'],[120,75]))}}
  (D/'THERMAL_WING_GEOMETRY.json').write_text(json.dumps(G,indent=2)+'\n')
 (D/'MECH_GEOMETRY.scad').write_text('mounts='+json.dumps(centres)+';\noutline='+json.dumps(list(final.exterior.coords))+';\n')
 L=D/'libraries/RevB.pretty';L.mkdir(parents=True,exist_ok=True)
 for ref,name in [('MH901','MountingHole_4.4mm_Outboard'),('F101','Fuse_Littelfuse-NANO2-885')]:
  if ref not in newrefs:continue
  f=copy.deepcopy(newrefs[ref]);f[1]=name;f[:]=[o for o in f if k(o) not in ['at','uuid','path','sheetname','sheetfile']]
  for p in kids(f,'property'):
   if p[1]=='Reference':p[2]='REF**'
  for p in kids(f,'pad'):p[:]=[o for o in p if k(o) not in ['net','pinfunction','pintype','uuid']]
  (L/(name+'.kicad_mod')).write_text(sx.dumps(f)+'\n')
 print(json.dumps(manifest))
if __name__=='__main__':main()
