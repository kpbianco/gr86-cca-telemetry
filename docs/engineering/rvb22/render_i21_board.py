from pathlib import Path
import sys,json,math
import argparse
p=argparse.ArgumentParser();p.add_argument('recovery',type=Path);args=p.parse_args()
W=args.recovery;D=W/'current'
sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor'),str(W/'control')]
from shapely import wkt
from shapely.geometry import Polygon
from shapely.affinity import rotate,translate
import check_combined_copper as c
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as Patch
b,items,_=c.collect(W/'iterations/I20_controlled_handoff/candidate_kicad/GR86_CCA_RevB.kicad_pcb')
outline=Polygon(json.loads((W/'iterations/I18_capacitor_thermal_tab/CURRENT_OUTLINE_GEOMETRY.json').read_text())['outline_mm'])
logo=wkt.loads((W/'iterations/I12_branding_models_thermal/LOGO_GEOMETRY.wkt').read_text())
def draw(ax,g,color,z=1):
 for q in g.geoms if hasattr(g,'geoms') else [g]:
  if q.geom_type!='Polygon':continue
  ax.add_patch(Patch(list(q.exterior.coords),facecolor=color,edgecolor='none',zorder=z))
  for h in q.interiors:ax.add_patch(Patch(list(h.coords),facecolor='#153e35',edgecolor='none',zorder=z+.01))
fig,(ax,zoom)=plt.subplots(1,2,figsize=(13,6),gridspec_kw={'width_ratios':[1.3,1]})
for a in [ax,zoom]:
 a.set_facecolor('#f4f5f3');draw(a,outline,'#153e35')
 for it in items:
  if it['layer']=='F.Cu':draw(a,it['geometry'],'#406456',2)
 for f in c.child(b,'footprint'):
  for p in c.child(f,'pad'):
   if str(p[2])=='np_thru_hole':
    draw(a,c.pad_shape(f,p),'#f4f5f3',3)
   elif 'F.Mask' in c.get(p,'layers',[]) or '*.Mask' in c.get(p,'layers',[]):
    draw(a,c.pad_shape(f,p),'#c2af76',3)
    dr=c.get(p,'drill')
    if dr and isinstance(dr[0],(int,float)):
     from shapely.geometry import Point
     draw(a,Point(c.pad_shape(f,p).centroid).buffer(dr[0]/2),'#f4f5f3',3.2)
 # Plot actual footprint and board silk line geometry. Text is left to native plots.
 for f in c.child(b,'footprint'):
  at=c.get(f,'at');angle=at[2] if len(at)>2 else 0
  for o in c.child(f,'fp_line'):
   if c.get(o,'layer')!=['F.SilkS']:continue
   from shapely.geometry import LineString
   g=translate(rotate(LineString([c.get(o,'start'),c.get(o,'end')]),-angle,origin=(0,0)),*at[:2])
   a.plot(*g.xy,color='#edf3ef',lw=.6,zorder=4)
 draw(a,logo,'#edf3ef',5);a.set_aspect('equal');a.set_xlabel('Board X (mm)');a.set_ylabel('Board Y (mm)')
ax.set(xlim=(-3,97),ylim=(54,-13),title='I20 source PCB with Compact TW logo')
zoom.set(xlim=(27,41),ylim=(1,-11),title='8 mm compact TW — artwork detail')
fig.text(.05,.025,'Approved TranquilWorks vector paths converted to one-color F.SilkS. Source geometry preview; matched native legend and zero DRC verified.\nCopper shown schematically from explicit source objects; components and solder mask are not a photorealistic assembly view.',fontsize=9)
fig.tight_layout(rect=(0,.09,1,1));fig.savefig(D/'GR86_I20_TranquilWorks_PCB.png',dpi=200)
