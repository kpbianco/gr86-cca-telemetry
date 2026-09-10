"""Current C03 carrier plan geometry for source-fit and thermal-shadow checks."""
from mechanics_runtime import *
from pathlib import Path
import json,math
from shapely.geometry import box,Point,LineString
from shapely.ops import unary_union
D=Path(__file__).resolve().parent
anchor_centres=[(-4,31),(-4,37),(62.5,33),(62.5,39)]
def obround(x,y,L,H):return LineString([(x-(L-H)/2,y),(x+(L-H)/2,y)]).buffer(H/2,quad_segs=128)
def carrier_plan(previous=False,cut_slots=True):
 body=unary_union([box(-7,-15,66.5,58),box(-13,-12,-5,-1),box(-13,42.5,-5,55)])
 if not previous:body=body.union(box(-8,28,0,40))
 windows=[box(2,4,57.5,37).buffer(2,quad_segs=128),box(14.5,-8.5,53.5,-1.5).buffer(1,quad_segs=128),box(14.5,42.8,53.5,49.8).buffer(1,quad_segs=128)]
 if not previous:windows[0]=windows[0].difference(box(58.5,30,66.5,42))
 for w in windows:body=body.difference(w)
 g=json.loads((D/'THERMAL_WING_GEOMETRY.json').read_text())
 for x,y in g['mount_centres_mm']:body=body.difference(Point(x,y).buffer(1.1,quad_segs=128))
 for x,y in [(-9,-6),(-9,47.5),(57,-11.5),(57,54.5)]:body=body.difference(Point(x,y).buffer(1.7,quad_segs=128))
 if cut_slots:
  for x,y in [(-3.5,31),(-3.5,37),(63,33),(63,39)] if previous else anchor_centres:body=body.difference(obround(x,y,3 if previous else 3.9,1.5 if previous else 1.6))
 return body
