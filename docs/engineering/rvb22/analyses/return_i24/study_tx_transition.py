"""Unadopted TX transition feasibility; no candidate board mutation."""
from pathlib import Path
import sys,json,math
sys.path.insert(0,str(Path(__file__).resolve().parent));import build_candidate as m
c=m.c;P=m.Point;L=m.LineString;U=m.unary_union;net=26;width=.2;rows=[]
# Signal via diameter/drill follows existing0.60/0.30mm controlled through-vias.
foreign={l:U([i['geometry']for i in m.items if i['layer']==l and i['net']!=net])for l in c.L}
allforeign=U(list(foreign.values()));holem=m.holes.buffer(.3+.25)
protected={'PROTECTED_12V','PROTECT_CTRL_VIN','INPUT_GATE','INPUT_GATE_DRIVE','INPUT_GATE_SLEW','BUCK_FEED','5V_VIN','5V_SW','5V_BST'};raw={'RAW_12V','FUSED_12V','REV_BLOCKED_12V'}
vp=U([i['geometry']for i in m.items if m.names[i['net']]in protected]);vr=U([i['geometry']for i in m.items if m.names[i['net']]in raw])
# Same-name source footprint zones may separately forbid vias; honor them too.
via_ko=[]
for parent in [m.b]+c.child(m.b,'footprint'):
 for z in c.child(parent,'zone'):
  ko=c.child(z,'keepout')
  if not ko or str((c.get(ko[0],'vias')or [''])[0])!='not_allowed':continue
  for poly in c.child(z,'polygon'):via_ko.append(m.Polygon([p[1:]for p in c.child(c.child(poly,'pts')[0],'xy')]))
via_ko=U(via_ko)
for x in [80.4+i*.1 for i in range(22)]:
 for y in [15.3+i*.1 for i in range(17)]:
  point=P(x,y);g=point.buffer(.3)
  if any(g.distance(v)<.15002 for v in foreign.values())or g.distance(vp)<.25002 or g.distance(vr)<.60002 or point.intersects(holem)or g.intersects(via_ko):continue
  rows.append({'at':[round(x,5),round(y,5)],'min_foreign':min(g.distance(v)for v in foreign.values()),'pad_center_distance':math.dist((x,y),(81.684766,14.25))})
rows.sort(key=lambda r:r['pad_center_distance'])
(m.D/'TX_TRANSITION_VIA_FEASIBILITY.json').write_text(json.dumps(rows,indent=2)+'\n')
print(json.dumps(rows[:20],indent=2))
