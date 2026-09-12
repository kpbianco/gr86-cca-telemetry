from pathlib import Path
import sys,json,math
sys.path.insert(0,str(Path(__file__).resolve().parent))
import build_candidate as m
c=m.c;LineString=m.LineString;U=m.unary_union
critical=m.ns['critical'];rows=[]
for name in ['OIL_5V','OIL_SIG','3V3_BULK_DAMPED']:
 net=next(k for k,v in m.names.items()if v==name)
 for seq,pts in m.chains_for(net,'In1.Cu'):
  if name=='OIL_SIG'and max(p[0]for p in pts)<20:continue
  width=max(c.get(s,'width')[0]for s in seq)
  layer='In2.Cu'
  foreign=U([i['geometry']for i in m.items if i['layer']==layer and i['net']!=net])
  projected=U([LineString([c.get(s,'start'),c.get(s,'end')]).buffer(c.get(s,'width')[0]/2+(.95 if m.names[c.get(s,'net')[0]]in ['GPS_EXT_ANT','GPS_ANT_RF_BIASED']else .05))for s in m.segs if c.get(s,'layer')[0]in ['B.Cu','In1.Cu']and m.names[c.get(s,'net')[0]]in critical])
  ob=U([foreign.buffer(width/2+.15002),m.holes.buffer(width/2+.25002),projected.buffer(width/2+.155),m.keepouts[layer].buffer(width/2+.01),m.ns['voltage_obstacles'](layer,width)])
  print('TRY',name,len(seq),pts[0],pts[-1],width,flush=True)
  path,diag=m.route(pts[0],pts[-1],ob,width)
  row=dict(net=name,original_uuids=[m.uid(s)for s in seq],original_path_mm=pts,width_mm=width,path_mm=path,diagnostics=diag)
  if path:row.update(old_length_mm=LineString(pts).length,new_length_mm=LineString(path).length)
  rows.append(row);(m.D/'INNER_TRIALS.json').write_text(json.dumps(rows,indent=2)+'\n');print(path,diag,flush=True)
