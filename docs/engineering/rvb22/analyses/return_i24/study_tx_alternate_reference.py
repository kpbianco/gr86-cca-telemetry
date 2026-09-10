"""Study removal of I23's LED-created simultaneous inner-plane gap."""
from pathlib import Path
import sys,json
sys.path.insert(0,str(Path(__file__).resolve().parent));import build_candidate as m
from shapely import from_wkt
c=m.c;U=m.unary_union;L=m.LineString
_,m.items,_=c.collect(m.W/'iterations/I24_return_clearance/candidate_kicad/GR86_CCA_RevB.kicad_pcb')
net=next(k for k,v in m.names.items()if v=='LED_CAN');rows=[]
assert m.names[net]=='LED_CAN'
missing=json.loads((m.W/'analyses/review_i23/RETURN_AND_RF_REVIEW.json').read_text())['remaining_segments']
# Where nearest In1 is interrupted, do not cut the alternate In2 copper too.
# This does not equate alternate copper presence with a proved return transfer.
fallback=U([from_wkt(r['adjacent_gap_WKT']).buffer(.15)for r in missing if r['net']=='CAN_TX_MCU'])
projected=U([L([c.get(s,'start'),c.get(s,'end')]).buffer(c.get(s,'width')[0]/2+(.95 if m.names[c.get(s,'net')[0]]in ['GPS_EXT_ANT','GPS_ANT_RF_BIASED']else .05))for s in m.segs if c.get(s,'layer')[0]in ['B.Cu','In1.Cu']and m.names[c.get(s,'net')[0]]in m.ns['critical']]+[fallback])
for seq,pts in m.chains_for(net,'In2.Cu'):
 if not L(pts).buffer(.26).intersects(fallback):continue
 width=c.get(seq[0],'width')[0];foreign=U([i['geometry']for i in m.items if i['layer']=='In2.Cu'and i['net']!=net]);ob=U([foreign.buffer(width/2+.15002),m.holes.buffer(width/2+.25002),projected.buffer(width/2+.155),m.keepouts['In2.Cu'].buffer(width/2+.01),m.ns['voltage_obstacles']('In2.Cu',width)])
 print('TRY',pts[0],pts[-1],len(seq),flush=True);path,diag=m.route(pts[0],pts[-1],ob,width)
 row={'net':'LED_CAN','original_uuids':[m.uid(s)for s in seq],'original_path_mm':pts,'width_mm':width,'path_mm':path,'diagnostics':diag,'original_length_mm':L(pts).length,'new_length_mm':L(path).length if path else None};rows.append(row);print(path,diag,flush=True)
(m.D/'TX_ALTERNATE_REFERENCE_TRIAL.json').write_text(json.dumps(rows,indent=2)+'\n')
