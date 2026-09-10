"""Current full-stack populated clearance and fabrication-hole review."""
from pathlib import Path
import sys,json,hashlib,math,collections
D=Path(__file__).resolve().parent;W=D.parents[1]
sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor'),str(W/'control')]
import check_combined_copper as c
from shapely.geometry import box,Polygon,Point
from shapely import from_wkt
from shapely.ops import unary_union
N=W/'runtime/hosted/run22/extracted/native_I06_hosted'
P=N/'candidate_kicad/GR86_CCA_RevB.kicad_pcb'
b,items,u=c.collect(P);assert not u
M=W/'iterations/I15_service_mechanics/candidate_mechanics'
G=json.loads((M/'COOLING_GEOMETRY.json').read_text())
rows=json.loads((W/'analyses/mated_i22/RESULTS.json').read_text())['body_rows']
stack=dict(solder=.25,capture_freedom=.51,warp=.75,local_deflection=.20)
allowance=sum(stack.values());assert abs(allowance-1.71)<1e-12
base=from_wkt(G['carrier_base_WKT']).buffer(.15)
supports=unary_union([from_wkt(x['WKT']).buffer(.15)for x in json.loads((M/'C05_SUPPORT_RELIEF.json').read_text())['support_plans']])
obstacles=[('C05_base',base,-9,-6),('C05_supports',supports,-6,0)]
for x in json.loads((M/'LANDING_CLEARANCE.json').read_text())['landings']:
    obstacles.append((x['name'],box(*x['bounds_xy_mm']),*x['Z_mm']))
checks=[]
def prism(ref,name,xy,z,og,oz):
    zgap=max(0,oz[0]-z[1],z[0]-oz[1])
    overlap=xy.intersection(og).area*max(0,min(z[1],oz[1])-max(z[0],oz[0]))
    checks.append(dict(ref=ref,obstacle=name,overlap_mm3=overlap,clearance_mm=math.hypot(xy.distance(og),zgap)))
for r in rows:
    ref=r['ref'];x0,y0,x1,y1=r['courtyard_plus0p3_bounds_mm'];h=r['height_max_mm']+allowance
    if r['side']=='F.Cu':
        checks.append(dict(ref=ref,obstacle='16mm_front_component_reservation',overlap_mm3=max(0,h-16)*(x1-x0)*(y1-y0),clearance_mm=16-h))
        continue
    xy=box(x0,y0,x1,y1);z=(-h,0)
    for name,og,a,z1 in obstacles:prism(ref,name,xy,z,og,(a,z1))
    # Cross-section extrusion is exact for the authored foil solids. The .8mm
    # buffer includes .5mm lateral bow and .3mm positioning. Current polygons
    # already use the2.2mm maximum200-leaf packet; no minimum-thickness credit.
    og=Polygon(G['direct_T02_foil_xz_mm']).buffer(.8,join_style=2)
    prism(ref,'T03_full_packet_plus_bow',box(x0,-h,x1,0),(y0,y1),og,(12.8-.8,27.9+.8))
    for name,key in [('W02_upper','upper_foil_yz_mm'),('W02_lower','lower_foil_yz_mm')]:
        og=Polygon(G[key]).buffer(.8,join_style=2)
        prism(ref,name,box(y0,-h,y1,0),(x0,x1),og,(15-.8,53+.8))
hits=[x for x in checks if x['overlap_mm3']>1e-7]
report=dict(status='PASS_DECLARED_FULL_STACK_CLEARANCE'if not hits else'FINDINGS',PCB_sha256=hashlib.sha256(P.read_bytes()).hexdigest(),
  total_fitted_references=len(rows),bottom_references=sum(r['side']=='B.Cu'for r in rows),
  height_stack_mm=stack,full_extra_height_mm=allowance,checks_count=len(checks),interferences=hits,
  nearest=sorted(checks,key=lambda x:x['clearance_mm'])[:20],all_checks=checks,
  scope='Design clearance for complete declared maximum component/lead XY+0.3mm and manufacturer maximum heights. Full1.71mm solder/capture/warp/deflection stack; actual installed fit and supplier material/process acceptance remain separate.',
  physical_measurement_claimed=False)
(D/'POPULATED_CLEARANCE_RESULTS.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:report[k]for k in ['status','total_fitted_references','bottom_references','checks_count','interferences','nearest']},indent=2))

# Independent drill table from actual filled source; native Excellon equality
# was already established by a separate Gerbonara parser in I20/I21.
via=[];pins=[];npth=[]
for v in c.child(b,'via'):
    dr=c.get(v,'drill')[0];dia=c.get(v,'size')[0]
    via.append(dict(xy_mm=c.get(v,'at')[:2],finished_drill_mm=dr,land_diameter_mm=dia,nominal_annular_ring_mm=(dia-dr)/2,net=c.get(v,'net')[0]))
for f in c.child(b,'footprint'):
    for p in c.child(f,'pad'):
        typ=str(p[2])
        if typ not in ['thru_hole','np_thru_hole']:continue
        dr=c.get(p,'drill')[0];g=c.pad_shape(f,p);xy=list(g.centroid.coords)[0]
        row=dict(ref=c.prop(f)['Reference'],pin=str(p[1]),finished_drill_mm=dr,xy_mm=xy,land_mm=c.get(p,'size'))
        (pins if typ=='thru_hole'else npth).append(row)
drill_counts=collections.Counter(x['finished_drill_mm']for x in via+pins)
holes=dict(status='PASS_SOURCE_AND_MATCHED_DRILL_RECONCILIATION',source_PCB_sha256=report['PCB_sha256'],
  plated_count=len(via)+len(pins),NPTH_count=len(npth),plated_size_counts_mm=dict(sorted(drill_counts.items())),
  minimum_nominal_via_annular_ring_mm=min(x['nominal_annular_ring_mm']for x in via),minimum_finished_via_drill_mm=min(x['finished_drill_mm']for x in via),
  source_stackup_thickness_mm=sum(c.get(x,'thickness',[0])[0]for x in c.child(c.child(c.child(b,'setup')[0],'stackup')[0],'layer')),
  board_thickness_acceptance_mm=[1.44,1.76],maximum_nominal_finished_via_aspect_ratio=1.76/min(x['finished_drill_mm']for x in via),
  maximum_tolerance_bound_finished_via_aspect_ratio=1.76/(min(x['finished_drill_mm']for x in via)-.08),
  required_minimum_plated_wall_um=15,published_PTH_diameter_tolerance_mm=[-.08,.13],NPTH_diameter_tolerance_mm=None,
  tolerance_source='https://jlcpcb.com/capabilities/pcb-capabilities, accessed2026-09-10. Public through-hole tolerance is +0.13/-0.08mm; this is not supplier acceptance of this order. No unsupported NPTH tolerance is assigned.',
  via_design_note='0.15mm nominal ring is a design dimension. Finished hole position/diameter and etch must be evaluated by supplier; no process guarantee is inferred.',
  PTH_pins=pins,NPTH=npth,via_rows=via,
  native_export_evidence='analyses/manufacturing_i20/RESULTS.json',
  source_export_scope='Nominal CAD/export reconciliation only. Exact finished-hole/pin fit and drawing/DFM agreement remain DFM02. The aspect ratios here use finished-hole diameters, not unplated drill-tool diameters. Supplier approval, finished-board measurement, annular-ring registration and via filling remain DFM01/04/07/VIA08 gates.',
  physical_measurement_claimed=False)
assert holes['plated_count']==338 and holes['NPTH_count']==10
(D/'DRILL_RECONCILIATION.json').write_text(json.dumps(holes,indent=2)+'\n')
print(json.dumps({k:v for k,v in holes.items()if k not in ['via_rows','PTH_pins','NPTH']},indent=2))
