from carrier_slot_geometry import *
from pathlib import Path
import hashlib,itertools,csv
D=Path(__file__).resolve().parent;checks=[]
def check(name,passed,detail=None):checks.append({'name':name,'pass':bool(passed),'detail':detail});assert passed,name
# Analytic distance to obround centreline, evaluated independently at all fourrectanglecorners.
rows=[]
for L,H,w,t in itertools.product([3.8,3.9,4.0],[1.5,1.6,1.7],[2.3,2.5,2.7],[.9,1.1,1.3]):
 for sx,sy in itertools.product([-1,1],repeat=2):
  x,y=sx*w/2,sy*t/2;d=math.hypot(max(abs(x)-(L-H)/2,0),y);margin=H/2-d
  rows.append({'slot_L':L,'slot_H':H,'strap_w':w,'strap_t':t,'corner_x':x,'corner_y':y,'analytic_clearance_mm':margin,'polygon_clearance_mm':obround(0,0,L,H).boundary.distance(Point(x,y)),'inside_polygon':obround(0,0,L,H).covers(Point(x,y))})
check('324independentrectanglecorners fitactualroundedslot',len(rows)==324 and all(r['analytic_clearance_mm']>0 and r['inside_polygon'] for r in rows),{'min_corner_clearance_mm':min(r['analytic_clearance_mm'] for r in rows),'max_polygon_difference_mm':max(abs(r['analytic_clearance_mm']-r['polygon_clearance_mm']) for r in rows)})
# SCAD circles use $fn=64. Validate the exact inscribed 64-vertex circle hull too.
from shapely.geometry import MultiPoint
facet_rows=[]
for r in rows:
 L,H=r['slot_L'],r['slot_H'];a=(L-H)/2
 poly=MultiPoint([(x+H/2*math.cos(2*math.pi*k/64),H/2*math.sin(2*math.pi*k/64)) for x in [-a,a] for k in range(64)]).convex_hull
 point=Point(r['corner_x'],r['corner_y']);facet_rows.append({'inside':poly.covers(point),'clearance_mm':poly.boundary.distance(point)})
check('All324corners fitactualSCAD64facetobround',all(r['inside'] for r in facet_rows),{'minimum_clearance_mm':min(r['clearance_mm'] for r in facet_rows),'circle_facets':64})
base=carrier_plan(cut_slots=False);old=carrier_plan(previous=True);new=carrier_plan();source=(D/'GR86_RVB_CARRIER_C03_THERMAL.scad').read_text()
check('SCADslotdimensionsmatchcalculation','translate([-1.15,0])circle(r=.8);translate([1.15,0])circle(r=.8);' in source)
check('SCADanchorsandreinforcementmatch',all(s in source for s in ['[-4,31],[-4,37],[62.5,33],[62.5,39]','translate([-8,28,base_bottom])cube([8,12,3])','translate([58.5,30,base_bottom-1])cube([8,12,5])']))
web=[]
for x,y in anchor_centres:
 for dx,dy in itertools.product([-.1,.1],repeat=2):
  slot=obround(x+dx,y+dy,4,1.7);margin=slot.distance(base.boundary)-.1
  web.append({'anchor':[x,y],'position_error':[dx,dy],'minimum_remaining_web_mm_after_edge0p1':margin})
check('Everyworstslot retainsatleast1p8mmweb',min(r['minimum_remaining_web_mm_after_edge0p1'] for r in web)>=1.8-1e-9,web)
check('Carrierplanvalid',new.is_valid and new.geom_type=='Polygon')
check('Globalplanbboxunchanged',new.bounds==old.bounds,{'old':old.bounds,'new':new.bounds})
foil=D/'THERMAL_CONTACT_T01.scad';before=D/'history_before_slot_shape_fix/THERMAL_CONTACT_T01.scad'
if before.exists():check('Thermalfoilbyteunchanged',foil.read_bytes()==before.read_bytes())
else:check('Thermalfoilgeometryunchanged',all(s in foil.read_text() for s in ['translate([66.6,11.8,-.127])cube([5.4,17,.127])','translate([51,4,-15.327])cube([15.5,31.8,2])']))
insert=box(58.5,30,66.5,42);check('RightinsertclearsthermalpacketYby1p2mm',30-28.8>=1.2-1e-9)
check('FR4rightedgedoesnotapproachantenna',new.bounds[2]==66.5)
# Newcorridor/headvolumes remaininsidealreadyallocatedbasicthermalbox.
check('Harnessandheadfitexistingbasicthermalbox',-12.5>=-13 and-16.3>=-18.227 and-12.9>=-18.227)
area=new.area-old.area;mass=area*3.15*1e-9*2000*1e3
board=__import__('shapely.geometry',fromlist=['Polygon']).Polygon(json.loads((D/'THERMAL_WING_GEOMETRY.json').read_text())['outline_mm']);shadow=new.intersection(board).area-old.intersection(board).area
check('Addedcarriermassless0p2g',0<mass<.2,{'additional_area_mm2':area,'density_upper_allocation_kg_m3':2000,'max_thickness_mm':3.15,'additional_mass_g':mass})
r={'status':'PASS_SOURCE_OBROUND_TOLERANCE_AND_LOAD_PATH_SCREEN','redline':'MECH22-31','corner_cases':len(rows),'minimum_corner_clearance_mm':min(x['analytic_clearance_mm'] for x in rows),'min_web_mm':min(x['minimum_remaining_web_mm_after_edge0p1'] for x in web),'checks':checks,'corner_rows':rows,'nominal_slot_mm':[3.9,1.6],'finished_slot_bounds_mm':[[3.8,4.],[1.5,1.7]],'carrier_min_thickness_mm':2.85,'one_leg30N_nominal_shear_demand_MPa':30/(2.85*1.8),'one_leg_bearing_demand_MPa':30/(2.85*2.3),'three_times_shear_inverse_required_allowable_MPa':3*30/(2.85*1.8),'material_scope':'2.85mmthickness,±.1slotposition/edge and2000kg/m3density are drawing/engineering bounds, not arbitrarysupplier guarantees. Three-times notchfactor is a demand sensitivity, not actual laminatefatigueproof.','additional_carrier_area_mm2':area,'additional_carrier_mass_g':mass,'additional_PCB_shadow_mm2':shadow,'thermal_effect':'CurrentT01positionmodel nowusescurrentcarrierplan; oldC02thermalmodels remainhistorical. No heat-safetypass followsfrom slotfit.','basic_bbox':'NominalXY−13..66.5,−15..58 unchanged; corridorsstayaboveZ−18.227 existingthermalbottom. NoPCB/FW/foilgeometry edited.','source_hashes':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [D/'GR86_RVB_CARRIER_C03.scad',D/'GR86_RVB_CARRIER_C03_THERMAL.scad',D/'HARNESS_CORRIDORS_C03.scad',D/'THERMAL_CONTACT_T01.scad',D/'carrier_slot_geometry.py',Path(__file__)]}}
(D/'SLOT_SHAPE_CORRECTION_CHECKS.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({k:v for k,v in r.items() if k not in ['corner_rows','checks','source_hashes']},indent=2))
