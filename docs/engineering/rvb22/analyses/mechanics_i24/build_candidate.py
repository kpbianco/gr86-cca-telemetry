"""Coordinated C06/W03/T04 source candidate with explicit bonded terminals.

Calculations are design screens. No manufactured material, nonlinear motion,
adhesive endurance, installed boundary or qualification is inferred.
"""
from pathlib import Path
import sys, json, math, hashlib, inspect, itertools

D = Path(__file__).resolve().parent
W = D.parents[1]
O = W / 'iterations/I24_thermal_mechanics/candidate_mechanics'
sys.path[:0] = [str(W/'runtime/local'), str(W/'runtime/vendor'), str(W/'analyses/mechanics_i23')]
import numpy as np
import shapely as sh
from shapely.geometry import Polygon, LineString, Point, box
from shapely.affinity import translate, scale
from shapely.ops import unary_union
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import spsolve
from foil_clamps import frame as historical_frame

assert (D/'REDLINE_FREEZE.json').is_file()
O.mkdir(parents=True, exist_ok=True)
# Reuse the independently checked frame algebra, exposing nodal displacements
# so clearance does not confuse the rigid bonded terminal with a free leaf.
frame_source = inspect.getsource(historical_frame)
frame_source = frame_source.replace(
    'return dict(mass_g=',
    'return dict(hot_end_moment_Nmm=float(res[2]*1000),cold_end_moment_Nmm=float(res[-1]*1000),points_mm=(p*1000).tolist(),displacements_mm=(u.reshape((-1,3))[:,:2]*1000).tolist(),mass_g=')
exec(frame_source, globals())
old = W/'iterations/I15_service_mechanics/candidate_mechanics'
G = json.loads((old/'COOLING_GEOMETRY.json').read_text())
body_rows = json.loads((W/'analyses/mated_i22/RESULTS.json').read_text())['body_rows']
mounts = json.loads((W/'lanes/mechanics/THERMAL_WING_GEOMETRY.json').read_text())['mount_centres_mm']
base = sh.from_wkt(G['carrier_base_WKT']).difference(box(13.5,-12.5,54.5,.7))

LEAVES = 100
T_MIN, T_MAX = .020, .022
PACKET_MIN, PACKET_MAX = LEAVES*T_MIN, LEAVES*T_MAX
POSITION, WIDTH_MOTION = .3, .5
E_MIN, E_MAX = 80e9, 140e9
STACK = dict(solder=.25, capture_freedom=.51, warp=.75, local_deflection=.20)
HOT_PAD = [67.5,12.8,71.3,27.9]
SHOE_H = 12.8

# Upper wing: immutable6 mm hot terminal, tangent semicircle/straight return,
# 14 mm unbonded cold straight, then a separate4 mm welded distal terminal.
wing = [(-7.6+2.6*math.cos(a),-3.827+2.6*math.sin(a))
        for a in np.linspace(math.pi/2,math.pi,33)]
wing += [(-10.2,-8.727)]
wing += [(-7.6+2.6*math.cos(a),-8.727+2.6*math.sin(a))
         for a in np.linspace(math.pi,1.5*math.pi,33)[1:]]
wing += [(6.4,-11.327)]

# T04 replaces the old reverse-tangent overlap. Its fixed hot transfer shoe
# places the flex below the carrier. The leaf leaves the LEFT hot-terminal edge
# leftward, turns down and left, then reaches a distinct distal clamp.
central = [(67.5+3*math.cos(a),-14.027+3*math.sin(a))
           for a in np.linspace(math.pi/2,math.pi,33)]
central += [(64.5,-17.027)]
central += [(61.5+3*math.cos(a),-17.027+3*math.sin(a))
            for a in np.linspace(0,-math.pi/2,33)[1:]]
central += [(51.5,-20.027)]
central = [(x,z-3) for x,z in central]

def pbuffer(points):
    return LineString(points).buffer(PACKET_MAX/2, cap_style=2, join_style=1, quad_segs=32)

def frame_cases(points, width):
    # Long element interior curvature cannot be represented by a chord between
    # its translated end nodes. Resolve every beam segment before sweeping.
    original_points = points
    dense = [np.asarray(points[0])]
    for a,b in zip(points,points[1:]):
        a,b = np.asarray(a),np.asarray(b)
        count = int(math.ceil(float(np.linalg.norm(b-a))/.1))
        dense.extend(a+(b-a)*q/count for q in range(1,count+1))
    points = [p.tolist() for p in dense]
    cases = {
        'axial2mm': frame(points,width,LEAVES,T_MAX,(0,2),E=E_MAX),
        'in_plane0p5mm': frame(points,width,LEAVES,T_MAX,(.5,0),E=E_MAX),
        'Z20g': frame(points,width,LEAVES,T_MAX,acceleration=(0,20*9.80665),E=E_MAX),
        'X30g': frame(points,width,LEAVES,T_MAX,acceleration=(30*9.80665,0),E=E_MAX),
        'Z30g': frame(points,width,LEAVES,T_MAX,acceleration=(0,30*9.80665),E=E_MAX),
    }
    displacement_cases = {
        'axial2mm': frame(points,width,LEAVES,T_MIN,(0,2),E=E_MIN),
        'in_plane0p5mm': frame(points,width,LEAVES,T_MIN,(.5,0),E=E_MIN),
        'X30g': frame(points,width,LEAVES,T_MIN,acceleration=(30*9.80665,0),E=E_MIN),
        'Z30g': frame(points,width,LEAVES,T_MIN,acceleration=(0,30*9.80665),E=E_MIN),
    }
    nominal = np.asarray(points)
    ua = np.asarray(displacement_cases['axial2mm']['displacements_mm'])
    ut = np.asarray(displacement_cases['in_plane0p5mm']['displacements_mm'])
    ux = np.asarray(displacement_cases['X30g']['displacements_mm'])
    uz = np.asarray(displacement_cases['Z30g']['displacements_mm'])
    # Independent orthogonal30g unit cases define the continuous circular load
    # envelope. A conservative nodewise box with radii hypot(ux,uz) contains it.
    # Sweep16 directions too as explicit discrete examples; add a circumscribed
    # per-node rectangular hull to avoid treating finite angles as exhaustive.
    hot_solids, cold_solids = [], []
    linear_path_crossings = []
    for sign,xsign in itertools.product([-1,0,1],repeat=2):
        for a in np.linspace(0,2*math.pi,17)[:-1]:
            u = sign*ua + xsign*ut + math.cos(a)*ux + math.sin(a)*uz
            pp = nominal+u
            if not LineString(pp).is_simple:
                linear_path_crossings.append([sign,float(a)])
            cold_solids.append(pbuffer(pp))
            hot_solids.append(pbuffer(pp-[xsign*.5,sign*2]))
        radii = np.hypot(ux,uz)
        for i in range(len(points)-1):
            cs = []
            hs = []
            for j in [i,i+1]:
                center = nominal[j]+sign*ua[j]+xsign*ut[j]
                for dx in [-radii[j,0],radii[j,0]]:
                    for dz in [-radii[j,1],radii[j,1]]:
                        cs.append((center[0]+dx,center[1]+dz))
                        hs.append((center[0]+dx-xsign*.5,center[1]+dz-sign*2))
            cold_solids.append(sh.MultiPoint(cs).convex_hull.buffer(PACKET_MAX/2,quad_segs=16))
            hot_solids.append(sh.MultiPoint(hs).convex_hull.buffer(PACKET_MAX/2,quad_segs=16))
    # The endpoint cap rounding in the conservative hull may cross its bonded
    # mating plane. This is a bounding solid, not a claim of free bonded material.
    hot = unary_union(hot_solids).buffer(POSITION+.001,quad_segs=16)
    cold = unary_union(cold_solids).buffer(POSITION+.001,quad_segs=16)
    result = dict(cases=cases,displacement_cases=displacement_cases,free_length_mm=LineString(points).length,maximum_element_length_mm=.1,
                  force_material_bounds=dict(E_GPa=E_MAX/1e9,leaf_t_mm=T_MAX),
                  displacement_material_bounds=dict(E_GPa=E_MIN/1e9,leaf_t_mm=T_MIN),
                  mass_g=cases['axial2mm']['mass_g'],
                  hot_reaction_allocated_N=2*(abs(cases['axial2mm']['hot_reaction_N'][1])+abs(cases['in_plane0p5mm']['hot_reaction_N'][1])),
                  conservative_hot_frame_WKT=hot.wkt,
                  conservative_cold_frame_WKT=cold.wkt,
                  linear_centerline_self_crossings=linear_path_crossings)
    return result,hot,cold

wing_result, wing_hot, wing_cold = frame_cases(wing,38)
central_result, central_hot, central_cold = frame_cases(central,15.1)
wing_nominal, central_nominal = pbuffer(wing),pbuffer(central)
wing_hot_terminal = box(-7.6,-2.327,-1.6,-.127)
wing_cold_terminal = box(6.4,-12.427,10.4,-10.227)
central_hot_terminal = box(67.5,-15.127,71.3,-12.927)
central_cold_terminal = box(47.5,-24.127,51.5,-21.927)

# Landing bars only under the separate distal welded packets. Their six holes
# are deliberately outside the foil widths; the free cold straights have no
# underlying bar, adhesive or support contact.
landings = [
    dict(name='UPPER',bounds_xy_mm=[7.4,6.4,61.5,14.4],Z_mm=[-14.427,-12.427],holes_xy_mm=[[11,10.4],[57.5,10.4]]),
    dict(name='LOWER',bounds_xy_mm=[7.4,27.6,61.5,35.6],Z_mm=[-14.427,-12.427],holes_xy_mm=[[11,31.6],[57.5,31.6]]),
    dict(name='CENTRAL',bounds_xy_mm=[43.5,4,51.5,35.8],Z_mm=[-26.127,-24.127],holes_xy_mm=[[47.5,7.5],[47.5,32.3]]),
]
for row in landings:
    row.update(material='C11000',plate_thickness_mm=2,max_hole_diameter_mm=3.55)
    row['sink_boundary'] = 'Entire underside of this landing, including the full welded-packet projection, <=70C under combined4.815W. Bolt-site temperature alone is insufficient.'
    row['minimum_hole_edge_ligament_mm'] = min(
        min(x-row['bounds_xy_mm'][0],row['bounds_xy_mm'][2]-x,
            y-row['bounds_xy_mm'][1],row['bounds_xy_mm'][3]-y)-3.55/2-.15
        for x,y in row['holes_xy_mm'])
    assert row['minimum_hole_edge_ligament_mm']>1.5

# C06: insulating root-bearing supports and positive upper capture caps. The
# outer ear is relieved0.5 mm so the load enters the PCB at a1 mm root band;
# no credited clamp friction or printed-thread holding force.
supports, caps, root_bands, capture_hardware = [], [], [], []
for x,y in mounts:
    upper = y<0
    root_y = 0 if upper else y-4
    x0,x1 = x-3.5, (65 if upper and x==63 else x+3.5)
    y0,y1 = (y-3.5,-.2) if upper else (root_y+.2,y+3.5)
    plan = box(x0,y0,x1,y1).difference(Point(x,y).buffer(1.1,quad_segs=32))
    band = box(x0,root_y-1.2 if upper else root_y+.2,x1,root_y-.2 if upper else root_y+1.2)
    post_plan=plan
    if upper and x==63:
        post_plan=plan.difference(box(64.4,-1.8,68,1))
        band=band.intersection(box(-100,-1.0,64.4,100))
    supports += [dict(name=f'POST_{x}_{y}',geometry=post_plan,z=(-6,-.5)),
                 dict(name=f'ROOT_LOWER_{x}_{y}',geometry=band,z=(-.5,0))]
    # Height1.90..2.0 is an explicit machined capture condition;0.14..0.56mm
    # free stack can exceed the old0.51mm if2.0 is accepted. Therefore cap lower
    # bearing face is restricted to1.85..1.95mm, preserving0.09..0.51mm.
    caps += [dict(name=f'CAP_{x}_{y}',geometry=plan,z=(2.45,5.45)),
             dict(name=f'ROOT_UPPER_{x}_{y}',geometry=band,z=(1.95,2.45))]
    # Independent insulating edge keys locate the PCB; loose bolt clearance
    # sleeves do not. Keys are integral to removable upper caps and assembled
    # from above. Nominal0.20mm gap minus0.10PCB-edge/0.05machining tolerance
    # retains0.05mm insertion gap and bounds local motion by0.35mm.
    side_y0,side_y1=y-2,y+1
    keys=[box(x-4.7,side_y0,x-3.7,side_y1),box(x+3.7,side_y0,x+4.7,side_y1)]
    keys += [box(x-2.3,y-4.2,x+2.3,y-3.2) if upper else box(x-2.3,y+3.2,x+2.3,y+4.2)]
    for j,key in enumerate(keys):
        cap_bridge=box(min(x-3.5,key.bounds[0]),min(y-3.5,key.bounds[1]),max(x+3.5,key.bounds[2]),max(y+1,key.bounds[3]))
        # Bridge remains above the PCB; side keys stop atZ−0.10, clear of the
        # declared coax depth slab whose upper face is−0.60mm.
        caps += [dict(name=f'KEY_{x}_{y}_{j}',geometry=key,z=(-.1,2.45)),
                 dict(name=f'KEY_BRIDGE_{x}_{y}_{j}',geometry=cap_bridge,z=(2.45,5.45))]
    sleeve = Point(x,y).buffer(1.6,quad_segs=32).difference(Point(x,y).buffer(1.1,quad_segs=32))
    capture_hardware += [dict(name=f'CAPTURE_SLEEVE_{x}_{y}',geometry=sleeve,z=(-.5,2.45)),
                         dict(name=f'M2_HEAD_{x}_{y}',geometry=Point(x,y).buffer(1.9,quad_segs=32),z=(5.45,7.45)),
                         dict(name=f'M2_NUT_{x}_{y}',geometry=Point(x,y).buffer(2.45,quad_segs=32),z=(-10.6,-9))]
    root_bands.append(dict(center=[x,y],root_y_mm=root_y,band_bounds_mm=list(band.bounds),
                           minimum_bearing_width_mm=band.bounds[2]-band.bounds[0]-.3,
                           max_root_lever_mm=1.15 if upper and x==63 else 1.35))

checks=[]
def prism(ref,name,xy,z,obstacle,oz):
    zgap=max(0,oz[0]-z[1],z[0]-oz[1])
    depth=max(0,min(z[1],oz[1])-max(z[0],oz[0]))
    overlap=xy.intersection(obstacle).area*depth
    checks.append(dict(ref=ref,obstacle=name,overlap_mm3=float(overlap),
                       clearance_mm=float(math.hypot(xy.distance(obstacle),zgap))))

obstacles=[dict(name='C06_BASE',geometry=base.buffer(.15),z=(-9,-6))]
obstacles += [dict(name=p['name'],geometry=p['geometry'].buffer(.15),z=p['z']) for p in supports+caps+capture_hardware]
for row in landings:
    obstacles.append(dict(name=row['name']+'_LANDING',geometry=box(*row['bounds_xy_mm']).buffer(.15),z=row['Z_mm']))
    for x,y in row['holes_xy_mm']:
        obstacles.append(dict(name=row['name']+'_M3_HEAD',geometry=Point(x,y).buffer(3.15,quad_segs=32),z=(row['Z_mm'][1],row['Z_mm'][1]+2.4)))
obstacles += [dict(name='T04_HOT_SHOE',geometry=box(*HOT_PAD).buffer(.3),z=(-12.927,-.127))]

lower_hot=translate(scale(wing_hot,xfact=-1,yfact=1,origin=(0,0)),xoff=42)
for row in body_rows:
    xy=box(*row['courtyard_plus0p3_bounds_mm'])
    h=row['height_max_mm']+sum(STACK.values())
    z=(-h,0) if row['side']=='B.Cu' else (1.44,1.76+h)
    for ob in obstacles:prism(row['ref'],ob['name'],xy,z,ob['geometry'],ob['z'])
    if row['side']=='B.Cu':
        x0,y0,x1,y1=xy.bounds
        for name,g in [('W03_UPPER',wing_hot),('W03_LOWER',lower_hot)]:
            prism(row['ref'],name,box(y0,-h,y1,0),(x0,x1),g,(15-.8,53+.8))
        prism(row['ref'],'T04_FREE',box(x0,-h,x1,0),(y0,y1),central_hot,(12.8-.8,27.9+.8))
        # Fixed hot foil terminals are not assigned free flex motion.
        prism(row['ref'],'T04_HOT_TERMINAL',box(x0,-h,x1,0),(y0,y1),central_hot_terminal.buffer(.3),(12.8-.3,27.9+.3))

interferences=[x for x in checks if x['overlap_mm3']>1e-7]

# Thermal conduction includes maximum concentric-path difference across both
# 90-degree bends and0.5mm total cut allowance. Interfaces remain acceptance
# allocations; deriving them from a catalogue typical value is not qualification.
wing_cut=6+wing_result['free_length_mm']+4+math.pi*PACKET_MAX/2+.5
central_cut=3.8+central_result['free_length_mm']+4+math.pi*PACKET_MAX/2+.5
wing_metal_R=wing_cut/(300*PACKET_MIN*38)*1000
central_metal_R=central_cut/(300*PACKET_MIN*15.1)*1000
shoe_R=SHOE_H/(300*3.8*15.1)*1000
thermal=dict(
    wing=dict(contact_rectangles_mm=[[15,-7.6,53,-1.6],[15,43.6,53,49.6]],
              hot_areal_R_K_W=1.7+.00003/(.2*228e-6),
              foil_R_K_W=wing_metal_R,terminal_R_allocation_K_W=.5,
              landing_through_thickness_R_within_terminal_allocation_K_W=.002/(300*4*38e-6),
              remaining_weld_attachment_R_allocation_K_W=.5-.002/(300*4*38e-6),
              max_cut_length_mm=wing_cut,model_total_R_K_W=5),
    central=dict(contact_rectangle_mm=HOT_PAD,
                 hot_areal_R_K_W=(6.399442314395266+2.614151272220288)*57.38/57.09737,
                 shoe_R_K_W=shoe_R,foil_R_K_W=central_metal_R,
                 terminal_R_allocation_K_W=1,max_cut_length_mm=central_cut,
                 landing_through_thickness_R_within_terminal_allocation_K_W=.002/(300*4*15.1e-6),
                 remaining_weld_attachment_R_allocation_K_W=1-.002/(300*4*15.1e-6),
                 model_total_R_K_W=15),
    air_C=65,landing_C=70,board_heat_W=4.815,
    sink_boundary='Complete underside of all three landing bars, including welded-packet projections, <=70C. No bolt-only cooling or floating-plate credit.',
    scope='Specified path allocation with copper k>=300W/mK and accepted interfaces; no measured contact, uniform-terminal temperature or package qualification.')
thermal['wing']['sum_R_K_W']=thermal['wing']['hot_areal_R_K_W']+wing_metal_R+.5
thermal['central']['sum_R_K_W']=thermal['central']['hot_areal_R_K_W']+shoe_R+central_metal_R+1
assert thermal['wing']['sum_R_K_W']<=5 and thermal['central']['sum_R_K_W']<=15

# All rigid hot-side copper mass is counted in addition to the unchanged100g
# populated PCB allowance. Cold terminals/plates are supported independently.
hot_mass_g=(2*6*38*PACKET_MAX+3.8*15.1*PACKET_MAX+3.8*15.1*SHOE_H)*.00896
inertia=2*abs(wing_result['cases']['Z20g']['hot_reaction_N'][1])+abs(central_result['cases']['Z20g']['hot_reaction_N'][1])
spring=2*wing_result['hot_reaction_allocated_N']+central_result['hot_reaction_allocated_N']
load=(.100+hot_mass_g/1000)*20*9.80665+inertia+spring
minwidth=min(x['minimum_bearing_width_mm'] for x in root_bands)
mindepth=min(x['band_bounds_mm'][3]-x['band_bounds_mm'][1]-.3 for x in root_bands)
root_stress=3*6*load*.00135/(minwidth*.001*.00144**2)/1e6
cap_stress=3*6*load*.0045/(minwidth*.001*.00285**2)/1e6
root_bearing=load/(minwidth*mindepth)
structure=dict(one_root_carries_all20g_N=load,hot_copper_mass_g=hot_mass_g,
               free_span_hot_inertia_N=inertia,allocated_spring_N=spring,
               PCB_mass_allocation_g=100,minimum_bearing_width_mm=minwidth,
               bearing_band_min_depth_mm=mindepth,max_root_lever_mm=1.35,
               Kt3_PCB_root_MPa=root_stress,hot_notched_PCB_flexural_requirement_MPa=150,
               Kt3_capture_cap_flexural_MPa=cap_stress,capture_material_hot_flexural_requirement_MPa=150,
               local_bearing_MPa=root_bearing,bearing_requirement_MPa=50,
               capture_gap_mm=[.09,.51],root_bands=root_bands,
               material='Machined electrical-grade insulating laminate; hot properties are procurement requirements, not published guarantees.',
               required_load_path='M2 through fasteners,2.85..2.95mm insulating sleeves between postZ−0.5 and cap-bodyZ2.35..2.45, retained nuts and C06 root contact bands; bearing faces remain1.85..1.95mm above PCB bottom. No clamp-friction or printed-thread credit.',
               sleeve_OD_mm=[3.1,3.3],sleeve_ID_mm=[2.05,2.35],
               sleeve_min_hole_radial_clearance_mm=(4.25-3.3)/2-.3,
               sleeve_max_compression_MPa=load/(math.pi/4*(3.1**2-2.35**2)),
               M2_installation_torque_Nm=[.02,.04],nut_factor_condition=[.15,.30],
               preload_interval_N=[.02/(.3*.002),.04/(.15*.002)],
               sleeve_preload_plus_service_compression_MPa=(.04/(.15*.002)+load)/(math.pi/4*(3.1**2-2.35**2)),
               hot_compression_requirement_MPa=100,
               preload_scope='Torque window is a proposed assembly control conditional on verified frictionfactor, hardware and retained preload. Hot creep, locking process and thermal-cycle capture gap require qualification.',
               edge_key_nominal_gap_mm=.20,PCB_edge_position_tolerance_mm=.10,key_machining_position_tolerance_mm=.05,
               minimum_insertion_gap_mm=.05,maximum_key_local_clearance_mm=.35,
               scope='Nominal static beam/bearing allocation; superseded for root bearing and stress by the all-pose LOCATING_MOTION.json screen. No impact-contact dynamics, PCB strain, full assembly FEA, material qualification, fatigue or retention test.')

def polygon_scad(g):
    if g.geom_type=='MultiPolygon':return 'union(){'+''.join(polygon_scad(p) for p in g.geoms)+'}'
    vertices=[];paths=[]
    for ring in [g.exterior,*g.interiors]:
        v=list(ring.coords)[:-1];paths.append(list(range(len(vertices),len(vertices)+len(v))));vertices+=v
    return 'polygon(points='+json.dumps(vertices)+',paths='+json.dumps(paths)+');'

def xy_scad(g,z0,h):return f'translate([0,0,{z0}])linear_extrude({h})'+polygon_scad(g)
def yz_scad(g,x0,width):return f'translate([{x0},0,0])rotate([90,0,90])linear_extrude({width})'+polygon_scad(g)
def xz_scad(g,y0,width):return f'translate([0,{y0+width},0])rotate([90,0,0])linear_extrude({width})'+polygon_scad(g)

header='// I24 C06/W03/T04 engineering candidate. See CANDIDATE_SPEC.json.\n$fn=64;\n'
carrier_scad=header+'color("ivory"){'+xy_scad(base,-9,3)+''.join(xy_scad(p['geometry'],p['z'][0],p['z'][1]-p['z'][0]) for p in supports+caps+capture_hardware)+'}\n'
(O/'GR86_RVB_CARRIER_C06.scad').write_text(carrier_scad)
wu=unary_union([wing_nominal,wing_hot_terminal,wing_cold_terminal])
wl=translate(scale(wu,xfact=-1,yfact=1,origin=(0,0)),xoff=42)
(O/'THERMAL_WING_W03.scad').write_text(header+'color([.8,.4,.1]){'+yz_scad(wu,15,38)+yz_scad(wl,15,38)+'}\n')
central_all=unary_union([central_nominal,central_hot_terminal,central_cold_terminal])
(O/'THERMAL_CONTACT_T04.scad').write_text(header+'color([.8,.4,.1]){'+xz_scad(central_all,12.8,15.1)+xy_scad(box(*HOT_PAD),-12.927,12.8)+'}\ncolor([1,1,1,.6])'+xy_scad(box(*HOT_PAD),-.127,.127)+'\n')
landing_scad=header+'color([.8,.4,.1]){'
for row in landings:
    plan=box(*row['bounds_xy_mm'])
    for x,y in row['holes_xy_mm']:plan=plan.difference(Point(x,y).buffer(1.7,quad_segs=32))
    row['WKT']=plan.wkt
    landing_scad+=xy_scad(plan,row['Z_mm'][0],2)
    for x,y in row['holes_xy_mm']:
        landing_scad+=f'color("silver")translate([{x},{y},{row["Z_mm"][1]}])cylinder(d=6,h=2.4);'
landing_scad+='}\n'
(O/'THERMAL_LANDINGS_C06.scad').write_text(landing_scad)
(O/'ASSEMBLY_C06_W03_T04.scad').write_text(header+'include <GR86_RVB_CARRIER_C06.scad>\ninclude <THERMAL_WING_W03.scad>\ninclude <THERMAL_CONTACT_T04.scad>\ninclude <THERMAL_LANDINGS_C06.scad>\n')

current_pcb=W/'iterations/I24_return_clearance_final/candidate_kicad/GR86_CCA_RevB.kicad_pcb'
assert current_pcb.is_file(), 'Current source must exist; final_review.py independently validates inherited footprint poses.'
report=dict(status='AUTHORED_REWORK_NOT_ACCEPTED',construction=dict(leaves=LEAVES,leaf_thickness_mm=[T_MIN,T_MAX],
             total_thickness_mm=[PACKET_MIN,PACKET_MAX],material='Annealed C11000; unbonded free leaves; welded/brazed terminal packets only'),
            source_PCB=str(current_pcb.relative_to(W)),source_PCB_sha256=hashlib.sha256(current_pcb.read_bytes()).hexdigest(),
            source_effectivity_evidence='analyses/mechanics_i24/SOURCE_EFFECTIVITY.json; normalized pose/value/side comparison is required before clearance credit.',
            retained_hot_contact_rectangles_mm=thermal['wing']['contact_rectangles_mm']+[HOT_PAD],
            thermal=thermal,structure=structure,landings=landings,
            carrier_base_WKT=base.wkt,
            carrier_solids=[dict(name=p['name'],WKT=p['geometry'].wkt,Z_mm=p['z']) for p in supports+caps+capture_hardware],
            nominal_free_centerlines_mm=dict(wing=wing,central=central),
            terminal_rectangles_cross_section_mm=dict(wing_hot=list(wing_hot_terminal.bounds),wing_cold=list(wing_cold_terminal.bounds),central_hot=list(central_hot_terminal.bounds),central_cold=list(central_cold_terminal.bounds)),
            full_component_height_stack_mm=STACK,
            assembly_nominal_bounds_mm=[-13,-15,-29.127,95.454766,58,17.76],
            RF_antenna_start_X_mm=88.254766,RF_requirement_mm=15,
            RF_hot_shoe_minimum_metal_setback_mm=88.254766-(71.3+.3),
            out_of_plane_width_motion_allocation_mm=WIDTH_MOTION,
            populated_checks_count=len(checks),populated_interferences=interferences,
            nearest=sorted(checks,key=lambda x:x['clearance_mm'])[:20],
            all_checks=checks,
            open_conditions=['Unbonded leaf friction, nonlinear finite displacement and out-of-plane torsion require independent analysis/qualification; the0.5mm width motion remains an explicit unproven allocation.',
              'Hot foil, tape/mask, shoe joints and terminal resistance must be accepted; thermal model assumptions do not establish manufactured joints.',
              'Leaf procurement/forming/weld isolation, insulating laminate hot properties, retained hardware and root-bearing contact require supplier/process acceptance.',
              'The complete underside of each of the three landing bars and full welded-packet projections must remain <=70C at full 4.815W; six bolt-site temperatures alone are insufficient. Installed cavity depth and attachment remain unobserved.',
              'Adafruit85160C rating conflict, package temperatures, thermal mesh topology/convergence and hardware qualification remain open.'],
            physical_measurements_claimed=False)
(O/'CANDIDATE_SPEC.json').write_text(json.dumps(report,indent=2)+'\n')
(O/'THERMAL_PATHS.json').write_text(json.dumps(thermal,indent=2)+'\n')
(D/'FRAME_RESULTS.json').write_text(json.dumps(dict(wing=wing_result,central=central_result),indent=2)+'\n')
(D/'POPULATED_CLEARANCE.json').write_text(json.dumps(dict(status=report['status'],checks=checks,interferences=interferences,nearest=report['nearest']),indent=2)+'\n')
print(json.dumps(dict(interferences=interferences,nearest=report['nearest'][:8],thermal=thermal,structure=structure),indent=2))
