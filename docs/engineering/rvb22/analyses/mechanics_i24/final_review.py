"""Frozen-source effectivity, rigid locating motion and review figures."""
from pathlib import Path
import sys,json,hashlib,math,itertools
D=Path(__file__).resolve().parent;W=D.parents[1]
sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor'),str(W/'control')]
import numpy as np
import shapely as sh
from shapely.geometry import box,LineString,Point,Polygon
from shapely.affinity import rotate,translate
from shapely.ops import unary_union
from check_combined_copper import child,get,prop
import sexpdata
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Polygon as MP

O=W/'iterations/I24_thermal_mechanics/candidate_mechanics'
sp=O/'CANDIDATE_SPEC.json';spec=json.loads(sp.read_text())
frames=json.loads((D/'FRAME_RESULTS.json').read_text())
source_data=W/'analyses/mated_i22/RESULTS.json'
mated=json.loads(source_data.read_text());bodies=mated['body_rows']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
reference=W/'runtime/hosted/run26/extracted/native_I06_hosted/candidate_kicad/GR86_CCA_RevB.kicad_pcb'
pcb=W/'iterations/I24_return_clearance_final/candidate_kicad/GR86_CCA_RevB.kicad_pcb'
if not pcb.is_file():pcb=W/'iterations/I23_return_clearance/candidate_kicad/GR86_CCA_RevB.kicad_pcb'
def poses(p):
    tree=sexpdata.loads(p.read_text())
    rows={}
    for f in child(tree,'footprint'):
        at=get(f,'at'); rows[prop(f)['Reference']]=dict(xy=at[:2],rotation=(at[2]if len(at)>2 else 0)%360,layer=str(get(f,'layer')[0]),value=prop(f).get('Value'))
    return rows
old,new=poses(reference),poses(pcb)
assert set(old)==set(new)
differences=[]
for ref,a in old.items():
    b=new[ref]
    if max(abs(x-y)for x,y in zip(a['xy'],b['xy']))>1e-6 or a['rotation']!=b['rotation'] or a['layer']!=b['layer'] or a['value']!=b['value']:differences.append(ref)
assert not differences,differences
binding=dict(source_PCB=str(pcb.relative_to(W)),source_PCB_sha256=sha(pcb),
             native_pose_reference=str(reference.relative_to(W)),native_pose_reference_sha256=sha(reference),
             inherited_body_evidence=str(source_data.relative_to(W)),body_evidence_sha256=sha(source_data),
             checked_footprints=len(old),fitted_body_rows=len(bodies),differences=differences,
             equivalent_rotations_normalized=True,coordinate_tolerance_mm=1e-6,
             geometry_spec_sha256=sha(sp),scope='Pose/value/side inheritance binds declared body envelopes. It does not certify manufacturer3D models or physical component placement.')
(D/'SOURCE_EFFECTIVITY.json').write_text(json.dumps(binding,indent=2)+'\n')

# Ear edge keys impose a local±0.35mm location bound. With small-angle rigid
# motion tx,ty,theta about the lower-left coordinate origin, enumerate every
# vertex of the convex set. Exact sine/cosine evaluations add0.01mm padding
# to cover the linearized key constraint at the maximum observed rotation.
mounts=[r['center']for r in spec['structure']['root_bands']]
origin=np.mean(np.array(mounts),axis=0)
rows=[]
for x,y in np.array(mounts)-origin:
    rows += [[1,0,-y],[-1,0,y],[0,1,x],[0,-1,-x]]
A=np.array(rows,float);bound=.35
states=[]
for inds in itertools.combinations(range(len(A)),3):
    q=A[list(inds)]
    if abs(np.linalg.det(q))<1e-9:continue
    u=np.linalg.solve(q,np.full(3,bound))
    if np.max(A@u-bound)<1e-10 and not any(np.max(abs(u-v))<1e-8 for v in states):states.append(u)
assert states
motion_checks=[]
fixture=[('C06_BASE',sh.from_wkt(spec['carrier_base_WKT']).buffer(.15),(-9,-6))]
fixture += [(r['name'],sh.from_wkt(r['WKT']).buffer(.15),r['Z_mm'])for r in spec['carrier_solids']]
for body in bodies:
    p=box(*body['courtyard_plus0p3_bounds_mm'])
    moved=unary_union([translate(rotate(p,s[2],origin=tuple(origin),use_radians=True),xoff=s[0],yoff=s[1]) for s in states]).convex_hull.buffer(.01)
    h=body['height_max_mm']+1.71
    z=(-h,0)if body['side']=='B.Cu'else(1.44,1.76+h)
    for name,g,oz in fixture:
        depth=max(0,min(z[1],oz[1])-max(z[0],oz[0]));gap=max(0,oz[0]-z[1],z[0]-oz[1])
        motion_checks.append(dict(ref=body['ref'],obstacle=name,overlap_mm3=float(moved.intersection(g).area*depth),clearance_mm=float(math.hypot(moved.distance(g),gap))))
hits=[r for r in motion_checks if r['overlap_mm3']>1e-7]
service_motion=[]
for row in mated['envelopes']:
    p=box(*row['bounds_xy_mm'])
    moved=unary_union([translate(rotate(p,s[2],origin=tuple(origin),use_radians=True),xoff=s[0],yoff=s[1])for s in states]).convex_hull.buffer(.01)
    for name,g,oz in fixture:
        depth=max(0,min(row['z'][1],oz[1])-max(row['z'][0],oz[0]))
        service_motion.append(dict(service=row['name'],obstacle=name,overlap_mm3=float(moved.intersection(g).area*depth)))
cable_points=np.asarray(mated['pigtail']['centerline_mm'])
traces=[]
for a,b in zip(cable_points,cable_points[1:]):
    segment=LineString([a,b])
    traces.append(unary_union([translate(rotate(segment,s[2],origin=tuple(origin),use_radians=True),xoff=s[0],yoff=s[1])for s in states]).convex_hull.buffer(1.51,quad_segs=32))
cable_sweep=unary_union(traces)
for name,g,oz in fixture:
    depth=max(0,min(-.6,oz[1])-max(-3.6,oz[0]))
    service_motion.append(dict(service='Adafruit851_full150mm_board_following_sweep',obstacle=name,overlap_mm3=float(cable_sweep.intersection(g).area*depth),
                               clearance_mm=float(math.hypot(cable_sweep.distance(g),max(0,oz[0]+.6,-3.6-oz[1])))))
board_outline=Polygon(json.loads((W/'iterations/I18_capacitor_thermal_tab/CURRENT_OUTLINE_GEOMETRY.json').read_text())['outline_mm'])
board_material=board_outline.buffer(-.1)
for xy in mounts:board_material=board_material.difference(Point(*xy).buffer(2.275+.15,quad_segs=64))
bearing=[]
for root in spec['structure']['root_bands']:
    band=box(*root['band_bounds_mm']).buffer(-.15)
    # A rectangle contained in the local ear/root provides a conservative
    # convex outer material bound; hole void swept hull is subtracted separately.
    x,y=root['center'];ry=root['root_y_mm']
    local=box(x-3.5,ry-3,x+3.5,ry+3).buffer(-.1)
    local=local.intersection(board_outline.buffer(-.1))
    core=None;holes=[]
    for s in states:
        moved=translate(rotate(local,s[2],origin=tuple(origin),use_radians=True),xoff=s[0],yoff=s[1])
        core=moved if core is None else core.intersection(moved)
        holes.append(translate(rotate(Point(x,y).buffer(2.275+.15,quad_segs=64),s[2],origin=tuple(origin),use_radians=True),xoff=s[0],yoff=s[1]))
    guaranteed=core.buffer(-.01).difference(unary_union(holes).convex_hull.buffer(.01))
    area=band.intersection(guaranteed).area
    depth=band.bounds[3]-band.bounds[1]
    minimum_width=area/depth
    center=np.array([root['center'][0],root['root_y_mm']])
    max_shift=max(abs(translate(rotate(Point(*center),s[2],origin=tuple(origin),use_radians=True),xoff=s[0],yoff=s[1]).y-center[1])for s in states)
    tilt=max(abs(math.tan(s[2]))for s in states)*(band.bounds[2]-band.bounds[0])/2
    lever=root['max_root_lever_mm']+max_shift+.10+.01+tilt
    force=spec['structure']['one_root_carries_all20g_N']
    stress=3*6*force*(lever*.001)/(minimum_width*.001*.00144**2)/1e6
    bearing.append(dict(center=root['center'],minimum_contact_area_mm2=area,minimum_effective_width_mm=minimum_width,
                        max_root_lever_mm=lever,one_root_force_only_Kt3_MPa=stress,
                        max_bearing_MPa=force/area,scope='All-pose conservative ear core, hole swept void, key motion, outline/fixture tolerance and root-line tilt included; thermal end moments, joint/contact stress and nonlinear impact remain open.'))
loc=dict(status='CONDITIONAL_RIGID_KEY_SCREEN',origin_mm=origin.tolist(),
         local_key_motion_bound_mm=bound,states=[s.tolist()for s in states],
         maximum_rotation_deg=max(abs(s[2])for s in states)*180/math.pi,
         key_gap_nominal_mm=.2,PCB_edge_position_tolerance_mm=.1,key_machining_tolerance_mm=.05,
         minimum_insertion_gap_mm=.05,linearization_padding_mm=.01,
         checks_count=len(motion_checks),interferences=hits,
         service_checks_count=len(service_motion),service_interferences=[r for r in service_motion if r['overlap_mm3']>1e-7],
         cable_sweep_segments=len(cable_points)-1,cable_sweep_radius_mm=1.51,
         cable_checks=[r for r in service_motion if r['service']=='Adafruit851_full150mm_board_following_sweep'],
         corrected_root_bearing_force_only_screens=bearing,
         nearest=sorted(motion_checks,key=lambda x:x['clearance_mm'])[:12],
         limitations=['Finished-outline and key-position tolerances are explicit assembly acceptance conditions, not guaranteed supplier process.',
                     'Rigid-body corner sweep excludes deformable PCB modes, side-key contact stress/impact and cap/locator elastic deflection.',
                     'Separate captured PCB Zstack1.71mm remains unchanged. End rotations and out-of-plane flex response remain open.'],
         geometry_spec_sha256=sha(sp),physical_measurements_claimed=False)
(D/'LOCATING_MOTION.json').write_text(json.dumps(loc,indent=2)+'\n')

# Export force/moment and common-centerline elastic stress demand, retaining the
# independent per-leaf yield/nonlinear qualification concern explicitly.
demands=[]
for name,width in [('wing',38),('central',15.1)]:
    c=frames[name]['cases'];t=.022;n=100
    for end in ['hot','cold']:
        key=end+'_end_moment_Nmm'
        imposed=abs(c['axial2mm'][key])+abs(c['in_plane0p5mm'][key])
        inertia=math.hypot(c['X30g'][key],c['Z30g'][key])
        M=imposed+inertia
        demands.append(dict(part=name,end=end,bending_moment_Nmm=M,
                            fiber_stress_MPa=6*M/(n*width*t*t),leaf_t_mm=t,
                            load='±2mm Z,±0.5mm in-plane translation,arbitrary in-plane30g direction',E_GPa=140))
joint=dict(status='ELASTIC_VALIDITY_AND_JOINT_MOMENTS_OPEN',common_centerline_end_demands=demands,
           central_shoe_height_mm=12.8,
           central_hot_horizontal_reaction_N=frames['central']['cases']['axial2mm']['hot_reaction_N'][0],
           shoe_joint_additional_lever_moment_Nmm=abs(frames['central']['cases']['axial2mm']['hot_reaction_N'][0])*(12.8+1.227),
           open=['AnnealedC110 per-leaf hot yield/elastic validity is not established by a150MPa laminate requirement.',
                 'Concentric individual leaves have different path length, curvature and stress; common-centerline demand is not a worst-leaf proof.',
                 'Hot/cold end moments and horizontal shoe lever loads require joint peel/rotation, landing restraint and full mount load distribution analysis.',
                 '80..140GPa E and150MPa laminate requirements must be demonstrated across actual local material temperatures; air65C does not set material temperature.',
                 'Force-only PCB/cap screens are partial static estimates and do not establish complete structural acceptance.'],
           geometry_spec_sha256=sha(sp),physical_measurements_claimed=False)
(D/'JOINT_AND_ELASTIC_DEMANDS.json').write_text(json.dumps(joint,indent=2)+'\n')

plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9})
fig,axs=plt.subplots(2,2,figsize=(19,12),layout='constrained')
fig.suptitle('I24 C06 / W03 / T04 — authored mechanical rework, acceptance OPEN',fontsize=17,weight='bold')
def patch(ax,g,color,alpha=1,label=None):
    if g.is_empty:return
    if g.geom_type=='MultiPolygon':
        for j,p in enumerate(g.geoms):patch(ax,p,color,alpha,label if j==0 else None)
        return
    if g.geom_type!='Polygon':return
    ax.add_patch(MP(list(g.exterior.coords),facecolor=color,edgecolor=color,alpha=alpha,label=label,lw=.7))
    for h in g.interiors:ax.add_patch(MP(list(h.coords),facecolor='white',edgecolor=color,lw=.5))
def format_ax(ax,title,xlabel,ylabel):
    ax.set_title(title,loc='left',fontsize=12,weight='bold');ax.set_xlabel(xlabel);ax.set_ylabel(ylabel)
    ax.set_aspect('equal');ax.grid(alpha=.15)

ax=axs[0,0]
board=Polygon(json.loads((W/'iterations/I18_capacitor_thermal_tab/CURRENT_OUTLINE_GEOMETRY.json').read_text())['outline_mm'])
patch(ax,board,'#e2eae4',label='PCB outline')
for r in bodies:
    if r['side']=='B.Cu':patch(ax,box(*r['courtyard_plus0p3_bounds_mm']),'#808d99',.30)
for r in spec['carrier_solids']:
    if r['name'].startswith(('ROOT_UPPER','KEY_')):patch(ax,sh.from_wkt(r['WKT']),'#24679b',.8)
for r in spec['landings']:patch(ax,box(*r['bounds_xy_mm']),'#b87631',.32)
for r in spec['retained_hot_contact_rectangles_mm']:patch(ax,box(*r),'#d46b22',.9)
ax.plot(*np.array(mated['pigtail']['centerline_mm']).T,color='#8355ac',lw=1.3,label='851:150mm cable / R15')
ax.axvline(73.254766,color='#b43a38',ls='--',label='15mm antenna metal boundary')
ax.set_xlim(-16,98);ax.set_ylim(60,-18)
format_ax(ax,'Plan: fixed hot contacts, distal landings, insulating edge keys','PCB X(mm)','PCB Y(mm)')
ax.legend(loc='lower left',fontsize=8)

ax=axs[0,1]
patch(ax,box(58,0,75,1.6),'#668b70',label='PCB1.44–1.76mm')
patch(ax,box(67.5,-12.927,71.3,-.127),'#ce8b45',label='12.8mm C110 hot shoe')
for name in ['central_hot','central_cold']:
    patch(ax,box(*spec['terminal_rectangles_cross_section_mm'][name]),'#8c421a',label='Bonded terminal'if name.endswith('hot')else None)
path=spec['nominal_free_centerlines_mm']['central'];patch(ax,LineString(path).buffer(1.1,cap_style=2),'#dca363',label='2.2mm maximum packet')
patch(ax,sh.from_wkt(frames['central']['conservative_cold_frame_WKT']),'#e9bc6b',.25,label='Conditional linear motion envelope')
patch(ax,box(43.5,-26.127,51.5,-24.127),'#794222',label='Full underside≤70°C required')
patch(ax,box(58,-9,66.5,-6),'#b0bbc4',.6,label='Carrier Z−9..−6')
for ref in ['R408','L406']:
    r=next(x for x in bodies if x['ref']==ref);x0,y0,x1,y1=r['courtyard_plus0p3_bounds_mm'];h=r['height_max_mm']+1.71
    patch(ax,box(x0,-h,x1,0),'#785d75',.65)
ax.text(64,-3.7,'R408 / L406',ha='center',va='top',fontsize=8)
ax.text(44,-29,'Cold clamp adds4mm; actual free length22.424mm\nCentral path budget14.574K/W; model uses15K/W',fontsize=8)
ax.set_xlim(42,76);ax.set_ylim(-31,3)
format_ax(ax,'T04 X–Z section: tangent-continuous free span below carrier','PCB X(mm)','Z(mm); PCB bottom=0')
ax.legend(loc='upper left',bbox_to_anchor=(1.01,1),fontsize=7)

ax=axs[1,0]
patch(ax,box(-7,0,0,1.6).difference(box(-6.2,0,-1.8,1.6)),'#668b70',label='PCB ear /4.4mm hole')
patch(ax,box(-7.5,-6,-.2,-.5),'#a5b9c6',label='C06 insulating support')
patch(ax,box(-1.2,-.5,-.2,0),'#24679b',label='Lower root bearing band')
patch(ax,box(-7.5,2.45,-.2,5.45),'#a5b9c6',label='Removable capture cap')
patch(ax,box(-1.2,1.95,-.2,2.45),'#24679b',label='Upper root bearing band')
patch(ax,box(-5.6,-.5,-2.4,2.45).difference(box(-5.1,-.5,-2.9,2.45)),'#d0a067',label='Insulating clearance sleeve')
patch(ax,box(-5.9,5.45,-2.1,7.45),'#777777',label='M2 head;0.02–0.04Nm proposal')
patch(ax,box(-8.2,-.1,-7.2,2.45),'#24679b',label='Separate edge locator key')
ax.annotate('0.09–0.51mm captured Z freedom',xy=(-.7,1.9),xytext=(1.3,2.8),arrowprops={'arrowstyle':'->'},fontsize=8)
ax.text(-8.5,-8,'Service+preload compression must be qualified.\nKey gap0.20mm nominal;0.05mm minimum insertion gap.\nPCB/key tolerances and retained preload are acceptance conditions.',fontsize=8)
ax.set_xlim(-9,9);ax.set_ylim(-9,9)
format_ax(ax,'C06 upper-ear section: root capture and independent XY location','Distance along PCB Y(mm)','Z(mm)')
ax.legend(loc='upper left',bbox_to_anchor=(1.01,1),fontsize=7)

ax=axs[1,1]
wing=spec['nominal_free_centerlines_mm']['wing']
patch(ax,box(-10,0,10,1.6),'#668b70',label='PCB thermal wing')
patch(ax,LineString(wing).buffer(1.1,cap_style=2),'#dca363',label='27.067mm truly free span')
patch(ax,sh.from_wkt(frames['wing']['conservative_cold_frame_WKT']),'#e9bc6b',.3,label='Conditional linear sweep')
for name in ['wing_hot','wing_cold']:patch(ax,box(*spec['terminal_rectangles_cross_section_mm'][name]),'#8c421a',label='6mm hot /4mm cold terminal'if name.endswith('hot')else None)
patch(ax,box(6.4,-14.427,14.4,-12.427),'#794222',label='Distal landing; free region remains open')
patch(ax,box(.7,-9,14.4,-6),'#b0bbc4',.6,label='Recessed carrier window')
ax.text(-12,-17,'100×20–22µm free laminae;2.0–2.2mm packet.\n±2mm Z +±0.5mm in-plane motion;30g linear sensitivity.\nYield, nonlinear response, out-of-plane motion and installed fit OPEN.',fontsize=8)
ax.set_xlim(-13,16);ax.set_ylim(-19,3)
format_ax(ax,'W03 Y–Z section: cold straight is not bonded to a landing','PCB Y(mm)','Z(mm)')
ax.legend(loc='upper left',bbox_to_anchor=(1.01,1),fontsize=7)
fig.savefig(D/'I24_MECHANICAL_REVIEW.png',dpi=180)
fig.savefig(D/'I24_MECHANICAL_REVIEW.svg')
plt.close(fig)
manifest={p.name:sha(p)for p in O.iterdir()if p.is_file()}
(D/'GEOMETRY_FREEZE.json').write_text(json.dumps(dict(status='AUTHORED_REWORK_NOT_ACCEPTED',files=manifest,source_effectivity=binding,physical_measurements_claimed=False),indent=2)+'\n')
print(json.dumps(dict(source_pose_differences=differences,locator_states=len(states),locator_hits=hits,locator_nearest=loc['nearest'][:4],root_bearing=bearing,service_hits=loc['service_interferences'],elastic_end_demands=demands),indent=2))
