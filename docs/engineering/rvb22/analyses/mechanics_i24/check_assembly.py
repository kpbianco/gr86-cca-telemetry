"""Independent prism integration for authored free-span/carrier/service checks."""
from pathlib import Path
import json, sys, math
D=Path(__file__).resolve().parent;W=D.parents[1]
sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor')]
import numpy as np
import shapely as sh
from shapely.geometry import box, LineString, Point
from shapely.affinity import scale,translate
from shapely.ops import transform

O=W/'iterations/I24_thermal_mechanics/candidate_mechanics'
spec=json.loads((O/'CANDIDATE_SPEC.json').read_text())
frames=json.loads((D/'FRAME_RESULTS.json').read_text())
mated=json.loads((W/'analyses/mated_i22/RESULTS.json').read_text())

def volume(profile,extent,plan,z,plane='xz'):
    """Exact-to-polygon-quadrature product of two independent prism slices.

    On every polygon-vertex interval the two slice lengths are affine; their
    product is quadratic, integrated by two-point Gauss quadrature. Polygon
    approximation of curved geometry remains a separate modeling condition.
    """
    if plane=='yz':plan=transform(lambda x,y,z=None:(y,x),plan)
    a,b=extent
    pp=profile.intersection(box(-1e4,z[0],1e4,z[1]))
    plan=plan.intersection(box(-1e4,a,1e4,b))
    if pp.is_empty or plan.is_empty:return 0.
    lo=max(pp.bounds[0],plan.bounds[0]);hi=min(pp.bounds[2],plan.bounds[2])
    if hi<=lo:return 0.
    coords=np.r_[sh.get_coordinates(pp)[:,0],sh.get_coordinates(plan)[:,0],[lo,hi]]
    xx=np.unique(coords[(coords>=lo)&(coords<=hi)])
    mid=(xx[:-1]+xx[1:])/2;half=(xx[1:]-xx[:-1])/2
    gauss=np.r_[mid-half/math.sqrt(3),mid+half/math.sqrt(3)]
    lines=sh.linestrings(np.array([np.c_[gauss,np.full(len(gauss),-1e4)],np.c_[gauss,np.full(len(gauss),1e4)]]).transpose(1,0,2))
    f=sh.length(sh.intersection(pp,lines))*sh.length(sh.intersection(plan,lines))
    return float(np.sum(half*(f[:len(mid)]+f[len(mid):])))

# Analytic rectangular and triangular products verify the generic integration.
assert abs(volume(box(0,0,2,3),(0,5),box(0,0,2,5),(0,3))-30)<1e-10
triangle=sh.Polygon([(0,0),(2,0),(2,2)])
assert abs(volume(triangle,(0,5),box(0,0,2,5),(0,3))-10)<1e-10

wing=sh.from_wkt(frames['wing']['conservative_cold_frame_WKT'])
lower=translate(scale(wing,xfact=-1,yfact=1,origin=(0,0)),xoff=42)
central=sh.from_wkt(frames['central']['conservative_cold_frame_WKT'])
parts=[('W03_UPPER_FREE',wing,(14.2,53.8),'yz'),('W03_LOWER_FREE',lower,(14.2,53.8),'yz'),('T04_FREE',central,(12,28.7),'xz')]
obstacles=[('BASE',sh.from_wkt(spec['carrier_base_WKT']).buffer(.15),(-9,-6))]
for r in spec['carrier_solids']:
    obstacles.append((r['name'],sh.from_wkt(r['WKT']).buffer(.15),r['Z_mm']))
for r in spec['landings']:
    obstacles.append((r['name']+'_LANDING',box(*r['bounds_xy_mm']),r['Z_mm']))
results=[]
for n,p,e,axis in parts:
    for name,plan,z in obstacles:
        v=volume(p,e,plan,z,axis)
        # Free-envelope rounding/registration dilation crosses its intended
        # distal mating plane. Keep this visible instead of silently exempting it.
        results.append(dict(part=n,obstacle=name,overlap_mm3=v,
                            mating_plane_review=(n.startswith('W03_UPPER') and name=='UPPER_LANDING')or(n.startswith('W03_LOWER') and name=='LOWER_LANDING')or(n=='T04_FREE' and name=='CENTRAL_LANDING')))

service=[]
for row in mated['envelopes']:
    service.append((row['name'],box(*row['bounds_xy_mm']),row['z']))
cable=mated['pigtail']
service.append(('Adafruit851_full150mm_route',LineString(cable['centerline_mm']).buffer(1.5,quad_segs=64),(-3.6,-.6)))
service.append(('SMA_and_tool',box(*cable['bulkhead_and_tool_bounds_xy_mm']),(-9.5,10)))
service_checks=[]
for name,xy,z in service:
    for obname,plan,oz in obstacles:
        depth=max(0,min(z[1],oz[1])-max(z[0],oz[0]))
        service_checks.append(dict(service=name,obstacle=obname,overlap_mm3=float(xy.intersection(plan).area*depth)))
    # Free shape in the PCB frame applies to connector and cable reservations.
    for part,result,e,axis in [('W03_UPPER',frames['wing'],(14.2,53.8),'yz'),('T04',frames['central'],(12,28.7),'xz')]:
        p=sh.from_wkt(result['conservative_hot_frame_WKT'])
        service_checks.append(dict(service=name,obstacle=part,overlap_mm3=volume(p,e,xy,z,axis)))
        if part=='W03_UPPER':
            p=translate(scale(p,xfact=-1,yfact=1,origin=(0,0)),xoff=42)
            service_checks.append(dict(service=name,obstacle='W03_LOWER',overlap_mm3=volume(p,e,xy,z,axis)))
    shoe=box(*spec['retained_hot_contact_rectangles_mm'][-1]).buffer(.3)
    service_checks.append(dict(service=name,obstacle='T04_HOT_SHOE',overlap_mm3=float(xy.intersection(shoe).area*max(0,min(z[1],-.127)-max(z[0],-9.927)))))

out=dict(status='REVIEW_REQUIRED',integration_checks='rectangular30mm3 and triangular10mm3 exact',
         free_carrier_checks=results,free_carrier_hits=[r for r in results if r['overlap_mm3']>1e-6],
         service_checks=service_checks,service_hits=[r for r in service_checks if r['overlap_mm3']>1e-6],
         limitations=['Free sweep remains a conditional linear elastic model; no claim of actual collision or elastic validity.',
           'Mating-plane overlaps from conservative endpoint dilation require correlated joint geometry; no free-envelope exemption is hidden.',
           'Connector envelopes/150mm cable/SMA tool geometry retain I22 supplier and installed-position assumptions. PCB-to-carrier locating play is separately unresolved.'],
         physical_measurements_claimed=False)
(D/'ASSEMBLY_CLEARANCE.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({k:out[k]for k in ['status','free_carrier_hits','service_hits']},indent=2))
