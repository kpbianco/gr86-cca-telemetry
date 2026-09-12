"""Source-controlled four-layer face-conductance FVM; not a junction-temperature model.
Adapted from the project's I18 solver. Inputs are assigned by thermal_native.py.
No guessed fill, dynamic source replacement, or fake pyamg module is used.
"""
from pathlib import Path
import math
import numpy as np
import shapely as sh
from shapely.geometry import Polygon, Point, LineString, box
from shapely.ops import unary_union
from scipy.sparse import coo_matrix, diags
from scipy.sparse.linalg import spsolve, cg
board = None
carrier = None
heat = None
G = None
def only_dimension(g, dim):
    if g.is_empty:return Polygon() if dim==2 else LineString()
    if (dim==2 and g.geom_type=='Polygon') or (dim==1 and g.geom_type in ('LineString','LinearRing')):return g
    if hasattr(g,'geoms'):return unary_union([only_dimension(q,dim) for q in g.geoms])
    return Polygon() if dim==2 else LineString()

def solve(extracted, step=1., mode='provisional', plating_um=20., heat_scale=1., landing=70., ambient=65., save_map=False, method='newton', contact_box=(67.5,12.8,71.3,27.9), contact_R=13.7663529685, wing_R=5., edge_mode='face', wing_boxes=((15,-7.6,53,-1.6),(15,43.6,53,49.6)), extra_contacts=()):
    explicit, provisional, vias, drilled, cu, diel, source_layers, meta = extracted
    masks = provisional if mode=='provisional' else explicit if mode=='explicit_only' else [board]*4
    a,b,c,d=board.bounds
    x,y=np.meshgrid(np.arange(a,c,step), np.arange(b,d,step));x=x.ravel();y=y.ravel()
    assert all(lo==0 and hi==3 for xy,drill,lo,hi in vias), 'Blind holes need layer-specific material masks'
    material=board.difference(drilled)
    raw_cells=sh.box(x,y,x+step,y+step);cells=np.empty(len(x),dtype=object)
    for yy in np.unique(y):
        ix=np.flatnonzero(y==yy);strip=material.intersection(box(a-1,yy,c+1,yy+step))
        cells[ix]=sh.intersection(raw_cells[ix],strip)
    # Polygon intersection is associative. Compare a deterministic spatial sample
    # against direct full-board intersection and conserve the total material area.
    selected=np.linspace(0,len(x)-1,min(257,len(x)),dtype=int)
    direct=sh.intersection(raw_cells[selected],material)
    errors=sh.area(sh.symmetric_difference(cells[selected],direct))
    ar=sh.area(cells)
    assert max(errors)<1e-8 and abs(sum(ar)-material.area)<1e-5,(max(errors),sum(ar),material.area)
    print('Initial mesh clipped by row;257 direct cells and total material area verified',flush=True)
    good=ar>1e-6
    x=x[good];y=y[good];cells=np.array([only_dimension(q,2) for q in cells[good]],dtype=object);ar=ar[good]*1e-6; N=len(ar)
    ids={(round((xx-a)/step),round((yy-b)/step)):i for i,(xx,yy) in enumerate(zip(x,y))}
    def overlap(g):
        # Clip complex copper masks by mesh row before per-cell intersection.
        # This preserves the exact area operation while avoiding repeated traversal
        # of every distant route vertex for every cell.
        result=np.zeros(N)
        for yy in np.unique(y):
            ix=np.flatnonzero(y==yy)
            strip=g.intersection(box(a-1,yy,c+1,yy+step))
            if not strip.is_empty:result[ix]=sh.area(sh.intersection(cells[ix],strip))*1e-6
        return result
    solid=ar.copy()
    copper_cells=[]
    for mask in masks:
        gg=np.empty(N,dtype=object)
        for yy in np.unique(y):
            ix=np.flatnonzero(y==yy);strip=mask.intersection(box(a-1,yy,c+1,yy+step))
            gg[ix]=[only_dimension(q,2) for q in sh.intersection(cells[ix],strip)]
        copper_cells.append(gg)
    frac=np.array([sh.area(gg)*1e-6/ar for gg in copper_cells])
    disconnected_copper_cells=sum(sum(1 for g in gg if g.geom_type=='MultiPolygon' and len(g.geoms)>1) for gg in copper_cells)
    print('Mesh geometry ready: '+str(N)+' cells per layer; '+str(disconnected_copper_cells)+' multicomponent copper cells',flush=True)
    # Dielectric assigned to adjoining sheet halves. SI sheet conductance W/K per square.
    assigned=np.array([diel[0]/2,(diel[0]+diel[1])/2,(diel[1]+diel[2])/2,diel[2]/2])
    ks=300*cu[:,None]*frac+.25*assigned[:,None]*(solid/ar)
    rows=[];cols=[];data=[];diag=np.zeros(4*N)
    def edge(i,j,g):
        if g<=0:return
        diag[i]+=g;diag[j]+=g;rows.extend([i,j]);cols.extend([j,i]);data.extend([-g,-g])
    pairs=[(i,ids[k2]) for key,i in ids.items() for k2 in [(key[0]+1,key[1]),(key[0],key[1]+1)] if k2 in ids]
    for start in range(0,len(pairs),4096):
        ij=np.asarray(pairs[start:start+4096]);ii,jj=ij[:,0],ij[:,1]
        faces=sh.intersection(cells[ii],cells[jj],grid_size=1e-7)
        bad=np.flatnonzero(~np.isin(sh.get_type_id(faces),[1,2,5]))
        for k in bad:faces[k]=only_dimension(faces[k],1)
        shared=sh.length(faces)
        for l in range(4):
            if edge_mode=='area':
                kval=2*ks[l,ii]*ks[l,jj]/np.maximum(ks[l,ii]+ks[l,jj],1e-30)
                gg=kval*shared/step
            else:
                fi=sh.intersection(copper_cells[l][ii],faces,grid_size=1e-7)
                fj=sh.intersection(copper_cells[l][jj],faces,grid_size=1e-7)
                for arr in (fi,fj):
                    for k in np.flatnonzero(~np.isin(sh.get_type_id(arr),[1,2,5])):arr[k]=only_dimension(arr[k],1)
                widths=sh.length(sh.intersection(fi,fj,grid_size=1e-7))
                gg=(300*cu[l]*widths+.25*assigned[l]*shared)/step
            good_edges=gg>0;aa=l*N+ii[good_edges];bb=l*N+jj[good_edges];gg=gg[good_edges]
            np.add.at(diag,aa,gg);np.add.at(diag,bb,gg)
            rows.extend(aa.tolist());rows.extend(bb.tolist());cols.extend(bb.tolist());cols.extend(aa.tolist());data.extend((-gg).tolist());data.extend((-gg).tolist())
    print('In-plane conductance assembled',flush=True)
    dz=diel+(cu[:-1]+cu[1:])/2
    gv=np.array([.25*solid/z for z in dz])
    t=plating_um*1e-6
    cell_tree=sh.STRtree(cells)
    assigned_barrel_area=0.;theoretical_barrel_area=0.
    for xy,drill,lo,hi in vias:
        if t==0:continue
        annulus=Point(*xy).buffer(drill/2+t*1000,quad_segs=64).difference(Point(*xy).buffer(drill/2,quad_segs=64))
        ix=cell_tree.query(annulus,predicate='intersects')
        areas_annulus=sh.area(sh.intersection(cells[ix],annulus))*1e-6
        area=math.pi*((drill*.0005+t)**2-(drill*.0005)**2)
        assigned_barrel_area+=sum(areas_annulus);theoretical_barrel_area+=area
        for l in range(lo,hi):gv[l,ix]+=300*areas_annulus/dz[l]
    for l in range(3):
        for i in range(N):edge(l*N+i,(l+1)*N+i,gv[l,i])
    base=coo_matrix((data,(rows,cols)),shape=(4*N,4*N)).tocsr()+diags(diag)
    q=np.zeros(4*N)
    for name,p,g in heat:
        v=overlap(g);assert sum(v)>0;l=source_layers.get(name,0)
        q[l*N:(l+1)*N]+=p*(4.815/4.79)*heat_scale*v/sum(v)
    exposed=np.zeros((4,N));exposed[0]=ar;exposed[3]=np.maximum(ar-overlap(carrier),0)
    gc=np.zeros((4,N));contact=box(*contact_box)
    for g,r in [(contact,contact_R),(box(*wing_boxes[0]),wing_R),(box(*wing_boxes[1]),wing_R), *extra_contacts]:
        if r is None:continue
        v=overlap(g)
        if sum(v) <= 0 or r <= 0: raise ValueError('Contact must intersect board and have positive total resistance')
        gc[3]+=v/(sum(v)*r);exposed[3]=np.maximum(exposed[3]-v,0)
    gc=gc.ravel();areas=exposed.ravel();T=np.full(4*N,ambient+20);Ta=ambient+273.15;H=G['board_size_mm'][1]*.001
    def film_loss(temp):
        film=(temp+273.15+Ta)/2
        mu=1.716e-5*(film/273)**1.5*(273+111)/(film+111)
        k=.0241*(film/273)**1.5*(273+194)/(film+194);rho=70000/(287.05*film)
        nu=mu/rho;alpha=k/(rho*1007);Pr=mu*1007/k
        Ra=9.80665/film*np.abs(temp-ambient)*H**3/(nu*alpha)
        Nu=(.825+.387*np.maximum(Ra,1e-8)**(1/6)/(1+(.492/Pr)**(9/16))**(8/27))**2
        tk=temp+273.15; h=.7*Nu*k/H+.3*5.670374419e-8*(tk+Ta)*(tk*tk+Ta*Ta)
        return h*areas
    preconditioner=None;linear_residuals=[]
    if 4*N>350000:
        try: import pyamg
        except ImportError as e: raise RuntimeError('This mesh needs pyamg; install it or use <=350000 nodes. No fake AMG fallback.') from e
        film0=film_loss(T)
        amg=pyamg.smoothed_aggregation_solver(base+diags(film0+gc),symmetry='symmetric',max_coarse=200)
        preconditioner=amg.aspreconditioner()
        print('AMG hierarchy ready',flush=True)
    def linear_solve(A,b):
        if preconditioner is None:return spsolve(A,b)
        v,info=cg(A,b,M=preconditioner,rtol=1e-11,atol=1e-13,maxiter=2000)
        residual=float(np.max(np.abs(A@v-b)));linear_residuals.append({'info':int(info),'max_abs_residual':residual})
        assert info==0 and residual<1e-8,linear_residuals[-1]
        return v
    for it in range(150):
        loss=film_loss(T);rhs=q+ambient*loss+landing*gc
        if method=='newton':
            eps=.001
            slope=(film_loss(T+eps)*(T+eps-ambient)-film_loss(T-eps)*(T-eps-ambient))/(2*eps)
            residual=base@T+(loss+gc)*T-rhs
            new=T-linear_solve(base+diags(slope+gc),residual)
        else:
            new=linear_solve(base+diags(loss+gc),rhs)
        assert np.all(np.isfinite(new)), 'Nonfinite thermal solution; no pass permitted'
        diff=float(max(abs(new-T)));T=new if method=='newton' else .5*(new+T)
        if diff<1e-7:break
    loss=film_loss(T);rhs=q+ambient*loss+landing*gc
    residual=base@T+(loss+gc)*T-rhs
    assert diff<1e-6 and max(abs(residual))<1e-7
    back=T[3*N:];cap_layer=T[:N];probe=overlap(box(78.35,-6.15,85.65,-1.85))
    result={'mesh_mm':step,'mode':mode,'contact_box':list(contact_box),'contact_R':contact_R,'wing_R':wing_R,'edge_mode':edge_mode,'solver':method,'plating_um_condition':plating_um,'heat_scale':heat_scale,'nodes':4*N,
            'C206_board_region_max_C':float(max(cap_layer[probe>0])), 'C206_board_region_mean_C':float(sum(cap_layer*probe)/sum(probe)),
            'max_board_C':float(max(T)),'min_board_C':float(min(T)),'source_region_mean_C':{n:float(sum(T[source_layers.get(n,0)*N:(source_layers.get(n,0)+1)*N]*overlap(g))/sum(overlap(g))) for n,p,g in heat},
            'heat_W':float(sum(q)), 'heat_to_landing_W':float(sum(gc*(T-landing))), 'heat_to_air_W':float(sum(loss*(T-ambient))),
            'energy_balance_residual_W':float(sum(gc*(T-landing))+sum(loss*(T-ambient))-sum(q)),
            'max_nodal_residual_W':float(max(abs(residual))),'linear_solver_residuals':linear_residuals,'iterations':it+1,'last_iteration_change_K':diff,
            'max_front_back_difference_K':float(max(abs(T[:N]-back))), 'package_temperature_proven':False,'disconnected_copper_cells':int(disconnected_copper_cells),'wing_boxes':wing_boxes,'face_overlay_precision_mm':1e-7,
            'barrel_area_capture_ratio':float(assigned_barrel_area/theoretical_barrel_area) if theoretical_barrel_area else None}
    if save_map:np.savez_compressed(Path(save_map),x=x,y=y,area=ar,T=T.reshape(4,N),copper_fraction=frac)
    return result

