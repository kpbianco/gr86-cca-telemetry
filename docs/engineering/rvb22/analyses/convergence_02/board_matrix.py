"""C02 matrix assembly inherited from C01; no changed heat or native-copper geometry.
The returned matrix excludes convection, contact and sink boundaries.
Source-controlled four-layer face-conductance FVM; not a junction-temperature model.
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

def prepare(extracted, step=1., mode='provisional', plating_um=20., heat_scale=1., landing=70., ambient=65., save_map=False, method='newton', contact_box=(67.5,12.8,71.3,27.9), contact_R=13.7663529685, wing_R=5., edge_mode='face', wing_boxes=((15,-7.6,53,-1.6),(15,43.6,53,49.6)), extra_contacts=()):
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
    return {"base":base, "q":q, "N":N, "areas":ar, "cells":cells,
            "x":x, "y":y, "frac":frac, "overlap":overlap, "heat":heat,
            "carrier":carrier, "board":board, "height_m":G["board_size_mm"][1]*.001,
            "source_layers":source_layers, "step":step,
            "disconnected_copper_cells":int(disconnected_copper_cells),
            "barrel_area_capture_ratio":float(assigned_barrel_area/theoretical_barrel_area)}
