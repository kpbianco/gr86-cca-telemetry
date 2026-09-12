"""C02 shared-shoe and finite-sink thermal network; board temperatures only."""
import numpy as np
import shapely as sh
from scipy.sparse import coo_matrix, bmat, diags
from scipy.sparse.linalg import spsolve, cg
from shapely.ops import unary_union

def solve_network(m, contacts, sink_R=None, landing=70., ambient=65., save_map=None):
    """Each contact has local interface R, ONE shared shoe, then ONE bridge to a common sink.

    sink_R=None: prescribed landing temperature. Positive sink_R: sink connects
    through that resistance to bulk ambient. No thermal credit for a floating plate.
    """
    N=m["N"];nb=4*N;size=nb+len(contacts)+1;sink=size-1
    if sink_R is not None and (not np.isfinite(sink_R) or sink_R<=0):
        raise ValueError("Finite sink resistance must be positive")
    base=bmat([[m["base"],None],[None,coo_matrix((size-nb,size-nb))]],format="csr")
    rr=[];cc=[];vv=[];diag=np.zeros(size)
    areas_contact=[];occluders=[m["carrier"]]
    def edge(i,j,g):
        if g<=0:raise ValueError("Positive conductance required")
        rr.extend([i,j]);cc.extend([j,i]);vv.extend([-g,-g]);diag[i]+=g;diag[j]+=g
    for k,contact in enumerate(contacts):
        ri=contact["interface_R"];rb=contact["bridge_R"]
        if min(ri,rb)<=0:raise ValueError("Both path resistances must be positive")
        geom=sh.from_wkt(contact["wkt"]);a=m["overlap"](geom);total=a.sum()
        if total<=0:raise ValueError("Contact misses board")
        for ix in np.flatnonzero(a):edge(3*N+int(ix),nb+k,a[ix]/total/ri)
        edge(nb+k,sink,1/rb);areas_contact.append(a);occluders.append(geom)
    base=base+coo_matrix((vv,(rr,cc)),shape=(size,size)).tocsr()+diags(diag)
    q=np.zeros(size);q[:nb]=m["q"]
    air_area=np.zeros(size);air_area[:N]=m["areas"]
    # Remove union once: a contact cutout in the carrier cannot remove area twice.
    air_area[3*N:4*N]=np.maximum(m["areas"]-m["overlap"](unary_union(occluders)),0)
    Ta=ambient+273.15;H=m["height_m"]
    def loss(temp):
        film=(temp+273.15+Ta)/2
        mu=1.716e-5*(film/273)**1.5*(273+111)/(film+111)
        k=.0241*(film/273)**1.5*(273+194)/(film+194);rho=70000/(287.05*film)
        nu=mu/rho;alpha=k/(rho*1007);Pr=mu*1007/k
        Ra=9.80665/film*np.abs(temp-ambient)*H**3/(nu*alpha)
        Nu=(.825+.387*np.maximum(Ra,1e-8)**(1/6)/(1+(.492/Pr)**(9/16))**(8/27))**2
        tk=temp+273.15
        return (.7*Nu*k/H+.3*5.670374419e-8*(tk+Ta)*(tk*tk+Ta*Ta))*air_area
    fixed=sink_R is None
    if fixed:
        free=np.arange(size-1);A=base[:-1,:-1].tocsr()
        source=q[:-1]-base[:-1,-1].toarray().ravel()*landing
        T=np.full(size,ambient+20);T[-1]=landing
    else:
        sink_diag=np.zeros(size);sink_diag[-1]=1/sink_R
        A=base+diags(sink_diag);source=q+sink_diag*ambient
        free=np.arange(size);T=np.full(size,ambient+20)
    pre=None;linear=[]
    if len(free)>100000:
        import pyamg
        pre=pyamg.smoothed_aggregation_solver(A+diags(loss(T)[free]),symmetry="symmetric",max_coarse=200).aspreconditioner()
    for it in range(100):
        l=loss(T)[free]
        r=A@T[free]+l*(T[free]-ambient)-source
        eps=.001
        slope=(loss(T+eps)[free]*(T[free]+eps-ambient)-loss(T-eps)[free]*(T[free]-eps-ambient))/(2*eps)
        J=A+diags(slope)
        if pre is None:delta=spsolve(J,r)
        else:
            delta,info=cg(J,r,M=pre,rtol=1e-11,atol=1e-13,maxiter=3000)
            if info!=0:raise RuntimeError("Linear iteration did not converge")
        T[free]-=delta
        if not np.all(np.isfinite(T)):raise RuntimeError("Nonfinite temperature")
        linear.append(float(np.max(np.abs(J@delta-r))))
        if np.max(np.abs(delta))<1e-7:break
    residual=A@T[free]+loss(T)[free]*(T[free]-ambient)-source
    bridge_heat=[float((T[nb+k]-T[sink])/c["bridge_R"]) for k,c in enumerate(contacts)]
    to_sink=sum(bridge_heat);air=float(np.sum(loss(T)*(T-ambient)))
    energy=to_sink+air-float(q.sum())
    if abs(energy)>1e-6 or np.max(np.abs(residual))>1e-7:
        raise RuntimeError("Thermal balance failed")
    if not fixed and abs(to_sink-(T[sink]-ambient)/sink_R)>1e-6:
        raise RuntimeError("Sink energy balance failed")
    boardT=T[:nb];hottest=int(np.argmax(boardT))
    result={"mesh_mm":m["step"],"nodes":size,"boundary":"prescribed_landing" if fixed else "finite_sink_to_air",
        "sink_R_K_W":sink_R,"sink_C":float(T[sink]),"bulk_air_C":ambient,
        "max_board_C":float(boardT.max()),"min_board_C":float(boardT.min()),
        "hotspot":{"layer":hottest//N,"x_mm":float(m["x"][hottest%N]),"y_mm":float(m["y"][hottest%N])},
        "source_region_mean_C":{},"contact_shoe_C":{},"contact_heat_W":{},
        "heat_W":float(q.sum()),"heat_to_sink_W":to_sink,"heat_to_air_W":air,
        "energy_balance_residual_W":energy,"max_nodal_residual_W":float(np.max(np.abs(residual))),
        "iterations":it+1,"max_linear_residual_W":max(linear),
        "exposed_bottom_area_mm2":float(air_area[3*N:4*N].sum()*1e6),
        "package_temperature_proven":False,"physical_test":False,
        "shared_contact_nodes":True,"occlusion_uses_union":True,
        "disconnected_copper_cells":m["disconnected_copper_cells"]}
    for name,p,g in m["heat"]:
        a=m["overlap"](g);l=m["source_layers"].get(name,0)
        result["source_region_mean_C"][name]=float(np.sum(boardT[l*N:(l+1)*N]*a)/a.sum())
    for k,c in enumerate(contacts):
        result["contact_shoe_C"][c["name"]]=float(T[nb+k])
        result["contact_heat_W"][c["name"]]=bridge_heat[k]
    if save_map:
        np.savez_compressed(save_map,x=m["x"],y=m["y"],T=boardT.reshape(4,N),area=m["areas"],shoe_and_sink=T[nb:])
    return result
