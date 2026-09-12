"""Check contact XY envelopes against the exact filled board and retained RF allocation."""
import argparse, json, sys
from pathlib import Path
D=Path(__file__).resolve().parent
sys.path.insert(0,str(D.parent/"convergence_01"))
import thermal_native as tn
import shapely as sh
import numpy as np
from shapely.geometry import box
from shapely.ops import unary_union
def check(pcb):
    extracted,b=tn.extract_native(pcb);c=tn.c
    cfg=json.loads((D/"CONTACT_PROPOSALS.json").read_text())
    footprints={}
    for f in c.child(b,"footprint"):
        if c.get(f,"layer")!=["B.Cu"] or c.prop(f).get("Assembly") not in ("FACTORY","MANUAL_GPS"):continue
        pts=[];x,y,*a=c.get(f,"at");ang=np.deg2rad(a[0] if a else 0)
        for item in c.child(f,"fp_line")+c.child(f,"fp_rect"):
            if c.get(item,"layer")!=["B.CrtYd"]:continue
            ends=[c.get(item,k) for k in ("start","end")]
            if str(item[0])=="fp_rect":
                (u0,v0),(u1,v1)=ends;ends=[(u0,v0),(u0,v1),(u1,v0),(u1,v1)]
            pts += [(x+u*np.cos(ang)+v*np.sin(ang),y-u*np.sin(ang)+v*np.cos(ang)) for u,v in ends]
        if not pts:raise ValueError("Missing bottom courtyard "+c.prop(f)["Reference"])
        footprints[c.prop(f)["Reference"]]=sh.MultiPoint(pts).convex_hull
    mech=json.loads((D.parents[1]/"current/mechanics/I24_trial/CANDIDATE_SPEC.json").read_text())
    rows=[];checks=0
    for name,record in cfg["proposal_details"].items():
        g=sh.from_wkt(record["wkt"])
        distances={ref:float(g.distance(f)) for ref,f in footprints.items()};checks+=len(distances)
        if min(distances.values())<.5-1e-8:raise ValueError("Bottom courtyard clearance failed")
        setback=mech["RF_antenna_start_X_mm"]-g.bounds[2]
        if setback-.3<mech["RF_requirement_mm"]:raise ValueError("Retained RF setback failed")
        if abs(g.area-record["area_mm2"])>1e-8:raise ValueError("Area mismatch")
        A=g.area*1e-6;ri=(25e-6/.2+.5e-3/6+2/10000+2e-3/300)/A
        if abs(ri-record["interface_R"])>1e-10:raise ValueError("Interface path budget mismatch")
        rows.append({"contact":name,"setback_mm":setback,"minimum_courtyard_gap_mm":min(distances.values()),
            "checks":len(distances),"carrier_base_overlap_mm2":g.intersection(tn.model.carrier).area})
    old,oldspec=tn.contact_screen(b)
    return {"status":"PASS_SCOPED_XY_NOT_FULL_MECHANICAL_ACCEPTANCE","rows":rows,"courtyard_checks":checks,
        "C01_setback_mm":mech["RF_antenna_start_X_mm"]-old.bounds[2],
        "C01_fails_retained_15mm_XY_screen":mech["RF_antenna_start_X_mm"]-old.bounds[2]<15,
        "source_binding":extracted[-1],"solid_bridge_preload_clearance_reviewed":False,
        "installed_RF_test":False,"board_changed":False}
if __name__=="__main__":
    p=argparse.ArgumentParser();p.add_argument("--pcb",type=Path,required=True);p.add_argument("--out",type=Path,required=True)
    a=p.parse_args();r=check(a.pcb);a.out.parent.mkdir(parents=True,exist_ok=True)
    a.out.write_text(json.dumps(r,indent=2)+"\n");print(json.dumps(r,indent=2))
