#!/usr/bin/env python3
"""Run the inherited face-conductance model on verified native copper, explicitly.

Usage: python thermal_native.py --native-dir path/to/native_I06_hosted --mesh 0.5 --out results/thermal
This computes board-region temperatures, NOT semiconductor junction temperatures.
"""
from __future__ import annotations
import argparse, hashlib, json, sys
from pathlib import Path
import numpy as np
import shapely as sh
from shapely.geometry import Polygon, Point, box
from shapely.ops import unary_union
D=Path(__file__).resolve().parent
sys.path.insert(0,str(D/'support'))
import check_combined_copper as c
import thermal_fvm as model

SOURCE_HASH='5b373f6033fdbd18f126f8ca054b619abada2568778c5a8fc303c4e756fb06c8'
FILLED_HASH='11533ea91c3bc4c61dd7066e0001914a72bc90b27afb38d321c43b8feb90e3b1'
def digest(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def extract_native(pcb:Path):
    if digest(pcb)!=FILLED_HASH: raise ValueError('Native copper differs from the reviewed I24 candidate; rebind explicitly before analysis')
    b,items,unsupported=c.collect(pcb)
    if unsupported: raise ValueError(unsupported)
    geom=json.loads((D/'THERMAL_GEOMETRY.json').read_text()); model.G=geom['outline']
    model.board=Polygon(model.G['outline_mm'])
    for xy in model.G['mount_centres_mm']: model.board=model.board.difference(Point(*xy).buffer(2.2,quad_segs=64))
    model.carrier=sh.from_wkt(geom['carrier_base_WKT'])
    model.heat=[('U201',2.58225,box(77.094766,19.55,80.994766,23.45)),('U151',.6455358313,box(33.3,10.5,36.7,15.5)),('U121',1.1662141687,box(45.525,24.05,48.475,28.95)),('U403',.101,box(37.2,4.5,40.8,7.5)),('U401',.165,box(46.5,1,62.5,17)),('other5V_allocation',.13,box(17,7,24,12))]
    layers=c.child(c.child(c.child(b,'setup')[0],'stackup')[0],'layer')
    cu=np.array([c.get(next(x for x in layers if x[1]==l),'thickness')[0]for l in c.L])*1e-3
    diel=np.array([c.get(x,'thickness')[0]for x in layers if str(x[1]).startswith('dielectric')])*1e-3
    fps=c.child(b,'footprint'); source_layers={c.prop(f)['Reference']:c.L.index(c.get(f,'layer')[0])for f in fps}
    vias=[];holes=[]
    for v in c.child(b,'via'):
        xy=c.get(v,'at')[:2];dr=c.get(v,'drill')[0];lo,hi=sorted(c.L.index(x)for x in c.get(v,'layers'))
        vias.append((xy,dr,lo,hi));holes.append(Point(*xy).buffer(dr/2,quad_segs=64))
    for f in fps:
        for pad in c.child(f,'pad'):
            if str(pad[2])!='thru_hole':continue
            dr=c.get(pad,'drill'); assert len(dr)==1 and isinstance(dr[0],(int,float))
            xy=list(c.pad_shape(f,pad).centroid.coords)[0];vias.append((xy,dr[0],0,3));holes.append(Point(*xy).buffer(dr[0]/2,quad_segs=64))
    drilled=unary_union(holes)
    explicit=[unary_union([x['geometry']for x in items if x['layer']==l]).difference(drilled).intersection(model.board)for l in c.L]
    fills={l:[]for l in c.L}; count=0
    for zone in c.child(b,'zone'):
        for fill in c.child(zone,'filled_polygon'):
            layer=c.get(fill,'layer')[0]
            if layer not in fills:continue
            shape=sh.make_valid(Polygon([x[1:]for x in c.child(c.child(fill,'pts')[0],'xy')]))
            fills[layer].append(shape);count+=1
    if not count:raise ValueError('Unfilled PCB: no native polygons; approximate fill is not substituted')
    native=[unary_union([explicit[i],*fills[l]]).difference(drilled).intersection(model.board)for i,l in enumerate(c.L)]
    meta={'source_PCB_sha256':SOURCE_HASH,'native_PCB_sha256':digest(pcb),'native_polygons':count,'barrels':len(vias),'copper_m':cu.tolist(),'dielectric_m':diel.tolist(),'native_geometry':True,'provisional_fill':False}
    # The inherited solver's second mask slot is named provisional; the actual arrays here are native only.
    return (explicit,native,vias,drilled,cu,diel,source_layers,meta),b

def contact_screen(b):
    """A notched insulated contact trial; never a bare mask-opening proposal."""
    g=box(73.8,16.3,84.0,25.5); exclusions=[]
    # Native courtyard outlines, expanded 0.5 mm for XY location and shoe machining.
    for f in c.child(b,'footprint'):
        if c.get(f,'layer')!=['B.Cu'] or c.prop(f).get('Assembly') not in ('FACTORY','MANUAL_GPS'):continue
        pts=[]; x,y,*a=c.get(f,'at');angle=np.deg2rad(a[0]if a else 0)
        for line in c.child(f,'fp_line') + c.child(f,'fp_rect'):
            if c.get(line,'layer')!=['B.CrtYd']:continue
            endpoints=[c.get(line,key) for key in ('start','end')]
            if str(line[0])=='fp_rect':
                (u1,v1),(u2,v2)=endpoints;endpoints=[(u1,v1),(u1,v2),(u2,v1),(u2,v2)]
            for u,v in endpoints:
                pts.append((x+u*np.cos(angle)+v*np.sin(angle),y-u*np.sin(angle)+v*np.cos(angle)))
        if not pts:raise ValueError('Missing bottom courtyard '+c.prop(f)['Reference'])
        bounds=sh.MultiPoint(pts).convex_hull.buffer(.5,join_style='mitre')
        if g.intersects(bounds):exclusions.append({'ref':c.prop(f)['Reference'],'keepout_WKT':bounds.wkt})
        g=g.difference(bounds)
    if g.is_empty or g.geom_type!='Polygon':raise ValueError('Contact trial fragmented or empty')
    area=g.area*1e-6
    # Engineering bounds, not purchased-material certificates; two contacts are counted explicitly.
    terms={'solder_mask':25e-6/(.2*area),'insulating_TIM_bulk':.5e-3/(6*area),'two_interfaces':2/(1e4*area),'copper_through_thickness':2e-3/(300*area),'axial_bridge_25x12x3_mm':.025/(300*.012*.003),'cold_joint':.5,'spreading_allowance':1.0}
    return g,{'contact_WKT':g.wkt,'area_mm2':g.area,'courtyard_exclusions':exclusions,'path_terms_K_W':terms,'total_to_landing_K_W':sum(terms.values()),'status':'INSULATED_CONTACT_FEASIBILITY_NOT_ADOPTED','electrical':'Retain solder mask and independent insulating TIM; no bare copper land or mask opening. Copper trace under contact is not shorted by design.','remaining':['Selected TIM conductivity, dielectric and pressure data','Preload/load-path and carrier-window redesign','Full swept solid/cable/service geometry','BLE antenna metal proximity review','Actual complete cold-landing construction <=70C'],'all_properties_are_allocations':True}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--native-dir',type=Path,required=True);ap.add_argument('--mesh',type=float,default=.5);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--case',choices=['baseline','trial','both'],default='both');a=ap.parse_args()
    a.out.mkdir(parents=True,exist_ok=True)
    ex,b=extract_native(a.native_dir/'candidate_kicad/GR86_CCA_RevB.kicad_pcb');contact,spec=contact_screen(b)
    (a.out/'CONTACT_TRIAL.json').write_text(json.dumps(spec,indent=2)+'\n')
    report={'status':'BOARD_MODEL_ONLY','extraction':ex[-1],'results':[],'package_temperature_proven':False,'release':False}
    for label,contacts in [('baseline',()),('trial',[(contact,spec['total_to_landing_K_W'])])]:
        if a.case!='both' and a.case!=label:continue
        r=model.solve(ex,step=a.mesh,mode='provisional',plating_um=15.,contact_R=15.,wing_R=5.,extra_contacts=contacts,save_map=a.out/f'{label}_{a.mesh}.npz')
        r['mode']='native_filled';r['case']=label;r['additional_contact_total_K_W']=spec['total_to_landing_K_W']if contacts else None
        report['results'].append(r);(a.out/'RESULTS.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(r),flush=True)
if __name__=='__main__':main()
