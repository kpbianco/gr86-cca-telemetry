#!/usr/bin/env python3
"""T01 reviewable insulated PCB-back thermal strap. No native solid or heat-safety pass implied."""
from mechanics_runtime import *
from pathlib import Path
import json,math
from shapely.geometry import LineString,box
D=Path(__file__).resolve().parent
# Cross-section XZ; continuous formed foil packet, unbonded in the free span.
path=[(66.6,-1.227),(72,-1.227)]
path += [(72+3*math.cos(math.radians(a)),-4.227+3*math.sin(math.radians(a))) for a in range(90,181,3)]
path += [(69,-9.227)]
path += [(66+3*math.cos(math.radians(a)),-9.227+3*math.sin(math.radians(a))) for a in range(0,-91,-3)]
path += [(58,-12.227)]
shape=LineString(path).buffer(1.1,cap_style=2,join_style=1)
# Bonded terminal above free packet; open flex geometry must not become an adhesive-laminated solid.
poly=list(shape.exterior.coords)
scad='''// T01 thermal correction candidate, PCB bottomZ0. Package/copper heat-flow execution gate remains.
$fn=64;
// Copper packet is a volume envelope only:40 unbonded C110 laminae, each0.050..0.055mm.
module foil_packet(){translate([0,28.8,0])rotate([90,0,0])linear_extrude(17)polygon(points=POLY);}
color([.8,.4,.1])foil_packet();
//8805 insulating adhesive plus existing PCB soldermask; copper shall never touch live copper/vias.
color([1,1,1,.6])translate([66.6,11.8,-.127])cube([5.4,17,.127]);
//Chassis landing:mechanically attached to vehicle heat path; two M3 holes are below the PCB.
color("silver")difference(){translate([51,4,-15.327])cube([15.5,31.8,2]);for(y=[7.5,32.3])translate([54,y,-16])cylinder(d=3.4,h=4);}
//Dashed/transparent view envelopes are not additional hardware.
color([1,0,0,.08])translate([88.254766,-1.025,-18])cube([6,48.05,38]);
'''.replace('POLY',json.dumps(poly))
(D/'THERMAL_CONTACT_T01.scad').write_text(scad)
# With the thermal strap, remove only the two anchors covered beneath the carrier.
s=(D/'GR86_RVB_CARRIER_C03.scad').read_text().replace('[63,16],[63,22],','').replace('[63,30],[63,36]','[63,33],[63,39]').replace('translate([68.029766,-1.025,-6])cube([41.25,48.05,25])','translate([73.254766,-1,-15])cube([36,48,35])')
(D/'GR86_RVB_CARRIER_C03_THERMAL.scad').write_text(s)
A=(72-66.6)*(28.8-11.8);Rmask=.000030/(.2*A*1e-6);Rtape_typ=3.2/(A*.01);Rfoil=.034/(300*.017*.002);Rtotal=Rmask+4+Rfoil+1
r={'status':'SOURCE_CANDIDATE_WITH_ALLOCATED_ACCEPTANCE_BOUNDS','contact_mm':[66.6,11.8,72,28.8],'contact_area_mm2':A,'tape':{'MPN':'3M8805','nominal_mm':.127,'typical_impedance_C_cm2_W':3.2,'typical_R_K_W':Rtape_typ,'allocated_installed_R_K_W':4,'scope':'Includes published test contact impedance; installed mask/copper roughness, adhesion and hot aging are acceptance conditions, not guaranteed by typical data.'},'mask':{'max_thickness_um_allocation':30,'min_k_W_mK_allocation':.2,'R_K_W':Rmask},'foil':{'material':'C11000 annealed copper','lamina_count':40,'each_thickness_mm':[.050,.055],'min_total_mm':2,'width_mm':17,'max_developed_heat_path_mm':34,'reference_centerline_length_mm':LineString(path).length,'min_k_allocation_W_mK':300,'R_K_W':Rfoil,'max_packet_mass_g':.034*.017*.0022*8960*1e3,'free_span':'Unbonded laminae; weld/braze only terminal packets before attachment. No rigid epoxy across flex span.'},'terminal_and_landing_R_allocation_K_W':1,'total_R_K_W':Rtotal,'model_total_R_K_W':10,'landing_boundary_C':70,'RF':{'antenna_start_X_mm':88.254766,'metal_max_X_mm':72,'module_and_cut_position_allocation_mm':.3,'free_span_bow_allocation_mm':.5,'minimum_clearance_mm':88.254766-72-.3-.5,'requirement_mm':15,'scope':'Revision of broader legacy exclusion supported geometrically by official Espressif15mm guidance; this does not prove unchanged BLE performance.'},'basic_thermal_assembly_bbox_mm':[108.454766,73,36.50],'removed_anchor_centres_mm':[[63,16],[63,22]],'retained_anchor_centres_mm':[[-3.5,31],[-3.5,37],[63,33],[63,39]],'checks':{'budget_at_most10K_W':Rtotal<=10,'nominal_centerline_under34mm':LineString(path).length<=34,'RF15mm_with_bow':88.254766-72-.3-.5>=15,'free_vertical_bundle_carrier_gap_after1mm':69-1.1-66.5-1>0},'remaining_execution_gate':'Actual filled-copper/interlayer/package thermal model and native carrier/component solid collision execution. Whole-assembly heat safety is not passed by this candidate.'}
(D/'THERMAL_CONTACT_T01_SPEC.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({'checks':r['checks'],'R_total':Rtotal,'length':LineString(path).length,'RFclearance':r['RF']['minimum_clearance_mm']}))
