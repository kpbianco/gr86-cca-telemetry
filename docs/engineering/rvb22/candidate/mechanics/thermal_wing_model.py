#!/usr/bin/env python3
from thermal_network import *
from scipy.optimize import brentq
import itertools
rows=[]
for P,e,p,feed in itertools.product([1.070,1.642,2.214,4.790,5.942],[.3,.6,.9],[70000,85000,101325],[10,50,100]):rows.append({'Pboard_W':P,'epsilon':e,'pressure_Pa':p,'correlation_multiplier':.7,'lateral_feed_R_each_K_W':feed,**solve(P,e,p,.7,feed)})
vent=[]
for P,p in itertools.product([1.070,1.642,2.214,4.790,5.942],[70000,85000,101325]):
 rho=air(338.15,p)[4];dT=(P/(rho*1007*.5*.004*math.sqrt(2*9.80665*.15/338.15)))**(2/3)
 for e in [.6,.9]:vent.append({'Pboard_W':P,'epsilon':e,'pressure_Pa':p,'upper_outlet_air_rise_K':dT,'lateral_feed_R_each_K_W':100,**solve(P,e,p,.7,100,65+dT,65)})
req=[]
for p,e,target in itertools.product([70000,85000,101325],[.6,.9],[103,105]):
 rho=air(338.15,p)[4]
 def outlet(a):return (2.214/(rho*1007*.5*a*math.sqrt(2*9.80665*.15/338.15)))**(2/3)
 def body(a):return solve(2.214,e,p,.7,100,65+outlet(a),65)['body_average_plus10K_C']
 if body(1)>target:req.append({'pressure_Pa':p,'epsilon':e,'body_target_C':target,'result':'No useful finite opening reaches target under these boundaries.'})
 else:
  a=brentq(lambda a:body(a)-target,1e-5,1,xtol=1e-10);req.append({'pressure_Pa':p,'epsilon':e,'body_target_C':target,'each_effective_free_opening_mm2':a*1e6,'stack_height_mm':150,'effective_Cd':.5,'outlet_rise_K':outlet(a),'check_body_C':body(a)})
checks={'existing_and_wing_polygons_disjoint_area':all(pcb.intersection(w).area<1e-6 for w in [wt,wl]),'radiative_view_sums_below_one':bool(np.all(F.sum(axis=1)<1)),'view_reciprocity':bool(np.max(abs(A[:,None]*F-A[None,:]*F.T))<1e-12),'weighted_view_quadrature_converged0p5percent':max(abs((x-y)/y) for x,y in zip(views[0]['exchange_areas_m2'],views[1]['exchange_areas_m2']))<.005,'all135steady_energy_balance':all(r['energy_balance_residual_W']<1e-6 for r in rows),'all30vent_energy_balance':all(r['energy_balance_residual_W']<1e-6 for r in vent),'positive_finite_neck_R':neck>0}
r={'reconstructed_and_rerun':True,'source_wing_geometry':'THERMAL_WING_GEOMETRY.json','C206_MPN':'T598X477M006ATE025','C206_max_case_C':125,'body105_target_is_not_part_maximum':True,'areas_each_face_m2':A.tolist(),'projected_overlap_to_carrier_m2':overlaps.tolist(),'wing_neck_R_K_W':neck,'neck_assumptions':{'credited_inner_planes':2,'min_copper_um_each':24,'min_effective_width_mm':30,'length_to_mean_wing_mm':5,'copper_k_W_mK':300,'FR4_k_W_mK':.25,'min_FR4_thickness_mm':1.44,'native_fill_and_lateral_feed_proof':'PENDING'},'viewfactor_quadrature':views,'viewfactor_matrix':F.tolist(),'cases':rows,'ventilated_cavity_cases':vent,'required_vent_openings_at_2p214W':req,'checks':checks,'conditions':['Surfaceemissivity,10Klocaloffset andcontinuousload aredeclaredboundsnotmeasured.','Verticalmount25mmfacegaps/40mminlet-outletclearspace;65Clocalairandmeanradiantboundary.','ESP85Cratingisambientimmediatelyoutsidemodule,notcasemaximum.','OnlyC206uses125Cscreen;otherdevicesretaintheirownlimits.']}
(D/'THERMAL_WING_MODEL.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({'checks':checks,'case_counts':[len(rows),len(vent),len(req)],'full_load_70kPa_eps06_feed100':next(x for x in rows if x['Pboard_W']==4.79 and x['pressure_Pa']==70000 and x['epsilon']==.6 and x['lateral_feed_R_each_K_W']==100)}))
