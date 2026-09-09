#!/usr/bin/env python3
"""Recovered bounded mechanics model; declared material minima are requirements, not certificates."""
from mechanics_runtime import *
import json, math
from pathlib import Path
D=Path(__file__).resolve().parent
G=json.loads((D/'THERMAL_WING_GEOMETRY.json').read_text())
# All lengths in SI for calculations. The outboard strip is a conservative beam sensitivity, not a plate FEM.
E=10e9;t=.00144;b=.041254763;L=(94.254766-63)*.001;I=b*t**3/12
rhoFR=2200;rhoCu=8960;area=L*b
mboard=area*(.00176*rhoFR+.000240*rhoCu) # full Cu plus full laminate deliberately double counts copper volume

def pointdef(x,a,F):
 return F*x*x*(3*a-x)/(6*E*I) if x<=a else F*a*a*(3*x-a)/(6*E*I)
def uniformdef(x,w): return w*x*x*(6*L*L-4*L*x+x*x)/(24*E*I)
def state(mod_g,other_g,acc_g):
 a=(81.504766-63)*.001;ac=(82.6-63)*.001;g=acc_g*9.80665
 loads=[(a,mod_g*.001*g),(ac,.001*g),(L,other_g*.001*g)];w=mboard*g/L
 def d(x):return uniformdef(x,w)+sum(pointdef(x,a,F) for a,F in loads)
 M=w*L*L/2+sum(a*F for a,F in loads)
 # Difference from straight chord quantifies imposed board curvature across a nominally rigid package.
 def curvature(x0,x1):
  return max(abs(d(x0+(x1-x0)*i/100)-(d(x0)+(d(x1)-d(x0))*i/100)) for i in range(101))
 return {'module_mass_g':mod_g,'other_tip_mass_g':other_g,'acceleration_g':acc_g,'tip_deflection_mm':d(L)*1e3,'C206_far_edge_deflection_mm':d((86.25-63)*.001)*1e3,'root_surface_strain_microstrain':M*t/(2*E*I)*1e6,'module_chord_curvature_mm':curvature((68.754766-63)*.001,(94.254766-63)*.001)*1e3,'C206_chord_curvature_mm':curvature((78.95-63)*.001,(86.25-63)*.001)*1e3}
cases=[state(m,o,a) for m in [1,3,10] for o in [0,2,5] for a in [5,10,20]]
ctecases=[]
for dppm in [5,10,15]:
 for DT in [40,65,105]:
  for package,dnp in [('U201',math.hypot(25.5,18)/2),('C206',math.hypot(7.3,4.3)/2)]:
   for h in [.1,.25]:
    disp=dppm*1e-6*DT*dnp;ctecases.append({'package':package,'CTE_mismatch_ppm_K':dppm,'deltaT_K':DT,'solder_height_mm':h,'DNP_mm':dnp,'free_mismatch_mm':disp,'kinematic_shear_strain':disp/h})
F=.100*20*9.80665
lugstress=3*6*F*.004/(.007*.00144**2)/1e6
bearing=F/(.004*.00144)/1e6
carrier_F=F;lc=.0095;bc=.012;tc=.003;Ic=bc*tc**3/12
r={'status':'EXECUTED_CONDITIONAL_BEAM_AND_TOLERANCE_SCREEN','source_pcb_sha256':G['effectivity']['output_sha256'],'material_requirements':{'E_min_GPa':10,'PCB_t_min_mm':1.44,'flexural_allowable_MPa':150,'bearing_allowable_MPa':50,'scope':'Engineering procurement/acceptance requirements, not guaranteed NP510A or JLC laminate minima. Notched fastener-hole and laminate fatigue strength are not inferred from bulk flexural strength.'},'mass_scope':{'assembly_max_g':100,'C206_engineering_allocation_g':1,'C206_supplier_typical_g':.554,'outboard_PCB_conservative_g':mboard*1e3},'mount_screen':{'one_lug_carries_full20g_load_N':F,'Kt_allocation':3,'cantilever_lever_mm':4,'ear_flexural_MPa':lugstress,'sleeve_bearing_MPa':bearing,'min_hole_edge_ligament_mm':1.025,'z_float_mm':[1.85-1.76,1.95-1.44],'global_relative_motion_allocation_mm':1,'hardware_to_original_courtyard_margin_mm':1.75-1,'hardware_to_legacy_ESP_exclusion_margin_mm':68.029766-66.5-1,'carrier_gross_deflection_mm':carrier_F*lc**3/(3*E*Ic)*1e3,'carrier_gross_bending_MPa':6*carrier_F*lc/(bc*tc**2)/1e6,'M3_window_min_ligament_mm':.65,'local_notch_fastener_stress_scope':'Not covered by gross carrier beam. All four chassis mounts required; root/local material analysis remains a conditional geometry acceptance.'},'outboard_beam_cases':cases,'CTE_joint_kinematic_cases':ctecases,'C206_clearance':{'under_module_design_depth_mm':7,'body_max_mm':4.3,'solder_standoff_allocation_mm':.25,'mount_float_max_mm':.51,'warp_allocation_mm':.75,'local_deflection_allocation_mm':.2,'remaining_mm':7-4.3-.25-.51-.75-.2},'fit':{'bare_board_mm':G['board_size_mm'],'basic_installed_bbox_mm':[108.454766,67,30.27],'scope':'Board/carrier/hardware only; actual vehicle cavity fit, connector mating/tool access, cable bend radius, antenna clearance and thermal air gaps are additional constraints.'},'checks':{'ear_flexural_below_requirement':lugstress<150,'bearing_below_requirement':bearing<50,'positive_Z_float':1.85>1.76,'positive_RF_hardware_margin':68.029766-66.5-1>0,'C206_deflection_within_allocation':max(x['C206_far_edge_deflection_mm'] for x in cases)<.2,'C206_depth_positive':7-4.3-.25-.51-.75-.2>0},'limits':'Static beam and kinematic CTE sensitivity are reproducible reasoning, not modal response, joint fatigue life, physical drop/vibration or actual material acceptance. The 100g mass and20g acceleration are declared installation loads.'}
r['thermal_hardware_125g_screen']={'total_mass_requirement_g':125,'base100g_ear_stress_MPa':lugstress,'with_25g_hardware_allowance_MPa':lugstress*1.25,'flexural_requirement_MPa':150,'pass_under_declared_material_load':lugstress*1.25<150}
r['MECH22_31_total_126g_screen']={'total_mass_requirement_g':126,'prior_125g_screen_retained_as_history':True,'added_carrier_mass_upper_g':.086,'one_ear_flexural_MPa':lugstress*1.26,'flexural_requirement_MPa':150,'pass_under_declared_material_load':lugstress*1.26<150,'scope':'Added carrier mass is supported by carrier mounts; placing the entire126g allocation onto one PCB ear is conservative for this static screen. No modal/fatigue guarantee.'}
(D/'STRUCTURAL_SERVICE_MODEL.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps({'checks':r['checks'],'representative':state(3,2,20),'max_cap_deflection_mm':max(x['C206_far_edge_deflection_mm'] for x in cases)}))
