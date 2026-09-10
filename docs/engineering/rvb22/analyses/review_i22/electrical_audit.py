"""Bounded via heating, accessory DC applicability, EP and RF reconciliation.

No undocumented silicon behavior or physical temperature is manufactured here.
All bounds are saved separately from unresolved installed qualification.
"""
from pathlib import Path
import sys,json,math,hashlib
D=Path(__file__).resolve().parent;W=D.parents[1]
sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor'),str(W/'control')]
import check_combined_copper as c
import numpy as np
from shapely.geometry import Point
from shapely.ops import unary_union
N=W/'runtime/hosted/run22/extracted/native_I06_hosted'
P=N/'candidate_kicad/GR86_CCA_RevB.kicad_pcb'
b,it,u=c.collect(P);assert not u
fps={c.prop(f)['Reference']:f for f in c.child(b,'footprint')}
names={x[1]:x[2]for x in c.child(b,'net')}

# 1D uniformly heated copper tube, conduction to its two board end planes:
# R=rho*L/A; Tmax-Tend=I^2*rho*L^2/(8*k*A^2). This isolates via self-rise,
# not board or package temperature. No sharing among parallel vias is credited.
# Use .08mm negative finished-hole allowance, maximum1.76mm board, minimum
#15um wall, rho4e-8ohm*m and k300W/m/K. These are deliberately poor hot-copper
#properties. Unknown average-versus-minimum plating remains a process gate.
L=.00176;t=15e-6;rho=4e-8;k=300.;I=2.
vias=[]
for v in c.child(b,'via'):
    net=c.get(v,'net')[0];nom=c.get(v,'drill')[0];dmin=(nom-.08)*1e-3
    assert dmin>0
    A=math.pi*((dmin/2+t)**2-(dmin/2)**2)
    R=rho*L/A;rise=I*I*rho*L*L/(8*k*A*A)
    vias.append(dict(net=names[net],xy_mm=c.get(v,'at')[:2],nominal_drill_mm=nom,minimum_drill_mm=nom-.08,
      barrel_resistance_ohm=R,loss_at2A_W=I*I*R,self_rise_at2A_K=rise))
vreport=dict(status='PASS_DECLARED_VIA_SELF_HEATING_BUDGET',via_count=len(vias),each_via_current_screen_A=I,
  maximum_board_thickness_mm=L*1e3,minimum_wall_um=t*1e6,negative_finished_hole_allowance_mm=.08,
  copper_resistivity_ohm_m=rho,copper_conductivity_W_m_K=k,allocated_self_rise_limit_K=10,
  maximum_self_rise_K=max(x['self_rise_at2A_K']for x in vias),maximum_single_barrel_R_ohm=max(x['barrel_resistance_ohm']for x in vias),
  equation='I^2*rho*L^2/(8*k*A^2), A=pi*((d/2+t)^2-(d/2)^2)',rows=vias,
  applicability=['2A in every individual via is a conservative normal-load screen against the1A LM5164 and1.5A LM63615 stage ratings; no parallel current sharing credit.',
    'Two board-end temperatures are prescribed. Add calculated rise to the hotter end; board/package absolute temperatures remain THERM02/REG02.',
    'Plated-wall minimum15um and intact annular rings are fabrication acceptance conditions. JLCPCB average18um is not a guaranteed minimum.',
    'This is not fuse-clearing, abnormal fault survival, solder-joint or transient ground-bounce proof.'],physical_measurement_claimed=False)
assert vreport['maximum_self_rise_K']<10
(D/'VIA_CURRENT_THERMAL_BUDGET.json').write_text(json.dumps(vreport,indent=2)+'\n')

# Independent exact exposed-pad connectivity/nearby returns, including TPS2660
#RTN isolation. Bare thermal vias outside mask/paste do not need fill on that
#basis; any mask/paste overlap is explicitly counted, not inferred from centers.
eps=[]
for ref,pin,expected in [('U121','9','GND'),('U151','17','GND'),('U201','41','GND'),('U301','9','GND'),('U501','17','OIL_EFUSE_RTN')]:
    f=fps[ref];pads=c.child(f,'pad');p=next(p for p in pads if str(p[1])==pin);g=c.pad_shape(f,p);net=c.get(p,'net')[1];assert net==expected
    layer=c.get(f,'layer')[0];paste=unary_union([c.pad_shape(f,p)for p in pads if layer.replace('Cu','Paste')in(c.get(p,'layers')or[])])
    near=[]
    for v in c.child(b,'via'):
        if names[c.get(v,'net')[0]]!=net:continue
        pt=Point(c.get(v,'at')[:2]);dist=g.distance(pt)
        if dist<=1.01:
            dr=c.get(v,'drill')[0]
            near.append(dict(xy_mm=c.get(v,'at')[:2],distance_from_EP_mm=dist,drill_to_paste_overlap_mm2=pt.buffer(dr/2).intersection(paste).area))
    assert len(near)>=2
    eps.append(dict(ref=ref,EP=pin,net=net,nearby_same_net_vias=near))
ereport=dict(status='PASS_EXACT_EP_NET_AND_LAND_REVIEW',rows=eps,
  mask_and_paste_quantities='PACKAGE_ORIENTATION_RESULTS.json',
  interpretation=['U121,U151 GND thermal copper uses manufacturer-size mask windows. Oversized buried copper is not exposed solder area.',
  'U201 retains the nine manufacturer .9mm paste/mask lands;48 filled/capped thermal vias are controlled separately.',
  'U301 EP connects to GND and two through vias .4mm from the pad edge; the datasheet recommends multiple returns, not a mandatory via-in-pad topology.',
  'U501 EP is RTN, not indiscriminately shorted to system GND. This preserves TPS2660 reverse-current/protection topology.',
  'Component limits, thermal performance, fill/capping and stencil/reflow approval remain separate gates.'],physical_measurement_claimed=False)
(D/'EXPOSED_PAD_REVIEW.json').write_text(json.dumps(ereport,indent=2)+'\n')

# Exact PID960 typical current table gives a compatibility indication, not a
#guaranteed cold/hot supply bound. Derive resistance budgets for later bench
#acceptance instead of assigning fabricated cable or detector tolerances.
currents=[(2.5,6.6),(3,8.6),(4,12.6),(5,16.6)]
typical_at33=8.6+(3.3-3)*(12.6-8.6)
accessory=dict(status='IDENTITIES_BOUND_DC_COMPATIBILITY_PARTIAL',adapter_PID=851,antenna_PID=960,
  manufacturer_recommended_pair='PID960 product page explicitly links PID851 as the required U.FL/SMA adapter.',
  cable_mm=dict(nominal_length=150,tolerance=3,outer_diameter=1.8,allocated_maximum_outer_diameter=2.0),
  cable_envelope_basis='2.0mm OD is an engineering receiving allocation, not a published manufacturer tolerance; 0.5mm lateral route uncertainty is added independently.',
  SMA_mating='Product-level standard SMA jack to SMA male is documented. Linked851 drawing still labels RP-SMA; incoming center-contact inspection remains mandatory.',
  antenna_operating_V=[2.3,5.5],antenna_temperature_C=[-30,85],adapter_temperature_C=[-10,60],
  GPS_supply_operating_contract_V=[3.0,3.6],PA1616D_external_detection_mA=4,
  PA1616D_documented_current_limit_mA_at_V={'3.0':25,'3.3':28,'3.6':31},
  PID960_typical_current_mA=[dict(V=v,mA=i)for v,i in currents],interpolated_typical_current_at3p3V_mA=typical_at33,
  typical_detection_margin_mA=typical_at33-4,typical_current_limit_headroom_at3p3V_mA=28-typical_at33,
  available_external_path_drop_at3p0V_V=3.0-2.3,
  maximum_combined_internal_feed_cable_contact_R_at25mA_ohm=(3.0-2.3)/.025,
  RF_band='PID960 is GPS L1. No GLONASS antenna gain/noise coverage is asserted.',
  unresolved=['Antenna min/max hot/cold current and PA1616D bias source drop are not guaranteed by the typical current table.',
    'The .7V/28ohm calculation is the complete allowable bias-feed drop, not a measured cable resistance.',
    'Open/short/hot-plug device behavior is described by the module datasheet, but exact response timing and actual-unit behavior are unmeasured.',
    '851 full operating range does not cover the declared65C air case or cold cases below-10C.'],
  physical_measurement_claimed=False)
assert c.get(next(p for p in c.child(fps['U401'],'pad')if str(p[1])=='11'),'net')[1]=='GPS_ANT_RF_BIASED'
assert c.get(next(p for p in c.child(fps['J401'],'pad')if str(p[1])=='1'),'net')[1]=='GPS_EXT_ANT'
(D/'ACCESSORY_APPLICABILITY.json').write_text(json.dumps(accessory,indent=2)+'\n')

# A discrete maximum-principle check for a passive network: at an interior
#minimum below every boundary, all conductive/convective outward heat flows
#are negative, incompatible with a nonnegative heat source. Thus no reduction
#in passive contact resistance alone can reach60C with boundaries>=65C.
mp=dict(status='PASSIVE_COOLING_CANNOT_CLOSE_851_TEMPERATURE_CONFLICT',minimum_boundary_C=65,
  declared_air_C=65,landing_C=70,adapter_operating_ceiling_C=60,minimum_shortfall_K=5,
  argument='For conductancesG>=0 and nonnegative heat, sum(G*(T-Tneighbor))+film(T-Tair)=Q>=0. A temperature below all boundaries at a global minimum makes the left side negative. Therefore Tmin>=65C for this passive steady-state case.',
  applicability='No external source below65C, active refrigeration, evaporative cooling or transient thermal storage is assumed. The result does not assert actual dashboard air temperature.',
  design_action='Keep851 selected but temperature qualification blocked. A verified lower installation/cable environment or an assembly rating covering the declared environment is required; contact optimization alone is insufficient.')
maps=W/'analyses/thermal_i18/face_15.0/map_provisional_0.125_15.npz'
z=np.load(maps);x=z['x'];y=z['y'];T=z['T'];m=(x<=67.475)&(x+.125>=64.075)&(y<=10.6)&(y+.125>=5.3)
mp['existing_fine_model_J401_bottom_region_C']=[float(T[3,m].min()),float(T[3,m].max())]
mp['map_sha256']=hashlib.sha256(maps.read_bytes()).hexdigest()
mp['temperature_is_board_region_not_cable_or_connector']=True
mp['existing_thermal_result_not_superseded']='I21 fine mesh still143.800C maximum board region. No new cooling design or package-temperature pass is claimed.'
(D/'THERMAL_ACCESSORY_BOUND.json').write_text(json.dumps(mp,indent=2)+'\n')

rf=json.loads((W/'analyses/manufacturing_i20/RESULTS.json').read_text())['reference']
assert len(rf['RF'])==12 and max(x['adjacent_gap_after_own_antipads_mm']for x in rf['RF'])==0
network_path=W/'analyses/rf_i18/EXTENDED_NETWORK_RESULTS.json'
network=json.loads(network_path.read_text())
assert network['status']=='PASS_FINITE_ALLOCATED_RF_MODEL'
assert sum(r['cases']for r in network['rows'])==network['case_count']==3981312
assert sum(r['misses']for r in network['rows'])==0
min_rl=min(r['min_both_port_RL_dB']for r in network['rows'])
max_il=max(r['max_IL_dB']for r in network['rows'])
assert min_rl>=10 and max_il<=1
summary=dict(status='RF_REFERENCE_RECONFIRMED_DIGITAL_GAPS_REMAIN',RF_segments=12,RF_adjacent_reference_gaps=0,
 digital_segment_findings=len(rf['other_findings']),digital_both_plane_gap_count=sum(x['both_ground_planes_gap_mm']>1e-6 for x in rf['other_findings']),
 digital_affected_length_mm=sum(x['adjacent_gap_after_own_antipads_mm']for x in rf['other_findings']),
 SI12_disposition='Analysis-only board impedance acceptance under the existing user-approved model workflow, limited to documented stack/etch/device allocations. Native geometry agrees with the finite RF network. This does not close GPS09/12/13 or supplier stackup acceptance.',
 current_network_evidence='analyses/rf_i18/EXTENDED_NETWORK_RESULTS.json',
 current_network_cases=network['case_count'],min_both_port_return_loss_dB=min_rl,max_board_insertion_loss_dB=max_il,
 current_network_sha256=hashlib.sha256(network_path.read_bytes()).hexdigest(),
 RL_acceptance_dB=10,IL_acceptance_dB=1,
 GND02_disposition='OPEN. Alternate-plane availability is useful evidence but does not erase54 adjacent-plane interruptions or establish return-current transfer across them.',
 physical_measurement_claimed=False)
(D/'RF_RETURN_REVIEW.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(dict(max_via_self_rise_K=vreport['maximum_self_rise_K'],EPs=len(eps),GPS_typical_mA=typical_at33,
 J401_fine_board_region_C=mp['existing_fine_model_J401_bottom_region_C'],digital_findings=summary['digital_segment_findings']),indent=2))
