#!/usr/bin/env python3
"""Exact-population electrical proof plus explicitly non-guaranteed optical inference."""
from __future__ import annotations
import argparse, hashlib, itertools, json, math, re, sys
from pathlib import Path
D=Path(__file__).resolve().parent; W=D.parents[1]
sys.path.insert(0,str(W/'analyses/convergence_01/support'))
import sexpdata as sx
import check_combined_copper as c
EXPECTED='5b373f6033fdbd18f126f8ca054b619abada2568778c5a8fc303c4e756fb06c8'

def require(ok, message):
    if not ok: raise ValueError(message)

def pins(fp):
    return {str(p[1]):c.get(p,'net')[1]for p in c.child(fp,'pad')if c.get(p,'net')}

def current_upper_mA(resistance_ohm:float, aging:bool=False)->float:
    require(resistance_ohm>0,'Resistance must be positive')
    return 3600/(resistance_ohm*.99*.98*(.98 if aging else 1))

def interpolate(x:float,xs:list[float],ys:list[float])->float:
    require(xs[0]<=x<=xs[-1],'No curve extrapolation')
    for i in range(1,len(xs)):
        if x<=xs[i]:
            f=(x-xs[i-1])/(xs[i]-xs[i-1]);return ys[i-1]+f*(ys[i]-ys[i-1])
    raise AssertionError('Unreachable')

def predicted_current(v:float,resistance:float,r_driver:float,temp_C:float)->float:
    """mA; visually approximate typical I-V curve, NOT a guaranteed diode model."""
    require(-10<=temp_C<=85,'Do not extrapolate typical -2mV/K coefficient')
    currents=[0.,1.,2.,5.,10.,20.,25.]
    voltages=[1.65,1.80,1.88,1.96,2.06,2.20,2.28]
    lo,hi=0.,25.
    for _ in range(60):
        mid=(lo+hi)/2
        vf=interpolate(mid,currents,voltages)-.002*(temp_C-25)
        if vf+mid/1000*(resistance+r_driver)>v:hi=mid
        else:lo=mid
    return (lo+hi)/2

def analyze(pcb:Path):
    raw=pcb.read_bytes();require(hashlib.sha256(raw).hexdigest()==EXPECTED,'PCB effectivity changed')
    tree=sx.loads(raw.decode());fps={c.prop(f)['Reference']:f for f in c.child(tree,'footprint')}
    channels=[('D601','R601','LED_PWR','15','8'),('D602','R602','LED_BLE','6','6'),('D603','R603','LED_CAN','7','7'),('D604','R604','LED_GPS','8','12'),('D605','R605','LED_SYS','10','18'),('D606','R606','LED_OIL','9','17'),('D401','R405','GPS_SEARCH_BUFFER',None,'3')]
    actual_leds={ref for ref,f in fps.items()if c.prop(f).get('MPN')=='APT1608SGC'and c.prop(f).get('Assembly')=='FACTORY'}
    require(actual_leds=={x[0]for x in channels},'LED population changed')
    header=(W/'candidate/firmware/cca_telemetry/src/led_status.h').read_text()
    for _,_,signal,gpio,_ in channels:
        if gpio:
            require(re.search(r'#define\s+'+signal+r'_ACTIVE_LOW\s+0\b',header),'LED firmware polarity changed')
            require(re.search(r'#define\s+'+signal+r'_GPIO\s+'+gpio+r'\b',header),'LED firmware GPIO mapping changed')
    rows=[]
    for diode,resistor,signal,gpio,pad in channels:
        f,r=fps[diode],fps[resistor];anode=pins(f)['2']
        require(pins(f)['1']=='GND','LED cathode not grounded: '+diode)
        require(pins(r)=={'1':signal,'2':anode},'LED limiting route changed: '+diode)
        require(c.prop(r)['MPN']=='CRCW06031K00FKEA'and c.prop(r)['Assembly']=='FACTORY','Wrong fitted limiter '+resistor)
        source='U201'if gpio else'U404'
        require(pins(fps[source])[pad]==signal,'Driver pin mismatch '+diode)
        if not gpio:require(c.prop(fps[source])['MPN']=='SN74LVC2G125DCUR','GPS search driver changed')
        i=current_upper_mA(1000);aged=current_upper_mA(1000,True)
        rows.append({'LED':diode,'resistor':resistor,'driver_reference':source,'driver_pad':pad,'GPIO':gpio,'driver_net':signal,'anode_net':anode,'cathode_net':'GND','LED_MPN':'APT1608SGC','limiter_MPN':'CRCW06031K00FKEA','active_high':True,'hard_current_upper_mA':i,'hard_current_upper_including_2pct_drift_mA':aged,'limiter_power_upper_mW':3.6*aged,'LED_power_upper_mW':3.6**2/(4*(1000*.99*.98*.98))*1000,'LED_power_equation':'Vf*(Vrail-Vf)/R <= Vrail^2/(4R), for 0<=Vf<=Vrail. GPIO voltage drop only reduces LED power.'})
    cases=[]
    for voltage,resistance,driver,temp in itertools.product([3.,3.3,3.6],[970.2,1030.2],[0.,100.],[-10.,25.,65.,85.]):
        current=predicted_current(voltage,resistance,driver,temp)
        factor=interpolate(temp,[-10.,25.,65.,85.],[1.35,1.,.67,.57])
        cases.append({'rail_V':voltage,'limiter_ohm':resistance,'assumed_driver_ohm':driver,'temperature_C':temp,'predicted_current_mA':current,'conditional_scaled_5mcd_bin_mcd':5*current/20*factor,'conditional_scaled_12mcd_typ_mcd':12*current/20*factor})
    candidates=[]
    for resistance in [1000.,820.,680.,470.]:
        i=current_upper_mA(resistance,True)
        candidates.append({'resistance_ohm':resistance,'worst_current_mA_with_drift':i,'nominal_current_mA':predicted_current(3.3,resistance,25,25),'margin_to_allocated_85C_5mA_screen_mA':5-i,'passes_5mA_screen':i<=5,'is_adopted':resistance==1000})
    return {'status':'ELECTRICAL_DESKTOP_COMPLETE_OPTICAL_CONFIRMATION_REQUIRED','source_PCB_sha256':EXPECTED,'date':'2026-09-12','fitted_LED_count':7,'GPIO_driven_LED_count':6,'buffer_driven_LED_count':1,'rows':rows,'all_seven_current_upper_with_drift_mA':sum(x['hard_current_upper_including_2pct_drift_mA']for x in rows),'six_GPIO_total_current_upper_with_drift_mA':sum(x['hard_current_upper_including_2pct_drift_mA']for x in rows if x['GPIO']),'hard_limit_assumptions':{'rail_ceiling_V':3.6,'resistor_initial_tolerance':.01,'additional_temperature_allowance':.02,'optional_service_drift_allowance':.02,'minimum_LED_Vf_for_current_stress_V':0.,'source_output_cannot_exceed_rail':True},'manufacturer_limits':{'LED_operating_ambient_max_C':85,'LED_junction_max_C':110,'LED_current_25C_max_mA':25,'LED_intensity_20mA_25C_min_mcd':5,'LED_intensity_20mA_25C_typ_mcd':12,'LED_full_half_intensity_angle_deg':150,'limiter_standard_power_70C_W':.1,'limiter_film_limit_C':155,'limiter_power_at_112p883C_mW':100*(155-112.8829809018675)/(155-70)},'electrical_interpretation':['The source current is hard bounded even for a shorted LED. This is not a guarantee of loaded ESP32 VOH at hot/cold.','ESP32 40mA source drive is TYPICAL at stated conditions; its VOH minimum is specified with high-impedance load. Neither is used as a guaranteed hot loaded-output specification.','The selected 1kohm resistor retains more hot-current margin than smaller values. No LED/resistor/BOM/CAD change is adopted.','The 5mA at85C comparison is a conservative engineering screen read from a typical thermal-derating graph, not an independently guaranteed assembled-board thermal limit. Actual local ambient and junction limits remain gates.','Published LED thermal resistance assumes >=16mm2 pad per terminal. That thermal resistance is not applied to this smaller footprint as a guaranteed junction prediction.'],'optical_prediction':{'case_count':len(cases),'typical_curve_only':True,'voltage_driver_temperature_cases':cases,'conditional_current_min_mA':min(x['predicted_current_mA']for x in cases),'conditional_current_max_mA':max(x['predicted_current_mA']for x in cases),'conditional_scaled_bin_min_mcd':min(x['conditional_scaled_5mcd_bin_mcd']for x in cases),'conditional_typical_intensity_max_mcd':max(x['conditional_scaled_12mcd_typ_mcd']for x in cases),'conditional_45deg_view_min_mcd':.7*min(x['conditional_scaled_5mcd_bin_mcd']for x in cases),'guaranteed_low_current_hot_optical_min_mcd':0.,'curve_method':'Approximate manufacturer typical plots digitized visually; piecewise-linear I-V and flux-temperature scaling. Driver0..100ohm, linear flux-current scaling, and angular factor0.7 are explicit inference assumptions. No source guarantees this constructed envelope.','no_cold_extrapolation_below_C':-10},'resistor_comparison':candidates,'qualification_gate':'I26-LED-01 in QUALIFICATION_GATES.json','failure_disposition':'First tune same-footprint efficiency/bin, resistor within hot/current allocation, or enclosure/shroud/light-pipe. Reopen PCB only if optical/mechanical solution cannot meet the actual installed visibility requirement.','sources':[{'url':'https://www.kingbrightusa.com/images/catalog/spec/apt1608sgc.pdf','revision':'V.22B, Dec7 2023','pages':[1,2,3],'used_for':'Exact part, pin polarity,20mA intensity, electrical/thermal limits, typical I-V/current/temperature/angle plots'},{'url':'https://www.vishay.com/docs/20035/dcrcwe3.pdf','revision':'Apr14 2026','pages':[2,3,6],'used_for':'CRCW0603 rating, tolerance/TCR and power derating'},{'url':'https://documentation.espressif.com/esp32-s3_datasheet_en.pdf','revision':'v2.2','pages':[65],'used_for':'High-impedance VOH footnote; typical drive current must not be treated as a guaranteed loaded specification'},{'url':'https://www.ti.com/lit/ds/symlink/sn74lvc2g125.pdf','revision':'SCES204Q Mar2017','pages':[6,7],'used_for':'D401 buffer drive range and output limits'}],'original_literal_WCA07_closed':False,'physical_tests_claimed':False}

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--pcb',type=Path,default=W/'candidate/cad/GR86_CCA_RevB.kicad_pcb');ap.add_argument('--out',type=Path,default=D/'LED_BOUND.json');a=ap.parse_args();result=analyze(a.pcb);a.out.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:result[k]for k in ['status','fitted_LED_count','six_GPIO_total_current_upper_with_drift_mA','all_seven_current_upper_with_drift_mA']}))
