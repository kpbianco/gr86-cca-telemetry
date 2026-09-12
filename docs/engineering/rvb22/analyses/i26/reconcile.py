#!/usr/bin/env python3
"""Preserve all 290 original questions; reconcile only the four I26 desktop rows."""
from __future__ import annotations
import collections, copy, hashlib, json
from pathlib import Path
D=Path(__file__).resolve().parent; W=D.parents[1]
SOURCE='5b373f6033fdbd18f126f8ca054b619abada2568778c5a8fc303c4e756fb06c8'
IDS={'WCA-07','REG-02','GND-02','THERM-02'}

def require(ok,message):
    if not ok: raise ValueError(message)

def load(path): return json.loads(path.read_text())
def save(path,value): path.write_text(json.dumps(value,indent=2,ensure_ascii=False)+'\n')
def sha(path): return hashlib.sha256(path.read_bytes()).hexdigest()

def qualification_gates():
    return {'revision':'I26-A','date':'2026-09-12','physical_tests_performed':False,
      'scope':'New deterministic gates for the three reconciled desktop rows plus their selected-accessory dependency. Other existing qualification procedures remain in force; this is not a claim that every external gate has been revalidated.',
      'uncertainty_rule':'Pass upper limits only when measured value plus stated expanded uncertainty is within limit; pass lower limits only when value minus uncertainty is within limit. Record calibration, bandwidth, probe loading and ambient placement.',
      'gates':[
       {'id':'I26-LED-01','criteria':['WCA-07'],'status':'NOT_EXECUTED',
        'predicted':'All seven actual 1kohm chains bounded at3.786302mA including specified allowances. Typical-curve optical inference is not a guaranteed minimum. Fine-mesh LED board neighborhoods plus mesh allowance are approximately79.96..83.63C, not junction or local-air results.',
        'setup':['Fit final APT1608SGC population and final enclosure/viewing opening; include D401 GPS search and all six GPIO LEDs.',
                 'Measure resistor voltage and resistance using high-impedance probes; compute each LED current. Use a calibrated photometer/goniometer or equivalent luminous-intensity setup; do not substitute camera autoexposure.',
                 'Characterize at rail3.0/3.3/3.6V on a current-limited isolated component/rail fixture within device ratings. Do not backfeed the assembled regulator. Include cold design condition and65C bulk-air release hot case; record local LED air and pad/case temperature.',
                 'Install actual enclosure and evaluate observer position0.5m and0/plus-or-minus45deg at1000lux and10000lux ambient, then the maximum measured representative dashboard illumination if higher. Use final optical window/shroud, not bare LED.'],
        'numeric_acceptance':{'per_LED_current_max_mA':3.787,'six_GPIO_total_max_mA':22.718,'all_seven_total_max_mA':26.505,'LED_local_air_max_C':85,'LED_junction_max_C':110,'minimum_intensity_at_required_angles_mcd':.1,'randomized_on_off_trials_per_condition':20,'correct_identifications_min':19,'identification_time_max_s':1},
        'allocation_note':'0.1mcd and19/20 recognition are explicit I26 engineering acceptance allocations, not manufacturer guarantees or newly claimed user-specified thresholds. Both radiometric and actual dashboard recognition tests must pass; passing0.1mcd alone does not prove daylight visibility.',
        'junction_method':'Use a calibrated low-current forward-voltage thermometry method and uncertainty budget, or a validated assembled thermal model correlated to measured pad/case temperature. Do not use the datasheet510K/W figure for pads smaller than its stated16mm2 condition.',
        'failure_implication':'Optical failure first requires same-footprint higher-efficiency/bin LED or shroud/light-pipe/assembly-value evaluation. Do not reduce resistance below the retained thermal/current envelope without reanalysis. Reopen PCB only if an acceptable assembly/enclosure solution cannot fit; current or temperature failure reopens component/thermal analysis.'},
       {'id':'I26-PWR-01','criteria':['REG-02'],'status':'NOT_EXECUTED',
        'predicted':'I25 power-flow-coupled allocation2.940859375W, U151loss0.3873375W and U121heat0.608171875W including0.025W magnetic allowance.',
        'setup':['Use the controlled minimum, nominal and upper normal input-voltage corners from the existing power/harness contract; do not invent a new automotive input envelope.',
                 'Use simultaneous calibrated voltage/current measurements at input and output of each converter, including return offsets and sense insertion loss. Average instantaneous V*I for switching current; do not multiply unrelated asynchronous meter readings.',
                 'Exercise sustained CAN receive500kbit/s, GPS10Hz, oil processing and BLE120notifications/s,160MHz,+3dBm,WiFi disabled, all indicators on for current budgeting. Record current at>=1kS/s and compute sliding1s and60s means after thermal stabilization.'],
        'numeric_acceptance':{'main_release_current_sliding_1s_and_60s_max_A':.45,'each_converter_efficiency_min':.80,'alternative_U151_loss_max_W':.3873375,'alternative_U121_total_heat_max_W':.608171875,'alternative_U121_conversion_loss_max_W':.583171875,'main_profile_voltage_model_bound_V':3.443},
        'logic':'Accept efficiency>=80percent OR a properly partitioned measured-loss result no greater than the corresponding C04 loss allocation. Do not count magnetic loss twice, and do not call a current allocation a hardware current limiter.',
        'failure_implication':'Reopen coupled thermal/power analysis. First test workload/firmware duty and actual conversion loss; assembly/passive or converter/thermal-path changes may be needed. PCB respin is conditional on the corrected power-flow model, not automatic.'},
       {'id':'I26-THERM-01','criteria':['THERM-02','REG-02'],'status':'NOT_EXECUTED',
        'predicted':'Existing-cooling0.25mm board maximum107.26665C; screening allowance5.61633C gives112.88298C. Not mesh-converged, not a junction-temperature prediction. C02 is not adopted.',
        'setup':['Use final enclosure, orientation, mounting/landing and representative harness in65C bulk-air chamber at upper controlled normal input and I26-PWR-01 workload.',
                 'Place calibrated fine-wire thermocouples in immediate air just outside U201 without touching shield/PCB or disturbing airflow; record at multiple exposed faces including the hottest sampled location. Instrument U201 shield, U121, U151, C206 region, PCB hot region and enclosure/landing.',
                 'For IR, establish emissivity/reflected-temperature correction using a calibrated reference spot; do not report shiny shield IR as true case temperature.',
                 'Continue until all monitored temperatures change by<0.1C/min for30consecutive minutes. A run that never stabilizes does not pass; a safety stop is a failed/aborted gate, not stable operation.'],
        'numeric_acceptance':{'bulk_air_test_C':65,'U201_immediate_air_max_C':85,'main_release_current_max_A':.45,'converter_efficiency_min_or_loss_alternative':.80,'PCB_model_correlation_screen_max_C':112.8829809018675,'stability_slope_abs_max_C_per_min':.1,'stable_interval_min':30,'unexpected_resets_or_latchups_max':0,'LED_local_air_max_C':85,'LED_junction_max_C':110},
        'component_rule':'All component-specific ratings and derating limits in the controlled BOM/datasheet register remain mandatory. Board-region and shield temperatures cannot be relabeled as junction temperatures. Exceeding112.883C reopens model correlation even when a particular component has a higher absolute rating.',
        'failure_implication':'Reopen I25 on any gate violation. Diagnose workload/loss, installation airflow/landing, assembly contact and measurement error before changing PCB. C02 may only be adopted after new evidence and its separate mechanical/insulation/RF qualification.'},
       {'id':'I26-GPS-ACCESSORY-01','criteria':['GPS-01','REG-02','THERM-02'],'status':'NOT_EXECUTED',
        'setup':['Instrument the supplied Adafruit851 body and representative cable section at the actual installed harness/accessory location during the same worst-case hot soak. Independently measure delivered bias at the antenna end with a suitable RF-safe bias fixture.',
                 'Test the controlled accessory operating/load corners, then open circuit, current-limited short, and hot plug on an isolated bench fixture before vehicle installation.'],
        'numeric_acceptance':{'Adafruit851_local_environment_max_C':60,'delivered_bias_min_V':2.3,'complete_path_drop_budget_at_25mA_V':.7,'complete_path_equivalent_resistance_budget_ohm':28},
        'interpretation':'28ohm is the complete bias-path design budget, not a measured pigtail resistance. The approximately28mA module limit and9.8mA antenna draw are not new guaranteed hot/cold values. Apply the exact module fault/current criteria from the retained GPS procedure.',
        'failure_implication':'Relocate/protect accessory into<=60C zone or approve a new correctly rated equivalent. Bias failure first requires harness/module/load investigation; PCB change only if board feed fails its separately bounded allocation.'}
      ]}

def reconcile_register(reg,led,ground,thermal,gate_path):
    before=copy.deepcopy(reg)
    require(len(reg['rows'])==290 and len({r['id'] for r in reg['rows']})==290,'Criterion identity loss')
    require({r['id'] for r in reg['rows'] if r['id'] in IDS}==IDS,'Missing controlled criterion')
    require(led['fitted_LED_count']==7 and led['optical_prediction']['guaranteed_low_current_hot_optical_min_mcd']==0,'LED evidence invalid')
    require(ground['count']==54 and not ground['gnd02_desktop_complete'],'Unreviewed GND closure')
    require(thermal['release_basis']['release_heat_W']==2.940859375 and not thermal['decisions']['adopt_c02_cooler'],'I25 release basis changed')
    descriptions={
      'WCA-07':('HARDWARE_OR_INSTALLATION_ACCEPTANCE','qualification_only',
       'All seven fitted APT1608SGC/1kohm paths, including D401 from U404, pass source/polarity/limiter checks. Per-LED current upper3.786302mA includes1percent initial,2percent temperature and2percent service-drift allowances. Six-GPIO total22.717807mA. Forty-eight typical-curve cases predict approximately0.999844..1.904399mA and a conditionally scaled minimum0.164849mcd axial; the guaranteed low-current/hot optical lower bound remains0mcd. 1kohm is retained; no BOM/PCB change. LED board neighborhoods plus I25 allowance are79.96..83.63C, not measured air/junction temperatures.',
       'Execute I26-LED-01 with the actual enclosure/angles/dashboard illumination. Confirm loaded output/current, local-air/junction limits and optical recognition. Prefer same-footprint/bin or optical/assembly corrections before PCB redesign.',
       ['analyses/i26/LED_BOUND.json','analyses/i26/LED_THERMAL_REGIONS.json','analyses/i26/QUALIFICATION_GATES.json']),
      'REG-02':('COMPLETED_CONDITIONAL_MODEL','bounded_inference_closed',
       'Desktop thermal/power engineering complete under I25/C04:2.940859375W coupled release heat, sustained main-profile<=0.45A and converter efficiency>=80percent or measured losses no greater than the modeled allocations. The stale4.815W residual-filled release interpretation is superseded; that envelope remains stress-only. The model does not establish measured efficiency/current or junction temperatures.',
       'Execute I26-PWR-01 and I26-THERM-01. Preserve160MHz,+3dBm,WiFi disabled and120notifications/s; failed current/loss/thermal gates reopen the coupled design. Do not imply measured confirmation.',
       ['I25_THERMAL_MILESTONE.json','analyses/convergence_04/RELEASE_POWER_PROFILE.json','analyses/i26/QUALIFICATION_GATES.json']),
      'THERM-02':('COMPLETED_CONDITIONAL_MODEL','bounded_inference_closed',
       'I25 desktop model complete to the documented conditional design limit: existing-cooling max101.650320C at0.50mm and107.266651C at0.25mm; one-step allowance5.616330C gives112.882981C board-region screen. Mesh convergence, semiconductor junction temperatures, measured hot spots and physical correlation are NOT claimed. C02 is contingency-only and not fitted.',
       'Execute I26-THERM-01:<=85C immediate air outside U201 at65C bulk dashboard air; fixed release current/efficiency gates; record U201/U121/U151/PCB/enclosure temperatures with calibrated thermocouple/IR methods and component limits. Maintain851 accessory<=60C separately. Failure reopens I25.',
       ['I25_THERMAL_MILESTONE.json','analyses/i26/QUALIFICATION_GATES.json']),
      'GND-02':('DESKTOP_WORK_REMAINING','desktop_work_remaining',
       'Individually reconciled all54 historical UUIDs against current native copper:48 prior adjacent-plane interruptions are removed and6 remain on the R301-DNP-isolated CAN_TX_MCU stub (total1.341746mm). All12 RF segments retain adjacent reference in the retained native review. Expanded inspection explicitly includes CAN_RXD before R302, the true GPIO18 GPS receive net, reset/boot and indicator/control applicability;60 signal-via transfers are enumerated. Two CAN_RXD adjacent gaps totaling1.918029mm and other low-speed/control findings require scoped transfer/margin analysis. Nearby vias or another ground plane are not a pass.',
       'Complete source/receiver edge and overshoot/settling bounds for active paths; validate actual etched copper transfer corridors/neck widths and ground-via connections on both planes; resolve the seven finite-search failures without treating them as proven PCB defects. The53 exploratory candidate corridors and thin-filament L*dI/dt sensitivities are NOT closure evidence. Freeze only justified routing/stitching fixes, then one coordinated revision and complete native/RF/thermal/firmware recheck if source changes.',
       ['analyses/i26/HISTORICAL_54_RETURN_FINDINGS.json','analyses/i26/EXPANDED_RETURN_SCOPE.json','analyses/i26/TRANSFER_SCREEN_EXPLORATORY.json','analyses/return_i24/NATIVE_REFERENCE_REVIEW.json'])}
    for row in reg['rows']:
        if row['id'] not in IDS: continue
        status,pre,observation,remaining,evidence=descriptions[row['id']]
        row.setdefault('pre_I26A_disposition',{k:copy.deepcopy(row.get(k)) for k in ['closure','desktop_status','observation','remaining','evidence','evidence_sha256','current_source_PCB_sha256']})
        row.update(desktop_status=status,prehardware_status=pre,observation=observation,remaining=remaining,
                   evidence=evidence,evidence_sha256={p:sha(W/p) for p in evidence},current_source_PCB_sha256=SOURCE,
                   physical_test_claimed=False,fresh_native_execution_claimed=False,original_criterion_unchanged=True,
                   I26_reassessment='Original question, required evidence and literal closure retained. Desktop completion is separate from external acceptance.')
    for old,new in zip(before['rows'],reg['rows']):
        for key in ['id','row','A','B','C','D','E','F','closure']:
            require(old[key]==new[key],'Changed original criterion '+old['id']+' '+key)
        if old['id'] not in IDS: require(old==new,'Unscoped row change')
    counts=dict(collections.Counter(r['desktop_status'] for r in reg['rows']))
    require(counts['DESKTOP_WORK_REMAINING']==1,'Unexpected desktop count')
    reg.setdefault('pre_I26A_summary',copy.deepcopy(reg['summary']))
    groups={
      'desktop_source_documentary':{'CURRENT_EVIDENCE_REVIEWED','COMPLETED_CURRENT_SOURCE_VERIFICATION','COMPLETED_SOURCE_DOCUMENTARY_SCOPE','COMPLETED_I22_ORIGINAL_DESIGN_SCOPE'},
      'conditional_model':{'COMPLETED_CONDITIONAL_MODEL'},
      'hardware_supplier_installation':{'HARDWARE_OR_INSTALLATION_ACCEPTANCE','SUPPLIER_OR_RESPONSIBLE_ACCEPTANCE','CURRENT_NATIVE_COMPLETE_OTHER_EVIDENCE_REQUIRED'},
      'not_applicable':{'NOT_APPLICABLE'},'desktop_work_remaining':{'DESKTOP_WORK_REMAINING'}}
    require(set(counts)==set().union(*groups.values()),'Unclassified desktop state')
    aggregate={name:sum(counts.get(state,0) for state in states) for name,states in groups.items()}
    require(sum(aggregate.values())==290,'Aggregate row loss')
    reg['summary'].update(desktop_status_counts=counts,actual_source_thermal_engineering_complete=True,
      release_status='I26-A: WCA electrical/optical inference complete; REG/THERM reconciled to I25; GND-02 remains actionable. Fabrication not released.',
      actionable_prehardware_ids=['GND-02'],physical_tests=0,
      prehardware_aggregate=aggregate)
    reg['summary']['iterations'].append({'id':'I26-A','status':'CURRENT_DESKTOP_RECONCILIATION','description':'7LED bound;54return findings reconciled; expanded60via scope;I25REG/THERM reconciliation;one actionable GND criterion remains.'}) if not any(x['id']=='I26-A' for x in reg['summary']['iterations']) else None
    reg.update(checkpoint='RVB22_I26A_PREHARDWARE_CLOSURE_PROGRESS',date='2026-09-12',status='ONE_ACTIONABLE_DESKTOP_CRITERION_REMAINS',PCB_sha256=SOURCE)
    reg['meaning_of_zero']='Target remains zero actionable prehardware work; current value is1 (GND-02). Literal open physical/supplier criteria and historical redlines are not reclassified as PCB defects or falsely passed.'
    return reg

def run():
    require(sha(W/'candidate/cad/GR86_CCA_RevB.kicad_pcb')==SOURCE,'CAD effectivity changed')
    save(D/'QUALIFICATION_GATES.json',qualification_gates())
    reg=reconcile_register(load(W/'FINAL_REVIEW_REGISTER.json'),load(D/'LED_BOUND.json'),load(D/'HISTORICAL_54_RETURN_FINDINGS.json'),load(W/'I25_THERMAL_MILESTONE.json'),D/'QUALIFICATION_GATES.json')
    save(W/'FINAL_REVIEW_REGISTER.json',reg)
    master=load(W/'I26_PREHARDWARE_CLOSURE.json')
    master.update(current_actionable_prehardware_ids=['GND-02'],current_actionable_prehardware_count=1,current_counts=reg['summary']['prehardware_aggregate'],original_290_literal_statuses_preserved=True)
    master['design_lanes']['ground_return']={'class':'desktop_work_remaining','historical_findings_reconciled':54,'previously_removed':48,'isolated_TX_stub_segments':6,'expanded_signal_vias':60,'evidence':'analyses/i26/EXPANDED_RETURN_SCOPE.json','next':'Quantitative active-path edge/return-transfer/neck-width margins, then only justified coordinated routing fixes.'}
    master['design_lanes']['leds']={'class':'qualification_only','desktop_electrical_complete':True,'conditional_optical_inference_complete':True,'fitted_count':7,'guaranteed_hot_low_current_min_mcd':0,'gate':'I26-LED-01','resistor_value_ohm':1000,'BOM_changed':False}
    master['next_reconciliation']['add_parallel_prehardware_status']=False
    master['next_reconciliation']['parallel_prehardware_status_added']=True
    save(W/'I26_PREHARDWARE_CLOSURE.json',master)
    gates=load(W/'FINAL_GATES.json');gates['actionable_prehardware_actual']=1;gates['actionable_prehardware_ids']=['GND-02']
    gates['gates']=[g for g in gates['gates'] if g['id'] not in ['G22-12','G22-13']]
    gates['gates'] += [{'id':'G22-12','status':'DESKTOP_WORK_REMAINING','criterion':'GND-02','description':'54historical findings individually reconciled; expanded active/control return-transfer proof remains incomplete. No board release on alternate-plane availability alone.','evidence':'analyses/i26/EXPANDED_RETURN_SCOPE.json'},
      {'id':'G22-13','status':'QUALIFICATION_ONLY_LED_VISIBILITY','criterion':'WCA-07','description':'7actual paths electrically bounded; optical typical-curve inference retained without guaranteed hot minimum.','qualification':'analyses/i26/QUALIFICATION_GATES.json#I26-LED-01'}]
    for g in gates['gates']:
        if g['id']=='G22-09':g['description']='The PCB RF matching/reference/keepout model is bounded in its stated RF scope. This does not close the wider non-RF return-transfer criterion GND-02. Installed nonlinear clamp/radiated/coexistence behavior remains qualification.'
    save(W/'FINAL_GATES.json',gates)
    audit={'status':'I26A_VERIFIED_PROGRESS_NOT_ZERO','source_PCB_sha256':SOURCE,'controlled_rows':290,'literal_closure_counts':dict(collections.Counter(r['closure'] for r in reg['rows'])),'desktop_status_counts':reg['summary']['desktop_status_counts'],'prehardware_counts':reg['summary']['prehardware_aggregate'],'actionable_prehardware_ids':['GND-02'],'rows_reconciled':sorted(IDS),'all_original_questions_and_required_evidence_preserved':True,'physical_tests_performed':0,'PCB_BOM_firmware_changes':False,'C02_adopted':False,'fabrication_release':False}
    save(D/'RECONCILIATION_AUDIT.json',audit)
    print(json.dumps(audit,indent=2))

if __name__=='__main__': run()
