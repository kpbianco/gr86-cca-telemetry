#!/usr/bin/env python3
"""Source-bound product convergence checks. Engineering completion != qualification.

The baseline register is preserved. Results distinguish repeatable checks,
reused bounded analysis, redesign work and physical-only acceptance.
"""
from __future__ import annotations
import argparse, copy, csv, hashlib, json, re, sys
from collections import Counter
from pathlib import Path
D=Path(__file__).resolve().parent
sys.path.insert(0,str(D/'support'))
import sexpdata as sx
import check_combined_copper as c
W=D.parents[1]
PCB_HASH='5b373f6033fdbd18f126f8ca054b619abada2568778c5a8fc303c4e756fb06c8'
FILLED_HASH='11533ea91c3bc4c61dd7066e0001914a72bc90b27afb38d321c43b8feb90e3b1'

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def save(path,data):Path(path).write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n')
def fp_map(tree):return {c.prop(f)['Reference']:f for f in c.child(tree,'footprint')}
def pins(f):return {str(p[1]):str(c.get(p,'net')[1])for p in c.child(f,'pad')if c.get(p,'net')}
def require(ok,msg):
    if not ok:raise ValueError(msg)
def fitted(f):return c.prop(f).get('Assembly')in('FACTORY','MANUAL_GPS')

def receive_only(tree,sketch):
    fs=fp_map(tree);p=c.prop
    require(p(fs['R301'])['Assembly']=='DNP','R301 must be unpopulated in receive-only product')
    require(pins(fs['R301'])=={'1':'CAN_TX_MCU','2':'CAN_TXD'},'Unexpected TX isolation topology')
    require(fitted(fs['R303'])and p(fs['R303'])['MPN']=='CRCW060310K0FKEA','Missing exact fitted 10k recessive pull-up')
    require(pins(fs['R303'])=={'1':'+3V3','2':'CAN_TXD'},'Pull-up wiring changed')
    require(pins(fs['U301'])['1']=='CAN_TXD','TXD pin route changed')
    require(p(fs['R306'])['Assembly']=='DNP','Permanent bus termination must remain unpopulated')
    # No other fitted two-terminal link may bypass R301; examine physical population, not only firmware strings.
    bypass=[]
    for ref,f in fs.items():
        ns=set(pins(f).values())
        if fitted(f) and ns=={'CAN_TX_MCU','CAN_TXD'}:bypass.append(ref)
    require(not bypass,'Fitted TX bridge: '+str(bypass))
    code=re.sub(r'/\*.*?\*/|//[^\n]*','',sketch,flags=re.S)
    require(len(re.findall(r'\bTWAI_MODE_LISTEN_ONLY\b',code))==1,'Listen-only mode missing or ambiguous')
    require(not re.search(r'\bTWAI_MODE_(NORMAL|NO_ACK)\b|\btwai_transmit\s*\(',code),'Transmit-enabled code path present')
    require(re.search(r'general\.tx_queue_len\s*=\s*0\s*;',code),'Transmit queue not disabled')
    for gpio,num in [('CAN_TX_GPIO',5),('CAN_RX_GPIO',4)]:
        require(re.search(r'\b'+gpio+r'\s*=\s*'+str(num)+r'\s*;',code),'CAN pin mapping changed')
    return {'status':'PASS_SOURCE_RECEIVE_ONLY_VARIANT','isolating_link':'R301 DNP','recessive_pullup':'R303 10k +3V3 to TXD','termination':'R306 DNP','transmit_calls':0,'tx_queue_length':0,'six_CAN_TX_reference_gaps':'Real geometry findings, but on an isolated MCU stub, not the populated CAN receiver path. Do not reroute active power solely to erase these flags.','valid_only_for':'Exact receive-only BOM and firmware configuration. A future transmit-enabled variant requires renewed route/reference review.','not_proven':'Powered-off or internally failed transceiver behavior, shorts across the DNP link, real bus loading and vehicle non-interference.'}

def led_review(tree,header):
    fs=fp_map(tree);rows=[]
    # The selected 1k/1% part is screened with an explicit conservative 2% additional thermal resistance allowance.
    rmin=1000*.99*.98;rmax=1000*1.01*1.02;vmax=3.6
    for idx,(signal,gpio,pad) in enumerate([('PWR',15,'8'),('BLE',6,'6'),('CAN',7,'7'),('GPS',8,'12'),('SYS',10,'18'),('OIL',9,'17')],1):
        rr,dd=f'R60{idx}',f'D60{idx}';rn='LED_'+signal;an=rn+'_A'
        require(fitted(fs[rr])and fitted(fs[dd]),rr+' or '+dd+' not populated')
        require(c.prop(fs[rr])['MPN']=='CRCW06031K00FKEA','Unexpected LED resistor')
        require(c.prop(fs[dd])['MPN']=='APT1608SGC','Unexpected LED MPN')
        require(pins(fs[rr])=={'1':rn,'2':an}and pins(fs[dd])=={'1':'GND','2':an},'LED chain/polarity error '+signal)
        require(pins(fs['U201'])[pad]==rn,'Module LED pad net mismatch '+signal)
        require(re.search(r'#define\s+LED_'+signal+r'_GPIO\s+'+str(gpio)+r'\b',header),'Firmware LED GPIO mismatch')
        require(re.search(r'#define\s+LED_'+signal+r'_ACTIVE_LOW\s+0\b',header),'Firmware polarity mismatch')
        rows.append({'resistor':rr,'LED':dd,'signal':signal,'GPIO':gpio,'U201_pad':pad,'R_min_ohm':rmin,'R_max_ohm':rmax,'hard_current_upper_mA':1000*vmax/rmin,'resistor_power_upper_mW':1000*vmax*vmax/rmin,'brightness_lower_bound_mcd':0,'brightness_note':'No guaranteed positive luminance can be deduced at this low-current operating point from the available limits. This is not a claimed optical pass.'})
    return {'status':'PASS_ELECTRICAL_CHAIN_AND_FINITE_CURRENT_BOUND_OPTICAL_ACCEPTANCE_SEPARATE','rows':rows,'all_six_current_upper_mA':sum(r['hard_current_upper_mA']for r in rows),'allocation_notes':['3.6V operating ceiling; LED Vf is conservatively set to zero for the current upper bound.','Resistor minimum includes 1% initial tolerance and an additional 2% thermal decrease, an engineering allowance rather than a measured resistor temperature.','Current upper bounds do not assert a guaranteed loaded GPIO high voltage or light output.'],'respin_disposition':'Existing current-limited standard 0603 resistor/LED footprints permit assembly-value or optical-bin tuning without a new PCB layout. No brightness-dependent safety or measurement function is credited.','original_WCA_07_closed':False}

def current_handling(tree,old):
    previous_by_ref={ref:r for r in old['rows']for ref in r['references'].split()}
    by_mpn={r['mpn']:r for r in old['rows']}; groups={};excluded=[];changed=[]
    for ref,f in fp_map(tree).items():
        p=c.prop(f);mpn=p.get('MPN','')
        if not fitted(f):excluded.append({'reference':ref,'assembly':p.get('Assembly'),'mpn':mpn});continue
        require(bool(mpn),'Missing fitted MPN '+ref)
        groups.setdefault(mpn,[]).append(ref)
        if ref not in previous_by_ref or previous_by_ref[ref]['mpn']!=mpn:changed.append({'reference':ref,'before':previous_by_ref.get(ref,{}).get('mpn'),'after':mpn})
    rows=[]
    for mpn,refs in sorted(groups.items()):
        if mpn in by_mpn:r=copy.deepcopy(by_mpn[mpn])
        elif mpn in ('TNPU060311K8HWEA00','TNPU06034K99HWEA00'):
            r={'mpn':mpn,'manufacturer':'Vishay','process':'GLOBAL_REFLOW','msl':None,'manufacturer_peak_C':None,'source':'https://www.vishay.com/docs/28779/tnpue3.pdf','source_evidence':'PRIMARY_FAMILY_DATASHEET_28779_REV_04_MAR_2025_REVIEWED','handling':'TNPU e3 manufacturer Assembly section permits automatic wave/reflow/vapor-phase processing and common electronics cleaning solvents. Coating/potting compatibility remains application-specific. Numeric MSL and a numeric production peak are not stated here and are not invented.','current_release_condition':'Use the board-wide compatible process and incoming lot label. TNPU film limit125C and power derating apply; part-family solvent compatibility does not authorize cleaning the full assembly.'}
        else:raise ValueError('New fitted MPN needs explicit handling review: '+mpn)
        r.update(references=' '.join(sorted(refs)),quantity=len(refs));rows.append(r)
    require(sum(r['quantity']for r in rows)==153,'Unexpected fitted reference count')
    return {'status':'CURRENT_SOURCE_MPN_INVENTORY_RECONCILED','unique_fitted_MPNs':len(rows),'fitted_references':153,'rows':rows,'excluded':excluded,'corrected_references':changed,'MSL_primary_known_MPNs':sum(r['msl']is not None for r in rows),'numeric_MSL_primary_known_MPNs':sum(isinstance(r['msl'],int)for r in rows),'unknown_msl_MPNs':sum(r['msl']is None for r in rows),'physical_assembly_acceptance':False}

def verify_regressions(pcb,firmware):
    n=read(D/'results/NATIVE_RECHECK.json');f=read(D/'results/FIRMWARE_REGRESSION.json')
    require(digest(pcb)==PCB_HASH,'Candidate PCB changed; invalidate/re-run affected evidence')
    require(n['source_PCB_sha256']==PCB_HASH and n['filled_PCB_sha256']==FILLED_HASH,'Native binding mismatch')
    require(n['independent_manufacturing_status']=='PASS_INTENDED_COPPER_AND_EXPORTS','Independent native/export review failed')
    require(all(n[x]==0 for x in ['native_DRC','native_unconnected','native_parity','native_ERC']),'Native findings present')
    require(f['status']=='PASS' and len(f['results'])==9 and all(x['passed']for x in f['results']),'Firmware regression failure')
    for path,sha in f['source_files_sha256'].items():require(digest(firmware/path)==sha,'Firmware changed since regression: '+path)
    return {'native_postconditions':n['native_postconditions_passed'],'intended_nets':n['continuity']['nets'],'disconnected_pad_nets':n['continuity']['disconnected_pad_nets'],'firmware_suites':len(f['results']),'firmware_exact_source_files_bound':len(f['source_files_sha256']),'CAD_native_run_reused':34499455561,'native_copper_and_exports_independently_recomputed_this_iteration':True,'fresh_KiCad_execution_this_iteration':False,'ASan':True,'UBSan':True,'physical_testing':False}

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=D/'results/review');ap.add_argument('--evidence-root',type=Path,default=W);a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    pcb=W/'candidate/cad/GR86_CCA_RevB.kicad_pcb';fw=W/'candidate/firmware';tree=sx.loads(pcb.read_text());reg=read(W/'FINAL_REVIEW_REGISTER.json')
    evidence=verify_regressions(pcb,fw)
    rx=receive_only(tree,(fw/'cca_telemetry/cca_telemetry.ino').read_text());led=led_review(tree,(fw/'cca_telemetry/src/led_status.h').read_text())
    old=read(D/'support/HANDLING_I22.json');handling=current_handling(tree,old);handling['pcb_sha256']=digest(pcb)
    for name,data in [('RECEIVE_ONLY_DISPOSITION',rx),('LED_REVIEW',led),('CURRENT_HANDLING_REGISTER',handling),('REGRESSION_BINDING',evidence)]:save(a.out/(name+'.json'),data)
    original_counts=Counter(r['closure']for r in reg['rows']);coverage=[];filechecks={}
    for r in reg['rows']:
        files=[]
        for p,expected in r.get('evidence_sha256',{}).items():
            if p not in filechecks:
                q=a.evidence_root/p
                filechecks[p]={'path':p,'exists':q.is_file(),'expected_historical_sha256':expected,'actual_sha256':digest(q)if q.is_file()else None}
            files.append(filechecks[p])
        status=r['desktop_status']; category=''; action='';domain=r['id'].split('-')[0]
        if r['closure']=='na':category='NOT_APPLICABLE_RETAINED';action='Original not-applicable rationale retained; no fabricated result.'
        elif r['id']in ('REG-02','THERM-02'):
            category='ACTIVE_THERMAL_DESIGN';action='Finish full-load local thermal construction and package/local-air bounds; reject partial-path or colliding shoe proposals.'
        elif r['id']=='GND-02':
            category='RETURN_SCOPE_REVIEWED_BROADER_CLOSE_PENDING';action='Six remaining gaps are isolated CAN_TX stub geometry; actual receiver/RF paths remain referenced. Broader neck-down/return evidence must retain its stated finite scope.'
        elif r['id']=='WCA-07':
            category='NO_PCB_CHANGE_INDICATED_OPTICS_UNPROVEN';action='Six actual current-limited chains pass source/current checks. Optical lower bound remains zero; tune populated LED/resistor on existing footprints rather than redesign PCB for an unspecified visibility guarantee.'
        elif domain=='THERM':
            category='THERMAL_DEPENDENCY_RECHECK';action='Re-review even formerly closed thermal rows against the final heat path, source temperatures and actual selected material limits.'
        elif status=='COMPLETED_CONDITIONAL_MODEL':
            category='BOUNDED_ANALYSIS_RETAINED';action='Existing finite assumptions and failure cases remain explicit. Do not turn missing universal supplier guarantees into endless new calculations; do not imply an unknown is proven.'
        elif r['closure']=='closed':
            category='PRIOR_DESIGN_SCOPE_RETAINED';action='Retain original design-scope disposition, subject to source-dependency invalidation after the coordinated redesign. This is not a new physical or independent-human pass.'
        else:
            category='EXTERNAL_ACCEPTANCE_PARKED';action='Original test/supplier/installation requirement remains open outside this desktop completion denominator. Any design implication remains in its applicable engineering workstream.'
        coverage.append({'id':r['id'],'domain':r['B'],'original_status':r['closure'],'original_question':r['E'],'original_required_evidence':r['F'],'desktop_workstream':category,'disposition':action,'remaining_original_condition':r.get('remaining',''),'evidence_count':len(files),'missing_evidence_paths':[f['path']for f in files if not f['exists']],'changed_historical_evidence_paths':[f['path']for f in files if f['exists']and f['actual_sha256']!=f['expected_historical_sha256']],'post_redesign_review_required':r['closure']!='na','physical_test_claimed':False})
    require(len(coverage)==290 and len({r['id']for r in coverage})==290,'Criterion identity loss')
    save(a.out/'ALL_CRITERIA_COVERAGE.json',{'criterion_count':290,'original_counts':dict(original_counts),'workstream_counts':dict(Counter(r['desktop_workstream']for r in coverage)),'meaning':'Coverage/triage and dependency register, NOT 290 fresh engineering passes. Historical statuses are not rewritten by taxonomy.','rows':coverage})
    save(a.out/'EVIDENCE_AVAILABILITY.json',{'file_count':len(filechecks),'files':list(filechecks.values()),'hash_match_is_not_engineering_acceptance':True})
    summary={'status':'PASS_EXECUTED_SOURCE_AND_REGRESSION_CHECKS_CONVERGENCE_REMAINS_OPEN','source_PCB_sha256':digest(pcb),'original_criteria':dict(original_counts),'all_290_rows_retained':True,'four_original_DESKTOP_WORK_REMAINING':[r['id']for r in reg['rows']if r['desktop_status']=='DESKTOP_WORK_REMAINING'],'regression':evidence,'corrected_handling_refs':handling['corrected_references'],'actual_PCB_routing_changes_this_iteration':False,'finished_all_desktop_work':False,'fabrication_release':False,'notes':['The user means avoiding another PCB order AFTER first fabrication, not preserving the current draft.','Free pre-fabrication redesign is authorized within the product scope.','Changing taxonomy does not count as closing original criteria.','Mechanical/thermal design is the principal active coupled workstream; exact materials and model applicability remain consequential.','The high-temperature pigtail is a validation candidate, not an installed-unit claim.']}
    save(a.out/'SUMMARY.json',summary);print(json.dumps(summary,indent=2))
if __name__=='__main__':main()
