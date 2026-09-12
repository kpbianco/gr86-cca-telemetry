#!/usr/bin/env python3
"""Verify finite I26 outputs; optionally create an unreferenced, scoped Git commit.

Never updates a branch, merges, releases, flashes, or operates hardware.
"""
import argparse, base64, hashlib, json, os, subprocess, urllib.request, zipfile
from pathlib import Path
D=Path(__file__).resolve().parent; W=D.parents[1]; ROOT=W.parents[2]
REPO='tranquilWorks/gr86-cca-telemetry'
BRANCH='codex/rvb24-coordinated-closure'
P='docs/engineering/rvb22/'
OUT=ROOT/'i26-publication';OUT.mkdir(exist_ok=True)
GENERATED=['FINAL_REVIEW_REGISTER.json','FINAL_GATES.json','I26_PREHARDWARE_CLOSURE.json','I26_PREHARDWARE_CLOSURE.md','analyses/convergence_01/run_review.py']
GENERATED += ['analyses/i26/'+n for n in ['LED_BOUND.json','LED_THERMAL_REGIONS.json','HISTORICAL_54_RETURN_FINDINGS.json','EXPANDED_RETURN_SCOPE.json','TRANSFER_SCREEN_PATHS_EXPLORATORY.json','TRANSFER_SCREEN_EXPLORATORY.json','QUALIFICATION_GATES.json','RECONCILIATION_AUDIT.json']]
GENERATED += ['analyses/i26/current_review/'+n for n in ['ALL_CRITERIA_COVERAGE.json','CURRENT_HANDLING_REGISTER.json','EVIDENCE_AVAILABILITY.json','LED_REVIEW.json','RECEIVE_ONLY_DISPOSITION.json','REGRESSION_BINDING.json','SUMMARY.json']]
BEFORE={'FINAL_REVIEW_REGISTER.json':'84d7aa87d80d2efaf108d651c802ddaf597d28337430c677bc1c352539b159d3','I26_PREHARDWARE_CLOSURE.json':'68536e22fd25fa55a3eeeef45252839b05753c8b032c82f7bf89773de23f7763','FINAL_GATES.json':'72f2b7bd0a3bd3d36e663186accee0eecdda5e75f05f594bf95d91198fe4476c','analyses/convergence_01/run_review.py':'345308287d5a84fedaddf1ac8f885da235fd00743e4d0ef16179b8c1f5842cbb','I26_PREHARDWARE_CLOSURE.md':'34f2982e0ca42810888f7c7d3279d64e5873d865c9464ce2963c2c80b6c75c05'}

def run(args,log=None):
    result=subprocess.run(args,cwd=ROOT,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    if log:(OUT/log).write_text(result.stdout)
    print(result.stdout[-12000:],flush=True)
    if result.returncode:raise RuntimeError('Command failed: '+str(args))
    return result.stdout

def member(artifact_id,suffix,expected):
    archive=OUT/(str(artifact_id)+'.zip')
    with archive.open('wb') as f:
        subprocess.run(['gh','api',f'repos/{REPO}/actions/artifacts/{artifact_id}/zip'],stdout=f,check=True)
    with zipfile.ZipFile(archive) as z:
        matches=[n for n in z.namelist() if n.endswith(suffix)]
        if len(matches)!=1:raise ValueError('Ambiguous artifact member '+suffix)
        data=z.read(matches[0])
    if hashlib.sha256(data).hexdigest()!=expected:raise ValueError('Artifact content identity mismatch '+suffix)
    path=OUT/Path(suffix).name;path.write_bytes(data);return path

def verify():
    for name,expected in BEFORE.items():
        if hashlib.sha256((W/name).read_bytes()).hexdigest()!=expected:raise ValueError('Baseline advanced: '+name)
    native=member(10161339481,'native_I06_hosted/candidate_kicad/GR86_CCA_RevB.kicad_pcb','11533ea91c3bc4c61dd7066e0001914a72bc90b27afb38d321c43b8feb90e3b1')
    thermal=member(10273346331,'thermal/release_existing_cooling.npz','7e465b2b047e868bc58abd70ba62e3a202d7fe0a74c3e12b50341523f2221997')
    run(['python',str(D/'led_bound.py')],'led.log')
    run(['python',str(D/'return_review.py'),'--native',str(native)],'return.log')
    run(['python',str(D/'extract_led_thermal.py'),str(thermal)],'led-thermal.log')
    run(['python',str(D/'transfer_exploration.py'),'--native',str(native)],'transfer.log')
    run(['python',str(D/'summarize_transfer.py')],'transfer-summary.log')
    run(['git','apply','--check',str(D/'legacy_scope.patch')])
    run(['git','apply',str(D/'legacy_scope.patch')])
    run(['python',str(D/'reconcile.py')],'reconcile.log')
    run(['python','-m','unittest','discover','-s',str(D/'tests'),'-v'],'i26-tests.log')
    run(['python','-m','unittest','discover','-s',str(W/'analyses/convergence_01/tests'),'-v'],'regression-tests.log')
    checks=json.loads(run(['python',str(W/'analyses/convergence_04/check_release_profile.py')],'release-profile.log'))
    if len(checks)!=12 or not all(checks.values()):raise ValueError('Release guard incomplete')
    run(['python',str(W/'analyses/convergence_01/run_review.py'),'--out',str(D/'current_review')],'current-review.log')
    run(['python',str(W/'i26_audit_open_rows.py')],'register-audit.log')
    verification={'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),'I26_unit_tests':32,'existing_regression_tests':31,'release_profile_checks':12,'all_passed':True,'physical_tests':0,'native_KiCad_executed_in_this_job':False,'actionable_prehardware_ids':['GND-02'],'full_external_gate_reaudit_claimed':False}
    (OUT/'VERIFICATION.json').write_text(json.dumps(verification,indent=2)+'\n')
    with zipfile.ZipFile(OUT/'I26_GENERATED_EVIDENCE.zip','w',zipfile.ZIP_DEFLATED) as z:
        for name in GENERATED:z.write(W/name,P+name)
        for path in OUT.glob('*.log'):z.write(path,'execution/'+path.name)
        z.write(OUT/'VERIFICATION.json','execution/VERIFICATION.json')

def api(method,endpoint,payload=None):
    req=urllib.request.Request('https://api.github.com/repos/'+REPO+'/'+endpoint,data=json.dumps(payload).encode() if payload is not None else None,method=method,headers={'Authorization':'Bearer '+os.environ['GH_TOKEN'],'Accept':'application/vnd.github+json','Content-Type':'application/json','User-Agent':'i26-scoped-publication'})
    with urllib.request.urlopen(req,timeout=90) as response:return json.load(response)

def prepare_commit():
    evidence=json.loads((OUT/'VERIFICATION.json').read_text());head=evidence['source_commit']
    if os.environ.get('GITHUB_REPOSITORY')!=REPO or not evidence['all_passed']:raise ValueError('Wrong repository or verification state')
    if api('GET','git/ref/heads/'+BRANCH)['object']['sha']!=head:raise ValueError('Branch advanced; do not overwrite')
    allowed={P+n for n in GENERATED}
    modified=set(subprocess.check_output(['git','diff','--name-only'],cwd=ROOT,text=True).splitlines())
    if modified-allowed:raise ValueError('Out-of-scope tracked mutations: '+str(modified-allowed))
    elements=[];manifest={}
    for name in GENERATED:
        data=(W/name).read_bytes();manifest[P+name]=hashlib.sha256(data).hexdigest()
        blob=api('POST','git/blobs',{'content':base64.b64encode(data).decode(),'encoding':'base64'})
        elements.append({'path':P+name,'mode':'100644','type':'blob','sha':blob['sha']})
    state=api('GET','git/commits/'+head)
    tree=api('POST','git/trees',{'base_tree':state['tree']['sha'],'tree':elements})
    commit=api('POST','git/commits',{'message':'Reconcile I26 LED and thermal criteria; retain explicit ground-return desktop work','tree':tree['sha'],'parents':[head]})
    result={'parent':head,'prepared_commit':commit['sha'],'prepared_tree':tree['sha'],'branch_updated':False,'physical_tests':0,'verification':evidence,'sha256':manifest}
    (OUT/'PUBLICATION_RESULT.json').write_text(json.dumps(result,indent=2)+'\n')
    print('I26_PREPARED_COMMIT '+commit['sha'])

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('action',choices=['verify','prepare-commit']);args=parser.parse_args()
    verify() if args.action=='verify' else prepare_commit()
