#!/usr/bin/env python3
import json, pathlib, collections
P=pathlib.Path(__file__).with_name('FINAL_REVIEW_REGISTER.json')
data=json.loads(P.read_text(encoding='utf-8'))
rows=data['rows']

def norm(v): return str(v or '').strip().lower().replace('_',' ').replace('-',' ')
def compact(v,n=2400):
    s=json.dumps(v,ensure_ascii=False,sort_keys=True) if isinstance(v,(dict,list)) else str(v)
    return s if len(s)<=n else s[:n]+'…'

print('I26_CONTROLLED_ROWS_AUDIT_V3')
print('I26_ROWS_SCHEMA '+json.dumps({'count':len(rows),'first_keys':sorted(rows[0].keys())},sort_keys=True))
counts=collections.Counter(str(r.get('desktop_status')) for r in rows)
print('I26_DESKTOP_COUNTS '+json.dumps(dict(counts),sort_keys=True))

nonclosed=[]
for i,r in enumerate(rows):
    ds=str(r.get('desktop_status') or '')
    # Existing register uses Closed/Open/N/A in desktop_status.
    if norm(ds) not in ('closed','n/a','na','not applicable'):
        nonclosed.append((i,r))
print('I26_NONCLOSED_COUNT '+str(len(nonclosed)))
for i,r in nonclosed:
    payload={
      'index':i,'id':r.get('id'),'row':r.get('row'),'desktop_status':r.get('desktop_status'),
      'A':compact(r.get('A')),'B':compact(r.get('B')),'C':compact(r.get('C')),'D':compact(r.get('D')),'E':compact(r.get('E')),'F':compact(r.get('F')),
      'observation':compact(r.get('observation')),'closure':compact(r.get('closure')),
      'remaining':compact(r.get('remaining')),'I22_reassessment':compact(r.get('I22_reassessment')),
      'evidence_status':r.get('evidence_status'),'physical_test_claimed':r.get('physical_test_claimed'),
      'fresh_native_execution_claimed':r.get('fresh_native_execution_claimed')
    }
    print('I26_ROW_NONCLOSED '+json.dumps(payload,ensure_ascii=False,sort_keys=True))
