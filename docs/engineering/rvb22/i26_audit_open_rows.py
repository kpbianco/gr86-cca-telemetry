#!/usr/bin/env python3
import json, pathlib, collections

P = pathlib.Path(__file__).with_name('FINAL_REVIEW_REGISTER.json')
data = json.loads(P.read_text(encoding='utf-8'))
rows = data.get('rows')
if not isinstance(rows, list):
    raise SystemExit(f"FINAL_REVIEW_REGISTER rows is {type(rows).__name__}, expected list")

STATUS_KEYS=('status','Status','current_status','review_status')
ID_KEYS=('id','ID','criterion_id','criterion','check_id','requirement_id')
TEXT_KEYS=('category','lane','title','name','criterion','requirement','description','text','acceptance','acceptance_criteria','evidence','rationale','reason','open_reason','disposition','notes','action','next_action','closure_action','closure_basis','remaining')

def get_any(d,keys):
    for k in keys:
        if k in d and d[k] not in (None,''):
            return d[k]
    return None

def norm(v):
    return str(v or '').strip().lower().replace('_',' ').replace('-',' ')

def compact(v,limit=1800):
    s=json.dumps(v,ensure_ascii=False,sort_keys=True) if isinstance(v,(dict,list)) else str(v)
    return s if len(s)<=limit else s[:limit]+'…'

print('I26_CONTROLLED_ROWS_AUDIT_V2')
print('I26_ROWS_SCHEMA '+json.dumps({'count':len(rows),'first_keys':sorted(rows[0].keys()) if rows and isinstance(rows[0],dict) else []},sort_keys=True))

out=[]
for i,d in enumerate(rows):
    if not isinstance(d,dict):
        print('I26_BAD_ROW '+json.dumps({'index':i,'type':type(d).__name__}))
        continue
    rid=get_any(d,ID_KEYS)
    st=get_any(d,STATUS_KEYS)
    fields={k:d[k] for k in TEXT_KEYS if k in d and d[k] not in (None,'')}
    out.append({'index':i,'id':str(rid) if rid is not None else f'ROW-{i:03d}','status':str(st),'fields':fields,'keys':sorted(d.keys())})

counts=collections.Counter(norm(r['status']) for r in out)
open_rows=[r for r in out if 'open' in norm(r['status']) or norm(r['status']) in ('fail','failed')]
closed_rows=[r for r in out if 'closed' in norm(r['status']) or norm(r['status']) in ('pass','passed')]
na_rows=[r for r in out if norm(r['status']) in ('na','n/a','not applicable') or 'not applicable' in norm(r['status'])]
print('I26_ROWS_COUNTS '+json.dumps({'rows':len(out),'status_counts':dict(counts),'open':len(open_rows),'closed_like':len(closed_rows),'na_like':len(na_rows)},sort_keys=True))
for r in open_rows:
    rr=dict(r)
    rr['fields']={k:compact(v) for k,v in r['fields'].items()}
    print('I26_ROW_OPEN '+json.dumps(rr,sort_keys=True,ensure_ascii=False))
