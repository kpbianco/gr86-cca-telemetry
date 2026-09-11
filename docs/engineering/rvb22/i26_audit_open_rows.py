#!/usr/bin/env python3
import json, pathlib, collections

P = pathlib.Path(__file__).with_name('FINAL_REVIEW_REGISTER.json')
data = json.loads(P.read_text(encoding='utf-8'))

STATUS_KEYS = ('status','Status','current_status','disposition_status','review_status')
ID_KEYS = ('id','ID','criterion_id','criterion','check_id','requirement_id')
TEXT_KEYS = ('title','name','criterion','requirement','description','text','acceptance','acceptance_criteria','evidence','rationale','reason','open_reason','disposition','notes','action','next_action','closure_basis')

def walk(x, path='$'):
    if isinstance(x, dict):
        yield path, x
        for k,v in x.items():
            yield from walk(v, f'{path}.{k}')
    elif isinstance(x, list):
        for i,v in enumerate(x):
            yield from walk(v, f'{path}[{i}]')

def get_any(d, keys):
    for k in keys:
        if k in d and d[k] not in (None,''):
            return d[k]
    return None

def norm_status(v):
    return str(v or '').strip().lower().replace('_',' ').replace('-',' ')

rows=[]
seen=set()
for path,d in walk(data):
    st=get_any(d,STATUS_KEYS)
    rid=get_any(d,ID_KEYS)
    if st is None or rid is None:
        continue
    s=norm_status(st)
    if not any(t in s for t in ('open','closed','pass','fail','not applicable','n/a','na')):
        continue
    key=(str(rid),path)
    if key in seen: continue
    seen.add(key)
    fields={k:d[k] for k in TEXT_KEYS if k in d and d[k] not in (None,'')}
    rows.append({'path':path,'id':str(rid),'status':str(st),'fields':fields,'keys':sorted(d.keys())})

# Prefer the largest homogeneous criterion collection if recursive duplicates exist.
# Report every candidate, but also emit unique criterion ids using the shallowest occurrence.
byid={}
for r in sorted(rows,key=lambda z:(z['path'].count('.'),z['path'].count('['),len(z['path']))):
    byid.setdefault(r['id'],r)
uniq=list(byid.values())
counts=collections.Counter(norm_status(r['status']) for r in uniq)
open_rows=[r for r in uniq if 'open' in norm_status(r['status']) or 'fail' in norm_status(r['status'])]

print('I26_REGISTER_AUDIT_V1')
print(json.dumps({'unique_rows':len(uniq),'status_counts':dict(counts),'open_rows':len(open_rows)},sort_keys=True))
for r in sorted(open_rows,key=lambda z:z['id']):
    print('I26_OPEN '+json.dumps(r,sort_keys=True,ensure_ascii=False))

# Also dump root/schema hints so the audit can be repaired without guessing if discovery missed rows.
root_keys=sorted(data.keys()) if isinstance(data,dict) else []
print('I26_ROOT '+json.dumps({'type':type(data).__name__,'keys':root_keys},sort_keys=True))
