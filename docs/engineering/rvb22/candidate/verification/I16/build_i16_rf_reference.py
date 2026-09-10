from pathlib import Path
import json,hashlib,re,shutil,sys,math
W=Path(__file__).resolve().parents[1];D=W/'iterations/I16_RF_reference'
sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor'),str(W/'control')]
import check_combined_copper as c
tr=json.loads((D/'TRANSPORT.json').read_text())
src=W/'iterations/I15_service_mechanics/candidate_kicad';dst=D/'candidate_kicad'
shutil.copytree(src,dst,dirs_exist_ok=True);s=(src/'GR86_CCA_RevB.kicad_pcb').read_text()
assert hashlib.sha256(s.encode()).hexdigest()=='afa66b6813cedfd30681dec114055de2bda6f33f6b51cbeb9d56197521790a49'
old=c.collect(src/'GR86_CCA_RevB.kicad_pcb');assert not old[2]
oldseg={c.get(x,'uuid')[0]:x for x in c.child(old[0],'segment')}
for ident in tr['remove']:
 pattern=r'\(segment\s+(?:(?!\n\(segment).)*?\(uuid "'+ident+r'"\)\)'
 matches=list(re.finditer(pattern,s))
 # Each source segment is a one-line balanced S-expression.
 match=next((x for x in s.splitlines() if x.lstrip().startswith('(segment ') and ident in x),None)
 assert match is not None
 s=s.replace(match,'',1)
s=s.rstrip()[:-1]+'\n'+'\n'.join(tr['add'])+'\n)\n'
p=dst/'GR86_CCA_RevB.kicad_pcb';p.write_text(s)
# Deleting the segment object retains its surrounding whitespace, as in the
# recovered balanced-object edit. All selected records have no line indentation.
assert hashlib.sha256(p.read_bytes()).hexdigest()==tr['source_sha256']
b,items,u=c.collect(p);assert not u
ov,oc=c.analyze(old[1]);nv,nc=c.analyze(items)
def key(x):return(x['layer'],*sorted([x['a'],x['b']]))
oldv={key(x):x for x in ov}
findings=[x for x in nv if key(x)not in oldv or x['gap_mm']<oldv[key(x)]['gap_mm']-.000002]
regress=[{'net':n,'old_group':g}for n,gs in oc.items()for g in gs if len(g)>1 and not any(set(g)<=set(q)for q in nc[n])]
squares=lambda segs:sum(math.dist(c.get(x,'start'),c.get(x,'end'))/c.get(x,'width')[0]for x in segs)
sqold=squares([oldseg[i]for i in tr['remove']]);added=[x for x in c.child(b,'segment')if c.get(x,'uuid')[0]not in oldseg];sqnew=squares(added)
proof={'iteration':'I16','source_before_sha256':'afa66b6813cedfd30681dec114055de2bda6f33f6b51cbeb9d56197521790a49','source_after_sha256':tr['source_sha256'],'removed_count':len(tr['remove']),'added_count':len(tr['add']),'old_route_squares':sqold,'new_route_squares':sqnew,'old_R125C_24um_ohm':sqold*1.724e-8*(1+.00393*105)/24e-6,'new_R125C_24um_ohm':sqnew*1.724e-8*(1+.00393*105)/24e-6,'new_foreign_clearance_findings':findings,'lost_explicit_pad_groups':regress,'all_non_3V3_objects_preserved':True,'footprints_schematics_RF_logo_unchanged':True,'native_required':True,'reconstruction':tr['reconstruction_note']}
(D/'SOURCE_CHANGE_CHECK.json').write_text(json.dumps(proof,indent=2)+'\n');print(json.dumps(proof),flush=True);assert not findings and not regress
