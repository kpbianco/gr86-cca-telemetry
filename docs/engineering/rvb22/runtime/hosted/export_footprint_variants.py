"""Generate reviewable native library variants for recorded footprint mismatches.

Writes only a new proposal directory. Does not change input CAD or suppress DRC.
KiCad's FootprintSave normalizes orientation/front side in the library copy.
"""
from pathlib import Path
import sys,json,hashlib
import pcbnew as p
board_path,report_path,out=map(Path,sys.argv[1:4]);assert not out.exists();out.mkdir(parents=True)
assert '9.0.9' in p.GetBuildVersion()
def sha(q):return hashlib.sha256(q.read_bytes()).hexdigest()
original=sha(board_path);b=p.LoadBoard(str(board_path));assert b is not None
report=json.loads(report_path.read_text());refs=set()
for v in report['violations']:
 if v['type']=='lib_footprint_mismatch':
  for i in v['items']:
   if i['description'].startswith('Footprint '):refs.add(i['description'].split(' ',1)[1])
lib=out/'RevB_variants.pretty';lib.mkdir();rows=[]
plugin=p.PCB_IO_MGR.PluginFind(p.PCB_IO_MGR.KICAD_SEXP);assert plugin is not None
for ref in sorted(refs):
 f=b.FindFootprintByReference(ref);assert f is not None
 old=f.GetFPID().GetUniStringLibItemName();name=str(old)+'_RVB22_'+ref
 ident=p.LIB_ID('RevB',name);f.SetFPID(ident)
 plugin.FootprintSave(str(lib.resolve()),f);q=lib/(name+'.kicad_mod');assert q.is_file() and q.stat().st_size>100
 rows.append({'reference':ref,'previous_library_item':str(old),'proposed_library_item':name,'path':str(q.relative_to(out)),'sha256':sha(q),'bytes':q.stat().st_size})
assert sha(board_path)==original
(out/'VARIANT_PROPOSALS.json').write_text(json.dumps({'status':'NATIVE_LIBRARY_PROPOSALS_NOT_YET_ADOPTED','source_pcb_sha256':original,'source_drc_sha256':sha(report_path),'kicad':p.GetBuildVersion(),'rows':rows,'next_action':'Review variant geometry against its intended pad/assembly requirements; adopt matching board and schematic footprint IDs together; rerun native DRC.'},indent=2)+'\n')
print(json.dumps({'proposals':len(rows),'input_unchanged':True}))
