#!/usr/bin/env python3
"""Find all independently rejected top-level PCB records using native KiCad.

Diagnostic files are incomplete boards, never release candidates or DRC passes.
The input board is read only. No native exceptions escape through PCB_IO_MGR.
"""
from pathlib import Path
import sys,json,hashlib,re,time
import pcbnew as p
src=Path(sys.argv[1]).resolve();out=Path(sys.argv[2]).resolve();out.mkdir(parents=True,exist_ok=False)
text=src.read_text();before=hashlib.sha256(src.read_bytes()).hexdigest()
def blocks(text):
 depth=0;quoted=False;esc=False;start=None
 for i,c in enumerate(text):
  if quoted:
   if esc:esc=False
   elif c=='\\':esc=True
   elif c=='"':quoted=False
  elif c=='"':quoted=True
  elif c=='(':
   if depth==1:start=i
   depth+=1
  elif c==')':
   depth-=1
   if depth==1 and start is not None:yield start,i+1,text[start:i+1]
 assert depth==0 and not quoted
records=[]
for a,b,s in blocks(text):
 tag=re.match(r'\(\s*([^\s()]+)',s)[1]
 records.append({'tag':tag,'start_byte':len(text[:a].encode()),'end_byte':len(text[:b].encode()),'text':s})
meta={'version','generator','generator_version','general','paper','layers','setup','net','embedded_fonts'}
base=[r for r in records if r['tag'] in meta];items=[r for r in records if r['tag'] not in meta]
count=0;tests=[]
def test(rs,label):
 global count
 count+=1;path=out/f'probe_{count:04d}.kicad_pcb'
 path.write_text('(kicad_pcb\n'+'\n'.join(r['text'] for r in rs)+'\n)\n')
 t=time.monotonic();board=p.LoadBoard(str(path));ok=board is not None
 tests.append({'index':count,'label':label,'records':len(rs),'load_pass':ok,'seconds':round(time.monotonic()-t,3)})
 print(json.dumps(tests[-1]),flush=True)
 return ok
bad=[]
def split(rs,label):
 if not rs or test(base+rs,label):return
 if len(rs)==1:
  r=rs[0];name=f'rejected_{len(bad)+1:03d}_{r["tag"]}.sexp';(out/name).write_text(r['text']+'\n')
  r2={k:v for k,v in r.items() if k!='text'};r2.update({'record_file':name,'sha256':hashlib.sha256(r['text'].encode()).hexdigest(),'reference':re.search(r'\(property\s+"Reference"\s+"([^"]+)"',r['text']).group(1) if '(property "Reference"' in r['text'] else None});bad.append(r2);return
 mid=len(rs)//2;split(rs[:mid],label+'.a');split(rs[mid:],label+'.b')
base_ok=test(base,'source metadata only')
if base_ok:
 for tag in sorted({r['tag'] for r in items}):split([r for r in items if r['tag']==tag],tag)
else:
 # Report exact metadata blocks rather than pretending downstream isolation is meaningful.
 for r in base:
  if r['tag']!='net':(out/f'metadata_{r["tag"]}.sexp').write_text(r['text']+'\n')
result={'status':'NATIVE_PARSER_ISOLATION_DIAGNOSTIC','version':p.GetBuildVersion(),'source_sha256':before,'base_load_pass':base_ok,'tests':tests,'rejected_records':bad,'not_a_DRC_or_board_connectivity_pass':True,'source_unchanged':hashlib.sha256(src.read_bytes()).hexdigest()==before,'interaction_limit':'This isolates individually rejected records with shared metadata. Combination-only failures and metadata failures require follow-up.'}
(out/'RESULT.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({k:v for k,v in result.items() if k!='tests'}),flush=True)
