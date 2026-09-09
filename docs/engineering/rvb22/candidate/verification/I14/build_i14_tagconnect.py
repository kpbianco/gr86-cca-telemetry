"""I14 local service-header correction, preserving all other source objects."""
from pathlib import Path
import json,hashlib,uuid,re,sys,shutil
W=Path(__file__).resolve().parents[1];D=W/'iterations/I14_service_clearance';D.mkdir(exist_ok=True)
sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor'),str(W/'control')]
import check_combined_copper as c
import sexpdata as sx
from shapely.geometry import LineString,Point
src=W/'iterations/I13_thermal_vias/candidate_kicad';dst=D/'candidate_kicad'
pcb=src/'GR86_CCA_RevB.kicad_pcb';s=pcb.read_text();b,old_items,u=c.collect(pcb);assert not u
header=next(f for f in c.child(b,'footprint') if c.prop(f)['Reference']=='J201')
pin={str(p[1]):p for p in c.child(header,'pad') if p[1]};nets={k:c.get(p,'net')[0] for k,p in pin.items()}
def service_findings(items):
 out=[]
 for k,p in pin.items():
  g=c.pad_shape(header,p)
  for o in items:
   if o['layer']!='F.Cu' or o['net']==nets[k] or o['id'].startswith('J201.'):continue
   d=g.distance(o['geometry'])
   if d<.508-1e-7:out.append({'pad':k,'object':o['id'],'type':o['type'],'gap_mm':d})
 return out
before=service_findings(old_items)
freeze={'iteration':'I14','base_source_sha256':hashlib.sha256(pcb.read_bytes()).hexdigest(),'known_redlines':[
 {'id':'I14-01','finding':'Tag-Connect manufacturer requires .508mm from contact pads to other traces/signals; native generic clearance did not encode this.','evidence':before,'action':'Outward contact exits, remove close ground via, move pin4 via, add dedicated rule; full native refill and DRC.'},
 {'id':'I14-02','finding':'Full-load thermal calculation remains mesh-sensitive and package temperatures unproven; I13 via trial lowers MCU hotspot but increases local capacitor-board heating.','action':'Retain full4.815W corner and complete native-fill thermal comparison; no thermal approval from DRC.'},
 {'id':'I14-03','finding':'Inferred Adafruit851 PDF limits cable to60C and calls its interface RP-SMA while product page calls SMA. Exact purchased adapter identity was not given.','action':'Document incompatibility with65C design air; dimension an explicit SMA/U.FL compatible high-temperature alternative for review.'},
 {'id':'I14-04','finding':'Mated connector/service geometry and updated 290-criterion register still incomplete.','action':'Complete assembly-envelope and controlled-evidence disposition.'}],
 'rule':'Freeze before source edits; retain all failed trials and keep physical/supplier conditions distinct.'}
(D/'REDLINE_FREEZE.json').write_text(json.dumps(freeze,indent=2)+'\n')
shutil.copytree(src,dst,dirs_exist_ok=True)
def children(text):
 depth=0;quote=False;esc=False;start=None
 for i,ch in enumerate(text):
  if quote:
   if esc:esc=False
   elif ch=='\\':esc=True
   elif ch=='"':quote=False
  elif ch=='"':quote=True
  elif ch=='(': 
   if depth==1:start=i
   depth+=1
  elif ch==')':
   depth-=1
   if depth==1 and start is not None:yield start,i+1,text[start:i+1]
 assert depth==0 and not quote
records=list(children(s));byid={}
for lo,hi,t in records:
 if re.match(r'\((segment|via|arc)\s',t):
  ident=re.search(r'\(uuid\s+"([^"]+)"\)',t).group(1);byid[ident]=(lo,hi,t)
remove=['f883f0b5-c956-4186-b9bd-acb9e6f1e802','2095b26c-025b-4bdb-96b9-d0ecc052ee30','2baf002d-c43e-4d38-8231-a298760823f0','723d44f4-5450-4ee5-9f8a-76d1ca3c9a84','850dccfb-508a-4950-a8e6-054933156fe8',
 '03b76b6e-cd21-473b-b571-ac336050385e','5d39fb2f-ff7b-4a91-96c2-f2b556eeb54f','7b76a129-7f14-4d8e-bb4b-aea648cfc3ce','b37841d1-de70-443f-a429-904a0690b650','ca9632d6-af98-427a-970d-2ac7c090de17']
edits=[(*byid[k][:2],'') for k in remove]
modified=[]
for ident in ['2fc920da-e441-49b3-99ee-0bc322e4db58','aafc8db5-4786-4911-a842-6b7277ddfa88']:
 lo,hi,t=byid[ident];q=t.replace('70.4351 4.6893','71 4.79');assert q!=t
 edits.append((lo,hi,q));modified.append(ident)
new=[]
def route(k,points,width,layer='F.Cu'):
 for a,z in zip(points,points[1:]):
  key='I14:J201:'+str(k)+':'+str(a)+':'+str(z)+':'+layer
  ident=str(uuid.uuid5(uuid.NAMESPACE_URL,key));new.append('(segment (start %s %s) (end %s %s) (width %s) (layer "%s") (net %s) (uuid "%s"))'%(*a,*z,width,layer,nets[k],ident))
route('1',[(69.73,6.635),(69.73,8.4),(71.8243,8.7292)],.6)
route('3',[(71,6.635),(71,7.6)],.3)
route('4',[(71,5.365),(71,4.79)],.2)
route('6',[(72.27,5.365),(72.27,4.5),(73,3.8),(74,3.8),(74.5887,4.3887),(74.5887,6)],.2)
for lo,hi,t in sorted(edits,reverse=True):s=s[:lo]+t+s[hi:]
s=s.rstrip();assert s.endswith(')');s=s[:-1]+'\n'+'\n'.join(new)+'\n)\n'
out=dst/pcb.name;out.write_text(s)
rule='''
# Tag-Connect TC2030-IDC-NL RevB, footprint note2:0.020in.
# Adjacent intended contact pads are the manufacturer's array; other copper
# is kept0.508mm away. Different-net clearance is evaluated natively.
(rule "J201 programming contact foreign-copper clearance"
  (condition "A.Type == 'Pad' && A.Reference == 'J201' && A.Pad_Type == 'SMD' && !(B.Type == 'Pad' && B.Reference == 'J201')")
  (constraint clearance (min 0.508mm)))
'''
p=dst/'GR86_CCA_RevB.kicad_dru';p.write_text(p.read_text()+rule)
_,items,u=c.collect(out);assert not u
after=service_findings(items);changed=set(modified)|{re.search(r'\(uuid "([^"]+)"',q).group(1) for q in new};hits=[]
for a in items:
 if a['id'] not in changed:continue
 for z in items:
  if a['layer']!=z['layer'] or a['net']==z['net'] or a['id']==z['id']:continue
  gap=a['geometry'].distance(z['geometry'])
  if gap<.15-1e-7:hits.append({'a':a['id'],'b':z['id'],'layer':a['layer'],'gap_mm':gap})
report={'source_before_sha256':freeze['base_source_sha256'],'source_after_sha256':hashlib.sha256(out.read_bytes()).hexdigest(),'removed_objects':remove,'modified_objects':modified,'added_segments':len(new),'pin_nets':nets,'manufacturer_clearance_findings_before':before,'manufacturer_clearance_findings_after':after,'changed_copper_general_preflight':hits,'native_refill_and_DRC_required':True,'scope':'Explicit copper only; native custom-rule refill must validate zones, no-fly areas, holes and connectivity. Removed pin3 GND via is replaced by outward front copper connection to native GND pour.'}
(D/'SOURCE_CHANGE_CHECK.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2));assert not after and not hits
