"""Independent intended-copper, reference-plane and manufacturing-export checks.

Consumes native KiCad outputs with a separate Gerbonara parser and Shapely.
No claim is made about a manufactured contact, solder joint or plated barrel.
"""
from pathlib import Path
import sys,json,hashlib,collections,math,xml.etree.ElementTree as ET,argparse,csv
W=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(Path(__file__).resolve().parent/'support'))
import check_combined_copper as c
import shapely as sh
from shapely.geometry import Polygon,LineString,Point
from shapely.ops import unary_union
from shapely.strtree import STRtree
from gerbonara.rs274x import GerberFile
from gerbonara.excellon import ExcellonFile
ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('--native-dir',type=Path,required=True)
ap.add_argument('--expected-hash',required=True,help='Expected SHA256 of native filled PCB')
ap.add_argument('--source-pcb',type=Path,required=True)
ap.add_argument('--expected-source-hash',required=True)
ap.add_argument('--output-dir',type=Path,default=Path(__file__).resolve().parent)
args=ap.parse_args()
N=args.native_dir.resolve();D=args.output_dir.resolve();D.mkdir(parents=True,exist_ok=True)
assert hashlib.sha256(args.source_pcb.read_bytes()).hexdigest()==args.expected_source_hash,'Source PCB hash mismatch'
assert hashlib.sha256((N/'candidate_kicad/GR86_CCA_RevB.kicad_pcb').read_bytes()).hexdigest()==args.expected_hash,'Filled PCB hash mismatch'
P=N/'candidate_kicad/GR86_CCA_RevB.kicad_pcb';b,items,unsupported=c.collect(P);assert not unsupported
names={n[1]:n[2]for n in c.child(b,'net')};pcbsha=hashlib.sha256(P.read_bytes()).hexdigest()
native=json.loads((N/'RESULT.json').read_text());dr=json.loads((N/'DRC.json').read_text());er=json.loads((N/'ERC.json').read_text())
assert native['inputs']['cad']['GR86_CCA_RevB.kicad_pcb']==args.expected_source_hash,'Native input differs from candidate'
assert len(native['output_postconditions'])==14 and all(v['pass']for v in native['output_postconditions'].values()),'Native postcondition failure'
assert native['native_report_finding_count']==0,'Native reports contain findings'
report={'filled_PCB_sha256':pcbsha,'source_PCB_sha256':native['inputs']['cad']['GR86_CCA_RevB.kicad_pcb'],'native_status':native['status'],'native_DRC':len(dr['violations']),'native_unconnected':len(dr['unconnected_items']),'native_parity':len(dr['schematic_parity']),'native_ERC':sum(len(s['violations'])for s in er['sheets']),'scope':'Intended CAD and matched export evidence; no manufactured continuity, plating, solder or cable-contact measurement.'}
report['iteration']='I24'
report['native_directory']=str(N.relative_to(W)) if N.is_relative_to(W) else str(N)
report['analyzer_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
report['native_postconditions_passed']=len(native['output_postconditions'])
report['native_result_sha256']=hashlib.sha256((N/'RESULT.json').read_bytes()).hexdigest()
filled={l:[]for l in c.L};gnd={l:[x['geometry']for x in items if x['layer']==l and x['net']==30]for l in c.L}
for z in c.child(b,'zone'):
 if c.child(z,'keepout'):continue
 for f in c.child(z,'filled_polygon'):
  layer=c.get(f,'layer')[0];g=sh.make_valid(Polygon([p[1:]for p in c.child(c.child(f,'pts')[0],'xy')]));filled[layer].append(g)
  items.append({'id':str(c.get(z,'uuid')[0])+':'+str(len(items)),'net':c.get(z,'net')[0],'layer':layer,'geometry':g,'type':'zone','bridge':None})
  if c.get(z,'net')[0]==30:gnd[layer].append(g)
gnd={l:unary_union(gg)for l,gg in gnd.items()}
# Connectivity graph: geometric intersections on each copper layer, explicit
# via/PTH barrel bridges between layers. 2nm contact tolerance handles export
# decimal quantization, not fabrication tolerance or over-etch.
ds=c.DSU(len(items));bridge={}
for i,x in enumerate(items):
 if x['bridge']:
  key=(x['bridge'],x['net'])
  if key in bridge:ds.union(i,bridge[key])
  else:bridge[key]=i
for layer in c.L:
 ids=[i for i,x in enumerate(items)if x['layer']==layer];gg=[items[i]['geometry']for i in ids];tree=STRtree(gg)
 for j,g in enumerate(gg):
  for k in tree.query(g.buffer(.000002)):
   if k<=j:continue
   a,z=ids[j],ids[int(k)]
   if items[a]['net']==items[z]['net']and g.distance(gg[k])<=.000002:ds.union(a,z)
groups=collections.defaultdict(lambda:collections.defaultdict(set));allgroups=collections.defaultdict(list)
for i,x in enumerate(items):
 allgroups[(x['net'],ds.find(i))].append(x['id'])
 if x['type']=='pad'and x['net']:groups[names[x['net']]][ds.find(i)].add(x['id'])
disconnected={n:[sorted(q)for q in gs.values()]for n,gs in groups.items()if len(gs)>1}
padless=[{'net':names.get(n,n),'items':v}for(n,k),v in allgroups.items()if n and not any(items[i]['type']=='pad'and ds.find(i)==k for i in range(len(items)))]
report['continuity']={'nets':len(groups),'copper_objects':len(items),'disconnected_pad_nets':disconnected,'padless_components':padless}
print('Continuity complete',flush=True)
# Reference screen: classify every critical outer signal and its own via/pad
# antipads. The other reference plane is retained as an alternate-depth screen.
critical_names={'ADC_NODE','CANH','CANL','CAN_TX_MCU','CAN_RX_MCU','GPS_ANT_RF_BIASED','GPS_EXT_ANT','GPS_TX_RAW','GPS_TX_MCU','GPS_TX_BUFFER','GPS_RX_MODULE','GPS_PPS_RAW','GPS_PPS_BUFFER','GPS_PPS_MCU','OIL_EXCITATION_ADC','UART0_RX','UART0_TX'}
refs=[]
for s in c.child(b,'segment'):
 net=c.get(s,'net')[0];name=names[net];layer=c.get(s,'layer')[0]
 if name not in critical_names or layer not in ['F.Cu','B.Cu']:continue
 ref='In1.Cu'if layer=='F.Cu'else'In2.Cu';other='In2.Cu'if layer=='F.Cu'else'In1.Cu';line=LineString([c.get(s,'start'),c.get(s,'end')]);own=unary_union([x['geometry'].buffer(.155)for x in items if x['layer']==ref and x['net']==net and x['type']in ['via','pad']])
 missing=line.difference(gnd[ref].buffer(.000002)).difference(own)
 both=missing.difference(gnd[other].buffer(.000002))
 refs.append({'net':name,'uuid':c.get(s,'uuid')[0],'layer':layer,'reference':ref,'start':c.get(s,'start'),'end':c.get(s,'end'),'length_mm':line.length,'adjacent_gap_after_own_antipads_mm':missing.length,'both_ground_planes_gap_mm':both.length,'adjacent_gap_WKT':missing.wkt,'both_gap_WKT':both.wkt})
report['reference']={'RF':[x for x in refs if x['net']in ['GPS_ANT_RF_BIASED','GPS_EXT_ANT']],'other_findings':[x for x in refs if x['net']not in ['GPS_ANT_RF_BIASED','GPS_EXT_ANT']and x['adjacent_gap_after_own_antipads_mm']>.005],'all_checked_segments':len(refs),'all_checked_segment_results':refs,'own_antipad_allowance':'Native lands +0.155mm, including0.150mm clearance and up to0.005mm polygon/zone-resolution envelope; not credited as ground.'}
# Independent Gerber primitive bijections, including net/component attributes.
ext={'F.Cu':'F_Cu.gtl','In1.Cu':'In1_Cu.g1','In2.Cu':'In2_Cu.g2','B.Cu':'B_Cu.gbl'};exports=[]
for layer,suffix in ext.items():
 p=N/'review_gerbers'/('GR86_CCA_RevB-'+suffix);g=GerberFile.open(p);lines=[x for x in g.objects if type(x).__name__=='Line'];flashes=[x for x in g.objects if type(x).__name__=='Flash'];regions=[x for x in g.objects if type(x).__name__=='Region'];assert len(lines)+len(flashes)+len(regions)==len(g.objects)
 def linekey(a,z,w,net):return(*sorted([tuple(round(t,6)for t in a),tuple(round(t,6)for t in z)]),round(w,6),net)
 actual=collections.Counter(linekey((x.x1,-x.y1),(x.x2,-x.y2),x.aperture.diameter,x.attrs.get('.N',('',))[0])for x in lines)
 expected=collections.Counter(linekey(c.get(x,'start'),c.get(x,'end'),c.get(x,'width')[0],names[c.get(x,'net')[0]])for x in c.child(b,'segment')if c.get(x,'layer')==[layer])
 # Flashes match net, XY and independent aperture bounding box. Footprint
 # angle and roundrect details are also covered by native library parity.
 def fk(g,net,ref):
  a,z=g;return tuple(round(v,5)for v in [a[0],-z[1],z[0],-a[1]])+(net,ref)
 af=collections.Counter(fk(x.bounding_box(),x.attrs.get('.N',('',))[0],x.attrs.get('.P',x.attrs.get('.C',('',)))[0])for x in flashes)
 ef=collections.Counter()
 for x in items:
  if x['layer']!=layer or x['type']not in ['pad','via']:continue
  xmin,ymin,xmax,ymax=x['geometry'].bounds;ref=x['id'].split('.')[0]if x['type']=='pad'else''
  ef[tuple(round(v,5)for v in [xmin,ymin,xmax,ymax])+(names.get(x['net'],''),ref)]+=1
 assert all(x.polarity_dark for x in g.objects),'Unexpected clear polarity'
 assert not any(any(v is not None for v in x.arc_centers)for x in regions),'Unhandled Gerber region arc'
 gu=unary_union([sh.make_valid(Polygon([(x,-y)for x,y in q.outline]))for q in regions]);nu=unary_union(filled[layer]);diff=gu.symmetric_difference(nu).area
 exports.append({'layer':layer,'segments':len(lines),'flashes':len(flashes),'regions':len(regions),'missing_segments':list((expected-actual).items()),'unexpected_segments':list((actual-expected).items()),'missing_flashes':list((ef-af).items()),'unexpected_flashes':list((af-ef).items()),'filled_zone_difference_mm2':diff,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
report['Gerbers']=exports
drills=[]
for plated,suffix in [(True,'PTH'),(False,'NPTH')]:
 g=ExcellonFile.open(N/'review_gerbers'/('GR86_CCA_RevB-'+suffix+'.drl'));actual=[(x.x,-x.y,x.aperture.diameter)for x in g.objects];expected=[]
 if plated:expected.extend((*c.get(x,'at')[:2],c.get(x,'drill')[0])for x in c.child(b,'via'))
 for f in c.child(b,'footprint'):
  for p in c.child(f,'pad'):
   if str(p[2])!=('thru_hole'if plated else'np_thru_hole'):continue
   gg=c.pad_shape(f,p);dd=c.get(p,'drill');assert dd and isinstance(dd[0],(int,float))
   expected.append((gg.centroid.x,gg.centroid.y,dd[0]))
 unmatched=list(actual);miss=[]
 for e in expected:
  k=next((i for i,v in enumerate(unmatched)if all(abs(a-z)<=.00050001 for a,z in zip(e,v))),None)
  if k is None:miss.append(e)
  else:unmatched.pop(k)
 drills.append({'plated':plated,'expected':len(expected),'actual':len(actual),'missing':miss,'unexpected':unmatched,'rounding_tolerance_mm':.00050001})
report['drills']=drills
netxml=ET.parse(N/'SCHEMATIC_NETLIST.xml').getroot();nmap={}
for n in netxml.findall('./nets/net'):
 for p in n.findall('node'):nmap[(p.attrib['ref'],p.attrib['pin'])]=n.attrib['name']
two=[];mismatch=[];pcb_pad_map={};critical_component_pads={}
for f in c.child(b,'footprint'):
 ref=c.prop(f)['Reference'];pads=[p for p in c.child(f,'pad')if c.get(p,'net')];nn={str(p[1]):c.get(p,'net')[1]for p in pads}
 for pin,name in nn.items():pcb_pad_map[(ref,pin)]=name
 if ref in {'U201','U202','U301','U401','U402','U403','U404','U501','U502','J201','J401','F1'}:critical_component_pads[ref]=nn
 for pin,name in nn.items():
  if (ref,pin)in nmap and nmap[(ref,pin)]!=name:mismatch.append([ref,pin,name,nmap[(ref,pin)]])
 if len(nn)==2:two.append({'reference':ref,'nodes':nn,'distinct':len(set(nn.values()))==2})
missing_schematic_pads=sorted(set(pcb_pad_map)-set(nmap));missing_PCB_pads=sorted(set(nmap)-set(pcb_pad_map))
report['netlist']={'pcb_net_bearing_pads':len(pcb_pad_map),'schematic_net_nodes':len(nmap),'PCB_pads_absent_from_schematic':missing_schematic_pads,'schematic_nodes_absent_from_PCB':missing_PCB_pads,'critical_component_pads':critical_component_pads,'schematic_components':len(netxml.findall('./components/comp')),'schematic_nets':len(netxml.findall('./nets/net')),'pad_net_mismatches':mismatch,'two_terminal_same_node':[q for q in two if not q['distinct']],'two_terminal_checked':len(two)}
failed=bool(missing_schematic_pads or missing_PCB_pads or len(groups)!=133 or report['native_DRC']or report['native_unconnected']or report['native_parity']or report['native_ERC']or disconnected or padless or mismatch or report['netlist']['two_terminal_same_node']or any(x['missing']or x['unexpected']for x in drills)or any(x['missing_segments']or x['unexpected_segments']or x['missing_flashes']or x['unexpected_flashes']or x['filled_zone_difference_mm2']>1e-6 for x in exports))
report['independent_manufacturing_status']='FINDINGS'if failed else'PASS_INTENDED_COPPER_AND_EXPORTS'
(D/'RESULTS.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items()if k not in ['reference','Gerbers','continuity']},indent=2),flush=True)

if failed:raise SystemExit(1)
