"""Dimensioned mated/maintenance envelopes; declared allocations, not observed fit."""
from pathlib import Path
import sys,json,math,hashlib
D=Path(__file__).resolve().parent;W=D.parents[1];D.mkdir(exist_ok=True)
sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor'),str(W/'control')]
from shapely.geometry import Polygon,Point,LineString,box
from shapely import wkt
from shapely.ops import unary_union
import check_combined_copper as c
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
import cadquery as cq
PCB=W/'iterations/I20_controlled_handoff/candidate_kicad/GR86_CCA_RevB.kicad_pcb'
MECH=W/'iterations/I15_service_mechanics/candidate_mechanics'
b,items,unsupported=c.collect(PCB);assert not unsupported
rows=json.loads((W/'analyses/populated_envelopes/RESULTS.json').read_text())['body_rows']
# Source position parity is required for reuse of the 153 existing envelopes.
old,_,_=c.collect(W/'iterations/I09_native_hole_and_models/candidate_kicad/GR86_CCA_RevB.kicad_pcb')
oldfp={c.prop(f)['Reference']:f for f in c.child(old,'footprint')}
for f in c.child(b,'footprint'):
 ref=c.prop(f)['Reference'];o=oldfp[ref]
 if ref=='C206':assert c.get(f,'at')==[82,-4,0] and c.get(f,'layer')==['F.Cu']
 else:assert c.get(f,'at')==c.get(o,'at') and c.get(f,'layer')==c.get(o,'layer'),ref
u=next(f for f in c.child(b,'footprint') if c.prop(f)['Reference']=='U201')
assert c.get(u,'at')==[81.504766,23,-90]
for r in rows:
 if r['ref']=='C206':
  r.update(side='F.Cu',courtyard_plus0p3_bounds_mm=[78.05,-6.45,85.95,-1.55],height_max_mm=4.3,XY_basis='KEMET maximum7.3x4.3mm body +0.3mm placement allocation at82/-4 F.Cu.')
 if r['ref']=='C201':
  r['courtyard_plus0p3_bounds_mm']=[76.45,7.35,80.55,10.65]
  r['XY_basis']='Primary maximum3.5x2.7mm body plus0.3mm assembly placement/solder envelope; source center78.5/9mm.'
 if r['ref']=='J401':
  r['courtyard_plus0p3_bounds_mm']=[64.075,7.05,67.475,10.55]
  r['XY_basis']='Hirose body bound3.4x3.5mm including assembly placement, centered on native socket axis65.775/8.8mm; contact lands do not occupy a full-height courtyard.'
 if r['ref']=='U201':
  r['prior_courtyard_includes_RF_keepout']=r['courtyard_plus0p3_bounds_mm']
  # Module25.5+/-.2 by18+/-.2, plus.3 placement envelope about centered Fab body.
  r['courtyard_plus0p3_bounds_mm']=[81.504766-12.85-.3,23-9.1-.3,81.504766+12.85+.3,23+9.1+.3]
  r['XY_basis']='Espressif module dimensions, source-centered Fab body and0.3mm placement allocation; antenna keepout remains independently enforced.'
G=json.loads((MECH/'COOLING_GEOMETRY.json').read_text());base=wkt.loads(G['carrier_base_WKT'])
mounts=json.loads((W/'iterations/I18_capacitor_thermal_tab/CURRENT_OUTLINE_GEOMETRY.json').read_text())['mount_centres_mm']
obstacles=[('C05_base',base,-9,-6),('C05_supports',unary_union([wkt.loads(q['WKT']).buffer(.15) for q in json.loads((MECH/'C05_SUPPORT_RELIEF.json').read_text())['support_plans']]),-6,0)]
envelopes=[
 {'name':'F1_mated_housing','xy':box(-18,5.8,1,26.2),'z':[1.6,14.4],'side':'F.Cu','exclude':['F1'],'basis':'Molex43025-1200 width<=19.2,depth<=14.25,height<=8.53mm; conservative19x20.4x12.8 installation prism includes latch/mating-datum uncertainty.'},
 {'name':'F1_free_wire_exit','xy':box(-48,5.8,-.5,26.2),'z':[1.6,14.4],'side':'F.Cu','exclude':['F1'],'basis':'Wire-exit datum allocatedX[-18,-.5]; straight relaxed free length25.4..30mm before first tie/bend/twist. This is space reservation, not a measured housing datum.'},
 {'name':'F1_latch_service','xy':box(-20,3,3,29),'z':[14.4,30],'side':'F.Cu','exclude':['F1'],'basis':'Open hand/latch service access allocated; fixture or enclosure cannot intrude.'},
 {'name':'J201_programming_tool','xy':box(71-5.203,6-3.298,71+5.203,6+3.298),'z':[1.6,41.6],'side':'F.Cu','exclude':[],'basis':'TC2030-IDC-NL drawing9.906x6.096mm plus0.25mm positioning allowance each side; tool alignment-hole fit must maintain that bound;40mm vertical tool space. No optional retention clip credited.'},
 {'name':'J401_mated_plug','xy':box(64.075,5.3,67.475,10.6),'z':[-3.25,0],'side':'B.Cu','exclude':['J401'],'basis':'Socket mating axisX65.775/Y8.8; selected negative-Y exit. Maximum installed plug/cable-nose3.4x5.3mm and3.25mm depth are supplier dimensional allocations.'}]
checks=[]
def check(name,xy,z,side,exclude):
 for row in rows:
  if row['side']!=side or row['ref'] in exclude:continue
  a,bz=(1.6,1.6+row['height_max_mm']+1.71) if side=='F.Cu' else (-row['height_max_mm']-1.71,0)
  overlap=xy.intersection(box(*row['courtyard_plus0p3_bounds_mm'])).area*max(0,min(z[1],bz)-max(z[0],a))
  checks.append({'a':name,'b':row['ref'],'overlap_mm3':overlap})
 if side=='B.Cu':
  for n,q,a,bz in obstacles:checks.append({'a':name,'b':n,'overlap_mm3':xy.intersection(q).area*max(0,min(z[1],bz)-max(z[0],a))})
for e in envelopes:check(e['name'],e['xy'],e['z'],e['side'],e['exclude'])
# Alignment pins, through source NPTH positions, allocated4.25mm below board.
j=next(f for f in c.child(b,'footprint') if c.prop(f)['Reference']=='J201')
for i,p in enumerate(c.child(j,'pad')):
 if str(p[2])=='np_thru_hole':check('J201_alignment_pin_'+str(i),c.pad_shape(j,p).buffer(.1),[-4.25,0],'B.Cu',[])
# Full150mm planar pigtail: 15mm-radius S bend and three tangent15mm-radius turns and straight lengths.
# World Z=-2.1mm. Tube+position/bow radius1.5mm includes2.0mm cable OD acceptance envelope (1.8mm nominal plus .2mm allocation).
origin=(65.775,8.8);path=[origin,(65.775,5.3)]
def arc(center,R,a,b):
 for i in range(1,121):
  t=math.radians(a+(b-a)*i/120);path.append((center[0]+R*math.cos(t),center[1]+R*math.sin(t)))
angle=32;theta=math.radians(angle);dx=30*(1-math.cos(theta));dy=30*math.sin(theta)
arc((80.775,5.3),15,180,180+angle)
arc((65.775+dx-15,5.3-dy),15,angle,0)
path.append((65.775+dx,-14.2))
arc((50.775+dx,-14.2),15,0,-90);path.append((20.775+dx,-29.2))
arc((20.775+dx,-44.2),15,90,180);path.append((5.775+dx,-57.2))
arc((20.775+dx,-57.2),15,180,270)
first_length=3.5+30*theta+(19.5-dy)
tail=150-(first_length+30+13+3*math.pi*15/2);path.append((20.775+dx+tail,-72.2))
cable=LineString(path);tube=cable.buffer(1.5,quad_segs=64)
check('150mm_RG178_pigtail',tube,[-3.6,-.6],'B.Cu',['J401'])
# Full cold-metal and foil geometry: cable reserves a conservative fixed Z slab.
foil=Polygon(G['direct_T02_foil_xz_mm']);cxz=box(tube.bounds[0],-3.6,tube.bounds[2],-.6)
cy=box(-100,12.8-.8,200,27.9+.8)
central_possible=tube.intersection(cy).area>0 and cxz.intersection(foil.buffer(.8,join_style=2)).area>0
checks.append({'a':'150mm_RG178_pigtail','b':'T03_with0p8mm_allowance','overlap_mm3':1 if central_possible else 0})
for n,rect in [('upper_wing',box(15-.8,-11.3-.8,53+.8,-.5+.8)),('lower_wing',box(15-.8,42.5-.8,53+.8,53.3+.8))]:
 checks.append({'a':'150mm_RG178_pigtail','b':n,'overlap_mm3':tube.intersection(rect).area*2.8})
# Panel connector and torque-tool reservation wholly outside board envelope.
end=path[-1];sma=box(end[0]-4-3.0,end[1]-7,end[0]+30+3.0,end[1]+7)
check('SMA_bulkhead_and_wrench',sma,[-9.1,4.9],'B.Cu',[])
hits=[x for x in checks if x['overlap_mm3']>1e-7]
RFsetback=88.254766-max(tube.bounds[2],sma.bounds[2],67.475)
assert RFsetback>=15,RFsetback
r={'status':'PASS_DECLARED_ENVELOPES' if not hits else 'REDLINES_REMAIN','source_pcb_sha256':hashlib.sha256(PCB.read_bytes()).hexdigest(),'checks':len(checks),'interferences':hits,'refined_U201_XY':next(x for x in rows if x['ref']=='U201'),'envelopes':[{**{k:v for k,v in e.items() if k!='xy'},'bounds_xy_mm':list(e['xy'].bounds)} for e in envelopes],
 'pigtail':{'centerline_mm':path,'length_analytic_mm':150,'polygon_length_mm':cable.length,'bend_radius_mm':15,'cable_OD_allocation_mm':2.0,'tube_position_and_bow_radius_mm':1.5,'Z_center_mm':-2.1,'length_tolerance_mm':3.0,'tail_adjustment_mm':[-3.0,3.0],'bulkhead_and_tool_bounds_xy_mm':list(sma.bounds),'RF_metal_setback_mm':RFsetback},
 'selected_accessory':'User-confirmed Adafruit851, drawing SMASFN8-178B-150IX RevA:150+/-3mm RG178, OD1.80mm. No alternate cable selected.',
 'original_accessory_finding':'Adafruit851 confirmed identity: linkedPDF−10..60C and RP-SMA conflict with product-pageSMA; cannot accept at65C design air without corrected supplier evidence or substitution.',
 'conditions':['All other153 component XY bounds remain source-courtyard+0.3mm allocations with sourced maximum heights; solder/warp/deflection allowances retained.',
 'Mated U.FL plug exits negativeY; manufacturer plug dimensions must fit3.4x5.3x3.25mm prism and cable OD<=2.0mm.',
 'Free Micro-Fit wires remain relaxed25.4..30mm before a bend/tie/twist; downstream harness bend radius>=30mm and external support required. The exit prism includes datum uncertainty.',
 'Pigtail is attached before carrier installation. Unplug/replace U.FL with board removed, using axial connector tool; do not pull the cable. Maintain mating-cycle limits.',
 'Pigtail retained on independent insulating clips along the reserved loop; exact grip/friction and clip material remain process conditions. No force from150mm cable or5m puck cable is assigned to the U.FL joint.',
 'Panel bulkhead takes SMA mating torque and the5m GPS cable load. Do not reuse the alternate cable panel-thickness rating. Use an insulating bulkhead plate and verify thread engagement against the supplied851 hardware; thickness remains an assembly acceptance condition.',
 'Program withTC2030-IDC-NL from top, manually held; optional bottom retention clip is not included. No enclosure above the40mm service prism.',
 'This substantial loop reservation reachesY−79.2mm including wrench; actual dashboard fit remains an installation condition, not an observed match.',
 'The60C Adafruit cable and any unconfirmedRP-SMA interface remain unacceptable in the65C modeled environment; no thermal acceptance or alternate-part substitution is implied.'],
 'sources':['https://www.tag-connect.com/wp-content/uploads/bsk-pdf-manager/2019/12/TC2030-IDC-NL-Datasheet-Rev-B.pdf','https://www.espressif.com/sites/default/files/documentation/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf','https://www.adafruit.com/product/851','https://www.adafruit.com/product/960','https://cdn-shop.adafruit.com/product-files/851/C934-001_datasheet.pdf'],
 'full_body_height_allowance_mm':{'solder':.25,'capture_freedom':.51,'warp':.75,'local_deflection':.20,'sum':1.71},
 'scope':'Conservative, inspectable allocated mated/service geometry and a full-length cable route; no claim of manufactured part fit, fatigue, supplier acceptance or installed vehicle fit.'}
r['body_rows']=rows
(D/'RESULTS.json').write_text(json.dumps(r,indent=2)+'\n')
parts=[]
for e in envelopes:
 x0,y0,x1,y1=e['xy'].bounds;z0,z1=e['z'];v=cq.Workplane('XY',origin=(x0,y0,z0)).box(x1-x0,y1-y0,z1-z0,centered=(False,False,False));parts.append(v.val())
cq.exporters.export(cq.Compound.makeCompound(parts),str(D/'MATED_SERVICE_ENVELOPES.step'))
fig,ax=plt.subplots(figsize=(9,11));board=Polygon(json.loads((W/'iterations/I18_capacitor_thermal_tab/CURRENT_OUTLINE_GEOMETRY.json').read_text())['outline_mm']);x,y=board.exterior.xy;ax.fill(x,y,color='#dde7de',label='PCB outline')
for e in envelopes:
 x,y=e['xy'].exterior.xy;ax.plot(x,y,lw=1,label=e['name'])
xx,yy=cable.xy;ax.plot(xx,yy,color='#7b4092',lw=2,label='150mm pigtail, R15mm');x,y=sma.exterior.xy;ax.plot(x,y,ls='--',label='SMA and wrench space')
ax.set_aspect('equal');ax.invert_yaxis();ax.set(xlabel='PCB X (mm)',ylabel='PCB Y (mm)',title='I22 Adafruit851 mated/service space - full1.71mm stack');ax.grid(alpha=.25);ax.legend(loc='upper left',fontsize=8);fig.tight_layout();fig.savefig(D/'MATED_SERVICE_ENVELOPES.png',dpi=160)
print(json.dumps({'status':r['status'],'checks':len(checks),'interferences':hits,'RF_metal_setback_mm':RFsetback},indent=2))
assert not hits,hits
