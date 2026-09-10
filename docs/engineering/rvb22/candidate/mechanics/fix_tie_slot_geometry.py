"""MECH22-31: source-derived obround/strap tolerance correction, no PCB mutation."""
from pathlib import Path
import json,hashlib,math,shutil
D=Path(__file__).resolve().parent;H=D/'history_before_slot_shape_fix';H.mkdir(exist_ok=True)
names=['GR86_RVB_CARRIER_C03.scad','GR86_RVB_CARRIER_C03_THERMAL.scad','HARNESS_CORRIDORS_C03.scad','build_thermal_contact.py','draw_c03_review.py','thermal_contact_t01_model.py','THERMAL_CONTACT_T01_MODEL.json','finalize_dispositions.py','REDLINE_DISPOSITIONS.json','MECHANICAL_INSTALLATION_RVB22.md','THERMAL_CONTACT_T01_REVIEW.md','C03_CARRIER_CHECKS.json','STRUCTURAL_SERVICE_MODEL.json']
for name in names:
 p=D/name
 if p.exists() and not(H/name).exists():shutil.copy2(p,H/name)
if not(D/'SLOT_SHAPE_REDLINE_BEFORE_CORRECTION.json').exists():
 (D/'SLOT_SHAPE_REDLINE_BEFORE_CORRECTION.json').write_text(json.dumps({'id':'MECH22-31','status':'RECORDED_BEFORE_SOURCE_CORRECTION','source_hashes':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in H.iterdir() if p.is_file()},'finding':'Bounding-box fit overclaimed a maximum2.7x1.3mm strap inside a3x1.5mm obround. Atminimum2.9x1.4mm slot, allfourstrapcorners lie0.1845903mm outside the roundedend.','reproducer':'For L=2.9,H=1.4,w=2.7,t=1.3: arc centre±(L-H)/2=±.75,r=.7; strapcorner(1.35,.65) hasdistance sqrt(.6²+.65²)=.8845903>r.','correction':'3.9x1.6mm nominal obround,finished3.8..4.0x1.5..1.7mm; local8mmspines retain2mm minimumweb. Exactfourcorner/tolerancesweep follows.'},indent=2)+'\n')
s=(H/'GR86_RVB_CARRIER_C03.scad').read_text()
s=s.replace('module base(){difference(){union(){','module base(){difference(){union(){\n //MECH22-31: local left-spine reinforcement; global fitbox unchanged.\n translate([-8,28,base_bottom])cube([8,12,3]);')
old='translate([0,2,base_bottom-1])linear_extrude(5)offset(r=2)translate([2,2])square([55.5,33]);'
new='difference(){\n '+old+'\n //Preserve2mm sidewebs beside wider right slots; insert staysbelowT01Y28.8.\n translate([58.5,30,base_bottom-1])cube([8,12,5]);\n }'
assert old in s;s=s.replace(old,new)
s=s.replace('[-3.5,31],[-3.5,37]','[-4,31],[-4,37]').replace('[63,30],[63,36]','[62.5,33],[62.5,39]').replace('[63,33],[63,39]','[62.5,33],[62.5,39]')
s=s.replace('translate([-.75,0])circle(r=.75);translate([.75,0])circle(r=.75);','translate([-1.15,0])circle(r=.8);translate([1.15,0])circle(r=.8);')
# Obsoleteauxanchors16/22 are removed fromC03too; they conflictwithT01andarenotcurrentassemblyfeatures.
s=s.replace('[63,16],[63,22],','')
(D/'GR86_RVB_CARRIER_C03.scad').write_text(s)
b=(H/'build_thermal_contact.py').read_text().replace("[[-3.5,31],[-3.5,37],[63,33],[63,39]]","[[-4,31],[-4,37],[62.5,33],[62.5,39]]")
(D/'build_thermal_contact.py').write_text(b)
h=(H/'HARNESS_CORRIDORS_C03.scad').read_text().replace('translate([-7,25,-16.3])','translate([-7.5,25,-16.3])').replace('translate([-12,y-2.6,-12.9])','translate([-12.5,y-2.6,-12.9])').replace('translate([61.5,y-.75,-11.8])cube([3,1.5,2.8])','translate([60.55,y-.8,-11.8])cube([3.9,1.6,2.8])').replace('translate([60.4,y-2.6,-12.9])','translate([59.9,y-2.6,-12.9])')
(D/'HARNESS_CORRIDORS_C03.scad').write_text(h)
# Bind currentthermalshadow toexactnewC03geometry, without modifying historicalC02thermalmodels.
t=(H/'thermal_contact_t01_model.py').read_text().replace('from thermal_network import *','from thermal_network import *\nfrom carrier_slot_geometry import carrier_plan\ncarrier=carrier_plan() #MECH22-31: actualcurrentC03shadow; historicalC02models unchanged')
(D/'thermal_contact_t01_model.py').write_text(t)
p=(H/'draw_c03_review.py').read_text().replace('from matplotlib.patches import Polygon as PatchPolygon,Rectangle,Circle','from matplotlib.patches import Polygon as PatchPolygon,Rectangle,Circle\nfrom carrier_slot_geometry import carrier_plan,anchor_centres,obround')
pos=p.index("ax.add_patch(PatchPolygon(G['outline_mm']")
p=p[:pos]+"#MECH22-31 localspineinsert and actualobroundslots.\nax.add_patch(Rectangle((-8,28),8,12,facecolor='#ded6bf',edgecolor='#8a7c55'))\nax.add_patch(Rectangle((58.5,30),8,12,facecolor='#ded6bf',edgecolor='#8a7c55'))\n"+p[pos:]
start=p.index('for x,y in [(-3.5,31)');end=p.index('\n',start)
p=p[:start]+"for x,y in anchor_centres:\n ax.add_patch(PatchPolygon(list(obround(x,y,3.9,1.6).exterior.coords),facecolor='#2757a5'))"+p[end:]
(D/'draw_c03_review.py').write_text(p)
f=(H/'finalize_dispositions.py').read_text().replace("pair(-3.5,31/37)","pair(-4,31/37)").replace('Widen leftspine7mm; primaryslotsX−3.5;1.95mmminimumweb','Local8mmspines; primaryslotsX−4/rightX62.5;2.0mmminimumweb afteractualslotshape correction')
f=f.replace('rows=[]',"items.append((31,'Rectangular strap corners do not fit nominal obround slot','3.9x1.6mm nominal obrounds,fulltolerance/cornerproof andlocalwebreinforcement','CORRECTED_SOURCE_MODEL',['SLOT_SHAPE_CORRECTION_CHECKS.json','GR86_RVB_CARRIER_C03_THERMAL.scad'],['RDA-07','MECH-03'],'Actualjacketfriction,hotretainedtension andmaterialallowables remainconditional.'))\nrows=[]")
(D/'finalize_dispositions.py').write_text(f)
print('Source correction generated; rungeometrychecks beforefinalization')
