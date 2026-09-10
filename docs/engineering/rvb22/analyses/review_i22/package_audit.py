"""Manufacturer-numbered land overlays, independently transformed through PnP.

Run against the I20 native outputs. Drawing dimensions below are transcribed
from the archived primary sources, not fitted to extracted PCB pad coordinates.
This checks design orientation; it is not independent human sign-off or stencil
approval. Manufacturers' example lands can differ from IPC alternative lands.
"""
from pathlib import Path
import sys, json, math, csv, hashlib
D = Path(__file__).resolve().parent
W = D.parents[1]
sys.path[:0] = [str(W/'runtime/local'), str(W/'runtime/vendor'), str(W/'control')]
import check_combined_copper as c
from shapely.geometry import box
from shapely.affinity import rotate, translate, scale
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as Patch

N = W/'runtime/hosted/run22/extracted/native_I06_hosted'
PCB = N/'candidate_kicad/GR86_CCA_RevB.kicad_pcb'
b, items, unsupported = c.collect(PCB)
assert not unsupported
fps = {c.prop(f)['Reference']:f for f in c.child(b,'footprint')}
pos = {x['Ref']:x for x in csv.DictReader((N/'REVIEW_POSITIONS.csv').open())}
source = {x['name']:x for x in json.loads((D/'sources/MANIFEST.json').read_text())}

def dip(n, pitch, x, length, width):
    pads = {}
    for i in range(n//2):
        y = (i-(n/2-1)/2)*pitch
        pads[str(i+1)] = [-x,y,length,width]
        pads[str(n-i)] = [x,y,length,width]
    return pads

def spec(mpn, source_name, pages, pads, body, note=''):
    return dict(mpn=mpn, source=source_name, pages=pages, pads=pads, body=body, note=note)

specs = {
 'U101':spec('LTC4367HMS8#WTRPBF','ltc4367',[2,19],dip(8,.65,2.1695,.889,.42),[3,3],
   'MS8 drawing rotated 90 degrees into left/right-row convention. 3.45mm inner span and .889mm land length selected from the published range; IPC source lands are longer.'),
 'U121':spec('LM5164DDAR','lm5164',[3,28,29],dip(8,1.27,2.7,1.55,.6),[3.9,4.9]),
 'U151':spec('LM63615DQPWPRQ1','lm63615_q1',[4,53,54],dip(16,.65,2.9,1.5,.45),[4.4,5],
   'PWP0016K. Source signal lands .4mm wide versus .45mm TI example, with the same .65mm pitch and numbering; assembly process acceptance is separate.'),
 'U202':spec('TPS3808G33QDBVRQ1','tps3808_q1',[3,20,21],dip(6,.95,1.3,1.1,.6),[1.6,2.9]),
 'U301':spec('TCAN3403DRBRQ1','tcan3403_q1',[3,43,44],dip(8,.65,1.4,.6,.31),[3,3]),
 'U402':spec('SN74LVC2G125DCUR','sn74lvc2g125',[4,24,25],dip(8,.5,1.55,.85,.3),[2,2.3],
   'DCU0008A, not DCT or YZP. Source land length .8mm versus TI .85mm example; numbering/pitch/centerline agree.'),
 'U403':spec('MIC5504-3.3YM5-TR','mic5504',[1,7,18,19,20],
   {'1':[-1.4,-.95,1.1,.6],'2':[-1.4,0,1.1,.6],'3':[-1.4,.95,1.1,.6],
    '4':[1.4,.95,1.1,.6],'5':[1.4,-.95,1.1,.6]},[1.6,2.9],
   'Five-lead M5/SOT-23. Rev C drawing C04-2091-6BX differs from generic KiCad example lands; pin order and nominal package lead coverage agree.'),
 'U501':spec('TPS26600PWPR','tps2660',[4,5,52,53],dip(16,.65,2.9,1.5,.45),[4.4,5]),
 'U502':spec('OPA2333AIDR','opa2333',[3,46,47],dip(8,1.27,2.7,1.55,.6),[3.9,4.9],
   'D0008A SOIC, not OPA333 five-pin option. Source uses longer IPC lands; outer edges differ by .025mm from TI example.'),
 'U401':spec('PA1616D','pa1616d',[7,8,9],dip(20,1.5,7.8,2,1),[16,16],
   'Current Adafruit-linked manufacturer PDF is V.05. Its numbered top-view drawing is independently checked; the legacy footprint name v06 is not treated as a drawing revision assertion.'),
}
specs['U404'] = dict(specs['U402'])
ep = {
 'U121':('9',[0,0,2.95,4.9],[0,0,2.71,3.4],'GND'),
 'U151':('17',[0,0,3.4,5],[0,0,2.46,2.3],'GND'),
 'U301':('9',[0,0,1.65,2.4],[0,0,1.65,2.4],'GND'),
 'U501':('17',[0,0,3.4,5],[0,0,3.3,3.3],'OIL_EFUSE_RTN'),
}
# Espressif figure11-1, translated from the 18x25.5mm outline to the
# footprint's centered body datum. Side pitch1.27, first pad7.49 below top.
esp = {}
for i in range(14):
    esp[str(i+1)] = [-8.75,-5.26+i*1.27,1.5,.9]
    esp[str(40-i)] = [8.75,-5.26+i*1.27,1.5,.9]
for i in range(12): esp[str(15+i)] = [-6.985+i*1.27,12.5,.9,1.5]
specs['U201'] = spec('ESP32-S3-WROOM-1-N8R2','esp32_s3',[8,9,43,45],esp,[18,25.5],
 'Figure11-1 nine .9mm solder lands on1.4mm pitch form3.7x3.7mm array; source retains them within3.9x3.9mm grounded copper pad41 with48 filled/capped vias. Module antenna is at negative localY.')
ep['U201'] = ('41',[-1.5,2.46,3.7,3.7],[-1.5,2.46,3.7,3.7],'GND')

# Deliberately separate expected circuit functions from KiCad symbol labels.
gps_functions = ['VCC','NRESET','GND','VBACKUP','3D-FIX','NC','NC','GND','TX0','RX0',
                 'EX_ANT','GND','1PPS','RX1/SCL','TX1/SDA','NC','NC','NC','GND','NC']
gps_nets = ['GPS_3V3',None,'GND',None,'GPS_FIX_RAW',None,None,'GND','GPS_TX_RAW','GPS_RX_MODULE',
            'GPS_ANT_RF_BIASED','GND','GPS_PPS_RAW',None,None,None,None,None,'GND',None]

def rectangle(p):
    x,y,w,h = p
    return box(x-w/2,y-h/2,x+w/2,y+h/2)

def through_placement(g, row):
    # Footprint zero-angle convention: bottom source libraries are reflected in
    # localY. Exported placement is an XY-up board datum, same numeric rotation.
    if row['Side']=='bottom': g = scale(g,xfact=1,yfact=-1,origin=(0,0))
    g = rotate(g,-float(row['Rot']),origin=(0,0))
    return translate(g,float(row['PosX']),41.288863-float(row['PosY']))

out = D/'package_overlays'
out.mkdir(exist_ok=True)
results=[];negative=[]
for ref,s in sorted(specs.items()):
    f=fps[ref];p=c.prop(f);assert p['MPN']==s['mpn'],ref
    row=pos.get(ref)
    manual = row is None
    if manual:
        assert ref=='U401'
        row=dict(Ref=ref,PosX=54.5,PosY=32.288863,Rot=0,Side='top')
    actual={str(q[1]):q for q in c.child(f,'pad')if str(q[1])}
    fig,axs=plt.subplots(1,2,figsize=(11.5,5.6))
    records=[]
    for pin,land in s['pads'].items():
        expected=through_placement(rectangle(land),row)
        actual_g=c.pad_shape(f,actual[pin])
        frac=expected.intersection(actual_g).area/expected.area
        # An orientation screen, not a stencil or exact recommended-land test.
        assert frac>.80,(ref,pin,frac)
        # All expected terminal tangential centerlines must align exactly.
        at=c.get(actual[pin],'at');canonical_y=(-at[1]if row['Side']=='bottom'else at[1])
        if ref!='U201' or int(pin)<=14 or int(pin)>=27:
            assert abs(canonical_y-land[1])<1e-6,(ref,pin)
        else: assert abs(at[0]-land[0])<1e-6,(ref,pin)
        mismatch=expected.distance(actual_g)
        records.append(dict(pin=pin,manufacturer_function=gps_functions[int(pin)-1]if ref=='U401'else None,
          actual_net=c.get(actual[pin],'net',[0,''])[1],example_land_overlap_fraction=frac,
          example_land_to_actual_gap_mm=mismatch,PCB_pin_center_mm=list(actual_g.centroid.coords)[0]))
        if ref=='U401':
            n=c.get(actual[pin],'net',[0,''])[1];target=gps_nets[int(pin)-1]
            assert n==target if target else n.startswith('unconnected-'),(pin,n,target)
        for ax in axs:
            for geom,color,alpha,fill in [(actual_g,'#06736d',.5,True),(expected,'#d97d00',1,False)]:
                ax.add_patch(Patch(list(geom.exterior.coords),closed=True,facecolor=color if fill else 'none',edgecolor=color,lw=1.1,alpha=alpha))
            ctr=actual_g.centroid
            ax.text(ctr.x,ctr.y,pin,ha='center',va='center',fontsize=6 if ref=='U201'else 8,color='#101827')
    # In-memory negative control: rotating the placement by180 must break pin1.
    bad={**row,'Rot':float(row['Rot'])+180}
    test=through_placement(rectangle(s['pads']['1']),bad).intersection(c.pad_shape(f,actual['1'])).area
    assert test<1e-7
    negative.append(dict(ref=ref,wrong_rotation_degrees=180,pin1_overlap_mm2=test,rejected=True))
    ep_review=None
    if ref in ep:
        pin,land,mask,net=ep[ref]
        assert c.get(actual[pin],'net')[1]==net
        # Exact openings are read from source, including unnumbered mask pads.
        layer=c.get(f,'layer')[0];ml=layer.replace('Cu','Mask');pl=layer.replace('Cu','Paste')
        from shapely.ops import unary_union
        maskg=unary_union([c.pad_shape(f,q)for q in c.child(f,'pad')if ml in (c.get(q,'layers')or[])])
        copper=c.pad_shape(f,actual[pin]);opening=maskg.intersection(copper)
        paste=unary_union([c.pad_shape(f,q)for q in c.child(f,'pad')if pl in(c.get(q,'layers')or[])]).intersection(copper)
        vias=[v for v in c.child(b,'via')if copper.covers(__import__('shapely').geometry.Point(c.get(v,'at')[:2]))]
        ep_review=dict(pin=pin,net=net,copper_mm2=copper.area,mask_opening_mm2=opening.area,paste_mm2=paste.area,
          paste_fraction_of_opening=paste.area/opening.area,via_centers_in_pad=len(vias),
          manufacturer_example_mask_mm=mask[2:],scope='Exposed-pad net, mask and paste geometry. Thermal performance and assembler process acceptance remain separate.')
        assert opening.area>0 and paste.area>0
        for ax in axs:
            ax.add_patch(Patch(list(copper.exterior.coords),closed=True,facecolor='#06736d',alpha=.25,edgecolor='#06736d'))
            ax.text(copper.centroid.x,copper.centroid.y,'EP '+pin+'\n'+net,ha='center',va='center',fontsize=6)
    body=through_placement(rectangle([0,0,*s['body']]),row)
    for ax in axs:
        ax.add_patch(Patch(list(body.exterior.coords),closed=True,fill=False,edgecolor='#6b7280',ls='--',lw=.8))
    ctr=(float(row['PosX']),41.288863-float(row['PosY']))
    for ax,view in zip(axs,['Through board from top','Viewed from bottom (X mirrored)']):
        ax.autoscale_view();ax.set_aspect('equal');ax.invert_yaxis();ax.grid(alpha=.15)
        ax.set_title(view,fontsize=10);ax.set(xlabel='PCB X (mm)',ylabel='PCB Y (mm)')
    axs[1].invert_xaxis()
    fig.suptitle(f'{ref} | {s["mpn"]}\nNative {row["Side"]}, rotation {float(row["Rot"]):g} degrees | '+('manual placement instruction'if manual else'native placement CSV'),fontsize=12)
    fig.text(.5,.018,'Teal: actual copper. Orange: manufacturer example lands. Pin numbers follow manufacturer top view.\nOrange/teal size differences are recorded alternatives; this is an orientation check, not stencil approval.',ha='center',fontsize=8)
    fig.tight_layout(rect=[0,.15,1,.88]);fig.savefig(out/(ref+'.png'),dpi=145);fig.savefig(out/(ref+'.svg'));plt.close(fig)
    results.append(dict(ref=ref,MPN=s['mpn'],side=row['Side'],rotation_deg=float(row['Rot']),placement_XY_mm=[float(row['PosX']),float(row['PosY'])],
       manual_instruction=manual,drawing=source[s['source']],drawing_pages=s['pages'],notes=s['note'],pins=records,
       minimum_example_land_overlap_fraction=min(x['example_land_overlap_fraction']for x in records),exposed_pad=ep_review,
       status='PASS_NUMBERING_AND_PLACEMENT_ORIENTATION'))
assert set(specs)=={r for r in fps if r.startswith('U')}
report=dict(status='PASS_ALL_12_IC_MODULE_ORIENTATIONS',source_PCB_sha256=hashlib.sha256(PCB.read_bytes()).hexdigest(),
  placement_sha256=hashlib.sha256((N/'REVIEW_POSITIONS.csv').read_bytes()).hexdigest(),references=len(results),
  numbered_signal_lands=sum(len(x['pins'])for x in results),manufacturer_drawing_transcription=specs,
  method='Manufacturer-numbered example lands are independently transformed through native placement CSV, then intersected with actual KiCad copper. Manual PA1616D has an explicit datum instruction. Bottom-view transforms are shown explicitly.',
  negative_controls=negative,results=results,physical_measurement_claimed=False,independent_human_review_claimed=False,
  limitations=['Alternative land sizes are documented and do not imply assembler stencil/process approval.',
  'LIB-02 independent check means a separate drawing/PnP calculation, not another human reviewer. DRW-03 human-review requirement stays open.',
  'Source PDFs are hashed snapshots; top/bottom nomenclature is not inferred from generic STEP bodies.'])
(D/'PACKAGE_ORIENTATION_RESULTS.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items()if k in ['status','references','numbered_signal_lands']}))
