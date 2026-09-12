#!/usr/bin/env python3
"""Reconcile every historical finding and widen applicability without inventing SI closure."""
from __future__ import annotations
import argparse, collections, csv, hashlib, json, math, sys
from pathlib import Path
import shapely
from shapely.geometry import Point, Polygon, LineString
from shapely.ops import unary_union
D = Path(__file__).resolve().parent
W = D.parents[1]
sys.path.insert(0, str(W / 'analyses/convergence_01/support'))
import check_combined_copper as copper
SOURCE_SHA = '5b373f6033fdbd18f126f8ca054b619abada2568778c5a8fc303c4e756fb06c8'
FILLED_SHA = '11533ea91c3bc4c61dd7066e0001914a72bc90b27afb38d321c43b8feb90e3b1'
CRITICAL = {
 'ADC_NODE','CANH','CANL','CAN_TX_MCU','CAN_RX_MCU','CAN_RXD','CAN_TXD',
 'GPS_ANT_RF_BIASED','GPS_EXT_ANT','GPS_TX_RAW','GPS_TX_MCU','GPS_TX_BUFFER',
 'GPS_TX_MODULE_BUFFERED','GPS_RX_MODULE','GPS_RX_BUFFER','GPS_PPS_RAW',
 'GPS_PPS_BUFFER','GPS_PPS_MCU','OIL_EXCITATION_ADC','UART0_RX','UART0_TX',
 'ESP_EN','ESP_GPIO0','OIL_FILTERED','OIL_EXC_FILTERED','OIL_SIG','OIL_FB',
 'OIL_EXC_FB','OIL_AMP_OUT','OIL_EXC_AMP_OUT','GPS_FIX_RAW','GPS_SEARCH_BUFFER'}
SOURCE_CLASSES = {
 'CAN_TX_MCU':'GPIO5 launch; R301 DNP isolates this stub from TXD. No intended release transitions.',
 'CAN_RX_MCU':'U301 RXD through R302 1kohm into GPIO4; receive path.',
 'CAN_RXD':'U301 RXD BEFORE R302 1kohm; do not confuse this with the filtered MCU-side net.',
 'CAN_TXD':'U301 TXD held recessive by fitted R303 10kohm; R301 DNP; no intentional TX.',
 'GPS_TX_RAW':'PA1616D TX0 into U402 LVC input; raw UART source edge not specified.',
 'GPS_PPS_RAW':'PA1616D PPS into U402 LVC input; PPS repetition frequency is not edge rate.',
 'GPS_TX_BUFFER':'U402 LVC output BEFORE R402 220ohm.',
 'GPS_PPS_BUFFER':'U402 LVC output BEFORE R403 220ohm.',
 'GPS_TX_MODULE_BUFFERED':'U402 via R402 220ohm to GPIO18; active GPS receive path.',
 'GPS_TX_MCU':'GPIO17 output into U404 input; active GPS command path.',
 'GPS_RX_BUFFER':'U404 LVC output BEFORE R404 220ohm.',
 'GPS_RX_MODULE':'U404 via R404 220ohm to PA1616D RX0.',
 'GPS_PPS_MCU':'U402 via R403 220ohm to GPIO16.',
 'GPS_SEARCH_BUFFER':'U404 LVC output drives R405 1kohm and D401; indicator, not timing input.',
 'ESP_EN':'Supervisor/reset/pullup network; reset immunity must be assessed separately from UART.',
 'ESP_GPIO0':'Boot strap/programming fixture network; not a high-rate data interface.'}

def require(value, message):
    if not value:
        raise ValueError(message)

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def load_native(path):
    require(digest(path) == FILLED_SHA, 'Native filled PCB effectivity changed')
    require(digest(W/'candidate/cad/GR86_CCA_RevB.kicad_pcb') == SOURCE_SHA, 'Source PCB effectivity changed')
    tree, items, unsupported = copper.collect(path)
    require(not unsupported, 'Unsupported copper geometry')
    names = {n[1]:n[2] for n in copper.child(tree,'net')}
    gnd = next(n for n,name in names.items() if name == 'GND')
    ground = {layer:[i['geometry'] for i in items if i['net']==gnd and i['layer']==layer] for layer in copper.L}
    for zone in copper.child(tree,'zone'):
        if copper.child(zone,'keepout') or copper.get(zone,'net') != [gnd]:
            continue
        for fill in copper.child(zone,'filled_polygon'):
            pts = [p[1:] for p in copper.child(copper.child(fill,'pts')[0],'xy')]
            ground[copper.get(fill,'layer')[0]].append(shapely.make_valid(Polygon(pts)))
    # The 2nm geometric roundoff allowance is not an etch/minimum-width allowance.
    return tree,items,names,{layer:unary_union(parts).buffer(.000002) for layer,parts in ground.items()}

def nearest_vias(tree,names,line):
    vias = [v for v in copper.child(tree,'via') if names[copper.get(v,'net')[0]]=='GND']
    vias.sort(key=lambda v:line.distance(Point(copper.get(v,'at')[:2])))
    return [{'uuid':copper.get(v,'uuid')[0],'xy_mm':copper.get(v,'at')[:2],
             'distance_to_signal_segment_mm':line.distance(Point(copper.get(v,'at')[:2])),
             'layers':copper.get(v,'layers'),'drill_mm':copper.get(v,'drill')[0],
             'pad_diameter_mm':copper.get(v,'size')[0]} for v in vias[:3]]

def review(native, out):
    tree,items,names,ground = load_native(native)
    nums = {name:num for num,name in names.items()}
    current = json.loads((W/'analyses/manufacturing_i24/RESULTS.json').read_text())
    require(current['source_PCB_sha256']==SOURCE_SHA and current['filled_PCB_sha256']==FILLED_SHA,'Prior native reference provenance mismatch')
    previous = {r['uuid']:r for r in current['reference']['all_checked_segment_results']}
    segments = {copper.get(s,'uuid')[0]:s for s in copper.child(tree,'segment')}
    provenance = json.loads((D/'HISTORICAL_REFERENCE_PROVENANCE.json').read_text())
    with (D/'HISTORICAL_54.csv').open(newline='') as f:
        historical = list(csv.DictReader(f))
    require(len(historical)==54 and len({r['uuid'] for r in historical})==54,'Lost historical finding identity')
    cache = {}
    def own(net,layer):
        key=net,layer
        if key not in cache:
            cache[key]=unary_union([i['geometry'].buffer(.155) for i in items if i['net']==net and i['layer']==layer and i['type'] in ['via','pad']])
        return cache[key]
    rows=[]
    for old in historical:
        cur=previous[old['uuid']];seg=segments[old['uuid']]
        start=[float(old['start_x']),float(old['start_y'])];end=[float(old['end_x']),float(old['end_y'])]
        require(cur['net']==old['net'] and cur['layer']==old['layer'] and cur['start']==start and cur['end']==end,'Historical geometry changed: '+old['uuid'])
        line=LineString([start,end]);net=nums[cur['net']];ref=cur['reference']
        gap=line.difference(ground[ref]).difference(own(net,ref))
        require(abs(gap.length-cur['adjacent_gap_after_own_antipads_mm'])<1e-5,'Independent native gap disagreement')
        resolved=gap.length<1e-7
        checked_line=line.difference(own(net,ref))
        diameter=2*checked_line.distance(ground[ref].boundary) if resolved and not checked_line.is_empty else None
        rows.append({'finding_id':old['finding_id'],'uuid':old['uuid'],'net':cur['net'],'layer':cur['layer'],
          'start':start,'end':end,'reference':ref,'historical_adjacent_gap_mm':float(old['historical_adjacent_gap_mm']),
          'current_adjacent_gap_mm':gap.length,'current_gap_WKT':gap.wkt,'current_both_plane_gap_mm':cur['both_ground_planes_gap_mm'],
          'source_scope':SOURCE_CLASSES[cur['net']],
          'edge_rate_class':'0.5/1/2/5ns source-edge sensitivity is pending, not a claimed silicon bound. Interface bit rate is not used as rise time.',
          'same_layer_signal_at_finding':True,'nearest_actual_ground_vias':nearest_vias(tree,names,line),
          'nominal_centerline_clearance_diameter_mm':diameter,
          'current_disposition':'REMOVED_BY_PRIOR_COORDINATED_COPPER_REVISION' if resolved else 'ISOLATED_TX_STUB_NO_RECEIVE_PATH',
          'historical_gap_excess_return_detour_mm':0 if resolved else None,
          'historical_gap_excess_loop_inductance_nH':0 if resolved else None,
          'zero_excess_interpretation':'Only the incremental detour from this removed historical void is zero; trace/via loop inductance is NOT zero.' if resolved else 'No numerical loop bound is claimed for the persistent void.',
          'reference_transfer_proven':False,'physical_test_claimed':False})
    findings=[];counts=collections.Counter();totals=collections.defaultdict(float)
    for uuid,seg in segments.items():
        net=copper.get(seg,'net')[0];name=names[net];layer=copper.get(seg,'layer')[0]
        if name not in CRITICAL or layer not in ['F.Cu','B.Cu']:
            continue
        ref='In1.Cu' if layer=='F.Cu' else 'In2.Cu'
        line=LineString([copper.get(seg,'start'),copper.get(seg,'end')])
        gap=line.difference(ground[ref]).difference(own(net,ref))
        counts[name]+=1;totals[name]+=line.length
        if gap.length>.005:
            findings.append({'uuid':uuid,'net':name,'layer':layer,'reference':ref,'start':list(line.coords[0]),'end':list(line.coords[-1]),
             'adjacent_gap_mm':gap.length,'gap_WKT':gap.wkt,'nearest_actual_ground_vias':nearest_vias(tree,names,line),
             'source_scope':SOURCE_CLASSES.get(name,'Analog, differential or programming interface; see existing signal-specific contract.'),
             'disposition':'APPLICABILITY_AND_TRANSFER_BOUND_PENDING_NOT_A_PROVEN_PCB_DEFECT'})
    transfers=[]
    for via in copper.child(tree,'via'):
        name=names[copper.get(via,'net')[0]]
        if name in CRITICAL:
            p=Point(copper.get(via,'at')[:2])
            transfers.append({'signal_via_uuid':copper.get(via,'uuid')[0],'net':name,'xy_mm':copper.get(via,'at')[:2],
              'signal_via_layers':copper.get(via,'layers'),'signal_via_drill_mm':copper.get(via,'drill')[0],
              'nearby_ground_vias':nearest_vias(tree,names,p),'source_scope':SOURCE_CLASSES.get(name,'Analog/differential/programming interface'),
              'current_return_transfer_status':'QUANTITATIVE_TRANSFER_VALIDATION_PENDING'})
    historical_result={'source_report':provenance,'source_PCB_sha256':SOURCE_SHA,'filled_PCB_sha256':FILLED_SHA,'count':54,
      'counts':dict(collections.Counter(r['current_disposition'] for r in rows)),'rows':rows,'gnd02_desktop_complete':False,'physical_test_claimed':False}
    require(historical_result['counts']=={'REMOVED_BY_PRIOR_COORDINATED_COPPER_REVISION':48,'ISOLATED_TX_STUB_NO_RECEIVE_PATH':6},'Changed historical reconciliation')
    extended={'source_PCB_sha256':SOURCE_SHA,'filled_PCB_sha256':FILLED_SHA,'status':'EXPANDED_REFERENCE_SCOPE_NOT_FULL_RETURN_ACCEPTANCE',
      'critical_net_set':sorted(CRITICAL),'outer_segment_count':sum(counts.values()),'segments_by_net':dict(counts),'outer_length_mm_by_net':dict(totals),
      'finding_counts':dict(collections.Counter(r['net'] for r in findings)),'findings':findings,'signal_via_count':len(transfers),'signal_vias':transfers,
      'limitations':['Reference polygons and signal centerlines are nominal; no universal etched neck-width or RF-current-distribution proof.',
      'Own signal antipads are explicitly excluded, not credited as ground copper.',
      'CAN_RXD before R302, buffered GPS receive into GPIO18, EN, GPIO0 and indicator/control applicability now appear explicitly.',
      'A nearby through-ground-via is not accepted without a real copper path to both return planes.',
      'Existing RF matching, CAN receive-branch and private DC return models retain their separate scopes.'],
      'gnd02_desktop_complete':False,'physical_test_claimed':False}
    out.mkdir(parents=True,exist_ok=True)
    for name,result in [('HISTORICAL_54_RETURN_FINDINGS',historical_result),('EXPANDED_RETURN_SCOPE',extended)]:
        (out/(name+'.json')).write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({'historical':historical_result['counts'],'expanded_findings':extended['finding_counts'],'signal_vias':len(transfers),'gnd02_desktop_complete':False}))
    return historical_result,extended

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('--native',type=Path,required=True);ap.add_argument('--out',type=Path,default=D)
    args=ap.parse_args();review(args.native,args.out)
