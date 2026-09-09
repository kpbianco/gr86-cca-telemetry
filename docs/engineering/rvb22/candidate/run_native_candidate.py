#!/usr/bin/env python3
"""Collect native evidence from explicit Rev B candidates; never edits inputs.

Run on the already authorized local engineering machine. This wrapper does not
install runtimes and does not flash or open device ports. Provision the external
tools and dependencies before use; their own initialization remains tool-owned.
Its exports are engineering review files, not an approved manufacturing release.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

FQBN = ('esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default,MSCOnBoot=default,'
        'DFUOnBoot=default,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=8M,'
        'PartitionScheme=default_8MB,DebugLevel=none,PSRAM=enabled,LoopCore=1,'
        'EventsCore=1,EraseFlash=none,UploadSpeed=115200')
LAYERS = 'F.Cu,In1.Cu,In2.Cu,B.Cu,F.Mask,B.Mask,F.Paste,B.Paste,F.SilkS,B.SilkS,Edge.Cuts'
REFILL = r'''
import json,sys
from pathlib import Path
import pcbnew as p
project, board = map(Path, sys.argv[1:3])
assert '9.0.9' in p.GetBuildVersion(), p.GetBuildVersion()
manager = p.GetSettingsManager()
manager.LoadProject(str(project))
# Compare pad identities/names before accepting native load or refill.
# A balanced lexer is sufficient for these quoted scalar fields; no CAD edits occur.
import re
def children(text):
 depth=0; quoted=False; escaped=False; start=None
 for i,ch in enumerate(text):
  if quoted:
   if escaped: escaped=False
   elif ch=='\\': escaped=True
   elif ch=='"': quoted=False
  elif ch=='"': quoted=True
  elif ch=='(':
   if depth==1:start=i
   depth+=1
  elif ch==')':
   depth-=1
   if depth==1 and start is not None:yield text[start:i+1]
 assert depth==0 and not quoted
expected={}
for fp in children(board.read_text()):
 if not fp.startswith('(footprint '):continue
 fields=list(children(fp))
 ref=json.loads(next(re.match(r'\(property\s+"Reference"\s+("(?:[^"\\]|\\.)*")',x).group(1) for x in fields if x.startswith('(property "Reference"')))
 for pad in fields:
  if not pad.startswith('(pad '):continue
  pf=list(children(pad)); ident=next(json.loads(re.match(r'\(uuid\s+("[^"]+")',x).group(1)) for x in pf if x.startswith('(uuid '))
  net=next((json.loads(re.match(r'\(net\s+\d+\s+("(?:[^"\\]|\\.)*")',x).group(1)) for x in pf if x.startswith('(net ')), '')
  expected[(ref,ident)]=net
def check_pad_nets(board_obj,stage):
 actual={(f.GetReference(),pad.m_Uuid.AsString()):pad.GetNetname() for f in board_obj.GetFootprints() for pad in f.Pads()}
 differences=[{'reference':key[0],'pad_uuid':key[1],'source':expected.get(key),'native':actual.get(key)} for key in expected.keys()|actual.keys() if expected.get(key)!=actual.get(key)]
 assert not differences, stage+' pad-net identity changed: '+json.dumps(differences)

b = p.LoadBoard(str(board))
assert b is not None, "Native PCB parser rejected input; inspect parser-isolation diagnostic"
check_pad_nets(b,"native load/reload")
b.SetProject(manager.GetProject(str(project)))
b.SynchronizeNetsAndNetClasses(False)
p.ZONE_FILLER(b).Fill(b.Zones())
p.SaveBoard(str(board), b)
b = p.LoadBoard(str(board))
assert b is not None, "Native PCB parser rejected input; inspect parser-isolation diagnostic"
check_pad_nets(b,"native load/reload")
zones = [z for z in b.Zones() if not z.GetIsRuleArea()]
details = []
for z in zones:
 for layer in [p.F_Cu,p.In1_Cu,p.In2_Cu,p.B_Cu]:
  if not z.IsOnLayer(layer): continue
  poly = z.GetFilledPolysList(layer)
  area = 0.0
  for i in range(poly.OutlineCount()):
   outline = poly.Outline(i)
   pts = [(p.ToMM(outline.CPoint(j).x),p.ToMM(outline.CPoint(j).y)) for j in range(outline.PointCount())]
   if len(pts) >= 3:
    area += abs(sum(a[0]*c[1]-c[0]*a[1] for a,c in zip(pts,pts[1:]+pts[:1])))/2
  details.append({'uuid':z.m_Uuid.AsString(),'net':z.GetNetname(),
                  'layer':b.GetLayerName(layer),'outline_count':poly.OutlineCount(),
                  'outer_area_mm2':area})
print(json.dumps({'version':p.GetBuildVersion(),'zones':len(zones),
 'copper_zones':len({d['uuid'] for d in details}), 'zone_layer_details':details,
 'footprints':len(list(b.GetFootprints())), 'tracks_and_vias':len(list(b.GetTracks()))}))
'''

# These check structure and completeness, not electrical correctness. The native
# reports, raw geometry and project-specific release validators still own that.
GERBER_FUNCTIONS = {
    'F.Cu':'Copper,L1,Top', 'In1.Cu':'Copper,L2,Inr',
    'In2.Cu':'Copper,L3,Inr', 'B.Cu':'Copper,L4,Bot',
    'F.Mask':'Soldermask,Top', 'B.Mask':'Soldermask,Bot',
    'F.Paste':'Paste,Top', 'B.Paste':'Paste,Bot',
    'F.SilkS':'Legend,Top', 'B.SilkS':'Legend,Bot', 'Edge.Cuts':'Profile,NP'}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def validate_refill(log: str, pcb: Path, original: Path) -> dict:
    reports = []
    for line in log.splitlines():
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(d, dict) and 'copper_zones' in d:
            reports.append(d)
    require(len(reports) == 1, 'One structured refill report required')
    d = reports[0]
    require(isinstance(d.get('copper_zones'), int) and d['copper_zones'] > 0,
            'Refill must contain applicable copper zones')
    details = d.get('zone_layer_details')
    require(isinstance(details, list) and details, 'Missing per-zone/per-layer fill evidence')
    require(len({x['uuid'] for x in details}) == d['copper_zones'], 'Zone count mismatch')
    require(all(x.get('outline_count', 0) > 0 and x.get('outer_area_mm2', 0) > 0 for x in details),
            'Applicable copper zone has empty fill')
    source = original.read_text()
    saved = pcb.read_text()
    require(saved.lstrip().startswith('(kicad_pcb'), 'Saved artifact is not a KiCad board')
    footprints = len(re.findall(r'\(footprint\s', source))
    tracks = len(re.findall(r'\((?:segment|via|arc)\s', source))
    require(footprints > 0 and d.get('footprints') == footprints, 'Footprint inventory changed or empty')
    require(d.get('tracks_and_vias') == tracks and tracks > 0, 'Track/via inventory changed or empty')
    require(len(re.findall(r'\(footprint\s', saved)) == footprints, 'Saved footprint count mismatch')
    filled_layers = re.findall(r'\(filled_polygon\s+\(layer\s+"([^"]+)"', saved)
    require(filled_layers, 'Saved board contains no filled polygons')
    for layer in {x['layer'] for x in details}:
        require(layer in filled_layers, 'Saved board lacks reported fill layer '+layer)
    require({'In1.Cu','In2.Cu'} <= set(filled_layers), 'RVB reference planes are absent')
    return {'copper_zones':d['copper_zones'], 'zone_layer_pairs':len(details),
            'saved_filled_polygon_count':len(filled_layers), 'details':details}


def resolve_model_export(pcb: Path, output: Path) -> Path:
    """Resolve actual installed STEP names in an export-only board copy."""
    export_dir=output/'model_export';export_dir.mkdir()
    records=[];cache={}
    def replace(match):
        original=json.loads(match.group(2))
        expanded=os.path.expandvars(original.replace('${KIPRJMOD}',str(pcb.parent)))
        source=Path(expanded)
        candidates=[source.with_suffix('.step'),source.with_suffix('.stp'),source] if source.suffix.lower()=='.wrl' else [source]
        chosen=next((q for q in candidates if q.is_file()),None)
        row={'source_reference':original,'expanded':expanded,'resolved':str(chosen) if chosen else None}
        if chosen:
            resolved=chosen.resolve()
            if str(resolved) not in cache:cache[str(resolved)]=sha(resolved)
            row.update(sha256=cache[str(resolved)],bytes=resolved.stat().st_size)
        records.append(row)
        return match.group(1)+json.dumps(str(chosen.resolve()) if chosen else original)
    text=re.sub(r'(\(model\s+)("(?:[^"\\]|\\.)*")',replace,pcb.read_text())
    target=export_dir/pcb.name;target.write_text(text)
    (output/'MODEL_RESOLUTION.json').write_text(json.dumps({'source_pcb_sha256':sha(pcb),'export_copy_sha256':sha(target),'model_entries':records,'resolved_entries':sum(x['resolved'] is not None for x in records),'unresolved_entries':sum(x['resolved'] is None for x in records),'scope':'Only model file references differ. Missing footprint model entries and exact manufacturer envelope correctness remain separate checks.'},indent=2)+'\n')
    return target


def validate_component_log(path: Path) -> dict:
    log = path.read_text()
    missing = re.findall(r"Could not add 3D model for ([^.]+)\.", log)
    require(not missing, 'Missing component models: '+', '.join(missing))
    require('File not found:' not in log, 'Unresolved model path in STEP export')
    return {'missing_model_log_entries':0, 'scope':'Exporter log only; source model presence and envelope correctness remain separate checks.'}


def validate_native_report(path: Path, kind: str) -> dict:
    d = json.loads(path.read_text())
    require(isinstance(d, dict), 'Report is not an object')
    require(d.get('$schema') == f'https://schemas.kicad.org/{kind}.v1.json', 'Wrong native report schema')
    require(re.match(r'^9\.0\.9(?:$|[-+])', str(d.get('kicad_version',''))), 'Wrong report tool version')
    expected = 'GR86_CCA_RevB.kicad_sch' if kind == 'erc' else 'GR86_CCA_RevB.kicad_pcb'
    require(Path(str(d.get('source',''))).name == expected, 'Wrong report source')
    require(set(d.get('included_severities',[])) >= {'error','warning','exclusion'}, 'Incomplete severity coverage')
    if kind == 'erc':
        require(isinstance(d.get('sheets'),list) and d['sheets'], 'Missing ERC sheets')
        arrays = [x.get('violations') for x in d['sheets']]
    else:
        arrays = [d.get(k) for k in ['violations','unconnected_items','schematic_parity']]
    require(all(isinstance(x,list) for x in arrays), 'Missing native finding arrays')
    violations = [v for a in arrays for v in a]
    require(all(isinstance(v,dict) and v.get('severity') in {'error','warning','exclusion'} for v in violations),
            'Malformed finding record')
    return {'findings':len(violations), 'errors':sum(v['severity']=='error' for v in violations),
            'warnings':sum(v['severity']=='warning' for v in violations),
            'exclusions':sum(v['severity']=='exclusion' for v in violations)}


def validate_xml(path: Path) -> dict:
    root = ET.parse(path).getroot()
    require(root.tag == 'export', 'Wrong netlist XML root')
    components, nets = root.findall('./components/comp'), root.findall('./nets/net')
    require(components and nets and any(n.findall('node') for n in nets), 'Empty schematic netlist')
    return {'components':len(components),'nets':len(nets)}


def validate_d356(path: Path) -> dict:
    lines = path.read_text().splitlines()
    records = [s for s in lines if re.match(r'^(317|327|367)',s)]
    require(records and any(s.startswith('999') for s in lines), 'Missing IPC-D-356 records/terminator')
    return {'test_records':len(records)}


def validate_gerbers(folder: Path) -> dict:
    found = {}
    for p in folder.iterdir():
        if not p.is_file() or p.suffix.lower() == '.drl': continue
        text = p.read_text(errors='replace')
        match = re.search(r'%TF\.FileFunction,([^*]+)\*%', text)
        if match:
            function = match.group(1)
            require(function not in found, 'Duplicate Gerber layer '+function)
            require('%FSLAX' in text and '%MOMM*%' in text and 'M02*' in text,
                    'Malformed Gerber '+p.name)
            require(re.search(r'X-?\d+Y-?\d+D0[123]\*', text), 'Gerber has no coordinate records '+p.name)
            found[function] = p.name
    missing = [layer for layer,function in GERBER_FUNCTIONS.items() if function not in found]
    require(not missing, 'Missing requested Gerber layers: '+','.join(missing))
    return {layer:found[function] for layer,function in GERBER_FUNCTIONS.items()}


def validate_drills(folder: Path) -> dict:
    found = {}
    for p in folder.glob('*.drl'):
        text = p.read_text()
        require(text.startswith('M48') and 'M30' in text and 'METRIC' in text, 'Malformed Excellon '+p.name)
        match = re.search(r'TF\.FileFunction,(NonPlated|Plated),', text)
        require(match, 'Missing plating identity '+p.name)
        require(re.search(r'^T\d+C[\d.]+',text,re.M), 'No drill tools '+p.name)
        require(re.search(r'^X[-\d.]+Y[-\d.]+',text,re.M), 'No drill hits '+p.name)
        require(match.group(1) not in found, 'Duplicate drill plating file')
        found[match.group(1)] = p.name
    require(set(found)=={'Plated','NonPlated'}, 'Separate PTH and NPTH drill files required')
    return found


def validate_csv(path: Path, required: set[str]) -> dict:
    rows = list(csv.DictReader(io.StringIO(path.read_text())))
    require(rows and required <= set(rows[0]), 'CSV missing rows or required columns')
    return {'rows':len(rows),'columns':list(rows[0])}


def validate_magic(path: Path, magic: bytes) -> dict:
    data = path.read_bytes()
    require(len(data)>len(magic) and data.startswith(magic), 'Missing or wrong artifact type '+path.name)
    return {'bytes':len(data),'sha256':sha(path)}


def validate_firmware(folder: Path) -> dict:
    elf = folder/'cca_telemetry.ino.elf'
    app = folder/'cca_telemetry.ino.bin'
    a = validate_magic(elf,b'\x7fELF')
    b = validate_magic(app,b'\xe9')
    data = elf.read_bytes()
    require(len(data)>52 and data[4] == 1 and data[5] == 1 and int.from_bytes(data[18:20],'little')==94,
            'ELF is not 32-bit little-endian Xtensa')
    require(len(app.read_bytes())>32, 'ESP application binary too short')
    return {'elf':a,'app_bin':b}


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inventory(root: Path) -> dict[str, str]:
    result = {}
    for p in sorted(root.rglob('*')):
        if p.is_symlink():
            raise ValueError('Candidate contains a symlink; materialize explicitly: ' + str(p))
        if p.is_file():
            result[str(p.relative_to(root))] = sha(p)
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cad-dir', type=Path, required=True)
    parser.add_argument('--firmware-dir', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='New output directory')
    parser.add_argument('--kicad-cli', default=os.getenv('RVB_KICAD_CLI', 'kicad-cli'))
    parser.add_argument('--pcbnew-python', default=os.getenv('RVB_NATIVE_PYTHON', sys.executable))
    parser.add_argument('--arduino-cli', default=os.getenv('ARDUINO_CLI', 'arduino-cli'))
    parser.add_argument('--arduino-config', type=Path)
    parser.add_argument('--nimble-dir', type=Path, help='Explicit NimBLE-Arduino 2.3.6 library root')
    parser.add_argument('--preflight-only', action='store_true')
    parser.add_argument('--component-step', action='store_true',
                        help='Also export STEP with available component models; completeness still needs review')
    args = parser.parse_args()
    cad = args.cad_dir.resolve(strict=True)
    fw = args.firmware_dir.resolve(strict=True) if args.firmware_dir else None
    output = args.output.resolve()
    if output == cad or output.is_relative_to(cad) or (fw and (output == fw or output.is_relative_to(fw))):
        parser.error('Output must be outside both input trees')
    if output.exists():
        parser.error('Output exists; choose a new output directory')
    output.mkdir(parents=True)
    (output/'logs').mkdir()
    result = {'schema_version':1, 'mode':'PREFLIGHT_ONLY' if args.preflight_only else 'NATIVE_EVIDENCE',
              'status':'IN_PROGRESS', 'runner_sha256':sha(Path(__file__)), 'commands':[],
              'inputs':{'cad':inventory(cad)}, 'native_gates_run':False,
              'firmware_build_run':False, 'manufacturing_release':False,
              'output_postconditions':{}, 'copied_source_inventories':{},
              'remaining_project_exports_and_checks':[
                  'JLC panelization and panel Gerber/drill parity via existing project export tools',
                  'Manufacturing BOM/CPL reconciliation, assembly variants and exact supplier fields',
                  'All installed components represented in collision-checked STEP geometry',
                  'Controlled fabrication/assembly drawings and rendered schematic PDF review',
                  'Exact native finding waivers, independent Gerber connectivity and clearance reconstruction',
                  'Firmware actual dependency/compiler hashes and target timing verification']}
    if fw:
        result['inputs']['firmware'] = inventory(fw)
    def save():
        (output/'RESULT.json').write_text(json.dumps(result, indent=2)+'\n')
    def run(name, command, cwd=None, timeout=1800):
        command = [str(x) for x in command]
        start = time.monotonic()
        try:
            p = subprocess.run(command, cwd=cwd, capture_output=True, text=True,
                               timeout=timeout, env=dict(os.environ, TZ='UTC'))
            rc, stdout, stderr = p.returncode, p.stdout, p.stderr
        except (OSError, subprocess.TimeoutExpired) as exc:
            rc, stdout, stderr = 127, '', str(exc)
        (output/'logs'/f'{name}.stdout').write_text(stdout)
        (output/'logs'/f'{name}.stderr').write_text(stderr)
        rec = {'name':name,'argv':command,'exit_code':rc,
               'elapsed_seconds':round(time.monotonic()-start,3),
               'stdout':f'logs/{name}.stdout','stderr':f'logs/{name}.stderr'}
        result['commands'].append(rec)
        save()
        print(name, rc, flush=True)
        return rc, stdout, stderr
    def pinned(text, version):
        return re.search(r'(?<![\d.])'+re.escape(version)+r'(?![\d.])', text) is not None
    def postcondition(name, function, *values):
        try:
            value = function(*values)
            record = {'pass':True,'details':value}
        except Exception as exc:
            record = {'pass':False,'reason':str(exc)}
        result['output_postconditions'][name] = record
        save()
        return record['pass']

    kr = run('kicad_version', [args.kicad_cli, 'version'], timeout=30)
    pr = run('pcbnew_version', [args.pcbnew_python, '-c',
             'import pcbnew;print(pcbnew.GetBuildVersion())'], timeout=30)
    cad_ready = kr[0] == pr[0] == 0 and all(pinned(x[1], '9.0.9') for x in (kr,pr))
    result['preflight'] = {'kicad_9_0_9_and_bindings':cad_ready}
    cli = [args.arduino_cli]
    if args.arduino_config:
        cli += ['--config-file', str(args.arduino_config.resolve(strict=True))]
    fw_ready = False
    if fw:
        ar = run('arduino_version', cli+['version'], timeout=30)
        cores = run('arduino_core_list', cli+['core','list'], timeout=30)
        fw_ready = ar[0] == cores[0] == 0 and pinned(ar[1], '1.3.1')
        fw_ready = fw_ready and bool(re.search(r'^esp32:esp32\s+3\.3\.6(?:\s|$)', cores[1], re.M))
        nimble = args.nimble_dir.resolve(strict=True) if args.nimble_dir else None
        props = (nimble/'library.properties').read_text() if nimble and (nimble/'library.properties').exists() else ''
        fw_ready = fw_ready and bool(re.search(r'^version=2\.3\.6\s*$', props, re.M))
        result['preflight']['pinned_firmware_dependencies'] = fw_ready
        if nimble:
            result['inputs']['NimBLE'] = inventory(nimble)
    save()
    if args.preflight_only:
        result['status'] = 'PREFLIGHT_READY' if cad_ready and (not fw or fw_ready) else 'BLOCKED_RUNTIME'
    else:
        if cad_ready:
            target = output/'candidate_kicad'
            shutil.copytree(cad, target)
            result['copied_source_inventories']['cad_before_native'] = inventory(target)
            require(result['copied_source_inventories']['cad_before_native']==result['inputs']['cad'],
                    'CAD copy does not match frozen input')
            pcb = target/'GR86_CCA_RevB.kicad_pcb'
            pro = pcb.with_suffix('.kicad_pro')
            sch = pcb.with_suffix('.kicad_sch')
            original_pro = pro.read_bytes()
            refill = run('zone_refill', [args.pcbnew_python,'-c',REFILL,pro,pcb], cwd=target)
            # pcbnew can rewrite project rules when its SettingsManager exits.
            if pro.read_bytes() != original_pro:
                result['project_settings_restored_after_pcbnew'] = True
                pro.write_bytes(original_pro)
            refill_valid = postcondition('refilled_copper', validate_refill, refill[1], pcb,
                                        cad/'GR86_CCA_RevB.kicad_pcb')
            run('erc', [args.kicad_cli,'sch','erc','--format','json','--severity-all',
                '--exit-code-violations','-o',output/'ERC.json',sch], cwd=target)
            run('schematic_netlist', [args.kicad_cli,'sch','export','netlist','--format',
                'kicadxml','-o',output/'SCHEMATIC_NETLIST.xml',sch], cwd=target)
            run('schematic_bom', [args.kicad_cli,'sch','export','bom','--exclude-dnp',
                '--fields','Reference,Value,Footprint,Manufacturer,MPN,LCSC,${QUANTITY}',
                '--labels','Reference,Value,Footprint,Manufacturer,MPN,LCSC,Quantity',
                '-o',output/'REVIEW_BOM.csv',sch], cwd=target)
            run('schematic_pdf', [args.kicad_cli,'sch','export','pdf',
                '-o',output/'REVIEW_SCHEMATIC.pdf',sch], cwd=target)
            if refill[0] == 0 and refill_valid:
                result['refilled_pcb_sha256'] = sha(pcb)
                result['native_gates_run'] = True
                run('drc', [args.kicad_cli,'pcb','drc','--format','json','--severity-all',
                    '--all-track-errors','--schematic-parity','--exit-code-violations',
                    '-o',output/'DRC.json',pcb], cwd=target)
                run('pcb_netlist', [args.kicad_cli,'pcb','export','ipcd356','-o',
                    output/'PCB_NETLIST.d356',pcb], cwd=target)
                gerbers = output/'review_gerbers'
                gerbers.mkdir()
                run('gerbers', [args.kicad_cli,'pcb','export','gerbers','--layers',LAYERS,
                    '-o',str(gerbers)+os.sep,pcb], cwd=target)
                run('drills', [args.kicad_cli,'pcb','export','drill','--format','excellon',
                    '--excellon-units','mm','--excellon-separate-th','-o',str(gerbers)+os.sep,pcb], cwd=target)
                run('component_positions', [args.kicad_cli,'pcb','export','pos','--format','csv',
                    '--units','mm','--side','both','--exclude-dnp','--use-drill-file-origin',
                    '-o',output/'REVIEW_POSITIONS.csv',pcb], cwd=target)
                if args.component_step:
                    model_pcb=resolve_model_export(pcb,output)
                    run('component_step', [args.kicad_cli,'pcb','export','step','--no-dnp',
                        '--subst-models','-o',output/'REVIEW_COMPONENTS.step',model_pcb], cwd=target)
            if refill[0] != 0 or not refill_valid:
                run('unfilled_input_drc_diagnostic', [args.kicad_cli,'pcb','drc','--format','json',
                    '--severity-all','--exit-code-violations','-o',output/'UNFILLED_INPUT_DRC_DIAGNOSTIC.json',
                    cad/'GR86_CCA_RevB.kicad_pcb'], cwd=cad)
                result['unfilled_diagnostic_is_not_final_DRC'] = True
            postcondition('erc_report',validate_native_report,output/'ERC.json','erc')
            postcondition('drc_report',validate_native_report,output/'DRC.json','drc')
            postcondition('schematic_netlist',validate_xml,output/'SCHEMATIC_NETLIST.xml')
            postcondition('pcb_netlist',validate_d356,output/'PCB_NETLIST.d356')
            postcondition('requested_gerbers',validate_gerbers,output/'review_gerbers')
            postcondition('separate_drills',validate_drills,output/'review_gerbers')
            postcondition('schematic_bom',validate_csv,output/'REVIEW_BOM.csv',
                          {'Reference','Value','Footprint','MPN','LCSC','Quantity'})
            postcondition('component_positions',validate_csv,output/'REVIEW_POSITIONS.csv',
                          {'Ref','Val','Package','PosX','PosY','Rot','Side'})
            postcondition('schematic_pdf',validate_magic,output/'REVIEW_SCHEMATIC.pdf',b'%PDF-')
            if args.component_step:
                postcondition('component_step_file',validate_magic,output/'REVIEW_COMPONENTS.step',b'ISO-10303-21;')
                postcondition('component_model_log',validate_component_log,output/'logs/component_step.stdout')
            result['copied_source_inventories']['cad_after_native'] = inventory(target)
        if fw and fw_ready:
            target_fw = output/'candidate_firmware'
            shutil.copytree(fw, target_fw)
            result['copied_source_inventories']['firmware_before_compile'] = inventory(target_fw)
            require(result['copied_source_inventories']['firmware_before_compile']==result['inputs']['firmware'],
                    'Firmware copy does not match frozen input')
            result['firmware_build_run'] = True
            run('firmware_compile', cli+['compile','--fqbn',FQBN,'--warnings','all','--verbose',
                '--clean','--library',nimble,'--build-path',output/'firmware_build',
                '--output-dir',output/'firmware_images',target_fw/'cca_telemetry'], cwd=target_fw)
            postcondition('firmware_artifacts',validate_firmware,output/'firmware_images')
            result['copied_source_inventories']['firmware_after_compile'] = inventory(target_fw)
        failures = [c['name'] for c in result['commands'] if c['exit_code'] != 0]
        result['nonzero_commands'] = failures
        incomplete = [name for name,rec in result['output_postconditions'].items() if not rec['pass']]
        result['incomplete_outputs'] = incomplete
        native_findings = sum(result['output_postconditions'].get(name,{}).get('details',{}).get('findings',0)
                              for name in ['erc_report','drc_report'])
        result['native_report_finding_count'] = native_findings
        result['status'] = ('BLOCKED_RUNTIME' if not cad_ready or (fw and not fw_ready) else
                            'INCOMPLETE_NATIVE_OUTPUT' if incomplete else
                            'NATIVE_FINDINGS_OR_FAILURES' if failures or native_findings else
                            'NATIVE_OUTPUTS_COMPLETE')
        result['acceptance_note'] = ('No review criteria automatically closed. Inspect all native findings, '
            'explicit waivers, library selection and source/output parity. An exported review file is not '
            'a release and a compiled image is not target execution.')
    result['inputs_unchanged'] = inventory(cad) == result['inputs']['cad'] and (not fw or inventory(fw) == result['inputs']['firmware'])
    if fw and nimble:
        result['inputs_unchanged'] = result['inputs_unchanged'] and inventory(nimble) == result['inputs']['NimBLE']
    if not result['inputs_unchanged']:
        result['status'] = 'INPUT_CHANGED_DURING_RUN'
    copied_path = output/'COPIED_SOURCE_INVENTORY.json'
    copied_path.write_text(json.dumps(result.pop('copied_source_inventories'),indent=2)+'\n')
    result['copied_source_inventory'] = {'path':copied_path.name,'sha256':sha(copied_path)}
    manifest_path = output/'OUTPUT_MANIFEST.json'
    manifest = {'schema_version':1,'excludes':['RESULT.json','OUTPUT_MANIFEST.json'],
                'files':{str(p.relative_to(output)):{'sha256':sha(p),'bytes':p.stat().st_size}
                         for p in sorted(output.rglob('*')) if p.is_file() and p not in
                         {output/'RESULT.json',manifest_path}}}
    manifest_path.write_text(json.dumps(manifest,indent=2)+'\n')
    result['output_manifest'] = {'path':manifest_path.name,'sha256':sha(manifest_path)}
    save()
    print(json.dumps({'status':result['status'],'output':str(output),'inputs_unchanged':result['inputs_unchanged']}))
    return 0 if result['status'] in ('PREFLIGHT_READY','NATIVE_OUTPUTS_COMPLETE') else 2


if __name__ == '__main__':
    raise SystemExit(main())
