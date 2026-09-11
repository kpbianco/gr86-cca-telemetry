#!/usr/bin/env python3
"""Reproduce the bounded feedback-layout trial from the exact I24 source.

Writes a separate candidate only. No live hardware, source adoption or release.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil
import uuid

BASE = '5b373f6033fdbd18f126f8ca054b619abada2568778c5a8fc303c4e756fb06c8'
KEEP_VIA = '4eabd859-1c65-44f0-b0f2-197b528fff61'
LEAVES = {'8cb33166-2699-5311-b448-a141ed5f9309',
          '99a942b9-65b0-580e-bde6-aa983db1a3a0',
          '70849dce-f027-53f5-81fb-448694c6de51'}
PATHS = [
 (132, 'B.Cu', [(37.8625,10.725),(39.1,10.725)]),
 (132, 'In1.Cu', [(39.1,10.725),(38.9,10.925),(38.9,13.5),(38.9,13.8),(39.2,14.1)]),
 (132, 'F.Cu', [(39.2,14.1),(39.2,13.925),(39.4,13.725)]),
 (132, 'F.Cu', [(39.2,14.1),(39.2,15.075),(39.4,15.275)]),
 (5, 'F.Cu', [(39.5,12.1),(39.425,12.1),(39.4,12.075)]),
 (5, 'In2.Cu', [(39.5,12.1),(40.4,13.0),(40.4,21.5),(40.1,21.8),(40.1,23.5),(40.1,24.2),(40.7,24.8)]),
 (30, 'F.Cu', [(39.4,16.925),(39.4,18.5515),(39.8485,19.0)])]


def records(text):
    depth=0; quoted=False; escaped=False; begin=None
    for i,ch in enumerate(text):
        if quoted:
            if escaped: escaped=False
            elif ch=='\\': escaped=True
            elif ch=='"': quoted=False
            continue
        if ch=='"': quoted=True
        elif ch=='(':
            depth+=1
            if depth==2: begin=i
        elif ch==')':
            if depth==2: yield begin,i+1,text[begin:i+1]
            depth-=1
            if depth<0: raise ValueError('Unbalanced input')
    if depth or quoted: raise ValueError('Unbalanced input')


def transform(text):
    if hashlib.sha256(text.encode()).hexdigest()!=BASE:
        raise ValueError('This trial requires the exact I24 source; do not apply to another revision.')
    changes=[]; removed=[]; moved=[]
    for a,z,rec in records(text):
        match=re.search(r'\(uuid "([^"]+)"\)',rec)
        if not match: continue
        ident=match.group(1)
        if ((rec.startswith('(segment ') or rec.startswith('(via '))
            and re.search(r'\(net 132\)',rec) and ident!=KEEP_VIA) or ident in LEAVES:
            changes.append((a,z,'')); removed.append(ident)
        elif rec.startswith('(footprint ') and any('(property "Reference" "'+r+'"' in rec for r in ['R155','R156']):
            ref='R155' if '(property "Reference" "R155"' in rec else 'R156'
            y=12.9 if ref=='R155' else 16.1
            children=list(records(rec)); at=next((aa,zz) for aa,zz,t in children if t.startswith('(at '))
            rec=rec[:at[0]]+f'(at 39.4 {y} 270)'+rec[at[1]:]
            rec=rec.replace('"B.Cu"','"F.Cu"').replace('"B.Paste"','"F.Paste"').replace('"B.Mask"','"F.Mask"').replace('"B.Fab"','"F.Fab"').replace('"B.CrtYd"','"F.CrtYd"').replace('"B.SilkS"','"F.SilkS"')
            rec=rec.replace('(justify mirror)','')
            rec=re.sub(r'\(at (-?0\.825) 0 (?:90|270)\)',r'(at \1 0 270)',rec)
            changes.append((a,z,rec)); moved.append(ref)
    if len(removed)!=89 or sorted(moved)!=['R155','R156']:
        raise ValueError(f'Unexpected edit census: {len(removed)} / {moved}')
    for a,z,rec in reversed(changes): text=text[:a]+rec+text[z:]
    added=[]
    for net,name,p in [(132,'3V3_FB_ADJ',(39.2,14.1)),(5,'3V3_SOURCE',(39.5,12.1))]:
        ident=str(uuid.uuid5(uuid.NAMESPACE_URL,'desktop-wave1-feedback-via:'+name))
        added.append(f'(via (at {p[0]} {p[1]}) (size 0.6) (drill 0.3) (layers "F.Cu" "B.Cu") (net {net}) (uuid "{ident}"))')
    for net,layer,points in PATHS:
        for a,z in zip(points,points[1:]):
            ident=str(uuid.uuid5(uuid.NAMESPACE_URL,'desktop-wave1-feedback:'+json.dumps([net,layer,a,z])))
            added.append(f'(segment (start {a[0]} {a[1]}) (end {z[0]} {z[1]}) (width 0.2) (layer "{layer}") (net {net}) (uuid "{ident}"))')
    return text.rstrip()[:-1]+'\n'+'\n'.join(added)+'\n)\n',removed


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,default=Path(__file__).resolve().parents[2]/'candidate/cad')
    ap.add_argument('--out',type=Path,required=True)
    args=ap.parse_args()
    if args.out.resolve()==args.source.resolve(): raise ValueError('An isolated output directory is required')
    p=args.source/'GR86_CCA_RevB.kicad_pcb'
    text,removed=transform(p.read_text())
    shutil.copytree(args.source,args.out,dirs_exist_ok=True)
    (args.out/p.name).write_text(text)
    data=dict(base_sha256=BASE,candidate_sha256=hashlib.sha256(text.encode()).hexdigest(),
        changed_footprints=['R155','R156'],removed_uuids=removed,
        added_vias_requiring_fill_cap_planarize=[{'xy_mm':[39.2,14.1],'drill_mm':.3,'ref':'R155.2'},{'xy_mm':[39.5,12.1],'drill_mm':.3,'ref':'R155.1'}],
        existing_fill_cap_process_count=49,new_total_fill_cap_count=51,
        native_checks_performed=False,fabrication_release=False)
    (args.out.parent/'FEEDBACK_BINDING.json').write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps({k:v for k,v in data.items()if k!='removed_uuids'}))

if __name__=='__main__': main()
