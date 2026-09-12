#!/usr/bin/env python3
"""Create an optional grounded thermal-contact window, not a qualified cooler."""
import argparse, hashlib, json, re
from pathlib import Path
from apply_feedback import records
BASE='17bd9ad2737c776cc6deb2470c446485782f07f992da4cbaaf1efe5235597fa0'
RESULT='a5a7916f426f455736b02c3b7c4073b066417838758f50ebd199c354d08fb976'
REMOVE={'a9a4f9b3-a3af-48da-86d9-84df2e098f3b','bb6e2330-54ce-4ce4-b60d-6f6d893e58a5','ecdc64b5-640f-40ec-915b-98d8dd21d8dc'}
ADD=[
 '(segment (start 81.1 22.4) (end 81.1 23.8) (width .2) (layer "B.Cu") (net 23) (uuid "5076ee0d-d344-591e-a77f-07fadf11407a"))',
 '(segment (start 81.1 23.8) (end 80.6 24.3) (width .2) (layer "B.Cu") (net 23) (uuid "64f1dacb-7f06-5bce-8cd2-b42af7d53f9b"))',
 '(segment (start 80.6 24.3) (end 79.3 24.3) (width .2) (layer "B.Cu") (net 23) (uuid "693f4bb4-de56-571c-aac8-1db4c7fbf91e"))',
 '(segment (start 79.3 24.3) (end 78.3 25.3) (width .2) (layer "B.Cu") (net 23) (uuid "5a56e029-2b5e-5c0c-8e18-3c6ace81629e"))',
 '(gr_rect (start 76.819766 19.725) (end 80.819766 23.725) (stroke (width 0) (type solid)) (fill solid) (layer "B.Mask") (uuid "2fe48054-08a3-55bd-ba52-55bc80f44b6a"))']
def transform(text):
    if hashlib.sha256(text.encode()).hexdigest()!=BASE: raise ValueError('Unexpected feedback candidate')
    changes=[]
    for a,z,t in records(text):
        m=re.search(r'\(uuid "([^"]+)"\)',t)
        if m and m.group(1)in REMOVE: changes.append((a,z))
    if len(changes)!=3: raise ValueError('Unexpected CAN_RX branch census')
    for a,z in reversed(changes): text=text[:a]+text[z:]
    text=text.rstrip()[:-1]+'\n'+'\n'.join(ADD)+'\n)\n'
    if hashlib.sha256(text.encode()).hexdigest()!=RESULT: raise ValueError('Unexpected generated candidate')
    return text
if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('pcb',type=Path);a=ap.parse_args()
    a.pcb.write_text(transform(a.pcb.read_text()))
    data=dict(source_sha256=RESULT,thermal_window_B_Mask_mm=[76.819766,19.725,80.819766,23.725],
        moved_CAN_RX_branch_mm=[[81.1,22.4],[81.1,23.8],[80.6,24.3],[79.3,24.3],[78.3,25.3]],
        design_intent='Native refill must show only connected GND beneath the entire window. Keep adjacent signal mask intact.',
        construction='Expose the existing filled/capped U201 ground via array; no added external metal is adopted.',
        limits=['No contact, bridge, sink, module-temperature or RF installed qualification is conferred.','A future removable insulated contact must fit within the land with tolerance; do not treat the whole 4 mm mask opening as an allowed moving metal envelope.'],
        fabrication_release=False)
    (a.pcb.parent.parent/'THERMAL_ACCESS_BINDING.json').write_text(json.dumps(data,indent=2)+'\n')
    print(json.dumps(data))
