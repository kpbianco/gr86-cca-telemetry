#!/usr/bin/env python3
"""Extract LED board-region temperatures from the retained I25 fine-mesh map."""
import argparse, hashlib, json
from pathlib import Path
import numpy as np
D=Path(__file__).resolve().parent

def extract(path):
    expected=json.loads((D/'LED_THERMAL_REGIONS.json').read_text())
    if hashlib.sha256(path.read_bytes()).hexdigest()!=expected['map_sha256']:
        raise ValueError('Wrong I25 fine-mesh map')
    with np.load(path,allow_pickle=False) as data:
        rows=[]
        for old in expected['rows']:
            x,y=old['center_mm'];h=old['region_halfwidth_mm']
            select=(abs(data['x']-x)<h)&(abs(data['y']-y)<h)
            if not select.any(): raise ValueError('Missing LED region')
            value=float(np.max(data['T'][:,select]))
            rows.append(dict(old,cell_count=int(select.sum()),max_board_region_C=value,
                             board_region_with_global_mesh_allowance_C=value+expected['global_mesh_allowance_C']))
    return dict(expected,rows=rows)

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('map',type=Path);ap.add_argument('--out',type=Path,default=D/'LED_THERMAL_REGIONS.json')
    args=ap.parse_args();args.out.write_text(json.dumps(extract(args.map),indent=2)+'\n')
