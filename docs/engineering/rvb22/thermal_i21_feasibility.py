#!/usr/bin/env python3
"""Optimistic existing-contact boundary test; no CAD/cooling credit is adopted."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('recovery', type=Path)
args = parser.parse_args()
w = args.recovery
source = w / 'analyses/thermal_i18'
sys.path.insert(0, str(source))
import extract as ex
import model_fast as model

out = w / 'analyses/thermal_i21'
out.mkdir(exist_ok=True)
model.D = out
e = ex.extract()
model.board = ex.m.board
result = model.solve(e, step=.25, mode='provisional', plating_um=15, contact_R=.01, wing_R=.01, edge_mode='face', save_map=True)
result.update(source_PCB_sha256=ex.SHA, mode='actual_native_fill', interpretation='Optimistic 0.01 K/W at each existing contact, nearly fixed at the 70 C landing. This is a feasibility comparison only, not an achievable contact construction or a release result.')
baseline = next(r for r in json.loads((source / 'face_15.0/RESULTS.json').read_text()) if r['mesh_mm'] == .25)
data = {'status': 'FEASIBILITY_ONLY', 'source_model_sha256': hashlib.sha256((source / 'model_fast.py').read_bytes()).hexdigest(), 'baseline': baseline, 'optimistic_existing_contacts': result, 'same_mesh_hotspot_reduction_C': baseline['max_board_C'] - result['max_board_C'], 'CAD_changed': False, 'physical_test': False, 'package_temperature_proven': False}
(out / 'EXISTING_CONTACT_FEASIBILITY.json').write_text(json.dumps(data, indent=2) + '\n')
print(json.dumps(data, indent=2), flush=True)
