#!/usr/bin/env python3
"""Label finite geometric-search results without promoting them to acceptance."""
import copy, json, math
from pathlib import Path
D=Path(__file__).resolve().parent
rows=json.loads((D/'TRANSFER_SCREEN_PATHS_EXPLORATORY.json').read_text())
if len(rows)!=60 or {r['index'] for r in rows}!=set(range(60)):
    raise ValueError('Incomplete or duplicated transfer inventory')
rows=copy.deepcopy(rows)
for row in rows:
    if row['logic_margin_proven']:
        raise ValueError('Exploration cannot claim receiver-margin proof')
    if row['solution']:
        for path in row['solution']['paths'].values():path.pop('xy',None)
found=sum(r['solution'] is not None for r in rows)
result={'status':'EXPLORATORY_NOT_A_CLOSURE_PROOF','source_filled_PCB_sha256':'11533ea91c3bc4c61dd7066e0001914a72bc90b27afb38d321c43b8feb90e3b1','signal_vias':60,'candidate_corridors_found':found,'unresolved_finite_searches':60-found,'method':{'grid_step_mm':.1,'etch_allowance_mm':.03,'corridor_halfwidth_mm':.025,'full_cell_erosion_mm':.1/math.sqrt(2),'ground_via_candidates_per_signal':5,'finite_search_margin_mm':3,'inductance_screen':'0.4*ell_mm*max(1,ln(2*ell_mm/0.025)) nH; deliberately thin-filament assumed estimate; not extracted impedance or a proven upper bound','current_edge_sensitivities':'10/20/40mA over0.5/1/2/5ns are selected sensitivity cases, not guaranteed driver bounds. V=L*dI/dt cannot establish settled receiver timing by itself.'},'limitations':['Endpoint snapping and local via-cap treatment require independent path-width/etch validation.','Finite-search failures do not prove PCB discontinuities.','The source current/edge envelope, receiver settling, voltage overshoot and clamp behavior have NOT been bounded.','No route is accepted or rejected merely by this exploratory inductance screen.','No physical tests or new native KiCad execution occurred in this extraction.'],'rows':rows,'gnd02_desktop_complete':False,'physical_test_claimed':False}
if found!=53:raise ValueError('Search result changed; engineering adjudication required')
(D/'TRANSFER_SCREEN_EXPLORATORY.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k not in ('rows','method','limitations')}))
