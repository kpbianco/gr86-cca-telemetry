#!/usr/bin/env python3
import argparse,json,sys
from pathlib import Path
D=Path(__file__).resolve().parent
sys.path.insert(0,str(D.parent/'convergence_01'))
sys.path.insert(0,str(D.parent/'convergence_02'))
import thermal_native as tn
import board_matrix as bm
from shared_network import solve_network

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--native-dir',type=Path,required=True);ap.add_argument('--out',type=Path,required=True);ap.add_argument('--mesh',type=float,required=True)
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    profile=json.loads((D/'RELEASE_POWER_PROFILE.json').read_text())
    contact_cfg=json.loads((D.parent/'convergence_02/CONTACT_PROPOSALS.json').read_text())
    baseline_contacts=contact_cfg['baseline_contacts']
    contacts=contact_cfg['rf_safe_trial_contacts']
    pcb=a.native_dir/'candidate_kicad/GR86_CCA_RevB.kicad_pcb'
    ex,b=tn.extract_native(pcb)
    original=list(tn.model.heat)
    release=[]
    for name,power,geom in original:
        release.append((name,profile['u201_heat_allocation_W'] if name=='U201' else power,geom))
    tn.model.heat=release
    for n in ('G','board','carrier','heat'): setattr(bm,n,getattr(tn.model,n))
    m=bm.prepare(ex,step=a.mesh,mode='provisional',plating_um=15.)
    results=[]
    for name,cfg,sink in [
      ('release_existing_cooling',baseline_contacts,1.0),
      ('release_c02_contingency',contacts,1.0),
      ('legacy_full_u201_stress_c02',contacts,1.0)]:
        if name.startswith('legacy'):
            tn.model.heat=original;bm.heat=original
            m=bm.prepare(ex,step=a.mesh,mode='provisional',plating_um=15.)
        r=solve_network(m,cfg,sink_R=sink,save_map=a.out/(name+'.npz'))
        r['case']=name;results.append(r)
        (a.out/(name+'.json')).write_text(json.dumps(r,indent=2)+'\n')
    out={'profile':profile,'results':results,'source':ex[-1],
      'scope':'Desktop board-region thermal screen. Release power profile is software-constrained but current allocation remains a bring-up acceptance value, not a hardware limiter.',
      'package_temperature_proven':False,'physical_test':False}
    (a.out/'RESULTS.json').write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps([{'case':r['case'],'max_C':r['max_board_C'],'u201_C':r['source_region_mean_C']['U201'],'u121_C':r['source_region_mean_C']['U121'],'sink_C':r['sink_C']} for r in results],indent=2))
if __name__=='__main__': main()
