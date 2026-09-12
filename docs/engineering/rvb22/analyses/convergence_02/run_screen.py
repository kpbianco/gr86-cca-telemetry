import argparse,json,sys,hashlib
from pathlib import Path
D=Path(__file__).resolve().parent
sys.path.insert(0,str(D.parent/"convergence_01"))
import thermal_native as tn
import board_matrix as bm
from shared_network import solve_network
def main():
    p=argparse.ArgumentParser()
    p.add_argument("--native-dir",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    p.add_argument("--mesh",type=float,default=.5)
    p.add_argument("--sink-resistances",type=float,nargs="*",default=[1.,2.])
    a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    pcb=a.native_dir/"candidate_kicad/GR86_CCA_RevB.kicad_pcb"
    ex,b=tn.extract_native(pcb)
    for n in ("G","board","carrier","heat"):setattr(bm,n,getattr(tn.model,n))
    m=bm.prepare(ex,step=a.mesh,mode="provisional",plating_um=15.)
    cfg=json.loads((D/"CONTACT_PROPOSALS.json").read_text())
    results=[]
    cases=[("shared_baseline",cfg["baseline_contacts"],None),
           ("rf_safe_fixed",cfg["rf_safe_trial_contacts"],None)]
    cases += [(f"rf_safe_sink_{r:g}",cfg["rf_safe_trial_contacts"],r) for r in a.sink_resistances]
    for name,contacts,res in cases:
        result=solve_network(m,contacts,sink_R=res,save_map=a.out/(name+".npz"))
        result["case"]=name;results.append(result)
        (a.out/(name+".json")).write_text(json.dumps(result,indent=2)+"\n")
        print(json.dumps({"case":name,"max_C":result["max_board_C"],"sink_C":result["sink_C"],
            "heat_W":result["heat_W"],"energy_error_W":result["energy_balance_residual_W"]}),flush=True)
    report={"source":ex[-1],"cases":results,
       "scope":"Unadopted geometry / allocated contacts / native-copper board-only model. No junction, installed-air, material or mechanical acceptance.",
       "sources_sha256":{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in
             [D/"run_screen.py",D/"board_matrix.py",D/"shared_network.py",D/"CONTACT_PROPOSALS.json"]}}
    (a.out/"RESULTS.json").write_text(json.dumps(report,indent=2)+"\n")
if __name__=="__main__":main()
