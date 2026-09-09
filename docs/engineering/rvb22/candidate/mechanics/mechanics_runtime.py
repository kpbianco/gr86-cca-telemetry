from pathlib import Path
import sys,os
HERE=Path(__file__).resolve().parent
for parent in [HERE,*HERE.parents]:
 for candidate in [parent/'runtime/rf',parent/'current/runtime/rf']:
  if (candidate/'sexpdata.py').is_file():sys.path.insert(0,str(candidate))
def discover_project():
 if os.environ.get('RVB22_PROJECT_ROOT'):return Path(os.environ['RVB22_PROJECT_ROOT']).resolve()
 for parent in [HERE,*HERE.parents]:
  for c in [parent,parent/'baseline/GR86_CCA_RevB',parent/'GR86_CCA_RevB']:
   if (c/'audit/online_resolution_rvb21/reset/candidate_kicad/GR86_CCA_RevB.kicad_pcb').is_file():return c
 return None
