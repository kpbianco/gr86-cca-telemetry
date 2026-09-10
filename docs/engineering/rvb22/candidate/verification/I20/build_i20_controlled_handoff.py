from pathlib import Path
import sys,json,hashlib,shutil
W=Path(__file__).resolve().parents[1];sys.path[:0]=[str(W/'runtime/local'),str(W/'runtime/vendor'),str(W/'control')]
import check_combined_copper as c,sexpdata as sx
I=W/'iterations/I20_controlled_handoff';I.mkdir(exist_ok=True)
freeze={'iteration':'I20','redlines':[{'id':'I20-D01','finding':'Power schematic annotations still state superseded23.7k/10k and old voltage bounds.'},{'id':'I20-D02','finding':'Candidate labels still say native validation pending after the executed I19 pass.'},{'id':'I20-D03','finding':'Current mechanics and acceptance instructions must point to C05/W02/T03 and49 filled/capped locations; historical copies must be visibly superseded.'},{'id':'I20-D04','finding':'Final review must use the user73/213/4 baseline and preserve every original criterion, with model and physical/supplier status separately recorded.'}], 'geometry_change_authorized':False,'rule':'Freeze before edit; only two schematic note objects and non-electrical qualification labels in existing candidate source. Current docs/manifest are generated separately.'}
(I/'REDLINE_FREEZE.json').write_text(json.dumps(freeze,indent=2)+'\n')
old=W/'iterations/I19_precision_feedback/candidate_kicad';new=I/'candidate_kicad';shutil.copytree(old,new,dirs_exist_ok=True)
transport={};changes={}
for p in list(new.glob('*.kicad_sch'))+[new/'GR86_CCA_RevB.kicad_pcb']:
 before=p.read_text();after=before.replace('RVB22 reconstructed source candidate; native validation pending','RVB22 engineering candidate; see current native evidence and acceptance conditions').replace('RVB22 reconstructed power candidate; native validation pending','RVB22 engineering candidate; see current native evidence and acceptance conditions')
 if p.name=='Power_3V3.kicad_sch':
  pairs=[('ADJUSTABLE3.37V MAIN','ADJUSTABLE3.36473V MAIN'),('R15523.7k/R15610k','R15511.8k/R1564.99k (TNPU0.02%,2ppm/K)'),('Source tolerance3.297890–3.442858V','Source bound3.306564–3.423112V including stated drift'),('adjustable3.37V,23.7k/10k feedback','adjustable3.36473V,11.8k/4.99k feedback'),('Native ERC/refill/DRC not run; numerical model conditions are controlled separately.','I19 native ERC/refill/DRC and target build passed; current I20 report and model conditions control.')]
  for a,z in pairs:assert a in after,a;after=after.replace(a,z)
 if after==before:continue
  # All electrical/topological objects are unchanged; metadata labels and two
  # notes are removed before the semantic comparison.
 def normalize(v):
  if isinstance(v,list):
   if v and str(v[0])=='text':return None
   if v and str(v[0])=='property' and len(v)>1 and str(v[1])=='Qualification':return None
   return [q for x in v if (q:=normalize(x))is not None]
  return v
 assert normalize(sx.loads(before))==normalize(sx.loads(after)),p.name
 p.write_text(after);changes[p.name]={'before_sha256':hashlib.sha256(before.encode()).hexdigest(),'after_sha256':hashlib.sha256(after.encode()).hexdigest(),'all_non_note_non_qualification_objects_identical':True}
 transport[p.name]={'text':after,'sha256':changes[p.name]['after_sha256']}
readme='''# RVB22 I20 engineering candidate

This folder is the current CAD source. Native KiCad outputs are bound by SHA256 to the exact source and pinned tool versions in the current verification manifest. Engineering iteration I20 corrects annotations; its circuit, pads, placement, copper, outline and TranquilWorks artwork are identical to I19.

The coordinated mechanical candidate is C05 carrier with W02 wing flex links and T03 central contact. The bulk capacitor C206 is on F.Cu at (82,-4) on the added grounded tab. All dimensions are millimetres. The overall PCB envelope remains 88.254766 by61.275846mm; the ESP module overhang is additional.

The 3V3 feedback divider is TNPU060311K8HWEA00 / TNPU06034K99HWEA00,11.8k/4.99k,0.02%,2ppm/K. Do not substitute the superseded23.7k/10k parts. C206 is T598X477M006ATE025 and R158 is WSLP0603R0820FEA. Current land-specific library variants and all153 fitted component envelopes are included.

Use current generated BOM, placement, Gerbers, drills, schematic PDF and STEP together.49 filled/capped/planarized via locations are required:48 at U201 pad41 and one at R153. Plating minimum15um is an engineering condition, not implied by a supplier average.

The native checks and calculations concern intended design. Model applicability, purchased accessories, assembly process, thermal installation and unit qualification are explicitly recorded in CURRENT_ACCEPTANCE_CONDITIONS.md and the review register. This folder does not authorize manufacture, energization or vehicle use. Superseded snapshots remain historical evidence.
'''
(new/'CANDIDATE_README.md').write_text(readme)
(new/'RVB22_POWER_CANDIDATE_ONLY.txt').write_text('RVB22 I20 engineering candidate. Current source-bound native outputs and finite circuit/thermal/RF models are provided. Assembly, controller/model correlation and installation acceptance remain explicit; no manufactured or vehicle pass is claimed. See CANDIDATE_README.md and the current controlled manifest.\n')
proof={'iteration':'I20','source_changes':changes,'PCB_sha256':hashlib.sha256((new/'GR86_CCA_RevB.kicad_pcb').read_bytes()).hexdigest(),'geometry_and_electrical_properties_identical_to_I19':True,'note':'Only two top-level schematic note objects and non-electrical Qualification labels changed; candidate README/status files replaced. No electrical, geometric or firmware behavior change.'}
(I/'SOURCE_CHANGE_CHECK.json').write_text(json.dumps(proof,indent=2)+'\n')
# Large PCB is transported through an exact literal metadata substitution;
# small source files can be loaded directly with their expected hashes.
(I/'TRANSPORT.json').write_text(json.dumps({'replacements':[['RVB22 reconstructed source candidate; native validation pending','RVB22 engineering candidate; see current native evidence and acceptance conditions'],['RVB22 reconstructed power candidate; native validation pending','RVB22 engineering candidate; see current native evidence and acceptance conditions']],'files':changes},indent=2)+'\n')
print(json.dumps(proof,indent=2))
