#!/usr/bin/env python3
"""CAN22-02: separate relaxed connector exits and twisted differential branch.
Finite normal-operation RLGC sensitivity, not a vehicle or as-built qualification.
"""
from pathlib import Path
import json,itertools,hashlib,math,time
import numpy as np
from scipy.linalg import eig
O=Path(__file__).resolve().parent;W=O.parents[1]
EPS0=8.8541878128e-12;MU0=1.25663706212e-6;C0=1/math.sqrt(EPS0*MU0)
WIRE_LENGTH=.30;WIRE_LOOP_R=.03723;BOARD_R=.025
TIME=np.unique(np.r_[np.arange(0,20e-9,.025e-9),np.arange(20e-9,200e-9,.1e-9),np.arange(200e-9,2e-6,2e-9),1.6e-6])

def straight(name,length,spacing,diameter,epsilon):
 a=math.acosh(spacing/diameter)
 L=MU0/math.pi*a*length;C=math.pi*EPS0*epsilon/a*length
 return {'name':name,'length_m':length,'pair_spacing_m':spacing,'conductor_envelope_m':diameter,'effective_epsilon':epsilon,'L_H':L,'C_F':C,'R_ohm':WIRE_LOOP_R*length/WIRE_LENGTH,'G_S':0.,'Z_ohm':math.sqrt((MU0/math.pi*a)/(math.pi*EPS0*epsilon/a)),'delay_s':math.sqrt(L*C)}

def geometries():
 out=[]
 for lf,la,d,df,ef in itertools.product([.0254,.030],[0.,.015],[.00078,.00080],[.003,.004],[1.,8.]):
  acases=[(.003,1.)] if la==0 else itertools.product([.003,.004],[1.,8.])
  for da,ea in acases:
   out.append({'id':len(out),'molex':straight('Molex_relaxed_exit',lf,df,d,ef),'ASC':straight('ASC_free_exit',la,da,d,ea),'twisted_conductor_length_m':WIRE_LENGTH-lf-la})
 return out

def sections(geo,z,td_full):
 length=geo['twisted_conductor_length_m'];td=td_full*length/WIRE_LENGTH
 twist={'name':'remaining_twisted_AVSS','length_m':length,'L_H':z*td,'C_F':td/z,'R_ohm':WIRE_LOOP_R*length/WIRE_LENGTH,'G_S':0.,'Z_ohm':z,'delay_s':td}
 return [geo['ASC'],twist,geo['molex']]

def system(geo,z,td,rs,cextra,cpin=20e-12,n=16):
 # Cable section order follows source ASC -> twisted harness -> F1 -> board -> receiver.
 ss=sections(geo,z,td);ls=[];cs=[];rr=[];cell_table=[]
 for section in ss:
  if section['length_m']==0:continue
  cells=n if section['name']=='remaining_twisted_AVSS' else max(1,math.ceil(n*section['length_m']/WIRE_LENGTH))
  ls.extend([section['L_H']/cells]*cells);cs.extend([section['C_F']/cells]*cells);rr.extend([section['R_ohm']/cells]*cells)
  cell_table.append({'name':section['name'],'cells':cells,**section})
 cable_cells=len(ls);nb=max(2,n//8)
 ls.extend([80*.1e-9/nb]*nb);cs.extend([.1e-9/80/nb]*nb);rr.extend([BOARD_R/nb]*nb)
 ls=np.array(ls);cs=np.array(cs);rr=np.array(rr);count=len(ls);caps=np.zeros(count+1)
 caps[:-1]+=cs/2;caps[1:]+=cs/2;caps[cable_cells]+=cextra;caps[-1]+=cpin
 N=count+1;A=np.zeros((N+count,N+count));B=np.zeros(N+count);scale=60.
 A[0,0]-=1/(rs*caps[0]);B[0]=1/(rs*caps[0]);A[N-1,N-1]-=1/(25000*caps[-1])
 for i,(L,R) in enumerate(zip(ls,rr)):
  A[i,N+i]-=1/(scale*caps[i]);A[i+1,N+i]+=1/(scale*caps[i+1]);A[N+i,i]+=scale/L;A[N+i,i+1]-=scale/L;A[N+i,N+i]-=R/L
 steady=np.linalg.solve(-A,B);ev,V=eig(A)
 assert np.max(ev.real)<0,'Unstable passive ladder'
 co=np.linalg.solve(V,-steady);indices=[0,N-1];mix=V[indices,:]*co
 return {'ev':ev,'mix':mix,'steady':steady[indices],'cell_table':cell_table,'cable_cells':cable_cells,'board_cells':nb,'series_R_total':float(rr.sum()),'nodal_DC_error':float(abs(steady[N-1]-25000/(25000+rs+WIRE_LOOP_R+BOARD_R))),'passivity_max_eigen_real':float(np.max(ev.real))}

def evaluate(s,tr,t=TIME):
 ev=s['ev'];tt=np.minimum(t,tr);modal=np.expm1(ev[:,None]*tt)/(ev[:,None]*tr)
 tail=t>tr;modal[:,tail]=np.exp(ev[:,None]*(t[tail]-tr))*np.expm1(ev[:,None]*tr)/(ev[:,None]*tr)
 y=(s['steady'][:,None]*np.minimum(t/tr,1)+s['mix']@modal).real
 ref=np.minimum(t/tr,1);err=np.max(np.abs(y-ref),axis=0);bad=np.flatnonzero(err>.01)
 settle=t[bad[-1]+1] if len(bad) and bad[-1]+1<len(t) else (0. if not len(bad) else math.inf)
 row={'source_ramp_ns':tr*1e9,'load_dc_ratio':float(s['steady'][1]),'receiver_at_sample_ratio':float(y[1,np.searchsorted(t,1.6e-6)]),'one_percent_settle_ns':float(settle*1e9),'tap_peak_error_fraction':float(np.max(abs(y[0]-ref))),'receiver_min_ratio':float(y[1].min()),'receiver_max_ratio':float(y[1].max()),'crossings_at0p9V':int(np.count_nonzero(np.diff(y[1]>.6))),'crossings_at0p5V':int(np.count_nonzero(np.diff(y[1]>(1/3))))}
 return y,row

def response(geo,z,td,rs,cextra,cpin,tr,n=16,t=TIME):
 s=system(geo,z,td,rs,cextra,cpin,n);y,row=evaluate(s,tr,t);return t,y,s['steady'],row,s

def main():
 assert (O/'CAN22-02_REDLINE_BEFORE_CORRECTION.json').exists()
 start=time.monotonic();geos=geometries();rows=[];max_dc=0.;max_r=0.;worst={};families=0
 for geo,z,td,rs,extra in itertools.product(geos,[25.,55.,100.,160.,200.],[1e-9,3.1e-9],[45.,60.,75.],[7.5e-12,32.5e-12]):
  s=system(geo,z,td,rs,extra);max_dc=max(max_dc,s['nodal_DC_error']);max_r=max(max_r,abs(s['series_R_total']-(WIRE_LOOP_R+BOARD_R)))
  sec=sections(geo,z,td)
  identity={'geometry_id':geo['id'],'twisted_Z_ohm':z,'twisted_delay_full0p30m_ns':td*1e9,'bus_thevenin_ohm':rs,'board_connector_ESD_extra_pF':extra*1e12,'receiver_pF':20.,'cable_C_pF':sum(q['C_F'] for q in sec)*1e12,'cable_L_nH':sum(q['L_H'] for q in sec)*1e9,'cable_delay_ns':sum(q['delay_s'] for q in sec)*1e9,'Molex_Z_ohm':geo['molex']['Z_ohm'],'ASC_Z_ohm':geo['ASC']['Z_ohm'],'cable_cells':s['cable_cells']}
  for tr in [2e-9,20e-9,100e-9]:
   y,r=evaluate(s,tr);r={**identity,**r};rows.append(r)
   for metric in ['one_percent_settle_ns','tap_peak_error_fraction','receiver_max_ratio']:
    if metric not in worst or r[metric]>worst[metric]['row'][metric]:worst[metric]={'row':r,'args':(geo,z,td,rs,extra,20e-12,tr),'wave':y.copy()}
  families+=1
  if families%300==0:
   print(json.dumps({'families':families,'cases':len(rows),'elapsed_s':time.monotonic()-start,'max_settle_ns':worst['one_percent_settle_ns']['row']['one_percent_settle_ns']}),flush=True)
 refinement=[];refined_waves={};fine=np.unique(np.r_[np.arange(0,20e-9,.0125e-9),np.arange(20e-9,200e-9,.05e-9),np.arange(200e-9,2e-6,1e-9),1.6e-6])
 for metric,wc in worst.items():
  args=wc['args'];t,y,dc,r,s=response(*args,n=16);_,y32,dc32,r32,s32=response(*args,n=32);tf,yf,_,rf,_=response(*args,n=32,t=fine)
  # Same n32 poles evaluated on a finer time grid separates time sampling from spatial-cell refinement.
  dy_time=max(abs(r32['tap_peak_error_fraction']-rf['tap_peak_error_fraction']),abs(r32['receiver_max_ratio']-rf['receiver_max_ratio']))*1.5
  refinement.append({'metric':metric,'case':wc['row'],'cells16_to32_max_difference_V_at1p5Vdrive':float(np.max(abs(y-y32))*1.5),'cells32_one_percent_settle_ns':r32['one_percent_settle_ns'],'time_grid_refined_one_percent_settle_ns':rf['one_percent_settle_ns'],'time_grid_refined_peak_change_V_at1p5Vdrive':dy_time,'dc_identity_error':s32['nodal_DC_error'],'base_cells_by_section':s['cell_table'],'refined_cells_by_section':s32['cell_table'],'refined_receiver_peak_V':rf['receiver_max_ratio']*1.5,'refined_tap_disturbance_fraction':rf['tap_peak_error_fraction']})
  refined_waves[metric]=(tf,yf)
 summary={'cases':len(rows),'unique_exit_geometries':len(geos),'passive_ladder_families':families,'receiver_min_sample_V_for1p5V_bus':min(x['receiver_at_sample_ratio'] for x in rows)*1.5,'max_one_percent_settle_ns':max(x['one_percent_settle_ns'] for x in rows),'max_transient_tap_disturbance_fraction':max(x['tap_peak_error_fraction'] for x in rows),'worst_receiver_peak_V_for1p5V_bus':max(x['receiver_max_ratio'] for x in rows)*1.5,'steady_min_ratio':min(x['load_dc_ratio'] for x in rows),'bit_time_ns':2000,'example_sample_ns':1600,'multiple0p9V_crossing_cases':sum(x['crossings_at0p9V']!=1 for x in rows),'multiple0p5V_crossing_cases':sum(x['crossings_at0p5V']!=1 for x in rows),'maximum_DC_identity_error':max_dc,'maximum_total_series_R_partition_error_ohm':max_r,'maximum_Molex_exit_Z_ohm':max(q['molex']['Z_ohm'] for q in geos),'maximum_total_conductor_length_m':WIRE_LENGTH,'wire_loop_R_ohm':WIRE_LOOP_R,'board_contact_R_ohm':BOARD_R,'elapsed_s':time.monotonic()-start}
 sources=[W/'baseline/GR86_CCA_RevB/evidence/manufacturer/tcan3403_q1.pdf',W/'baseline/GR86_CCA_RevB/evidence/manufacturer/esd2can24_q1.pdf',W/'analyses/harness_wire/HARNESS_WIRE_DELTA.json',W/'analyses/harness_wire/FREE_EXIT_SECTION_ENVELOPE.json']
 result={'status':'EXECUTED_CONDITIONAL_SECTIONAL_CAN_BRANCH_MODEL','redline':'CAN22-02','source_binding':{str(p.relative_to(W)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},'model_script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'summary':summary,'refinement':{'cases':refinement,'maximum_DC_identity_error':max_dc,'maximum_R_partition_error_ohm':max_r},'worst_settle_case':worst['one_percent_settle_ns']['row'],'exit_geometries':geos,'cases':rows,'limits':['Molex free exit25.4–30mm and ASC free exit0–15mm are separate line sections. ASC zero is a limiting model endpoint, not a manufacturing instruction.','Both free sections independently use3–4mm spacing and effectiveepsilon1/8 endpoints; the same exact wire-envelope.78/.80mm applies to both.4mm andepsilon8 are engineering allocations, not manufacturer maxima.','Twisted section retains the predecessor independent Z25/55/100/160/200ohm and full0.30m delay1/3.1ns sensitivities, scaled by its remaining conductor length. No extra helix length is added to the maximum0.30m conductor cut length.','Entire wire-loop resistance0.03723ohm includes both0.30m conductors at80C; the exact source estimate is0.037222296ohm. It is not a per-conductor value. Resistance is partitioned by conductor length; board/contact0.025ohm remains separate.','TCAN CID20pF maximum is the complete differential receiver load; CI40pF is not double-counted. RID25kohm minimum applies normal supplied operation.','D3015pF/channel maximum at25C becomes2.5pF differential; extra7.5/32.5pF includes explicit PCB/connector/hot-capacitance allocation, not a supplier guarantee. Board line remains0.1ns/80ohm.','Source45/60/75ohm,1.5V minimum dominant level and2/20/100ns ramps are representative installed-bus allocations. Example1.6us sampling at500kbit/s does not describe every ECU timing phase.','This finite rising-transition model also describes falling transitions by linear superposition only within its linear normal-operation assumptions. Multiple real traffic edges, arbitrary shorter harnesses, common mode, powered-off states and die nonlinearities are not proved.','Transient tap disturbance and possible threshold recrossings are reported explicitly rather than hidden by late-sample settling.','Sectional cell/time refinements assess numerical sensitivity, not a full transmission-line/IBIS or as-built qualification. Dielectric conductance is set to zero; skin/proximity loss, radiation and mode conversion are excluded.','The349 preserved PCB CAN objects are unchanged. Only harness model geometry is corrected; final result hash must be rebound by bind_final_source.py.']}
 (O/'COMPLETE_CAN_BRANCH_RESULTS.json').write_text(json.dumps(result,indent=2)+'\n')
 waves={}
 for key,(t,y) in refined_waves.items():waves[key+'_time_s']=t;waves[key+'_tap']=y[0];waves[key+'_receiver']=y[1]
 np.savez(O/'WORST_BRANCH_WAVEFORM.npz',**waves)
 print(json.dumps({'summary':summary,'refinement':refinement}),flush=True)
if __name__=='__main__':main()
