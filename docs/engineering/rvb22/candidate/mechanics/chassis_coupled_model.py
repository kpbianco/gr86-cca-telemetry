#!/usr/bin/env python3
from thermal_network import *
import itertools
rows=[]
for P,e,p,feed,Tland in itertools.product([2.214,4.790,5.942],[.3,.6,.9],[70000,85000,101325],[50,100],[65,70]):rows.append({'Pboard_W':P,'epsilon':e,'pressure_Pa':p,'feed_R_each_K_W':feed,'Rinterface_each_K_W':8,'Ral_each_K_W':12,'Rlanding_shared_K_W':4,'chassis_reference_C':Tland,**solve_chassis(P,e,p,feed,8,12,4,Tland)})
r={'reconstructed_and_rerun':True,'model':'5nodes;wingrearandcarrierrearheatlossuncredited','interface':'two38x6mm3M8810patches;typical5.8Ccm2/W /2.28cm2=2.544K/W;8K/Wbudgetincludesmask/contact/agingallocation','thermal_neck_each_K_W':neck,'case_count':len(rows),'cases':rows,'checks':{'all108energy_balances':all(x['energy_residual_W']<1e-6 for x in rows),'positive_thermal_resistance':neck>0},'conditions':['Actualcontactretention,materialeffectivityandlandingtemperaturemustmeetthemodel.','Innerplaneneck/lateralfeedmustbenativelyverified.','10KlocalC206offsetmustbeboundedbyrippleheatandplanegradient.','Chassisreferenceisactualmetaltemperature,notairtemperature.','NoAlconvectionorwingbacksideheatlosscredited.']}
(D/'CHASSIS_COUPLED_MODEL.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({'checks':r['checks'],'case_count':len(rows),'full_load_worst':[x for x in rows if x['Pboard_W']==4.79 and x['epsilon']==.3 and x['pressure_Pa']==70000 and x['feed_R_each_K_W']==100]}))
