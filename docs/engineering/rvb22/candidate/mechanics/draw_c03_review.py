#!/usr/bin/env python3
from mechanics_runtime import *
from pathlib import Path
import json
import matplotlib;matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as PatchPolygon,Rectangle,Circle
from carrier_slot_geometry import carrier_plan,anchor_centres,obround
D=Path(__file__).resolve().parent;G=json.loads((D/'THERMAL_WING_GEOMETRY.json').read_text());C=json.loads((D/'FINAL_COMPONENT_ENVELOPES.json').read_text())
fig,(ax,detail)=plt.subplots(1,2,figsize=(14,8),gridspec_kw={'width_ratios':[1.3,1]});fig.patch.set_facecolor('white')
ax.add_patch(Rectangle((-7,-15),73.5,73,facecolor='#ded6bf',edgecolor='#8a7c55',lw=1.4,label='Carrier FR4 (C03)'))
for x,y,w,h in [(-13,-12,8,11),(-13,42.5,8,12.5)]:ax.add_patch(Rectangle((x,y),w,h,facecolor='#ded6bf',edgecolor='#8a7c55'))
for x,y,w,h in [(0,2,59.5,37),(13.5,-9.5,41,9),(13.5,41.8,41,9)]:ax.add_patch(Rectangle((x,y),w,h,facecolor='white',edgecolor='#8a7c55',linestyle=':'))
#MECH22-31 localspineinsert and actualobroundslots.
ax.add_patch(Rectangle((-8,28),8,12,facecolor='#ded6bf',edgecolor='#8a7c55'))
ax.add_patch(Rectangle((58.5,30),8,12,facecolor='#ded6bf',edgecolor='#8a7c55'))
ax.add_patch(PatchPolygon(G['outline_mm'],closed=True,facecolor='#d9e9df',alpha=.55,edgecolor='#187045',lw=1.4,label='PCB outline'))
for x,y in G['mount_centres_mm']:
 ax.add_patch(Circle((x,y),2.2,facecolor='white',edgecolor='#187045'));ax.text(x,y,'M2',ha='center',va='center',fontsize=6)
for x,y in [(-9,-6),(-9,47.5),(57,-11.5),(57,54.5)]:
 ax.add_patch(Circle((x,y),1.7,facecolor='white',edgecolor='#555'));ax.text(x,y-3,'M3',ha='center',fontsize=7)
for x,y in anchor_centres:
 ax.add_patch(PatchPolygon(list(obround(x,y,3.9,1.6).exterior.coords),facecolor='#2757a5'))
ax.add_patch(Rectangle((66.6,11.8),5.4,17,facecolor='#e99739',edgecolor='#ac5e00',lw=1.8,label='Insulated thermal contact'))
ax.add_patch(Rectangle((51,4),15.5,31.8,fill=False,edgecolor='#777',linestyle='--',lw=1.4,label='Chassis landing below board'))
ax.add_patch(Rectangle((73.254766,-1),36,48,fill=False,edgecolor='#b92d3c',lw=1.4,linestyle=':',label='15 mm antenna envelope'))
ax.add_patch(Rectangle((88.254766,14),6,18,facecolor='#ecced3',edgecolor='#b92d3c',lw=1.2))
ax.text(91.25,23,'ESP\nante nna'.replace('ante nna','antenna'),ha='center',va='center',fontsize=8)
ax.set_xlim(-17,112);ax.set_ylim(61,-18);ax.set_aspect('equal');ax.set_xlabel('X (mm)');ax.set_ylabel('Y (mm)');ax.set_title('C03/T01 source geometry — plan view',loc='left',fontweight='bold');ax.grid(alpha=.15);ax.legend(loc='lower left',bbox_to_anchor=(0,1.07),ncol=2,fontsize=8,framealpha=.9)
for row in C['rows']:
 if row['ref'] in ['J401','R408','L406','R203','C524','R158','C206']:
  b=row['courtyard_bounds_mm'];x,y,x1,y1=b;detail.add_patch(Rectangle((x,y),x1-x,y1-y,fill=False,edgecolor='#526678',lw=1.2));detail.text((x+x1)/2,(y+y1)/2,row['ref'],ha='center',va='center',fontsize=8)
detail.add_patch(Rectangle((66.6,11.8),5.4,17,facecolor='#e99739',alpha=.7,edgecolor='#ac5e00',lw=1.8));detail.text(69.3,21,'T01\n91.8 mm²\n10 K/W\nallocated',ha='center',va='center',fontsize=10)
detail.annotate('0.50 mm to J401 court',xy=(66.9,11.55),xytext=(60,4),arrowprops={'arrowstyle':'->','color':'#333'},fontsize=8)
detail.annotate('0.32 mm to R408 court',xy=(66.44,18),xytext=(60,25),arrowprops={'arrowstyle':'->','color':'#333'},fontsize=8)
detail.text(60,38.5,'Contact: X66.6…72.0 / Y11.8…28.8 mm\nAntenna starts X88.254766 mm\nMinimum metal clearance with tolerances:15.455 mm',fontsize=9,linespacing=1.5)
detail.set_xlim(59,88);detail.set_ylim(41,2);detail.set_aspect('equal');detail.set_xlabel('X (mm)');detail.set_ylabel('Y (mm)');detail.set_title('Final PCB underside clearance screen',loc='left',fontweight='bold');detail.grid(alpha=.15)
fig.suptitle('RVB22 mechanical correction review',fontsize=18,fontweight='bold',x=.07,ha='left');fig.text(.07,.04,'Basic thermal assembly:108.455 ×73 ×36.50 mm. Native populated solids and actual-source thermal safety remain execution gates.\nPlan projections show source envelopes at different Z levels; overlapping projections alone are not 3D collisions.',fontsize=9,color='#444');fig.tight_layout(rect=[.03,.09,.98,.94]);fig.savefig(D/'C03_T01_REVIEW.png',dpi=180);fig.savefig(D/'C03_T01_REVIEW.svg');plt.close(fig)
