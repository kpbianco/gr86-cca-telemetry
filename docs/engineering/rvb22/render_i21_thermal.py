#!/usr/bin/env python3
"""Plot the actual saved thermal field and mesh sensitivity without interpolation credit."""
import argparse
import json
import sys
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('recovery', type=Path)
args = parser.parse_args()
w = args.recovery
sys.path.insert(0, str(w / 'runtime/local'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

d = w / 'analyses/thermal_i18/face_15.0'
rows = sorted(json.loads((d / 'RESULTS.json').read_text()), key=lambda r: r['mesh_mm'], reverse=True)
z = np.load(d / 'map_provisional_0.125_15.npz')
x, y, temp = z['x'], z['y'], z['T'][0]
h = .125
xs, ys = np.unique(x), np.unique(y)
field = np.full((len(ys), len(xs)), np.nan)
field[np.rint((y-ys[0])/h).astype(int), np.rint((x-xs[0])/h).astype(int)] = temp
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':10})
fig, (ax, bx) = plt.subplots(1, 2, figsize=(12.4, 5.3), gridspec_kw={'width_ratios':[1.25,1]})
mesh = ax.pcolormesh(np.r_[xs, xs[-1]+h], np.r_[ys, ys[-1]+h], field, cmap='inferno', vmin=65, vmax=150, rasterized=True, shading='flat')
ax.invert_yaxis(); ax.set_aspect('equal'); ax.set(xlabel='Board X (mm)',ylabel='Board Y (mm)',title='I20-equivalent board surface, 0.125 mm mesh')
for label,xy,xytext in [('C206',(82,-4),(56,-4)),('U201',(79,21.5),(67,36)),('U121',(47,26.5),(5,27))]:
    ax.annotate(label,xy=xy,xytext=xytext,arrowprops={'arrowstyle':'-','color':'#285367'},color='#18394a',fontsize=10,bbox={'facecolor':'white','edgecolor':'none','alpha':.85})
fig.colorbar(mesh,ax=ax,shrink=.8,label='Board surface temperature (°C)')
index = np.arange(len(rows))
for key, label, color in [('max_board_C','Maximum board region','#a33022'),('C206_board_region_max_C','C206 board region','#157889')]:
    vals = [r[key] for r in rows]
    bx.plot(index, vals, marker='o', linewidth=2, color=color, label=label)
    for i,v in enumerate(vals):bx.annotate(f'{v:.1f}°', (i,v),xytext=(0,9),textcoords='offset points',ha='center',color=color)
bx.set(xticks=index,xticklabels=[f"{r['mesh_mm']:g}" for r in rows],xlabel='Mesh cell size (mm)',ylabel='Temperature (°C)',ylim=(90,153),title='Finer mesh still changes the result')
bx.grid(axis='y',color='#d8e0e3');bx.spines[['top','right']].set_visible(False);bx.legend(loc='lower right',frameon=False)
fig.suptitle('Thermal closure remains open',fontsize=16,ha='left',x=.055,color='#18394a')
fig.text(.055,.015,'4.815 W · 65°C bulk air · 70°C cold landings · 70 kPa · 15 µm minimum via wall\nFinite board model; no package, die, shield or local-air temperature claim.',fontsize=9,color='#405565')
fig.tight_layout(rect=(0,.10,1,.92))
out=w/'current'
fig.savefig(out/'GR86_I21_Thermal_Refinement.png',dpi=180)
fig.savefig(out/'GR86_I21_Thermal_Refinement.svg')
print(out/'GR86_I21_Thermal_Refinement.png')
