# Current checkpoint: convergence 01

See `analyses/convergence_01/README.md` relative to the rvb22 root and `current/CONVERGENCE_01_EFFECTIVITY.json`. The former 6.5/7 mm exposed-shoe whole-path selection is withdrawn. New native-copper trial refinement gives 111.533 C board maximum at 0.25 mm; no thermal/mechanical acceptance or new PCB fabrication release is claimed. Earlier numerical statements below retain their historical configuration and scope.

## Historical review

# I22 engineering review

Open original criteria decrease from **152 to 146**. The register now has **140 closed / 146 open / 4 not applicable**, preserving all 290 original questions, required-evidence cells and user inputs. The original user baseline remains 73/213/4. All 351 prior redlines plus six I22 findings are retained.

| Criterion closed | Evidence completing its original scope |
|---|---|
| LIB-02 | Exact manufacturer drawing-to-footprint overlays for all 12 IC/module references, independently transformed from native placement. All 151 signal lands and 12 negative controls pass. |
| LIB-08 | Exact exposed-pad net, copper, thermal-return and mask/paste review for U121/U151/U201/U301/U501, including the isolated TPS2660 RTN connection. |
| VIA-06 | Fabrication-tolerance via-current budget covers all 322 vias: 2 A per individual via, 15 µm minimum wall, 1.76 mm thickness and −0.08 mm finished-hole allowance; maximum 5.103 K self-rise against 10 K. |
| SI-12 | The original criterion permits justified analysis-only acceptance. Native RF ground-reference continuity and the retained 3,981,312-case board network support that limited disposition; external-system and supplier conditions remain separate. |
| GPS-01 | PA1616D top-view numbering, all 20 pad functions/nets and manual placement/pin-1 coordinates independently reconciled. |
| MECH-03 | Complete declared height/XY/fastener/foil tolerance stack: 1,154 populated and 776 mated checks, no intersections, 0.120 mm minimum modeled clearance. |

The exact Adafruit 851 adapter and 960 puck are now controlled inputs. The mated model no longer names the unadopted Amphenol candidate. Cable length is 150 ±3 mm to the SMA shoulder; maximum allocated OD is 2.0 mm, with separate 0.5 mm route uncertainty. The cable and plug metal remain 16.421 mm from the ESP32 antenna region. The height model now includes the previously omitted 0.51 mm capture freedom, bringing the added height stack to 1.71 mm. These are model and assembly-instruction corrections; no CAD or firmware revision is claimed.

The I20 CAD/target source remains exact: zero ERC, DRC, warnings, unconnected or parity findings in native run 34420381540; 153 fitted models; 151 automatic placements and F101/U401 manual exceptions. The 8 mm Compact TW logo from the supplied brand pack remains on F.SilkS. Existing power and RF finite bounds remain applicable. I21's 1,042 digest checks and full independent copper/export reconstruction are retained, with fresh I22 source/evidence identity checks recorded separately.

Remaining engineering work is material. The 0.125 mm thermal estimate is still 143.800°C maximum board region, with an 8.376 K last-refinement change. Adafruit 851's published 60°C ceiling does not cover the declared 65°C air case; passive cooling cannot remove that boundary conflict. Its J401 board region is estimated at 92.438–94.443°C, without asserting cable/connector temperature. All 54 digital adjacent-plane gaps remain explicit under GND-02. DFM-02 still requires exact hole/pin/hardware fit and drawing/DFM agreement; GPS-09/14 retain bias, plug and retention conditions; WCA-07 retains guaranteed optical performance; DRW-03 retains independent human review. Supplier and actual-unit/installation qualification are not replaced by calculation.

Primary package PDFs, source hashes, all twelve SVG/PNG overlays, revised mated STEP/route output, calculations, failure history and the current register are included in the recovery archive. The mated STEP contains connector/service envelope solids; the complete coax path and its tolerance envelope are documented in the route PNG/JSON. The plan and selected evidence are forwarded through PR #45. This remains a draft engineering candidate with thermal work and qualification open.
