#!/usr/bin/env python3
"""Coarse U201 backside-contact sensitivity screen; no package or release credit."""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import types

# The repository pins binary analysis wheels for the hosted Python. Preload a
# system Shapely when available so a newer developer Python can still execute
# this coarse screen; pyamg is not used below because the 2 mm system remains
# below the model's AMG threshold.
try:
    import shapely  # noqa: F401
except Exception:
    pass

D = Path(__file__).resolve().parent
W = D.parents[1]
RUNTIME = W / "runtime" / "local"
if RUNTIME.is_dir():
    sys.path.insert(0, str(RUNTIME))
try:
    import pyamg as _pyamg_probe  # noqa: F401
except Exception:
    sys.modules["pyamg"] = types.SimpleNamespace(__version__="coarse-screen-unused-stub")

from scipy.sparse.linalg import LinearOperator, cg  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--out", type=Path, required=True)
parser.add_argument(
    "--extra-r",
    dest="extra_R",
    action="append",
    type=float,
    help="Additional U201 board-back-to-landing R in K/W; may be repeated. Default: 5.",
)
args = parser.parse_args()
args.out.mkdir(parents=True, exist_ok=True)

source_path = W / "analyses" / "thermal_i18" / "model_fast.py"
source = source_path.read_text()
old_sig = "wing_boxes=((15,-7.6,53,-1.6),(15,43.6,53,49.6))):"
new_sig = "wing_boxes=((15,-7.6,53,-1.6),(15,43.6,53,49.6)), extra_contacts=()):"
old_loop = "for g,r in [(contact,contact_R),(box(*wing_boxes[0]),wing_R),(box(*wing_boxes[1]),wing_R)]:"
new_loop = "for g,r in [(contact,contact_R),(box(*wing_boxes[0]),wing_R),(box(*wing_boxes[1]),wing_R)] + [(box(*b),r) for b,r in extra_contacts]:"
assert source.count(old_sig) == 1, "thermal model signature changed; review patch"
assert source.count(old_loop) == 1, "thermal contact assembly changed; review patch"
patched = source.replace(old_sig, new_sig).replace(old_loop, new_loop)

tmp = source_path.with_name("_model_fast_i24_u201_screen_tmp.py")
tmp.write_text(patched)
try:
    spec = importlib.util.spec_from_file_location("model_fast_i24_u201_screen", tmp)
    model = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    spec.loader.exec_module(model)
finally:
    tmp.unlink(missing_ok=True)

# Deterministic iterative solve avoids a large sparse factorization for a
# deliberately coarse sensitivity screen. The original nonlinear residual
# gates remain active in model.solve().
def coarse_linear_solve(A, b):
    diagonal = A.diagonal()
    assert (diagonal > 0).all()
    preconditioner = LinearOperator(A.shape, matvec=lambda x: x / diagonal)
    x, info = cg(A, b, M=preconditioner, rtol=1e-8, atol=1e-10, maxiter=3000)
    assert info == 0, info
    return x


model.spsolve = coarse_linear_solve

paths_file = W / "current" / "mechanics" / "I24_trial" / "THERMAL_PATHS.json"
if paths_file.exists():
    paths = json.loads(paths_file.read_text())
    central_R = float(paths["central"]["model_total_R_K_W"])
    wing_R = float(paths["wing"]["model_total_R_K_W"])
    path_source = str(paths_file.relative_to(W))
else:
    # Allows the archived I22 recovery to reproduce the method. The PR branch
    # has THERMAL_PATHS.json and must not use this fallback.
    central_R, wing_R = 15.0, 5.0
    path_source = "I22 recovery fallback: I24 allocations supplied explicitly"

u201 = next((name, power, geom) for name, power, geom in model.heat if name == "U201")
u201_box = tuple(float(x) for x in u201[2].bounds)
extracted = model.extract()

common = dict(
    step=2.0,
    mode="provisional",
    plating_um=15.0,
    heat_scale=1.0,
    landing=70.0,
    ambient=65.0,
    save_map=False,
    contact_R=central_R,
    wing_R=wing_R,
)

cases = []
case_defs = [("baseline", None)] + [(f"u201_backside_R{r:g}", r) for r in (args.extra_R or [5.0])]
for name, extra_R in case_defs:
    extra = () if extra_R is None else ((u201_box, extra_R),)
    result = model.solve(extracted, extra_contacts=extra, **common)
    cases.append(
        {
            "case": name,
            "extra_U201_contact_R_K_W": extra_R,
            "max_board_C": result["max_board_C"],
            "U201_source_region_mean_C": result["source_region_mean_C"]["U201"],
            "U121_source_region_mean_C": result["source_region_mean_C"]["U121"],
            "U151_source_region_mean_C": result["source_region_mean_C"]["U151"],
            "heat_to_landing_W": result["heat_to_landing_W"],
            "heat_to_air_W": result["heat_to_air_W"],
            "energy_balance_residual_W": result["energy_balance_residual_W"],
            "max_nodal_residual_W": result["max_nodal_residual_W"],
            "nodes": result["nodes"],
        }
    )

report = {
    "status": "FEASIBILITY_ONLY_NOT_PACKAGE_OR_RELEASE_CREDIT",
    "method": "retained four-layer model with added bottom-layer U201 thermal-boundary sensitivity",
    "model_sha256": hashlib.sha256(source_path.read_bytes()).hexdigest(),
    "source_PCB_sha256": extracted[-1]["pcb_sha256"],
    "thermal_path_source": path_source,
    "mesh_mm": 2.0,
    "mode": "provisional",
    "plating_um": 15.0,
    "bulk_air_C": 65.0,
    "landing_C": 70.0,
    "total_heat_W": 4.815,
    "central_contact_R_K_W": central_R,
    "wing_contact_R_K_W_each": wing_R,
    "U201_source_box_mm": u201_box,
    "cases": cases,
    "interpretation": (
        "Screen only. Improvement establishes value of local extraction but does not establish mesh convergence, "
        "solder-mask/TIM/contact resistance, module-internal temperature, local air temperature, or release margin."
    ),
}

baseline = cases[0]
assert all(c["max_board_C"] < baseline["max_board_C"] for c in cases[1:])
assert max(abs(c["energy_balance_residual_W"]) for c in cases) < 1e-7
assert max(c["max_nodal_residual_W"] for c in cases) < 1e-7

out = args.out / "U201_LOCAL_CONTACT_SCREEN.json"
out.write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2), flush=True)
