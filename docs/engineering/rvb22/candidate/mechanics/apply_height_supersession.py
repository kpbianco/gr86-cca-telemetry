"""Bind the preserved placement inventory to the primary-source height addendum."""
from pathlib import Path
import hashlib
import json

D = Path(__file__).resolve().parent
P = D / 'FINAL_COMPONENT_ENVELOPES.json'
A = D.parent.parent / 'analyses/assembly_handling/PRIMARY_BODY_HEIGHT_ADDENDUM.json'
H = D / 'history_before_height_addendum/FINAL_COMPONENT_ENVELOPES.json'
current = json.loads(P.read_text())
addendum = json.loads(A.read_text())
assert current['source_PCB_sha256'] == addendum['PCB_sha256']
assert hashlib.sha256(H.read_bytes()).hexdigest() == addendum['mechanics_inventory_sha256']
original = json.loads(H.read_text())
current['raw_missing_primary_height_refs_before_addendum'] = original['missing_primary_height_refs']
current['missing_primary_height_refs'] = sorted(
    [row['reference'] for row in addendum['remaining_unknown']]
    + addendum['counts']['nonfitted_unbound_refs_excluded']
)
current['primary_height_supersession'] = {
    'authoritative_current_height_evidence': '../../analyses/assembly_handling/PRIMARY_BODY_HEIGHT_ADDENDUM.json',
    'raw_inventory_snapshot': 'history_before_height_addendum/FINAL_COMPONENT_ENVELOPES.json',
    'raw_inventory_sha256': hashlib.sha256(H.read_bytes()).hexdigest(),
    'counts': addendum['counts'],
    'remaining_unbound_fitted_references': [row['reference'] for row in addendum['remaining_unknown']],
    'row_height_scope': 'The rows retain the original position/courtyard/height extraction. Their height fields are superseded by the source-bound addendum; use its exact per-reference body, overall and stand-off distinctions.',
    'remaining_geometry_gate': 'Height coverage is not a populated native solid, mated connector, lead, service or installed dashboard collision pass. Eighteen missing model references remain.'
}
P.write_text(json.dumps(current, indent=2) + '\n')
print(json.dumps({'fitted_height_bounds': addendum['counts']['total_bound_fitted_refs'], 'fitted_references': addendum['counts']['fitted_refs'], 'remaining_unknown': current['primary_height_supersession']['remaining_unbound_fitted_references']}))
