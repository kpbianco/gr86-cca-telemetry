"""Pin electrical/manufacturing content while tolerating native-generated drawing UUIDs."""
from pathlib import Path
import hashlib
import json
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent / "support"))
import sexpdata as sx

SOURCE_HASH = "5b373f6033fdbd18f126f8ca054b619abada2568778c5a8fc303c4e756fb06c8"
REFERENCE_RAW = "11533ea91c3bc4c61dd7066e0001914a72bc90b27afb38d321c43b8feb90e3b1"
REFERENCE_CONTENT = "c8a95ea1f0b1a9e5075b57c921191d87f32fddd9e01818cad815e9ef91c5d045"
NON_ELECTRICAL_LAYERS = {"F.Fab", "B.Fab", "F.CrtYd", "B.CrtYd"}

def canonicalize(node):
    """Preserve all values, geometry, order, copper and pad UUIDs; omit only drawing IDs."""
    if not isinstance(node, list):
        return str(node) if isinstance(node, sx.Symbol) else node
    tag = str(node[0]) if node else ""
    layer = next((str(x[1]) for x in node[1:] if isinstance(x, list)
                  and len(x) > 1 and str(x[0]) == "layer"), None)
    omit_id = tag == "property" or (tag.startswith("fp_") and layer in NON_ELECTRICAL_LAYERS)
    return [canonicalize(x) for x in node if not (
        omit_id and isinstance(x, list) and x and str(x[0]) == "uuid")]

def content_hash(node):
    data = json.dumps(canonicalize(node), separators=(",", ":"), ensure_ascii=False).encode()
    return hashlib.sha256(data).hexdigest()

def verify_filled(path):
    data = Path(path).read_bytes()
    raw = hashlib.sha256(data).hexdigest()
    stable = content_hash(sx.loads(data.decode()))
    if stable != REFERENCE_CONTENT:
        raise ValueError("Native board content differs: explicit engineering rebind required")
    return {"raw_sha256": raw, "content_sha256": stable,
            "reference_raw_sha256": REFERENCE_RAW,
            "raw_equal_to_reference": raw == REFERENCE_RAW,
            "only_generated_non_electrical_ids_may_differ": True}

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcb", type=Path)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    result = verify_filled(args.pcb)
    text = json.dumps(result, indent=2) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text)
    print(text)
