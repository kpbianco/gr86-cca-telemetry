#!/usr/bin/env python3
"""Restore explicitly authorized Convergence01 UTF-8 source, never adopt PCB trials."""
from pathlib import Path, PurePosixPath
import hashlib, json, subprocess, tarfile, tempfile

BASE = '5d772f12cb7b7c3fdf40c792726c2e01c8025a73'
DICT_SHA = 'c5305fbaf06d7846719587e8add506ab66e95722f76c5e60d5c7e1264c87fdb3'
PAYLOAD_SHA = '27e65c6baa786aff560edc60f842d405d1b48ef4e52dcae53b934e4bcd5eccc1'
TAR_SHA = '599341959efea5b80a253cc13c90039d6b84aced8aef1ac0b9a58400d174cf25'
ROOT = Path(__file__).resolve().parents[5]
HERE = Path(__file__).resolve().parent
WORKFLOWS = {'.github/workflows/desktop-due-diligence.yml', '.github/workflows/rvb22-native.yml'}

def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT)

def digest(data):
    return hashlib.sha256(data).hexdigest()

def main():
    records = []
    with tempfile.TemporaryDirectory() as td:
        td = Path(td)
        reference = bytearray()
        old = {}
        for line in git('ls-tree', '-r', BASE).splitlines():
            meta, path = line.split(b'\t', 1)
            mode, kind, sha = meta.split()
            if kind != b'blob':
                continue
            data = git('cat-file', 'blob', sha.decode())
            reference.extend(path + b'\0' + str(len(data)).encode() + b'\0' + data)
            old[path.decode()] = data
        if digest(reference) != DICT_SHA:
            raise RuntimeError('Recovery dictionary differs from pinned baseline')
        (td / 'reference.dict').write_bytes(reference)
        chunks = sorted(HERE.glob('part*.bin'))
        if len(chunks) != 16:
            raise RuntimeError('Incomplete publication payload')
        payload = b''.join(p.read_bytes() for p in chunks)
        if len(payload) != 81857 or digest(payload) != PAYLOAD_SHA:
            raise RuntimeError('Publication payload hash mismatch')
        (td / 'payload.zst').write_bytes(payload)
        subprocess.run(['zstd', '-d', '--long=27', '--patch-from=' + str(td/'reference.dict'), str(td/'payload.zst'), '-o', str(td/'payload.tar')], check=True)
        if digest((td/'payload.tar').read_bytes()) != TAR_SHA:
            raise RuntimeError('Expanded source archive mismatch')
        pending = []
        with tarfile.open(td/'payload.tar') as archive:
            members = archive.getmembers()
            if len(members) != 55 or len({m.name for m in members}) != 55:
                raise RuntimeError('Unexpected source file census')
            for member in members:
                path = PurePosixPath(member.name)
                if not member.isfile() or path.is_absolute() or '..' in path.parts:
                    raise RuntimeError('Unsafe archive member')
                if member.name not in WORKFLOWS and not member.name.startswith('docs/engineering/rvb22/'):
                    raise RuntimeError('Unexpected publication path')
                if '/candidate/' in member.name:
                    raise RuntimeError('This checkpoint must not change adopted CAD or firmware')
                data = archive.extractfile(member).read()
                data.decode('utf-8')
                destination = ROOT / member.name
                current = destination.read_bytes() if destination.exists() else None
                if current not in (old.get(member.name), data):
                    raise RuntimeError('Intervening change must be reconciled: ' + member.name)
                records.append({'path': member.name, 'sha256': digest(data), 'size': len(data), 'disposition': 'connector_workflow_update_required' if member.name in WORKFLOWS else 'source_restored'})
                if member.name not in WORKFLOWS:
                    pending.append((destination, data))
        for destination, data in pending:
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(data)
    manifest = {'baseline': BASE, 'payload_sha256': PAYLOAD_SHA, 'source_archive_sha256': TAR_SHA, 'restored_source_files': len(pending), 'workflow_files_pending': 2, 'binary_map_archives': 'Retained in original Convergence01 working package; not part of this UTF-8 source payload', 'files': records, 'pcb_adopted': False, 'physical_tests': False, 'fabrication_release': False}
    (HERE / 'IMPORTED_SOURCE_MANIFEST.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps({'restored_source_files': len(pending), 'workflow_files_pending': 2, 'payload_verified': True}))

if __name__ == '__main__':
    main()
