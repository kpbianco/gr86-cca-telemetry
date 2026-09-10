#!/usr/bin/env python3
"""Verify restored I20 evidence against its manifest and the checked-out candidate."""
import argparse
import hashlib
import json
from pathlib import Path


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('recovery', type=Path)
    parser.add_argument('--repository', type=Path, default=Path(__file__).resolve().parent)
    args = parser.parse_args()
    w, repo = args.recovery, args.repository
    native = w / 'runtime/hosted/run22/extracted/native_I06_hosted'
    binding = json.loads((w / 'iterations/I20_controlled_handoff/NATIVE_BINDING.json').read_text())
    result = json.loads((native / 'RESULT.json').read_text())
    manifest = json.loads((native / 'OUTPUT_MANIFEST.json').read_text())
    checks = []

    def check(label, path, expected, size=None):
        actual = digest(path) if path.is_file() else None
        passed = actual == expected and (size is None or path.stat().st_size == size)
        checks.append({'item': label, 'sha256': actual, 'expected': expected, 'pass': passed})

    check('native artifact archive', w / 'runtime/hosted/run22/GR86_RVB22_I20_Native_Artifact.zip', binding['artifact_SHA256'])
    check('native output manifest', native / 'OUTPUT_MANIFEST.json', result['output_manifest']['sha256'])
    for name, entry in manifest['files'].items():
        check('output/' + name, native / name, entry['sha256'], entry['bytes'])
    for group in ['cad', 'firmware']:
        for name, expected in result['inputs'][group].items():
            check('repository/' + group + '/' + name, repo / 'candidate' / group / name, expected)
            source = w / 'iterations/I20_controlled_handoff/candidate_kicad' if group == 'cad' else w / 'lanes/firmware_oil/candidate'
            check('recovery/' + group + '/' + name, source / name, expected)
    summary = {
        'checkpoint': 'I21_RESTORED_I20_VERIFICATION',
        'date': '2026-09-10',
        'base_commit': binding['commit'],
        'source_PCB_sha256': binding['source_PCB_sha256'],
        'filled_PCB_sha256': binding['filled_PCB_sha256'],
        'native_run': binding['run'],
        'native_run_url': 'https://github.com/tranquilWorks/gr86-cca-telemetry/actions/runs/' + str(binding['run']),
        'verification_count': len(checks),
        'output_file_count': len(manifest['files']),
        'CAD_input_files': len(result['inputs']['cad']),
        'firmware_input_files': len(result['inputs']['firmware']),
        'failures': [r for r in checks if not r['pass']],
        'native_postconditions_pass': all(r['pass'] for r in result['output_postconditions'].values()),
        'native_finding_count': result['native_report_finding_count'],
        'fresh_native_execution_claimed': False,
        'meaning': 'Fresh digest verification of previously executed native evidence; source-bound copper/export reconstruction is separately rerun.',
        'checks': checks,
    }
    summary['status'] = 'PASS' if not summary['failures'] and summary['native_postconditions_pass'] and summary['native_finding_count'] == 0 else 'FAIL'
    out = w / 'current/I21_SOURCE_VERIFICATION.json'
    out.write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({k: v for k, v in summary.items() if k != 'checks'}, indent=2))
    if summary['status'] != 'PASS':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
