"""Dispatch exactly one run through the existing V2 supervisor, without changing it."""
import argparse
import importlib.util
import json
from pathlib import Path
import hashlib
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def sha(path):
    """Hash the selected frozen artifact."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    """Refuse version drift or invalid admission; never retry a run."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=['D01_t2', 'D01_t4', 'D01_t6'], required=True)
    parser.add_argument('--tag', required=True)
    parser.add_argument('--domain', type=int, default=108)
    args = parser.parse_args()
    freeze = json.loads((HERE / 'freeze.json').read_text())
    assert Path(args.tag).name == args.tag and args.tag not in ('.', '..')
    for name, expected in freeze['source_sha256'].items():
        assert sha(ROOT / name) == expected, name
    assert sha(Path(freeze['plugin'])) == freeze['plugin_sha256']
    assert sha(ROOT / 'config/nav2_params.yaml') == freeze['official_params_sha256']
    for name, expected in freeze['script_sha256'].items():
        assert sha(ROOT / name) == expected, name
    spec = importlib.util.spec_from_file_location(
        'unchanged_v2_supervisor', ROOT / 'benchmark/scan_emergency_20261022/run_round.py')
    supervisor = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(supervisor)
    supervisor.HERE = HERE  # Only relocate newly produced artifacts.
    ok = supervisor.run(args.case, args.tag, args.domain)
    output = HERE / 'runs' / f'{args.tag}_{args.case}'
    manifest = json.loads((output / 'manifest.json').read_text())
    summary = json.loads((output / 'trial/summary.json').read_text())
    assert ok, manifest
    assert summary.get('valid'), summary
    assert summary.get('action_accepted') and summary.get('action_error') is None, summary
    assert summary.get('selected_plugin_sha256') == freeze['plugin_sha256'], summary
    assert manifest.get('runtime_mapping_verified'), manifest
    assert (output / 'bag/metadata.yaml').exists()
    assert manifest['source_sha256'] == freeze['source_sha256']
    print(json.dumps(dict(admitted=True, output=str(output),
                          result=summary.get('action_result'), stop_reason=summary.get('stop_reason'))))
    return 0


if __name__ == '__main__':
    sys.exit(main())
