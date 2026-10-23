"""Reuse the existing smoke runner, keeping this comparison in its own directory."""

import argparse
import importlib.util
from pathlib import Path

HERE = Path(__file__).resolve().parent


def main():
    """Select an installed binary and original scenes without modifying old results."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', type=Path, required=True)
    parser.add_argument('--tag', required=True)
    parser.add_argument('--cases', nargs='+', required=True)
    parser.add_argument('--domain', type=int, default=97)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location(
        'existing_smoke', HERE.parent / 'path_conflict_v01_20261022/run_smoke.py')
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    runner.HERE = HERE
    runner.PREFIX = args.prefix.resolve()
    runner.PLUGIN = runner.PREFIX / 'lib/libmy_dwa_controller.so'
    for case in args.cases:
        if not runner.run(case, args.tag, args.domain):
            return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
