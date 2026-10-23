"""Extract directly auditable control and observation evidence from frozen runs."""
import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('matrix', HERE / 'analyze.py')
matrix = importlib.util.module_from_spec(spec)
spec.loader.exec_module(matrix)


def main():
    """Distinguish first motion, first prediction, scan freshness and object loss."""
    details = []
    for r in json.loads((HERE / 'analysis.json').read_text()):
        path = Path(r['directory'])
        status = matrix.records(path/'launch.log', '[SCAN_STATUS]')
        gt = matrix.prior.rows(path/'trial/ground_truth.jsonl')
        source = None
        if r['first_prediction']:
            seq = r['first_prediction']['seq']
            s = next(s for s in status if s['seq'] == seq)
            source = dict(source_elapsed=s['stamp']-r['epoch'],
                          query_elapsed=s['now']-r['epoch'], age=s['now']-s['stamp'],
                          line=s['line'])
        risk_end = r['first_recovering']['t'] if r['first_recovering'] else r['epoch']+r['duration']
        stopped = [g for g in gt if r['first_linear_stop'] and
                   r['first_linear_stop']['t'] <= g['t'] <= risk_end]
        waits = [(a['elapsed'], b['elapsed']) for a, b in
                 zip(r['transitions'], r['transitions'][1:]) if a['state'] == 'wait']
        details.append(dict(name=r['name'], prediction_scan=source, wait_intervals=waits,
                            stop_to_recover=(risk_end-r['first_linear_stop']['t'])
                            if r['first_linear_stop'] else None,
                            stop_to_recover_max_abs_v=max((abs(g['vx']) for g in stopped), default=None),
                            stop_to_recover_max_abs_w=max((abs(g['wz']) for g in stopped), default=None)))
    (HERE/'diagnostic_details.json').write_text(json.dumps(details, indent=2)+'\n')
    for d in details:
        print(d)


if __name__ == '__main__':
    main()
