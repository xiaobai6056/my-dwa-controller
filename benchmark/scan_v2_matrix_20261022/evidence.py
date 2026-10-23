"""Build compact timelines and check hypotheses without changing frozen inputs."""
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import re

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
spec = importlib.util.spec_from_file_location(
    'matrix', HERE / 'analyze.py')
matrix = importlib.util.module_from_spec(spec)
spec.loader.exec_module(matrix)


def sha(path):
    """Hash an artifact without touching archived runs."""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    """Validate comparability and keep every state/evidence transition visible."""
    rows = json.loads((HERE/'analysis.json').read_text())
    baseline = rows[-1]
    reference = yaml.safe_load((ROOT/'config/nav2_params.yaml').read_text())
    reference['controller_server']['ros__parameters']['FollowPath']['path_conflict'] = {
        'enabled': True}
    reference['controller_server']['ros__parameters']['FollowPath']['scan_tracker'] = {
        'enabled': True}
    timeline, summary, findings = [], [], []
    for r in rows:
        directory = Path(r['directory'])
        params = yaml.safe_load((directory/'params_enabled.yaml').read_text())
        assert params == reference, r['name']
        m = json.loads((directory/'manifest.json').read_text())
        scene = json.loads((directory/'effective_scene.json').read_text())
        scene_old = json.loads((Path(baseline['directory'])/'effective_scene.json').read_text())
        comparison = dict(name=r['name'], same_plugin=r['plugin_sha256'] == baseline['plugin_sha256'],
                          same_source=r['source_sha256'] == baseline['source_sha256'],
                          same_parameter_file=r['parameters_sha256'] == baseline['parameters_sha256'],
                          same_runtime_parameters=r['runtime_parameters'] == baseline['runtime_parameters'],
                          same_scene=scene == scene_old,
                          metric='all runs including prior V2 recomputed by the same analyzer')
        events = []
        motion = r['first_accepted_motion']
        if motion:
            events.append(dict(event='first accepted actor motion', elapsed=motion['source_elapsed'],
                               time_kind='Scan source', query_elapsed=motion['query_elapsed'],
                               line=motion['line'], log_system_stamp=motion['log_system_stamp']))
        for state in r['transitions']:
            events.append(dict(event='state '+state['state'], elapsed=state['elapsed'],
                               time_kind='Controller ROS now', line=state['line'],
                               cap=state['cap'], available=state['available'], stop=state['stop'],
                               core=state['core'], aligned=state['aligned'], region=state['region'],
                               log_system_stamp=state['log_system_stamp']))
        for key in ('first_linear_stop', 'first_full_stop'):
            g = r[key]
            if g:
                events.append(dict(event=key, elapsed=g['elapsed'], time_kind='GT receipt /clock',
                                   robot=g['robot'], vx=g['vx'], wz=g['wz']))
        for key in ('first_local_clear', 'first_recovering'):
            e = r[key]
            if e:
                events.append(dict(event=key, elapsed=e['elapsed'], time_kind='Controller ROS now',
                                   line=e['line'], log_system_stamp=e['log_system_stamp']))
        for e in r['releases']:
            events.append(dict(event='release', elapsed=e['elapsed'], time_kind='Controller ROS now',
                               line=e['line'], log_system_stamp=e['log_system_stamp']))
        contact = r['first_contact']
        if contact:
            events.append(dict(event='physical contact', elapsed=contact['stamp']-r['epoch'],
                               time_kind='ContactsState source',
                               receipt_elapsed=contact['t']-r['epoch'], pair=contact['pair']))
        events.append(dict(event='navigation '+r['result'], elapsed=r['duration'],
                           time_kind='runner terminal /clock'))
        timeline.extend(dict(run=r['name'], **e) for e in sorted(events, key=lambda x:x['elapsed']))
        first_risk = r['first_risk']
        cycle = r['last_control_cumulative']
        risk_rows = matrix.records(directory/'launch.log', '[PATH_CONFLICT]')
        fresh_risk = [x for x in risk_rows if first_risk and
                      x['t'] >= first_risk['t'] and not x['fresh']]
        persistent_clear = []
        if r['first_recovering']:
            last_false = max((x['t'] for x in risk_rows
                              if x['t'] <= r['first_recovering']['t'] and not x['area_clear']),
                             default=r['epoch'])
            persistent_clear = [x for x in risk_rows if x['t'] > last_false and
                                x['t'] <= r['first_recovering']['t'] and x['area_clear']]
        summary.append(dict(run=r['name'], result=r['result'], stop_reason=r['stop_reason'],
                            duration=r['duration'], contact=r['physical_contact'],
                            min_gap_approx=r['min_gap_approx'],
                            first_motion_source=motion['source_elapsed'] if motion else None,
                            first_risk=first_risk['elapsed'] if first_risk else None,
                            linear_stop=r['first_linear_stop']['elapsed'] if r['first_linear_stop'] else None,
                            full_stop=r['first_full_stop']['elapsed'] if r['first_full_stop'] else None,
                            wait_records=r['wait_records'],
                            persistent_clear_start=persistent_clear[0]['t']-r['epoch'] if persistent_clear else None,
                            recover=r['first_recovering']['elapsed'] if r['first_recovering'] else None,
                            release=r['releases'][0]['elapsed'] if r['releases'] else None,
                            stale_active_risk=len(fresh_risk),
                            tf_failures=r['max_tf_failures'], scan_dropped=r['max_scan_dropped'],
                            unknown_actor=r['unknown_actor_samples'], max_track_age=r['max_track_age'],
                            calls=cycle['calls'], over50=cycle['over50'],
                            mean_ms=cycle['mean_ms'], max_ms=cycle['max_ms']))
        findings.append(dict(comparison=comparison,
                             clear_confirmation_sequences=[(x['t']-r['epoch'], x['seq'])
                                                           for x in persistent_clear],
                             first_wait_to_fallback=next((x for a,x in zip(r['transitions'],r['transitions'][1:])
                                                         if a['state']=='wait' and x['state']=='dwa_fallback'), None)))
    matrix.prior.write_csv(HERE/'timeline.csv', timeline)
    matrix.prior.write_csv(HERE/'summary.csv', summary)
    (HERE/'findings.json').write_text(json.dumps(findings, indent=2)+'\n')
    for row in summary:
        print(row)
    for row in findings:
        print(row['comparison'])


if __name__ == '__main__':
    main()
