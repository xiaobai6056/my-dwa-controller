"""Verify append-only diagnosis, acquisition integrity and exported evidence."""
import ast
import hashlib
import json
from pathlib import Path
import sqlite3
import subprocess

from scoring import admission_errors

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]


def main():
    before=json.loads((HERE/'evidence/before.json').read_text())
    changed=[];missing=[]
    for filename,expected in before['files'].items():
        p=Path(filename)
        if not p.is_file():missing.append(filename)
        elif hashlib.sha256(p.read_bytes()).hexdigest()!=expected:changed.append(filename)
    head=subprocess.check_output(['git','--no-optional-locks','rev-parse','HEAD'],cwd=ROOT).decode().strip()
    diff=subprocess.check_output(['git','--no-optional-locks','diff','--binary'],cwd=ROOT).decode()
    status=subprocess.check_output(['git','--no-optional-locks','status','--short'],cwd=ROOT).decode()
    acquisition=[]
    for run in sorted((HERE/'runs').iterdir()):
        if not run.is_dir():continue
        manifest=json.loads((run/'manifest.json').read_text())
        summary=json.loads((run/'trial/summary.json').read_text())
        errors=admission_errors(summary,manifest)
        if not (run/'bag/metadata.yaml').exists():errors.append('missing_bag_metadata')
        checks={}
        for db in (run/'bag').glob('*.db3'):
            con=sqlite3.connect(f'file:{db}?mode=ro',uri=True)
            checks[db.name]=con.execute('pragma quick_check').fetchone()[0];con.close()
        if not checks or any(v!='ok' for v in checks.values()):errors.append('sqlite_integrity')
        acquisition.append(dict(run=run.name,errors=errors,bag_sqlite_checks=checks,
            actual_loaded_plugin_sha256=manifest['plugin_sha256'],
            runner_reported_underlay_sha256=summary.get('original_underlay_mydwa_sha256',summary.get('mydwa_library_sha256')),
            runtime_mapping_verified=manifest.get('runtime_mapping_verified'),
            controller_parameters_match=manifest.get('controller_parameters_match_historical')))
    syntax=[]
    for path in HERE.glob('*.py'):ast.parse(path.read_text(),filename=str(path));syntax.append(path.name)
    scores=json.loads((HERE/'outputs/score_verification.json').read_text())
    maps=json.loads((HERE/'outputs/maps_summary.json').read_text())
    report=dict(protected_existing_files=len(before['files']),changed_existing_files=changed,
        missing_existing_files=missing,git_head_unchanged=head==before['git_head'],
        original_git_diff_unchanged=diff==before['git_diff'],original_git_status_unchanged=status==before['git_status'],
        acquisition=acquisition,syntax_parsed=syntax,score_verification=scores,
        map_runs=len(maps),map_count=sum(r['raw_maps'] for r in maps),
        map_rejections=sum(r['rejected_maps'] for r in maps))
    report['pass']=not changed and not missing and all(report[k] for k in
        ('git_head_unchanged','original_git_diff_unchanged','original_git_status_unchanged')) and all(
        not r['errors'] for r in acquisition) and scores['runs']==8 and scores['mismatches']==0 and report['map_rejections']==0
    (HERE/'evidence/final_audit.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    if not report['pass']:raise SystemExit(1)


if __name__=='__main__':main()
