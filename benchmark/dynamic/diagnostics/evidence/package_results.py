"""Archive new diagnostic raw data without modifying V0.1 or overwriting archives."""
import hashlib
import json
from pathlib import Path
import tarfile
import zipfile
HERE=Path(__file__).resolve().parents[1]
ROOT=HERE.parents[2]
archive=HERE/'outputs/diagnostic_runs.tar.gz'
if archive.exists():raise RuntimeError('archive exists; choose a new destination rather than overwrite')
with tarfile.open(archive,'x:gz') as tf:
    tf.add(HERE/'runs',arcname='runs')
    tf.add(ROOT/'benchmark/results/dynamic/v02_headless_compat_001',arcname='headless_compat')
with tarfile.open(archive,'r:gz') as tf:
    names=tf.getnames()
    assert sum(n.endswith('/trial/summary.json') for n in names)==5
    assert 'headless_compat/results.json' in names
result=dict(archive=str(archive),sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),bytes=archive.stat().st_size,members=len(names))
(HERE/'outputs/archive_manifest.json').write_text(json.dumps(result,indent=2)+'\n')
zipdest=Path('/path/to/local-experiment-inputs/dynamic_benchmark_v0.2_20261020.zip')
with zipfile.ZipFile(zipdest,'x',zipfile.ZIP_DEFLATED) as zf:
    for p in HERE.rglob('*'):
        if p.is_file() and not any(x in p.relative_to(HERE).parts for x in ('runs','perf_build','__pycache__')):
            zf.write(p,p.relative_to(HERE))
print(json.dumps(result,indent=2))
print(zipdest)