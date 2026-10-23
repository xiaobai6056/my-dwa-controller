"""Create append-only raw/binary archive and Windows-readable delivery copy."""
import hashlib
import json
from pathlib import Path
import tarfile
import zipfile
HERE=Path(__file__).resolve().parent
archive=HERE/'outputs/raw_and_build_evidence.tar.gz'
with tarfile.open(archive,'x:gz') as tf:
    tf.add(HERE/'runs',arcname='runs')
    tf.add(HERE/'evidence',arcname='evidence',filter=lambda info:None if '__pycache__' in info.name else info)
    tf.add(HERE/'sequence_first.json',arcname='sequence_first.json')
with tarfile.open(archive,'r:gz') as tf:
    names=tf.getnames()
    assert sum(n.endswith('/trial/summary.json') for n in names)==6
    assert sum(n.endswith('/manifest.json') for n in names)==6
result=dict(archive=archive.name,sha256=hashlib.sha256(archive.read_bytes()).hexdigest(),bytes=archive.stat().st_size,members=len(names))
(HERE/'outputs/archive_manifest.json').write_text(json.dumps(result,indent=2)+'\n')
zipdest=Path('/path/to/local-experiment-inputs/release_comparison_20261020.zip')
with zipfile.ZipFile(zipdest,'x',zipfile.ZIP_DEFLATED) as zf:
    for p in HERE.rglob('*'):
        if p.is_file() and not any(x in p.relative_to(HERE).parts for x in ('runs','__pycache__')):
            zf.write(p,p.relative_to(HERE))
print(json.dumps(result,indent=2))
print(zipdest)