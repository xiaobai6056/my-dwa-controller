#!/usr/bin/env python3
"""Immutable new local export with complete new attempts, never replacing old results."""
import argparse,hashlib,json,shutil,tarfile
from pathlib import Path
H=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('destination',type=Path);a=p.parse_args()
a.destination.mkdir(parents=True,exist_ok=False)
for name in ('REPORT_AVOIDABILITY.md','README.md'):
    shutil.copyfile(H/name,a.destination/name)
shutil.copytree(H/'outputs',a.destination/'outputs')
files=[f for f in H.rglob('*') if f.is_file() and '__pycache__' not in f.parts and 'probe_build' not in f.parts and f.suffix!='.pyc']
manifest={str(f.relative_to(H)):dict(bytes=f.stat().st_size,sha256=hashlib.sha256(f.read_bytes()).hexdigest()) for f in files}
(a.destination/'CONTENTS.json').write_text(json.dumps(manifest,indent=2)+'\n')
archive=a.destination/'avoidability_raw_and_tools.tar.gz'
with tarfile.open(archive,'w:gz') as tar:
    for f in files:tar.add(f,arcname='avoidability/'+str(f.relative_to(H)))
(a.destination/'archive_sha256.txt').write_text(hashlib.sha256(archive.read_bytes()).hexdigest()+'  '+archive.name+'\n')
print(json.dumps(dict(destination=str(a.destination),files=len(files),archive_bytes=archive.stat().st_size),indent=2))