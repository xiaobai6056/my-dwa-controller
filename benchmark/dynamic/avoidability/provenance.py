#!/usr/bin/env python3
import hashlib,json,shutil
from pathlib import Path
H=Path(__file__).resolve().parent
root=H.parents[2]
old=H.parent/'diagnostics/release_comparison/evidence'
identity=json.loads((old/'build_identity.json').read_text())
release=identity['release']
release['probe_source_sha256']=hashlib.sha256((H/'decision_probe.cpp').read_bytes()).hexdigest()
release['probe_sha256']=hashlib.sha256((H/'probe_build/libdecision.so').read_bytes()).hexdigest()
release['controller_rebuilt_this_task']=False
(H/'evidence/build_identity.json').write_text(json.dumps(release,indent=2)+'\n')
for src,dest in [(old/'release_compile_commands.json',H/'evidence/release_compile_commands.json'),(H/'probe_build/libdecision.so',H/'evidence/libdecision_used.so'),(Path('/opt/ros/humble/share/nav2_bringup/worlds/waffle.model'),H/'evidence/waffle.model')]:shutil.copyfile(src,dest)
for name in ('diagnostic_t2_linked','diagnostic_t4_linked'):
    m=json.loads((H/'runs'/name/'manifest.json').read_text());maps=(H/'runs'/name/'plugin_maps.txt').read_text()
    assert m['preload_sha256']==release['probe_sha256'] and m['probe_source_sha256']==release['probe_source_sha256']
    assert str(H/'probe_build/libdecision.so') in maps
(H/'evidence/provenance_verified.json').write_text(json.dumps(dict(controller_sha256=release['plugin_sha256'],probe_sha256=release['probe_sha256'],runtime_probe_and_plugin_maps_verified=True),indent=2)+'\n')