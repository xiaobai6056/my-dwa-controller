"""Check actual compilation commands and freeze both installed artifacts."""
import hashlib
import json
from pathlib import Path
import shlex
import shutil
from settings import selection,WS
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    before=json.loads((HERE/'evidence/before.json').read_text())
    assert all(sha(ROOT/name)==digest for name,digest in before['source'].items())
    result={}
    command_sets={}
    for variant in ('unoptimized','release'):
        prefix,plugin=selection(variant)
        build=WS/'build_compile_comparison_20261020'/variant
        commands=json.loads((build/'compile_commands.json').read_text())
        business=[c for c in commands if str(ROOT/'src')+'/' in c['file']]
        command_sets[variant]={c['file']:shlex.split(c['command']) for c in business}
        if variant=='release':assert all('-O3' in c and '-DNDEBUG' in c for c in command_sets[variant].values())
        else:assert all(not any(x.startswith('-O') for x in c) for c in command_sets[variant].values())
        for item in ('compile_commands.json','CMakeCache.txt'):
            shutil.copyfile(build/item,HERE/f'evidence/{variant}_{item}')
        shutil.copyfile(plugin,HERE/f'evidence/{variant}_libmy_dwa_controller.so')
        result[variant]=dict(plugin=str(plugin),plugin_sha256=sha(plugin),build=str(build),install=str(prefix),business_compile_commands=business)
    stripped={name:[t for t in cmd if t not in ('-O3','-DNDEBUG')] for name,cmd in command_sets['release'].items()}
    assert stripped==command_sets['unoptimized'],'business compile commands differ beyond Release flags'
    result['only_business_command_difference']=['-O3','-DNDEBUG']
    probe=HERE.parent/'perf_build/libnavperf.so'
    result['probe_sha256']=sha(probe)
    shutil.copyfile(probe,HERE/'evidence/libnavperf.so')
    shutil.copyfile(HERE.parent/'perf_probe.cpp',HERE/'evidence/perf_probe.cpp')
    (HERE/'evidence/build_identity.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Same sources; only business compile command difference: -O3 -DNDEBUG')
if __name__=='__main__':main()