"""Capture only relevant process identity and loaded controller library evidence."""
import hashlib
import json
from pathlib import Path
import sys

output = []
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():
        continue
    try:
        args = proc.joinpath('cmdline').read_bytes().replace(b'\0', b' ').decode()
        env = proc.joinpath('environ').read_bytes().split(b'\0')
        if b'ROS_DOMAIN_ID=77' not in env:
            continue
        executable = args.split()[0] if args.split() else ''
        if Path(executable).name not in ('controller_server', 'component_container_isolated',
                                         'gzserver', 'gzclient', 'rviz2'):
            continue
        mappings = proc.joinpath('maps').read_text().splitlines()
        libs = sorted({line.split()[-1] for line in mappings
                       if 'libmy_dwa_controller.so' in line})
        output.append(dict(pid=int(proc.name), command=args, libraries=[
            dict(path=p, sha256=hashlib.sha256(Path(p).read_bytes()).hexdigest())
            for p in libs]))
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        continue
destination = Path(__file__).parent / (sys.argv[1] if len(sys.argv) > 1
                                      else 'runtime_processes.json')
destination.write_text(json.dumps(output, indent=2) + '\n')
print(json.dumps(output, indent=2))
