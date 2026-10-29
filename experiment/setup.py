import pathlib, subprocess, json, hashlib
root = pathlib.Path('/home/user/ros2_experiments/simplified_bypass_20261029')
root.mkdir(exist_ok=False)
(root/'logs').mkdir()
formal = pathlib.Path('/home/user/ros2_ws/src/my_dwa_controller')
def git(*args, cwd=formal):
    return subprocess.check_output(['git', *args], cwd=cwd, text=True).strip()
assert git('rev-parse', 'main') == 'f7b6a45e4d858cee15e0fce22a1bd9b90e0c8446'
assert git('status', '--porcelain') == ''
protected = [formal, pathlib.Path('/home/user/ros2_ws/install/my_dwa_controller'), pathlib.Path('/home/user/ros2_experiments/crossing_yield_reserve020_20261029')]
hashes = {}
inodes = set()
for directory in protected:
    for p in directory.rglob('*'):
        if p.is_file():
            hashes[str(p)] = hashlib.sha256(p.read_bytes()).hexdigest()
            inodes.add((p.stat().st_dev, p.stat().st_ino))
(root/'protected_before.json').write_text(json.dumps(hashes, indent=2))
subprocess.run(['git', 'clone', '--no-hardlinks', str(formal), str(root/'source')], check=True)
git('switch', '-c', 'codex/simplified-bypass-20261029', 'f7b6a45e4d858cee15e0fce22a1bd9b90e0c8446', cwd=root/'source')
shared = [str(p) for p in (root/'source').rglob('*') if p.is_file() and (p.stat().st_dev, p.stat().st_ino) in inodes]
assert not shared
(root/'isolation.json').write_text(json.dumps({'baseline': git('rev-parse','main'), 'shared_inodes': shared, 'protected_files': len(hashes)}, indent=2))
print(root)
