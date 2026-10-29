import pathlib,hashlib,json,subprocess,shutil,os
os.environ['GIT_OPTIONAL_LOCKS']='0'
base=pathlib.Path('/home/user/ros2_experiments/locked_reference_20261029')
root=base.parent/'reference_sweep_20261029'
local=pathlib.Path('/path/to/local-experiment-inputs/reference_sweep_20261029')
formal=pathlib.Path('/home/user/ros2_ws/src/my_dwa_controller')
def git(p,*a):return subprocess.check_output(['git','-C',str(p),*a],text=True).strip()
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
assert not root.exists()
assert git(base/'source','rev-parse','HEAD')=='b43493e11e977e0705c03b909a52dbcdd38e3a70'
assert git(formal,'rev-parse','main')=='f7b6a45e4d858cee15e0fce22a1bd9b90e0c8446'
for p in [pathlib.Path('/AGENTS.md'),pathlib.Path('/home/AGENTS.md'),pathlib.Path('/home/user/AGENTS.md'),root.parent/'AGENTS.md',base/'AGENTS.md']:
    if p.exists():print(str(p)+'\n'+p.read_text())
root.mkdir()
for n in ['logs','params','results','scripts']:(root/n).mkdir()
protected=json.loads((base/'protected_before.json').read_text())
for p in base.rglob('*'):
    if p.is_file():protected[str(p)]=sha(p)
(root/'protected_before.json').write_text(json.dumps(protected,indent=2))
subprocess.run(['git','clone','--no-hardlinks',str(base/'source'),str(root/'source')],check=True)
git(root/'source','switch','-c','codex/reference-sweep-20261029')
for k in ['user.name','user.email']:git(root/'source','config','--local',k,git(formal,'config','--get',k))
old={(p.stat().st_dev,p.stat().st_ino) for p in (base/'source').rglob('*') if p.is_file()}
shared=[str(p) for p in (root/'source').rglob('*') if p.is_file() and (p.stat().st_dev,p.stat().st_ino) in old]
assert not shared
(root/'isolation.json').write_text(json.dumps({'parent':git(base/'source','rev-parse','HEAD'),'shared_inodes':shared,'protected_files':len(protected)},indent=2))
for n in ['input.json','params_reference.yaml','world_reference.sdf']:shutil.copy2(base/n,root/n)
shutil.copy2(local/'PLAN.md',root/'PLAN.md')
plan=root/'source/experiment/reference_sweep';plan.mkdir()
shutil.copy2(root/'PLAN.md',plan/'PLAN.md')
git(root/'source','add','-f','experiment/reference_sweep/PLAN.md')
git(root/'source','commit','-m','docs: fix strong Heading and conservative reference margin experiment matrix')
print(json.dumps({'root':str(root),'protected_files':len(protected),'baseline_status':git(root/'source','status','--porcelain')}))
