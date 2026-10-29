import pathlib,subprocess,json,hashlib,shutil,os
os.environ['GIT_OPTIONAL_LOCKS']='0'
base=pathlib.Path('/home/user/ros2_experiments/reference_sweep_20261029');root=base.parent/'polyline_bypass_20261029'
local=pathlib.Path('/path/to/local-experiment-inputs/polyline_bypass_20261029')
assert not root.exists();root.mkdir()
for n in ['logs','results','scripts','params']:(root/n).mkdir()
def git(p,*a):return subprocess.check_output(['git','-C',str(p),*a],text=True).strip()
protected=json.loads((base/'protected_before.json').read_text())
for p in base.rglob('*'):
 if p.is_file():protected[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
(root/'protected_before.json').write_text(json.dumps(protected,indent=2))
subprocess.run(['git','clone','--no-hardlinks',str(base/'source'),str(root/'source')],check=True)
git(root/'source','switch','-c','codex/polyline-bypass-20261029')
for k in ['user.name','user.email']:git(root/'source','config','--local',k,git(base/'source','config','--get',k))
old={(p.stat().st_dev,p.stat().st_ino) for p in (base/'source').rglob('*') if p.is_file()}
assert not [p for p in (root/'source').rglob('*') if p.is_file() and (p.stat().st_dev,p.stat().st_ino) in old]
for n in ['input.json','world_reference.sdf','params_reference.yaml']:shutil.copy2(base/n,root/n)
shutil.copy2(local/'PLAN.md',root/'PLAN.md')
(root/'isolation.json').write_text(json.dumps({'parent':git(base/'source','rev-parse','HEAD'),'shared_inodes':0,'protected_files':len(protected)},indent=2))
shutil.copy2(base/'scripts/common.py',root/'scripts/common.py')
p=root/'scripts/common.py';p.write_text(p.read_text().replace('reference_sweep_20261029','polyline_bypass_20261029'))
shutil.copy2(base/'build.sh',root/'build.sh');p=root/'build.sh';p.write_text(p.read_text().replace('reference_sweep_20261029','polyline_bypass_20261029'))
print(root,len(protected))
