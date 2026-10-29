"""Create a no-hardlink experiment and preserve all prior evidence."""
import pathlib,subprocess,json,hashlib,shutil
base=pathlib.Path('/home/user/ros2_experiments/rolling_bypass_20261029')
root=base.parent/'score_ablation_20261029';root.mkdir(exist_ok=False)
for n in ['logs','results','params']:(root/n).mkdir()
formal=pathlib.Path('/home/user/ros2_ws/src/my_dwa_controller')
def git(p,*a):return subprocess.check_output(['git','-C',str(p),*a],text=True).strip()
assert git(formal,'rev-parse','main')=='f7b6a45e4d858cee15e0fce22a1bd9b90e0c8446'
protected={}
for folder in [base,base.parent/'rolling_bypass_diagnosis_20261029',formal,
               pathlib.Path('/home/user/ros2_ws/install/my_dwa_controller'),
               base.parent/'simplified_bypass_20261029/baseline_install']:
    for p in folder.rglob('*'):
        if p.is_file():protected[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
(root/'protected_before.json').write_text(json.dumps(protected,indent=2))
subprocess.run(['git','clone','--no-hardlinks',str(base/'source'),str(root/'source')],check=True)
git(root/'source','switch','-c','codex/score-ablation-20261029','5cd10fc0a0e00c9b81b9296a936ef362b7b331e8')
for key in ['user.name','user.email']:git(root/'source','config','--local',key,git(formal,'config','--get',key))
inodes={(p.stat().st_dev,p.stat().st_ino) for p in (base/'source').rglob('*') if p.is_file()}
shared=[str(p) for p in (root/'source').rglob('*') if p.is_file() and (p.stat().st_dev,p.stat().st_ino) in inodes]
assert not shared
(root/'isolation.json').write_text(json.dumps({'parent':git(base/'source','rev-parse','HEAD'),'shared_inodes':shared,'protected_files':len(protected)},indent=2))
for name in ['input.json','params_reference.yaml','world_reference.sdf']:shutil.copy2(base/name,root/name)
schemes={'A':[1,1,1,1,1],'B':[0,1,1,1,1],'C':[0,0,1,1,1],'D':[0,0,1,2,1],
         'E':[0,0,1,4,1],'F':[0,0,.5,2,1],'G':[0,0,0,2,1]}
original={'H':.05,'P':.2,'G':.6,'V':.1,'O':.1}
for scheme,factors in schemes.items():
    weights={k:original[k]*f for k,f in zip(original,factors)}
    (root/'params'/f'{scheme}.json').write_text(json.dumps({'scheme':scheme,'factors':dict(zip(original,factors)),
       'weights':weights,'normalize_weights':False,'heading_enabled':True,'end_time':14.0},indent=2))
print(root,'protected files',len(protected))
