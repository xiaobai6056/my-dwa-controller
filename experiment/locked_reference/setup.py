"""Isolate the reference-guidance experiment and protect all prior evidence."""
import pathlib,subprocess,json,hashlib,shutil
base=pathlib.Path('/home/user/ros2_experiments/score_ablation_20261029')
root=base.parent/'locked_reference_20261029';root.mkdir(exist_ok=False)
for n in ['logs','results','params']:(root/n).mkdir()
formal=pathlib.Path('/home/user/ros2_ws/src/my_dwa_controller')
def git(p,*a):return subprocess.check_output(['git','-C',str(p),*a],text=True).strip()
assert git(formal,'rev-parse','main')=='f7b6a45e4d858cee15e0fce22a1bd9b90e0c8446'
protected={}
for folder in [base,base.parent/'rolling_bypass_20261029',base.parent/'rolling_bypass_diagnosis_20261029',
               formal,pathlib.Path('/home/user/ros2_ws/install/my_dwa_controller'),
               base.parent/'simplified_bypass_20261029/baseline_install']:
    for p in folder.rglob('*'):
        if p.is_file():protected[str(p)]=hashlib.sha256(p.read_bytes()).hexdigest()
(root/'protected_before.json').write_text(json.dumps(protected,indent=2))
subprocess.run(['git','clone','--no-hardlinks',str(base/'source'),str(root/'source')],check=True)
git(root/'source','switch','-c','codex/locked-reference-20261029','a072e771125a6482fc8801cc75b460fa4ceffbe5')
for k in ['user.name','user.email']:git(root/'source','config','--local',k,git(formal,'config','--get',k))
inodes={(p.stat().st_dev,p.stat().st_ino) for p in (base/'source').rglob('*') if p.is_file()}
shared=[str(p) for p in (root/'source').rglob('*') if p.is_file() and (p.stat().st_dev,p.stat().st_ino) in inodes]
assert not shared
(root/'isolation.json').write_text(json.dumps({'parent':git(base/'source','rev-parse','HEAD'),'shared_inodes':shared,'protected_files':len(protected)},indent=2))
for n in ['input.json','params_reference.yaml','world_reference.sdf']:shutil.copy2(base/n,root/n)
params={'extra_clearance':.10,'forward_beyond_exit':.10,'freshness_ns':350000000,'required_scans':4,
 'candidate_support':'admitted original DWA candidate reduces target heading error or moves rearward; zero command does not count',
 'certificate':'two axis-aligned-in-frozen-basis segments, diagnostic connectivity only; never supplied to DWA',
 'weights':{'heading':.05,'path':.2,'goal':.6,'speed':.1,'obstacle':.1},'end_time':16.,
 'start':4.,'observation':'5Hz complete scene envelope and exact current-pose 360-beam rays; not real Scan/Tracker evidence',
 'initial_stop_latch':True,'crossing_yield_release':'unchanged original code with synthetic current rays, no scripted release time',
 'heading':'atan2(locked_target.y-current_robot.y,locked_target.x-current_robot.x), compare to nominal endpoint yaw / pi',
 'screening':{'rearward_displacement_before_88':.05,'sustained_seconds':.2,'pass_gain_seconds':.2}}
(root/'params/design.json').write_text(json.dumps(params,indent=2))
print(root,len(protected))
