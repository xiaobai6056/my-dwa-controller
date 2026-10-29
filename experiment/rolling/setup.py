"""Prepare an independent, offline-only rolling experiment."""
import pathlib, subprocess, json, hashlib, shutil, xml.etree.ElementTree as ET
base = pathlib.Path('/home/user/ros2_experiments/simplified_bypass_20261029')
root = base.parent/'rolling_bypass_20261029'
root.mkdir(exist_ok=False)
(root/'logs').mkdir();(root/'results').mkdir()
formal = pathlib.Path('/home/user/ros2_ws/src/my_dwa_controller')
def git(p,*a): return subprocess.check_output(['git','-C',str(p),*a],text=True).strip()
assert git(formal,'rev-parse','main') == 'f7b6a45e4d858cee15e0fce22a1bd9b90e0c8446'
protected = {}
for directory in [formal, pathlib.Path('/home/user/ros2_ws/install/my_dwa_controller'),base]:
    for p in directory.rglob('*'):
        if p.is_file(): protected[str(p)] = hashlib.sha256(p.read_bytes()).hexdigest()
(root/'protected_before.json').write_text(json.dumps(protected,indent=2))
subprocess.run(['git','clone','--no-hardlinks',str(base/'source'),str(root/'source')],check=True)
git(root/'source','switch','-c','codex/rolling-bypass-analysis-20261029','8285416c604cda2d8f16eba8745010387691bf1c')
for k in ['user.name','user.email']: git(root/'source','config','--local',k,git(formal,'config','--get',k))
old_inodes={(p.stat().st_dev,p.stat().st_ino) for p in (base/'source').rglob('*') if p.is_file()}
shared=[str(p) for p in (root/'source').rglob('*') if p.is_file() and (p.stat().st_dev,p.stat().st_ino) in old_inodes]
assert not shared
(root/'isolation.json').write_text(json.dumps({'parent':git(base/'source','rev-parse','HEAD'),'shared_inodes':shared,'protected_files':len(protected)},indent=2))
history = json.loads((base/'inputs.json').read_text())
run = pathlib.Path(history['run']); configs=run/'trial/configs'
scene=json.loads((run/'admission.json').read_text())['effective_scene']
gt=[json.loads(s) for s in (run/'trial/ground_truth.jsonl').read_text().splitlines()]
epoch=json.loads((run/'trial/dynamic_events.jsonl').read_text().splitlines()[0])['t']
stopped=[g for g in gt if abs(g['t']-epoch-4.0)<.001 and abs(g['vx'])<.01 and abs(g['wz'])<.05][-1]
row=min(history['rows'],key=lambda r:abs(r['decision']['now_ns']/1e9-epoch-4.0))
walls=[]
for model in ET.parse(configs/'world_world.sdf').getroot().findall('world/model'):
    if model.attrib['name']=='dynamic_obstacle': continue
    pose=list(map(float,model.findtext('pose','0 0 0 0 0 0').split()))
    assert abs(pose[5])<1e-10
    for box in model.findall('link/collision/geometry/box'):
        size=list(map(float,box.findtext('size').split()))
        walls.append([pose[0]-size[0]/2,pose[1]-size[1]/2,pose[0]+size[0]/2,pose[1]+size[1]/2])
input={'scene':scene,'epoch':epoch,'initial':stopped['robot'],'initial_velocity':[0,0],
       'initial_source_time':stopped['t'],'path':[p[:2] for p in row['plan']['poses']],
       'path_frame_assumption':'saved map reference treated in scene world coordinates; no AMCL drift replay',
       'walls':walls,'start_times':[4,5,6,7,8],'historical_release':8.8,
       'history_run':str(run),'observation':'complete scene geometry, NOT historical Scan',
       'params_file':str(configs/'params_params_B.yaml')}
assert scene['keyframes'][1][0]==2.0
assert len(walls)>0
(root/'input.json').write_text(json.dumps(input,indent=2))
shutil.copy2(configs/'params_params_B.yaml',root/'params_reference.yaml')
shutil.copy2(configs/'world_world.sdf',root/'world_reference.sdf')
print('root',root,'walls',len(walls),'initial',stopped['robot'],'path points',len(input['path']))
