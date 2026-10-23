"""Offline reconstruction of existing observations, never a controller/scorer."""
import math
import numpy as np


def rollout(initial, commands, sim_time=1.7, dt=.05):
    """Match TrajectoryPredictor's constant-command, forward-Euler time grid."""
    if not (math.isfinite(sim_time) and math.isfinite(dt) and sim_time > 0 and dt > 0):
        raise ValueError('positive finite prediction intervals required')
    commands=np.asarray(commands,dtype=float).reshape((-1,2))
    initial=np.asarray(initial,dtype=float)
    if initial.shape!=(3,) or not np.isfinite(initial).all() or not np.isfinite(commands).all():
        raise ValueError('finite SE2 pose and velocity pairs required')
    pose=np.repeat(initial[None,:],len(commands),axis=0)
    poses=[pose.copy()]
    horizon=np.longdouble(sim_time);interval=np.longdouble(dt)
    tolerance=8*np.finfo(float).eps*horizon
    elapsed=np.longdouble(0);index=1
    while elapsed<horizon:
        next_time=min(index*interval,horizon)
        if horizon-next_time<=tolerance:next_time=horizon
        step=float(next_time-elapsed)
        pose[:,0]+=commands[:,0]*np.cos(pose[:,2])*step
        pose[:,1]+=commands[:,0]*np.sin(pose[:,2])*step
        pose[:,2]+=commands[:,1]*step
        poses.append(pose.copy());elapsed=next_time;index+=1
    return np.stack(poses,axis=1)


def sample_statistics(poses, costmap, k=5):
    """Two Top-K descriptions: repeated pose samples and distinct map cells.

    Raw costs remain 0..255. Rejected candidates are descriptive only. No cost
    is reported as a complete trajectory mean if any point lies outside the map.
    """
    if not isinstance(k,int) or k<1:raise ValueError('k must be a positive integer')
    poses=np.asarray(poses,dtype=float)
    if poses.ndim!=3 or poses.shape[2]!=3:raise ValueError('poses must be [candidate,point,3]')
    width=int(costmap['width']);height=int(costmap['height']);res=float(costmap['resolution'])
    values=np.asarray(costmap['data'],dtype=np.int16)
    if width<=0 or height<=0 or len(values)!=width*height or res<=0 or not math.isfinite(res):
        raise ValueError('invalid map geometry')
    finite=np.isfinite(poses).all(axis=2)
    coords=np.zeros(poses.shape[:2]+(2,),dtype=np.int64)
    for axis in (0,1):
        a=(poses[:,:,axis]-costmap['origin'][axis])/res
        coords[:,:,axis]=np.floor(np.where(finite,a,-1)).astype(np.int64)
    ok=finite&(coords[:,:,0]>=0)&(coords[:,:,0]<width)&(coords[:,:,1]>=0)&(coords[:,:,1]<height)
    result=[]
    for i in range(len(poses)):
        ids=coords[i,ok[i],1]*width+coords[i,ok[i],0]
        unique=np.unique(ids);sample=values[ids];uc=values[unique]
        complete=bool(ok[i].all()) and len(sample)>0
        ns=min(k,len(sample));nu=min(k,len(uc))
        result.append(dict(sample_count=len(poses[i]),in_bounds_samples=len(sample),
            out_of_bounds=int((~ok[i]).sum()),complete_trajectory=complete,
            different_cells=len(unique),repeat_samples=len(sample)-len(unique),
            mean=float(sample.mean()) if complete else None,
            max=int(sample.max()) if complete else None,
            topk_sample_mean=float(np.sort(sample)[-ns:].mean()) if complete else None,
            topk_unique_mean=float(np.sort(uc)[-nu:].mean()) if complete else None,
            topk_k_samples=ns,topk_k_unique=nu,k_requested=k,
            threshold_cells=int((uc>=253).sum()),threshold_samples=int((sample>=253).sum()),
            collision_free=bool(complete and np.all(sample<253)),
            minimum_grid_boundary_distance_m=float(np.min(np.abs(
                (poses[i,ok[i],:2]-np.asarray(costmap['origin']))/res-
                np.rint((poses[i,ok[i],:2]-np.asarray(costmap['origin']))/res)))*res)
            if len(sample) else None))
    return result


def weighted_terms(raw, weights):
    if len(raw)!=5 or len(weights)!=5:raise ValueError('exactly five score terms required')
    return [float(a)*float(b) for a,b in zip(raw,weights)]


def selection_order(candidates, phase):
    """Describe the original Controller tie/terminal ordering, without commands."""
    available=[i for i,z in enumerate(candidates) if z['scores'] is not None]
    return sorted(available,key=lambda i:((candidates[i]['v'],) if phase=='slowing' else ())+
                  (candidates[i]['scores'][5],abs(candidates[i]['w']),i))


def admission_errors(summary, manifest=None, legacy=False):
    """Retain genuine navigation failures; exclude acquisition failures."""
    errors=[]
    if summary.get('valid') is not True:errors.append('invalid_trial')
    if manifest is None:
        errors.append('missing_manifest')
    else:
        if manifest.get('supervisor_error'):errors.append('supervisor_error')
        if manifest.get('interrupted'):errors.append('interrupted')
        if manifest.get('runtime_mapping_verified') is not True:errors.append('unverified_runtime_plugin')
        if manifest.get('cleanup_returncode')!=0:errors.append('cleanup_failed')
        if manifest.get('runner_returncode') not in (0,2):errors.append('runner_execution_failure')
        if not legacy and manifest.get('controller_parameters_match_historical') is not True:errors.append('parameter_mismatch')
        if manifest.get('recording_error') or (not legacy and manifest.get('bag_returncode')!=0):errors.append('recording_failed')
    return errors


def verify_observation(original, poses, stats, tolerance=1e-7):
    """Verify reconstruction against independently captured original returns."""
    errors=[]
    if max(abs(float(a)-float(b)) for a,b in zip(poses[-1],original['end']))>tolerance:
        errors.append('endpoint')
    if bool(original['free'])!=stats['collision_free']:errors.append('feasibility')
    if original['free']:
        if stats['mean'] is None or abs(stats['mean']-original['mean'])>tolerance:errors.append('mean')
        if stats['max']!=original['max']:errors.append('max')
    return errors
