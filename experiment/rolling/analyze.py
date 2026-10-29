"""Summarize rolling trials and separately certify geometric connectivity."""
import pathlib,json,collections,math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
root=pathlib.Path('/home/user/ros2_experiments/rolling_bypass_20261029')
data=json.loads((root/'input.json').read_text())
summaries=json.loads((root/'results/summary.json').read_text())
def connected(t):
    # Independent audit only: conservative L-infinity expanded wall boxes.
    # The certificate is never fed to the DWA reference path or sampler.
    h=.025; xmin=.5;ymin=-1.3;nx=77;ny=65;r=.283
    rear=-1.1+.25*min(max(t-2,0),8.8)-.52-.013
    def free(i,j):
        x=xmin+i*h;y=ymin+j*h
        if 1.08-.013<=x<=2.12+.013 and y>=rear:return False
        return not any(a-r<=x<=c+r and b-r<=y<=d+r for a,b,c,d in data['walls'])
    start=(round((data['initial'][0]-xmin)/h),round((data['initial'][1]-ymin)/h))
    todo=collections.deque([start]);seen={start};reached=False
    while todo:
        i,j=todo.popleft();x=xmin+i*h
        if x>2.12+.013:reached=True;break
        for u,v in [(i-1,j),(i+1,j),(i,j-1),(i,j+1)]:
            if 0<=u<nx and 0<=v<ny and (u,v) not in seen and free(u,v):
                seen.add((u,v));todo.append((u,v))
    return {'t':t,'connected':reached,'rear_limit_with_reserve':rear,
            'centre_channel_width':rear-(-1.5+.283),'visited':len(seen)}
geometry=[connected(t) for t in [3.5,3.6,3.7,4,5,6,7,8,8.8]]
analysis=[];series={}
for s in summaries:
    rows=[json.loads(x) for x in (root/f'results/start_{s["start"]:.2f}.jsonl').read_text().splitlines()]
    series[s['start']]=rows
    assert all(r['fallback_ok'] and r['dynamic_available'] for r in rows)
    assert all(r['executed_clearance']>.05 for r in rows)
    assert s['passed'] and s['rear_entry']>=8.8
    # Exact continuous guard independent of the helper's passed() return.
    assert s['final_pose'][0]>2.12
    assert s['final_pose'][1]<(-1.1+.25*min(s['pass_time']-2,8.8)-.52)
    early=[r for r in rows if r['t']<8.8]
    certificate=[r for r in rows if r['best_negative_rotation'] and r['score'] and
                 r['best_negative_rotation']['score']>r['score']['total']]
    item=dict(s)
    item.update({'cycles':len(rows),'sampled':sum(r['sampled'] for r in rows),
      'low_speed_time_v_lt_002':sum(abs(r['measured'][0])<.02 for r in rows)*.05,
      'minimum_y':min(r['next_pose'][1] for r in rows),'maximum_y':max(r['next_pose'][1] for r in rows),
      'pre_release_progress':early[-1]['next_pose'][0]-data['initial'][0] if early else 0,
      'first_ranked_rear_reaching_candidate':next((r['t'] for r in rows if r['rear_reaches_ranked']>0),None),
      'safe_negative_rotation_ranked_but_costlier_cycles':len(certificate),
      'rotation_example':certificate[len(certificate)//2] if certificate else None,
      'first_motion_lead_vs_release':8.8-s['first_motion'],
      'actual_rear_entry_lead_vs_release':8.8-s['rear_entry']})
    analysis.append(item)
result={'model':'complete-scene-geometry conditional simulation, not ROS/Gazebo',
        'geometry':geometry,'trials':analysis,'controller_parameter_changes':False,
        'any_rear_entry_before_historical_release':any(s['rear_entry']<8.8 for s in summaries),
        'analytic_channel_opens_after':3.664,
        'earliest_actual_rear_entry':min(s['rear_entry'] for s in summaries),
        'earliest_full_passage':min(s['pass_time'] for s in summaries)}
(root/'analysis.json').write_text(json.dumps(result,indent=2))
print('geometry',geometry)
for a in analysis:
    print({k:a[k] for k in ['start','first_motion','rear_entry','pass_time','minimum_clearance','stops',
      'low_speed_time_v_lt_002','minimum_y','first_ranked_rear_reaching_candidate',
      'safe_negative_rotation_ranked_but_costlier_cycles']})

fig,axes=plt.subplots(3,1,figsize=(10,9),sharex=True)
for start,rows in series.items():
    t=np.array([r['t'] for r in rows]);p=np.array([r['pose'] for r in rows]);v=np.array([r['measured'][0] for r in rows])
    axes[0].plot(t,p[:,0],label=f'start {start:.0f} s')
    axes[1].plot(t,p[:,1]);axes[2].plot(t,v)
for ax in axes:
    ax.axvline(8.8,color='black',ls='--',label='historical release 8.8 s' if ax is axes[0] else None)
    ax.grid(alpha=.25)
axes[0].axhline(1.08,color='gray',ls=':');axes[0].axhline(2.12,color='gray',ls=':')
axes[0].set_ylabel('World x (m)');axes[0].legend(ncol=3,fontsize=8)
axes[1].set_ylabel('World y (m)');axes[2].set_ylabel('Speed (m/s)')
axes[2].set_xlabel('Time since navigation dispatch (s)')
fig.suptitle('Original-score DWA rolling simulation: rear passage occurs after release')
fig.tight_layout();fig.savefig(root/'rolling_timeseries.png',dpi=180)
fig,axes=plt.subplots(1,3,figsize=(12,4),sharex=True,sharey=True)
for ax,t in zip(axes,[4,6,8]):
    rear=-1.1+.25*(t-2)-.52;cy=-1.1+.25*(t-2)
    for a,b,c,d in data['walls']:
        ax.add_patch(Rectangle((a,b),c-a,d-b,color='#333333'))
    ax.add_patch(Rectangle((1.08,rear),1.04,2-rear,color='#e89c83',alpha=.35))
    ax.add_patch(Rectangle((1.35,cy-.25),.5,.5,facecolor='#c44532',edgecolor='black'))
    for start,rows in series.items():
        ax.plot([r['pose'][0] for r in rows],[r['pose'][1] for r in rows],lw=1)
    ax.axhline(-1.217,color='gray',ls=':',lw=.8)
    ax.set_title(f't = {t} s, geometric rear channel open')
    ax.set_xlim(.5,2.4);ax.set_ylim(-1.55,1.5);ax.set_aspect('equal');ax.set_xlabel('World x (m)')
axes[0].set_ylabel('World y (m)');fig.suptitle('Scene geometry and conservative exclusion; paths remain near y = 0')
fig.tight_layout();fig.savefig(root/'geometry_and_paths.png',dpi=180)
