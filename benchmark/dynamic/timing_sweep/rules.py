"""Timing-only scenario adapter and read-only, explicitly approximate scoring rules."""
import copy
import json
import math
from pathlib import Path
import os


# External workspace install/build prefixes and baseline scripts are not bundled.
ARCHIVE_WS = Path(os.environ.get("MYDWA_WS", str(Path.home() / "ros2_ws")))

HERE=Path(__file__).resolve().parent
DYNAMIC=HERE.parent
ROOT=HERE.parents[2]
RELEASE=(ARCHIVE_WS / 'install_compile_comparison_20261020/release')

def original_scene():
    return dict(json.loads((DYNAMIC/'scenarios.json').read_text())['D01'],id='D01')

def scene(start_time):
    if start_time not in (2,3,4,5,6):raise ValueError('start time must be one of 2,3,4,5,6 seconds')
    value=copy.deepcopy(original_scene())
    delta=float(start_time)-4.
    for frame in value['keyframes'][1:]:frame[0]+=delta
    value['release_after']+=delta
    return value

def position(value,t):
    frames=value['keyframes']
    if t<=frames[0][0]:return frames[0][1:]
    for a,b in zip(frames,frames[1:]):
        if t<=b[0]:
            u=(t-a[0])/(b[0]-a[0])
            return [a[i]+u*(b[i]-a[i]) for i in (1,2)]
    return frames[-1][1:]

def selection(controller):
    if controller=='mydwa':return RELEASE,RELEASE/'lib/libmy_dwa_controller.so'
    if controller=='dwb':return RELEASE,Path('/opt/ros/humble/lib/libdwb_core.so')
    raise ValueError('controller must be mydwa or dwb')

def mapping_matches(text,expected):
    paths=set()
    for line in text.splitlines():
        if 'libmy_dwa_controller.so' in line or '/libdwb_core.so' in line:
            fields=line.split(None,5)
            if len(fields)!=6 or fields[5].endswith(' (deleted)'):return False
            paths.add(str(Path(fields[5]).resolve()))
    return paths=={str(Path(expected).resolve())}

def completed_trial(code,summary):
    return (code in (0,2) and summary.get('valid') is True and summary.get('status')=='finished'
        and summary.get('action_accepted') is True and summary.get('action_error') is None and summary.get('error') is None)

def verify_motion(gt,commands,value,epoch):
    start=value['keyframes'][1][0]
    initial=value['keyframes'][0][2]
    moved=next((r for r in gt if abs(r['obstacle'][1]-initial)>.005),None)
    observed=None if moved is None else moved['t']-epoch
    early=any(r['t']-epoch<start-.1 and abs(r['obstacle'][1]-initial)>.005 for r in gt)
    outside=[]
    for r in gt:
        elapsed=r['t']-epoch
        lower=position(value,elapsed-.2)[1]-.01
        upper=position(value,elapsed+.2)[1]+.01
        if abs(r['obstacle'][0]-1.6)>1e-5 or not lower<=r['obstacle'][1]<=upper:outside.append(elapsed)
    wrong_commands=[]
    for r in commands:
        expected=position(value,r['elapsed'])
        if abs(r['x']-expected[0])>1e-8 or abs(r['y']-expected[1])>1e-8:wrong_commands.append(r['elapsed'])
    moving_command=next((r['elapsed'] for r in commands if abs(r['y']-initial)>.005),None)
    valid=(observed is not None and start-.05<=observed<=start+.35 and not early and not outside and not wrong_commands)
    return dict(valid=valid,configured_start_s=start,observed_first_gt_motion_s=observed,
        gt_start_offset_s=None if observed is None else observed-start,
        first_moving_command_s=moving_command,early_gt_motion=early,
        gt_outside_receipt_clock_envelope=outside,wrong_motion_commands=wrong_commands,
        gt_samples=len(gt),command_samples=len(commands),
        rule='GT |dy|>0.005m, observed start within [-0.05,+0.35]s; full GT within schedule at receipt t +/-0.2s and0.01m; x fixed. Receipt clock ~0.1s, not exact physical onset.')

def response(samples,after,before,kind,persistence=.2):
    since=previous=None
    for t,v,w in samples:
        if t<after or t>=before:continue
        if previous is not None and t-previous>.35:since=None
        previous=t
        hit=v<=.21 if kind=='brake' else abs(w)>=.1
        if hit:
            since=t if since is None else since
            if t-since>=persistence-1e-8:return since
        else:since=None
    return None
def classify(features):
    """Describe observed passage; temporal separation alone never proves avoidance."""
    if features.get('command_evidence_complete') is False:
        return 'collision_behavior_unknown_command_evidence' if features['collision'] else ('pass_behavior_unknown_command_evidence' if features['success'] else 'navigation_failure_unknown')
    if features['collision']:
        return 'collision_with_turn' if features['turn_s'] is not None else 'collision_without_sustained_turn'
    if not features['success']:return 'navigation_failure_unknown'
    robots=features['robot_windows'];actors=features['obstacle_windows']
    if not robots or not actors:return 'interaction_unknown_incomplete_zone_evidence'
    robot_first=not features.get('robot_window_right_censored',False) and max(w[1] for w in robots)+.2<min(w[0] for w in actors)
    obstacle_first=not features.get('obstacle_window_right_censored',False) and max(w[1] for w in actors)+.2<min(w[0] for w in robots)
    if features['waiting_before_zone_s']>=.5 and obstacle_first:
        return 'observed_wait_then_pass'
    if (features['turn_s'] is not None and features['lateral_excursion_m']>=.15
            and features['min_gap_m']<.6):
        return 'turn_with_lateral_displacement_pass'
    if features['brake_s'] is not None and features['actual_brake_s'] is not None and (robot_first or obstacle_first):
        return 'slowed_separated_pass'
    no_substantial_response=(features['brake_s'] is None and features['actual_brake_s'] is None and features['waiting_before_zone_s']<.5 and features['lateral_excursion_m']<.15)
    if no_substantial_response and features['min_gap_m']>.05:
        if robot_first:return 'no_interaction_robot_first'
        if obstacle_first:return 'no_interaction_obstacle_first'
    return 'interaction_pass_unknown'
def command_coverage(samples,after,before):
    times=sorted(set(t for t,_,_ in samples if after-.2<=t<=before+.2))
    maximum=max((b-a for a,b in zip(times,times[1:])),default=None)
    complete=bool(len(times)>=3 and times[0]<=after+.2+1e-8 and times[-1]>=before-.2-1e-8 and maximum is not None and maximum<=.35)
    return dict(complete=complete,distinct_clock_samples=len(times),first_t=times[0] if times else None,
        last_t=times[-1] if times else None,max_gap_s=maximum,window=[after,before],
        rule='At least3 distinct clock samples; cover both window edges within0.2s; no intersample gap>0.35s. Incomplete does not mean no response.')