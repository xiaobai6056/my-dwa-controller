"""Read-only rosbag2 extraction and source-stamped SE2 alignment.

Bag recording uses simulation time. Header stamps remain authoritative for maps,
odom and TF. Unstamped ModelStates retains bag receipt time and an explicit
quantization limitation. No replay or publisher is used.
"""
import bisect
import gzip
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import numpy as np

HERE=Path(__file__).resolve().parent


def seconds(stamp):return stamp.sec+stamp.nanosec*1e-9


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def compose(a,b):
    c,s=math.cos(a[2]),math.sin(a[2])
    return [a[0]+c*b[0]-s*b[1],a[1]+s*b[0]+c*b[1],math.remainder(a[2]+b[2],2*math.pi)]


def inverse(a):
    c,s=math.cos(a[2]),math.sin(a[2])
    return [-c*a[0]-s*a[1],s*a[0]-c*a[1],-a[2]]


def interpolate_pose(samples, stamp, max_gap=.2):
    """Interpolate source-stamped data only; never silently use the latest TF."""
    if not samples:return None
    times=[s[0] for s in samples]
    i=bisect.bisect_left(times,stamp)
    if i<len(samples) and abs(times[i]-stamp)<1e-8:return list(samples[i][1])
    if i==0 or i==len(samples) or times[i]-times[i-1]>max_gap:return None
    u=(stamp-times[i-1])/(times[i]-times[i-1]);a=samples[i-1][1];b=samples[i][1]
    return [(1-u)*a[0]+u*b[0],(1-u)*a[1]+u*b[1],
            math.remainder(a[2]+u*math.remainder(b[2]-a[2],2*math.pi),2*math.pi)]


def lookup_pose(edges, static, target, source, stamp):
    queue=[(target,[0.,0.,0.])];visited={target}
    while queue:
        frame,total=queue.pop(0)
        if frame==source:return total
        for (parent,child),samples in edges.items():
            if frame not in (parent,child):continue
            step=list(samples[-1][1]) if (parent,child) in static else interpolate_pose(samples,stamp)
            if step is None:continue
            next_frame=child if frame==parent else parent
            if next_frame in visited:continue
            queue.append((next_frame,compose(total,step if frame==parent else inverse(step))))
            visited.add(next_frame)
    return None


def signature(grid):
    """Identity of a raw grid, tolerant only to serialized geometry precision."""
    geometry=[grid['width'],grid['height'],round(grid['resolution'],7),
              *[round(v,6) for v in grid['origin']]]
    return hashlib.sha256(json.dumps(geometry).encode()+bytes(grid['data'])).hexdigest()


def write_stream(path, records):
    with gzip.open(path,'wt') as f:
        for r in records:f.write(json.dumps(r,allow_nan=False)+'\n')


def export(directory):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import yaml
    bag=directory/'bag';metadata=yaml.safe_load((bag/'metadata.yaml').read_text())['rosbag2_bagfile_information']
    maps=[];rawmaps=[];tf=[];odom=[];gt=[];contact=[];counts={};stamps={};edges={};static=set()
    for db in metadata['relative_file_paths']:
        con=sqlite3.connect(f'file:{bag/db}?mode=ro',uri=True)
        topics={i:(name,typ) for i,name,typ in con.execute('select id,name,type from topics')}
        classes={i:get_message(typ) for i,(_,typ) in topics.items()}
        for topicid,timestamp,payload in con.execute('select topic_id,timestamp,data from messages order by timestamp,id'):
            name,typ=topics[topicid];counts[name]=counts.get(name,0)+1;t=timestamp*1e-9
            if name not in ['/local_costmap/costmap','/local_costmap/costmap_raw','/tf','/tf_static',
                            '/odom','/benchmark/model_states','/gazebo/model_states','/benchmark/contacts','/scan']:
                continue
            msg=deserialize_message(payload,classes[topicid])
            if hasattr(msg,'header'):stamps.setdefault(name,[]).append((seconds(msg.header.stamp),t))
            if name in ['/tf','/tf_static']:
                for tr in msg.transforms:
                    q=tr.transform;pose=[q.translation.x,q.translation.y,yaw(q.rotation)]
                    edge=(tr.header.frame_id,tr.child_frame_id);stamp=seconds(tr.header.stamp)
                    edges.setdefault(edge,{})[stamp]=pose
                    if name=='/tf_static':static.add(edge)
                    tf.append(dict(t=t,stamp=stamp,parent=edge[0],child=edge[1],pose=pose,static=name=='/tf_static'))
            elif name=='/odom':
                p=msg.pose.pose;odom.append(dict(t=t,stamp=seconds(msg.header.stamp),frame=msg.header.frame_id,
                    child=msg.child_frame_id,pose=[p.position.x,p.position.y,yaw(p.orientation)],
                    v=msg.twist.twist.linear.x,w=msg.twist.twist.angular.z))
            elif name in ['/benchmark/model_states','/gazebo/model_states']:
                names=list(msg.name)
                if 'turtlebot3_waffle' in names:
                    r=names.index('turtlebot3_waffle');p=msg.pose[r]
                    row=dict(t=t,stamp=None,clock_basis='sim-time bag receipt; ModelStates unstamped',
                        robot=[p.position.x,p.position.y,yaw(p.orientation)])
                    if 'dynamic_obstacle' in names:
                        p=msg.pose[names.index('dynamic_obstacle')];row['obstacle']=[p.position.x,p.position.y,yaw(p.orientation)]
                    gt.append(row)
            elif name=='/benchmark/contacts':
                for state in msg.states:
                    pair=[state.collision1_name,state.collision2_name]
                    if any('turtlebot3_waffle::' in z for z in pair):contact.append(dict(t=t,stamp=seconds(msg.header.stamp),pair=pair))
            elif name in ['/local_costmap/costmap','/local_costmap/costmap_raw']:
                info=msg.info if name.endswith('/costmap') else msg.metadata
                origin=info.origin
                row=dict(type='costmap',t=t,stamp=seconds(msg.header.stamp),frame=msg.header.frame_id,
                    width=info.width if name.endswith('/costmap') else info.size_x,
                    height=info.height if name.endswith('/costmap') else info.size_y,
                    resolution=info.resolution,origin=[origin.position.x,origin.position.y],
                    origin_yaw=yaw(origin.orientation),data=list(msg.data),topic=name,
                    cost_encoding='occupancy_0_100' if name.endswith('/costmap') else 'raw_0_255')
                if abs(row['origin_yaw'])>1e-9:raise ValueError('rotated grid unsupported; no silent axis alignment')
                if hasattr(info,'update_time'):row['metadata_update_time']=seconds(info.update_time)
                (maps if name.endswith('/costmap') else rawmaps).append(row)
        con.close()
    edges={e:sorted(v.items()) for e,v in edges.items()}
    alignment=[];epoch=json.loads((directory/'trial/summary.json').read_text())['navigation_start_sim_s']
    for m in maps:
        pose=lookup_pose(edges,static,m['frame'],'base_footprint',m['stamp'])
        near=min(gt,key=lambda z:abs(z['t']-m['stamp'])) if gt else None
        row=dict(stamp=m['stamp'],relative_s=m['stamp']-epoch,bag_receipt_s=m['t'],frame=m['frame'],
            source_time_tf_valid=pose is not None,robot_in_map_at_source_stamp=pose,
            gt_receipt_s=near['t'] if near else None,gt_receipt_distance_s=abs(near['t']-m['stamp']) if near else None,
            gt_source_stamp_available=False)
        if near and pose and abs(near['t']-m['stamp'])<=.15:
            world_to_grid=compose(pose,inverse(near['robot']))
            row['world_to_grid_approx']=world_to_grid
            if 'obstacle' in near:row['obstacle_in_grid_approx']=compose(world_to_grid,near['obstacle'])
            row['gt_projection_limit']='GT uses quantized bag receipt clock (~0.1 s), not a source stamp; offline reference only.'
        alignment.append(row)
    for filename,values in [('bag_maps.jsonl.gz',maps),('bag_raw_maps.jsonl.gz',rawmaps),
                            ('bag_tf.jsonl.gz',tf),('bag_odom.jsonl.gz',odom),
                            ('bag_gt.jsonl.gz',gt),('bag_contacts.jsonl.gz',contact),
                            ('bag_map_alignment.jsonl.gz',alignment)]:
        write_stream(directory/filename,values)
    raw_by_signature={}
    for m in rawmaps:raw_by_signature.setdefault(signature(m),[]).append(m['stamp'])
    match=[]
    for file in directory.glob('decisions_*.jsonl'):
        with file.open() as f:
            for i,line in enumerate(f):
                c=json.loads(line)
                if not c['costmap']['data']:continue
                times=raw_by_signature.get(signature(c['costmap']),[])
                earlier=[t for t in times if t<=c['input_stamp']+1e-8]
                match.append(dict(cycle=i,input_stamp=c['input_stamp'],relative_s=c['input_stamp']-epoch,
                    latest_same_content_published_stamp=max(earlier) if earlier else None,
                    elapsed_since_same_content_publication_s=c['input_stamp']-max(earlier) if earlier else None,
                    meaning='Content identity with an earlier publication; NOT actual age of the controller map or sensor observation.'))
    write_stream(directory/'bag_decision_content_matches.jsonl.gz',match)
    stampstats={}
    for topic,values in stamps.items():
        delta=np.array([t-s for s,t in values]);unique=sorted(set(s for s,t in values))
        stampstats[topic]=dict(messages=len(values),distinct_source_stamps=len(unique),
            source_interval_median_s=float(np.median(np.diff(unique))) if len(unique)>1 else None,
            bag_receipt_minus_source_min_s=float(delta.min()),bag_receipt_minus_source_max_s=float(delta.max()),
            note='Bag receipt clock is quantized by /clock; small negative values are not negative physical latency.')
    outcome=dict(run=directory.name,bag_metadata=str(bag/'metadata.yaml'),topic_counts=counts,
        exported_maps=len(maps),exported_raw_maps=len(rawmaps),source_time_tf_covered_maps=sum(z['source_time_tf_valid'] for z in alignment),
        tf_edges=[dict(parent=a,child=b,samples=len(v),static=(a,b) in static) for (a,b),v in edges.items()],
        stamp_statistics=stampstats,gt_records=len(gt),gt_has_source_stamps=False,
        physical_contact_source_stamps=sorted(set(z['stamp'] for z in contact)),
        decision_maps_examined=len(match),decision_maps_matching_earlier_published_content=sum(z['latest_same_content_published_stamp'] is not None for z in match),
        limitations=['Map header/metadata times describe upstream publications/updates, not per-cell sensor acquisition.',
            'ModelStates has no source timestamp; retain receipt-clock uncertainty.',
            'Planar SE2 projection of TF and GT; no claim of exact 3D geometry.',
            'Bag started after original runner dispatch and can miss initial task samples.'])
    (directory/'bag_analysis.json').write_text(json.dumps(outcome,indent=2)+'\n')
    print(directory.name,'maps',len(maps),'raw',len(rawmaps),'TF',outcome['source_time_tf_covered_maps'],flush=True)
    return outcome


def main():
    results=[export(p) for p in sorted((HERE/'runs').iterdir()) if (p/'bag/metadata.yaml').exists()]
    (HERE/'outputs/bag_summary.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__':main()
