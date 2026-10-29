"""Read-only export of recorded D01 t2 sensor/path inputs; no Gazebo launch."""
import json, pathlib, bisect
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

root = pathlib.Path('/home/user/ros2_experiments/simplified_bypass_20261029')
run = pathlib.Path('/home/user/ros2_experiments/crossing_yield_reserve020_20261029/closed_loop_r1_20261029/B_D01_reserve020_r1')
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=str(run/'bag'), storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
plans, scans = [], {}
while reader.has_next():
    topic, data, receipt = reader.read_next()
    if topic not in ('/plan', '/scan'): continue
    m = deserialize_message(data, types[topic])
    if topic == '/plan':
        plans.append((receipt, {'frame':m.header.frame_id, 'poses':[[p.pose.position.x,p.pose.position.y,p.pose.orientation.z,p.pose.orientation.w] for p in m.poses]}))
    else:
        scans[m.header.stamp.sec*10**9+m.header.stamp.nanosec] = receipt
times = [t for t,_ in plans]
rows = json.loads((run/'paired_decisions.json').read_text())
for row in rows:
    d = row['decision']; i = bisect.bisect_right(times, scans[d['scan_ns']])-1
    row['plan'] = plans[i][1] if i >= 0 else None
    row['plan_receipt'] = times[i] if i >= 0 else None
(root/'inputs.json').write_text(json.dumps({'run':str(run), 'rows':rows}, indent=2))
print('exported cycles',len(rows),'plans',len(plans),'missing',sum(r['plan'] is None for r in rows))
