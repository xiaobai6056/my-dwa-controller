"""Read-only export of recorded D01 t2 sensor/path inputs; no Gazebo launch."""
import json, pathlib, bisect, argparse
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('run_directory', type=pathlib.Path, help='Original external recording: bag/ and paired_decisions.json')
parser.add_argument('output_json', type=pathlib.Path, help='New exported JSON file')
args = parser.parse_args()
run = args.run_directory.expanduser().resolve()
if not (run/'bag').is_dir() or not (run/'paired_decisions.json').is_file():
    parser.error('run_directory must contain the original bag/ and paired_decisions.json')
if args.output_json.exists():
    parser.error('output_json already exists')
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
with args.output_json.open('x') as output:
    output.write(json.dumps({'run':str(run), 'rows':rows}, indent=2))
print('exported cycles',len(rows),'plans',len(plans),'missing',sum(r['plan'] is None for r in rows))
