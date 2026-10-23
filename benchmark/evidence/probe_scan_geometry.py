"""Compare raw laser endpoints to known S02 physics walls at Gazebo world pose."""
import json
import math
from pathlib import Path
import subprocess
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init()
node = Node('benchmark_geometry_probe')
messages = []
sub = node.create_subscription(LaserScan, '/scan', messages.append, qos_profile_sensor_data)
deadline = time.monotonic() + 5
while not messages and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.1)
pose = list(map(float, subprocess.check_output(
    ['gz', 'model', '-m', 'turtlebot3_waffle', '-p'], timeout=5).decode().split()))
rectangles = json.loads((Path(__file__).resolve().parents[1] /
                        'assets/S02/geometry.json').read_text())['obstacle_rectangles']
if messages:
    scan = messages[-1]
    differences = []
    for index, measured in enumerate(scan.ranges):
        angle = pose[5] + scan.angle_min + index * scan.angle_increment
        ox, oy = pose[0] - .064*math.cos(pose[5]), pose[1] - .064*math.sin(pose[5])
        dx, dy = math.cos(angle), math.sin(angle)
        hits = []
        for a, b, c, d in rectangles:
            for x in (a, c):
                if abs(dx) > 1e-9:
                    hit = (x-ox) / dx
                    if hit >= 0 and b <= oy+hit*dy <= d:
                        hits.append(hit)
            for y in (b, d):
                if abs(dy) > 1e-9:
                    hit = (y-oy) / dy
                    if hit >= 0 and a <= ox+hit*dx <= c:
                        hits.append(hit)
        expected = min(hits) if hits else math.inf
        if math.isfinite(measured) and .2 < expected < scan.range_max:
            differences.append(abs(measured-expected))
    print('world pose', pose, 'laser frame', scan.header.frame_id,
          'range min/max', scan.range_min, scan.range_max,
          'matched rays', len(differences), 'mean range residual',
          sum(differences)/len(differences) if differences else None)
    print('quarter ranges', [scan.ranges[i] for i in (0, 90, 180, 270)])
node.destroy_node()
rclpy.shutdown()
