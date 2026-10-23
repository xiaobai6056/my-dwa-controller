"""Compare actual Nav2 OccupancyGrid to generated S02 wall geometry."""
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid

rclpy.init()
node = Node('benchmark_map_probe')
messages = []
sub = node.create_subscription(OccupancyGrid, '/map', messages.append,
                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
deadline = time.monotonic() + 5
while not messages and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.1)
rectangles = json.loads((Path(__file__).resolve().parents[1] /
                        'assets/S02/geometry.json').read_text())['obstacle_rectangles']
if messages:
    msg = messages[-1]
    mismatches = 0
    for y in range(msg.info.height):
        for x in range(msg.info.width):
            wx = msg.info.origin.position.x + (x+.5)*msg.info.resolution
            wy = msg.info.origin.position.y + (y+.5)*msg.info.resolution
            occupied = any(a <= wx <= c and b <= wy <= d for a, b, c, d in rectangles)
            mismatches += int((msg.data[y*msg.info.width+x] == 100) != occupied)
    print('published frame', msg.header.frame_id, 'dimensions', msg.info.width, msg.info.height,
          'origin', msg.info.origin.position.x, msg.info.origin.position.y,
          'mismatched cells', mismatches, '/', len(msg.data))
node.destroy_node()
rclpy.shutdown()
