"""Diagnostic reproducer for first service response discovery, no mutation."""
import csv
import io
from pathlib import Path
import sys
import time
from types import SimpleNamespace
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_trial import Collector
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from lifecycle_msgs.srv import GetState

rclpy.init()
c = Collector(SimpleNamespace(odom_topic='/odom', controller_cmd_topic='/cmd_vel_nav',
                             robot_cmd_topic='/cmd_vel'), csv.DictWriter(io.StringIO(), []))
nav = BasicNavigator(node_name='probe_navigator')
client = c.node.create_client(GetState, '/amcl/get_state')
for attempt in range(3):
    deadline = time.monotonic() + 3
    while not client.service_is_ready() and time.monotonic() < deadline:
        c.tick()
    future = client.call_async(GetState.Request())
    while not future.done() and time.monotonic() < deadline:
        c.tick()
    print('attempt', attempt, 'done', future.done(), 'clock', c.clock.sim,
          'result', future.result() if future.done() else None, flush=True)
c.executor.shutdown()
c.node.destroy_node()
nav.destroy_node()
rclpy.shutdown()
