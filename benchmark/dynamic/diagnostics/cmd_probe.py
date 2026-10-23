#!/usr/bin/env python3
"""Independent native command receipt timing; no publications or control calls."""
import json
from pathlib import Path
import signal
import sys
import time


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from rosgraph_msgs.msg import Clock
    rclpy.init()
    node = Node('diagnostic_command_observer')
    sim = None
    def clock(msg):
        nonlocal sim
        sim = msg.clock.sec+msg.clock.nanosec*1e-9
    with Path(sys.argv[1]).open('w', buffering=1) as output:
        def record(msg, topic):
            output.write(json.dumps(dict(topic=topic, sim=sim,
                monotonic_ns=time.monotonic_ns(), system_ns=time.time_ns(),
                vx=msg.linear.x, wz=msg.angular.z))+'\n')
        subscriptions = [node.create_subscription(Clock, '/clock', clock, qos_profile_sensor_data)]
        for topic in ('/cmd_vel_nav', '/cmd_vel'):
            subscriptions.append(node.create_subscription(Twist, topic,
                lambda msg, t=topic: record(msg, t), qos_profile_sensor_data))
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()