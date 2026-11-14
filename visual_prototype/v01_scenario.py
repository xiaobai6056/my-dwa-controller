"""Drive V01 from Gazebo /clock and record detector and simulator output."""

from collections import deque
import json
from math import cos, sin
from pathlib import Path
from statistics import mean
import sys
from time import monotonic

from gazebo_msgs.srv import GetEntityState, SetEntityState
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from v01_config import PEDESTRIAN, ROBOT, events, pedestrian_xy, robot_speed, sim_label

PREFIX = "/visual_prototype"
SECOND = 1_000_000_000


def ns(stamp):
    return stamp.sec * SECOND + stamp.nanosec


def xyz(vector):
    return [vector.x, vector.y, vector.z]


class Scenario(Node):
    def __init__(self, directory):
        super().__init__("v01_scenario")
        if not self.get_parameter("use_sim_time").value:
            raise RuntimeError("V01 requires use_sim_time:=true")
        self.config = json.loads((directory / "config.json").read_text())
        self.events = deque(events(self.config))
        self.timeline_log = (directory / "timeline.jsonl").open("w", buffering=1)
        self.metrics_log = (directory / "detections.jsonl").open("w", buffering=1)
        self.truth_log = (directory / "ground_truth.jsonl").open("w", buffering=1)
        self.stats_log = (directory / "statistics.jsonl").open("w", buffering=1)
        self.clock_ns = None
        self.origin_ns = None
        self.epoch = 0
        self.complete = False
        self.detector_stamp_ns = None
        self.latest_target = None
        self.set_pending = None
        self.restore = deque()
        self.restore_after_ns = None
        self.last_truth_ns = None
        self.truth_pending = set()
        self.previous_truth = {}
        self.samples = []
        self.rqt_windows = self.declare_parameter("rqt_windows", 0).value
        self.rqt_clients = {}
        self.rqt_pending = set()
        self.rqt_ready = set()
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 1)
        self.set_state = self.create_client(SetEntityState, f"{PREFIX}/v01_set_pose")
        self.get_state = self.create_client(GetEntityState, f"{PREFIX}/get_entity_state")
        self.create_subscription(Clock, "/clock", self.on_clock,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(String, f"{PREFIX}/metrics", self.on_detection, 10)
        print("Waiting for Gazebo /clock, motion services and the first fresh detector frame",
              flush=True)

    def on_clock(self, message):
        now = ns(message.clock)
        if self.clock_ns is not None:
            if now < self.clock_ns:
                self.reset(now)
            elif now == self.clock_ns:
                return
        self.clock_ns = now
        if len(self.rqt_ready) < self.rqt_windows:
            self.configure_rqt()

        if self.restore:
            self.publish_speed(0.0)
            if (now >= self.restore_after_ns and self.set_pending is None
                    and self.set_state.service_is_ready()):
                entity, name = self.restore.popleft()
                actor = self.config[name]
                self.set_pose(entity, actor["x_m"], actor["y_m"], actor["yaw_rad"])
            return
        if self.origin_ns is None:
            if (self.set_pending is not None or not self.set_state.service_is_ready()
                    or not self.get_state.service_is_ready()
                    or not self.cmd.get_subscription_count()
                    or self.detector_stamp_ns is None
                    or not 0 <= now - self.detector_stamp_ns <= SECOND):
                return
            self.origin_ns = now
            self.latest_target = pedestrian_xy(self.config, 0)

        if self.complete:
            return
        elapsed = (now - self.origin_ns) / SECOND
        speed = robot_speed(self.config, elapsed)
        self.publish_speed(speed)
        target = pedestrian_xy(self.config, elapsed)
        if target != self.latest_target and self.set_pending is None:
            self.set_pose(PEDESTRIAN, *target, self.config["pedestrian"]["yaw_rad"])
            self.latest_target = target
        self.sample_truth(now)
        while self.events and self.events[0][0] <= elapsed:
            planned, event = self.events[0]
            if event in ("Pedestrian stops", "Motion completed", "Scenario completed") and (
                    target != self.latest_target or self.set_pending is not None):
                # Log the end after Gazebo has applied the last position command.
                break
            self.events.popleft()
            self.log_event(event, now, planned, elapsed)
            if event == "Scenario completed":
                self.finish_stats("completed")
                self.complete = True

    def configure_rqt(self):
        # Humble rqt constructs its C++ node without forwarding the CLI's ROS parameters.
        for name, namespace in self.get_node_names_and_namespaces():
            if not name.startswith("rqt_gui_cpp_node_"):
                continue
            full_name = f"{namespace.rstrip('/')}/{name}"
            if full_name not in self.rqt_clients:
                self.rqt_clients[full_name] = self.create_client(
                    SetParameters, f"{full_name}/set_parameters")
        for name, client in self.rqt_clients.items():
            if name in self.rqt_ready or name in self.rqt_pending or not client.service_is_ready():
                continue
            parameter = Parameter("use_sim_time", value=True).to_parameter_msg()
            future = client.call_async(SetParameters.Request(parameters=[parameter]))
            self.rqt_pending.add(name)
            future.add_done_callback(
                lambda response, name=name: self.rqt_configured(response, name))

    def rqt_configured(self, future, name):
        self.rqt_pending.remove(name)
        result = future.result().results[0]
        if result.successful:
            self.rqt_ready.add(name)
            self.get_logger().info(f"{name} use_sim_time=true")
        else:
            self.get_logger().warning(f"Could not set {name} use_sim_time: {result.reason}",
                                      throttle_duration_sec=5)

    def reset(self, now):
        if self.origin_ns is not None and not self.complete:
            self.finish_stats("reset")
        self.epoch += 1
        self.log_event("Clock reset; restoring initial poses", now, None, None)
        self.publish_speed(0.0)
        self.origin_ns = None
        self.complete = False
        self.detector_stamp_ns = None
        self.latest_target = None
        self.events = deque(events(self.config))
        self.restore = deque(((ROBOT, "robot"), (PEDESTRIAN, "pedestrian")))
        # Stock Waffle drive limits acceleration to 1 m/s². Brake before reapplying
        # the model pose so residual wheel rotation cannot pull it off the reset pose.
        braking_s = max(0.5, abs(self.config["robot"]["speed_mps"]) + 0.2)
        self.restore_after_ns = now + round(braking_s * SECOND)
        self.last_truth_ns = None
        self.previous_truth.clear()

    def publish_speed(self, speed):
        command = Twist()
        command.linear.x = float(speed)
        self.cmd.publish(command)

    def set_pose(self, entity, x, y, yaw):
        request = SetEntityState.Request()
        request.state.name = entity
        request.state.reference_frame = "world"
        pose = request.state.pose
        pose.position.x, pose.position.y = float(x), float(y)
        pose.orientation.z = sin(yaw / 2)
        pose.orientation.w = cos(yaw / 2)
        self.set_pending = self.set_state.call_async(request)
        self.set_pending.add_done_callback(self.pose_applied)

    def pose_applied(self, response):
        self.set_pending = None
        if not response.result().success:
            raise RuntimeError("Gazebo rejected the V01 pose command")

    def sample_truth(self, now):
        if self.last_truth_ns is not None and now - self.last_truth_ns < SECOND // 5:
            return
        self.last_truth_ns = now
        for entity in (ROBOT, PEDESTRIAN):
            if entity in self.truth_pending:
                continue
            self.truth_pending.add(entity)
            future = self.get_state.call_async(
                GetEntityState.Request(name=entity, reference_frame="world"))
            future.add_done_callback(lambda response, entity=entity, epoch=self.epoch:
                                     self.log_truth(response, entity, epoch))

    def log_truth(self, future, entity, epoch):
        self.truth_pending.remove(entity)
        result = future.result()
        if not result.success or epoch != self.epoch or self.origin_ns is None:
            return
        stamp = ns(result.header.stamp)
        if stamp < self.origin_ns:
            return
        state = result.state
        position = xyz(state.pose.position)
        rotation = state.pose.orientation
        previous = self.previous_truth.get(entity)
        estimated = None
        if previous and stamp > previous[0]:
            dt = (stamp - previous[0]) / SECOND
            estimated = [(new - old) / dt for new, old in zip(position, previous[1])]
        self.previous_truth[entity] = (stamp, position)
        self.write(self.truth_log, {
            "epoch": epoch, "entity": entity,
            "stamp": [result.header.stamp.sec, result.header.stamp.nanosec],
            "elapsed_s": (stamp - self.origin_ns) / SECOND, "frame_id": "world",
            "position_m": position, "orientation_xyzw": xyz(rotation) + [rotation.w],
            "linear_mps": xyz(state.twist.linear), "angular_radps": xyz(state.twist.angular),
            "pose_delta_linear_mps": estimated,
        })

    def on_detection(self, message):
        result = json.loads(message.data)
        sec, nanosec = result["stamp"]
        stamp = sec * SECOND + nanosec
        if self.clock_ns is None or not -SECOND <= stamp - self.clock_ns <= SECOND // 4:
            return
        self.detector_stamp_ns = stamp
        if self.origin_ns is None or stamp < self.origin_ns or self.complete:
            return
        elapsed = (stamp - self.origin_ns) / SECOND
        self.write(self.metrics_log, {**result, "epoch": self.epoch, "elapsed_s": elapsed})
        # Wall time is used exclusively to describe processing throughput.
        self.samples.append((
            stamp, monotonic(), result["predict_ms"], result["forward_ms"], bool(result["boxes"])))

    def finish_stats(self, status):
        count = len(self.samples)
        data = {
            "epoch": self.epoch, "mode": self.config["mode"], "status": status, "frames": count}
        if count:
            first, last = self.samples[0], self.samples[-1]
            wall_span = last[1] - first[1]
            sim_span = (last[0] - first[0]) / SECOND
            data.update(
                frames_with_person=sum(row[4] for row in self.samples),
                mean_predict_ms=mean(row[2] for row in self.samples),
                mean_forward_ms=mean(row[3] for row in self.samples),
                wall_output_hz=(count - 1) / wall_span if wall_span > 0 else None,
                sim_output_hz=(count - 1) / sim_span if sim_span > 0 else None,
            )
        self.write(self.stats_log, data)
        print(f"V01 epoch {self.epoch} statistics: {json.dumps(data)}", flush=True)
        self.samples.clear()

    def log_event(self, event, now, planned, elapsed):
        self.write(self.timeline_log, {
            "epoch": self.epoch, "event": event, "elapsed_s": elapsed,
            "scheduled_elapsed_s": planned, "stamp": list(divmod(now, SECOND)),
        })
        displayed = sim_label(elapsed if elapsed is not None else now / SECOND)
        print(f"[SIM {displayed}] {event} (epoch {self.epoch}, /clock {now / SECOND:.3f}s)",
              flush=True)

    @staticmethod
    def write(stream, row):
        stream.write(json.dumps(row, separators=(",", ":")) + "\n")

    def close(self):
        if self.origin_ns is not None and not self.complete:
            self.finish_stats("interrupted")
        for stream in (self.timeline_log, self.metrics_log, self.truth_log, self.stats_log):
            stream.close()


def main():
    directory = Path(sys.argv[1])
    rclpy.init(args=[sys.argv[0], *sys.argv[2:]])
    node = None
    try:
        node = Scenario(directory)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
