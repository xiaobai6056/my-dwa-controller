"""Read-only ground truth, contact and sensor evidence, isolated from navigation."""
import gzip
import json
import math
import time

from evaluation import ROBOT_BOUNDS, box_polygon, polygon_gap
from run_trial import yaw, stamp_seconds


def transform_point(x, y, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return pose[0] + c * x - s * y, pose[1] + s * x + c * y


def local_point(x, y, pose):
    dx, dy = x - pose[0], y - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return c * dx + s * dy, -s * dx + c * dy


class Observer:
    def __init__(self, collector, scene, output):
        from gazebo_msgs.msg import ModelStates, ContactsState
        from sensor_msgs.msg import LaserScan
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
        self.c, self.scene = collector, scene
        names = (
            'ground_truth',
            'contacts',
            'perception',
            'commands',
            'motion',
            'dynamic_events')
        self.streams = {name: (output / (name + '.jsonl')).open('w')
                        for name in names}
        self.sensors = gzip.open(output / 'sensors.jsonl.gz', 'wt')
        self.robot = self.actor = None
        self.last_gt_wall = None
        self.contacts = 0
        self.contact_callbacks = 0
        self.gt_count = 0
        self.release = None
        self.scan_count = self.scan_hits = self.cost_count = self.cost_hits = 0
        self.max_motion_error = 0.
        node = collector.node
        self.subs = [
            node.create_subscription(
                ModelStates,
                '/benchmark/model_states',
                self.ground_truth,
                qos_profile_sensor_data),
            node.create_subscription(
                ContactsState,
                '/benchmark/contacts',
                self.contact,
                qos_profile_sensor_data),
            node.create_subscription(
                LaserScan,
                '/scan',
                self.scan,
                qos_profile_sensor_data),
            node.create_subscription(
                OccupancyGrid,
                '/local_costmap/costmap',
                self.costmap,
                QoSProfile(
                    depth=1,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL))]

    def emit(self, name, value):
        self.streams[name].write(json.dumps(value, allow_nan=False) + '\n')

    def ground_truth(self, msg):
        try:
            ri = msg.name.index('turtlebot3_waffle')
            ai = msg.name.index('dynamic_obstacle')
        except ValueError:
            return
        rp, ap = msg.pose[ri], msg.pose[ai]
        self.robot = [rp.position.x, rp.position.y, yaw(rp.orientation)]
        self.actor = [ap.position.x, ap.position.y, yaw(ap.orientation)]
        self.last_gt_wall = time.monotonic()
        self.gt_count += 1
        if not self.c.recording:
            return
        w, h, _ = self.scene['size']
        gap = polygon_gap(
            box_polygon(*self.robot, ROBOT_BOUNDS),
            box_polygon(*self.actor, [-w / 2, -h / 2, w / 2, h / 2]))
        twist = msg.twist[ri]
        forward = math.cos(self.robot[2]) * twist.linear.x + \
            math.sin(self.robot[2]) * twist.linear.y
        now = self.c.clock.sim
        self.emit(
            'ground_truth',
            dict(
                t=now,
                robot=self.robot,
                obstacle=self.actor,
                gap_approx_m=gap,
                vx=forward,
                wz=twist.angular.z))
        if (self.scene['id'] == 'D02' and self.release is None and
                self.c.motion.epoch is not None and now - self.c.motion.epoch >= 20 and
                self.actor[1] - h / 2 >= .45):
            self.release = now
            self.emit(
                'dynamic_events',
                dict(
                    t=now,
                    event='corridor_clear',
                    obstacle_y=self.actor[1],
                    corridor_edge=.45))

    def contact(self, msg):
        self.contact_callbacks += 1
        if not self.c.recording:
            return
        for state in msg.states:
            pair = [state.collision1_name, state.collision2_name]
            if not any('turtlebot3_waffle::' in name for name in pair):
                continue
            self.contacts += 1
            self.emit(
                'contacts', dict(
                    t=self.c.clock.sim, stamp=stamp_seconds(
                        msg.header.stamp), pair=pair, depths=list(
                        state.depths)))

    def command(self, msg):
        if self.c.recording:
            self.emit('commands', dict(t=self.c.clock.sim, vx=msg.linear.x,
                                       vy=msg.linear.y, wz=msg.angular.z))

    def scan(self, msg):
        if not self.c.recording or self.robot is None or self.actor is None:
            return
        try:
            tf = self.c.buffer.lookup_transform(
                'base_footprint', msg.header.frame_id, self.c.Time())
        except Exception:
            return
        rel = tf.transform
        laser_xy = transform_point(
            rel.translation.x,
            rel.translation.y,
            self.robot)
        laser = [*laser_xy, self.robot[2] + yaw(rel.rotation)]
        w, h, _ = self.scene['size']
        hits = 0
        for i, distance in enumerate(msg.ranges):
            if not math.isfinite(
                    distance) or not msg.range_min <= distance <= msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x, y = transform_point(
                distance * math.cos(angle),
                distance * math.sin(angle),
                laser)
            u, v = local_point(x, y, self.actor)
            if abs(u) <= w / 2 + .08 and abs(v) <= h / 2 + .08:
                hits += 1
        self.scan_count += 1
        self.scan_hits += bool(hits)
        self.emit(
            'perception',
            dict(
                type='scan',
                range_min=float(msg.range_min), range_max=float(msg.range_max),
                t=self.c.clock.sim,
                stamp=stamp_seconds(
                    msg.header.stamp),
                hit_rays=hits))
        self.sensors.write(
            json.dumps(
                dict(
                    type='scan',
                    range_min=float(msg.range_min), range_max=float(msg.range_max),
                    t=self.c.clock.sim,
                    stamp=stamp_seconds(
                        msg.header.stamp),
                    frame=msg.header.frame_id,
                    laser_world=laser,
                    obstacle_world=self.actor,
                    angle_min=msg.angle_min,
                    angle_increment=msg.angle_increment,
                    ranges=[
                        float(r) if math.isfinite(r) else None for r in msg.ranges])) +
            '\n')

    def costmap(self, msg):
        if not self.c.recording or self.robot is None or self.actor is None:
            return
        try:
            tf = self.c.buffer.lookup_transform(
                msg.header.frame_id, 'base_footprint', self.c.Time())
        except Exception:
            return
        t = tf.transform
        grid_robot = [t.translation.x, t.translation.y, yaw(t.rotation)]
        u, v = local_point(self.actor[0], self.actor[1], self.robot)
        x, y = transform_point(u, v, grid_robot)
        angle = self.actor[2] - self.robot[2] + grid_robot[2]
        origin = msg.info.origin
        resolution, width, height = msg.info.resolution, msg.info.width, msg.info.height
        w, h, _ = self.scene['size']
        lethal = tested = 0
        for j in range(
                max(0, int((y - h - origin.position.y) / resolution)),
                min(height, int((y + h - origin.position.y) / resolution) + 1)):
            for i in range(
                    max(0, int((x - w - origin.position.x) / resolution)),
                    min(width, int((x + w - origin.position.x) / resolution) + 1)):
                cx = origin.position.x + (i + .5) * resolution
                cy = origin.position.y + (j + .5) * resolution
                a, b = local_point(cx, cy, [x, y, angle])
                if abs(a) <= w / 2 and abs(b) <= h / 2:
                    tested += 1
                    lethal += msg.data[j * width + i] >= 99
        self.cost_count += 1
        self.cost_hits += lethal > 0
        self.emit(
            'perception',
            dict(
                type='costmap',
                t=self.c.clock.sim,
                stamp=stamp_seconds(
                    msg.header.stamp),
                frame=msg.header.frame_id,
                tested_cells=tested,
                lethal_cells=lethal,
                obstacle_grid_pose=[
                    x,
                    y,
                    angle]))
        self.sensors.write(
            json.dumps(
                dict(
                    type='costmap',
                    t=self.c.clock.sim,
                    stamp=stamp_seconds(
                        msg.header.stamp),
                    frame=msg.header.frame_id,
                    obstacle_grid_pose=[
                        x,
                        y,
                        angle],
                    width=width,
                    height=height,
                    resolution=resolution,
                    origin=[
                        origin.position.x,
                        origin.position.y],
                    data=list(
                        msg.data))) +
            '\n')

    def close(self):
        for stream in self.streams.values():
            stream.close()
        self.sensors.close()
