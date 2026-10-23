"""Prescribed sim-time obstacle motion; only the obstacle may be actuated here."""
import time
from scenario import position


class Motion:
    def __init__(self, collector, scene, emit):
        from gazebo_msgs.srv import SetEntityState
        self.c, self.scene, self.emit = collector, scene, emit
        self.service_type = SetEntityState
        self.client = collector.node.create_client(
            SetEntityState, '/benchmark/set_entity_state')
        self.epoch = None
        self.pending = None
        self.last = None
        self.error = None
        self.sent = 0
        self.acknowledged = 0

    def begin(self, epoch):
        self.epoch = epoch
        self.emit('dynamic_events', dict(event='motion_epoch', t=epoch,
                                         trigger='task_dispatch_sim_time'))

    def tick(self):
        if self.epoch is None or self.error:
            return
        if self.pending is not None:
            if not self.pending.done():
                if time.monotonic() - self.sent_wall > 2.:
                    self.error = 'obstacle SetEntityState response timeout'
                return
            try:
                response = self.pending.result()
                if not response.success:
                    raise RuntimeError('SetEntityState returned false')
                self.acknowledged += 1
            except Exception as exc:
                self.error = repr(exc)
                return
            self.pending = None
        now = self.c.clock.sim
        if now == self.last:
            return
        self.last = now
        x, y = position(self.scene, now - self.epoch)
        request = self.service_type.Request()
        request.state.name = 'dynamic_obstacle'
        request.state.reference_frame = 'world'
        request.state.pose.position.x, request.state.pose.position.y = x, y
        request.state.pose.position.z = self.scene['size'][2] / 2
        request.state.pose.orientation.w = 1.
        self.pending = self.client.call_async(request)
        self.sent_wall = time.monotonic()
        self.sent += 1
        self.emit('motion', dict(t=now, elapsed=now - self.epoch, x=x, y=y))
