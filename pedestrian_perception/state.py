"""Fused motion histories and source-time velocity estimation."""

from collections import Counter, deque
from dataclasses import dataclass, field
from math import dist, hypot
from statistics import linear_regression

from .association import CIRCLE_RADIUS_M, CLUSTER_DISTANCE_M

SECOND = 1_000_000_000
ZERO = (0., 0.)
WINDOW_NS = 500_000_000
DIAGNOSTIC_WINDOW_NS = 800_000_000
MOVING_MPS = .05  # Existing R2 evaluation floor for direction at near-zero speed.


def _fit(samples):
    if len(samples) < 2:
        return None
    end = samples[-1][0]
    seconds = [(stamp - end) / SECOND for stamp, _ in samples]
    xy = tuple(zip(*(position for _, position in samples)))
    if len(samples) == 2:
        velocity = tuple((b - a) / -seconds[0] for a, b in xy)
    else:
        velocity = tuple(linear_regression(seconds, coordinate).slope for coordinate in xy)
    return velocity, tuple(stamp for stamp, _ in samples)


@dataclass
class Track:
    """Last trusted measurement, motion prior and contiguous samples."""

    sort_id: str
    stamp: int = 0
    position: tuple = ZERO
    velocity: tuple = ZERO
    direct: tuple = ZERO
    diagnostic: tuple = ZERO
    velocity_stamp_ns: int = 0
    motion: tuple = ZERO
    motion_stamp: int = 0
    sample_stamps: tuple = ()
    direct_stamps: tuple = ()
    diagnostic_stamps: tuple = ()
    history: deque = field(default_factory=deque)

    def motion_candidate(self, stamp, position, max_age):
        """Measure positional residual and optional directional and speed support."""
        dt = (stamp - self.stamp) / SECOND
        moving = self.motion_stamp and stamp - self.motion_stamp <= max_age
        vx, vy = self.motion if moving else ZERO
        x, y = self.position
        residual = dist((x + dt * vx, y + dt * vy), position)
        dx, dy = position[0] - x, position[1] - y
        speed = hypot(vx, vy)
        supports = (speed >= MOVING_MPS and dx * vx + dy * vy > 0
                    and abs(hypot(dx, dy) - dt * speed) <= CLUSTER_DISTANCE_M)
        return residual, supports

    def observe(self, sort_id, stamp, position, cadence_ns, continuous):
        """Accept one real position and fit only samples from its continuous segment."""
        if not continuous:
            self.history.clear()
        elif self.motion_stamp == self.stamp and self.reverses(stamp, position):
            self.history.clear()
            self.history.append((self.stamp, self.position))
        self.sort_id, self.stamp, self.position = sort_id, stamp, position
        self.history.append((stamp, position))
        while len(self.history) > 2 and self.history[0][0] < stamp - DIAGNOSTIC_WINDOW_NS:
            self.history.popleft()
        history = list(self.history)
        for value_field, stamps_field, window in (
                ('direct', 'direct_stamps', None),
                ('velocity', 'sample_stamps', WINDOW_NS),
                ('diagnostic', 'diagnostic_stamps', DIAGNOSTIC_WINDOW_NS)):
            if window is None:
                samples = history[-2:]
            else:
                samples = [sample for sample in history if sample[0] >= stamp - window]
                if stamp - samples[0][0] < max(0, window - cadence_ns):
                    samples = []
            fit = _fit(samples)
            if fit is None:
                setattr(self, stamps_field, ())
            else:
                velocity, stamps = fit
                setattr(self, value_field, velocity)
                setattr(self, stamps_field, stamps)
        if self.sample_stamps:
            self.velocity_stamp_ns = stamp
            self.motion, self.motion_stamp = self.velocity, stamp
        elif self.direct_stamps:
            self.motion, self.motion_stamp = self.direct, stamp

    def reverses(self, stamp, position):
        """Detect a trusted step opposite to the latest measurable direction."""
        vx, vy = self.motion
        speed = hypot(vx, vy)
        dx, dy = position[0] - self.position[0], position[1] - self.position[1]
        dt = (stamp - self.stamp) / SECOND
        if speed < MOVING_MPS:
            return False
        along = (dx * vx + dy * vy) / speed
        return along < -MOVING_MPS * dt and speed * dt - along > CLUSTER_DISTANCE_M


@dataclass(frozen=True)
class Snapshot:
    """Latest stored track measurement and validity at the output time."""

    track_id: str
    sort_id: str
    position: tuple
    velocity: tuple
    direct: tuple
    diagnostic: tuple
    position_valid: bool
    velocity_valid: bool
    direct_valid: bool
    diagnostic_valid: bool
    last_observation_stamp: int
    observation_age_ns: int | None
    sample_stamps: tuple
    direct_stamps: tuple
    diagnostic_stamps: tuple
    velocity_stamp_ns: int
    velocity_held: bool


class StateHistory:
    """Keep independent motion histories while publishing only measured positions."""

    def __init__(self, cadence_ns=200_000_000, max_age_ns=500_000_000,
                 retention_ns=5_000_000_000):
        self.cadence_ns = cadence_ns
        self.max_age_ns = max_age_ns
        self.retention_ns = retention_ns
        self.tracks = {}
        self.last_scan_stamp = 0
        self.next_id = 1

    def reset(self):
        """Drop all measurements and identities on a clock epoch change."""
        self.tracks.clear()
        self.last_scan_stamp = 0
        self.next_id = 1

    def candidates(self, stamp, position, recent, bootstrap_stamp):
        """Keep all plausible histories unless both distance and motion separate them."""
        candidates = []
        for identity, track in recent.items():
            residual, supports = track.motion_candidate(stamp, position, self.max_age_ns)
            gate = CIRCLE_RADIUS_M
            if track.stamp == bootstrap_stamp and not track.motion_stamp:
                gate *= 2  # One adjacent observation pair can establish the initial velocity.
            if residual <= gate:
                candidates.append((residual, identity, supports))
        candidates.sort()
        if len(candidates) > 1:
            distance, best, supports = candidates[0]
            if (supports and candidates[1][0] - distance > CLUSTER_DISTANCE_M
                    and not any(item[2] for item in candidates[1:])):
                return [best]
        return [identity for _, identity, _ in candidates]

    def update(self, stamp, positions):
        """Commit mutually unambiguous observations and start isolated new histories."""
        if stamp <= self.last_scan_stamp:
            return
        previous = self.last_scan_stamp
        self.last_scan_stamp = stamp
        gap = stamp - previous > 1.5 * self.cadence_ns
        recent = {identity: track for identity, track in self.tracks.items()
                  if stamp - track.stamp <= self.max_age_ns}
        bootstrap = previous if not gap and len(recent) == len(positions) == 1 else 0
        candidates = {sort_id: self.candidates(stamp, position, recent, bootstrap)
                      for sort_id, position in positions.items()}
        claims = Counter(identity for options in candidates.values() for identity in options)
        for sort_id, position in sorted(positions.items()):
            options = candidates[sort_id]
            if len(options) == 1 and claims[options[0]] == 1:
                track = self.tracks[options[0]]
            elif not options and all(
                    dist(position, other) > CLUSTER_DISTANCE_M
                    for key, other in positions.items() if key != sort_id):
                track = self.tracks[f'motion-{self.next_id}'] = Track(sort_id)
                self.next_id += 1
            else:
                continue
            continuous = not gap and track.stamp == previous
            track.observe(sort_id, stamp, position, self.cadence_ns, continuous)

    def snapshot(self, now, invalidate=False, hold_velocity=True):
        """Report measured position and separately bounded OLS hold validity."""
        expired = [key for key, track in self.tracks.items()
                   if now - track.stamp > self.retention_ns]
        for key in expired:
            del self.tracks[key]
        rows = []
        for identity, track in sorted(self.tracks.items()):
            age = now - track.stamp
            current = track.stamp == self.last_scan_stamp
            fresh = not invalidate and 0 <= age <= self.max_age_ns
            valid = fresh and current
            held = (hold_velocity and fresh and not current
                    and track.velocity_stamp_ns == track.stamp)
            rows.append(Snapshot(
                identity, track.sort_id, track.position, track.velocity, track.direct,
                track.diagnostic, valid, held or (valid and bool(track.sample_stamps)),
                valid and bool(track.direct_stamps), valid and bool(track.diagnostic_stamps),
                track.stamp, age, track.sample_stamps, track.direct_stamps,
                track.diagnostic_stamps, track.velocity_stamp_ns, held,
            ))
        return rows
