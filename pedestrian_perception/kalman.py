"""Experimental constant-velocity state estimation downstream of motion association."""

from dataclasses import dataclass, replace
from math import dist, hypot

import numpy as np

from .state import SECOND, ZERO

IDENTITY = np.eye(4)


@dataclass(frozen=True)
class FilterConfig:
    """Measurement sigmas, hysteresis and continuous acceleration noise in SI units."""

    sigma_x: float
    sigma_y: float
    enter_speed: float
    exit_speed: float
    acceleration_density: float
    gate: float = 9.21034037197618
    dwell_s: float = .4

    def __post_init__(self):
        values = tuple(vars(self).values())
        if not all(np.isfinite(values)) or min(values) <= 0 or self.enter_speed >= self.exit_speed:
            raise ValueError('invalid KF noise, gate or speed thresholds')


def transition(dt, density):
    """Return the CV transition and integrated continuous white acceleration noise."""
    f = IDENTITY.copy()
    f[:2, 2:] *= 0.
    f[0, 2] = f[1, 3] = dt
    axes = np.eye(2)
    q = density**2 * np.block([
        [dt**3 / 3 * axes, dt**2 / 2 * axes],
        [dt**2 / 2 * axes, dt * axes],
    ])
    return f, q


class Filter:
    """A posterior at the last accepted source time plus its rejection history."""

    def __init__(self, config, stamp, position):
        self.config = config
        self.r = np.diag([config.sigma_x**2, config.sigma_y**2])
        self.seed(stamp, position)

    def seed(self, stamp, position):
        """Initialize from a real observation with unobserved velocity."""
        self.stamp = self.start = stamp
        self.x = np.array([*position, 0., 0.])
        self.p = np.diag([self.r[0, 0], self.r[1, 1], .5**2, .5**2])
        self.count = 1
        self.rejected = None
        self.quiet_since = None
        self.stationary = False

    def predict(self, stamp):
        """Project from the last posterior without altering accepted state time."""
        f, q = transition((stamp - self.stamp) / SECOND, self.config.acceleration_density)
        return f @ self.x, f @ self.p @ f.T + q

    def observe(self, stamp, position):
        """Apply the innovation test and optionally recover on consistent rejects."""
        predicted, covariance = self.predict(stamp)
        residual = np.asarray(position) - predicted[:2]
        s = covariance[:2, :2] + self.r
        nis = float(residual @ np.linalg.solve(s, residual))
        if nis > self.config.gate:
            previous = self.rejected
            self.rejected = (stamp, position)
            self.quiet_since = None
            if previous is None:
                return 'rejected', nis
            dt = (stamp - previous[0]) / SECOND
            if dt > .30 or dist(position, previous[1]) > 2. * dt:
                return 'rejected', nis
            self.seed(*previous)
            predicted, covariance = self.predict(stamp)
            residual = np.asarray(position) - predicted[:2]
            s = covariance[:2, :2] + self.r
            event = 'reacquired'
        else:
            event = 'accepted'
        gain = np.linalg.solve(s, covariance[:, :2].T).T
        self.x = predicted + gain @ residual
        corrected = IDENTITY.copy()
        corrected[:, :2] -= gain
        self.p = corrected @ covariance @ corrected.T + gain @ self.r @ gain.T
        self.p = (self.p + self.p.T) / 2
        self.count += 1
        self.stamp = stamp
        self.rejected = None
        self.update_stationary()
        return event, nis

    def update_stationary(self):
        """Apply hysteresis at source time while preserving the estimated state."""
        speed = hypot(*self.x[2:])
        if self.stationary:
            self.stationary = speed < self.config.exit_speed
            self.quiet_since = None
        elif speed >= self.config.enter_speed:
            self.quiet_since = None
        elif self.quiet_since is None:
            self.quiet_since = self.stamp
        elif self.stamp - self.quiet_since >= round(self.config.dwell_s * SECOND):
            self.stationary = True

    def velocity(self):
        """Return the output velocity after the deadband."""
        return ZERO if self.stationary else tuple(map(float, self.x[2:]))


class KalmanHistory:
    """Preserve raw histories and keep one independent filter per motion ID."""

    def __init__(self, raw, config):
        self.raw = raw
        self.config = config
        self.filters = {}
        self.diagnostics = []

    @property
    def last_scan_stamp(self):
        """Return the original history's most recent committed scan stamp."""
        return self.raw.last_scan_stamp

    def reset(self):
        """Discard raw and filter histories together on a clock epoch reset."""
        self.raw.reset()
        self.filters.clear()
        self.diagnostics = []

    def update(self, stamp, positions):
        """Update only tracks assigned a real observation by the original history."""
        self.diagnostics = []
        if stamp <= self.last_scan_stamp:
            return
        self.raw.update(stamp, positions)
        for identity, track in self.raw.tracks.items():
            filt = self.filters.get(identity)
            current = track.stamp == stamp
            if current and (filt is None or stamp - filt.stamp > self.raw.max_age_ns):
                filt = self.filters[identity] = Filter(self.config, stamp, track.position)
                event, nis = 'initialized', None
            elif current:
                event, nis = filt.observe(stamp, track.position)
            elif filt is not None and stamp - filt.stamp <= self.raw.max_age_ns:
                event, nis = 'missing', None
                filt.quiet_since = None
                filt.rejected = None
            else:
                continue
            estimate, covariance = filt.predict(stamp)
            self.diagnostics.append({
                'track_id': identity, 'sort_id': track.sort_id,
                'measurement': track.position if current else None, 'event': event, 'nis': nis,
                'stamp_ns': stamp, 'accepted_stamp_ns': filt.stamp,
                'state': estimate.tolist(), 'covariance': covariance.tolist(),
                'stationary': filt.stationary, 'output_velocity': filt.velocity(),
            })

    def snapshot(self, now, invalidate=False, hold_velocity=True):
        """Publish at the accepted measurement time; keep diagnostics raw."""
        rows = self.raw.snapshot(now, invalidate, hold_velocity=False)
        result = []
        for row in rows:
            filt = self.filters[row.track_id]
            age = now - filt.stamp
            fresh = not invalidate and 0 <= age <= self.raw.max_age_ns
            current = fresh and filt.stamp == self.last_scan_stamp
            valid = current and filt.count >= 2
            result.append(replace(
                row, position=tuple(map(float, filt.x[:2])), velocity=filt.velocity(),
                position_valid=current, velocity_valid=valid, last_observation_stamp=filt.stamp,
                observation_age_ns=age, velocity_stamp_ns=filt.stamp if filt.count >= 2 else 0,
                velocity_held=False,
            ))
        for identity in self.filters.keys() - self.raw.tracks.keys():
            del self.filters[identity]
        return result
