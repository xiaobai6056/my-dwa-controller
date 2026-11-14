"""R2 A contiguous-beam clustering and pedestrian measurement geometry."""

from collections import defaultdict
from dataclasses import dataclass
from itertools import combinations
from math import dist, isfinite

# GEMS human_female_1, source collision-mesh footprint from R2.
BODY_DIAMETER_M = 0.6787006861587888
CIRCLE_RADIUS_M = 0.3515418249751633
CLUSTER_DISTANCE_M = 0.12
MIN_POINTS = 3
# Frozen exploratory R2 overlap rule. Require an IPM measurement for every candidate.
IPM_MAX_DISTANCE_M = 0.500
IPM_MIN_MARGIN_M = 1.109


@dataclass(frozen=True)
class Beam:
    """A scan return in odom with its set of camera box candidates."""

    index: int
    xy: tuple[float, float]
    candidates: tuple[str, ...]


@dataclass(frozen=True)
class Group:
    """One contiguous R2 subcluster and its frame-local association."""

    beams: tuple[int, ...]
    candidates: tuple[str, ...]
    track_id: str | None
    reason: str


def aabb_center(points):
    """Return the midpoint of the odom-aligned bounding box."""
    x, y = zip(*points)
    return (min(x) + max(x)) / 2, (min(y) + max(y)) / 2


def trimmed_centroid(points):
    """Average each coordinate after trimming floor(n/5) values per end."""
    trim = len(points) // 5
    end = len(points) - trim
    return tuple(sum(sorted(axis)[trim:end]) / (end - trim) for axis in zip(*points))


POSITION_ESTIMATORS = {'aabb': aabb_center, 'trimmed_centroid': trimmed_centroid}


def associate(beams, ipm, cluster_distance=CLUSTER_DISTANCE_M):
    """Run R2 A within each unique box-candidate set, retaining small groups."""
    buckets = defaultdict(list)
    for beam in beams:
        buckets[beam.candidates].append(beam)

    groups = []
    for candidates, members in buckets.items():
        runs = []
        previous = None
        for beam in sorted(members, key=lambda member: member.index):
            if (previous is None or beam.index != previous.index + 1
                    or dist(beam.xy, previous.xy) > cluster_distance + 1e-12):
                runs.append([])
            runs[-1].append(beam)
            previous = beam
        for run in runs:
            identity, reason = _assign(candidates, run, ipm)
            groups.append(Group(tuple(beam.index for beam in run), candidates, identity, reason))
    return sorted(groups, key=lambda group: group.beams[0])


def _assign(candidates, run, ipm):
    if not candidates:
        return None, 'no_candidate'
    if len(candidates) == 1:
        return candidates[0], 'unique_box'
    if any(key not in ipm or not all(map(isfinite, ipm[key])) for key in candidates):
        return None, 'ipm_incomplete'
    center = aabb_center([beam.xy for beam in run])
    ranked = sorted((dist(center, ipm[key]), key) for key in candidates)
    (nearest, identity), (runner_up, _) = ranked[:2]
    if nearest > IPM_MAX_DISTANCE_M or runner_up - nearest < IPM_MIN_MARGIN_M:
        return None, 'overlap_unresolved'
    return identity, 'overlap_ipm'


def estimate_positions(beams, groups, min_points=MIN_POINTS, *, estimator='aabb'):
    """Pool a track's groups when the total support fits the R2 body diameter."""
    if estimator not in POSITION_ESTIMATORS:
        raise ValueError(f'unknown position_estimator: {estimator}')
    center = POSITION_ESTIMATORS[estimator]
    by_index = {beam.index: beam.xy for beam in beams}
    pooled = defaultdict(list)
    for group in groups:
        if group.track_id is not None:
            pooled[group.track_id].extend(by_index[index] for index in group.beams)
    return {
        identity: center(points)
        for identity, points in pooled.items()
        if len(points) >= min_points
        and all(dist(a, b) <= BODY_DIAMETER_M + 1e-12 for a, b in combinations(points, 2))
    }
