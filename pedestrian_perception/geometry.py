"""Source-time LiDAR projection for the calibrated, undistorted R2 camera."""

from dataclasses import dataclass
from math import cos, isclose, isfinite, sin

import numpy as np
from scipy.spatial.transform import Rotation

from .association import Beam
from .state import SECOND


def stamp_ns(stamp):
    """Return nanoseconds from a ROS time or duration."""
    return stamp.sec * SECOND + stamp.nanosec


@dataclass(frozen=True)
class Calibration:
    """Pinhole intrinsics from the source camera's CameraInfo."""

    width: int
    height: int
    matrix: tuple

    @classmethod
    def from_message(cls, message):
        """Validate the full-resolution, zero-distortion R2 camera contract."""
        k = tuple(message.k)
        roi = message.roi
        if (message.width <= 1 or message.height <= 1 or not all(map(isfinite, k))
                or not all(isfinite(d) and d == 0 for d in message.d)
                or message.distortion_model not in ('', 'plumb_bob', 'rational_polynomial')
                or k[0] <= 0 or k[4] <= 0 or k[6:] != (0., 0., 1.)
                or message.binning_x > 1 or message.binning_y > 1
                or any((roi.x_offset, roi.y_offset, roi.width, roi.height, roi.do_rectify))):
            return None
        return cls(message.width, message.height, k)


def read_tracks(message):
    """Validate the existing typed SORT batch and return axis-aligned boxes."""
    boxes = {}
    for detection in message.detections:
        if (not detection.id or detection.id in boxes
                or detection.header != message.header):
            return None
        box = detection.bbox
        x, y = box.center.position.x, box.center.position.y
        w, h = box.size_x / 2, box.size_y / 2
        if (not all(map(isfinite, (x, y, w, h, box.center.theta)))
                or w <= 0 or h <= 0 or abs(box.center.theta) > 1e-6):
            return None
        boxes[detection.id] = (x - w, y - h, x + w, y + h)
    return boxes


def read_scan(scan, start):
    """Return each finite in-range beam at its true acquisition time."""
    count = len(scan.ranges)
    if (not count or not all(map(isfinite, (scan.angle_min, scan.angle_max, scan.angle_increment,
                                            scan.time_increment, scan.range_min, scan.range_max)))
            or scan.angle_increment == 0 or scan.time_increment < 0
            or not 0 <= scan.range_min < scan.range_max
            or not isclose(scan.angle_min + (count - 1) * scan.angle_increment, scan.angle_max,
                           rel_tol=1e-5, abs_tol=1e-5)):
        return None
    rays = []
    for index, distance in enumerate(scan.ranges):
        if isfinite(distance) and scan.range_min <= distance <= scan.range_max:
            theta = scan.angle_min + index * scan.angle_increment
            rays.append((index, start + round(index * scan.time_increment * SECOND),
                         (distance * cos(theta), distance * sin(theta), 0.)))
    return rays


def pose(transform):
    """Convert a ROS transform to a finite rigid pose."""
    t, q = transform.translation, transform.rotation
    translation = np.array([t.x, t.y, t.z])
    if not np.isfinite(translation).all() or not all(map(isfinite, (q.x, q.y, q.z, q.w))):
        raise ValueError('non-finite transform')
    return translation, Rotation.from_quat([q.x, q.y, q.z, q.w])


def project(rays, scan_poses, optical_pose, calibration, tracks, near, far):
    """Transform each ray into odom, then use the camera pose at image time."""
    if not rays:
        return []
    origins = []
    for _, stamp, point in rays:
        origin, rotation = scan_poses[stamp]
        origins.append(origin + rotation.apply(point))
    xyz = np.asarray(origins)
    origin, rotation = optical_pose
    optical = rotation.inv().apply(xyz - origin)
    projected = optical @ np.asarray(calibration.matrix).reshape(3, 3).T
    beams = []
    for (index, _, _), point, coordinates in zip(rays, xyz, projected):
        depth = coordinates[2]
        candidates = ()
        if near <= depth <= far:
            u, v = coordinates[:2] / depth
            if 0 <= u < calibration.width and 0 <= v < calibration.height:
                candidates = tuple(sorted(key for key, (x1, y1, x2, y2) in tracks.items()
                                          if x1 <= u <= x2 and y1 <= v <= y2))
        beams.append(Beam(index, (float(point[0]), float(point[1])), candidates))
    return beams
