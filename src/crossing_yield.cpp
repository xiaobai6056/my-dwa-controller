#include "my_dwa_controller/crossing_yield.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace my_dwa_controller
{
namespace
{
constexpr int64_t kScanFreshnessNs = 350000000;
constexpr int64_t kRegionEmptyTimeoutNs = 4000000000;
}  // namespace

bool CrossingYield::canApproach(
  const std::vector<Pose2D> & path, const MotionRollout & current_brake,
  const MotionRollout & candidate_brake, double horizon, double step,
  double speed_bound) const
{
  if (!region_pending_) {return true;}
  if (stop_latched_) {return false;}
  if (path.size() < 2 || !(current_brake.stop_time <= horizon) ||
    !(candidate_brake.stop_time <= horizon) || !(step > 0) || !(speed_bound >= 0) ||
    candidate_brake.trajectory.command.v <= 0)
  {
    return false;
  }
  // The entry is an infinite half-plane: moving sideways cannot bypass it.
  // The existing .27 envelope already contains footprint plus safety margin.
  // Reserve a full sample's translation for excursions between sampled poses.
  const double reserve = speed_bound * step;
  const auto forward = [&](const Pose2D & p) {return p.x * tangent_x_ + p.y * tangent_y_;};
  const auto lateral = [&](const Pose2D & p) {
      return std::abs(p.x * normal_x_ + p.y * normal_y_ - path_normal_center_);
    };
  double previous = forward(path.front());
  for (const auto & p : path) {
    const double s = forward(p);
    if (!(lateral(p) <= .27) || !(s >= previous)) {return false;}
    previous = s;
  }
  for (const auto * rollout : {&current_brake, &candidate_brake}) {
    if (rollout->trajectory.poses.empty()) {return false;}
    // Keep the existing sample allowance; only candidate braking needs the fixed reserve.
    const double approach_reserve = rollout == &candidate_brake ? .20 : 0.;
    previous = forward(rollout->trajectory.poses.front());
    for (const auto & p : rollout->trajectory.poses) {
      const double s = forward(p);
      if (!(s + reserve + approach_reserve < region_low_ - .27) || !(lateral(p) + reserve < .27) ||
        !(s >= previous)) {return false;}
      previous = s;
    }
  }
  return previous > forward(candidate_brake.trajectory.poses.front());
}

bool CrossingYield::latchHold(bool hold)
{
  if (region_pending_ && hold) {stop_latched_ = true;}
  return region_pending_ && stop_latched_;
}

void CrossingYield::reset() {*this = CrossingYield{};}

std::vector<std::size_t> CrossingYield::targetClusters(const ScanSnapshot & scan) const
{
  if (!region_pending_ || cluster_index_ < 0) {return {};}
  const int64_t stamp = int64_t(scan.stamp.sec) * 1000000000 + scan.stamp.nanosec;
  int index = -1;
  if (stamp == last_stamp_) {
    index = cluster_index_;
  } else if (stamp > last_stamp_ && scan.previous_stamp_ns == last_stamp_) {
    for (std::size_t i = 0; i < scan.obstacles.size(); ++i) {
      if (scan.obstacles[i].previous_cluster_index == cluster_index_) {index = i;break;}
    }
  }
  if (index < 0 || std::size_t(index) >= scan.obstacles.size()) {return {};}
  std::vector<std::size_t> result{std::size_t(index)};
  if (scan.obstacles.size() > 1 && !scan.ranges.empty() &&
    (index == 0 || std::size_t(index) == scan.obstacles.size() - 1))
  {
    const auto & first = scan.obstacles.front();
    const auto & last = scan.obstacles.back();
    if (first.first_beam == 0 && last.last_beam == scan.ranges.size() - 1 &&
      !first.points.empty() && !last.points.empty() &&
      std::hypot(
        first.points.front()[0] - last.points.back()[0],
        first.points.front()[1] - last.points.back()[1]) <= scan.cluster_distance)
    {
      result = {0, scan.obstacles.size() - 1};
    }
  }
  return result;
}

void CrossingYield::fillDecision(
  YieldDecision & result, const std::vector<Pose2D> &, const Trajectory & brake) const
{
  result.hold = region_pending_;
  result.occupied = ever_occupied_;
  result.clear_scans = clear_scans_;
  result.evidence_rank = evidence_rank_;
  result.entry = region_low_ - .27;
  result.exit = region_high_ + .27;
  double end = -std::numeric_limits<double>::infinity();
  for (const auto & p : brake.poses) {
    end = std::max(end, p.x * tangent_x_ + p.y * tangent_y_);
  }
  result.margin = result.entry - end;
  result.safe_stop = !brake.poses.empty() && result.margin > 0;
}

namespace
{
// The original crossing predicate, shared by admission and the handover veto.
std::optional<std::size_t> crossingCluster(
  const ScanSnapshot & scan, const std::vector<Pose2D> & path, const Pose2D & robot,
  const std::vector<std::size_t> & excluded)
{
  if (path.size() < 2) {return std::nullopt;}
  const auto & a = path.front();
  const auto & b = path.back();
  const double length = std::hypot(b.x - a.x, b.y - a.y);
  if (!(length > .1)) {return std::nullopt;}
  const double tx = (b.x - a.x) / length, ty = (b.y - a.y) / length;
  const double nx = -ty, ny = tx;
  for (std::size_t i = 0; i < scan.obstacles.size(); ++i) {
    if (std::find(excluded.begin(), excluded.end(), i) != excluded.end()) {continue;}
    const auto & o = scan.obstacles[i];
    if (o.partial_motion_rank <= 0 || o.points.empty()) {continue;}
    double cx = 0, cy = 0;
    for (const auto & p : o.points) {
      cx += p[0];cy += p[1];
    }
    cx /= o.points.size();cy /= o.points.size();
    const double side = (cx - a.x) * nx + (cy - a.y) * ny;
    const double closing = -std::copysign(1., side) *
      (o.partial_motion_vx * nx + o.partial_motion_vy * ny);
    if (closing <= .10) {continue;}
    double low = std::numeric_limits<double>::infinity(), high = -low;
    for (const auto & p : o.points) {
      const double q = p[0] * tx + p[1] * ty;
      low = std::min(low, q);high = std::max(high, q);
    }
    if (high + .27 < robot.x * tx + robot.y * ty || low - .27 > b.x * tx + b.y * ty) {
      continue;
    }
    return i;
  }
  return std::nullopt;
}
}  // namespace

bool CrossingYield::hasOtherRisk(
  const ScanSnapshot & scan, const std::vector<Pose2D> & path, const Pose2D & robot,
  const std::vector<std::size_t> & excluded)
{
  return crossingCluster(scan, path, robot, excluded).has_value();
}

bool CrossingYield::establishRegion(
  const ScanSnapshot & scan, const std::vector<Pose2D> & path, const Pose2D & robot)
{
  const auto index = crossingCluster(scan, path, robot, {});
  if (!index) {return false;}
  const auto & a = path.front();
  const auto & b = path.back();
  const double length = std::hypot(b.x - a.x, b.y - a.y);
  const double tx = (b.x - a.x) / length, ty = (b.y - a.y) / length;
  double low = std::numeric_limits<double>::infinity(), high = -low;
  const auto & o = scan.obstacles[*index];
  for (const auto & p : o.points) {
    const double q = p[0] * tx + p[1] * ty;
    low = std::min(low, q);high = std::max(high, q);
  }
  region_pending_ = true;ever_occupied_ = false;clear_scans_ = 0;
  empty_duration_ns_ = 0;last_empty_stamp_ = 0;
  evidence_rank_ = o.partial_motion_rank;cluster_index_ = *index;
  tangent_x_ = tx;tangent_y_ = ty;normal_x_ = -ty;normal_y_ = tx;
  path_normal_center_ = -a.x * ty + a.y * tx;region_low_ = low;region_high_ = high;
  return true;
}

void CrossingYield::extendRegion(const ScanSnapshot & scan, int64_t previous_consumed_stamp)
{
  int next = -1;
  if (scan.previous_stamp_ns == previous_consumed_stamp && cluster_index_ >= 0) {
    for (size_t i = 0; i < scan.obstacles.size(); ++i) {
      if (scan.obstacles[i].previous_cluster_index == cluster_index_) {next = i;break;}
    }
  }
  // Loss of association freezes the existing union; it never cancels the risk.
  cluster_index_ = next;
  if (next >= 0) {
    const double old_low = region_low_, old_high = region_high_;
    for (const auto & p : scan.obstacles[next].points) {
      const double q = p[0] * tangent_x_ + p[1] * tangent_y_;
      region_low_ = std::min(region_low_, q);region_high_ = std::max(region_high_, q);
    }
    if (region_low_ != old_low || region_high_ != old_high) {
      empty_duration_ns_ = 0;last_empty_stamp_ = 0;
    }
  }
}

bool CrossingYield::observeOccupancy(const ScanSnapshot & scan)
{
  bool occupied = false;
  for (const auto & o : scan.obstacles) {
    for (const auto & p : o.points) {
      const double x = p[0] * tangent_x_ + p[1] * tangent_y_;
      const double y = p[0] * normal_x_ + p[1] * normal_y_;
      if (x >= region_low_ && x <= region_high_ &&
        std::abs(y - path_normal_center_) <= .27) {occupied = true;}
    }
  }
  ever_occupied_ = ever_occupied_ || occupied;
  return occupied;
}

CrossingYield::RegionObservation CrossingYield::observeRegion(const ScanSnapshot & scan) const
{
  if (scan.ranges.empty() || !(scan.angle_increment > 0)) {
    return RegionObservation::Unavailable;
  }
  bool covered = true;
  // Preserve the finite grid, but never treat a zero-iteration narrow grid as clear.
  const double first_x = region_low_ + .025 <= region_high_ ?
    region_low_ + .025 : (region_low_ + region_high_) / 2;
  for (double x = first_x; x <= region_high_; x += .05) {
    for (double y = path_normal_center_ - .25; y <= path_normal_center_ + .25001; y += .05) {
      const double px = x * tangent_x_ + y * normal_x_;
      const double py = x * tangent_y_ + y * normal_y_;
      const double dx = px - scan.laser_pose[0], dy = py - scan.laser_pose[1];
      const double distance = std::hypot(dx, dy);
      double angle = std::atan2(dy, dx) - scan.laser_pose[2] - scan.angle_min;
      const double tau = 2 * std::acos(-1.);
      angle = std::fmod(angle + 2 * tau, tau);
      long index = std::lround(angle / scan.angle_increment);
      if (index == long(scan.ranges.size()) &&
        std::abs(scan.ranges.size() * scan.angle_increment - tau) < .05) {index = 0;}
      if (index < 0 || index >= long(scan.ranges.size())) {
        covered = false;continue;
      }
      double range = scan.ranges[index];
      if (std::isnan(range) || range < scan.range_min) {
        covered = false;continue;
      }
      const double beam = scan.laser_pose[2] + scan.angle_min + index * scan.angle_increment;
      if (std::abs(-std::sin(beam) * dx + std::cos(beam) * dy) > .025) {
        covered = false;continue;
      }
      range = std::min(range, scan.range_max);
      if (distance >= range - .03) {return RegionObservation::Blocked;}
    }
  }
  return covered ? RegionObservation::Clear : RegionObservation::Unavailable;
}

bool CrossingYield::pathIntersectsRegion(const std::vector<Pose2D> & path) const
{
  if (path.empty()) {return false;}
  const double low[] = {region_low_ - .27, path_normal_center_ - .27};
  const double high[] = {region_high_ + .27, path_normal_center_ + .27};
  double previous[] = {
    path.front().x * tangent_x_ + path.front().y * tangent_y_,
    path.front().x * normal_x_ + path.front().y * normal_y_};
  if (previous[0] >= low[0] && previous[0] <= high[0] &&
    previous[1] >= low[1] && previous[1] <= high[1]) {return true;}
  // Clip each actual polyline segment in the frozen tangent/normal basis.
  for (size_t i = 1; i < path.size(); ++i) {
    const double current[] = {
      path[i].x * tangent_x_ + path[i].y * tangent_y_,
      path[i].x * normal_x_ + path[i].y * normal_y_};
    double enter = 0, leave = 1;
    for (int axis = 0; axis < 2; ++axis) {
      const double delta = current[axis] - previous[axis];
      if (delta == 0) {
        if (previous[axis] < low[axis] || previous[axis] > high[axis]) {leave = -1;break;}
      } else {
        double a = (low[axis] - previous[axis]) / delta;
        double b = (high[axis] - previous[axis]) / delta;
        if (a > b) {std::swap(a, b);}
        enter = std::max(enter, a);leave = std::min(leave, b);
      }
    }
    if (enter <= leave) {return true;}
    previous[0] = current[0];previous[1] = current[1];
  }
  return false;
}

bool CrossingYield::updateEmptyDuration(
  RegionObservation observation, bool occupied, int64_t stamp)
{
  if (occupied || observation == RegionObservation::Blocked) {
    empty_duration_ns_ = 0;last_empty_stamp_ = 0;
  } else if (observation == RegionObservation::Unavailable) {
    last_empty_stamp_ = 0;  // Pause credit; reconnect without bridging the blind interval.
  } else {
    const int64_t elapsed = stamp - last_empty_stamp_;
    if (last_empty_stamp_ > 0 && elapsed > 0 && elapsed <= kScanFreshnessNs) {
      empty_duration_ns_ += elapsed;
    }
    last_empty_stamp_ = stamp;
  }
  return empty_duration_ns_ >= kRegionEmptyTimeoutNs;
}

bool CrossingYield::canRelease(const Velocity2D & velocity, bool candidate_admitted) const
{
  return ever_occupied_ && clear_scans_ >= 2 && candidate_admitted &&
         std::abs(velocity.v) < .02 && std::abs(velocity.omega) < .05;
}

YieldDecision CrossingYield::update(
  const ScanSnapshot * scan, const std::vector<Pose2D> & path,
  const Pose2D & robot, const Velocity2D & velocity, const Trajectory & brake,
  int64_t now_ns, bool candidate_admitted)
{
  YieldDecision result;
  const int64_t stamp = scan ?
    int64_t(scan->stamp.sec) * 1000000000 + scan->stamp.nanosec : 0;
  if (!scan || stamp <= 0 || now_ns < stamp || now_ns - stamp > kScanFreshnessNs) {
    last_empty_stamp_ = 0;
  } else if (stamp == last_stamp_) {
    // Admission/speed and the path may change while the source Scan repeats.
    if (region_pending_ && canRelease(velocity, candidate_admitted)) {
      region_pending_ = false;result.released = true;
    }
  } else {
    const int64_t previous_stamp = last_stamp_;
    last_stamp_ = stamp;
    if (!region_pending_ && path.size() > 1) {
      result.triggered = establishRegion(*scan, path, robot);
    } else if (region_pending_) {
      extendRegion(*scan, previous_stamp);
    }
    if (region_pending_) {
      const bool occupied = observeOccupancy(*scan);
      const auto observation = observeRegion(*scan);
      result.clear = observation == RegionObservation::Clear;
      clear_scans_ = ever_occupied_ && result.clear ? clear_scans_ + 1 : 0;
      if (updateEmptyDuration(observation, occupied, stamp)) {
        reset();
        last_stamp_ = stamp;  // Keep this source consumed after clearing associated state.
        result.released = true;
      } else if (canRelease(velocity, candidate_admitted)) {
        region_pending_ = false;result.released = true;
      }
    }
  }
  if (!region_pending_) {stop_latched_ = false;}
  fillDecision(result, path, brake);
  return result;
}
}  // namespace my_dwa_controller
