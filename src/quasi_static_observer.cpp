#include "my_dwa_controller/quasi_static_observer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace my_dwa_controller
{
namespace
{
constexpr int64_t kFreshNs = 350000000LL;
constexpr int64_t kWindowNs = 2000000000LL;
constexpr int64_t kSettleNs = 500000000LL;

bool fresh(int64_t stamp, int64_t reference)
{
  return stamp <= reference && reference - stamp <= kFreshNs;
}

Point median(const std::vector<Point> & points)
{
  Point result;
  for (std::size_t axis = 0; axis < 2; ++axis) {
    std::vector<double> values(points.size());
    std::transform(
      points.begin(), points.end(), values.begin(),
      [axis](const Point & point) {return point[axis];});
    std::sort(values.begin(), values.end());
    const auto middle = values.size() / 2;
    result[axis] = values.size() % 2 ? values[middle] :
      (values[middle - 1] + values[middle]) / 2.0;
  }
  return result;
}
}  // namespace

QuasiStaticObserver::Result QuasiStaticObserver::update(
  int64_t stamp, int64_t now, const std::vector<Point> & points,
  bool observation_valid, bool robot_stopped, bool moving)
{
  const bool valid_points = points.size() >= 3 && std::all_of(
    points.begin(), points.end(), [](const Point & point) {
      return std::isfinite(point[0]) && std::isfinite(point[1]);
    });
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Point center = valid_points ? median(points) : Point{nan, nan};
  const bool valid_position = valid_points && std::isfinite(center[0]) && std::isfinite(center[1]);
  const char * reason = nullptr;
  if (!robot_stopped) {
    reason = "robot_not_settled";
  } else if (!fresh(stamp, now)) {
    reason = "stale_source";
  } else if (!observation_valid) {
    reason = "observation_invalid";
  } else if (moving) {
    reason = "tracker_motion";
  } else if (!valid_position) {
    reason = "position_invalid";
  }

  bool is_new = false;
  if (reason) {
    window_.clear();
    have_last_ = true;
    last_stamp_ = stamp;
  } else if (have_last_ && stamp == last_stamp_) {
    reason = "source_not_new";
  } else {
    if (have_last_ && stamp < last_stamp_) {
      reason = "clock_reversed";
    } else if (have_last_ && stamp - last_stamp_ > kFreshNs) {reason = "source_gap";}
    if (reason) {window_.clear();}
    have_last_ = true;
    last_stamp_ = stamp;
    is_new = true;
    window_.emplace_back(stamp, center);
    const auto cutoff = stamp - kWindowNs;
    // Retain one sample at/before the boundary to prove the full time span.
    while (window_.size() > 1 && window_[1].first <= cutoff) {window_.pop_front();}
  }

  double diameter = 0.0;
  for (const auto & a : window_) {
    for (const auto & b : window_) {
      diameter =
        std::max(diameter, std::hypot(a.second[0] - b.second[0], a.second[1] - b.second[1]));
    }
  }
  if (!std::isfinite(diameter)) {
    window_.clear();
    diameter = 0.0;
    is_new = false;
    reason = "position_invalid";
  }
  const int64_t span = window_.empty() ? 0 : stamp - window_.front().first;
  const bool mature = window_.size() >= 8 && span >= kWindowNs;
  const bool permission = is_new && mature && diameter <= 0.05 + 1e-12;
  if (!reason) {
    reason = permission ? "stable" : mature ? "range_exceeded" : "window_immature";
  }
  return {permission, window_.size(), span, diameter, center, reason};
}

void QuasiStaticObserver::reset()
{
  window_.clear();
  have_last_ = false;
  last_stamp_ = 0;
}

void RobotStopObserver::feed(int64_t stamp, double vx, double vy, double w)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (have_last_ && (stamp <= last_stamp_ || stamp - last_stamp_ > kFreshNs)) {
    quiet_ = false;
    // Reversed clocks start a new source timeline; equal sources replace their snapshot.
    if (stamp < last_stamp_) {
      history_.clear();
    } else if (stamp == last_stamp_ && !history_.empty()) {
      history_.pop_back();
    }
  }
  const bool quiet = std::isfinite(vx) && std::isfinite(vy) && std::isfinite(w) &&
    std::hypot(vx, vy) < 0.01 && std::abs(w) < 0.05;
  if (quiet && !quiet_) {quiet_since_ = stamp;}
  quiet_ = quiet;
  have_last_ = true;
  last_stamp_ = stamp;
  history_.push_back({stamp, quiet_since_, quiet_});
  // A usable snapshot is at most 350 ms old. One second leaves margin for
  // callback ordering; 256 entries also bounds memory for unusually fast odom.
  // If truncation removes the needed causal sample, stoppedAt refuses permission.
  // Current Scans recover as soon as a usable causal snapshot is available.
  while (!history_.empty() &&
    (stamp - history_.front().stamp > 1000000000LL || history_.size() > 256))
  {
    history_.pop_front();
  }
}

bool RobotStopObserver::covers(int64_t scan_stamp) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return !history_.empty() && history_.back().stamp >= scan_stamp;
}

bool RobotStopObserver::stoppedAt(int64_t scan_stamp, int64_t now) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto it = history_.rbegin(); it != history_.rend(); ++it) {
    if (it->stamp > scan_stamp) {continue;}
    return fresh(it->stamp, scan_stamp) && fresh(it->stamp, now) && it->quiet &&
           scan_stamp - it->quiet_since >= kSettleNs;
  }
  return false;
}

void RobotStopObserver::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  history_.clear();
  have_last_ = false;
  quiet_ = false;
  last_stamp_ = quiet_since_ = 0;
}
}  // namespace my_dwa_controller
