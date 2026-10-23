#include "my_dwa_controller/scan_obstacle_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace my_dwa_controller
{
namespace
{
double seconds(const builtin_interfaces::msg::Time & t)
{
  return t.sec + t.nanosec * 1e-9;
}

tf2::Transform interpolate(const tf2::Transform & a, const tf2::Transform & b, double u)
{
  return tf2::Transform(
    a.getRotation().slerp(b.getRotation(), u),
    a.getOrigin().lerp(b.getOrigin(), u));
}
}  // namespace

ScanObstacleTracker::ScanObstacleTracker(
  std::shared_ptr<tf2_ros::Buffer> tf,
  ScanTrackerConfig config)
: tf_(std::move(tf)), config_(std::move(config))
{
  if (!tf_ || config_.fixed_frame.empty() || config_.topic.empty()) {
    throw std::invalid_argument("scan tracker needs TF, a fixed frame and a topic");
  }
  for (double v : {config_.cluster_gap, config_.max_cluster_size, config_.association_gate,
      config_.fit_window, config_.max_age, config_.retention})
  {
    if (!std::isfinite(v) || v <= 0.0) {
      throw std::invalid_argument("scan tracker lengths and durations must be positive");
    }
  }
  if (config_.retention < config_.max_age) {
    throw std::invalid_argument("scan retention must cover freshness interval");
  }
}

void ScanObstacleTracker::start(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  stop();
  inbox_ = std::make_shared<Inbox>();
  const auto inbox = inbox_;
  subscription_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    config_.topic, rclcpp::SensorDataQoS(),
    [inbox](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
      std::lock_guard<std::mutex> lock(inbox->mutex);
      if (!inbox->accepting) {return;}
      if (inbox->scans.size() == 5) {inbox->scans.pop_front(); ++inbox->dropped;}
      inbox->scans.push_back(*scan);
    });
}

void ScanObstacleTracker::stop()
{
  {
    std::lock_guard<std::mutex> lock(inbox_->mutex);
    inbox_->accepting = false;
  }
  subscription_.reset();
  reset();
}

void ScanObstacleTracker::enqueue(const sensor_msgs::msg::LaserScan & scan)
{
  std::lock_guard<std::mutex> lock(inbox_->mutex);
  if (inbox_->scans.size() == 5) {inbox_->scans.pop_front(); ++inbox_->dropped;}
  inbox_->scans.push_back(scan);
}

void ScanObstacleTracker::reset()
{
  std::lock_guard<std::mutex> lock(inbox_->mutex);
  inbox_->scans.clear();
  inbox_->dropped = 0;
  tracks_.clear();
  stamp_ = last_query_ = 0.0;
  sequence_ = tf_failures_ = 0;
  next_id_ = 1;
  rays_.clear();
}

bool ScanObstacleTracker::process(const sensor_msgs::msg::LaserScan & scan)
{
  const double t = seconds(scan.header.stamp);
  if (t <= stamp_ || scan.header.frame_id.empty() || scan.ranges.empty() ||
    !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_increment) ||
    scan.angle_increment <= 0.0 || !std::isfinite(scan.time_increment) ||
    scan.time_increment < 0.0 || !std::isfinite(scan.range_min) ||
    !std::isfinite(scan.range_max) || scan.range_min < 0.0 || scan.range_max <= scan.range_min)
  {
    return true;  // Discard malformed/duplicate input without advancing freshness.
  }
  tf2::Transform first, last;
  try {
    const auto time = tf2::timeFromSec(t);
    tf2::fromMsg(
      tf_->lookupTransform(config_.fixed_frame, scan.header.frame_id, time).transform,
      first);
    const auto end = time + tf2::durationFromSec(
      (scan.ranges.size() - 1) * scan.time_increment);
    tf2::fromMsg(
      tf_->lookupTransform(config_.fixed_frame, scan.header.frame_id, end).transform,
      last);
  } catch (const tf2::TransformException &) {
    ++tf_failures_;
    return false;  // TF may lag the scan; retry from bounded inbox next cycle.
  }
  rays_.clear();
  std::vector<std::vector<Pose2D>> groups;
  std::vector<Pose2D> group;
  const auto finish = [&]() {
      if (!group.empty()) {groups.push_back(std::move(group)); group.clear();}
    };
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      finish();
      continue;
    }
    const double angle = scan.angle_min + i * scan.angle_increment;
    const auto tr = interpolate(
      first, last, static_cast<double>(i) /
      std::max<std::size_t>(1, scan.ranges.size() - 1));
    const auto p = tr * tf2::Vector3(range * std::cos(angle), range * std::sin(angle), 0.0);
    if (!group.empty() && std::hypot(p.x() - group.back().x, p.y() - group.back().y) >
      config_.cluster_gap) {finish();}
    Pose2D point;
    point.x = p.x(); point.y = p.y();
    ScanRay ray;
    ray.origin.x = tr.getOrigin().x(); ray.origin.y = tr.getOrigin().y();
    ray.end = point;
    ray.min_range = scan.range_min;
    rays_.push_back(ray);
    group.push_back(point);
  }
  finish();
  // A 360 degree scan has a seam; do not split a single obstacle at angle zero.
  if (groups.size() > 1 && scan.ranges.size() * scan.angle_increment > 6.2 &&
    std::isfinite(scan.ranges.front()) && std::isfinite(scan.ranges.back()) &&
    std::hypot(
      groups.front().front().x - groups.back().back().x,
      groups.front().front().y - groups.back().back().y) <= config_.cluster_gap)
  {
    groups.front().insert(groups.front().end(), groups.back().begin(), groups.back().end());
    groups.pop_back();
  }
  std::vector<Cluster> clusters;
  for (const auto & points : groups) {
    if (points.size() < 3) {continue;}
    double xmin = INFINITY, ymin = INFINITY, xmax = -INFINITY, ymax = -INFINITY;
    for (const auto & p : points) {
      xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
      ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
    }
    const double size = std::hypot(xmax - xmin, ymax - ymin);
    if (size < 0.06 || size > config_.max_cluster_size) {continue;}
    clusters.push_back(
      {0.5 * (xmin + xmax), 0.5 * (ymin + ymax),
        std::max(0.08, 0.5 * size), xmax - xmin, ymax - ymin});
  }
  associate(clusters, t);
  stamp_ = t;
  ++sequence_;
  return true;
}

void ScanObstacleTracker::fit(Track & track)
{
  auto & h = track.history;
  const double t = h.back().t;
  while (h.size() > 1 && t - h.front().t > config_.fit_window + 1e-6) {h.pop_front();}
  auto & v = track.value;
  v.samples = h.size();
  v.span = t - h.front().t;
  v.confidence = 0.0;
  v.motion = ScanMotion::Unknown;
  if (h.size() < 4 || v.span < .55) {return;}
  double min_wx = INFINITY, max_wx = 0.0, min_wy = INFINITY, max_wy = 0.0;
  for (const auto & p : h) {
    min_wx = std::min(min_wx, p.width_x); max_wx = std::max(max_wx, p.width_x);
    min_wy = std::min(min_wy, p.width_y); max_wy = std::max(max_wy, p.width_y);
  }
  v.support_change = std::max(max_wx - min_wx, max_wy - min_wy);
  double mt = 0, mx = 0, my = 0;
  for (const auto & p : h) {
    mt += p.t - t; mx += p.x; my += p.y;
  }
  mt /= h.size(); mx /= h.size(); my /= h.size();
  double tt = 0, tx = 0, ty = 0;
  for (const auto & p : h) {
    const double dt = p.t - t - mt;
    tt += dt * dt; tx += dt * (p.x - mx); ty += dt * (p.y - my);
  }
  v.vx = tx / tt; v.vy = ty / tt;
  double error = 0;
  for (const auto & p : h) {
    const double dt = p.t - t - mt;
    error += std::pow(p.x - mx - v.vx * dt, 2) + std::pow(p.y - my - v.vy * dt, 2);
  }
  v.residual = std::sqrt(error / h.size());
  v.confidence = std::clamp(1.0 - v.residual / .10, 0.0, 1.0) *
    std::min(1.0, v.span / .6);
  if (std::hypot(v.vx, v.vy) > 1.5) {v.confidence = 0.0;}
  // A sliding visible edge can fit a line perfectly while the physical surface
  // stays fixed. Both support extents must stay consistent over the same window.
  if (v.confidence >= .65 && v.support_change <= .10) {
    v.motion = std::hypot(v.vx, v.vy) >= .08 ? ScanMotion::Moving : ScanMotion::Stationary;
  }
}

void ScanObstacleTracker::associate(const std::vector<Cluster> & clusters, double t)
{
  std::vector<bool> used(clusters.size(), false);
  // Nearest-neighbour with forward AND reverse ambiguity checks, no forced match.
  for (auto & track : tracks_) {
    auto & v = track.value;
    v.observed = false;
    const double age = t - v.stamp;
    if (age > config_.retention) {continue;}
    std::size_t best = clusters.size();
    double distance = config_.association_gate, second = INFINITY;
    const double px = v.x + v.vx * age, py = v.y + v.vy * age;
    for (std::size_t i = 0; i < clusters.size(); ++i) {
      if (used[i] || std::abs(clusters[i].radius - v.radius) > .18) {continue;}
      const double d = std::hypot(clusters[i].x - px, clusters[i].y - py);
      if (d < distance) {second = distance; distance = d; best = i;} else {
        second = std::min(second, d);
      }
    }
    if (best == clusters.size() || second - distance < .08) {continue;}
    bool ambiguous = false;
    for (const auto & other : tracks_) {
      if (other.value.id == v.id || t - other.value.stamp > config_.retention) {continue;}
      const double dt = t - other.value.stamp;
      const double d = std::hypot(
        clusters[best].x - other.value.x - other.value.vx * dt,
        clusters[best].y - other.value.y - other.value.vy * dt);
      if (d < distance + .08) {ambiguous = true; break;}
    }
    if (ambiguous) {continue;}
    const auto & c = clusters[best];
    used[best] = true;
    v.x = c.x; v.y = c.y; v.radius = c.radius; v.stamp = t; v.observed = true;
    v.half_x = c.width_x / 2; v.half_y = c.width_y / 2;
    // Gaps are explicitly coasting; do not fit a discontinuous old history.
    if (age > config_.max_age) {track.history.clear();}
    track.history.push_back({t, c.x, c.y, c.width_x, c.width_y});
    fit(track);
  }
  for (std::size_t i = 0; i < clusters.size(); ++i) {
    if (used[i]) {continue;}
    const auto & c = clusters[i];
    Track track;
    track.value.id = next_id_++;
    track.value.x = c.x; track.value.y = c.y; track.value.radius = c.radius;
    track.value.stamp = t; track.value.observed = true;
    track.value.half_x = c.width_x / 2; track.value.half_y = c.width_y / 2;
    track.history.push_back({t, c.x, c.y, c.width_x, c.width_y});
    tracks_.push_back(std::move(track));
  }
  tracks_.erase(
    std::remove_if(
      tracks_.begin(), tracks_.end(), [&](const Track & track) {
        if (t - track.value.stamp <= config_.retention) {return false;}
        return true;
      }), tracks_.end());
}

ScanTrackSnapshot ScanObstacleTracker::snapshot(const std::string & target, double now)
{
  if (!std::isfinite(now)) {throw std::invalid_argument("scan query time must be finite");}
  if (now < last_query_) {reset();}
  last_query_ = now;
  std::deque<sensor_msgs::msg::LaserScan> pending;
  {
    std::lock_guard<std::mutex> lock(inbox_->mutex);
    pending.swap(inbox_->scans);
  }
  while (!pending.empty()) {
    const auto & scan = pending.front();
    if (seconds(scan.header.stamp) > now) {break;}
    if (now - seconds(scan.header.stamp) > config_.retention) {pending.pop_front(); continue;}
    if (!process(scan)) {break;}
    pending.pop_front();
  }
  ScanTrackSnapshot result;
  {
    std::lock_guard<std::mutex> lock(inbox_->mutex);
    inbox_->scans.insert(inbox_->scans.begin(), pending.begin(), pending.end());
    while (inbox_->scans.size() > 5) {inbox_->scans.pop_front(); ++inbox_->dropped;}
    result.dropped = inbox_->dropped;
  }
  result.frame = target;
  result.stamp = stamp_;
  result.sequence = sequence_;
  result.tf_failures = tf_failures_;
  result.fresh = sequence_ > 0 && now >= stamp_ && now - stamp_ <= config_.max_age;
  result.unresolved = !result.fresh;
  tf2::Transform transform;
  transform.setIdentity();
  if (target != config_.fixed_frame) {
    try {
      tf2::fromMsg(
        tf_->lookupTransform(
          target, config_.fixed_frame,
          tf2::timeFromSec(now)).transform,
        transform);
    } catch (const tf2::TransformException &) {
      result.fresh = false;
      result.unresolved = true;
      return result;
    }
  }
  for (const auto & track : tracks_) {
    auto v = track.value;
    v.age = now - v.stamp;
    v.observed = v.observed && v.stamp == stamp_ && v.age <= config_.max_age;
    if (!v.observed) {v.motion = ScanMotion::Unknown;}
    if (v.motion == ScanMotion::Unknown) {result.unresolved = true;}
    if (v.age > config_.retention) {continue;}
    const double dt = v.motion == ScanMotion::Moving ? std::max(0.0, v.age) : 0.0;
    const auto p = transform * tf2::Vector3(v.x + v.vx * dt, v.y + v.vy * dt, 0.0);
    const auto velocity = transform.getBasis() * tf2::Vector3(v.vx, v.vy, 0.0);
    v.x = p.x(); v.y = p.y(); v.vx = velocity.x(); v.vy = velocity.y();
    const double c = std::abs(transform.getBasis()[0][0]);
    const double s = std::abs(transform.getBasis()[0][1]);
    const double hx = c * v.half_x + s * v.half_y;
    v.half_y = s * v.half_x + c * v.half_y; v.half_x = hx;
    result.tracks.push_back(v);
  }
  for (auto ray : rays_) {
    const auto origin = transform * tf2::Vector3(ray.origin.x, ray.origin.y, 0);
    const auto end = transform * tf2::Vector3(ray.end.x, ray.end.y, 0);
    ray.origin.x = origin.x(); ray.origin.y = origin.y();
    ray.end.x = end.x(); ray.end.y = end.y();
    result.rays.push_back(ray);
  }
  return result;
}

const char * scanMotionName(ScanMotion motion)
{
  switch (motion) {
    case ScanMotion::Moving: return "moving";
    case ScanMotion::Stationary: return "stationary";
    case ScanMotion::Unknown: return "unknown";
  }
  return "unknown";
}
}  // namespace my_dwa_controller
