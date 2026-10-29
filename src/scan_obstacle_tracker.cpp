#include "my_dwa_controller/scan_obstacle_tracker.hpp"
#include "my_dwa_controller/surface_motion.hpp"
#include "measurement.hpp"
#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace my_dwa_controller
{
namespace
{

// At 5 Hz, allow one missed scan (0.4 s), but do not bridge a longer gap.
constexpr double kMaxAssociationInterval = 0.5;

std::optional<std::vector<ObstacleObservation>> clusterScan(
  const sensor_msgs::msg::LaserScan & scan, const tf2::Transform & transform,
  double cluster_distance, std::size_t min_points)
{
  std::vector<ObstacleObservation> obstacles;
  ObstacleAabb bounds;
  std::vector<std::array<double, 2>> points;
  std::size_t count = 0;
  double previous_x = 0.0, previous_y = 0.0;
  const auto finish_cluster = [&]() {
      if (count >= min_points) {
        ObstacleObservation observation;
        observation.aabb = bounds;
        observation.points = std::move(points);
        obstacles.push_back(std::move(observation));
      }
      count = 0;
      points.clear();
    };

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      finish_cluster();  // Invalid beams must break connectivity.
      continue;
    }
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double x = range * std::cos(angle), y = range * std::sin(angle);
    if (count > 0 && std::hypot(x - previous_x, y - previous_y) > cluster_distance) {
      finish_cluster();
    }
    const auto point = transform * tf2::Vector3(x, y, 0.0);
    if (!std::isfinite(point.x()) || !std::isfinite(point.y())) {
      return std::nullopt;
    }
    if (count == 0) {
      bounds = {point.x(), point.x(), point.y(), point.y()};
    } else {
      bounds.min_x = std::min(bounds.min_x, point.x());
      bounds.max_x = std::max(bounds.max_x, point.x());
      bounds.min_y = std::min(bounds.min_y, point.y());
      bounds.max_y = std::max(bounds.max_y, point.y());
    }
    points.push_back({point.x(), point.y()});
    previous_x = x;
    previous_y = y;
    ++count;
  }
  finish_cluster();
  return obstacles;
}

}  // namespace

ScanObstacleTracker::ScanObstacleTracker(
  std::shared_ptr<tf2_ros::Buffer> tf, double cluster_distance,
  std::size_t min_cluster_points, double association_distance)
: tf_(std::move(tf)), cluster_distance_(cluster_distance),
  min_cluster_points_(min_cluster_points), association_distance_(association_distance)
{
  if (!tf_ || !std::isfinite(cluster_distance_) || cluster_distance_ <= 0.0 ||
    min_cluster_points_ == 0 || !std::isfinite(association_distance_) ||
    association_distance_ <= 0.0)
  {
    throw std::invalid_argument("scan tracker requires TF, positive distances and minimum points");
  }
}

std::shared_ptr<ScanObstacleTracker> ScanObstacleTracker::newSession() const
{
  return std::make_shared<ScanObstacleTracker>(
    tf_, cluster_distance_, min_cluster_points_, association_distance_);
}

void ScanObstacleTracker::enqueue(sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto logger = rclcpp::get_logger("my_dwa_controller.scan_tracker");
  if (!scan || scan->header.frame_id.empty() || scan->header.frame_id.front() == '/' ||
    scan->header.stamp.sec < 0 || scan->header.stamp.nanosec >= 1000000000u ||
    (scan->header.stamp.sec == 0 && scan->header.stamp.nanosec == 0))
  {
    RCLCPP_WARN_THROTTLE(logger, diagnostic_clock_, 2000, "Scan rejected: invalid frame or stamp");
    snapshot_.reset();
    pending_.clear();
    return;
  }
  const auto stamp = rclcpp::Time(scan->header.stamp);
  const auto * latest = pending_.empty() ? (snapshot_ ? &snapshot_->stamp : nullptr) :
    &pending_.back()->header.stamp;
  if (latest && stamp <= rclcpp::Time(*latest)) {
    RCLCPP_WARN_THROTTLE(logger, diagnostic_clock_, 2000, "Scan dropped: non-increasing stamp");
    return;  // Never let a late arrival destroy newer association evidence.
  }
  // Five slots bound Scan bursts even while simulation time is paused. ROS data
  // age alone decides expiry, so slow/paused simulation is not aged by wall time.
  if (pending_.size() == 5u) {
    pending_.pop_front();
    RCLCPP_WARN_THROTTLE(logger, diagnostic_clock_, 2000, "Scan dropped: TF queue full (5)");
  }
  pending_.push_back(std::move(scan));
}

void ScanObstacleTracker::processPending(const std::string & target_frame, const rclcpp::Time & now)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto logger = rclcpp::get_logger("my_dwa_controller.scan_tracker");
  if (target_frame.empty() || target_frame.front() == '/' || now.get_clock_type() != RCL_ROS_TIME) {
    RCLCPP_WARN_THROTTLE(
      logger, diagnostic_clock_, 2000,
      "Scan rejected: invalid target frame/clock");
    pending_.clear();
    snapshot_.reset();
    return;
  }
  if (last_processing_time_ && now < *last_processing_time_) {
    // A clock reset starts a new observation epoch; a late Scan alone never does.
    pending_.clear();
    snapshot_.reset();
    RCLCPP_WARN_THROTTLE(
      logger, diagnostic_clock_, 2000,
      "ROS time moved backwards: Scan queue and association reset");
  }
  last_processing_time_ = now;
  while (!pending_.empty()) {
    const auto & scan = *pending_.front();
    const double age = (now - rclcpp::Time(scan.header.stamp)).seconds();
    if (age > 0.35) {
      RCLCPP_WARN_THROTTLE(
        logger, diagnostic_clock_, 2000,
        "Scan dropped: TF queue expired (ROS age=%.6f s)", age);
      pending_.pop_front();
      continue;
    }
    if (age < 0.0) {
      // TF can arrive before /clock advances. Keep the original stamp and wait
      // for the same ROS clock used by consumers, rather than allowing future data.
      RCLCPP_DEBUG_THROTTLE(
        logger, diagnostic_clock_, 2000,
        "Scan pending: ROS time has not reached stamp (lead=%.9f s)",
        -age);
      break;
    }
    // BufferCore's zero-wait query does not depend on a dedicated listener thread.
    // A not-yet-ready transform leaves the head queued and association untouched.
    if (scan.header.frame_id != target_frame &&
      !tf_->tf2::BufferCore::canTransform(
        target_frame, scan.header.frame_id, tf2_ros::fromMsg(scan.header.stamp)))
    {
      RCLCPP_DEBUG_THROTTLE(
        logger, diagnostic_clock_, 2000, "Scan pending: exact-stamp TF not ready (%s -> %s)",
        scan.header.frame_id.c_str(), target_frame.c_str());
      break;
    }
    updateLocked(scan, target_frame);
    RCLCPP_DEBUG(
      logger, "Scan release: stamp_ns=%ld now_ns=%ld age_ns=%ld processed=%s",
      rclcpp::Time(scan.header.stamp).nanoseconds(), now.nanoseconds(),
      now.nanoseconds() - rclcpp::Time(scan.header.stamp).nanoseconds(),
      snapshot_ ? "true" : "false");
    pending_.pop_front();
  }
}

void ScanObstacleTracker::update(
  const sensor_msgs::msg::LaserScan & scan, const std::string & target_frame)
{
  std::lock_guard<std::mutex> lock(mutex_);
  updateLocked(scan, target_frame);
}

void ScanObstacleTracker::updateLocked(
  const sensor_msgs::msg::LaserScan & scan, const std::string & target_frame)
{
  // The same frame serves as latest output and previous association input.
  // Moving it out makes every failure below invalidate both, without stale output.
  auto previous = std::move(snapshot_);
  if (scan.header.frame_id.empty() || target_frame.empty() ||
    scan.header.frame_id.front() == '/' || target_frame.front() == '/' ||
    scan.header.stamp.sec < 0 || scan.header.stamp.nanosec >= 1000000000u ||
    (scan.header.stamp.sec == 0 && scan.header.stamp.nanosec == 0) ||
    !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_increment) ||
    scan.angle_increment == 0.0f || !std::isfinite(scan.range_min) ||
    !std::isfinite(scan.range_max) || scan.range_min < 0.0f || scan.range_max <= scan.range_min)
  {
    return;
  }
  if (previous && previous->frame_id != target_frame) {
    previous.reset();
  }
  double dt = 0.0;
  if (previous) {
    dt = (rclcpp::Time(scan.header.stamp) - rclcpp::Time(previous->stamp)).seconds();
    if (dt <= 0.0) {
      return;
    }
    if (dt > kMaxAssociationInterval) {
      previous.reset();
    }
  }

  tf2::Transform transform = tf2::Transform::getIdentity();
  try {
    if (scan.header.frame_id != target_frame) {
      const auto value = tf_->lookupTransform(
        target_frame, scan.header.frame_id, tf2_ros::fromMsg(scan.header.stamp));
      tf2::fromMsg(value.transform, transform);
    }
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("my_dwa_controller.scan_tracker"), diagnostic_clock_, 2000,
      "Scan failed: exact-stamp TF query: %s", error.what());
    return;  // No latest-TF substitution and no confirmed-empty result.
  }
  auto obstacles = clusterScan(scan, transform, cluster_distance_, min_cluster_points_);
  if (!obstacles) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("my_dwa_controller.scan_tracker"), diagnostic_clock_, 2000,
      "Scan failed: invalid transformed geometry");
    return;
  }
  if (previous) {
    std::vector<bool> matched(previous->obstacles.size(), false);
    for (auto & current : *obstacles) {
      const double x = 0.5 * current.aabb.min_x + 0.5 * current.aabb.max_x;
      const double y = 0.5 * current.aabb.min_y + 0.5 * current.aabb.max_y;
      double closest_distance = std::numeric_limits<double>::infinity();
      std::size_t closest = previous->obstacles.size();
      for (std::size_t i = 0; i < previous->obstacles.size(); ++i) {
        if (matched[i]) {
          continue;
        }
        const auto & box = previous->obstacles[i].aabb;
        const double distance = std::hypot(
          x - (0.5 * box.min_x + 0.5 * box.max_x),
          y - (0.5 * box.min_y + 0.5 * box.max_y));
        if (distance <= association_distance_ && distance < closest_distance) {
          closest = i;
          closest_distance = distance;
        }
      }
      if (closest == previous->obstacles.size()) {
        continue;
      }
      matched[closest] = true;
      current.previous_cluster_index = static_cast<int>(closest);
      const auto partial = partial_surface::measure(
        previous->obstacles[closest].points,
        current.points, dt);
      current.partial_motion_rank = partial.rank;
      Eigen::Matrix2d h;
      h << partial.information[0], partial.information[1], partial.information[1],
        partial.information[2];
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen(h);
      Eigen::Vector2d observed = Eigen::Vector2d::Zero();
      for (int k = 0; k < 2; ++k) {
        if (eigen.eigenvalues()[k] <= 1e-9) {
          continue;
        }
        auto n = eigen.eigenvectors().col(k);
        observed += n *
          (n.dot(
            Eigen::Vector2d(
              partial.right_hand_side[0],
              partial.right_hand_side[1])) / eigen.eigenvalues()[k]);
      }
      current.partial_motion_vx = observed[0];current.partial_motion_vy = observed[1];
      surface_motion::Config config;
      config.association_distance = association_distance_;
      const auto motion = surface_motion::estimate(
        previous->obstacles[closest].points, current.points, dt, config);
      if (motion.value) {
        current.vx = (*motion.value)[0];
        current.vy = (*motion.value)[1];
        current.velocity_valid = true;
      }
    }
  }
  auto snapshot = std::make_shared<ScanSnapshot>();
  snapshot->stamp = scan.header.stamp;
  snapshot->previous_stamp_ns = previous ? rclcpp::Time(previous->stamp).nanoseconds() : 0;
  snapshot->laser_pose = {transform.getOrigin().x(), transform.getOrigin().y(), std::atan2(
      transform.getBasis()[1][0], transform.getBasis()[0][0])};
  snapshot->ranges = scan.ranges;snapshot->angle_min = scan.angle_min;
  snapshot->angle_increment = scan.angle_increment;
  snapshot->range_min = scan.range_min;snapshot->range_max = scan.range_max;
  snapshot->frame_id = target_frame;
  snapshot->obstacles = std::move(*obstacles);
  snapshot_ = std::move(snapshot);
  const auto known = std::count_if(
    snapshot_->obstacles.begin(), snapshot_->obstacles.end(),
    [](const auto & obstacle) {return obstacle.velocity_valid;});
  RCLCPP_DEBUG_THROTTLE(
    rclcpp::get_logger("my_dwa_controller.scan_tracker"), diagnostic_clock_, 2000,
    "Scan processed: stamp=%d.%09u frame=%s clusters=%zu known_velocity=%zu",
    scan.header.stamp.sec, scan.header.stamp.nanosec, target_frame.c_str(),
    snapshot_->obstacles.size(), static_cast<std::size_t>(known));
}

}  // namespace my_dwa_controller
