#include "my_dwa_controller/dynamic_trajectory_evaluator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace my_dwa_controller
{
namespace
{

using Point = std::array<long double, 2>;

// Closest distance over the whole relative-motion interval, not just its ends.
// Use squared distance in the point loop and take one root for the final result.
long double squaredPointSegmentDistance(
  const std::array<double, 2> & point, const Point & start, const Point & end)
{
  const long double dx = end[0] - start[0], dy = end[1] - start[1];
  const long double length_squared = dx * dx + dy * dy;
  const long double px = static_cast<long double>(point[0]) - start[0];
  const long double py = static_cast<long double>(point[1]) - start[1];
  const long double t = length_squared > 0.0L ?
    std::clamp((px * dx + py * dy) / length_squared, 0.0L, 1.0L) : 0.0L;
  const long double gap_x = px - t * dx, gap_y = py - t * dy;
  return gap_x * gap_x + gap_y * gap_y;
}

}  // namespace

DynamicTrajectoryEvaluator::DynamicTrajectoryEvaluator(
  double robot_radius, double safety_margin, double sim_time,
  double simulation_dt, double max_scan_age)
: clearance_radius_(robot_radius + safety_margin), sim_time_(sim_time),
  simulation_dt_(simulation_dt), max_scan_age_(max_scan_age)
{
  if (!std::isfinite(robot_radius) || robot_radius < 0.0 ||
    !std::isfinite(safety_margin) || safety_margin < 0.0 ||
    !std::isfinite(clearance_radius_) || !std::isfinite(sim_time_) || sim_time_ <= 0.0 ||
    !std::isfinite(simulation_dt_) || simulation_dt_ <= 0.0 ||
    !std::isfinite(max_scan_age_) || max_scan_age_ <= 0.0)
  {
    throw std::invalid_argument("invalid geometry, predictor time grid or scan age");
  }
}

DynamicTrajectoryResult DynamicTrajectoryEvaluator::evaluate(
  const Trajectory & trajectory, const ScanSnapshot * scan,
  const std::string & trajectory_frame, const rclcpp::Time & now) const
{
  if (!scan || trajectory_frame.empty() || trajectory_frame.front() == '/' ||
    scan->frame_id != trajectory_frame || now.get_clock_type() != RCL_ROS_TIME ||
    scan->stamp.sec < 0 || scan->stamp.nanosec >= 1000000000u ||
    (scan->stamp.sec == 0 && scan->stamp.nanosec == 0) || trajectory.poses.size() < 2 ||
    !std::isfinite(trajectory.command.v) || !std::isfinite(trajectory.command.omega))
  {
    return {};
  }
  const double age = static_cast<double>((static_cast<long double>(now.nanoseconds()) -
    rclcpp::Time(scan->stamp).nanoseconds()) / 1000000000.0L);
  if (age < 0.0 || age > max_scan_age_) {
    return {};
  }
  for (const auto & pose : trajectory.poses) {
    if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta)) {
      return {};
    }
  }
  DynamicTrajectoryResult result;
  result.valid = true;
  for (const auto & observation : scan->obstacles) {
    const auto & box = observation.aabb;
    if (!std::isfinite(box.min_x) || !std::isfinite(box.max_x) ||
      !std::isfinite(box.min_y) || !std::isfinite(box.max_y) ||
      box.min_x > box.max_x || box.min_y > box.max_y ||
      (observation.velocity_valid &&
      (!std::isfinite(observation.vx) || !std::isfinite(observation.vy))))
    {
      return {};
    }
    if (observation.points.empty()) {
      return {};  // A box without measured points cannot establish dynamic clearance.
    }
    for (const auto & point : observation.points) {
      if (!std::isfinite(point[0]) || !std::isfinite(point[1])) {
        return {};
      }
    }
    if (!observation.velocity_valid) {
      result.valid = false;
    }
  }

  const long double horizon = sim_time_;
  const long double time_tolerance = 8.0L * std::numeric_limits<double>::epsilon() * horizon;
  long double elapsed = 0.0L;
  std::optional<long double> minimum_squared_distance;
  for (std::size_t i = 1; i < trajectory.poses.size(); ++i) {
    // Reproduce the existing predictor's index-based mesh, not i*dt for its
    // shortened/snapped final interval. The full input must match that mesh.
    long double next = std::min(i * static_cast<long double>(simulation_dt_), horizon);
    if (horizon - next <= time_tolerance) {
      next = horizon;
    }
    if (next <= elapsed || (i == trajectory.poses.size() - 1) != (next == horizon)) {
      return {};
    }
    for (const auto & observation : scan->obstacles) {
      Point start{trajectory.poses[i - 1].x, trajectory.poses[i - 1].y};
      Point end{trajectory.poses[i].x, trajectory.poses[i].y};
      if (observation.velocity_valid) {
        // Move both endpoints into the measured points' translating frame at the
        // corresponding time; no whole-horizon union or robot re-prediction.
        start[0] -= observation.vx * (age + elapsed);
        start[1] -= observation.vy * (age + elapsed);
        end[0] -= observation.vx * (age + next);
        end[1] -= observation.vy * (age + next);
      } else {
        if (i != 1) {
          continue;
        }
        end = start;  // Unknown motion permits observed initial geometry only.
      }
      for (const auto & point : observation.points) {
        const long double squared_distance = squaredPointSegmentDistance(point, start, end);
        if (!minimum_squared_distance || squared_distance < *minimum_squared_distance) {
          minimum_squared_distance = squared_distance;
        }
      }
    }
    elapsed = next;
  }
  if (minimum_squared_distance) {
    // Each measured point gets the same circular radius+margin, subtracted once.
    const long double clearance = std::sqrt(*minimum_squared_distance) - clearance_radius_;
    if (!std::isfinite(clearance) || std::abs(clearance) > std::numeric_limits<double>::max()) {
      return {};
    }
    result.min_clearance = static_cast<double>(clearance);
    result.collision = *result.min_clearance <= 0.0;
  }
  return result;
}

bool DynamicTrajectoryEvaluator::collidesWithObservedPoints(
  const Trajectory & trajectory, const std::vector<std::array<double, 2>> & points) const
{
  if (points.empty()) {return false;}
  long double min_x = trajectory.poses.front().x, max_x = min_x;
  long double min_y = trajectory.poses.front().y, max_y = min_y;
  for (const auto & pose : trajectory.poses) {
    min_x = std::min(min_x, static_cast<long double>(pose.x));
    max_x = std::max(max_x, static_cast<long double>(pose.x));
    min_y = std::min(min_y, static_cast<long double>(pose.y));
    max_y = std::max(max_y, static_cast<long double>(pose.y));
  }
  const long double radius_squared = static_cast<long double>(clearance_radius_) *
    clearance_radius_;
  for (const auto & point : points) {
    // The whole piecewise-linear sweep lies inside its endpoint AABB. Its
    // distance is a lower bound, allowing far points to skip the segment loop.
    const long double dx = std::max({min_x - point[0], 0.0L, point[0] - max_x});
    const long double dy = std::max({min_y - point[1], 0.0L, point[1] - max_y});
    if (dx * dx + dy * dy > radius_squared) {continue;}
    for (std::size_t i = 1; i < trajectory.poses.size(); ++i) {
      const Point start{trajectory.poses[i - 1].x, trajectory.poses[i - 1].y};
      const Point end{trajectory.poses[i].x, trajectory.poses[i].y};
      if (squaredPointSegmentDistance(point, start, end) <= radius_squared) {return true;}
    }
  }
  return false;
}

std::optional<ScanSnapshot> DynamicTrajectoryEvaluator::prepareScan(
  const std::vector<const Trajectory *> & trajectories, const ScanSnapshot * scan,
  const std::string & trajectory_frame, const rclcpp::Time & now) const
{
  // Validate the original input before removing anything. A missing optional is a
  // bypass to evaluate(original), never a confirmed-empty or valid assessment.
  if (!scan || trajectories.empty() || trajectory_frame.empty() ||
    trajectory_frame.front() == '/' || scan->frame_id != trajectory_frame ||
    now.get_clock_type() != RCL_ROS_TIME || scan->stamp.sec < 0 ||
    scan->stamp.nanosec >= 1000000000u ||
    (scan->stamp.sec == 0 && scan->stamp.nanosec == 0))
  {
    return std::nullopt;
  }
  const double age = static_cast<double>((static_cast<long double>(now.nanoseconds()) -
    rclcpp::Time(scan->stamp).nanoseconds()) / 1000000000.0L);
  if (age < 0.0 || age > max_scan_age_) {
    return std::nullopt;
  }
  const long double horizon = sim_time_;
  const long double tolerance = 8.0L * std::numeric_limits<double>::epsilon() * horizon;
  // Extreme finite inputs can make the original distance calculation invalid.
  // Do not turn such a case into a valid empty subset. This is a numerical guard,
  // not an obstacle-speed, range or physical filtering threshold.
  const long double numeric_limit = std::sqrt(std::numeric_limits<double>::max()) / 16.0L;
  for (const auto * trajectory : trajectories) {
    if (!trajectory || trajectory->poses.size() < 2 ||
      !std::isfinite(trajectory->command.v) || !std::isfinite(trajectory->command.omega))
    {
      return std::nullopt;
    }
    for (const auto & pose : trajectory->poses) {
      if (!std::isfinite(pose.x) || !std::isfinite(pose.y) || !std::isfinite(pose.theta) ||
        std::abs(pose.x) > numeric_limit || std::abs(pose.y) > numeric_limit)
      {
        return std::nullopt;
      }
    }
    long double elapsed = 0.0L;
    for (std::size_t i = 1; i < trajectory->poses.size(); ++i) {
      long double next = std::min(i * static_cast<long double>(simulation_dt_), horizon);
      if (horizon - next <= tolerance) {
        next = horizon;
      }
      if (next <= elapsed || (i == trajectory->poses.size() - 1) != (next == horizon)) {
        return std::nullopt;
      }
      elapsed = next;
    }
  }
  for (const auto & observation : scan->obstacles) {
    const auto & box = observation.aabb;
    if (!observation.velocity_valid || !std::isfinite(observation.vx) ||
      !std::isfinite(observation.vy) || !std::isfinite(box.min_x) ||
      !std::isfinite(box.max_x) || !std::isfinite(box.min_y) || !std::isfinite(box.max_y) ||
      box.min_x > box.max_x || box.min_y > box.max_y || observation.points.empty() ||
      std::abs(observation.vx * (age + horizon)) > numeric_limit ||
      std::abs(observation.vy * (age + horizon)) > numeric_limit)
    {
      return std::nullopt;
    }
    for (const auto & point : observation.points) {
      if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
        std::abs(point[0]) > numeric_limit || std::abs(point[1]) > numeric_limit)
      {
        return std::nullopt;
      }
    }
  }

  ScanSnapshot filtered;
  filtered.stamp = scan->stamp;
  filtered.frame_id = scan->frame_id;
  for (const auto & observation : scan->obstacles) {
    long double min_x = std::numeric_limits<long double>::infinity();
    long double min_y = min_x, max_x = -min_x, max_y = -min_x;
    for (const auto * trajectory : trajectories) {
      for (std::size_t i = 0; i < trajectory->poses.size(); ++i) {
        long double time = std::min(i * static_cast<long double>(simulation_dt_), horizon);
        if (horizon - time <= tolerance) {
          time = horizon;
        }
        // Same relative coordinates and snapped time mesh as evaluate(). Every
        // continuous relative interval lies inside its endpoint AABB.
        const long double x = static_cast<long double>(trajectory->poses[i].x) -
          observation.vx * (age + time);
        const long double y = static_cast<long double>(trajectory->poses[i].y) -
          observation.vy * (age + time);
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
      }
    }
    const long double scale = std::max(
      {1.0L, std::abs(min_x), std::abs(max_x), std::abs(min_y), std::abs(max_y)});
    const long double padding = clearance_radius_ + 1e-10L +
      64.0L * std::numeric_limits<long double>::epsilon() * scale;
    min_x -= padding;
    max_x += padding;
    min_y -= padding;
    max_y += padding;
    ObstacleObservation retained;
    retained.aabb = observation.aabb;
    retained.vx = observation.vx;
    retained.vy = observation.vy;
    retained.velocity_valid = observation.velocity_valid;
    for (const auto & point : observation.points) {
      if (point[0] >= min_x && point[0] <= max_x && point[1] >= min_y && point[1] <= max_y) {
        retained.points.push_back(point);
      }
    }
    if (!retained.points.empty()) {
      filtered.obstacles.push_back(std::move(retained));
    }
  }
  return filtered;
}

}  // namespace my_dwa_controller
