#include "my_dwa_controller/path_conflict_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace my_dwa_controller
{
namespace
{

bool validBox(const ObstacleAabb & box)
{
  return std::isfinite(box.min_x) && std::isfinite(box.max_x) &&
         std::isfinite(box.min_y) && std::isfinite(box.max_y) &&
         box.min_x <= box.max_x && box.min_y <= box.max_y;
}

// Clip each real path segment against the expanded AABB. Fractions locate the
// boundary exactly; accumulated arc length is measured from the path projection.
std::optional<double> firstPathEntry(
  const std::vector<Pose2D> & path, const ObstacleAabb & box)
{
  if (path.size() == 1) {
    const auto & p = path.front();
    if (p.x >= box.min_x && p.x <= box.max_x && p.y >= box.min_y && p.y <= box.max_y) {
      return 0.0;
    }
    return std::nullopt;
  }
  long double traveled = 0.0L;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double start[2] = {path[i - 1].x, path[i - 1].y};
    const long double delta[2] = {
      static_cast<long double>(path[i].x) - start[0],
      static_cast<long double>(path[i].y) - start[1]};
    const double lower[2] = {box.min_x, box.min_y};
    const double upper[2] = {box.max_x, box.max_y};
    long double enter = 0.0L, leave = 1.0L;
    bool intersects = true;
    for (std::size_t axis = 0; axis < 2; ++axis) {
      if (delta[axis] == 0.0L) {
        if (start[axis] < lower[axis] || start[axis] > upper[axis]) {
          intersects = false;
          break;
        }
      } else {
        long double first = (static_cast<long double>(lower[axis]) - start[axis]) / delta[axis];
        long double last = (static_cast<long double>(upper[axis]) - start[axis]) / delta[axis];
        if (first > last) {
          std::swap(first, last);
        }
        enter = std::max(enter, first);
        leave = std::min(leave, last);
        if (enter > leave) {
          intersects = false;
          break;
        }
      }
    }
    const long double length = std::hypot(delta[0], delta[1]);
    const long double distance = traveled + (intersects ? enter * length : length);
    if (distance > std::numeric_limits<double>::max()) {
      throw std::overflow_error("forward path distance exceeds double range");
    }
    if (intersects) {
      return static_cast<double>(distance);
    }
    traveled = distance;
  }
  return std::nullopt;
}

}  // namespace

PathConflictEvaluator::PathConflictEvaluator(
  double robot_radius, double safety_margin, double prediction_horizon,
  double prediction_step, double max_scan_age)
: corridor_radius_(robot_radius + safety_margin), prediction_horizon_(prediction_horizon),
  prediction_step_(prediction_step), max_scan_age_(max_scan_age)
{
  if (!std::isfinite(robot_radius) || robot_radius < 0.0 ||
    !std::isfinite(safety_margin) || safety_margin < 0.0 ||
    !std::isfinite(corridor_radius_) || !std::isfinite(prediction_horizon_) ||
    prediction_horizon_ <= 0.0 || !std::isfinite(prediction_step_) ||
    prediction_step_ <= 0.0 || !std::isfinite(max_scan_age_) || max_scan_age_ <= 0.0)
  {
    throw std::invalid_argument("invalid corridor, prediction interval or scan age");
  }
}

PathConflictResult PathConflictEvaluator::evaluate(
  const ScanSnapshot * scan, const std::vector<Pose2D> & local_path,
  const Pose2D & robot, const std::string & path_frame, const rclcpp::Time & now,
  double normal_speed, std::optional<BrakingModel> braking) const
{
  if (!scan || path_frame.empty() || path_frame.front() == '/' ||
    scan->frame_id != path_frame || now.get_clock_type() != RCL_ROS_TIME ||
    scan->stamp.sec < 0 || scan->stamp.nanosec >= 1000000000u ||
    (scan->stamp.sec == 0 && scan->stamp.nanosec == 0) ||
    !std::isfinite(normal_speed) || normal_speed < 0.0)
  {
    return {};
  }
  if (braking && (!std::isfinite(braking->deceleration) || braking->deceleration <= 0.0 ||
    !std::isfinite(braking->reaction_time) || braking->reaction_time < 0.0))
  {
    return {};
  }

  try {
    const double age = (now - rclcpp::Time(scan->stamp)).seconds();
    if (age < 0.0 || age > max_scan_age_) {
      return {};
    }
    const auto projection = projectOntoPath(robot, local_path);
    if (!projection) {
      return {};
    }
    // Keep the caller's already cropped local path, starting at this projection.
    const auto forward = extractForwardPath(
      local_path, *projection, std::numeric_limits<double>::max());
    PathConflictResult result;
    result.valid = true;
    for (const auto & observation : scan->obstacles) {
      const auto & box = observation.aabb;
      if (!validBox(box) || (observation.velocity_valid &&
        (!std::isfinite(observation.vx) || !std::isfinite(observation.vy))))
      {
        return {};
      }
      if (!observation.velocity_valid) {
        result.valid = false;  // Observed geometry cannot establish future safety.
      }
      double t = 0.0;
      do {
        const double next = observation.velocity_valid ?
          std::min(prediction_horizon_, t + prediction_step_) : 0.0;
        if (observation.velocity_valid && next <= t) {
          return {};
        }
        const double dx0 = observation.velocity_valid ? observation.vx * (age + t) : 0.0;
        const double dx1 = observation.velocity_valid ? observation.vx * (age + next) : 0.0;
        const double dy0 = observation.velocity_valid ? observation.vy * (age + t) : 0.0;
        const double dy1 = observation.velocity_valid ? observation.vy * (age + next) : 0.0;
        // Endpoint hulls enclose continuous constant-velocity motion between
        // samples. Axis expansion conservatively contains the circular corridor;
        // corners can overestimate risk. Radius and margin are applied only here.
        const ObstacleAabb swept{
          box.min_x + std::min(dx0, dx1) - corridor_radius_,
          box.max_x + std::max(dx0, dx1) + corridor_radius_,
          box.min_y + std::min(dy0, dy1) - corridor_radius_,
          box.max_y + std::max(dy0, dy1) + corridor_radius_};
        if (!validBox(swept)) {
          return {};
        }
        const auto distance = firstPathEntry(forward, swept);
        if (distance && (!result.conflict_distance || *distance < *result.conflict_distance)) {
          result.conflict_distance = distance;
        }
        t = next;
      } while (observation.velocity_valid && t < prediction_horizon_);
    }
    if (!result.valid || !braking) {
      return result;
    }
    result.speed_limit = normal_speed;
    if (result.conflict_distance) {
      const long double distance = *result.conflict_distance;
      const long double a = braking->deceleration;
      const long double offset = a * braking->reaction_time;
      const long double twice_ad = 2.0L * a * distance;
      // Rationalized sqrt((a*tau)^2 + 2*a*L) - a*tau avoids cancellation.
      const long double cap = distance == 0.0L ? 0.0L :
        twice_ad / (std::sqrt(offset * offset + twice_ad) + offset);
      result.speed_limit =
        static_cast<double>(std::min(static_cast<long double>(normal_speed), cap));
    }
    return result;
  } catch (const std::invalid_argument &) {
    return {};
  } catch (const std::overflow_error &) {
    return {};
  }
}

}  // namespace my_dwa_controller
