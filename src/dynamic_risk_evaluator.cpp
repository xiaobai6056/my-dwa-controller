#include "my_dwa_controller/dynamic_risk_evaluator.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace my_dwa_controller
{

DynamicRiskEvaluator::DynamicRiskEvaluator(
  double robot_radius, double safety_margin, double sim_time,
  double simulation_dt, double max_scan_age)
: geometry_(robot_radius, safety_margin, sim_time, simulation_dt, max_scan_age),
  clearance_radius_(robot_radius + safety_margin), max_scan_age_(max_scan_age)
{
  if (clearance_radius_ <= 0.0) {
    throw std::invalid_argument("dynamic risk requires a positive clearance radius");
  }
}

PreparedDynamicRisk DynamicRiskEvaluator::prepare(
  const std::vector<const Trajectory *> & trajectories, const ScanSnapshot * scan,
  const std::string & frame, const rclcpp::Time & now, bool spatial_screen) const
{
  PreparedDynamicRisk result;
  if (!scan) {return result;}
  result.reason = "invalid frame/time";
  if (frame.empty() || frame.front() == '/' || scan->frame_id != frame ||
    now.get_clock_type() != RCL_ROS_TIME || scan->stamp.sec < 0 ||
    scan->stamp.nanosec >= 1000000000u ||
    (scan->stamp.sec == 0 && scan->stamp.nanosec == 0))
  {
    return result;
  }
  const double age = (now - rclcpp::Time(scan->stamp)).seconds();
  if (age < 0.0 || age > max_scan_age_) {
    result.reason = age < 0.0 ? "future scan stamp" : "expired scan";
    return result;
  }

  result.predicted.stamp = scan->stamp;
  result.predicted.frame_id = frame;
  result.now = now;
  result.reason = "invalid observation";
  for (const auto & observation : scan->obstacles) {
    const auto & box = observation.aabb;
    // Validate before selecting evidence: invalid unknown geometry must not be
    // transformed into an available, empty prediction set.
    if (!std::isfinite(box.min_x) || !std::isfinite(box.max_x) ||
      !std::isfinite(box.min_y) || !std::isfinite(box.max_y) ||
      box.min_x > box.max_x || box.min_y > box.max_y || observation.points.empty() ||
      (observation.velocity_valid &&
      (!std::isfinite(observation.vx) || !std::isfinite(observation.vy))))
    {
      return result;
    }
    for (const auto & point : observation.points) {
      if (!std::isfinite(point[0]) || !std::isfinite(point[1])) {return result;}
    }
    result.input_points += observation.points.size();
    if (observation.velocity_valid) {
      result.predicted.obstacles.push_back(observation);
      ++result.predicted_clusters;
    } else {
      ++result.unknown_clusters;
    }
  }
  // The existing kernel validates the predictor mesh and constructs a full
  // horizon query for each velocity. No static classification or re-prediction.
  auto screened = geometry_.prepareScan(trajectories, &result.predicted, frame, now);
  if (!screened) {
    result.reason = "invalid prediction geometry";
    return result;
  }
  if (spatial_screen) {result.predicted = std::move(*screened);}
  for (const auto & observation : result.predicted.obstacles) {
    result.retained_points += observation.points.size();
  }
  result.input_available = true;
  result.reason = "available evidence";
  return result;
}

DynamicRiskResult DynamicRiskEvaluator::evaluate(
  const Trajectory & trajectory, const PreparedDynamicRisk & prepared) const
{
  if (!prepared.input_available) {return {};}
  const auto geometry = geometry_.evaluate(
    trajectory, &prepared.predicted, prepared.predicted.frame_id, prepared.now);
  DynamicRiskResult result;
  result.input_available = geometry.valid;
  if (!geometry.valid) {return result;}
  result.min_clearance = geometry.min_clearance;
  if (geometry.min_clearance) {
    result.cost = std::clamp(-*geometry.min_clearance / clearance_radius_, 0.0, 1.0);
  }
  return result;
}

}  // namespace my_dwa_controller
