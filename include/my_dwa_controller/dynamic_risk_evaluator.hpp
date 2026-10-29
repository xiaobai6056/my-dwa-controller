#pragma once

#include "my_dwa_controller/dynamic_trajectory_evaluator.hpp"

namespace my_dwa_controller
{

// A private prediction subset for one frozen candidate batch. Unknown-motion
// observations remain in the Tracker snapshot and the Scan/Costmap pipeline.
struct PreparedDynamicRisk
{
  bool input_available = false;
  const char * reason = "no snapshot";
  ScanSnapshot predicted;
  rclcpp::Time now{0, 0, RCL_ROS_TIME};
  std::size_t predicted_clusters = 0, unknown_clusters = 0;
  std::size_t input_points = 0, retained_points = 0;
};

struct DynamicRiskResult
{
  bool input_available = false;  // Availability of evidence, never a safety certificate.
  double cost = 0.0;
  std::optional<double> min_clearance;  // To retained prediction points only.
};

// Nonnegative preference added to the original DWA score. A zero cost means no
// predicted overlap in available evidence; it does not establish future safety.
class DynamicRiskEvaluator
{
public:
  DynamicRiskEvaluator(
    double robot_radius, double safety_margin, double sim_time,
    double simulation_dt, double max_scan_age);

  // spatial_screen=false is the full-point oracle used in differential tests.
  PreparedDynamicRisk prepare(
    const std::vector<const Trajectory *> & trajectories, const ScanSnapshot * scan,
    const std::string & frame, const rclcpp::Time & now, bool spatial_screen = true) const;
  DynamicRiskResult evaluate(
    const Trajectory & trajectory, const PreparedDynamicRisk & prepared) const;

private:
  DynamicTrajectoryEvaluator geometry_;
  double clearance_radius_, max_scan_age_;
};

}  // namespace my_dwa_controller
