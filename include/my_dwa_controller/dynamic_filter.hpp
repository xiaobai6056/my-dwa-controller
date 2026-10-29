#pragma once

#include "my_dwa_controller/dynamic_risk_evaluator.hpp"

namespace my_dwa_controller
{
struct ClusterConflict
{
  std::size_t index;
  double clearance;
};

struct FilterResult
{
  bool available = false;
  std::vector<ClusterConflict> conflicts;
  std::optional<double> min_clearance;
};

struct PreparedFilter
{
  PreparedDynamicRisk input;
  // Validated occupied snapshot, kept separate from known-motion prediction.
  std::vector<std::array<double, 2>> unknown_points;
  std::vector<std::pair<std::size_t, ScanSnapshot>> clusters;
};

// Attribute the unchanged continuous geometry result to its original observation.
class DynamicFilter
{
public:
  DynamicFilter(double radius, double horizon, double step)
  : validation_(radius, 0.05, horizon, step, 0.35),
    geometry_(radius, 0.05, horizon, step, 0.35) {}

  PreparedFilter prepare(
    const std::vector<const Trajectory *> &, const ScanSnapshot *, const std::string &,
    const rclcpp::Time &) const;
  FilterResult evaluate(const Trajectory &, const PreparedFilter &) const;
  // Only meaningful with input_available. Constrains the observed occupied
  // snapshot, not unknown future motion; does not change evidence to complete.
  bool observedGeometryCollision(const Trajectory &, const PreparedFilter &) const;

private:
  DynamicRiskEvaluator validation_;
  DynamicTrajectoryEvaluator geometry_;
};
}  // namespace my_dwa_controller
