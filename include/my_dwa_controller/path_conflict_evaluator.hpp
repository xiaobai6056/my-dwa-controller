#pragma once

#include <optional>
#include <string>
#include <vector>

#include "rclcpp/time.hpp"
#include "my_dwa_controller/path_geometry.hpp"
#include "my_dwa_controller/scan_obstacle_tracker.hpp"

namespace my_dwa_controller
{

// Caller-supplied effective physical model, not acceleration or smoothing limits.
// No measured values are currently configured; tests use synthetic inputs only.
struct BrakingModel
{
  double deceleration;
  double reaction_time;
};

struct PathConflictResult
{
  bool valid = false;
  // Presence means a detected potential conflict; distance starts at the robot's
  // path projection and already includes radius + safety margin in the geometry.
  std::optional<double> conflict_distance;
  // Absent unless evaluation is valid AND an explicit braking model was supplied.
  std::optional<double> speed_limit;
};

class PathConflictEvaluator
{
public:
  // Explicit inputs reuse the caller's radius, margin, prediction horizon/step
  // and freshness policy. No ROS parameters, clocks, TF or history are owned here.
  PathConflictEvaluator(
    double robot_radius, double safety_margin, double prediction_horizon,
    double prediction_step, double max_scan_age);

  // All geometry must share path_frame; now must use the scan's ROS clock domain.
  // Reuses existing projection/extraction to discard the path behind the robot.
  // A fresh empty scan is valid/no detected conflict, never region-clear evidence.
  // Unknown velocities make prediction invalid, but observed overlap/distance can
  // still be returned. Never interpret an invalid result's missing hit as safety.
  // Optional explicit braking inputs are not inferred from static DWA limits.
  PathConflictResult evaluate(
    const ScanSnapshot * scan, const std::vector<Pose2D> & local_path,
    const Pose2D & robot, const std::string & path_frame, const rclcpp::Time & now,
    double normal_speed, std::optional<BrakingModel> braking = std::nullopt) const;

private:
  double corridor_radius_;
  double prediction_horizon_;
  double prediction_step_;
  double max_scan_age_;
};

}  // namespace my_dwa_controller
