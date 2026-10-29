#pragma once

#include <optional>
#include <vector>
#include <string>

#include "rclcpp/time.hpp"
#include "my_dwa_controller/scan_obstacle_tracker.hpp"
#include "my_dwa_controller/types.hpp"

namespace my_dwa_controller
{

struct DynamicTrajectoryResult
{
  bool valid = false;
  // Only valid=true permits a no-collision interpretation, within sim_time only.
  bool collision = false;
  // Minimum center-to-measured-point Euclidean distance minus radius+margin once.
  // <=0 means conflict, not penetration depth. Points do not imply a filled shape.
  // Empty scans have no
  // numeric distance. Distance always refers to the supplied point subset, not
  // implicitly to an unscreened snapshot. Invalid evaluations never establish
  // future clearance.
  std::optional<double> min_clearance;
};

// Stateless simultaneous robot/obstacle geometry, independent of DWA scoring.
class DynamicTrajectoryEvaluator
{
public:
  // sim_time and simulation_dt MUST match the TrajectoryPredictor that generated
  // the input; all lengths are meters, times seconds. No new sampling defaults.
  DynamicTrajectoryEvaluator(
    double robot_radius, double safety_margin, double sim_time,
    double simulation_dt, double max_scan_age);

  // poses.front() is t=0 at now; all poses must share trajectory_frame, which must
  // equal the scan's global frame. No TF or new robot prediction is performed.
  // Uses the predictor's time mesh, including a shortened final interval, and
  // piecewise linear position between its poses. Only this horizon is assessed.
  // now must use ROS time. Fresh empty != region clear; null/stale/future/invalid
  // input is invalid. Unknown velocity is never zero motion: only initial-pose
  // observed geometry may be returned as partial evidence, with valid=false.
  DynamicTrajectoryResult evaluate(
    const Trajectory & trajectory, const ScanSnapshot * scan,
    const std::string & trajectory_frame, const rclcpp::Time & now) const;

  // Conservative broad phase for exactly this candidate batch, frame and time.
  // Preserves every collision and colliding minimum clearance in evaluate()'s
  // existing motion model. Positive clearances are to retained points only and
  // can increase or be absent. Callers must not treat them as original global
  // nearest-obstacle distances. Do not reuse after changing candidates or now.
  // nullopt means bypass to the original snapshot, never a safe/empty assessment.
  // Original invalid/unknown-velocity inputs are never filtered into validity.
  std::optional<ScanSnapshot> prepareScan(
    const std::vector<const Trajectory *> & trajectories, const ScanSnapshot * scan,
    const std::string & trajectory_frame, const rclcpp::Time & now) const;

  // Pure spatial sweep against an occupied snapshot; no obstacle time or
  // velocity prediction. Inputs must already contain finite points and a valid
  // trajectory (as checked by DynamicFilter::prepare). Uses radius+margin once.
  // A false result says nothing about unknown future obstacle motion.
  bool collidesWithObservedPoints(
    const Trajectory &, const std::vector<std::array<double, 2>> &) const;

private:
  double clearance_radius_;
  double sim_time_;
  double simulation_dt_;
  double max_scan_age_;
};

}  // namespace my_dwa_controller
