#pragma once

#include "my_dwa_controller/scan_obstacle_tracker.hpp"
#include "my_dwa_controller/types.hpp"
#include "my_dwa_controller/motion_predictor.hpp"

namespace my_dwa_controller
{
struct YieldDecision
{
  bool hold = false, triggered = false, released = false;
  // Latched: the cached region has been occupied at least once, not necessarily now.
  bool occupied = false;
  // Fresh-source ray result for this call; repeated reads leave this false.
  bool clear = false;
  // Diagnostic brake endpoint margin only. Does not gate hold or prove safety.
  bool safe_stop = false;
  int clear_scans = 0, evidence_rank = 0;
  // Absolute projections onto the frozen path tangent, including the 0.27 m envelope.
  double entry = 0, exit = 0, margin = 0;
};

class CrossingYield
{
public:
  // All geometry, path, robot and brake poses must share the snapshot frame.
  // now_ns uses the same ROS clock as Scan source stamps. Null/stale/future
  // observations retain pending risk and hold until risk release.
  // Path changes alone cannot cancel hold. Admission describes this cycle's DWA
  // candidate; no command is cached here. Callers apply hold to their output.
  YieldDecision update(
    const ScanSnapshot * scan, const std::vector<Pose2D> & path, const Pose2D & robot,
    const Velocity2D & velocity, const Trajectory & brake, int64_t now_ns,
    bool candidate_admitted);
  // Read-only admission; risk tracking and release remain in update().
  bool canApproach(
    const std::vector<Pose2D> & path, const MotionRollout & current_brake,
    const MotionRollout & candidate_brake, double horizon, double step,
    double speed_bound) const;
  // Call after update and approach admission; only risk release/reset clears this latch.
  bool latchHold(bool hold);
  void reset();
  bool pending() const {return region_pending_;}
  bool holding() const {return region_pending_ && stop_latched_;}
  // Read-only view of this event on an adjacent (or already consumed) Scan.
  // Seam fragments use the Tracker's existing adjacency distance, without rebinding.
  std::vector<std::size_t> targetClusters(const ScanSnapshot & scan) const;
  static bool hasOtherRisk(
    const ScanSnapshot &, const std::vector<Pose2D> &, const Pose2D &,
    const std::vector<std::size_t> & excluded);

private:
  bool establishRegion(
    const ScanSnapshot & scan, const std::vector<Pose2D> & path, const Pose2D & robot);
  void extendRegion(const ScanSnapshot & scan, int64_t previous_consumed_stamp);
  bool observeOccupancy(const ScanSnapshot & scan);
  enum class RegionObservation {Unavailable, Blocked, Clear};
  RegionObservation observeRegion(const ScanSnapshot & scan) const;
  bool pathIntersectsRegion(const std::vector<Pose2D> & path) const;
  bool updateEmptyDuration(RegionObservation observation, bool occupied, int64_t stamp);
  bool canRelease(const Velocity2D & velocity, bool candidate_admitted) const;
  void fillDecision(
    YieldDecision & result, const std::vector<Pose2D> & path, const Trajectory & brake) const;

  bool region_pending_ = false, ever_occupied_ = false, stop_latched_ = false;
  int64_t last_stamp_ = 0;
  int64_t empty_duration_ns_ = 0, last_empty_stamp_ = 0;
  int clear_scans_ = 0, cluster_index_ = -1, evidence_rank_ = 0;
  double tangent_x_ = 1, tangent_y_ = 0, normal_x_ = 0, normal_y_ = 1;
  double path_normal_center_ = 0, region_low_ = 0, region_high_ = 0;
};
}  // namespace my_dwa_controller
