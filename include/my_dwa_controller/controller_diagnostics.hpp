#ifndef MY_DWA_CONTROLLER__CONTROLLER_DIAGNOSTICS_HPP_
#define MY_DWA_CONTROLLER__CONTROLLER_DIAGNOSTICS_HPP_

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "rclcpp/logger.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "my_dwa_controller/obstacle_evaluator.hpp"
#include "my_dwa_controller/trajectory_scorer.hpp"
#include "my_dwa_controller/risk_governor.hpp"

namespace my_dwa_controller
{

struct DiagnosticCandidate
{
  bool present = false;
  Velocity2D command{};
  ScoreResult score{};
  double mean_obstacle = 0.0;
  unsigned int max_obstacle = 0;
  double rotation_cost = 0.0;
};

struct DiagnosticSettings
{
  double follow_lookahead;
  double heading_lookahead;
  ScoreWeights weights;
};

enum class DiagnosticTiming {Path, MapLock, Prediction, Obstacle, Scoring};

/// Only the two clock readings cross the Controller state-lock boundary.
struct CycleTiming
{
  using Clock = std::chrono::steady_clock;
  Clock::time_point started = Clock::now();
  double state_wait_ms = 0.0;
  void stateLockAcquired();
};

/// One-cycle observer snapshots only; no trajectories or path vectors are retained.
struct CycleDiagnostics
{
  using Clock = CycleTiming::Clock;
  CycleDiagnostics();
  explicit CycleDiagnostics(const CycleTiming & timing);
  void setInput(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity, const DiagnosticSettings & config);
  void beginTiming(DiagnosticTiming timing);
  void finishTiming();
  void recordPath(const std::vector<Pose2D> & path);
  void recordSamples(const std::vector<Velocity2D> & candidates);
  void recordValid(const Velocity2D & candidate);
  void recordScored(
    const Velocity2D & candidate, const ScoreResult & score,
    const ObstacleEvaluation & obstacle, double rotation_cost);
  void recordBest(bool strictly_better);
  double elapsedMs() const;

  DiagnosticSettings settings{};
  // Borrows the computeVelocityCommands pose argument through the synchronous report.
  std::string_view frame;
  Velocity2D input_velocity{};
  Pose2D debug_pose{}, path_first{}, path_last{}, follow_reference{}, true_goal{};
  DiagnosticCandidate chosen, forward, fastest;
  std::array<std::size_t, 7> rejected{};
  std::size_t sampled = 0, processed = 0, valid = 0, valid_forward = 0, scored = 0, ties = 0;
  std::size_t terminal_rejected = 0, path_points = 0;
  double v_min = INFINITY, v_max = -INFINITY, w_min = INFINITY, w_max = -INFINITY;
  double state_wait_ms = 0.0, path_ms = 0.0, map_wait_ms = 0.0;
  double predict_ms = 0.0, obstacle_ms = 0.0, score_ms = 0.0;
  bool heading_enabled = false, xy_fallback = true, yaw_fallback = true;
  double distance_to_goal = std::numeric_limits<double>::quiet_NaN();
  double terminal_radius = 0.0, yaw_tolerance = 0.0;
  double yaw_error = 0.0, angular_cap = 0.0, approach_cap = 0.0;
  bool angular_braking = false;
  const char * goal_phase = "normal";
  const char * stage = "state";
  bool conflict_enabled = false;
  ConflictEvidence conflict;
  RiskDecision risk;
  double conflict_ms = 0.0, snapshot_ms = 0.0;
  std::size_t preferred_valid = 0;
  bool accelerating_escape = false;

private:
  Clock::time_point started_, mark_;
  DiagnosticTiming timing_ = DiagnosticTiming::Path;
  DiagnosticCandidate current_candidate_;
};

struct DiagnosticTotals
{
  std::size_t calls = 0;
  std::size_t over_50ms = 0;
  double sum_ms = 0.0;
  double max_ms = 0.0;
};

/// Synchronous logger; callers provide the existing Controller state-lock protection.
class ControllerDiagnostics
{
public:
  void reset();
  void report(
    const rclcpp::Logger & logger, const CycleDiagnostics & cycle,
    const char * error, bool failed = false);
  std::string format(
    const CycleDiagnostics & cycle, double elapsed_ms, const char * error) const;
  const DiagnosticTotals & totals() const {return totals_;}

private:
  DiagnosticTotals totals_;
};

}  // namespace my_dwa_controller
#endif  // MY_DWA_CONTROLLER__CONTROLLER_DIAGNOSTICS_HPP_
