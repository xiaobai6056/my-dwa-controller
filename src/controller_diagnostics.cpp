#include "my_dwa_controller/controller_diagnostics.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>

#include "rclcpp/rclcpp.hpp"

namespace my_dwa_controller
{
namespace
{
double milliseconds(
  CycleDiagnostics::Clock::time_point from,
  CycleDiagnostics::Clock::time_point to)
{
  return std::chrono::duration<double, std::milli>(to - from).count();
}
}  // namespace

void CycleTiming::stateLockAcquired()
{
  state_wait_ms = milliseconds(started, Clock::now());
}

CycleDiagnostics::CycleDiagnostics()
: CycleDiagnostics(CycleTiming{})
{
}

CycleDiagnostics::CycleDiagnostics(const CycleTiming & timing)
: state_wait_ms(timing.state_wait_ms), started_(timing.started)
{
}

void CycleDiagnostics::setInput(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  const DiagnosticSettings & config)
{
  settings = config;
  frame = pose.header.frame_id;
  input_velocity = {velocity.linear.x, velocity.angular.z};
  debug_pose.x = pose.pose.position.x;
  debug_pose.y = pose.pose.position.y;
  debug_pose.theta = std::numeric_limits<double>::quiet_NaN();
}

void CycleDiagnostics::beginTiming(DiagnosticTiming timing)
{
  timing_ = timing;
  switch (timing) {
    case DiagnosticTiming::Path: stage = "path"; break;
    case DiagnosticTiming::MapLock: stage = "map_lock"; break;
    case DiagnosticTiming::Prediction: stage = "prediction"; break;
    case DiagnosticTiming::Obstacle: stage = "obstacle"; break;
    case DiagnosticTiming::Scoring: stage = "scoring"; break;
  }
  mark_ = Clock::now();
}

void CycleDiagnostics::finishTiming()
{
  const auto elapsed = milliseconds(mark_, Clock::now());
  switch (timing_) {
    case DiagnosticTiming::Path: path_ms = elapsed; break;
    case DiagnosticTiming::MapLock: map_wait_ms = elapsed; break;
    case DiagnosticTiming::Prediction: predict_ms += elapsed; break;
    case DiagnosticTiming::Obstacle: obstacle_ms += elapsed; break;
    case DiagnosticTiming::Scoring: score_ms += elapsed; break;
  }
}

double CycleDiagnostics::elapsedMs() const
{
  return milliseconds(started_, Clock::now());
}

void CycleDiagnostics::recordPath(const std::vector<Pose2D> & path)
{
  path_points = path.size();
  if (!path.empty()) {
    path_first = path.front();
    path_last = path.back();
  }
}

void CycleDiagnostics::recordSamples(const std::vector<Velocity2D> & candidates)
{
  sampled = candidates.size();
  for (const auto & candidate : candidates) {
    v_min = std::min(v_min, candidate.v);
    v_max = std::max(v_max, candidate.v);
    w_min = std::min(w_min, candidate.omega);
    w_max = std::max(w_max, candidate.omega);
  }
}

void CycleDiagnostics::recordValid(const Velocity2D & candidate)
{
  ++valid;
  valid_forward += candidate.v > 0.0;
}

void CycleDiagnostics::recordScored(
  const Velocity2D & candidate, const ScoreResult & score,
  const ObstacleEvaluation & obstacle, double rotation_cost)
{
  ++scored;
  current_candidate_ =
  {true, candidate, score, obstacle.mean_cost, obstacle.max_cost, rotation_cost};
  if (candidate.v > 0.0 && (!forward.present || score.total_cost < forward.score.total_cost)) {
    forward = current_candidate_;
  }
  if (!fastest.present || candidate.v > fastest.command.v ||
    (candidate.v == fastest.command.v && score.total_cost < fastest.score.total_cost))
  {
    fastest = current_candidate_;
  }
}

void CycleDiagnostics::recordBest(bool strictly_better)
{
  chosen = current_candidate_;
  if (strictly_better) {
    ties = 1;
  }
}

void ControllerDiagnostics::reset()
{
  totals_ = {};
}

std::string ControllerDiagnostics::format(
  const CycleDiagnostics & c, double elapsed_ms, const char * error) const
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(6)
      << "[DWA_DEBUG] stage=" << c.stage << " error=" << error
      << " frame=" << c.frame
      << " pose=(" << c.debug_pose.x << "," << c.debug_pose.y << "," << c.debug_pose.theta << ")"
      << " follow_lookahead=" << c.settings.follow_lookahead
      << " heading_lookahead=" << c.settings.heading_lookahead
      << " follow_ref=(" << c.follow_reference.x << "," << c.follow_reference.y << ")"
      << " true_goal_distance=" << c.distance_to_goal
      << " terminal_radius=" << c.terminal_radius
      << " heading_enabled=" << c.heading_enabled
      << " goal_phase=" << c.goal_phase
      << " true_goal_local=(" << c.true_goal.x << "," << c.true_goal.y << "," <<
    c.true_goal.theta << ")"
      << " yaw_error=" << c.yaw_error << " yaw_tolerance=" << c.yaw_tolerance
      << " tolerance_fallback[xy,yaw]=[" << c.xy_fallback << "," << c.yaw_fallback << "]"
      << " approach_cap=" << c.approach_cap << " angular_cap=" << c.angular_cap
      << " angular_braking=" << c.angular_braking << " terminal_rejected=" <<
    c.terminal_rejected
      << " input_vel=(" << c.input_velocity.v << "," << c.input_velocity.omega << ")"
      << " sampled=" << c.sampled << " processed=" << c.processed
      << " valid=" << c.valid << " valid_forward=" << c.valid_forward << " scored=" << c.scored
      << " v_range=(" << c.v_min << "," << c.v_max << ")"
      << " w_range=(" << c.w_min << "," << c.w_max << ")"
      << " reject[empty,nonfinite,bounds,inscribed,lethal,unknown]=["
      << c.rejected[1] << "," << c.rejected[2] << "," << c.rejected[3] << ","
      << c.rejected[4] << "," << c.rejected[5] << "," << c.rejected[6] << "]"
      << " path_n=" << c.path_points
      << " path_first=(" << c.path_first.x << "," << c.path_first.y << ")"
      << " path_last=(" << c.path_last.x << "," << c.path_last.y << ")";
  const auto candidate = [&out](const char * name, const DiagnosticCandidate & value) {
      out << " " << name << "={present:" << value.present
          << ",v:" << value.command.v << ",w:" << value.command.omega
          << ",rotation_cost:" << value.rotation_cost
          << ",total:" << value.score.total_cost << ",costs[ob,speed,goal,path,heading]:["
          << value.score.obstacle_cost << "," << value.score.speed_cost << ","
          << value.score.goal_cost << "," << value.score.path_cost << ","
          << value.score.heading_cost
          << "],map_mean:" << value.mean_obstacle << ",map_max:" << value.max_obstacle <<
        "}";
    };
  candidate("best", c.chosen);
  candidate("best_forward", c.forward);
  candidate("fastest_valid", c.fastest);
  out << " best_ties=" << c.ties
      << " ms[total,state_wait,path,map_wait,predict,obstacle,score]=["
      << elapsed_ms << "," << c.state_wait_ms << "," << c.path_ms << "," << c.map_wait_ms
      << "," << c.predict_ms << "," << c.obstacle_ms << "," << c.score_ms << "]"
      << " cumulative[calls,over50,mean_ms,max_ms]=[" << totals_.calls << ","
      << totals_.over_50ms << "," << totals_.sum_ms / totals_.calls << "," << totals_.max_ms
      << "] weights=[" << c.settings.weights.obstacle << "," << c.settings.weights.speed << ","
      << c.settings.weights.goal << "," << c.settings.weights.path << ","
      << c.settings.weights.heading << "]";
  if (c.conflict_enabled) {
    out << " path_conflict[state,map_id,fresh,known,aligned,history,inner,outer_risk]="
        << "[" << riskStateName(c.risk.state) << "," << c.conflict.map_state_id << ","
        << c.conflict.new_map_state << "," << c.conflict.corridor_known << ","
        << c.conflict.path_aligned << "," << c.conflict.history_comparable << ","
        << c.conflict.inner_conflict << "," << c.conflict.outer_risk << "]"
        << " conflict_core[inner,outer,new]=[" << c.conflict.inner_core_cells << ","
        << c.conflict.outer_core_cells << "," << c.conflict.new_core_cells << "]"
        << " risk[cap,stop_distance,available,wait,cannot_stop]=["
        << c.risk.max_linear_speed << "," << c.risk.stopping_distance << ","
        << c.risk.available_distance << "," << c.risk.allow_wait << ","
        << c.risk.cannot_stop << "] snapshot_ms=" << c.snapshot_ms
        << " conflict_ms=" << c.conflict_ms
        << " known_inner_distance=" << c.conflict.known_inner_distance;
    if (c.risk.preferred_max_linear_speed) {
      out << " fallback_preference[cap,valid,accelerating_escape]=["
          << *c.risk.preferred_max_linear_speed << "," << c.preferred_valid << ","
          << c.accelerating_escape << "]";
    }
  }
  return out.str();
}

void ControllerDiagnostics::report(
  const rclcpp::Logger & logger, const CycleDiagnostics & cycle, const char * error, bool failed)
{
  const double elapsed_ms = cycle.elapsedMs();
  ++totals_.calls;
  totals_.over_50ms += elapsed_ms > 50.0;
  totals_.sum_ms += elapsed_ms;
  totals_.max_ms = std::max(totals_.max_ms, elapsed_ms);
  // Preserve independent INFO/WARN call-site throttles shared across instances.
  // Formatting stays inside the native throttle: no strings for suppressed cycles.
  static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
  if (failed) {
    RCLCPP_WARN_THROTTLE(
      logger, throttle_clock, 1000, "%s", format(
        cycle, elapsed_ms,
        error).c_str());
  } else {
    RCLCPP_INFO_THROTTLE(
      logger, throttle_clock, 1000, "%s", format(
        cycle, elapsed_ms,
        error).c_str());
  }
}

}  // namespace my_dwa_controller
