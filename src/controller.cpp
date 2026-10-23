#include "my_dwa_controller/controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "my_dwa_controller/obstacle_evaluator.hpp"
#include "my_dwa_controller/path_geometry.hpp"

namespace my_dwa_controller
{
namespace
{

template<typename T>
T readParameter(
  rclcpp_lifecycle::LifecycleNode & node, const std::string & name, const T & default_value)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, default_value);
  }
  return node.get_parameter(name).get_value<T>();
}

// Speed whose one-period travel plus constant-deceleration stopping distance fits.
double brakingSpeed(double distance, double acceleration, double dt)
{
  const long double a = acceleration;
  const long double step = a * dt;
  const long double twice_ad = 2.0L * a * std::max(0.0, distance);
  return static_cast<double>(twice_ad / (std::sqrt(step * step + twice_ad) + step));
}

double angleError(double target, double actual)
{
  return std::remainder(target - actual, 2.0 * std::acos(-1.0));
}

}  // 匿名命名空间

void MyDWAController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  auto node = parent.lock();
  if (!node || !tf || !costmap_ros || !costmap_ros->getLayeredCostmap()) {
    throw std::invalid_argument("controller requires a live node, TF buffer, and configured costmap");
  }
  if (costmap_ros_) {
    throw std::logic_error("controller must be cleaned up before reconfiguration");
  }
  if (name.empty() || costmap_ros->getGlobalFrameID().empty() ||
    costmap_ros->getBaseFrameID().empty())
  {
    throw std::invalid_argument("controller name and costmap frame identifiers must not be empty");
  }

  const auto parameter = [&node, &name](const std::string & key, double default_value) {
      return readParameter(*node, name + "." + key, default_value);
    };
  SamplingConfig sampling;
  sampling.max_v = parameter("max_vel_x", sampling.max_v);
  sampling.max_w = parameter("max_vel_theta", sampling.max_w);
  sampling.acc_v = parameter("acc_lim_x", sampling.acc_v);
  sampling.acc_w = parameter("acc_lim_theta", sampling.acc_w);
  const double frequency = readParameter(*node, "controller_frequency", 20.0);
  if (!std::isfinite(frequency) || frequency <= 0.0) {
    throw std::invalid_argument("controller_frequency must be finite and positive");
  }
  sampling.control_dt = 1.0 / frequency;
  const auto v_samples = readParameter(
    *node, name + ".vx_samples", static_cast<int64_t>(sampling.v_samples));
  const auto w_samples = readParameter(
    *node, name + ".vtheta_samples", static_cast<int64_t>(sampling.w_samples));
  if (v_samples < 2 || w_samples < 2) {
    throw std::invalid_argument("vx_samples and vtheta_samples must both be at least 2");
  }
  sampling.v_samples = static_cast<std::size_t>(v_samples);
  sampling.w_samples = static_cast<std::size_t>(w_samples);
  // 复用采样器的配置校验；这里只在 configure 时运行一次。
  (void)sampleVelocities({0.0, 0.0}, sampling);

  const double sim_time = parameter("sim_time", 1.7);
  const double simulation_dt = parameter("simulation_dt", 0.05);
  auto predictor = std::make_unique<TrajectoryPredictor>(sim_time, simulation_dt);
  ScoreWeights weights;
  weights.obstacle = parameter("obstacle_weight", weights.obstacle);
  weights.speed = parameter("speed_weight", weights.speed);
  weights.goal = parameter("goal_weight", weights.goal);
  weights.path = parameter("path_weight", weights.path);
  weights.heading = parameter("heading_weight", weights.heading);
  const double goal_ref_distance = parameter("goal_ref_distance", 1.0);
  const double path_ref_distance = parameter("path_ref_distance", 0.5);
  const double follow_lookahead = parameter("follow_lookahead", 0.6);
  const double heading_lookahead = parameter("heading_lookahead", 0.6);
  auto scorer = std::make_unique<TrajectoryScorer>(
    sampling.max_v, weights, goal_ref_distance, path_ref_distance,
    follow_lookahead, heading_lookahead);
  const double forward_length = parameter("forward_length", 2.0);
  if (!std::isfinite(forward_length) || forward_length < sampling.max_v * sim_time) {
    throw std::invalid_argument("forward_length must cover max_vel_x * sim_time");
  }
  const double stopped = parameter("trans_stopped_velocity", 0.02);
  const double fallback_xy_tolerance = parameter("terminal_xy_tolerance", 0.25);
  const double fallback_yaw_tolerance = parameter("terminal_yaw_tolerance", 0.05);
  for (double value : {stopped, fallback_xy_tolerance, fallback_yaw_tolerance}) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("terminal thresholds must be finite and positive");
    }
  }
  if (fallback_yaw_tolerance > std::acos(-1.0) || sampling.acc_v <= 0.0 ||
    sampling.acc_w <= 0.0 || sampling.max_w <= 0.0)
  {
    throw std::invalid_argument(
            "terminal control requires positive acceleration/angular limits and yaw tolerance <= pi");
  }
  auto path_handler = std::make_unique<PathHandler>(tf);

  std::unique_ptr<ScanObstacleTracker> scan_tracker;
  const bool scan_enabled = readParameter(*node, name + ".scan_tracker.enabled", false);
  if (scan_enabled && !readParameter(*node, name + ".path_conflict.enabled", false)) {
    throw std::invalid_argument("scan_tracker.enabled requires path_conflict.enabled");
  }
  if (scan_enabled) {
    ScanTrackerConfig scan;
    scan.topic = readParameter(*node, name + ".scan_tracker.topic", scan.topic);
    scan.fixed_frame = readParameter(*node, name + ".scan_tracker.fixed_frame", scan.fixed_frame);
    scan_tracker = std::make_unique<ScanObstacleTracker>(tf, scan);
  }
  std::unique_ptr<PathConflictMonitor> conflict_monitor;
  std::unique_ptr<RiskGovernor> risk_governor;
  if (readParameter(*node, name + ".path_conflict.enabled", false)) {
    PathConflictConfig conflict;
    conflict.lookahead = parameter("path_conflict.lookahead", conflict.lookahead);
    conflict.outer_margin = parameter("path_conflict.outer_margin", conflict.outer_margin);
    conflict.robot_radius = costmap_ros->getLayeredCostmap()->getCircumscribedRadius();
    if (conflict.robot_radius <= 0.0) {conflict.robot_radius = costmap_ros->getRobotRadius();}
    RiskGovernorConfig risk;
    risk.robot_radius = conflict.robot_radius;
    risk.control_dt = sampling.control_dt;
    risk.angular_acceleration = sampling.acc_w;
    risk.stopped_velocity = stopped;
    risk.warning_speed = parameter("path_conflict.warning_speed", risk.warning_speed);
    risk.brake_acceleration =
      parameter("path_conflict.brake_acceleration", risk.brake_acceleration);
    if (risk.brake_acceleration > sampling.acc_v) {
      throw std::invalid_argument("path_conflict.brake_acceleration must not exceed acc_lim_x");
    }
    risk.reaction_time = parameter("path_conflict.reaction_time", risk.reaction_time);
    risk.stop_margin = parameter("path_conflict.stop_margin", risk.stop_margin);
    const auto clear_states = readParameter(
      *node, name + ".path_conflict.clear_states", static_cast<int64_t>(risk.clear_states));
    if (clear_states <= 0) {
      throw std::invalid_argument("path_conflict.clear_states must be positive");
    }
    risk.clear_states = static_cast<std::size_t>(clear_states);
    conflict_monitor = std::make_unique<PathConflictMonitor>(conflict);
    risk_governor = std::make_unique<RiskGovernor>(risk);
  }

  // 全部配置验证成功后才提交状态，失败时不会留下可激活的半初始化模块。
  node_ = parent;
  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  active_sampling_config_ = sampling;
  configured_max_linear_speed_ = sampling.max_v;
  score_weights_ = weights;
  goal_ref_distance_ = goal_ref_distance;
  path_ref_distance_ = path_ref_distance;
  forward_length_ = forward_length;
  follow_lookahead_ = follow_lookahead;
  heading_lookahead_ = heading_lookahead;
  path_handler_ = std::move(path_handler);
  predictor_ = std::move(predictor);
  scorer_ = std::move(scorer);
  scan_tracker_ = std::move(scan_tracker);
  conflict_monitor_ = std::move(conflict_monitor);
  risk_governor_ = std::move(risk_governor);
  last_risk_state_ = RiskState::Normal;
  active_ = false;
  goal_phase_ = GoalPhase::Normal;
  trans_stopped_velocity_ = stopped;
  fallback_xy_tolerance_ = fallback_xy_tolerance;
  fallback_yaw_tolerance_ = fallback_yaw_tolerance;
  sim_time_ = sim_time;
  diagnostics_.reset();
  RCLCPP_INFO(
    node->get_logger(), "Controller %s configured: max_v=%.3f, horizon=%.3f, forward_length=%.3f",
    plugin_name_.c_str(), configured_max_linear_speed_, sim_time, forward_length_);
}

void MyDWAController::setPlan(const nav_msgs::msg::Path & path)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (path.poses.empty()) {
    throw std::invalid_argument("global plan must not be empty");
  }
  if (path.header.frame_id.empty() || path.header.frame_id.front() == '/') {
    throw std::invalid_argument("global plan frame must be nonempty and have no leading slash");
  }
  for (const auto & pose : path.poses) {
    if (!pose.header.frame_id.empty() && pose.header.frame_id != path.header.frame_id) {
      throw std::invalid_argument("global plan pose frame must match the path frame");
    }
    (void)toPose2D(pose.pose);
  }

  // Validate and allocate before committing either the plan or its terminal state.
  auto accepted_path = path;
  const auto new_goal = toPose2D(path.poses.back().pose);
  const auto old_goal = global_plan_.poses.empty() ? new_goal :
    toPose2D(global_plan_.poses.back().pose);
  const bool new_task = global_plan_.header.frame_id != path.header.frame_id ||
    std::hypot(new_goal.x - old_goal.x, new_goal.y - old_goal.y) > 1e-6 ||
    std::abs(angleError(new_goal.theta, old_goal.theta)) > 1e-6;
  static_assert(
    std::is_nothrow_move_assignable<nav_msgs::msg::Path>::value,
    "committing a validated plan must not throw");
  global_plan_ = std::move(accepted_path);
  if (new_task && conflict_monitor_) {
    conflict_monitor_->reset();
    if (scan_tracker_) {scan_tracker_->reset();}
    risk_governor_->reset();
    last_risk_state_ = RiskState::Normal;
  }
  // Nav2 resets its GoalChecker for every accepted plan, including same-goal replans.
  goal_phase_ = GoalPhase::Normal;
}

void MyDWAController::activate()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!path_handler_ || !predictor_ || !scorer_ || node_.expired()) {
    throw std::logic_error("controller is not configured");
  }
  if (scan_tracker_) {scan_tracker_->start(node_.lock());}
  active_ = true;
}

void MyDWAController::deactivate()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  active_ = false;
  goal_phase_ = GoalPhase::Normal;
  if (scan_tracker_) {scan_tracker_->stop();}
  if (conflict_monitor_) {conflict_monitor_->reset();}
  if (risk_governor_) {risk_governor_->reset();}
  last_risk_state_ = RiskState::Normal;
}

void MyDWAController::cleanup()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  active_ = false;
  goal_phase_ = GoalPhase::Normal;
  global_plan_ = nav_msgs::msg::Path{};
  path_handler_.reset();
  predictor_.reset();
  scorer_.reset();
  if (scan_tracker_) {scan_tracker_->stop();}
  scan_tracker_.reset();
  conflict_monitor_.reset();
  risk_governor_.reset();
  last_risk_state_ = RiskState::Normal;
  tf_.reset();
  costmap_ros_.reset();
  node_.reset();
  plugin_name_.clear();
  configured_max_linear_speed_ = 0.0;
}

void MyDWAController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!scorer_) {
    throw std::logic_error("controller is not configured");
  }
  if (!std::isfinite(speed_limit) || speed_limit < 0.0 || (percentage && speed_limit > 100.0)) {
    throw std::invalid_argument("speed limit must be nonnegative and percentage must not exceed 100");
  }
  double maximum = configured_max_linear_speed_;
  if (speed_limit != nav2_costmap_2d::NO_SPEED_LIMIT) {
    maximum = percentage ? configured_max_linear_speed_ * speed_limit / 100.0 :
      std::min(configured_max_linear_speed_, speed_limit);
  }
  auto scorer = std::make_unique<TrajectoryScorer>(
    maximum, score_weights_, goal_ref_distance_, path_ref_distance_,
    follow_lookahead_, heading_lookahead_);
  active_sampling_config_.max_v = maximum;
  scorer_ = std::move(scorer);
}

void MyDWAController::prepareNavigationSnapshot(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker, NavigationSnapshot & snapshot,
  CycleDiagnostics & cycle) const
{
  const auto local_frame = costmap_ros_->getGlobalFrameID();
  if (pose.header.frame_id != local_frame) {
    throw std::invalid_argument("robot pose frame must match costmap global frame");
  }
  // PathHandler 校验位姿和路径消息，并按本周期时间完成唯一一组路径 TF。
  cycle.beginTiming(DiagnosticTiming::Path);
  snapshot.local_path = path_handler_->getLocalPath(
    global_plan_, pose, local_frame,
    forward_length_, &snapshot.distance_to_goal, &snapshot.final_goal_pose);
  cycle.finishTiming();
  cycle.recordPath(snapshot.local_path);
  // One gate for all candidates, based on the actual robot and true goal.
  // getTolerances() is read-only; isGoalReached() would mutate a stateful checker.
  geometry_msgs::msg::Pose tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (goal_checker && goal_checker->getTolerances(tolerance, velocity_tolerance)) {
    if (std::isfinite(tolerance.position.x) && std::isfinite(tolerance.position.y) &&
      tolerance.position.x >= 0.0 && tolerance.position.y >= 0.0)
    {
      snapshot.xy_tolerance_radius = std::max(tolerance.position.x, tolerance.position.y);
      cycle.xy_fallback = false;
    }
    try {
      geometry_msgs::msg::Pose yaw_pose;
      yaw_pose.orientation = tolerance.orientation;
      snapshot.yaw_tolerance = std::abs(toPose2D(yaw_pose).theta);
      cycle.yaw_fallback = false;
    } catch (const std::invalid_argument &) {
      // Unsupported quaternion fields leave the explicit fallback active.
    }
  }
  cycle.follow_reference = lookAheadOnPath(
    snapshot.local_path, PathProjection{snapshot.local_path.front(), 0, 0.0, 0.0},
    follow_lookahead_).value().point;
  snapshot.robot_pose = toPose2D(pose.pose);
  cycle.debug_pose = snapshot.robot_pose;
  cycle.stage = "sampling";
  snapshot.robot_velocity = {velocity.linear.x, velocity.angular.z};
}

void MyDWAController::prepareControlPolicy(
  const NavigationSnapshot & snapshot, CycleControlPolicy & policy)
{
  auto candidates = sampleVelocities(snapshot.robot_velocity, active_sampling_config_);
  if (candidates.empty()) {
    throw nav2_core::PlannerException("No candidate velocities in the dynamic window");
  }
  const double min_v = candidates.front().v;
  double min_abs_w = std::numeric_limits<double>::infinity();
  for (const auto & candidate : candidates) {
    min_abs_w = std::min(min_abs_w, std::abs(candidate.omega));
  }
  if (goal_phase_ == GoalPhase::Normal &&
    snapshot.distance_to_goal <= snapshot.xy_tolerance_radius)
  {
    goal_phase_ = GoalPhase::Slowing;
  }
  if (goal_phase_ == GoalPhase::Slowing &&
    std::abs(snapshot.robot_velocity.v) <= trans_stopped_velocity_ && min_v == 0.0)
  {
    goal_phase_ = GoalPhase::Rotating;
  }
  policy.yaw_error = angleError(snapshot.final_goal_pose.theta, snapshot.robot_pose.theta);
  policy.use_heading_score = goal_phase_ == GoalPhase::Normal &&
    snapshot.distance_to_goal > snapshot.xy_tolerance_radius;
  policy.phase = goal_phase_;
  policy.sampling = active_sampling_config_;
  if (goal_phase_ == GoalPhase::Normal) {
    // Aim inside the XY region, without latching/stopping prematurely outside it.
    policy.linear_speed_cap = std::min(
      active_sampling_config_.max_v, std::max(
        min_v,
        brakingSpeed(
          snapshot.distance_to_goal - 0.5 * snapshot.xy_tolerance_radius,
          active_sampling_config_.acc_v, active_sampling_config_.control_dt)));
    policy.sampling.max_v = policy.linear_speed_cap;
  } else {
    policy.angular_speed_cap = std::min(
      {active_sampling_config_.max_w,
        brakingSpeed(
          std::abs(policy.yaw_error) - 0.5 * snapshot.yaw_tolerance,
          active_sampling_config_.acc_w, active_sampling_config_.control_dt),
        std::abs(policy.yaw_error) / sim_time_});
    if (std::abs(policy.yaw_error) <= snapshot.yaw_tolerance) {
      policy.angular_speed_cap = 0.0;
    }
    policy.needs_angular_braking =
      std::abs(snapshot.robot_velocity.omega) > policy.angular_speed_cap ||
      snapshot.robot_velocity.omega * policy.yaw_error < 0.0 || policy.angular_speed_cap == 0.0;
    // Re-sample the same dynamic window with a reachable cap. This includes its
    // boundary even for a coarse grid; never force an unreachable zero.
    policy.sampling.max_w = policy.needs_angular_braking ? min_abs_w :
      std::max(policy.angular_speed_cap, min_abs_w);
  }
}

nav2_costmap_2d::Costmap2D MyDWAController::snapshotCostmap(CycleDiagnostics & cycle) const
{
  auto * source = costmap_ros_->getCostmap();
  cycle.beginTiming(DiagnosticTiming::MapLock);
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*source->getMutex());
  cycle.finishTiming();
  const auto started = CycleTiming::Clock::now();
  // Costmap2D already owns geometry and world/grid conversion; no parallel grid type.
  nav2_costmap_2d::Costmap2D snapshot(*source);
  cycle.snapshot_ms = std::chrono::duration<double, std::milli>(
    CycleTiming::Clock::now() - started).count();
  return snapshot;
}

void MyDWAController::applyRiskPolicy(
  const NavigationSnapshot & snapshot, const nav2_costmap_2d::Costmap2D & costmap,
  double observation_time, CycleControlPolicy & policy, CycleDiagnostics & cycle)
{
  // Existing terminal slowing/yaw control remains authoritative near the true goal.
  if (!conflict_monitor_ || policy.phase != GoalPhase::Normal) {return;}
  cycle.conflict_enabled = true;
  const auto started = CycleTiming::Clock::now();
  std::optional<ScanTrackSnapshot> scan;
  if (scan_tracker_) {
    scan = scan_tracker_->snapshot(costmap_ros_->getGlobalFrameID(), observation_time);
  }
  cycle.conflict = conflict_monitor_->observe(
    snapshot.local_path, snapshot.robot_pose, snapshot.robot_velocity, costmap, observation_time,
    scan ? &*scan : nullptr);
  cycle.risk = risk_governor_->decide(
    cycle.conflict, snapshot.robot_velocity, policy.sampling.max_v);
  if (cycle.risk.release_scan_risk) {conflict_monitor_->releaseScanRisk();}
  policy.preferred_max_linear_speed = cycle.risk.preferred_max_linear_speed;
  // A sudden warning cap can be below the dynamic window. Keep its deceleration
  // boundary so the sampler never loses every candidate solely due to this cap.
  const double reachable_min = std::max(
    0.0, snapshot.robot_velocity.v - policy.sampling.acc_v * policy.sampling.control_dt);
  policy.sampling.max_v = std::max(
    reachable_min, std::min(policy.sampling.max_v, cycle.risk.max_linear_speed));
  policy.linear_speed_cap = policy.sampling.max_v;
  if (cycle.risk.allow_wait ||
    (cycle.risk.state == RiskState::Braking && cycle.risk.max_linear_speed == 0.0))
  {
    const double reachable_w = std::max(
      0.0, std::abs(snapshot.robot_velocity.omega) -
      policy.sampling.acc_w * policy.sampling.control_dt);
    policy.sampling.max_w = std::min(policy.sampling.max_w, reachable_w);
  }
  cycle.conflict_ms = std::chrono::duration<double, std::milli>(
    CycleTiming::Clock::now() - started).count();
  if (cycle.conflict.new_map_state || cycle.conflict.scan_new_state ||
    cycle.risk.state != last_risk_state_)
  {
    if (auto node = node_.lock()) {
      RCLCPP_INFO(
        node->get_logger(),
        "[PATH_CONFLICT] t=%.6f state=%s map=%zu kind=%d known=%d prefix=%.3f aligned=%d history=%d "
        "core=%zu/%zu new=%zu inner=%d outer=%d region=%.3f:%.3f "
        "cap=%.3f applied=%.3f stop=%.3f available=%.3f wait=%d cannot_stop=%d ms=%.3f "
        "scan=%d fresh=%d unresolved=%d seq=%zu predicted=%d "
        "area=%d area_known=%d rays_clear=%d map_clear=%d area_clear=%d release=%d",
        observation_time, riskStateName(
          cycle.risk.state), cycle.conflict.map_state_id,
        static_cast<int>(cycle.conflict.map_state), cycle.conflict.corridor_known,
        cycle.conflict.known_inner_distance, cycle.conflict.path_aligned, cycle.conflict.history_comparable,
        cycle.conflict.inner_core_cells, cycle.conflict.outer_core_cells,
        cycle.conflict.new_core_cells, cycle.conflict.inner_conflict, cycle.conflict.outer_risk,
        cycle.conflict.inner.start, cycle.conflict.inner.end,
        cycle.risk.max_linear_speed, policy.sampling.max_v, cycle.risk.stopping_distance,
        cycle.risk.available_distance, cycle.risk.allow_wait, cycle.risk.cannot_stop,
        cycle.conflict_ms, cycle.conflict.scan_enabled, cycle.conflict.scan_fresh,
        cycle.conflict.scan_unresolved, cycle.conflict.scan_sequence,
        cycle.conflict.scan_inner_conflict, cycle.conflict.scan_area_active,
        cycle.conflict.scan_area_known, cycle.conflict.scan_area_rays_clear,
        cycle.conflict.scan_area_map_clear, cycle.conflict.scan_area_clear,
        cycle.risk.release_scan_risk);
      if (scan && cycle.conflict.scan_new_state) {
        RCLCPP_INFO(
          node->get_logger(),
          "[SCAN_STATUS] now=%.6f stamp=%.6f frame=%s seq=%zu fresh=%d unresolved=%d "
          "tracks=%zu tf_failures=%zu dropped=%zu area_center=(%.5f,%.5f,%.5f) "
          "area_length=%.5f lateral=%.5f:%.5f",
          observation_time, scan->stamp, scan->frame.c_str(), scan->sequence,
          scan->fresh, scan->unresolved, scan->tracks.size(), scan->tf_failures, scan->dropped,
          cycle.conflict.scan_area.center.x, cycle.conflict.scan_area.center.y,
          cycle.conflict.scan_area.center.theta, cycle.conflict.scan_area.half_length,
          cycle.conflict.scan_area.lateral_min, cycle.conflict.scan_area.lateral_max);
        for (const auto & relation : cycle.conflict.scan_relations) {
          const auto & t = relation.track;
          RCLCPP_INFO(
            node->get_logger(),
            "[SCAN_TRACK] now=%.6f stamp=%.6f id=%zu x=%.5f y=%.5f vx=%.5f vy=%.5f "
            "radius=%.5f confidence=%.5f residual=%.5f span=%.3f samples=%zu age=%.3f "
            "observed=%d distance=%.5f along=%.5f approaching=%d conflict=%d hit=%.3f state=%s "
            "motion=%s support=%.5f",
            observation_time, t.stamp, t.id, t.x, t.y, t.vx, t.vy, t.radius, t.confidence,
            t.residual, t.span, t.samples, t.age, t.observed, relation.distance, relation.along,
            relation.approaching, relation.conflict, relation.first_hit,
            riskStateName(cycle.risk.state), scanMotionName(t.motion), t.support_change);
        }
      }
    }
  }
  last_risk_state_ = cycle.risk.state;
}

bool MyDWAController::satisfiesGoalConstraints(
  const Velocity2D & candidate, const NavigationSnapshot & snapshot,
  const CycleControlPolicy & policy) const
{
  return !(policy.phase != GoalPhase::Normal &&
         ((policy.phase == GoalPhase::Rotating && candidate.v != 0.0) ||
         (policy.phase == GoalPhase::Slowing &&
         candidate.v > std::abs(snapshot.robot_velocity.v)) ||
         (!policy.needs_angular_braking && candidate.omega * policy.yaw_error < 0.0)));
}

MyDWAController::CandidateScore MyDWAController::scoreCandidate(
  const Trajectory & trajectory, const ObstacleEvaluation & obstacle,
  const NavigationSnapshot & snapshot, const CycleControlPolicy & policy) const
{
  CandidateScore result;

  if (policy.phase == GoalPhase::Normal) {
    result.costs = scorer_->score(
      trajectory, snapshot.local_path, obstacle.mean_cost,
      policy.use_heading_score);
  } else {
    result.rotation_cost = std::abs(
      angleError(
        snapshot.final_goal_pose.theta,
        trajectory.poses.back().theta));
    result.costs.total_cost = result.rotation_cost;
  }
  return result;
}

std::vector<Velocity2D> MyDWAController::sampleControlCandidates(
  const NavigationSnapshot & snapshot, const CycleControlPolicy & policy) const
{
  auto candidates = sampleVelocities(snapshot.robot_velocity, policy.sampling);
  if (!policy.preferred_max_linear_speed || candidates.empty()) {return candidates;}
  const double boundary = *policy.preferred_max_linear_speed;
  if (boundary < candidates.front().v || boundary > candidates.back().v) {return candidates;}
  if (std::any_of(
      candidates.begin(), candidates.end(), [boundary](const Velocity2D & c) {
        return c.v == boundary;
      }))
  {
    return candidates;
  }
  // Governor's boundary lies inside this reachable window. Add that exact speed
  // to avoid unintended deceleration from a coarse grid, retaining every original
  // speed and angular sample for escape. The first row contains all angular values.
  const double first_speed = candidates.front().v;
  const auto row_end = std::find_if(
    candidates.begin(), candidates.end(), [first_speed](const Velocity2D & c) {
      return c.v != first_speed;
    });
  const auto angular_count = static_cast<std::size_t>(row_end - candidates.begin());
  candidates.reserve(candidates.size() + angular_count);
  for (std::size_t i = 0; i < angular_count; ++i) {
    const double omega = candidates[i].omega;
    candidates.push_back({boundary, omega});
  }
  return candidates;
}

Velocity2D MyDWAController::selectBestCandidate(
  const NavigationSnapshot & snapshot, const CycleControlPolicy & policy,
  const std::vector<Velocity2D> & candidates, const nav2_costmap_2d::Costmap2D & costmap,
  CycleDiagnostics & cycle) const
{
  std::optional<Velocity2D> best;
  double best_cost = std::numeric_limits<double>::infinity();
  // Prediction, map queries, scoring and diagnostics all use the immutable copy.
  for (const auto & candidate : candidates) {
    ++cycle.processed;
    if (!satisfiesGoalConstraints(candidate, snapshot, policy)) {
      ++cycle.terminal_rejected;
      continue;
    }
    cycle.beginTiming(DiagnosticTiming::Prediction);
    const auto trajectory = predictor_->predict(snapshot.robot_pose, candidate);
    cycle.finishTiming();
    cycle.beginTiming(DiagnosticTiming::Obstacle);
    ObstacleRejection rejection = ObstacleRejection::None;
    const auto obstacle = evaluateTrajectory(trajectory, costmap, &rejection);
    cycle.finishTiming();
    if (!obstacle.collision_free) {
      ++cycle.rejected[static_cast<std::size_t>(rejection)];
      continue;
    }
    cycle.recordValid(candidate);
    const bool preferred = !policy.preferred_max_linear_speed ||
      candidate.v <= *policy.preferred_max_linear_speed;
    const bool best_preferred = best && (!policy.preferred_max_linear_speed ||
      best->v <= *policy.preferred_max_linear_speed);
    if (policy.preferred_max_linear_speed && preferred) {++cycle.preferred_valid;}
    cycle.beginTiming(DiagnosticTiming::Scoring);
    const auto score = scoreCandidate(trajectory, obstacle, snapshot, policy);
    cycle.finishTiming();
    cycle.recordScored(candidate, score.costs, obstacle, score.rotation_cost);
    const bool higher_priority = best && preferred && !best_preferred;
    const bool same_speed_priority = !best || (preferred == best_preferred &&
      (policy.phase != GoalPhase::Slowing || candidate.v == best->v));
    const bool lower_speed = best && policy.phase == GoalPhase::Slowing &&
      candidate.v < best->v;
    if (same_speed_priority && score.costs.total_cost == best_cost) {
      ++cycle.ties;
    }
    // Slowing is lexicographic: smallest collision-free reachable v, then yaw.
    // Dangerous fallback first prefers feasible non-accelerating commands. Faster
    // commands stay eligible when that entire group is blocked. Within each group,
    // preserve the original weighted score and angular tie-break.
    if (!best || higher_priority || lower_speed || (same_speed_priority &&
      (score.costs.total_cost < best_cost ||
      (score.costs.total_cost == best_cost &&
      std::abs(candidate.omega) < std::abs(best->omega)))))
    {
      const bool strictly_better = higher_priority || lower_speed ||
        score.costs.total_cost < best_cost;
      best = candidate;
      best_cost = score.costs.total_cost;
      cycle.recordBest(strictly_better);
    }
  }
  cycle.stage = "selection";
  if (!best) {
    throw nav2_core::PlannerException("No collision-free feasible trajectory");
  }
  cycle.accelerating_escape = policy.preferred_max_linear_speed &&
    best->v > *policy.preferred_max_linear_speed;
  return *best;
}

geometry_msgs::msg::TwistStamped MyDWAController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  CycleTiming timing;
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  timing.stateLockAcquired();
  auto node = node_.lock();
  CycleDiagnostics cycle(timing);
  cycle.setInput(pose, velocity, {follow_lookahead_, heading_lookahead_, score_weights_});
  NavigationSnapshot snapshot;
  snapshot.xy_tolerance_radius = fallback_xy_tolerance_;
  snapshot.yaw_tolerance = fallback_yaw_tolerance_;
  CycleControlPolicy policy;
  policy.sampling = active_sampling_config_;
  policy.phase = goal_phase_;
  policy.angular_speed_cap = active_sampling_config_.max_w;
  policy.linear_speed_cap = active_sampling_config_.max_v;
  const auto logger = node ? node->get_logger() : rclcpp::get_logger("my_dwa_controller");
  const auto report_debug = [&](const char * error, bool failed = false) {
      cycle.heading_enabled = policy.use_heading_score;
      cycle.distance_to_goal = snapshot.distance_to_goal;
      cycle.terminal_radius = snapshot.xy_tolerance_radius;
      cycle.yaw_tolerance = snapshot.yaw_tolerance;
      cycle.true_goal = snapshot.final_goal_pose;
      cycle.yaw_error = policy.yaw_error;
      cycle.angular_cap = policy.angular_speed_cap;
      cycle.approach_cap = policy.linear_speed_cap;
      cycle.angular_braking = policy.needs_angular_braking;
      cycle.goal_phase = goal_phase_ == GoalPhase::Normal ? "normal" :
        goal_phase_ == GoalPhase::Slowing ? "slowing" : "rotating";
      diagnostics_.report(logger, cycle, error, failed);
    };
  try {
    if (!active_ || !node || !costmap_ros_) {
      throw std::logic_error("controller is not active and configured");
    }
    prepareNavigationSnapshot(pose, velocity, goal_checker, snapshot, cycle);
    prepareControlPolicy(snapshot, policy);
    const auto costmap = snapshotCostmap(cycle);
    applyRiskPolicy(snapshot, costmap, node->now().seconds(), policy, cycle);

    // The first (unconstrained) sampling inside prepareControlPolicy must stay before
    // phase updates; this second sampling uses the resulting reachable limits.
    const auto candidates = sampleControlCandidates(snapshot, policy);
    cycle.recordSamples(candidates);
    if (candidates.empty()) {
      throw nav2_core::PlannerException("No candidate velocities in the dynamic window");
    }
    const auto best = selectBestCandidate(snapshot, policy, candidates, costmap, cycle);
    if (policy.preferred_max_linear_speed && cycle.conflict.new_map_state) {
      RCLCPP_INFO(
        logger, "[FALLBACK_SELECTION] t=%.6f input=%.6f preferred_cap=%.6f "
        "preferred_valid=%zu valid=%zu sampled=%zu selected=(%.6f,%.6f) accelerating_escape=%d",
        node->now().seconds(), snapshot.robot_velocity.v, *policy.preferred_max_linear_speed,
        cycle.preferred_valid, cycle.valid, cycle.sampled, best.v, best.omega,
        cycle.accelerating_escape);
    }

    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = node->now();
    command.header.frame_id = costmap_ros_->getBaseFrameID();
    command.twist.linear.x = best.v;
    command.twist.angular.z = best.omega;
    cycle.stage = "output";
    report_debug("none");
    return command;
  } catch (const std::exception & error) {
    report_debug(error.what(), true);
    throw;
  } catch (...) {
    report_debug("non-standard exception", true);
    throw;
  }
}

}  // 命名空间 my_dwa_controller

PLUGINLIB_EXPORT_CLASS(my_dwa_controller::MyDWAController, nav2_core::Controller)
