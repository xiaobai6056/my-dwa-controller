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
  const double xy_fallback = parameter("terminal_xy_tolerance", 0.25);
  const double yaw_fallback = parameter("terminal_yaw_tolerance", 0.05);
  for (double value : {stopped, xy_fallback, yaw_fallback}) {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("terminal thresholds must be finite and positive");
    }
  }
  if (yaw_fallback > std::acos(-1.0) || sampling.acc_v <= 0.0 ||
    sampling.acc_w <= 0.0 || sampling.max_w <= 0.0)
  {
    throw std::invalid_argument(
            "terminal control requires positive acceleration/angular limits and yaw tolerance <= pi");
  }
  auto path_handler = std::make_unique<PathHandler>(tf);

  // 全部配置验证成功后才提交状态，失败时不会留下可激活的半初始化模块。
  node_ = parent;
  plugin_name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  sampling_config_ = sampling;
  configured_max_v_ = sampling.max_v;
  score_weights_ = weights;
  goal_ref_distance_ = goal_ref_distance;
  path_ref_distance_ = path_ref_distance;
  forward_length_ = forward_length;
  follow_lookahead_ = follow_lookahead;
  heading_lookahead_ = heading_lookahead;
  path_handler_ = std::move(path_handler);
  predictor_ = std::move(predictor);
  scorer_ = std::move(scorer);
  active_ = false;
  goal_phase_ = GoalPhase::Normal;
  trans_stopped_velocity_ = stopped;
  terminal_xy_tolerance_ = xy_fallback;
  terminal_yaw_tolerance_ = yaw_fallback;
  sim_time_ = sim_time;
  diagnostics_.reset();
  RCLCPP_INFO(
    node->get_logger(), "Controller %s configured: max_v=%.3f, horizon=%.3f, forward_length=%.3f",
    plugin_name_.c_str(), configured_max_v_, sim_time, forward_length_);
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
  static_assert(
    std::is_nothrow_move_assignable<nav_msgs::msg::Path>::value,
    "committing a validated plan must not throw");
  global_plan_ = std::move(accepted_path);
  // Nav2 resets its GoalChecker for every accepted plan, including same-goal replans.
  goal_phase_ = GoalPhase::Normal;
}

void MyDWAController::activate()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!path_handler_ || !predictor_ || !scorer_ || node_.expired()) {
    throw std::logic_error("controller is not configured");
  }
  active_ = true;
}

void MyDWAController::deactivate()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  active_ = false;
  goal_phase_ = GoalPhase::Normal;
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
  tf_.reset();
  costmap_ros_.reset();
  node_.reset();
  plugin_name_.clear();
  configured_max_v_ = 0.0;
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
  double maximum = configured_max_v_;
  if (speed_limit != nav2_costmap_2d::NO_SPEED_LIMIT) {
    maximum = percentage ? configured_max_v_ * speed_limit / 100.0 :
      std::min(configured_max_v_, speed_limit);
  }
  auto scorer = std::make_unique<TrajectoryScorer>(
    maximum, score_weights_, goal_ref_distance_, path_ref_distance_,
    follow_lookahead_, heading_lookahead_);
  sampling_config_.max_v = maximum;
  scorer_ = std::move(scorer);
}

void MyDWAController::prepareControlInputs(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker, ControlInputs & inputs, CycleDiagnostics & cycle) const
{
  const auto local_frame = costmap_ros_->getGlobalFrameID();
  if (pose.header.frame_id != local_frame) {
    throw std::invalid_argument("robot pose frame must match costmap global frame");
  }
  // PathHandler 校验位姿和路径消息，并按本周期时间完成唯一一组路径 TF。
  cycle.beginTiming(DiagnosticTiming::Path);
  inputs.local_path = path_handler_->getLocalPath(
    global_plan_, pose, local_frame,
    forward_length_, &inputs.distance_to_goal, &inputs.true_goal);
  cycle.finishTiming();
  cycle.recordPath(inputs.local_path);
  // One gate for all candidates, based on the actual robot and true goal.
  // getTolerances() is read-only; isGoalReached() would mutate a stateful checker.
  geometry_msgs::msg::Pose tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (goal_checker && goal_checker->getTolerances(tolerance, velocity_tolerance)) {
    if (std::isfinite(tolerance.position.x) && std::isfinite(tolerance.position.y) &&
      tolerance.position.x >= 0.0 && tolerance.position.y >= 0.0)
    {
      inputs.terminal_radius = std::max(tolerance.position.x, tolerance.position.y);
      cycle.xy_fallback = false;
    }
    try {
      geometry_msgs::msg::Pose yaw_pose;
      yaw_pose.orientation = tolerance.orientation;
      inputs.yaw_tolerance = std::abs(toPose2D(yaw_pose).theta);
      cycle.yaw_fallback = false;
    } catch (const std::invalid_argument &) {
      // Unsupported quaternion fields leave the explicit fallback active.
    }
  }
  cycle.follow_reference = lookAheadOnPath(
    inputs.local_path, PathProjection{inputs.local_path.front(), 0, 0.0, 0.0},
    follow_lookahead_).value().point;
  inputs.initial = toPose2D(pose.pose);
  cycle.debug_pose = inputs.initial;
  cycle.stage = "sampling";
  inputs.current = {velocity.linear.x, velocity.angular.z};
}

void MyDWAController::prepareGoalControl(
  const ControlInputs & inputs, GoalControlContext & goal)
{
  auto candidates = sampleVelocities(inputs.current, sampling_config_);
  if (candidates.empty()) {
    throw nav2_core::PlannerException("No candidate velocities in the dynamic window");
  }
  const double min_v = candidates.front().v;
  double min_abs_w = std::numeric_limits<double>::infinity();
  for (const auto & candidate : candidates) {
    min_abs_w = std::min(min_abs_w, std::abs(candidate.omega));
  }
  if (goal_phase_ == GoalPhase::Normal && inputs.distance_to_goal <= inputs.terminal_radius) {
    goal_phase_ = GoalPhase::Slowing;
  }
  if (goal_phase_ == GoalPhase::Slowing &&
    std::abs(inputs.current.v) <= trans_stopped_velocity_ && min_v == 0.0)
  {
    goal_phase_ = GoalPhase::Rotating;
  }
  goal.yaw_error = angleError(inputs.true_goal.theta, inputs.initial.theta);
  goal.heading_enabled = goal_phase_ == GoalPhase::Normal &&
    inputs.distance_to_goal > inputs.terminal_radius;
  goal.phase = goal_phase_;
  goal.sampling = sampling_config_;
  if (goal_phase_ == GoalPhase::Normal) {
    // Aim inside the XY region, without latching/stopping prematurely outside it.
    goal.approach_cap = std::min(
      sampling_config_.max_v, std::max(
        min_v,
        brakingSpeed(
          inputs.distance_to_goal - 0.5 * inputs.terminal_radius,
          sampling_config_.acc_v, sampling_config_.control_dt)));
    goal.sampling.max_v = goal.approach_cap;
  } else {
    goal.angular_cap = std::min(
      {sampling_config_.max_w,
        brakingSpeed(
          std::abs(goal.yaw_error) - 0.5 * inputs.yaw_tolerance,
          sampling_config_.acc_w, sampling_config_.control_dt),
        std::abs(goal.yaw_error) / sim_time_});
    if (std::abs(goal.yaw_error) <= inputs.yaw_tolerance) {
      goal.angular_cap = 0.0;
    }
    goal.angular_braking = std::abs(inputs.current.omega) > goal.angular_cap ||
      inputs.current.omega * goal.yaw_error < 0.0 || goal.angular_cap == 0.0;
    // Re-sample the same dynamic window with a reachable cap. This includes its
    // boundary even for a coarse grid; never force an unreachable zero.
    goal.sampling.max_w = goal.angular_braking ? min_abs_w : std::max(goal.angular_cap, min_abs_w);
  }
}

bool MyDWAController::isGoalCandidateAllowed(
  const Velocity2D & candidate, const ControlInputs & inputs,
  const GoalControlContext & goal) const
{
  return !(goal.phase != GoalPhase::Normal &&
         ((goal.phase == GoalPhase::Rotating && candidate.v != 0.0) ||
         (goal.phase == GoalPhase::Slowing && candidate.v > std::abs(inputs.current.v)) ||
         (!goal.angular_braking && candidate.omega * goal.yaw_error < 0.0)));
}

MyDWAController::CandidateScore MyDWAController::scoreCandidate(
  const Trajectory & trajectory, const ObstacleEvaluation & obstacle,
  const ControlInputs & inputs, const GoalControlContext & goal) const
{
  CandidateScore result;

  if (goal.phase == GoalPhase::Normal) {
    result.costs = scorer_->score(
      trajectory, inputs.local_path, obstacle.mean_cost,
      goal.heading_enabled);
  } else {
    result.rotation_cost = std::abs(
      angleError(
        inputs.true_goal.theta,
        trajectory.poses.back().theta));
    result.costs.total_cost = result.rotation_cost;
  }
  return result;
}

Velocity2D MyDWAController::selectBestCandidate(
  const ControlInputs & inputs, const GoalControlContext & goal,
  const std::vector<Velocity2D> & candidates, CycleDiagnostics & cycle) const
{
  std::optional<Velocity2D> best;
  double best_cost = std::numeric_limits<double>::infinity();
  auto * costmap = costmap_ros_->getCostmap();
  {
    // TF 已完成；仅在候选评估期间持有一次地图锁，所有候选读取同一地图状态。
    cycle.beginTiming(DiagnosticTiming::MapLock);
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*costmap->getMutex());
    cycle.finishTiming();
    for (const auto & candidate : candidates) {
      ++cycle.processed;
      if (!isGoalCandidateAllowed(candidate, inputs, goal)) {
        ++cycle.terminal_rejected;
        continue;
      }
      cycle.beginTiming(DiagnosticTiming::Prediction);
      const auto trajectory = predictor_->predict(inputs.initial, candidate);
      cycle.finishTiming();
      cycle.beginTiming(DiagnosticTiming::Obstacle);
      ObstacleRejection rejection = ObstacleRejection::None;
      const auto obstacle = evaluateTrajectory(trajectory, *costmap, &rejection);
      cycle.finishTiming();
      if (!obstacle.collision_free) {
        ++cycle.rejected[static_cast<std::size_t>(rejection)];
        continue;
      }
      cycle.recordValid(candidate);
      cycle.beginTiming(DiagnosticTiming::Scoring);
      const auto score = scoreCandidate(trajectory, obstacle, inputs, goal);
      cycle.finishTiming();
      cycle.recordScored(candidate, score.costs, obstacle, score.rotation_cost);
      const bool same_speed_priority = !best || goal.phase != GoalPhase::Slowing ||
        candidate.v == best->v;
      const bool lower_speed = best && goal.phase == GoalPhase::Slowing &&
        candidate.v < best->v;
      if (same_speed_priority && score.costs.total_cost == best_cost) {
        ++cycle.ties;
      }
      // Slowing is lexicographic: smallest collision-free reachable v, then yaw.
      // Ordinary navigation preserves its existing weighted score/tie policy.
      if (!best || lower_speed || (same_speed_priority &&
        (score.costs.total_cost < best_cost ||
        (score.costs.total_cost == best_cost &&
        std::abs(candidate.omega) < std::abs(best->omega)))))
      {
        const bool strictly_better = lower_speed || score.costs.total_cost < best_cost;
        best = candidate;
        best_cost = score.costs.total_cost;
        cycle.recordBest(strictly_better);
      }
    }
  }
  cycle.stage = "selection";
  if (!best) {
    throw nav2_core::PlannerException("No collision-free feasible trajectory");
  }
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
  ControlInputs inputs;
  inputs.terminal_radius = terminal_xy_tolerance_;
  inputs.yaw_tolerance = terminal_yaw_tolerance_;
  GoalControlContext goal;
  goal.sampling = sampling_config_;
  goal.phase = goal_phase_;
  goal.angular_cap = sampling_config_.max_w;
  goal.approach_cap = sampling_config_.max_v;
  const auto logger = node ? node->get_logger() : rclcpp::get_logger("my_dwa_controller");
  const auto report_debug = [&](const char * error, bool failed = false) {
      cycle.heading_enabled = goal.heading_enabled;
      cycle.distance_to_goal = inputs.distance_to_goal;
      cycle.terminal_radius = inputs.terminal_radius;
      cycle.yaw_tolerance = inputs.yaw_tolerance;
      cycle.true_goal = inputs.true_goal;
      cycle.yaw_error = goal.yaw_error;
      cycle.angular_cap = goal.angular_cap;
      cycle.approach_cap = goal.approach_cap;
      cycle.angular_braking = goal.angular_braking;
      cycle.goal_phase = goal_phase_ == GoalPhase::Normal ? "normal" :
        goal_phase_ == GoalPhase::Slowing ? "slowing" : "rotating";
      diagnostics_.report(logger, cycle, error, failed);
    };
  try {
    if (!active_ || !node || !costmap_ros_) {
      throw std::logic_error("controller is not active and configured");
    }
    prepareControlInputs(pose, velocity, goal_checker, inputs, cycle);
    prepareGoalControl(inputs, goal);

    // The first (unconstrained) sampling inside prepareGoalControl must stay before
    // phase updates; this second sampling uses the resulting reachable limits.
    const auto candidates = sampleVelocities(inputs.current, goal.sampling);
    cycle.recordSamples(candidates);
    if (candidates.empty()) {
      throw nav2_core::PlannerException("No candidate velocities in the dynamic window");
    }
    const auto best = selectBestCandidate(inputs, goal, candidates, cycle);

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
