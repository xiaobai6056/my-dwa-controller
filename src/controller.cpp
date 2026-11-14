#include "my_dwa_controller/controller.hpp"
#include "my_dwa_controller/path_conflict_evaluator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <iomanip>
#include <sstream>

#include "rcutils/logging.h"
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
  MotionConfig motion;
  motion.horizon = sim_time;
  motion.step = simulation_dt;
  motion.linear_acceleration = parameter("motion.linear_acceleration", 1.0);
  motion.angular_acceleration = parameter("motion.angular_acceleration", 1.0);
  motion.response_delay = parameter("motion.response_delay", 0.12);
  auto predictor = std::make_unique<MotionPredictor>(motion);
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
  std::shared_ptr<ScanObstacleTracker> scan_tracker;
  std::string scan_topic;
  if (readParameter(*node, name + ".scan_tracker.enabled", false)) {
    scan_topic = readParameter(*node, name + ".scan_tracker.topic", std::string("/scan"));
    const auto min_points = readParameter(
      *node, name + ".scan_tracker.min_cluster_points", int64_t{3});
    if (scan_topic.empty() || min_points <= 0) {
      throw std::invalid_argument("scan topic must be nonempty and minimum points positive");
    }
    scan_tracker = std::make_shared<ScanObstacleTracker>(
      tf, parameter("scan_tracker.cluster_distance", 0.12),
      static_cast<std::size_t>(min_points), parameter("scan_tracker.association_distance", 0.35));
  }
  auto path_handler = std::make_unique<PathHandler>(tf);
  const bool crossing_enabled = readParameter(*node, name + ".crossing_yield.enabled", false);
  const bool pedestrian_enabled =
    readParameter(*node, name + ".pedestrian_collision.enabled", false);
  if (crossing_enabled && pedestrian_enabled) {
    throw std::invalid_argument("pedestrian_collision and crossing_yield are mutually exclusive");
  }
  std::optional<PedestrianConfig> pedestrian_config;
  if (pedestrian_enabled) {
    PedestrianConfig config;
    const auto read = [&](const char * key, double & value) {
        value = parameter(std::string("pedestrian_collision.") + key, value);
      };
    read("max_position_age", config.max_position_age);
    read("max_velocity_age", config.max_velocity_age);
    read("retention", config.retention);
    read("kf_max_cache_age", config.kf_max_cache_age);
    read("horizon", config.horizon);
    read("max_speed", config.max_speed);
    read("min_difference_dt", config.min_difference_dt);
    read("max_difference_dt", config.max_difference_dt);
    read("position_buffer", config.position_buffer);
    read("ols_growth", config.ols_growth);
    read("difference_growth", config.difference_growth);
    read("held_growth", config.held_growth);
    read("unknown_growth", config.unknown_growth);
    read("max_growth", config.max_growth);
    config.soft_growth_enabled = readParameter(
      *node, name + ".pedestrian_collision.soft_growth_enabled", false);
    read("soft_growth_weight", config.soft_growth_weight);
    if (config.horizon < sim_time) {
      throw std::invalid_argument("pedestrian prediction horizon must cover sim_time");
    }
    // Validate before committing any of the controller configuration.
    (void)PedestrianInput(config, costmap_ros->getGlobalFrameID(), tf);
    pedestrian_config = config;
  }

  const bool post_loss_cap = readParameter(*node, name + ".post_loss_cap.enabled", true);
  const bool timing = readParameter(*node, name + ".diagnostics.control_timing", false);

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
  scan_tracker_ = std::move(scan_tracker);
  crossing_yield_ = crossing_enabled ? std::make_unique<CrossingYield>() : nullptr;
  pedestrian_config_ = pedestrian_config;
  post_loss_cap_enabled_ = post_loss_cap;
  timing_enabled_ = timing;
  visualization_enabled_ = readParameter(
    *node, plugin_name_ + ".diagnostics.visualization", false);
  RCLCPP_INFO(
    node->get_logger(), "Crossing yield configured: enabled=%s",
    crossing_yield_ ? "true" : "false");
  scan_topic_ = std::move(scan_topic);
  odom_topic_ = scan_tracker_ && crossing_yield_ ?
    readParameter(*node, "odom_topic", std::string("odom")) : "";
  resetDynamicState();
  path_handler_ = std::move(path_handler);
  predictor_ = std::move(predictor);
  scorer_ = std::move(scorer);
  active_ = false;
  goal_phase_ = GoalPhase::Normal;
  trans_stopped_velocity_ = stopped;
  terminal_xy_tolerance_ = xy_fallback;
  terminal_yaw_tolerance_ = yaw_fallback;
  sim_time_ = sim_time;
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
  auto node = node_.lock();
  if (!path_handler_ || !predictor_ || !scorer_ || !node) {
    throw std::logic_error("controller is not configured");
  }
  if (scan_tracker_) {
    scan_tracker_ = scan_tracker_->newSession();
    if (crossing_yield_) {crossing_yield_->reset();}
    const auto target_frame = costmap_ros_->getGlobalFrameID();
    const auto process = [tracker = std::weak_ptr<ScanObstacleTracker>(scan_tracker_),
        weak_node = node_, target_frame]() {
        if (auto current = tracker.lock()) {
          if (auto parent = weak_node.lock()) {
            current->processPending(target_frame, parent->now());
          }
        }
      };
    // Poll on this existing node/executor, without waiting inside Scan or TF callbacks.
    scan_tf_timer_ = node->create_wall_timer(std::chrono::milliseconds(10), process);
    scan_subscription_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [tracker = std::weak_ptr<ScanObstacleTracker>(scan_tracker_), process](
        sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
        if (auto current = tracker.lock()) {
          current->enqueue(std::move(scan));
          process();
        }
      });
  }
  if (scan_tracker_ && crossing_yield_) {
    robot_stop_ = std::make_shared<RobotStopObserver>();
    const auto frame = costmap_ros_->getGlobalFrameID();
    const auto base = costmap_ros_->getBaseFrameID();
    odom_subscription_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [observer = std::weak_ptr<RobotStopObserver>(robot_stop_), frame, base](
        nav_msgs::msg::Odometry::ConstSharedPtr odom) {
        if (auto current = observer.lock()) {
          if (odom->header.frame_id != frame ||
          (odom->child_frame_id != base && odom->child_frame_id != "base_footprint") ||
          odom->header.stamp.sec < 0 || odom->header.stamp.nanosec >= 1000000000u)
          {
            current->reset();return;
          }
          const auto & v = odom->twist.twist;
          current->feed(
            rclcpp::Time(odom->header.stamp).nanoseconds(),
            v.linear.x, v.linear.y, v.angular.z);
        }
      });
  }
  if (pedestrian_config_) {
    pedestrian_input_ = std::make_shared<PedestrianInput>(
      *pedestrian_config_, costmap_ros_->getGlobalFrameID(), tf_);
    pedestrian_subscription_ = node->create_subscription<msg::PedestrianStateArray>(
      "/my_dwa/pedestrians", rclcpp::QoS(10),
      [cache = std::weak_ptr<PedestrianInput>(pedestrian_input_), parent = node_](
        msg::PedestrianStateArray::ConstSharedPtr batch) {
        if (auto current = cache.lock()) {
          if (auto owner = parent.lock()) {
            current->receive(*batch, owner->now().nanoseconds());
          }
        }
      });
    all_dynamic_rejected_cycles_ = 0;
  }
  resetDynamicState();
  if (visualization_enabled_) {
    const auto qos = rclcpp::QoS(1).transient_local();
    global_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("/my_dwa/demo/global_plan", qos);
    local_plan_pub_ = node->create_publisher<nav_msgs::msg::Path>("/my_dwa/demo/local_plan", qos);
    trajectory_pub_ =
      node->create_publisher<nav_msgs::msg::Path>("/my_dwa/demo/dwa_trajectory", qos);
    prediction_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/my_dwa/demo/predictions", qos);
    global_plan_pub_->on_activate();
    local_plan_pub_->on_activate();
    trajectory_pub_->on_activate();
    prediction_pub_->on_activate();
  }
  active_ = true;
}

void MyDWAController::deactivate()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  active_ = false;
  resetDynamicState();
  global_plan_pub_.reset();
  local_plan_pub_.reset();
  trajectory_pub_.reset();
  prediction_pub_.reset();
  pedestrian_subscription_.reset();
  pedestrian_input_.reset();
  odom_subscription_.reset();
  robot_stop_.reset();
  scan_subscription_.reset();
  scan_tf_timer_.reset();
  if (scan_tracker_) {
    scan_tracker_->reset();
  }
  goal_phase_ = GoalPhase::Normal;
}

void MyDWAController::cleanup()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  active_ = false;
  resetDynamicState();
  global_plan_pub_.reset();
  local_plan_pub_.reset();
  trajectory_pub_.reset();
  prediction_pub_.reset();
  pedestrian_subscription_.reset();
  pedestrian_input_.reset();
  odom_subscription_.reset();
  robot_stop_.reset();
  odom_topic_.clear();
  goal_phase_ = GoalPhase::Normal;
  global_plan_ = nav_msgs::msg::Path{};
  scan_subscription_.reset();
  scan_tf_timer_.reset();
  scan_tracker_.reset();
  crossing_yield_.reset();
  pedestrian_config_.reset();
  scan_topic_.clear();
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

std::shared_ptr<const ScanSnapshot> MyDWAController::getScanSnapshot()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return active_ && scan_tracker_ ? scan_tracker_->snapshot() : nullptr;
}

void MyDWAController::prepareControlInputs(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker, ControlInputs & inputs) const
{
  const auto local_frame = costmap_ros_->getGlobalFrameID();
  if (pose.header.frame_id != local_frame) {
    throw std::invalid_argument("robot pose frame must match costmap global frame");
  }
  // PathHandler 校验位姿和路径消息，并按本周期时间完成唯一一组路径 TF。
  inputs.local_path = path_handler_->getLocalPath(
    global_plan_, pose, local_frame,
    forward_length_, &inputs.distance_to_goal, &inputs.true_goal);
  // One gate for all candidates, based on the actual robot and true goal.
  // getTolerances() is read-only; isGoalReached() would mutate a stateful checker.
  geometry_msgs::msg::Pose tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (goal_checker && goal_checker->getTolerances(tolerance, velocity_tolerance)) {
    if (std::isfinite(tolerance.position.x) && std::isfinite(tolerance.position.y) &&
      tolerance.position.x >= 0.0 && tolerance.position.y >= 0.0)
    {
      inputs.terminal_radius = std::max(tolerance.position.x, tolerance.position.y);
    }
    try {
      geometry_msgs::msg::Pose yaw_pose;
      yaw_pose.orientation = tolerance.orientation;
      inputs.yaw_tolerance = std::abs(toPose2D(yaw_pose).theta);
    } catch (const std::invalid_argument &) {
      // Unsupported quaternion fields leave the explicit fallback active.
    }
  }
  inputs.initial = toPose2D(pose.pose);
  inputs.current = {velocity.linear.x, velocity.angular.z};
}

void MyDWAController::prepareGoalControl(
  const ControlInputs & inputs, GoalControlContext & goal)
{
  auto candidates = sampleVelocities(inputs.current, sampling_config_);
  if (candidates.empty()) {
    goal.phase = goal_phase_;
    goal.sampling = sampling_config_;
    return;
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

void MyDWAController::resetDynamicState()
{
  post_loss_cap_ = PostLossCap{};
  dynamic_active_ = true;
  handoff_stamp_ = last_control_time_ = 0;
  quasi_static_.reset();
  if (robot_stop_) {robot_stop_->reset();}
  if (crossing_yield_) {crossing_yield_->reset();}
}

bool MyDWAController::hasMotionRisk(
  const ScanSnapshot & scan, const ControlInputs & inputs, const rclcpp::Time & now,
  const std::vector<std::size_t> & excluded) const
{
  ScanSnapshot moving;
  moving.stamp = scan.stamp;moving.frame_id = scan.frame_id;
  for (std::size_t i = 0; i < scan.obstacles.size(); ++i) {
    const auto & o = scan.obstacles[i];
    if (std::find(excluded.begin(), excluded.end(), i) != excluded.end()) {continue;}
    if (o.velocity_valid && o.previous_cluster_index >= 0 &&
      std::isfinite(o.vx) && std::isfinite(o.vy) && std::hypot(o.vx, o.vy) >= .05)
    {
      moving.obstacles.push_back(o);
    }
  }
  if (moving.obstacles.empty()) {return false;}
  const double radius = costmap_ros_->getUseRadius() ? costmap_ros_->getRobotRadius() :
    costmap_ros_->getLayeredCostmap()->getCircumscribedRadius();
  const PathConflictEvaluator risk(radius, .05, sim_time_, predictor_->step(), .35);
  const auto result = risk.evaluate(
    &moving, inputs.local_path, inputs.initial,
    costmap_ros_->getGlobalFrameID(), now, sampling_config_.max_v);
  return result.valid && result.conflict_distance.has_value();
}

void MyDWAController::updateDynamicMode(
  const ScanSnapshot * scan, const ControlInputs & inputs, const rclcpp::Time & now)
{
  if (last_control_time_ && now.nanoseconds() < last_control_time_) {resetDynamicState();}
  last_control_time_ = now.nanoseconds();
  if (!scan || scan->frame_id != costmap_ros_->getGlobalFrameID()) {
    quasi_static_.reset();return;
  }
  const auto stamp = rclcpp::Time(scan->stamp).nanoseconds();
  if (stamp <= 0 || now.nanoseconds() < stamp || now.nanoseconds() - stamp > 350000000) {
    quasi_static_.reset();return;
  }
  if (!dynamic_active_ && stamp > handoff_stamp_ && hasMotionRisk(*scan, inputs, now)) {
    resetDynamicState();
    last_control_time_ = now.nanoseconds();
    RCLCPP_INFO(
      node_.lock()->get_logger(),
      "Global handoff: {\"transition\":\"dynamic\",\"reason\":\"trusted_path_motion\",\"source\":%ld,\"now\":%ld}",
      stamp, now.nanoseconds());
  }
  if (!dynamic_active_ || !crossing_yield_ || !crossing_yield_->holding()) {
    quasi_static_.reset();return;
  }
  const auto members = crossing_yield_->targetClusters(*scan);
  if (members.empty()) {quasi_static_.reset();return;}
  // Like the experiment, wait for odom to cover this Scan, then query causally.
  if (!robot_stop_ || !robot_stop_->covers(stamp)) {return;}
  std::vector<Point> points;
  bool associated = true, moving = false;
  for (auto i : members) {
    const auto & o = scan->obstacles[i];
    associated = associated && o.previous_cluster_index >= 0;
    moving = moving || (o.velocity_valid && std::hypot(o.vx, o.vy) >= .05);
    points.insert(points.end(), o.points.begin(), o.points.end());
  }
  const bool stopped = robot_stop_->stoppedAt(stamp, now.nanoseconds());
  const auto result = quasi_static_.update(
    stamp,
    now.nanoseconds(), points, associated, stopped, moving);
  RCLCPP_DEBUG(
    node_.lock()->get_logger(),
    "M observation: {\"source\":%ld,\"now\":%ld,\"frames\":%zu,\"span_ns\":%ld,\"diameter\":%.12g,\"permission\":%s,\"stopped\":%s,\"reason\":\"%s\"}",
    stamp, now.nanoseconds(), result.frames, result.span_ns, result.diameter,
    result.permission ? "true" : "false", stopped ? "true" : "false", result.reason);
  if (!result.permission || hasMotionRisk(*scan, inputs, now, members) ||
    CrossingYield::hasOtherRisk(*scan, inputs.local_path, inputs.initial, members)) {return;}
  dynamic_active_ = false;
  handoff_stamp_ = stamp;
  crossing_yield_->reset();
  quasi_static_.reset();
  RCLCPP_INFO(
    node_.lock()->get_logger(),
    "Global handoff: {\"transition\":\"static\",\"reason\":\"median_static\",\"source\":%ld,\"now\":%ld}",
    stamp, now.nanoseconds());
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

ScoreResult MyDWAController::scoreCandidate(
  const Trajectory & trajectory, double obstacle_cost,
  const ControlInputs & inputs, const GoalControlContext & goal) const
{
  ScoreResult result{};

  if (goal.phase == GoalPhase::Normal) {
    result = scorer_->score(
      trajectory, inputs.local_path, obstacle_cost,
      goal.heading_enabled);
  } else {
    result.total_cost = std::abs(
      angleError(
        inputs.true_goal.theta,
        trajectory.poses.back().theta));
  }
  return result;
}

void MyDWAController::publishVisualization(
  const ControlInputs & inputs, const Trajectory & trajectory,
  const PedestrianSnapshot & pedestrians, const rclcpp::Time & now)
{
  const auto frame = costmap_ros_->getGlobalFrameID();
  const auto path = [&](const std::vector<Pose2D> & poses) {
      nav_msgs::msg::Path message;
      message.header.frame_id = frame;
      message.header.stamp = now;
      for (const auto & pose : poses) {
        auto & point = message.poses.emplace_back();
        point.header = message.header;
        point.pose.position.x = pose.x;
        point.pose.position.y = pose.y;
        point.pose.orientation.z = std::sin(pose.theta / 2.);
        point.pose.orientation.w = std::cos(pose.theta / 2.);
      }
      return message;
    };
  auto global = global_plan_;
  global.header.stamp = now;
  global_plan_pub_->publish(global);
  local_plan_pub_->publish(path(inputs.local_path));
  trajectory_pub_->publish(path(trajectory.poses));

  using Marker = visualization_msgs::msg::Marker;
  visualization_msgs::msg::MarkerArray batch;
  auto & clear = batch.markers.emplace_back();
  clear.header.frame_id = frame;
  clear.header.stamp = now;
  clear.action = Marker::DELETEALL;
  int id = 0;
  const bool soft = pedestrian_config_ && pedestrian_config_->soft_growth_enabled;
  for (const auto & person : pedestrians.people) {
    for (const double fraction : {0., 1. / 3., 2. / 3., 1.}) {
      const double t = fraction * person.horizon;
      for (const bool hard : {true, false}) {
        if (!soft && !hard) {continue;}
        const double radius = hard && soft ? person.physical_radius : person.radiusAt(t);
        auto & ring = batch.markers.emplace_back();
        ring.header.frame_id = frame;
        ring.header.stamp = now;
        ring.ns = hard ? "hard_occupancy" : "soft_envelope";
        ring.id = id++;
        ring.type = Marker::LINE_STRIP;
        ring.action = Marker::ADD;
        ring.pose.orientation.w = 1.;
        ring.scale.x = hard ? .024 : .014;
        ring.color.r = 1.;
        ring.color.g = hard ? .25 : .72;
        ring.color.b = .12;
        ring.color.a = fraction == 0. ? .98 : .62;
        ring.lifetime = rclcpp::Duration::from_seconds(.4);
        for (int i = 0; i <= 48; ++i) {
          const double angle = 6.283185307179586 * i / 48;
          auto & point = ring.points.emplace_back();
          point.x = person.x + person.vx * t + radius * std::cos(angle);
          point.y = person.y + person.vy * t + radius * std::sin(angle);
          point.z = hard ? .08 : .065;
        }
      }
    }
    if (std::hypot(person.vx, person.vy) <= .001) {continue;}
    auto & arrow = batch.markers.emplace_back();
    arrow.header.frame_id = frame;
    arrow.header.stamp = now;
    arrow.ns = "predicted_motion";
    arrow.id = id++;
    arrow.type = Marker::ARROW;
    arrow.action = Marker::ADD;
    arrow.pose.orientation.w = 1.;
    arrow.scale.x = .027;
    arrow.scale.y = .07;
    arrow.scale.z = .09;
    arrow.color.r = 1.;
    arrow.color.g = .83;
    arrow.color.b = .27;
    arrow.color.a = 1.;
    arrow.lifetime = rclcpp::Duration::from_seconds(.4);
    auto & start = arrow.points.emplace_back();
    start.x = person.x;
    start.y = person.y;
    start.z = .10;
    auto & end = arrow.points.emplace_back();
    end.x = person.x + person.vx * person.horizon;
    end.y = person.y + person.vy * person.horizon;
    end.z = .10;
  }
  prediction_pub_->publish(batch);
}

Velocity2D MyDWAController::selectBestCandidate(
  const ControlInputs & inputs, const GoalControlContext & goal,
  const std::vector<Velocity2D> & commands, const PedestrianSnapshot & pedestrians,
  const rclcpp::Time & now, bool post_loss_active)
{
  struct Candidate
  {
    Trajectory nominal, backup;
    double obstacle_cost, stop_time;
  };
  std::vector<Candidate> feasible;
  const auto fallback = predictor_->predict(inputs.initial, inputs.current, {}, 0);
  bool fallback_static_clear;
  std::size_t horizon_rejected = 0, static_rejected = 0, nominal_rejected = 0;
  const auto node = node_.lock();
  auto * map = costmap_ros_->getCostmap();
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> map_lock(*map->getMutex());
    if (rcutils_logging_logger_is_enabled_for(
        node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG))
    {
      // Capture exactly the locked bytes used for this cycle's static checks.
      constexpr char hex[] = "0123456789abcdef";
      const std::size_t cells = static_cast<std::size_t>(map->getSizeInCellsX()) *
        map->getSizeInCellsY();
      std::string bytes;
      bytes.reserve(2 * cells);
      for (std::size_t i = 0; i < cells; ++i) {
        const auto cost = map->getCharMap()[i];
        bytes.push_back(hex[cost >> 4]);
        bytes.push_back(hex[cost & 15]);
      }
      std::ostringstream snapshot;
      snapshot << std::setprecision(17) << "{\"cycle\":" << diagnostic_cycle_
               << ",\"origin\":[" << map->getOriginX() << ',' << map->getOriginY()
               << "],\"resolution\":" << map->getResolution()
               << ",\"width\":" << map->getSizeInCellsX()
               << ",\"height\":" << map->getSizeInCellsY()
               << ",\"data_hex\":" << std::quoted(bytes) << '}';
      RCLCPP_DEBUG(node->get_logger(), "Costmap snapshot: %s", snapshot.str().c_str());
    }
    fallback_static_clear = evaluateTrajectory(fallback.trajectory, *map).collision_free;
    for (const auto & command : commands) {
      if (!isGoalCandidateAllowed(command, inputs, goal)) {continue;}
      auto backup = predictor_->predict(
        inputs.initial, inputs.current, command, sampling_config_.control_dt);
      if (backup.stop_time > sim_time_) {++horizon_rejected; continue;}
      if (!evaluateTrajectory(backup.trajectory, *map).collision_free) {
        ++static_rejected; continue;
      }
      auto nominal = predictor_->predict(inputs.initial, inputs.current, command, sim_time_);
      if (!evaluateTrajectory(nominal.trajectory, *map).collision_free) {
        ++nominal_rejected; continue;
      }
      const double cost = trajectoryObstacleCost(nominal.trajectory, *map);
      feasible.push_back(
        {std::move(nominal.trajectory), std::move(
            backup.trajectory), cost, backup.stop_time});
    }
  }

  const auto scan = scan_tracker_ ? scan_tracker_->snapshot() : nullptr;
  updateDynamicMode(scan.get(), inputs, now);
  const auto footprint = pedestrians.people.empty() ?
    std::vector<geometry_msgs::msg::Point>{} : costmap_ros_->getUnpaddedRobotFootprint();
  const bool soften_growth = pedestrian_config_ && pedestrian_config_->soft_growth_enabled;
  const double soft_weight = soften_growth ? pedestrian_config_->soft_growth_weight : 0.;
  const auto dynamic_check = [&](const Trajectory & trajectory, double horizon) {
      return pedestrianCollision(
        trajectory, predictor_->step(), horizon, footprint, pedestrians.people, soften_growth);
    };
  PedestrianCollisionResult dynamic_result;
  std::size_t dynamic_rejected = 0, soft_penalized = 0;
  double selected_soft_penalty = 0.;
  std::optional<std::size_t> best;
  double best_score = std::numeric_limits<double>::infinity();
  std::size_t accepted = 0;
  for (std::size_t i = 0; i < feasible.size(); ++i) {
    const auto & c = feasible[i];
    const auto nominal = dynamic_check(c.nominal, sim_time_);
    const auto backup = dynamic_check(c.backup, std::min(sim_time_, c.stop_time));
    for (const auto * result : {&nominal, &backup}) {
      dynamic_result.min_clearance = std::min(
        dynamic_result.min_clearance, result->min_clearance);
      if (result->first_time < dynamic_result.first_time) {
        dynamic_result.first_time = result->first_time;
        dynamic_result.id = result->id;
      }
    }
    if (nominal.collision || backup.collision) {
      ++dynamic_rejected;
      continue;
    }
    ++accepted;
    const double soft_penalty = soft_weight * std::max(nominal.soft_cost, backup.soft_cost);
    soft_penalized += soft_penalty > 0.;
    const double score = scoreCandidate(
      c.nominal, c.obstacle_cost, inputs, goal).total_cost + soft_penalty;
    const auto & cmd = c.nominal.command;
    bool prefer = !best;
    if (best) {
      const auto & old = feasible[*best].nominal.command;
      prefer = goal.phase == GoalPhase::Slowing && cmd.v != old.v ? cmd.v < old.v :
        score < best_score || (score == best_score && std::abs(cmd.omega) < std::abs(old.omega));
    }
    if (prefer) {
      best = i;
      best_score = score;
      selected_soft_penalty = soft_penalty;
    }
  }
  const bool fallback_complete = fallback.stop_time <= sim_time_;
  const bool dynamic_blocked = !feasible.empty() && !best && dynamic_rejected == feasible.size();
  all_dynamic_rejected_cycles_ += dynamic_blocked;
  const bool fallback_dynamic_collision = !best && dynamic_check(
    fallback.trajectory, std::min(sim_time_, fallback.stop_time)).collision;
  const bool emergency = !best &&
    (!fallback_static_clear || !fallback_complete || fallback_dynamic_collision);
  const auto dwa_candidate = best ? feasible[*best].nominal.command : Velocity2D{0, 0};
  auto output_command = dwa_candidate;
  if (crossing_yield_ && dynamic_active_) {
    auto y = crossing_yield_->update(
      scan.get(), inputs.local_path, inputs.initial, inputs.current, fallback.trajectory,
      now.nanoseconds(), best.has_value());
    if (y.triggered || y.released) {quasi_static_.reset();}
    if (y.hold && best && fallback_complete && fallback_static_clear) {
      const auto & selected = feasible[*best];
      const MotionRollout backup{selected.backup, selected.stop_time};
      y.hold = !crossing_yield_->canApproach(
        inputs.local_path, fallback, backup, sim_time_, predictor_->step(),
        std::max(std::abs(inputs.current.v), std::abs(dwa_candidate.v)));
    }
    y.hold = crossing_yield_->latchHold(y.hold);
    if (y.hold) {
      output_command = Velocity2D{0, 0};
    }
    RCLCPP_INFO(
      node->get_logger(), "Crossing yield: {\"now_ns\":%ld,\"scan_ns\":%ld,\"hold\":%s,\"triggered\":%s,\"released\":%s,\"occupied\":%s,\"clear\":%s,\"safe_stop\":%s,\"margin\":%.9f,\"entry\":%.9f,\"exit\":%.9f,\"rank\":%d,\"clear_scans\":%d}",
      now.nanoseconds(), scan ? rclcpp::Time(
        scan->stamp).nanoseconds() : int64_t{0}, y.hold ? "true" : "false", y.triggered ? "true" : "false", y.released ? "true" : "false", y.occupied ? "true" : "false", y.clear ? "true" : "false", y.safe_stop ? "true" : "false", y.margin, y.entry, y.exit, y.evidence_rank,
      y.clear_scans);
  }
  std::ostringstream out;
  out << std::setprecision(17) << std::boolalpha
      << "{\"schema_version\":2,\"cycle\":" << diagnostic_cycle_
      << ",\"now_ns\":" << now.nanoseconds()
      << ",\"scan_ns\":" << (scan ? rclcpp::Time(scan->stamp).nanoseconds() : 0)
      << ",\"global_dynamic_active\":" << dynamic_active_
      << ",\"sampled\":" << commands.size() << ",\"horizon_rejected\":" << horizon_rejected
      << ",\"nominal_rejected\":" << nominal_rejected
      << ",\"static_rejected\":" << static_rejected << ",\"static_feasible\":" << feasible.size()
      << ",\"accepted\":" << accepted
      << ",\"mode\":" << std::quoted(best ? "navigate" : emergency ? "emergency" : "brake")
      << ",\"candidate\":[" << dwa_candidate.v << ',' << dwa_candidate.omega << ']'
      << ",\"output\":[" << output_command.v << ',' << output_command.omega << ']'
    // Compatibility: selected has always contained the final, overridden output.
      << ",\"selected\":[" << output_command.v << ',' << output_command.omega << ']'
      << ",\"measured\":[" << inputs.current.v << ',' << inputs.current.omega << ']'
      << ",\"initial\":[" << inputs.initial.x << ',' << inputs.initial.y << ',' <<
    inputs.initial.theta << ']'
      << ",\"selected_stop_time\":" << (best ? feasible[*best].stop_time : fallback.stop_time)
      << ",\"fallback_static_clear\":" << fallback_static_clear
      << ",\"fallback_complete\":" << fallback_complete
      << ",\"clusters\":[";
  if (scan) {
    for (std::size_t i = 0; i < scan->obstacles.size(); ++i) {
      const auto & o = scan->obstacles[i];
      if (i) {out << ',';}
      out << '[' << o.velocity_valid << ',' << o.vx << ',' << o.vy << ','
          << o.points.size() << ']';
    }
  }
  out << ']';
  if (pedestrian_input_) {
    std::array<std::size_t, 5> sources{};
    for (const auto & person : pedestrians.people) {
      ++sources[static_cast<std::size_t>(person.source)];
    }
    const auto number = [&out](double value) {
        if (std::isfinite(value)) {out << value;} else {out << "null";}
      };
    out << ",\"post_loss_cap_enabled\":" << post_loss_cap_enabled_
        << ",\"post_loss_cap_active\":" << post_loss_active
        << ",\"sample_max_v_mps\":" << goal.sampling.max_v
        << ",\"pedestrian_received\":" << pedestrians.received
        << ",\"pedestrian_position_valid\":" << pedestrians.valid_position
        << ",\"pedestrian_velocity_valid\":" << pedestrians.valid_velocity
        << ",\"pedestrian_input_rejected\":" << pedestrians.rejected
        << ",\"prediction_sources\":{\"unknown\":" << sources[0]
        << ",\"ols\":" << sources[1] << ",\"difference\":" << sources[2]
        << ",\"held\":" << sources[3] << ",\"kf\":" << sources[4] << '}'
        << ",\"soft_growth_enabled\":" << soften_growth
        << ",\"soft_penalized_candidates\":" << soft_penalized
        << ",\"selected_soft_penalty\":" << selected_soft_penalty
        << ",\"dynamic_rejected\":" << dynamic_rejected
        << ",\"dynamic_blocked\":" << dynamic_blocked
        << ",\"all_dynamic_rejected_cycles\":" << all_dynamic_rejected_cycles_
        << ",\"fallback_dynamic_collision\":" << fallback_dynamic_collision
        << ",\"min_dynamic_clearance\":";
    number(dynamic_result.min_clearance);
    out << ",\"first_dynamic_collision_s\":";
    number(dynamic_result.first_time);
    out << ",\"collision_track\":" << std::quoted(dynamic_result.id)
        << ",\"pedestrians\":[";
    bool first = true;
    for (const auto & person : pedestrians.people) {
      if (!first) {out << ',';}
      first = false;
      out << "{\"id\":" << std::quoted(person.id) << ",\"epoch\":" << person.epoch
          << ",\"position_age_s\":" << person.age
          << ",\"velocity_age_s\":" << person.velocity_age
          << ",\"source\":" << std::quoted(velocitySourceName(person.source))
          << ",\"x\":" << person.x << ",\"y\":" << person.y
          << ",\"vx\":" << person.vx << ",\"vy\":" << person.vy
          << ",\"physical_radius_m\":" << person.physical_radius
          << ",\"base_radius_m\":" << person.radius
          << ",\"growth_mps\":" << person.growth
          << ",\"max_growth_m\":" << person.max_growth
          << ",\"horizon_s\":" << person.horizon << '}';
    }
    out << ']';
  }
  out << '}';
  if (prediction_pub_) {
    const bool selected = best && output_command.v == dwa_candidate.v &&
      output_command.omega == dwa_candidate.omega;
    publishVisualization(
      inputs, selected ? feasible[*best].nominal : fallback.trajectory, pedestrians, now);
  }
  RCLCPP_INFO(node->get_logger(), "Stoppable decision: %s", out.str().c_str());
  return output_command;
}


geometry_msgs::msg::TwistStamped MyDWAController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const auto invocation_start = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> state_lock(state_mutex_);
  const auto locked_at = std::chrono::steady_clock::now();
  ++diagnostic_cycle_;
  auto node = node_.lock();
  if (!active_ || !node || !costmap_ros_) {
    throw std::logic_error("controller is not active and configured");
  }

  const auto report_timing = [&](bool completed) {
      const auto finished_at = std::chrono::steady_clock::now();
      RCLCPP_INFO_EXPRESSION(
        node->get_logger(), timing_enabled_ || rcutils_logging_logger_is_enabled_for(
          node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG),
        "Control timing: {\"cycle\":%lu,\"completed\":%s,\"start_steady_ns\":%ld,\"end_steady_ns\":%ld,\"compute_us\":%.3f,\"lock_wait_us\":%.3f}",
        diagnostic_cycle_, completed ? "true" : "false",
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          invocation_start.time_since_epoch()).count(),
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          finished_at.time_since_epoch()).count(),
        std::chrono::duration<double, std::micro>(finished_at - invocation_start).count(),
        std::chrono::duration<double, std::micro>(locked_at - invocation_start).count());
    };
  try {
    ControlInputs inputs;
    inputs.terminal_radius = terminal_xy_tolerance_;
    inputs.yaw_tolerance = terminal_yaw_tolerance_;
    GoalControlContext goal;
    prepareControlInputs(pose, velocity, goal_checker, inputs);
    prepareGoalControl(inputs, goal);

    // The first (unconstrained) sampling inside prepareGoalControl must stay before
    // phase updates; this second sampling uses the resulting reachable limits.
    const auto now = node->now();
    // Both arms use this same immutable snapshot for the speed window and collision scoring.
    const auto pedestrians = pedestrian_input_ ?
      pedestrian_input_->snapshot(now.nanoseconds()) : PedestrianSnapshot{};
    const bool has_close_prediction = std::any_of(
      pedestrians.people.begin(), pedestrians.people.end(), [&](const auto & person) {
        return std::hypot(person.x - inputs.initial.x, person.y - inputs.initial.y) <
        PostLossCap::close_distance;
      });
    const bool post_loss_active = post_loss_cap_enabled_ &&
      post_loss_cap_.update(now.nanoseconds(), has_close_prediction);
    if (post_loss_active) {
      PostLossCap::limit(goal.sampling, inputs.current.v);
    }
    const auto candidates = sampleVelocities(inputs.current, goal.sampling);
    const auto best = selectBestCandidate(
      inputs, goal, candidates, pedestrians, now,
      post_loss_active);

    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = node->now();
    command.header.frame_id = costmap_ros_->getBaseFrameID();
    command.twist.linear.x = best.v;
    command.twist.angular.z = best.omega;
    report_timing(true);
    return command;
  } catch (...) {
    report_timing(false);
    throw;
  }
}

}  // 命名空间 my_dwa_controller

PLUGINLIB_EXPORT_CLASS(my_dwa_controller::MyDWAController, nav2_core::Controller)
