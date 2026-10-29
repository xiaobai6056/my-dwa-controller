#pragma once

#include <memory>
#include <chrono>
#include <cstdint>
#include <limits>
#include <vector>
#include <mutex>
#include <string>

#include "nav2_core/controller.hpp"
#include "rclcpp/subscription.hpp"
#include "nav_msgs/msg/path.hpp"
#include "my_dwa_controller/dynamic_filter.hpp"
#include "my_dwa_controller/motion_predictor.hpp"
#include "my_dwa_controller/path_handler.hpp"
#include "my_dwa_controller/scan_obstacle_tracker.hpp"
#include "my_dwa_controller/obstacle_evaluator.hpp"
#include "my_dwa_controller/trajectory_predictor.hpp"
#include "my_dwa_controller/trajectory_scorer.hpp"
#include "my_dwa_controller/velocity_sampler.hpp"
#include "my_dwa_controller/crossing_yield.hpp"

namespace my_dwa_controller
{

class MyDWAController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void activate() override;

  void deactivate() override;

  void cleanup() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

  // Immutable latest scan; null means unavailable, not a confirmed empty region.
  std::shared_ptr<const ScanSnapshot> getScanSnapshot();

private:
  enum class GoalPhase {Normal, Slowing, Rotating};

  struct ControlInputs
  {
    std::vector<Pose2D> local_path;
    Pose2D initial{}, true_goal{};
    Velocity2D current{};
    double distance_to_goal = std::numeric_limits<double>::quiet_NaN();
    double terminal_radius = 0.0, yaw_tolerance = 0.0;
  };

  struct GoalControlContext
  {
    GoalPhase phase = GoalPhase::Normal;
    SamplingConfig sampling;
    double yaw_error = 0.0, approach_cap = 0.0, angular_cap = 0.0;
    bool heading_enabled = false, angular_braking = false;
  };

  // All helpers run under the caller's state_mutex_; none re-lock it.
  void prepareControlInputs(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker, ControlInputs & inputs) const;
  void prepareGoalControl(const ControlInputs & inputs, GoalControlContext & goal);
  bool isGoalCandidateAllowed(
    const Velocity2D & candidate, const ControlInputs & inputs,
    const GoalControlContext & goal) const;
  ScoreResult scoreCandidate(
    const Trajectory & trajectory, double obstacle_cost,
    const ControlInputs & inputs, const GoalControlContext & goal) const;
  Velocity2D selectBestCandidate(
    const ControlInputs & inputs, const GoalControlContext & goal,
    const std::vector<Velocity2D> & candidates);

  // 状态锁串行保护路径、配置、限速和生命周期；不与地图锁反向嵌套。
  std::mutex state_mutex_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  SamplingConfig sampling_config_;
  double configured_max_v_ = 0.0;
  ScoreWeights score_weights_;
  double goal_ref_distance_ = 1.0;
  double path_ref_distance_ = 0.5;
  double forward_length_ = 2.0;
  double follow_lookahead_ = 0.6;
  double heading_lookahead_ = 0.6;
  std::shared_ptr<ScanObstacleTracker> scan_tracker_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  std::string scan_topic_;
  rclcpp::TimerBase::SharedPtr scan_tf_timer_;
  std::uint64_t diagnostic_cycle_ = 0;
  std::unique_ptr<PathHandler> path_handler_;
  std::unique_ptr<MotionPredictor> predictor_;
  std::unique_ptr<DynamicFilter> dynamic_filter_;
  std::unique_ptr<CrossingYield> crossing_yield_;
  std::unique_ptr<TrajectoryScorer> scorer_;
  bool active_ = false;

  GoalPhase goal_phase_ = GoalPhase::Normal;
  double trans_stopped_velocity_ = 0.02;
  double terminal_xy_tolerance_ = 0.25;  // GoalChecker fallback only.
  double terminal_yaw_tolerance_ = 0.05;  // GoalChecker fallback only.
  double sim_time_ = 1.7;
};

}  // 命名空间 my_dwa_controller
