#pragma once

#include <memory>
#include <limits>
#include <vector>
#include <mutex>
#include <string>

#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/path.hpp"
#include "my_dwa_controller/path_handler.hpp"
#include "my_dwa_controller/controller_diagnostics.hpp"
#include "my_dwa_controller/trajectory_predictor.hpp"
#include "my_dwa_controller/trajectory_scorer.hpp"
#include "my_dwa_controller/velocity_sampler.hpp"
#include "my_dwa_controller/path_conflict_monitor.hpp"
#include "my_dwa_controller/risk_governor.hpp"

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

private:
  enum class GoalPhase {Normal, Slowing, Rotating};

  struct NavigationSnapshot
  {
    std::vector<Pose2D> local_path;
    Pose2D robot_pose{}, final_goal_pose{};
    Velocity2D robot_velocity{};
    double distance_to_goal = std::numeric_limits<double>::quiet_NaN();
    double xy_tolerance_radius = 0.0, yaw_tolerance = 0.0;
  };

  struct CycleControlPolicy
  {
    GoalPhase phase = GoalPhase::Normal;
    SamplingConfig sampling;
    std::optional<double> preferred_max_linear_speed;
    double yaw_error = 0.0, linear_speed_cap = 0.0, angular_speed_cap = 0.0;
    bool use_heading_score = false, needs_angular_braking = false;
  };

  struct CandidateScore
  {
    ScoreResult costs{};
    double rotation_cost = 0.0;
  };

  // All helpers run under the caller's state_mutex_; none re-lock it.
  // Output contexts retain partial progress for exception diagnostics.
  void prepareNavigationSnapshot(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker, NavigationSnapshot & snapshot,
    CycleDiagnostics & cycle) const;
  // May advance the persistent goal_phase_ state machine.
  void prepareControlPolicy(const NavigationSnapshot & snapshot, CycleControlPolicy & policy);
  nav2_costmap_2d::Costmap2D snapshotCostmap(CycleDiagnostics & cycle) const;
  void applyRiskPolicy(
    const NavigationSnapshot & snapshot, const nav2_costmap_2d::Costmap2D & costmap,
    double observation_time, CycleControlPolicy & policy, CycleDiagnostics & cycle);
  bool satisfiesGoalConstraints(
    const Velocity2D & candidate, const NavigationSnapshot & snapshot,
    const CycleControlPolicy & policy) const;
  std::vector<Velocity2D> sampleControlCandidates(
    const NavigationSnapshot & snapshot, const CycleControlPolicy & policy) const;
  CandidateScore scoreCandidate(
    const Trajectory & trajectory, const ObstacleEvaluation & obstacle,
    const NavigationSnapshot & snapshot, const CycleControlPolicy & policy) const;
  Velocity2D selectBestCandidate(
    const NavigationSnapshot & snapshot, const CycleControlPolicy & policy,
    const std::vector<Velocity2D> & candidates, const nav2_costmap_2d::Costmap2D & costmap,
    CycleDiagnostics & cycle) const;

  // 状态锁串行保护路径、配置、限速和生命周期；不与地图锁反向嵌套。
  std::mutex state_mutex_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string plugin_name_;

  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  SamplingConfig active_sampling_config_;
  double configured_max_linear_speed_ = 0.0;
  ScoreWeights score_weights_;
  double goal_ref_distance_ = 1.0;
  double path_ref_distance_ = 0.5;
  double forward_length_ = 2.0;
  double follow_lookahead_ = 0.6;
  double heading_lookahead_ = 0.6;
  std::unique_ptr<PathHandler> path_handler_;
  std::unique_ptr<TrajectoryPredictor> predictor_;
  std::unique_ptr<TrajectoryScorer> scorer_;
  std::unique_ptr<PathConflictMonitor> conflict_monitor_;
  std::unique_ptr<RiskGovernor> risk_governor_;
  std::unique_ptr<ScanObstacleTracker> scan_tracker_;
  RiskState last_risk_state_ = RiskState::Normal;
  bool active_ = false;

  GoalPhase goal_phase_ = GoalPhase::Normal;
  double trans_stopped_velocity_ = 0.02;
  double fallback_xy_tolerance_ = 0.25;  // GoalChecker fallback only.
  double fallback_yaw_tolerance_ = 0.05;  // GoalChecker fallback only.
  double sim_time_ = 1.7;

  // Observer only; protected by the same Controller state lock.
  ControllerDiagnostics diagnostics_;

};

}  // 命名空间 my_dwa_controller
