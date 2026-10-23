#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <iostream>
#include <iomanip>
#include <cstdint>
#include <sstream>
#include <stdexcept>
#include <chrono>
#include <thread>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "nav2_core/controller.hpp"
#include "my_dwa_controller/obstacle_evaluator.hpp"
#include "my_dwa_controller/path_handler.hpp"
#include "my_dwa_controller/trajectory_predictor.hpp"
#include "my_dwa_controller/trajectory_scorer.hpp"
#include "my_dwa_controller/velocity_sampler.hpp"

namespace my_dwa_controller
{
namespace
{
constexpr double kTolerance = 1e-9;
const double kPi = std::acos(-1.0);

// 只装入人工地图，不运行 Costmap 的配置、更新线程或传感器插件。
class TestCostmapROS : public nav2_costmap_2d::Costmap2DROS
{
public:
  TestCostmapROS()
  : Costmap2DROS("dwa_controller_test_costmap")
  {
    global_frame_ = "controller_local";
    robot_base_frame_ = "controller_base";
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(
      global_frame_, false, false);
    layered_costmap_->resizeMap(120, 120, 0.05, -3.0, -3.0);
    std::vector<geometry_msgs::msg::Point> footprint(4);
    footprint[0].x = 0.22;
    footprint[1].y = 0.22;
    footprint[2].x = -0.22;
    footprint[3].y = -0.22;
    layered_costmap_->setFootprint(footprint);
  }
};

geometry_msgs::msg::PoseStamped makePose(
  double x, double y, double yaw = 0.0, const std::string & frame = "controller_local")
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp.sec = 15;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}

nav_msgs::msg::Path straightPath()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "controller_local";
  path.poses = {makePose(0.0, 0.0), makePose(2.0, 0.0)};
  return path;
}

// Exercise unsupported/invalid tolerance fields without touching Nav2 state.
class UnsupportedGoalChecker : public nav2_core::GoalChecker
{
public:
  bool available = false;
  geometry_msgs::msg::Pose tolerance;
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(
    const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override
  {
    throw std::logic_error("Controller must not call GoalChecker success");
  }
  bool getTolerances(geometry_msgs::msg::Pose & pose, geometry_msgs::msg::Twist &) override
  {
    pose = tolerance;
    return available;
  }
};

class ControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides(
        {
          {"controller_frequency", 2.0}, {"FollowPath.max_vel_x", 0.4},
          {"FollowPath.max_vel_theta", 1.0}, {"FollowPath.acc_lim_x", 1.0},
          {"FollowPath.acc_lim_theta", 2.0}, {"FollowPath.vx_samples", 3},
          {"FollowPath.vtheta_samples", 3}, {"FollowPath.sim_time", 1.0},
          {"FollowPath.simulation_dt", 0.5}, {"FollowPath.forward_length", 1.0},
          {"FollowPath.obstacle_weight", 0.5}, {"FollowPath.speed_weight", 0.1},
          {"FollowPath.goal_weight", 0.2}, {"FollowPath.path_weight", 0.2},
          {"FollowPath.goal_ref_distance", 1.0}, {"FollowPath.path_ref_distance", 0.5},
          {"FollowPath.follow_lookahead", 1.0}, {"FollowPath.heading_weight", 0.0}});
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("dwa_controller_test", options);
    tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    costmap_ = std::make_shared<TestCostmapROS>();
    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_core::Controller>>(
      "nav2_core", "nav2_core::Controller");
    controller_ = loader_->createSharedInstance("my_dwa_controller::MyDWAController");
  }

  void configure()
  {
    controller_->configure(node_, "FollowPath", tf_, costmap_);
    controller_->setPlan(straightPath());
    controller_->activate();
  }

  geometry_msgs::msg::TwistStamped compute()
  {
    return controller_->computeVelocityCommands(makePose(0.0, 0.0), stopped_, nullptr);
  }

  void terminalConfigure()
  {
    ASSERT_TRUE(node_->set_parameter({"controller_frequency", 20.0}).successful);
    configure();
  }

  void setGoal(double x, double y, double yaw)
  {
    auto path = straightPath();
    path.poses.back() = makePose(x, y, yaw);
    controller_->setPlan(path);
  }

  void setWall(unsigned char cost)
  {
    auto * map = costmap_->getCostmap();
    for (unsigned int x = 0; x < map->getSizeInCellsX(); ++x) {
      for (unsigned int y = 0; y < map->getSizeInCellsY(); ++y) {
        double wx, wy;
        map->mapToWorld(x, y, wx, wy);
        if (wx > 0.3) {
          map->setCost(x, y, cost);
        }
      }
    }
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<TestCostmapROS> costmap_;
  geometry_msgs::msg::Twist stopped_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_core::Controller>> loader_;
  nav2_core::Controller::Ptr controller_;
};

TEST_F(ControllerTest, SelectsLowestCostFeasibleCandidateAndFillsHeader)
{
  configure();
  const auto before = node_->now();
  const auto command = compute();
  const auto after = node_->now();
  EXPECT_NEAR(command.twist.linear.x, 0.4, kTolerance);
  EXPECT_NEAR(command.twist.angular.z, 0.0, kTolerance);
  EXPECT_EQ(command.header.frame_id, "controller_base");
  EXPECT_GE(rclcpp::Time(command.header.stamp).nanoseconds(), before.nanoseconds());
  EXPECT_LE(rclcpp::Time(command.header.stamp).nanoseconds(), after.nanoseconds());
  EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.linear.z, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.angular.y, 0.0);

  SamplingConfig config;
  config.max_v = 0.4;
  config.max_w = 1.0;
  config.acc_v = 1.0;
  config.acc_w = 2.0;
  config.control_dt = 0.5;
  config.v_samples = 3;
  config.w_samples = 3;
  const TrajectoryPredictor predictor(1.0, 0.5);
  const TrajectoryScorer scorer(0.4, {0.5, 0.1, 0.2, 0.2, 0.0}, 1.0, 0.5, 1.0);
  const auto local_path = PathHandler(tf_).getLocalPath(
    straightPath(), makePose(0.0, 0.0), "controller_local", 1.0);
  const Pose2D initial{};
  const auto chosen = predictor.predict(initial, {command.twist.linear.x, command.twist.angular.z});
  const auto evaluated = evaluateTrajectory(chosen, *costmap_->getCostmap());
  ASSERT_TRUE(evaluated.collision_free);
  const auto chosen_score = scorer.score(chosen, local_path, evaluated.mean_cost);
  // 手算直线最高速的总分为 0.2 * (1.0 - 0.4) = 0.12。
  EXPECT_NEAR(chosen_score.total_cost, 0.12, kTolerance);
  for (const auto & candidate : sampleVelocities({0.0, 0.0}, config)) {
    const auto trajectory = predictor.predict(initial, candidate);
    const auto obstacle = evaluateTrajectory(trajectory, *costmap_->getCostmap());
    if (obstacle.collision_free) {
      EXPECT_LE(
        chosen_score.total_cost,
        scorer.score(trajectory, local_path, obstacle.mean_cost).total_cost + kTolerance);
    }
  }
}

TEST_F(ControllerTest, FiltersCollisionsAndChoosesSlowerSafeTrajectory)
{
  configure();
  setWall(nav2_costmap_2d::LETHAL_OBSTACLE);
  const auto command = compute();
  EXPECT_NEAR(command.twist.linear.x, 0.2, kTolerance);
  EXPECT_NEAR(command.twist.angular.z, 0.0, kTolerance);
}

TEST_F(ControllerTest, UsesMeanObstacleCostWhenBothTrajectoriesAreFeasible)
{
  configure();
  setWall(200);
  const auto command = compute();
  EXPECT_NEAR(command.twist.linear.x, 0.2, kTolerance);
  EXPECT_NEAR(command.twist.angular.z, 0.0, kTolerance);
}

TEST_F(ControllerTest, ReportsNoFeasibleTrajectory)
{
  configure();
  auto * map = costmap_->getCostmap();
  std::fill_n(
    map->getCharMap(), map->getSizeInCellsX() * map->getSizeInCellsY(),
    nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(compute(), nav2_core::PlannerException);
}

TEST_F(ControllerTest, ReportsEmptyDynamicWindowWithoutClampingCurrentSpeed)
{
  configure();
  geometry_msgs::msg::Twist speed;
  speed.linear.x = 2.0;
  EXPECT_THROW(
    controller_->computeVelocityCommands(makePose(0.0, 0.0), speed, nullptr),
    nav2_core::PlannerException);
}

TEST_F(ControllerTest, UsesControllerFrequencyAndBothCurrentVelocityComponents)
{
  ASSERT_TRUE(node_->set_parameter({"controller_frequency", 20.0}).successful);
  configure();
  EXPECT_NEAR(compute().twist.linear.x, 0.05, kTolerance);
  geometry_msgs::msg::Twist current;
  current.linear.x = 0.2;
  current.angular.z = 0.5;
  const auto command = controller_->computeVelocityCommands(makePose(0.0, 0.0), current, nullptr);
  EXPECT_NEAR(command.twist.linear.x, 0.25, kTolerance);
  EXPECT_GE(command.twist.angular.z, 0.4 - kTolerance);
  EXPECT_LE(command.twist.angular.z, 0.6 + kTolerance);
}

TEST_F(ControllerTest, LoadsActualNav2YamlParameters)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "--params-file", MY_DWA_TEST_PARAMS_FILE});
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("controller_server", options);
  EXPECT_DOUBLE_EQ(node->get_parameter("FollowPath.follow_lookahead").as_double(), 0.6);
  EXPECT_DOUBLE_EQ(node->get_parameter("FollowPath.heading_lookahead").as_double(), 0.6);
  EXPECT_DOUBLE_EQ(node->get_parameter("FollowPath.heading_weight").as_double(), 0.05);
  EXPECT_DOUBLE_EQ(node->get_parameter("FollowPath.trans_stopped_velocity").as_double(), 0.02);
  EXPECT_DOUBLE_EQ(node->get_parameter("FollowPath.terminal_xy_tolerance").as_double(), 0.25);
  EXPECT_DOUBLE_EQ(node->get_parameter("FollowPath.terminal_yaw_tolerance").as_double(), 0.05);
  controller_->configure(node, "FollowPath", tf_, costmap_);
  controller_->setPlan(straightPath());
  controller_->activate();
  const auto command = compute();
  // YAML 的 acc_lim_x=2.5、controller_frequency=20，从静止可达上限为 0.125。
  EXPECT_NEAR(command.twist.linear.x, 0.125, kTolerance);
  EXPECT_NEAR(command.twist.angular.z, 0.0, kTolerance);
}

TEST_F(ControllerTest, RejectsMissingAndEmptyGlobalPlan)
{
  controller_->configure(node_, "FollowPath", tf_, costmap_);
  controller_->activate();
  EXPECT_THROW(compute(), std::invalid_argument);
  EXPECT_THROW(controller_->setPlan(nav_msgs::msg::Path{}), std::invalid_argument);
}

TEST_F(ControllerTest, RejectsRobotFrameMismatchAndInvalidPose)
{
  configure();
  EXPECT_THROW(
    controller_->computeVelocityCommands(makePose(0.0, 0.0, 0.0, "map"), stopped_, nullptr),
    std::invalid_argument);
  auto pose = makePose(0.0, 0.0);
  pose.pose.orientation.w = 0.0;
  EXPECT_THROW(
    controller_->computeVelocityCommands(pose, stopped_, nullptr),
    std::invalid_argument);
  pose = makePose(0.0, 0.0);
  pose.pose.position.x = std::nan("");
  EXPECT_THROW(
    controller_->computeVelocityCommands(pose, stopped_, nullptr),
    std::invalid_argument);
}

TEST_F(ControllerTest, PropagatesInvalidVelocityAndPathContractFailures)
{
  configure();
  geometry_msgs::msg::Twist speed;
  speed.angular.z = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
    controller_->computeVelocityCommands(makePose(0.0, 0.0), speed, nullptr),
    std::invalid_argument);
  auto path = straightPath();
  path.poses.back().header.frame_id = "wrong_frame";
  EXPECT_THROW(controller_->setPlan(path), std::invalid_argument);
  EXPECT_GT(compute().twist.linear.x, 0.0);
}

TEST_F(ControllerTest, PropagatesUnavailableTf)
{
  configure();
  auto path = straightPath();
  path.header.frame_id = "map";
  for (auto & pose : path.poses) {
    pose.header.frame_id = "map";
  }
  controller_->setPlan(path);
  EXPECT_THROW(compute(), tf2::TransformException);
}

TEST_F(ControllerTest, PredictsInLocalFrameWhileCommandsRemainBodyVelocities)
{
  configure();
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "world";
  transform.child_frame_id = "controller_local";
  transform.transform.translation.x = 10.0;
  transform.transform.translation.y = 20.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, kPi / 2.0);
  transform.transform.rotation = tf2::toMsg(q);
  ASSERT_TRUE(tf_->setTransform(transform, "controller_test", true));
  auto path = straightPath();
  path.header.frame_id = "world";
  path.poses = {makePose(10.0, 20.0, kPi, "world"), makePose(8.0, 20.0, kPi, "world")};
  controller_->setPlan(path);
  const auto command = controller_->computeVelocityCommands(
    makePose(
      0.0, 0.0,
      kPi / 2.0), stopped_, nullptr);
  EXPECT_NEAR(command.twist.linear.x, 0.4, kTolerance);
  EXPECT_NEAR(command.twist.angular.z, 0.0, kTolerance);
  EXPECT_EQ(command.header.frame_id, "controller_base");
}

TEST_F(ControllerTest, SetPlanReplacesReferenceUsedByNextCycle)
{
  configure();
  EXPECT_NEAR(compute().twist.linear.x, 0.4, kTolerance);
  auto path = straightPath();
  path.poses.resize(1);
  controller_->setPlan(path);
  EXPECT_DOUBLE_EQ(compute().twist.linear.x, 0.0);
}

TEST_F(ControllerTest, AppliesPercentageAndAbsoluteSpeedLimitsAndRestoresConfiguredLimit)
{
  configure();
  controller_->setSpeedLimit(50.0, true);
  EXPECT_NEAR(compute().twist.linear.x, 0.2, kTolerance);
  controller_->setSpeedLimit(0.1, false);
  EXPECT_NEAR(compute().twist.linear.x, 0.1, kTolerance);
  controller_->setSpeedLimit(0.0, false);
  EXPECT_NEAR(compute().twist.linear.x, 0.4, kTolerance);
}

TEST_F(ControllerTest, RejectsInvalidSpeedLimitsWithoutChangingConfiguration)
{
  configure();
  EXPECT_THROW(controller_->setSpeedLimit(-1.0, false), std::invalid_argument);
  EXPECT_THROW(controller_->setSpeedLimit(std::nan(""), false), std::invalid_argument);
  EXPECT_THROW(controller_->setSpeedLimit(101.0, true), std::invalid_argument);
  EXPECT_NEAR(compute().twist.linear.x, 0.4, kTolerance);
}

TEST_F(ControllerTest, LifecycleDisallowsComputingWithoutActiveConfiguration)
{
  EXPECT_THROW(compute(), std::logic_error);
  configure();
  controller_->deactivate();
  EXPECT_THROW(compute(), std::logic_error);
  controller_->activate();
  EXPECT_NO_THROW(compute());
  controller_->cleanup();
  EXPECT_THROW(compute(), std::logic_error);
  configure();
  EXPECT_NO_THROW(compute());
}

TEST_F(ControllerTest, RejectsInvalidParametersDuringConfigure)
{
  const std::vector<rclcpp::Parameter> invalid{
    {"controller_frequency", 0.0}, {"FollowPath.max_vel_x", 0.0},
    {"FollowPath.acc_lim_x", -1.0}, {"FollowPath.vx_samples", 1},
    {"FollowPath.vtheta_samples", -1}, {"FollowPath.sim_time", 0.0},
    {"FollowPath.simulation_dt", 0.0}, {"FollowPath.forward_length", 0.1},
    {"FollowPath.obstacle_weight", -1.0}, {"FollowPath.goal_ref_distance", 0.0},
    {"FollowPath.follow_lookahead", 0.0}, {"FollowPath.heading_lookahead", -0.1},
    {"FollowPath.heading_weight", -0.1}};
  for (const auto & parameter : invalid) {
    SCOPED_TRACE(parameter.get_name());
    const auto original = node_->get_parameter(parameter.get_name());
    ASSERT_TRUE(node_->set_parameter(parameter).successful);
    auto controller = loader_->createSharedInstance("my_dwa_controller::MyDWAController");
    EXPECT_THROW(controller->configure(node_, "FollowPath", tf_, costmap_), std::invalid_argument);
    EXPECT_THROW(controller->activate(), std::logic_error);
    ASSERT_TRUE(node_->set_parameter(original).successful);
  }
}

TEST_F(ControllerTest, RejectsMissingDependenciesDuringConfigure)
{
  EXPECT_THROW(
    controller_->configure(node_, "FollowPath", nullptr, costmap_),
    std::invalid_argument);
  EXPECT_THROW(controller_->configure(node_, "FollowPath", tf_, nullptr), std::invalid_argument);
  EXPECT_THROW(controller_->configure({}, "FollowPath", tf_, costmap_), std::invalid_argument);
}

TEST_F(ControllerTest, PluginExportStillLoadsNav2Controller)
{
  EXPECT_TRUE(loader_->isClassAvailable("my_dwa_controller::MyDWAController"));
  EXPECT_NE(controller_, nullptr);
}


TEST_F(ControllerTest, UsesConfiguredFollowLookahead)
{
  ASSERT_TRUE(node_->set_parameter({"FollowPath.obstacle_weight", 0.0}).successful);
  ASSERT_TRUE(node_->set_parameter({"FollowPath.speed_weight", 0.0}).successful);
  ASSERT_TRUE(node_->set_parameter({"FollowPath.path_weight", 0.0}).successful);
  ASSERT_TRUE(node_->set_parameter({"FollowPath.goal_weight", 1.0}).successful);
  ASSERT_TRUE(node_->set_parameter({"FollowPath.follow_lookahead", 0.2}).successful);
  ASSERT_TRUE(node_->set_parameter({"FollowPath.heading_weight", 0.0}).successful);
  configure();
  EXPECT_NEAR(compute().twist.linear.x, 0.2, kTolerance);
}

TEST_F(ControllerTest, UsesFinalYawAtTrueGoalButKeepsProcessHeadingAtShortLocalTail)
{
  for (const char * key : {"obstacle_weight", "speed_weight", "goal_weight", "path_weight"}) {
    ASSERT_TRUE(node_->set_parameter({std::string("FollowPath.") + key, 0.0}).successful);
  }
  ASSERT_TRUE(node_->set_parameter({"FollowPath.max_vel_x", 0.05}).successful);
  ASSERT_TRUE(node_->set_parameter({"FollowPath.forward_length", 0.1}).successful);
  ASSERT_TRUE(node_->set_parameter({"FollowPath.heading_weight", 1.0}).successful);
  configure();
  const auto far = controller_->computeVelocityCommands(
    makePose(
      0.0, 0.0,
      kPi / 2.0), stopped_, nullptr);
  EXPECT_LT(far.twist.angular.z, 0.0);
  auto near = straightPath();
  near.poses.back() = makePose(0.1, 0.0);
  controller_->setPlan(near);
  const auto stopped = controller_->computeVelocityCommands(
    makePose(0.0, 0.0, kPi / 2.0), stopped_, nullptr);
  EXPECT_DOUBLE_EQ(stopped.twist.linear.x, 0.0);
  EXPECT_LT(stopped.twist.angular.z, 0.0);
}

TEST_F(ControllerTest, UsesActiveGoalCheckerToleranceForTerminalHeadingRegion)
{
  for (const char * key : {"obstacle_weight", "speed_weight", "goal_weight", "path_weight"}) {
    ASSERT_TRUE(node_->set_parameter({std::string("FollowPath.") + key, 0.0}).successful);
  }
  ASSERT_TRUE(node_->set_parameter({"FollowPath.heading_weight", 1.0}).successful);
  configure();
  auto path = straightPath();
  path.poses.back() = makePose(0.3, 0.0);
  controller_->setPlan(path);
  pluginlib::ClassLoader<nav2_core::GoalChecker> loader("nav2_core", "nav2_core::GoalChecker");
  auto checker = loader.createSharedInstance("nav2_controller::SimpleGoalChecker");
  node_->declare_parameter("test_goal_checker.xy_goal_tolerance", 0.4);
  checker->initialize(node_, "test_goal_checker", costmap_);
  const auto command = controller_->computeVelocityCommands(
    makePose(0.0, 0.0, kPi / 2.0), stopped_, checker.get());
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_LT(command.twist.angular.z, 0.0);
  // A new navigation lifecycle clears the terminal latch before comparing fallback.
  controller_->deactivate();
  controller_->activate();
  const auto outside_fallback = controller_->computeVelocityCommands(
    makePose(0.0, 0.0, kPi / 2.0), stopped_, nullptr);
  EXPECT_LT(outside_fallback.twist.angular.z, 0.0);
}


TEST_F(ControllerTest, SpeedLimitReconstructionPreservesFollowLookahead)
{
  for (const char * key : {"obstacle_weight", "speed_weight", "path_weight"}) {
    ASSERT_TRUE(node_->set_parameter({std::string("FollowPath.") + key, 0.0}).successful);
  }
  ASSERT_TRUE(node_->set_parameter({"FollowPath.goal_weight", 1.0}).successful);
  ASSERT_TRUE(node_->set_parameter({"FollowPath.follow_lookahead", 0.1}).successful);
  configure();
  controller_->setSpeedLimit(50.0, true);
  EXPECT_NEAR(compute().twist.linear.x, 0.1, kTolerance);
}


TEST_F(ControllerTest, TerminalSlowsWithinReachableWindowInsteadOfForcingZero)
{
  terminalConfigure();
  setGoal(0.1, 0.0, 1.0);
  geometry_msgs::msg::Twist current;
  current.linear.x = 0.3;
  for (int i = 0; i < 6; ++i) {
    const auto command = controller_->computeVelocityCommands(makePose(0.0, 0.0), current, nullptr);
    EXPECT_GE(command.twist.linear.x, std::max(0.0, current.linear.x - 0.05) - kTolerance);
    EXPECT_LE(command.twist.linear.x, current.linear.x + kTolerance);
    EXPECT_NEAR(command.twist.linear.x, std::max(0.0, current.linear.x - 0.05), kTolerance);
    current = command.twist;
  }
  EXPECT_NEAR(current.linear.x, 0.0, kTolerance);
}

TEST_F(ControllerTest, TerminalWaitsForReachableZeroEvenBelowStoppedThreshold)
{
  ASSERT_TRUE(node_->set_parameter({"FollowPath.acc_lim_x", 0.1}).successful);
  terminalConfigure();
  setGoal(0.1, 0.0, 1.0);
  geometry_msgs::msg::Twist current;
  current.linear.x = 0.01;
  const auto braking = controller_->computeVelocityCommands(makePose(0.0, 0.0), current, nullptr);
  EXPECT_NEAR(braking.twist.linear.x, 0.005, kTolerance);
  current = braking.twist;
  const auto rotating = controller_->computeVelocityCommands(makePose(0.0, 0.0), current, nullptr);
  EXPECT_DOUBLE_EQ(rotating.twist.linear.x, 0.0);
  EXPECT_GT(rotating.twist.angular.z, 0.0);
}

TEST_F(ControllerTest, TerminalRotatesBothWaysFromRestAndIgnoresOrdinaryWeights)
{
  terminalConfigure();
  for (double direction : {-1.0, 1.0}) {
    setGoal(0.1, 0.0, direction);
    const auto command = compute();
    EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
    EXPECT_GT(direction * command.twist.angular.z, 0.0);
    EXPECT_LE(std::abs(command.twist.angular.z), 0.1 + kTolerance);
  }
}

TEST_F(ControllerTest, TerminalBrakesLargeAngularSpeedWithoutIllegalZeroJump)
{
  terminalConfigure();
  setGoal(0.1, 0.0, 0.02);
  geometry_msgs::msg::Twist current;
  current.angular.z = 0.8;
  const auto command = controller_->computeVelocityCommands(makePose(0.0, 0.0), current, nullptr);
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_NEAR(command.twist.angular.z, 0.7, kTolerance);
}

TEST_F(ControllerTest, TerminalClosedLoopBrakesOvershootAndSettlesWithoutPersistentOscillation)
{
  terminalConfigure();
  setGoal(0.1, 0.0, 0.02);
  geometry_msgs::msg::Twist current;
  current.angular.z = 0.8;
  double yaw = 0.0;
  double max_error = 0.0;
  for (int i = 0; i < 300; ++i) {
    const auto command = controller_->computeVelocityCommands(
      makePose(
        0.0, 0.0,
        yaw), current, nullptr);
    EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
    EXPECT_LE(std::abs(command.twist.angular.z - current.angular.z), 0.1 + kTolerance);
    yaw += command.twist.angular.z * 0.05;
    max_error = std::max(max_error, std::abs(yaw - 0.02));
    current = command.twist;
  }
  EXPECT_LT(max_error, 0.2);
  EXPECT_LE(std::abs(yaw - 0.02), 0.05 + kTolerance);
  EXPECT_DOUBLE_EQ(current.angular.z, 0.0);
}

TEST_F(ControllerTest, TerminalWrapsBothPiBoundariesAndStopsAtZeroYawError)
{
  terminalConfigure();
  setGoal(0.1, 0.0, -kPi + 0.1);
  auto command = controller_->computeVelocityCommands(
    makePose(
      0.0, 0.0,
      kPi - 0.1), stopped_, nullptr);
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_GT(command.twist.angular.z, 0.0);
  setGoal(0.1, 0.0, kPi - 0.1);
  command = controller_->computeVelocityCommands(makePose(0.0, 0.0, -kPi + 0.1), stopped_, nullptr);
  EXPECT_LT(command.twist.angular.z, 0.0);
  setGoal(0.1, 0.0, 0.0);
  command = compute();
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
}

TEST_F(ControllerTest, TerminalSameGoalReplanRechecksXyAndAllowsTranslationAfterDrift)
{
  terminalConfigure();
  setGoal(0.1, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(compute().twist.linear.x, 0.0);
  auto replan = straightPath();
  replan.poses = {makePose(-1.0, 1.0), makePose(0.1, 0.0, 1.0)};
  replan.header.stamp.sec = 999;
  controller_->setPlan(replan);
  const auto command = controller_->computeVelocityCommands(makePose(-0.3, 0.0), stopped_, nullptr);
  EXPECT_GT(command.twist.linear.x, 0.0);
}

TEST_F(ControllerTest, TerminalWithoutNewPlanKeepsRotationLatchAfterPositionDrift)
{
  terminalConfigure();
  setGoal(0.1, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(compute().twist.linear.x, 0.0);
  const auto command = controller_->computeVelocityCommands(makePose(-0.3, 0.0), stopped_, nullptr);
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_GT(command.twist.angular.z, 0.0);
}

TEST_F(ControllerTest, TerminalSameGoalReplanInsideXyContinuesRotation)
{
  terminalConfigure();
  setGoal(0.1, 0.0, 1.0);
  const auto before = compute();
  EXPECT_DOUBLE_EQ(before.twist.linear.x, 0.0);
  EXPECT_GT(before.twist.angular.z, 0.0);
  setGoal(0.1, 0.0, 1.0);
  const auto after = compute();
  EXPECT_DOUBLE_EQ(after.twist.linear.x, 0.0);
  EXPECT_GT(after.twist.angular.z, 0.0);
}

TEST_F(ControllerTest, TerminalSameGoalReplanMatchesResetGoalCheckerOutsideXy)
{
  terminalConfigure();
  setGoal(0.1, 0.0, 0.6);
  pluginlib::ClassLoader<nav2_core::GoalChecker> loader("nav2_core", "nav2_core::GoalChecker");
  auto checker = loader.createSharedInstance("nav2_controller::SimpleGoalChecker");
  checker->initialize(node_, "replan_checker", costmap_);
  const auto inside = makePose(0.0, 0.0);
  const auto goal = makePose(0.1, 0.0, 0.6);
  EXPECT_DOUBLE_EQ(
    controller_->computeVelocityCommands(inside, stopped_, checker.get()).twist.linear.x,
    0.0);
  EXPECT_FALSE(checker->isGoalReached(inside.pose, goal.pose, stopped_));
  // Nav2 setPlannerPath calls setPlan before resetting the stateful GoalChecker.
  setGoal(0.1, 0.0, 0.6);
  checker->reset();
  const auto outside = makePose(-0.2, 0.0, 0.6);
  EXPECT_FALSE(checker->isGoalReached(outside.pose, goal.pose, stopped_));
  const auto command = controller_->computeVelocityCommands(outside, stopped_, checker.get());
  EXPECT_GT(command.twist.linear.x, 0.0);
}

TEST_F(ControllerTest, TerminalInvalidPlanPreservesPreviousPathAndLatch)
{
  terminalConfigure();
  setGoal(0.1, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(compute().twist.linear.x, 0.0);
  std::vector<nav_msgs::msg::Path> invalid{nav_msgs::msg::Path{}};
  auto bad = straightPath();
  bad.header.frame_id.clear();
  invalid.push_back(bad);
  bad = straightPath();
  bad.header.frame_id = "/controller_local";
  invalid.push_back(bad);
  bad = straightPath();
  bad.poses.front().header.frame_id = "wrong_frame";
  invalid.push_back(bad);
  bad = straightPath();
  bad.poses.front().pose.position.x = std::nan("");
  invalid.push_back(bad);
  bad = straightPath();
  bad.poses.front().pose.orientation.w = 0.0;
  invalid.push_back(bad);
  bad = straightPath();
  bad.poses.back().pose.orientation.w = 0.0;
  invalid.push_back(bad);
  for (const auto & path : invalid) {
    EXPECT_THROW(controller_->setPlan(path), std::invalid_argument);
    const auto command =
      controller_->computeVelocityCommands(makePose(-0.3, 0.0), stopped_, nullptr);
    EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
    EXPECT_GT(command.twist.angular.z, 0.0);
  }
}

TEST_F(ControllerTest, TerminalRejectedConfigurationPreservesValidStateOrStaysInactive)
{
  terminalConfigure();
  setGoal(0.1, 0.0, 1.0);
  compute();
  EXPECT_THROW(controller_->configure(node_, "FollowPath", tf_, costmap_), std::logic_error);
  EXPECT_DOUBLE_EQ(
    controller_->computeVelocityCommands(
      makePose(-0.3, 0.0), stopped_, nullptr).twist.linear.x, 0.0);
  controller_->cleanup();
  ASSERT_TRUE(node_->set_parameter({"FollowPath.sim_time", -1.0}).successful);
  EXPECT_THROW(controller_->configure(node_, "FollowPath", tf_, costmap_), std::invalid_argument);
  EXPECT_THROW(controller_->activate(), std::logic_error);
  EXPECT_THROW(compute(), std::logic_error);
  ASSERT_TRUE(node_->set_parameter({"FollowPath.sim_time", 1.0}).successful);
  terminalConfigure();
  EXPECT_GT(compute().twist.linear.x, 0.0);
}

TEST_F(ControllerTest, TerminalRepeatedReplansWithResetGoalCheckerFinishRotation)
{
  terminalConfigure();
  pluginlib::ClassLoader<nav2_core::GoalChecker> loader("nav2_core", "nav2_core::GoalChecker");
  auto checker = loader.createSharedInstance("nav2_controller::SimpleGoalChecker");
  node_->declare_parameter("repeat_checker.xy_goal_tolerance", 0.2);
  node_->declare_parameter("repeat_checker.yaw_goal_tolerance", 0.15);
  checker->initialize(node_, "repeat_checker", costmap_);
  const auto goal = makePose(0.1, 0.0, 1.0);
  for (int period : {1, 20}) {
    SCOPED_TRACE(period);
    geometry_msgs::msg::Twist current;
    double yaw = 0.0;
    bool reached = false;
    for (int step = 0; step < 300; ++step) {
      if (step % period == 0) {
        setGoal(0.1, 0.0, 1.0);
        checker->reset();
      }
      const auto command = controller_->computeVelocityCommands(
        makePose(0.0, 0.0, yaw), current, checker.get());
      EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
      EXPECT_LE(std::abs(command.twist.angular.z - current.angular.z), 0.1 + kTolerance);
      yaw += command.twist.angular.z * 0.05;
      current = command.twist;
      if (checker->isGoalReached(makePose(0.0, 0.0, yaw).pose, goal.pose, current)) {
        reached = true;
        break;
      }
    }
    EXPECT_TRUE(reached);
    EXPECT_LE(std::abs(yaw - 1.0), 0.15 + kTolerance);
  }
}

TEST_F(ControllerTest, TerminalResetsForNewPositionAndYawTargets)
{
  terminalConfigure();
  setGoal(0.1, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(compute().twist.linear.x, 0.0);
  setGoal(2.0, 0.0, 1.0);
  EXPECT_GT(compute().twist.linear.x, 0.0);
  // Latch this goal from inside its XY region, then change only its yaw.
  controller_->computeVelocityCommands(makePose(1.9, 0.0), stopped_, nullptr);
  setGoal(2.0, 0.0, -1.0);
  EXPECT_GT(compute().twist.linear.x, 0.0);
}

TEST_F(ControllerTest, TerminalResetsOnDeactivateAndCleanupReconfigure)
{
  terminalConfigure();
  setGoal(0.1, 0.0, 1.0);
  compute();
  controller_->deactivate();
  controller_->activate();
  auto command = controller_->computeVelocityCommands(makePose(-0.5, 0.0), stopped_, nullptr);
  EXPECT_GT(command.twist.linear.x, 0.0);
  compute();
  controller_->cleanup();
  terminalConfigure();
  EXPECT_GT(compute().twist.linear.x, 0.0);
}

TEST_F(ControllerTest, TerminalRotationStillRejectsCollisionAtRobotPosition)
{
  terminalConfigure();
  setGoal(0.1, 0.0, 1.0);
  auto * map = costmap_->getCostmap();
  unsigned int mx, my;
  ASSERT_TRUE(map->worldToMap(0.0, 0.0, mx, my));
  map->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(compute(), nav2_core::PlannerException);
}

TEST_F(ControllerTest, TerminalKeepsRotationLatchWhenZeroBecomesUnreachable)
{
  terminalConfigure();
  setGoal(0.1, 0.0, 1.0);
  compute();
  geometry_msgs::msg::Twist pushed;
  pushed.linear.x = 0.2;
  EXPECT_THROW(
    controller_->computeVelocityCommands(makePose(0.0, 0.0), pushed, nullptr),
    nav2_core::PlannerException);
}

TEST_F(ControllerTest, TerminalUsesTrueGoalYawThroughRotatedTf)
{
  terminalConfigure();
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "world";
  transform.child_frame_id = "controller_local";
  transform.transform.translation.x = 10.0;
  transform.transform.translation.y = 20.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, kPi / 2.0);
  transform.transform.rotation = tf2::toMsg(q);
  ASSERT_TRUE(tf_->setTransform(transform, "controller_test", true));
  auto path = straightPath();
  path.header.frame_id = "world";
  path.poses = {makePose(10.0, 20.0, 0.0, "world"),
    makePose(10.0, 20.1, 0.0, "world")};
  controller_->setPlan(path);
  const auto command = compute();
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_LT(command.twist.angular.z, 0.0);
}

TEST_F(ControllerTest, TerminalPrebrakesOutsideXyWindowWithoutStoppingThere)
{
  ASSERT_TRUE(node_->set_parameter({"FollowPath.acc_lim_x", 0.2}).successful);
  for (const char * key : {"obstacle_weight", "goal_weight", "path_weight"}) {
    ASSERT_TRUE(node_->set_parameter({std::string("FollowPath.") + key, 0.0}).successful);
  }
  ASSERT_TRUE(node_->set_parameter({"FollowPath.speed_weight", 1.0}).successful);
  terminalConfigure();
  setGoal(0.3, 0.0, 0.0);
  geometry_msgs::msg::Twist current;
  current.linear.x = 0.4;
  const auto command = controller_->computeVelocityCommands(makePose(0.0, 0.0), current, nullptr);
  EXPECT_NEAR(command.twist.linear.x, 0.39, kTolerance);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
}

TEST_F(ControllerTest, TerminalCoarseAngularGridStillAllowsSmallUsefulRotation)
{
  ASSERT_TRUE(node_->set_parameter({"FollowPath.vtheta_samples", 2}).successful);
  terminalConfigure();
  setGoal(0.1, 0.0, 0.06);
  const auto command = compute();
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_GT(command.twist.angular.z, 0.0);
  EXPECT_LT(command.twist.angular.z, 0.1);
}


TEST_F(ControllerTest, TerminalUsesGoalCheckerYawToleranceWithoutCallingSuccessCheck)
{
  terminalConfigure();
  setGoal(0.1, 0.0, 0.1);
  pluginlib::ClassLoader<nav2_core::GoalChecker> loader("nav2_core", "nav2_core::GoalChecker");
  auto checker = loader.createSharedInstance("nav2_controller::SimpleGoalChecker");
  node_->declare_parameter("terminal_checker.xy_goal_tolerance", 0.2);
  node_->declare_parameter("terminal_checker.yaw_goal_tolerance", 0.15);
  checker->initialize(node_, "terminal_checker", costmap_);
  const auto command = controller_->computeVelocityCommands(
    makePose(
      0.0,
      0.0), stopped_, checker.get());
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
  // Our read-only call must not latch SimpleGoalChecker's XY success state.
  EXPECT_FALSE(
    checker->isGoalReached(
      makePose(2.0, 0.0).pose, makePose(0.1, 0.0, 0.1).pose,
      stopped_));
}

TEST_F(ControllerTest, TerminalReplanWithQuaternionSignFlipAlsoResetsLatch)
{
  terminalConfigure();
  setGoal(0.1, 0.0, 1.0);
  compute();
  auto path = straightPath();
  path.poses.back() = makePose(0.1, 0.0, 1.0);
  auto & q = path.poses.back().pose.orientation;
  q.x *= -1; q.y *= -1; q.z *= -1; q.w *= -1;
  controller_->setPlan(path);
  const auto command = controller_->computeVelocityCommands(makePose(-0.5, 0.0), stopped_, nullptr);
  EXPECT_GT(command.twist.linear.x, 0.0);
}

TEST_F(ControllerTest, TerminalSlowdownRejectsUnknownSpaceWithoutUncheckedStop)
{
  // Even terminal braking must reject unknown cells; no unchecked zero shortcut.
  terminalConfigure();
  setGoal(0.1, 0.0, 1.0);
  auto * map = costmap_->getCostmap();
  std::fill_n(
    map->getCharMap(), map->getSizeInCellsX() * map->getSizeInCellsY(),
    nav2_costmap_2d::NO_INFORMATION);
  geometry_msgs::msg::Twist current;
  current.linear.x = 0.2;
  EXPECT_THROW(
    controller_->computeVelocityCommands(makePose(0.0, 0.0), current, nullptr),
    nav2_core::PlannerException);
}

TEST_F(ControllerTest, TerminalClosedLoopApproachSlowsThenRotatesAtSameGoal)
{
  terminalConfigure();
  setGoal(0.8, 0.0, 1.0);
  geometry_msgs::msg::Twist current;
  double x = 0.0, y = 0.0, yaw = 0.0;
  bool inside = false, rotating = false;
  for (int i = 0; i < 500; ++i) {
    const auto command =
      controller_->computeVelocityCommands(makePose(x, y, yaw), current, nullptr);
    const double v = command.twist.linear.x, w = command.twist.angular.z;
    EXPECT_LE(std::abs(v - current.linear.x), 0.05 + kTolerance);
    EXPECT_LE(std::abs(w - current.angular.z), 0.1 + kTolerance);
    if (std::hypot(x - 0.8, y) <= 0.25) {
      inside = true;
    }
    if (inside && v == 0.0) {
      rotating = true;
    }
    if (rotating) {
      EXPECT_DOUBLE_EQ(v, 0.0);
    }
    x += v * std::cos(yaw) * 0.05;
    y += v * std::sin(yaw) * 0.05;
    yaw += w * 0.05;
    current = command.twist;
    // Nav2 replans to the exact same endpoint periodically.
    if (i % 10 == 0) {
      setGoal(0.8, 0.0, 1.0);
    }
  }
  EXPECT_TRUE(inside);
  EXPECT_TRUE(rotating);
  EXPECT_LE(std::hypot(x - 0.8, y), 0.25 + kTolerance);
  EXPECT_LE(std::abs(yaw - 1.0), 0.05 + kTolerance);
  EXPECT_DOUBLE_EQ(current.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(current.angular.z, 0.0);
}

TEST_F(ControllerTest, TerminalRejectsInvalidThresholdsAndMissingBrakingAuthority)
{
  configure();
  controller_->cleanup();
  const std::vector<rclcpp::Parameter> invalid{
    {"FollowPath.trans_stopped_velocity", 0.0},
    {"FollowPath.trans_stopped_velocity", std::nan("")},
    {"FollowPath.terminal_xy_tolerance", -0.1},
    {"FollowPath.terminal_yaw_tolerance", 4.0},
    {"FollowPath.acc_lim_x", 0.0},
    {"FollowPath.acc_lim_theta", 0.0},
    {"FollowPath.max_vel_theta", 0.0}};
  for (const auto & parameter : invalid) {
    SCOPED_TRACE(parameter.get_name());
    const auto original = node_->get_parameter(parameter.get_name());
    ASSERT_TRUE(node_->set_parameter(parameter).successful);
    EXPECT_THROW(controller_->configure(node_, "FollowPath", tf_, costmap_), std::invalid_argument);
    EXPECT_THROW(controller_->activate(), std::logic_error);
    ASSERT_TRUE(node_->set_parameter(original).successful);
  }
}

TEST_F(ControllerTest, TerminalFallsBackForUnavailableAndMalformedGoalTolerances)
{
  terminalConfigure();
  setGoal(0.1, 0.0, 0.1);
  UnsupportedGoalChecker checker;
  for (bool available : {false, true}) {
    checker.available = available;
    checker.tolerance.position.x = std::numeric_limits<double>::lowest();
    checker.tolerance.position.y = std::numeric_limits<double>::quiet_NaN();
    // Default all-zero quaternion is unsupported, so fallback yaw remains 0.05.
    const auto command =
      controller_->computeVelocityCommands(makePose(0.0, 0.0), stopped_, &checker);
    EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
    EXPECT_GT(command.twist.angular.z, 0.0);
  }
}

TEST_F(ControllerTest, TerminalResetsWhenGoalCoordinateFrameChanges)
{
  terminalConfigure();
  setGoal(0.1, 0.0, 1.0);
  compute();
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "world";
  transform.child_frame_id = "controller_local";
  transform.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf_->setTransform(transform, "controller_test", true));
  auto path = straightPath();
  path.header.frame_id = "world";
  path.poses = {makePose(0.0, 0.0, 0.0, "world"), makePose(0.1, 0.0, 1.0, "world")};
  controller_->setPlan(path);
  const auto command = controller_->computeVelocityCommands(makePose(-0.5, 0.0), stopped_, nullptr);
  EXPECT_GT(command.twist.linear.x, 0.0);
}

// Fixed, deterministic input sweep also emits an exact floating-point transcript.
// Compare this transcript before/after each structural-refactor stage.
TEST_F(ControllerTest, RefactorCharacterizationCommandAndExceptionSweep)
{
  terminalConfigure();
  unsigned int index = 0;
  // Oracle captured from the unchanged controller; 1e-12 output precision avoids
  // binding the permanent test to insignificant platform-specific roundoff.
  std::uint64_t signature = 14695981039346656037ULL;
  const auto record_signature = [&](const std::string & record) {
      for (unsigned char byte : record) {
        signature = (signature ^ byte) * 1099511628211ULL;
      }
    };
  for (double distance : {0.1, 0.25, 0.250001, 0.5, 2.0}) {
    for (double v : {0.0, 0.01, 0.2, 0.4, 0.55}) {
      for (double w : {-0.8, 0.0, 0.8}) {
        for (double yaw : {-3.13, 0.0, 1.0}) {
          setGoal(2.5, 0.0, 2.0);  // Clear any previous terminal latch.
          setGoal(distance, 0.0, yaw);
          geometry_msgs::msg::Twist current;
          current.linear.x = v;
          current.angular.z = w;
          try {
            const auto result = controller_->computeVelocityCommands(
              makePose(0.0, 0.0), current, nullptr);
            EXPECT_LE(std::abs(result.twist.linear.x - v), 0.05 + kTolerance);
            EXPECT_LE(std::abs(result.twist.angular.z - w), 0.1 + kTolerance);
            EXPECT_EQ(result.header.frame_id, "controller_base");
            std::ostringstream record;
            record << index << ":" << std::fixed << std::setprecision(12)
                   << result.twist.linear.x << "," << result.twist.angular.z;
            record_signature(record.str());
            std::cout << "[DWA_REFACTOR_TRACE] " << index << " "
                      << std::hexfloat << result.twist.linear.x << " "
                      << result.twist.angular.z << std::defaultfloat << std::endl;
          } catch (const nav2_core::PlannerException & error) {
            EXPECT_GT(v, 0.4);
            record_signature(std::to_string(index) + ":PlannerException:" + error.what());
            std::cout << "[DWA_REFACTOR_TRACE] " << index << " PlannerException "
                      << error.what() << std::endl;
          }
          ++index;
        }
      }
    }
  }
  EXPECT_EQ(index, 225u);
  EXPECT_EQ(signature, 0xb4588442ba697cdcULL);
}

TEST_F(ControllerTest, RefactorRethrowsNonStandardGoalCheckerException)
{
  class ThrowingChecker : public UnsupportedGoalChecker
  {
public:
    bool getTolerances(geometry_msgs::msg::Pose &, geometry_msgs::msg::Twist &) override
    {
      throw 37;
    }
  } checker;
  configure();
  try {
    controller_->computeVelocityCommands(makePose(0.0, 0.0), stopped_, &checker);
    FAIL() << "Expected original non-standard exception";
  } catch (int value) {
    EXPECT_EQ(value, 37);
  }
}

TEST_F(ControllerTest, PathConflictOptInWaitsButDisabledKeepsBaseline)
{
  auto * grid = costmap_->getCostmap();
  unsigned int mx, my;
  ASSERT_TRUE(grid->worldToMap(0.8, 0.0, mx, my));
  grid->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  node_->declare_parameter("FollowPath.path_conflict.enabled", false);
  configure();
  const auto baseline = compute();
  EXPECT_GT(baseline.twist.linear.x, 0.0);
  controller_->cleanup();
  ASSERT_TRUE(node_->set_parameter({"FollowPath.path_conflict.enabled", true}).successful);
  configure();
  const auto governed = compute();
  EXPECT_DOUBLE_EQ(governed.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(governed.twist.angular.z, 0.0);
}

TEST_F(ControllerTest, PathConflictBrakingKeepsReachableWindowAndExternalSpeedLimit)
{
  node_->declare_parameter("FollowPath.path_conflict.enabled", true);
  terminalConfigure();
  auto * grid = costmap_->getCostmap();
  unsigned int mx, my;
  ASSERT_TRUE(grid->worldToMap(0.8, 0.0, mx, my));
  grid->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  geometry_msgs::msg::Twist current;
  current.linear.x = 0.2;
  const auto d = controller_->computeVelocityCommands(makePose(0, 0), current, nullptr);
  EXPECT_GE(d.twist.linear.x, 0.15 - kTolerance);
  EXPECT_LE(d.twist.linear.x, 0.185 + kTolerance);
  controller_->setSpeedLimit(50.0, true);
  // The persistent external cap survives transient risk and lifecycle reset.
  controller_->deactivate();
  controller_->activate();
  grid->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
  current.linear.x = 0.2;
  EXPECT_LE(
    controller_->computeVelocityCommands(makePose(0, 0), current, nullptr).twist.linear.x,
    0.2 + kTolerance);
  controller_->setSpeedLimit(nav2_costmap_2d::NO_SPEED_LIMIT, false);
  EXPECT_GT(
    controller_->computeVelocityCommands(makePose(0, 0), current, nullptr).twist.linear.x,
    0.2);
}

TEST_F(ControllerTest, PathConflictWaitDoesNotBypassFeasibilityOrTerminalRotation)
{
  node_->declare_parameter("FollowPath.path_conflict.enabled", true);
  terminalConfigure();
  setWall(nav2_costmap_2d::NO_INFORMATION);
  auto * grid = costmap_->getCostmap();
  unsigned int mx, my;
  ASSERT_TRUE(grid->worldToMap(0.0, 0.0, mx, my));
  grid->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(compute(), nav2_core::PlannerException);
  std::fill_n(grid->getCharMap(), grid->getSizeInCellsX() * grid->getSizeInCellsY(), 0);
  setGoal(0.1, 0.0, 1.0);
  const auto command = compute();
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  EXPECT_GT(command.twist.angular.z, 0.0);
}

TEST_F(ControllerTest, PathConflictAngularBrakingBeforeReachableStandstill)
{
  node_->declare_parameter("FollowPath.path_conflict.enabled", true);
  ASSERT_TRUE(node_->set_parameter({"FollowPath.acc_lim_theta", 0.1}).successful);
  terminalConfigure();
  auto * grid = costmap_->getCostmap();
  unsigned int mx, my;
  ASSERT_TRUE(grid->worldToMap(0.8, 0.0, mx, my));
  grid->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  geometry_msgs::msg::Twist current;
  current.angular.z = 0.04;
  for (int i = 0; i < 10; ++i) {
    const auto command = controller_->computeVelocityCommands(makePose(0, 0), current, nullptr);
    EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
    EXPECT_NEAR(command.twist.angular.z, std::max(0.0, current.angular.z - 0.005), kTolerance);
    current = command.twist;
  }
}

TEST_F(ControllerTest, PathConflictSameGoalReplanKeepsWaitNewGoalResetsIt)
{
  node_->declare_parameter("FollowPath.path_conflict.enabled", true);
  terminalConfigure();
  auto * grid = costmap_->getCostmap();
  unsigned int mx, my;
  ASSERT_TRUE(grid->worldToMap(0.8, 0.0, mx, my));
  grid->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_DOUBLE_EQ(compute().twist.linear.x, 0.0);
  controller_->setPlan(straightPath());
  EXPECT_DOUBLE_EQ(compute().twist.linear.x, 0.0);
  setGoal(2.0, 1.0, 0.0);  // A different task with a clear route must not inherit WAIT.
  EXPECT_GT(compute().twist.linear.x, 0.0);
}

TEST_F(ControllerTest, PathConflictWarningBelowWindowKeepsReachableDecelerationBoundary)
{
  node_->declare_parameter("FollowPath.path_conflict.enabled", true);
  terminalConfigure();
  auto * grid = costmap_->getCostmap();
  const auto add_row = [&](double y) {
      for (double x : {0.825, 0.875, 0.925}) {
        unsigned int mx, my;
        ASSERT_TRUE(grid->worldToMap(x, y, mx, my));
        grid->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    };
  add_row(0.525);
  compute();
  add_row(0.325);
  controller_->setSpeedLimit(0.38, false);
  geometry_msgs::msg::Twist current;
  current.linear.x = 0.4;
  const auto command = controller_->computeVelocityCommands(makePose(0, 0), current, nullptr);
  EXPECT_NEAR(command.twist.linear.x, 0.35, kTolerance);
}

TEST_F(ControllerTest, PathConflictClosedLoopStopsHoldsDuplicatesAndResumesAfterClearStates)
{
  node_->declare_parameter("FollowPath.path_conflict.enabled", true);
  terminalConfigure();
  auto * grid = costmap_->getCostmap();
  unsigned int mx, my;
  ASSERT_TRUE(grid->worldToMap(0.825, 0.025, mx, my));
  grid->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  geometry_msgs::msg::Twist current;
  current.linear.x = 0.2;
  double x = 0.0;
  for (int i = 0; i < 30; ++i) {
    if (i % 4 == 0) {controller_->setPlan(straightPath());}
    const auto command = controller_->computeVelocityCommands(makePose(x, 0), current, nullptr);
    EXPECT_LE(std::abs(command.twist.linear.x - current.linear.x), 0.05 + kTolerance);
    x += command.twist.linear.x * 0.05;
    current = command.twist;
  }
  EXPECT_DOUBLE_EQ(current.linear.x, 0.0);
  EXPECT_LT(x + 0.22 + 0.1, 0.8);
  grid->setCost(mx, my, 0);
  for (int i = 0; i < 20; ++i) {
    EXPECT_DOUBLE_EQ(
      controller_->computeVelocityCommands(makePose(x, 0), current, nullptr).
      twist.linear.x, 0.0);
  }
  for (unsigned char cost : {1, 2}) {
    grid->setCost(2, 2, cost);  // Two more actual content changes, outside the corridor.
    current = controller_->computeVelocityCommands(makePose(x, 0), current, nullptr).twist;
  }
  EXPECT_GT(current.linear.x, 0.0);
  for (int i = 0; i < 20; ++i) {
    const auto command = controller_->computeVelocityCommands(makePose(x, 0), current, nullptr);
    EXPECT_LE(std::abs(command.twist.linear.x - current.linear.x), 0.05 + kTolerance);
    x += command.twist.linear.x * 0.05;
    current = command.twist;
  }
  EXPECT_GT(current.linear.x, 0.1);
}

TEST_F(ControllerTest, FallbackPrefersCurrentSpeedAndKeepsAngularWindow)
{
  node_->declare_parameter("FollowPath.path_conflict.enabled", true);
  terminalConfigure();
  auto * grid = costmap_->getCostmap();
  unsigned int mx, my;
  ASSERT_TRUE(grid->worldToMap(0.15, 0.175, mx, my));
  grid->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  geometry_msgs::msg::Twist current;
  current.angular.z = 0.4;
  for (double v : {0.0, 0.03}) {
    current.linear.x = v;  // 0.03 is absent from the original grid {0, 0.04, 0.08}.
    const auto command = controller_->computeVelocityCommands(makePose(0, 0), current, nullptr);
    EXPECT_NEAR(command.twist.linear.x, v, kTolerance);
    EXPECT_GE(command.twist.angular.z, 0.3 - kTolerance);
    EXPECT_LE(command.twist.angular.z, 0.5 + kTolerance);
    EXPECT_TRUE(
      evaluateTrajectory(
        TrajectoryPredictor(1.0, 0.5).predict(
          Pose2D{}, {command.twist.linear.x, command.twist.angular.z}), *grid).collision_free);
  }
}

TEST_F(ControllerTest, FallbackKeepsAcceleratingEscapeWhenEveryPreferredCandidateIsBlocked)
{
  node_->declare_parameter("FollowPath.path_conflict.enabled", true);
  ASSERT_TRUE(node_->set_parameter({"FollowPath.simulation_dt", 0.05}).successful);
  terminalConfigure();
  auto * grid = costmap_->getCostmap();
  grid->resizeMap(400, 400, 0.005, -0.5, -0.5);
  const auto mark = [&](double x, double y, unsigned char cost) {
      unsigned int mx, my;
      ASSERT_TRUE(grid->worldToMap(x, y, mx, my));
      grid->setCost(mx, my, cost);
    };
  mark(0.15, 0.175, nav2_costmap_2d::LETHAL_OBSTACLE);  // Uncertifiable close conflict.
  // Block endpoints of the tighter, slower arcs. The faster, wider arc stays
  // clear; this exercises escape retention without jumping over a thin obstacle.
  const TrajectoryPredictor predictor(1.0, 0.05);
  for (double v : {0.07, 0.12}) {
    for (double w : {0.5, 0.6, 0.7}) {
      const auto end = predictor.predict(Pose2D{}, {v, w}).poses.back();
      mark(end.x, end.y, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
    }
  }
  geometry_msgs::msg::Twist current;
  current.linear.x = 0.12;
  current.angular.z = 0.6;
  const auto command = controller_->computeVelocityCommands(makePose(0, 0), current, nullptr);
  EXPECT_NEAR(command.twist.linear.x, 0.17, kTolerance);
  EXPECT_TRUE(
    evaluateTrajectory(
      TrajectoryPredictor(1.0, 0.005).predict(
        Pose2D{}, {command.twist.linear.x, command.twist.angular.z}), *grid).collision_free);
}

TEST_F(ControllerTest, FallbackPreferencePersistsUntilClearAndDoesNotCreateWait)
{
  node_->declare_parameter("FollowPath.path_conflict.enabled", true);
  terminalConfigure();
  auto * grid = costmap_->getCostmap();
  unsigned int mx, my;
  ASSERT_TRUE(grid->worldToMap(0.15, 0.175, mx, my));
  grid->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  geometry_msgs::msg::Twist current;
  current.linear.x = 0.12;
  EXPECT_LE(
    controller_->computeVelocityCommands(makePose(0, 0), current, nullptr).
    twist.linear.x, 0.12 + kTolerance);
  grid->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
  for (int i = 0; i < 5; ++i) {
    EXPECT_LE(
      controller_->computeVelocityCommands(makePose(0, 0), current, nullptr).
      twist.linear.x, 0.12 + kTolerance);
  }
  grid->setCost(1, 1, 1);
  EXPECT_LE(
    controller_->computeVelocityCommands(makePose(0, 0), current, nullptr).
    twist.linear.x, 0.12 + kTolerance);
  grid->setCost(1, 1, 2);
  EXPECT_GT(
    controller_->computeVelocityCommands(makePose(0, 0), current, nullptr).
    twist.linear.x, 0.12 + kTolerance);
}

}  // 匿名命名空间
}  // 命名空间 my_dwa_controller

namespace my_dwa_controller
{
namespace
{
TEST_F(ControllerTest, ScanFallbackSoftBoundaryCannotExceedPreservedHardCap)
{
  node_->declare_parameter("FollowPath.path_conflict.enabled", true);
  node_->declare_parameter("FollowPath.scan_tracker.enabled", true);
  terminalConfigure();
  auto * grid = costmap_->getCostmap();
  unsigned int x, y;
  ASSERT_TRUE(grid->worldToMap(.8, 0, x, y));
  grid->setCost(x, y, 254);
  geometry_msgs::msg::Twist current;
  current.linear.x = .2;
  const auto braking = controller_->computeVelocityCommands(makePose(0, 0), current, nullptr);
  ASSERT_LE(braking.twist.linear.x, .185 + kTolerance);
  grid->setCost(x, y, 0);
  ASSERT_TRUE(grid->worldToMap(.15, .175, x, y));
  grid->setCost(x, y, 254);
  const auto fallback = controller_->computeVelocityCommands(makePose(0, 0), current, nullptr);
  EXPECT_LE(fallback.twist.linear.x, .185 + kTolerance);
}

TEST_F(ControllerTest, PublishedScansEnterPluginRiskAndLifecycleDropsSubscription)
{
  node_->declare_parameter("FollowPath.path_conflict.enabled", true);
  node_->declare_parameter("FollowPath.scan_tracker.enabled", true);
  node_->declare_parameter("FollowPath.scan_tracker.fixed_frame", "controller_local");
  node_->declare_parameter("FollowPath.scan_tracker.topic", "/controller_test_scan");
  terminalConfigure();
  geometry_msgs::msg::TransformStamped tr;
  tr.header.frame_id = "controller_local";
  tr.child_frame_id = "test_laser";
  tr.transform.rotation.w = 1;
  ASSERT_TRUE(tf_->setTransform(tr, "test", true));
  auto pub = node_->create_publisher<sensor_msgs::msg::LaserScan>(
    "/controller_test_scan", rclcpp::SensorDataQoS());
  pub->on_activate();
  for (int i = 0; i < 100 && pub->get_subscription_count() == 0; ++i) {
    rclcpp::spin_some(node_->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_EQ(pub->get_subscription_count(), 1u);
  const double end = node_->now().seconds();
  for (int k = 0; k < 5; ++k) {
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id = "test_laser";
    scan.header.stamp = rclcpp::Time(static_cast<int64_t>((end - .79 + .19 * k) * 1e9));
    scan.angle_min = -1.5; scan.angle_increment = .005;
    scan.range_min = .1; scan.range_max = 4;
    scan.ranges.assign(600, INFINITY);
    const double x = .9, y = -.6 + .0475 * k;
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const double a = scan.angle_min + i * scan.angle_increment;
      const double dot = x * std::cos(a) + y * std::sin(a);
      const double d = dot * dot - (x * x + y * y - .18 * .18);
      if (d >= 0 && dot > 0) {scan.ranges[i] = dot - std::sqrt(d);}
    }
    pub->publish(scan);
    for (int i = 0; i < 4; ++i) {
      rclcpp::spin_some(node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
  // Empty Costmap. Only the scan's future crossing can request a wait here.
  const auto command = compute();
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
  controller_->setPlan(straightPath());  // Same-goal replanning retains risk.
  EXPECT_DOUBLE_EQ(compute().twist.linear.x, 0.0);
  controller_->deactivate();
  controller_->activate();
  EXPECT_GT(compute().twist.linear.x, 0.0);  // Reset requires new scan evidence.
  controller_->deactivate();
  for (int i = 0; i < 100 && pub->get_subscription_count() != 0; ++i) {
    rclcpp::spin_some(node_->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_EQ(pub->get_subscription_count(), 0u);
}
}  // namespace
}  // namespace my_dwa_controller
