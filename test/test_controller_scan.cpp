#include <gtest/gtest.h>
#include "scan_fixture.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <string>
#include <vector>
#include <stdexcept>
#include <functional>
#include <memory>
#include <thread>

#include "my_dwa_controller/controller.hpp"
#include "my_dwa_controller/dynamic_risk_evaluator.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "rcl/time.h"
#include "rcutils/logging.h"
#include "rclcpp/executors/single_threaded_executor.hpp"

namespace my_dwa_controller
{
namespace
{

class ScanTestCostmap : public nav2_costmap_2d::Costmap2DROS
{
public:
  ScanTestCostmap()
  : Costmap2DROS("v3_scan_test_costmap")
  {
    setCircularRadius(0.22);
    global_frame_ = "perception_world";
    robot_base_frame_ = "perception_base";
    layered_costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>(
      global_frame_, false, false);
    layered_costmap_->resizeMap(120, 120, 0.05, -3.0, -3.0);
  }

  void setCircularRadius(double radius)
  {
    robot_radius_ = radius;
    use_radius_ = true;
  }
};

class ScanTestExecutor : public rclcpp::executors::SingleThreadedExecutor
{
public:
  rclcpp::SubscriptionBase::SharedPtr selectSubscription()
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < end) {
      wait_for_work(std::chrono::milliseconds(50));
      rclcpp::AnyExecutable selected;
      if (get_next_ready_executable(selected) && selected.subscription) {
        if (std::string(selected.subscription->get_topic_name()) == "/v3_scan_test") {
          return selected.subscription;
        }
        execute_subscription(selected.subscription);
      }
    }
    return nullptr;
  }

  static void runSelected(const rclcpp::SubscriptionBase::SharedPtr & subscription)
  {
    execute_subscription(subscription);
  }
};

class ControllerScanTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("v3_scan_test_controller");
    node->declare_parameter("FollowPath.scan_tracker.topic", "/v3_scan_test");
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
    ASSERT_EQ(
      rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), 1200000000LL), RCL_RET_OK);
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    costmap = std::make_shared<ScanTestCostmap>();
    publisher_node = std::make_shared<rclcpp::Node>("v3_scan_test_publisher");
    publisher = publisher_node->create_publisher<sensor_msgs::msg::LaserScan>(
      "/v3_scan_test", rclcpp::SensorDataQoS());
    executor.add_node(node->get_node_base_interface());
    executor.add_node(publisher_node);
  }

  bool waitFor(const std::function<bool()> & condition)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < end) {
      executor.spin_some();
      if (condition()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  }

  void publish(int64_t stamp, const std::string & frame = "perception_world")
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = rclcpp::Time(stamp);
    scan.header.frame_id = frame;
    scan.angle_min = 0.0;
    scan.angle_increment = 0.01;
    scan.range_min = 0.12;
    scan.range_max = 3.5;
    scan.ranges = {1, 1, 1};
    publisher->publish(scan);
  }

  geometry_msgs::msg::TwistStamped command(
    const geometry_msgs::msg::Twist & velocity = geometry_msgs::msg::Twist{})
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "perception_world";
    pose.header.stamp = node->now();
    pose.pose.orientation.w = 1.0;
    return controller.computeVelocityCommands(pose, velocity, nullptr);
  }

  void setPlan()
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = "perception_world";
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = path.header.frame_id;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
    pose.pose.position.x = 2.0;
    path.poses.push_back(pose);
    controller.setPlan(path);
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::shared_ptr<ScanTestCostmap> costmap;
  std::shared_ptr<rclcpp::Node> publisher_node;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher;
  ScanTestExecutor executor;
  MyDWAController controller;
};

TEST_F(ControllerScanTest, DisabledByDefaultHasNoSubscriptionOrObservation)
{
  controller.configure(node, "FollowPath", tf, costmap);
  controller.activate();
  executor.spin_some();
  EXPECT_EQ(publisher->get_subscription_count(), 0u);
  EXPECT_FALSE(controller.getScanSnapshot());
  controller.deactivate();
  controller.cleanup();
}

TEST_F(ControllerScanTest, ActiveSubscriptionMovesFromUnavailableToObservedGeometry)
{
  node->declare_parameter("FollowPath.scan_tracker.enabled", true);
  controller.configure(node, "FollowPath", tf, costmap);
  EXPECT_FALSE(controller.getScanSnapshot());
  controller.activate();
  setPlan();
  const auto before = command();
  ASSERT_TRUE(waitFor([&]() {return publisher->get_subscription_count() == 1u;}));
  publish(1000000000LL);
  ASSERT_TRUE(waitFor([&]() {return static_cast<bool>(controller.getScanSnapshot());}));
  const auto first = controller.getScanSnapshot();
  ASSERT_EQ(first->obstacles.size(), 1u);
  EXPECT_EQ(first->frame_id, "perception_world");
  EXPECT_FALSE(first->obstacles[0].velocity_valid);
  const auto after = command();
  EXPECT_DOUBLE_EQ(before.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(before.twist.angular.z, 0.0);
  EXPECT_GT(after.twist.linear.x, 0.0);
  publish(1200000000LL);
  ASSERT_TRUE(
    waitFor(
      [&]() {
        const auto value = controller.getScanSnapshot();
        return value && value->stamp.nanosec == 200000000u;
      }));
  EXPECT_FALSE(controller.getScanSnapshot()->obstacles[0].velocity_valid);
  // Pending TF is not processing failure and must preserve the previous baseline.
  publish(1400000000LL, "missing_laser");
  const auto selected = executor.selectSubscription();
  ASSERT_TRUE(selected);
  executor.runSelected(selected);
  EXPECT_EQ(controller.getScanSnapshot()->stamp.nanosec, 200000000u);
  controller.deactivate();
  controller.cleanup();
}

TEST_F(ControllerScanTest, LifecycleClearsObservationAndRestartsWithoutOldVelocity)
{
  node->declare_parameter("FollowPath.scan_tracker.enabled", true);
  controller.configure(node, "FollowPath", tf, costmap);
  controller.activate();
  ASSERT_TRUE(waitFor([&]() {return publisher->get_subscription_count() == 1u;}));
  publish(1000000000LL);
  ASSERT_TRUE(waitFor([&]() {return static_cast<bool>(controller.getScanSnapshot());}));
  controller.deactivate();
  EXPECT_FALSE(controller.getScanSnapshot());
  ASSERT_TRUE(waitFor([&]() {return publisher->get_subscription_count() == 0u;}));
  publish(1200000000LL);
  executor.spin_some();
  EXPECT_FALSE(controller.getScanSnapshot());
  controller.activate();
  ASSERT_TRUE(waitFor([&]() {return publisher->get_subscription_count() == 1u;}));
  ASSERT_EQ(
    rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), 1400000000LL), RCL_RET_OK);
  publish(1400000000LL);
  ASSERT_TRUE(waitFor([&]() {return static_cast<bool>(controller.getScanSnapshot());}));
  ASSERT_EQ(controller.getScanSnapshot()->obstacles.size(), 1u);
  EXPECT_FALSE(controller.getScanSnapshot()->obstacles[0].velocity_valid);
  controller.deactivate();
  controller.cleanup();
  EXPECT_FALSE(controller.getScanSnapshot());
  EXPECT_EQ(publisher->get_subscription_count(), 0u);
}

TEST_F(ControllerScanTest, CallbackSelectedBeforeRestartCannotPopulateNewActivation)
{
  node->declare_parameter("FollowPath.scan_tracker.enabled", true);
  controller.configure(node, "FollowPath", tf, costmap);
  controller.activate();
  ASSERT_TRUE(waitFor([&]() {return publisher->get_subscription_count() == 1u;}));
  publish(1000000000LL);
  // Executor selection retains the old subscription even after reset(). Delay
  // its actual execution until the controller has entered a new activation.
  const auto selected = executor.selectSubscription();
  ASSERT_TRUE(selected);
  controller.deactivate();
  controller.activate();
  executor.runSelected(selected);
  EXPECT_FALSE(controller.getScanSnapshot());
  controller.deactivate();
  controller.cleanup();
}

TEST_F(ControllerScanTest, TimerProcessesDelayedExactStampTfWithoutInventingVelocity)
{
  node->declare_parameter("FollowPath.scan_tracker.enabled", true);
  controller.configure(node, "FollowPath", tf, costmap);
  controller.activate();
  ASSERT_TRUE(waitFor([&]() {return publisher->get_subscription_count() == 1u;}));
  geometry_msgs::msg::TransformStamped value;
  value.header.frame_id = "perception_world";
  value.child_frame_id = "laser";
  value.header.stamp = rclcpp::Time(1000000000LL);
  value.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf->setTransform(value, "unit_test"));
  publish(1000000000LL, "laser");
  ASSERT_TRUE(waitFor([&]() {return static_cast<bool>(controller.getScanSnapshot());}));
  const auto baseline = controller.getScanSnapshot();
  for (const int64_t stamp : {1200000000LL, 1400000000LL}) {
    publish(stamp, "laser");
    const auto selected = executor.selectSubscription();
    ASSERT_TRUE(selected);
    executor.runSelected(selected);
    EXPECT_EQ(controller.getScanSnapshot(), baseline);
  }
  value.header.stamp = rclcpp::Time(1400000000LL);
  value.transform.translation.x = 0.2;
  value.transform.translation.y = -0.08;
  ASSERT_TRUE(tf->setTransform(value, "unit_test"));
  ASSERT_EQ(
    rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), 1400000000LL), RCL_RET_OK);
  ASSERT_TRUE(
    waitFor(
      [&]() {
        return controller.getScanSnapshot()->stamp.nanosec == 400000000u;
      }));
  const auto & obstacle = controller.getScanSnapshot()->obstacles.at(0);
  EXPECT_FALSE(obstacle.velocity_valid);
  EXPECT_DOUBLE_EQ(obstacle.vx, 0.0);
  EXPECT_DOUBLE_EQ(obstacle.vy, 0.0);
  controller.deactivate();
  controller.cleanup();
}

// Reuse the scan subscription fixture and real Tracker; a fixed ROS clock makes
// freshness and obstacle prediction independent of test-machine execution time.
class ControllerDynamicTest : public ControllerScanTest
{
protected:
  void SetUp() override
  {
    ControllerScanTest::SetUp();
    ASSERT_EQ(rcl_enable_ros_time_override(node->get_clock()->get_clock_handle()), RCL_RET_OK);
    setTime(2000000000LL);
    ASSERT_EQ(
      rcutils_logging_set_logger_level(
        node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG), RCUTILS_RET_OK);
    // Synthetic 2 Hz fixture: explicit ideal actuator, distinct from real calibration.
    node->declare_parameter("FollowPath.motion.response_delay", 0.0);
    node->declare_parameter("FollowPath.motion.angular_acceleration", 2.0);
    node->declare_parameter("controller_frequency", 2.0);
    node->declare_parameter("FollowPath.max_vel_x", 0.4);
    node->declare_parameter("FollowPath.max_vel_theta", 1.0);
    node->declare_parameter("FollowPath.acc_lim_x", 1.0);
    node->declare_parameter("FollowPath.acc_lim_theta", 2.0);
    node->declare_parameter("FollowPath.vx_samples", 3);
    node->declare_parameter("FollowPath.vtheta_samples", 3);
    node->declare_parameter("FollowPath.sim_time", 1.0);
    node->declare_parameter("FollowPath.simulation_dt", 0.5);
    node->declare_parameter("FollowPath.forward_length", 1.0);
    node->declare_parameter("FollowPath.obstacle_weight", 0.5);
    node->declare_parameter("FollowPath.speed_weight", 0.1);
    node->declare_parameter("FollowPath.goal_weight", 0.2);
    node->declare_parameter("FollowPath.path_weight", 0.2);
    node->declare_parameter("FollowPath.heading_weight", 0.0);
    node->declare_parameter("FollowPath.follow_lookahead", 1.0);
  }

  void TearDown() override
  {
    controller.deactivate();
    controller.cleanup();
    EXPECT_EQ(
      rcutils_logging_set_logger_level(
        node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO), RCUTILS_RET_OK);
  }

  void setTime(int64_t stamp)
  {
    ASSERT_EQ(rcl_set_ros_time_override(node->get_clock()->get_clock_handle(), stamp), RCL_RET_OK);
  }

  void configureDecision(bool enabled = true)
  {
    if (enabled) {
      node->declare_parameter("FollowPath.scan_tracker.enabled", true);
    }
    controller.configure(node, "FollowPath", tf, costmap);
    setPlan();
    controller.activate();
    if (enabled) {
      ASSERT_TRUE(waitFor([&]() {return publisher->get_subscription_count() == 1u;}));
    }
  }

  void publishGeometry(int64_t stamp, double x, double y, bool empty = false)
  {
    auto scan = cornerScan(stamp, "perception_world", x, y);
    if (empty) {scan.ranges.clear();}
    publisher->publish(scan);
    ASSERT_TRUE(
      waitFor(
        [&]() {
          const auto latest = controller.getScanSnapshot();
          return latest && rclcpp::Time(latest->stamp).nanoseconds() == stamp;
        }));
  }

  void observe(double x, double y, double vx = 0.0, double vy = 0.0, bool known = true)
  {
    const int64_t stamp = node->now().nanoseconds() + 400000000LL;
    setTime(stamp);
    if (known) {
      publishGeometry(stamp - 200000000LL, x - vx * 0.2, y - vy * 0.2);
    }
    publishGeometry(stamp, x, y);
    const auto latest = controller.getScanSnapshot();
    ASSERT_TRUE(latest);
    ASSERT_EQ(latest->obstacles.size(), 1u);
    ASSERT_EQ(latest->obstacles[0].velocity_valid, known);
  }


  void observeNarrowCrossing()
  {
    // Crop the two faces so the cached crossing lies ahead of the robot.
    // The original 140-beam corner extends its surface back over the origin.
    const int64_t stamp = node->now().nanoseconds() + 400000000LL;
    setTime(stamp);
    for (const int64_t t : {stamp - int64_t{200000000}, stamp}) {
      auto scan = cornerScan(t, "perception_world", .8, t == stamp ? .8 : .84);
      scan.ranges = std::vector<float>(scan.ranges.begin() + 40, scan.ranges.begin() + 101);
      scan.angle_min += 40 * scan.angle_increment;
      publisher->publish(scan);
      ASSERT_TRUE(
        waitFor(
          [&]() {
            const auto latest = controller.getScanSnapshot();
            return latest && rclcpp::Time(latest->stamp).nanoseconds() == t;
          }));
    }
    ASSERT_EQ(controller.getScanSnapshot()->obstacles.size(), 1u);
    ASSERT_GT(controller.getScanSnapshot()->obstacles[0].partial_motion_rank, 0);
  }

  geometry_msgs::msg::TwistStamped decide(
    const geometry_msgs::msg::Twist & velocity = geometry_msgs::msg::Twist{})
  {
    testing::internal::CaptureStderr();
    try {
      const auto selected = command(velocity);
      diagnostics = testing::internal::GetCapturedStderr();
      return selected;
    } catch (...) {
      diagnostics = testing::internal::GetCapturedStderr();
      throw;
    }
  }

  void expectDiagnostic(const std::string & text)
  {
    EXPECT_NE(diagnostics.find(text), std::string::npos) << diagnostics;
  }

  double riskCost(const Velocity2D & candidate)
  {
    const auto snapshot = controller.getScanSnapshot();
    const auto trajectory = TrajectoryPredictor(1.0, 0.5).predict(Pose2D{}, candidate);
    DynamicRiskEvaluator evaluator(costmap->getRobotRadius(), 0.05, 1.0, 0.5, 0.35);
    const auto prepared = evaluator.prepare(
      {&trajectory}, snapshot.get(),
      "perception_world", node->now(), false);
    const auto result = evaluator.evaluate(trajectory, prepared);
    EXPECT_TRUE(result.input_available);
    return result.cost;
  }

  void expectFilteredWinner(const geometry_msgs::msg::TwistStamped & selected)
  {
    const MotionPredictor predictor({1.0, 2.0, 0.0, 1.0, 0.5});
    const TrajectoryScorer scorer(0.4, {0.5, 0.1, 0.2, 0.2, 0.0}, 1.0, 0.5, 1.0);
    std::vector<Pose2D> path(2);
    path.back().x = 1.0;
    double best = std::numeric_limits<double>::infinity();
    Velocity2D expected{};
    for (const auto & candidate : candidates()) {
      const auto trajectory = predictor.predict(Pose2D{}, {0, 0}, candidate, 1.0).trajectory;
      const auto backup = predictor.predict(Pose2D{}, {0, 0}, candidate, 0.5);
      if (backup.stop_time > 1.0) {continue;}
      DynamicTrajectoryEvaluator evaluator(costmap->getRobotRadius(), 0.05, 1.0, 0.5, 0.35);
      const auto snapshot = controller.getScanSnapshot();
      const auto dynamic = evaluator.evaluate(
        backup.trajectory,
        snapshot.get(), "perception_world", node->now());
      if (dynamic.collision) {continue;}
      const double cost = scorer.score(trajectory, path, 0.0).total_cost;
      if (cost < best || (cost == best && std::abs(candidate.omega) < std::abs(expected.omega))) {
        best = cost;
        expected = candidate;
      }
    }
    EXPECT_DOUBLE_EQ(selected.twist.linear.x, expected.v);
    EXPECT_DOUBLE_EQ(selected.twist.angular.z, expected.omega);
  }

  std::vector<Velocity2D> candidates()
  {
    SamplingConfig config;
    config.max_v = 0.4;
    config.max_w = 1.0;
    config.acc_v = 1.0;
    config.acc_w = 2.0;
    config.control_dt = 0.5;
    config.v_samples = 3;
    config.w_samples = 3;
    return sampleVelocities({0.0, 0.0}, config);
  }

  void setWall(unsigned char value)
  {
    auto * map = costmap->getCostmap();
    for (unsigned int x = 0; x < map->getSizeInCellsX(); ++x) {
      for (unsigned int y = 0; y < map->getSizeInCellsY(); ++y) {
        double wx, wy;
        map->mapToWorld(x, y, wx, wy);
        if (wx > 0.3) {
          map->setCost(x, y, value);
        }
      }
    }
  }

  std::string diagnostics;
};

TEST_F(ControllerDynamicTest, DebugSnapshotContainsExactLockedCostmapAndIsSilentAtInfo)
{
  configureDecision(false);
  auto * map = costmap->getCostmap();
  map->setCost(0, 0, 1);
  map->setCost(1, 0, nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
  map->setCost(2, 0, nav2_costmap_2d::NO_INFORMATION);
  (void)decide();
  expectDiagnostic("Costmap snapshot: {\"cycle\":1,");
  expectDiagnostic("\"origin\":[-3,-3]");
  expectDiagnostic("\"resolution\":0.050000000000000003");
  expectDiagnostic("\"width\":120,\"height\":120");
  const std::string bytes = "\"data_hex\":\"01fdff" +
    std::string(2 * (120 * 120 - 3), '0') + "\"";
  expectDiagnostic(bytes);

  ASSERT_EQ(
    rcutils_logging_set_logger_level(
      node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO), RCUTILS_RET_OK);
  (void)decide();
  EXPECT_EQ(diagnostics.find("Costmap snapshot:"), std::string::npos);
}

TEST_F(ControllerDynamicTest, DefaultDisabledKeepsOriginalCommand)
{
  configureDecision(false);
  const auto selected = decide();
  EXPECT_DOUBLE_EQ(selected.twist.linear.x, 0.4);
  EXPECT_DOUBLE_EQ(selected.twist.angular.z, 0.0);
  EXPECT_FALSE(controller.getScanSnapshot());

}

TEST_F(ControllerDynamicTest, MissingScanRejectsNormalCandidatesAndMarksZeroEmergency)
{
  configureDecision();
  const auto selected = decide();
  EXPECT_DOUBLE_EQ(selected.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(selected.twist.angular.z, 0.0);
  expectDiagnostic("\"evidence_state\":\"unavailable\"");
  expectDiagnostic("\"mode\":\"emergency\"");
  expectDiagnostic("\"accepted\":0");
  expectDiagnostic("\"fallback_dynamic_available\":false");
}

TEST_F(ControllerDynamicTest, CrossingEvidenceFiltersBeforeOriginalScoring)
{
  configureDecision();
  observe(0.66, 0.4, 0.0, -0.4);
  const auto selected = decide();
  expectFilteredWinner(selected);
  expectDiagnostic("\"known_input_available\":true");

  expectDiagnostic("\"accepted\"");
}

TEST_F(ControllerDynamicTest, KnownRiskFilteringDoesNotRequireHistoryState)
{
  configureDecision();
  observe(0.66, 0.4, 0.0, -0.4);
  const auto selected = decide();
  expectFilteredWinner(selected);
  expectDiagnostic("\"dynamic_rejected\":9");
}

TEST_F(ControllerDynamicTest, FarEvidenceKeepsOriginalScoring)
{
  configureDecision();
  observe(2.0, 2.0);
  const auto selected = decide();
  EXPECT_DOUBLE_EQ(selected.twist.linear.x, 0.4);
  EXPECT_DOUBLE_EQ(selected.twist.angular.z, 0.0);
  expectDiagnostic("\"retained_points\":0");
}

TEST_F(ControllerDynamicTest, AllKnownConflictsProduceBrakingAction)
{
  configureDecision();
  observe(0.18, 0.18);
  const auto selected = decide();
  for (const auto & candidate : candidates()) {
    EXPECT_GT(riskCost(candidate), 0.0);
  }
  expectFilteredWinner(selected);
  expectDiagnostic("\"mode\":\"emergency\"");
}

TEST_F(ControllerDynamicTest, EmptyAndUnknownScansSupplyNoPredictionPenalty)
{
  configureDecision();
  setTime(2400000000LL);
  publishGeometry(node->now().nanoseconds(), 1.0, 1.0, true);
  auto selected = decide();
  EXPECT_DOUBLE_EQ(selected.twist.linear.x, 0.4);
  expectDiagnostic("\"known\":0");
  observe(0.66, 0.4, 0.0, 0.0, false);
  selected = decide();
  EXPECT_DOUBLE_EQ(selected.twist.linear.x, 0.4);
  EXPECT_DOUBLE_EQ(selected.twist.angular.z, 0.0);
  expectDiagnostic("\"unknown\":1");
  EXPECT_EQ(controller.getScanSnapshot()->obstacles[0].points.size(), 141u);
}


TEST_F(ControllerDynamicTest, UnknownNearGeometryRejectsEvenWithFreeCostmap)
{
  configureDecision();
  observe(0.18, 0.18, 0.0, 0.0, false);
  const auto selected = decide();
  EXPECT_DOUBLE_EQ(selected.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(selected.twist.angular.z, 0.0);
  expectDiagnostic("\"evidence_state\":\"partial\"");
  expectDiagnostic("\"geometry_rejected\":9");
  expectDiagnostic("\"dynamic_rejected\":0");
  expectDiagnostic("\"fallback_geometry_collision\":true");
  expectDiagnostic("\"mode\":\"emergency\"");
}

TEST_F(ControllerDynamicTest, UnknownNominalGeometryIsCheckedBeyondClearBackup)
{
  configureDecision();
  // A narrow measured wall slice: the forward nominal reaches radius+margin,
  // while its stopping backup remains short of it. Costmap stays entirely free.
  const auto stamp = node->now().nanoseconds();
  auto scan = cornerScan(stamp, "perception_world");
  scan.angle_min = -0.01;
  scan.ranges = {static_cast<float>(0.55 / std::cos(-0.01)), 0.55F,
    static_cast<float>(0.55 / std::cos(0.01))};
  publisher->publish(scan);
  ASSERT_TRUE(waitFor([&]() {return static_cast<bool>(controller.getScanSnapshot());}));
  ASSERT_EQ(controller.getScanSnapshot()->obstacles.size(), 1u);
  ASSERT_FALSE(controller.getScanSnapshot()->obstacles[0].velocity_valid);
  const auto selected = decide();
  EXPECT_FALSE(selected.twist.linear.x == 0.4 && selected.twist.angular.z == 0.0);
  expectDiagnostic("\"geometry_rejected\":1");
  expectDiagnostic("\"fallback_geometry_collision\":false");
  expectDiagnostic("\"evidence_state\":\"partial\"");
}

TEST_F(ControllerDynamicTest, UnknownBrakingOccupancyMakesZeroFallbackEmergency)
{
  configureDecision();
  const auto stamp = node->now().nanoseconds();
  auto scan = cornerScan(stamp, "perception_world");
  scan.angle_min = -0.01;
  scan.ranges = {static_cast<float>(0.31 / std::cos(-0.01)), 0.31F,
    static_cast<float>(0.31 / std::cos(0.01))};
  publisher->publish(scan);
  ASSERT_TRUE(waitFor([&]() {return static_cast<bool>(controller.getScanSnapshot());}));
  geometry_msgs::msg::Twist current;
  current.linear.x = 0.4;
  const auto selected = decide(current);
  EXPECT_DOUBLE_EQ(selected.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(selected.twist.angular.z, 0.0);
  // The initial pose is clear (0.31 > 0.22+0.05), but the physical braking
  // sweep reaches the snapshot. A zero target cannot certify a collision-free stop.
  expectDiagnostic("\"fallback_static_clear\":true");
  expectDiagnostic("\"fallback_complete\":true");
  expectDiagnostic("\"fallback_geometry_collision\":true");
  expectDiagnostic("\"mode\":\"emergency\"");
}

TEST_F(ControllerDynamicTest, FarUnknownGeometryKeepsOriginalScoringAndPartialEvidence)
{
  configureDecision();
  observe(2.0, 2.0, 0.0, 0.0, false);
  const auto selected = decide();
  EXPECT_DOUBLE_EQ(selected.twist.linear.x, 0.4);
  EXPECT_DOUBLE_EQ(selected.twist.angular.z, 0.0);
  expectDiagnostic("\"geometry_rejected\":0");
  expectDiagnostic("\"evidence_state\":\"partial\"");
  expectDiagnostic("\"mode\":\"navigate\"");
}

TEST_F(ControllerDynamicTest, DetailedDiagnosticsDoNotChangeSelection)
{
  configureDecision();
  observe(0.66, 0.4, 0.0, -0.4);
  ASSERT_EQ(
    rcutils_logging_set_logger_level(
      node->get_logger().get_name(),
      RCUTILS_LOG_SEVERITY_INFO), RCUTILS_RET_OK);
  const auto without_detail = decide();
  ASSERT_EQ(
    rcutils_logging_set_logger_level(
      node->get_logger().get_name(),
      RCUTILS_LOG_SEVERITY_DEBUG), RCUTILS_RET_OK);
  const auto with_detail = decide();
  EXPECT_DOUBLE_EQ(with_detail.twist.linear.x, without_detail.twist.linear.x);
  EXPECT_DOUBLE_EQ(with_detail.twist.angular.z, without_detail.twist.angular.z);
  expectDiagnostic("Stoppable decision:");
  expectDiagnostic("Control timing:");
}

TEST_F(ControllerDynamicTest, StaleAndFutureScansRejectNormalCandidatesAndMarkZeroEmergency)
{
  configureDecision();
  observe(0.66, 0.4, 0.0, -0.4);
  const int64_t stamp = node->now().nanoseconds();
  for (int64_t time : {stamp + 350000001LL, stamp - 1LL}) {
    setTime(time);
    const auto selected = decide();
    EXPECT_DOUBLE_EQ(selected.twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(selected.twist.angular.z, 0.0);
    expectDiagnostic("\"mode\":\"emergency\"");
    expectDiagnostic("\"accepted\":0");
    expectDiagnostic(time > stamp ? "expired scan" : "future scan stamp");
    expectDiagnostic("\"known_input_available\":false");
  }
}

TEST_F(ControllerDynamicTest, CostmapHardCollisionCannotBeOverriddenByDynamicFilter)
{
  configureDecision();
  observe(0.66, 0.4, 0.0, -0.4);
  setWall(nav2_costmap_2d::LETHAL_OBSTACLE);
  const auto selected = decide();
  const auto trajectory = TrajectoryPredictor(1.0, 0.5).predict(
    Pose2D{}, {selected.twist.linear.x, selected.twist.angular.z});
  EXPECT_TRUE(evaluateTrajectory(trajectory, *costmap->getCostmap()).collision_free);
  EXPECT_LE(selected.twist.linear.x, 0.2);
}

TEST_F(ControllerDynamicTest, TerminalConflictBrakesInsteadOfRotatingThroughRisk)
{
  configureDecision();
  observe(0.18, 0.18);
  for (double yaw : {-1.0, 1.0}) {
    nav_msgs::msg::Path path;
    path.header.frame_id = "perception_world";
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
    pose.pose.position.x = 0.1;
    pose.pose.orientation.z = std::sin(yaw / 2.0);
    pose.pose.orientation.w = std::cos(yaw / 2.0);
    path.poses.push_back(pose);
    controller.setPlan(path);
    const auto selected = decide();
    EXPECT_DOUBLE_EQ(selected.twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(selected.twist.angular.z, 0.0);
    expectDiagnostic("\"mode\":\"emergency\"");
  }
}

TEST_F(ControllerDynamicTest, TerminalSlowingKeepsOriginalReachableSpeedPriority)
{
  ASSERT_TRUE(node->set_parameter({"controller_frequency", 20.0}).successful);
  configureDecision();
  observe(2.0, 2.0);
  nav_msgs::msg::Path path;
  path.header.frame_id = "perception_world";
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.position.x = 0.1;
  goal.pose.orientation.z = std::sin(0.5);
  goal.pose.orientation.w = std::cos(0.5);
  path.poses.push_back(goal);
  controller.setPlan(path);
  geometry_msgs::msg::PoseStamped robot;
  robot.header.frame_id = "perception_world";
  robot.header.stamp = node->now();
  robot.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist current;
  current.linear.x = 0.4;
  const auto selected = controller.computeVelocityCommands(robot, current, nullptr);
  // Preserve the original terminal ordering: minimum reachable v, then goal yaw.
  EXPECT_NEAR(selected.twist.linear.x, 0.35, 1e-12);
  EXPECT_NEAR(selected.twist.angular.z, 0.1, 1e-12);
}

TEST_F(ControllerDynamicTest, ExistingNoCandidateInputAndLifecycleExceptionsRemain)
{
  configureDecision();
  setTime(2400000000LL);
  publishGeometry(node->now().nanoseconds(), 1.0, 0.0, true);
  auto * map = costmap->getCostmap();
  const auto size = map->getSizeInCellsX() * map->getSizeInCellsY();
  std::fill_n(map->getCharMap(), size, nav2_costmap_2d::LETHAL_OBSTACLE);
  const auto braking = decide();
  EXPECT_DOUBLE_EQ(braking.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(braking.twist.angular.z, 0.0);
  expectDiagnostic("\"mode\":\"emergency\"");
  expectDiagnostic("\"fallback_static_clear\":false");
  std::fill_n(map->getCharMap(), size, nav2_costmap_2d::FREE_SPACE);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "perception_world";
  pose.pose.orientation.w = 1.0;
  geometry_msgs::msg::Twist unreachable;
  unreachable.linear.x = 2.0;
  const auto stop = controller.computeVelocityCommands(pose, unreachable, nullptr);
  EXPECT_DOUBLE_EQ(stop.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(stop.twist.angular.z, 0.0);
  pose.header.frame_id = "wrong_frame";
  EXPECT_THROW(
    controller.computeVelocityCommands(
      pose, geometry_msgs::msg::Twist{},
      nullptr), std::invalid_argument);
  controller.deactivate();
  EXPECT_THROW(command(), std::logic_error);
}

TEST_F(ControllerDynamicTest, ReusesActualNonDefaultPredictorTimeGrid)
{
  ASSERT_TRUE(node->set_parameter({"FollowPath.sim_time", 0.9}).successful);
  ASSERT_TRUE(node->set_parameter({"FollowPath.simulation_dt", 0.3}).successful);
  configureDecision();
  observe(2.0, 2.0);
  const auto selected = decide();
  EXPECT_DOUBLE_EQ(selected.twist.linear.x, 0.4);
  EXPECT_DOUBLE_EQ(selected.twist.angular.z, 0.0);
  expectDiagnostic("\"mode\":\"navigate\"");
}

TEST_F(ControllerDynamicTest, UsesCostmapRobotRadiusInsteadOfAnotherHardcodedRadius)
{
  costmap->setCircularRadius(0.1);
  configureDecision();
  observe(0.66, 0.4, 0.0, -0.4);
  const auto selected = decide();
  expectFilteredWinner(selected);
  expectDiagnostic("\"known_input_available\":true");
}

TEST_F(ControllerDynamicTest, StoppingAdmissibilityReplacesFixedSpeedStaticHorizon)
{
  configureDecision(false);
  setWall(254);
  const auto selected = decide();
  EXPECT_DOUBLE_EQ(selected.twist.linear.x, 0.4);
  const MotionPredictor model({1.0, 2.0, 0.0, 1.0, 0.5});
  const auto backup = model.predict(
    Pose2D{}, {0, 0},
    {selected.twist.linear.x, selected.twist.angular.z}, 0.5);
  EXPECT_TRUE(evaluateTrajectory(backup.trajectory, *costmap->getCostmap()).collision_free);
  const auto old = TrajectoryPredictor(1.0, 0.5).predict(
    Pose2D{},
    {selected.twist.linear.x, selected.twist.angular.z});
  EXPECT_FALSE(evaluateTrajectory(old, *costmap->getCostmap()).collision_free);
}

TEST_F(ControllerDynamicTest, NoStaticCandidateHasExplicitUnverifiedBrake)
{
  configureDecision(false);
  auto * map = costmap->getCostmap();
  for (unsigned int x = 0; x < map->getSizeInCellsX(); ++x) {
    for (unsigned int y = 0; y < map->getSizeInCellsY(); ++y) {
      map->setCost(x, y, 254);
    }
  }
  geometry_msgs::msg::TwistStamped selected;
  EXPECT_NO_THROW(selected = decide());
  EXPECT_DOUBLE_EQ(selected.twist.linear.x, 0.0);
  expectDiagnostic("\"mode\":\"emergency\"");
  expectDiagnostic("\"fallback_static_clear\":false");
}

TEST_F(ControllerDynamicTest, EmptyVelocityWindowUsesExplicitBrakingBranch)
{
  configureDecision(false);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "perception_world";
  pose.pose.orientation.w = 1;
  geometry_msgs::msg::Twist velocity;
  velocity.linear.x = 1.5;
  geometry_msgs::msg::TwistStamped selected;
  EXPECT_NO_THROW(selected = controller.computeVelocityCommands(pose, velocity, nullptr));
  EXPECT_DOUBLE_EQ(selected.twist.linear.x, 0.0);
}


TEST_F(ControllerDynamicTest, CrossingHoldKeepsCurrentCandidateSeparateFromOutput)
{
  node->declare_parameter("FollowPath.crossing_yield.enabled", true);
  configureDecision();
  observe(.8, .8, 0, -.2);
  const auto selected = decide();
  EXPECT_DOUBLE_EQ(selected.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(selected.twist.angular.z, 0.0);
  expectDiagnostic("\"triggered\":true");
  expectDiagnostic("\"hold\":true");
  expectDiagnostic("\"candidate\":[0.40000000000000002,0]");
  expectDiagnostic("\"output\":[0,0]");
  // Re-score this cycle against a newly blocked map while keeping the Scan.
  setWall(nav2_costmap_2d::LETHAL_OBSTACLE);
  (void)decide();
  expectDiagnostic("\"triggered\":false");
  expectDiagnostic("\"hold\":true");
  EXPECT_EQ(diagnostics.find("\"candidate\":[0.40000000000000002,0]"), std::string::npos);
}

TEST_F(ControllerDynamicTest, DisabledCrossingYieldPreservesCommandWithClosingEvidence)
{
  configureDecision();
  observe(.8, .8, 0, -.2);
  const auto selected = decide();
  EXPECT_GT(selected.twist.linear.x, 0.0);
  EXPECT_EQ(diagnostics.find("Crossing yield:"), std::string::npos);
}


TEST_F(ControllerDynamicTest, CrossingReleaseUsesNewSpeedLimitedCandidateFromSameCycle)
{
  node->declare_parameter("FollowPath.crossing_yield.enabled", true);
  configureDecision();
  observe(.8, .8, 0, -.2);
  EXPECT_DOUBLE_EQ(decide().twist.linear.x, 0.0);
  expectDiagnostic("\"hold\":true");
  expectDiagnostic("\"occupied\":true");
  controller.setSpeedLimit(.2, false);
  const int64_t first_clear_stamp = node->now().nanoseconds() + 200000000LL;
  for (int64_t stamp : {first_clear_stamp, first_clear_stamp + int64_t{200000000}}) {
    setTime(stamp);
    auto scan = cornerScan(stamp, "perception_world");
    scan.angle_min = 0;
    scan.angle_increment = 2 * std::acos(-1.) / 360;
    scan.ranges.assign(360, 5.f);
    publisher->publish(scan);
    ASSERT_TRUE(
      waitFor(
        [&]() {
          const auto latest = controller.getScanSnapshot();
          return latest && rclcpp::Time(latest->stamp).nanoseconds() == stamp;
        }));
    const auto selected = decide();
    if (stamp == first_clear_stamp) {
      EXPECT_DOUBLE_EQ(selected.twist.linear.x, 0.0);
      expectDiagnostic("\"hold\":true");
    } else {
      EXPECT_DOUBLE_EQ(selected.twist.linear.x, .2);
      expectDiagnostic("\"released\":true");
      expectDiagnostic("\"candidate\":[0.20000000000000001,0]");
      expectDiagnostic("\"output\":[0.20000000000000001,0]");
    }
  }
}

TEST_F(ControllerDynamicTest, CrossingEnabledWithoutIntentRetainsTerminalSlowing)
{
  node->declare_parameter("FollowPath.crossing_yield.enabled", true);
  ASSERT_TRUE(node->set_parameter({"controller_frequency", 20.0}).successful);
  configureDecision();
  observe(2.0, 2.0);
  nav_msgs::msg::Path path;
  path.header.frame_id = "perception_world";
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.position.x = .1;
  goal.pose.orientation.z = std::sin(.5);
  goal.pose.orientation.w = std::cos(.5);
  path.poses.push_back(goal);
  controller.setPlan(path);
  geometry_msgs::msg::Twist current;
  current.linear.x = .4;
  const auto selected = decide(current);
  EXPECT_NEAR(selected.twist.linear.x, .35, 1e-12);
  EXPECT_NEAR(selected.twist.angular.z, .1, 1e-12);
  expectDiagnostic("\"hold\":false");
}


TEST_F(ControllerDynamicTest, ReplanDetourAndReentryBothKeepWaiting)
{
  node->declare_parameter("FollowPath.crossing_yield.enabled", true);
  configureDecision();
  observeNarrowCrossing();
  ASSERT_DOUBLE_EQ(decide().twist.linear.x, 0.0);
  expectDiagnostic("\"hold\":true");

  nav_msgs::msg::Path path;
  path.header.frame_id = "perception_world";
  geometry_msgs::msg::PoseStamped p;
  p.pose.orientation.w = 1;
  path.poses.push_back(p);
  p.pose.position.y = 1;
  path.poses.push_back(p);
  controller.setPlan(path);  // The same fresh source Scan is reused.
  const auto detour = decide();
  expectDiagnostic("\"hold\":true");
  expectDiagnostic("\"released\":false");
  EXPECT_DOUBLE_EQ(detour.twist.linear.x, 0);
  EXPECT_DOUBLE_EQ(detour.twist.angular.z, 0);

  setPlan();
  const auto reentry = decide();
  EXPECT_DOUBLE_EQ(reentry.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(reentry.twist.angular.z, 0.0);
  expectDiagnostic("\"hold\":true");
  expectDiagnostic("\"triggered\":false");
}

TEST_F(ControllerDynamicTest, PendingRiskKeepsTerminalCommandsStopped)
{
  node->declare_parameter("FollowPath.crossing_yield.enabled", true);
  ASSERT_TRUE(node->set_parameter({"controller_frequency", 20.0}).successful);
  configureDecision();
  observeNarrowCrossing();
  EXPECT_DOUBLE_EQ(decide().twist.linear.x, 0.0);
  expectDiagnostic("\"hold\":true");
  nav_msgs::msg::Path path;
  path.header.frame_id = "perception_world";
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.position.x = .1;
  goal.pose.orientation.z = std::sin(.5);
  goal.pose.orientation.w = std::cos(.5);
  path.poses.push_back(goal);
  controller.setPlan(path);
  geometry_msgs::msg::Twist current;
  current.linear.x = .4;
  const auto selected = decide(current);
  EXPECT_DOUBLE_EQ(selected.twist.linear.x, 0);
  EXPECT_DOUBLE_EQ(selected.twist.angular.z, 0);
  expectDiagnostic("\"hold\":true");
  expectDiagnostic("\"released\":false");
  const auto rotating = decide();
  EXPECT_DOUBLE_EQ(rotating.twist.linear.x, 0);
  EXPECT_DOUBLE_EQ(rotating.twist.angular.z, 0);
  expectDiagnostic("\"hold\":true");
  expectDiagnostic("\"released\":false");
}


TEST_F(ControllerDynamicTest, MissingExactScanTfPausesFallbackUntilCoveredObservationsResume)
{
  node->declare_parameter("FollowPath.crossing_yield.enabled", true);
  configureDecision();
  observeNarrowCrossing();
  EXPECT_DOUBLE_EQ(decide().twist.linear.x, 0);
  expectDiagnostic("\"occupied\":false");
  const int64_t first_clear = node->now().nanoseconds() + 200000000;
  const auto clear_scan = [](int64_t stamp) {
      auto scan = cornerScan(stamp, "perception_world");
      scan.angle_min = 0;
      scan.angle_increment = 2 * std::acos(-1.) / 360;
      scan.ranges.assign(360, 5.f);
      return scan;
    };
  const auto publish_clear = [&](int64_t stamp) {
      setTime(stamp);
      publisher->publish(clear_scan(stamp));
      ASSERT_TRUE(
        waitFor(
          [&]() {
            const auto latest = controller.getScanSnapshot();
            return latest && rclcpp::Time(latest->stamp).nanoseconds() == stamp;
          }));
    };
  for (int i = 0; i <= 10; ++i) {
    publish_clear(first_clear + i * 200000000LL);
    EXPECT_DOUBLE_EQ(decide().twist.linear.x, 0);
    expectDiagnostic("\"hold\":true");
  }
  const int64_t missing_stamp = first_clear + 2200000000LL;
  setTime(missing_stamp);
  auto missing = clear_scan(missing_stamp);
  missing.header.frame_id = "missing_yield_laser";
  publisher->publish(missing);
  const auto selected_subscription = executor.selectSubscription();
  ASSERT_TRUE(selected_subscription);
  ScanTestExecutor::runSelected(selected_subscription);
  ASSERT_EQ(
    rclcpp::Time(controller.getScanSnapshot()->stamp).nanoseconds(),
    first_clear + 2000000000LL);
  (void)decide();  // Repeated snapshot adds no empty duration.
  setTime(missing_stamp + 400000000);
  (void)decide();  // Expired snapshot explicitly pauses the timer.
  expectDiagnostic("\"hold\":true");
  expectDiagnostic("\"released\":false");

  const int64_t resume = first_clear + 7400000000LL;
  publish_clear(resume);
  (void)decide();
  expectDiagnostic("\"hold\":true");
  for (int i = 1; i < 10; ++i) {
    publish_clear(resume + i * 200000000LL);
    (void)decide();
    expectDiagnostic("\"hold\":true");
  }
  publish_clear(resume + 2000000000LL);
  EXPECT_GT(decide().twist.linear.x, 0);
  expectDiagnostic("\"released\":true");
  expectDiagnostic("\"hold\":false");
}

TEST_F(ControllerDynamicTest, OrdinarySetPlanCannotUnlockLatchedRisk)
{
  node->declare_parameter("FollowPath.crossing_yield.enabled", true);
  ASSERT_TRUE(node->set_parameter({"controller_frequency", 20.0}).successful);
  configureDecision();observeNarrowCrossing();
  ASSERT_DOUBLE_EQ(decide().twist.linear.x, 0);
  nav_msgs::msg::Path detour;detour.header.frame_id = "perception_world";
  geometry_msgs::msg::PoseStamped p;p.pose.orientation.w = 1;
  detour.poses.push_back(p);p.pose.position.y = 1;detour.poses.push_back(p);
  controller.setPlan(detour);
  EXPECT_DOUBLE_EQ(decide().twist.linear.x, 0);
  setPlan();
  EXPECT_DOUBLE_EQ(decide().twist.linear.x, 0);
  EXPECT_DOUBLE_EQ(decide().twist.angular.z, 0);
  expectDiagnostic("\"hold\":true");
  expectDiagnostic("\"released\":false");
}

class ControllerMedianTest : public ControllerDynamicTest
{
protected:
  void configureMedian()
  {
    node->declare_parameter("FollowPath.crossing_yield.enabled", true);
    node->declare_parameter("odom_topic", "/v3_m_odom");
    odometry = publisher_node->create_publisher<nav_msgs::msg::Odometry>(
      "/v3_m_odom", rclcpp::SensorDataQoS());
    configureDecision();
    ASSERT_TRUE(waitFor([&]() {return odometry->get_subscription_count() == 1u;}));
    observeNarrowCrossing();
    EXPECT_DOUBLE_EQ(decide().twist.linear.x, 0);
  }

  void stationaryScan(int64_t stamp, bool raw_odom)
  {
    setTime(stamp);
    if (raw_odom) {
      for (const int64_t t : std::vector<int64_t>{stamp - 100000000LL, stamp}) {
        nav_msgs::msg::Odometry message;
        message.header.frame_id = "perception_world";
        message.child_frame_id = "perception_base";
        message.header.stamp = rclcpp::Time(t);
        odometry->publish(message);
      }
    }
    auto scan = cornerScan(stamp, "perception_world", .8, .8);
    scan.ranges = std::vector<float>(scan.ranges.begin() + 40, scan.ranges.begin() + 101);
    scan.angle_min += 40 * scan.angle_increment;
    publisher->publish(scan);
    ASSERT_TRUE(
      waitFor(
        [&]() {
          auto s = controller.getScanSnapshot();
          return s && rclcpp::Time(s->stamp).nanoseconds() == stamp;
        }));
    executor.spin_some();
  }

  void settleAndHandoff()
  {
    const auto start = node->now().nanoseconds();
    for (int k = 1; k <= 16; ++k) {
      stationaryScan(start + k * 200000000LL, true);
      (void)decide();
    }
    expectDiagnostic("\"global_dynamic_active\":false");
  }
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry;
};

TEST_F(ControllerMedianTest, ZeroPlanningVelocityWithoutRawOdomCannotPermitM)
{
  configureMedian();
  const auto start = node->now().nanoseconds();
  for (int k = 1; k <= 16; ++k) {
    stationaryScan(start + k * 200000000LL, false);
    EXPECT_DOUBLE_EQ(decide().twist.linear.x, 0);
  }
  expectDiagnostic("\"global_dynamic_active\":true");
}

TEST_F(ControllerMedianTest, MedianHandoffClearsYieldAndBypassesAllDynamicConsumers)
{
  configureMedian();
  settleAndHandoff();
  EXPECT_GT(decide().twist.linear.x, 0);
  EXPECT_EQ(diagnostics.find("Crossing yield:"), std::string::npos);
  // A fresh unknown cluster would block the original dynamic geometry filter.
  setTime(node->now().nanoseconds() + 200000000LL);
  publishGeometry(node->now().nanoseconds(), .18, .18);
  EXPECT_GT(decide().twist.linear.x, 0);
  expectDiagnostic("\"geometry_rejected\":0");
  expectDiagnostic("\"global_dynamic_active\":false");
  setPlan();  // A same-task replan must not restore the consumed old event.
  EXPECT_GT(decide().twist.linear.x, 0);
  auto * map = costmap->getCostmap();
  std::fill(
    map->getCharMap(), map->getCharMap() + 120 * 120,
    nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_DOUBLE_EQ(decide().twist.linear.x, 0);
  expectDiagnostic("\"fallback_static_clear\":false");
}

TEST_F(ControllerMedianTest, OnlyNewTrustedPathMotionRestoresDynamicProcessing)
{
  configureMedian();
  settleAndHandoff();
  (void)decide();
  expectDiagnostic("\"global_dynamic_active\":false");
  observe(.66, .4, 0, -.4);
  (void)decide();
  expectDiagnostic("\"global_dynamic_active\":true");
  expectDiagnostic("\"transition\":\"dynamic\"");
}

}  // namespace
}  // namespace my_dwa_controller
