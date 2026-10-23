#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "my_dwa_controller/scan_obstacle_tracker.hpp"

namespace my_dwa_controller
{
namespace
{
class ScanTrackerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("scan_tracker_test");
    tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tracker = std::make_unique<ScanObstacleTracker>(tf, ScanTrackerConfig{});
  }
  sensor_msgs::msg::LaserScan scan(double t, double y, double robot_x = 0.0)
  {
    geometry_msgs::msg::TransformStamped tr;
    tr.header.frame_id = "odom";
    tr.child_frame_id = "laser";
    tr.header.stamp = rclcpp::Time(static_cast<int64_t>(t * 1e9));
    tr.transform.translation.x = robot_x;
    tr.transform.rotation.w = 1.0;
    tf->setTransform(tr, "test");
    sensor_msgs::msg::LaserScan s;
    s.header.frame_id = "laser";
    s.header.stamp = tr.header.stamp;
    s.angle_min = -1.5;
    s.angle_increment = 0.005;
    s.range_min = 0.1;
    s.range_max = 4.0;
    s.ranges.resize(600, std::numeric_limits<float>::infinity());
    // Ray/circle intersection, radius .18, center (1.6,y) in fixed odom.
    const double x = 1.6 - robot_x;
    for (size_t i = 0; i < s.ranges.size(); ++i) {
      const double a = s.angle_min + i * s.angle_increment;
      const double dot = x * std::cos(a) + y * std::sin(a);
      const double d = dot * dot - (x * x + y * y - 0.18 * 0.18);
      if (d >= 0 && dot > 0) {s.ranges[i] = dot - std::sqrt(d);}
    }
    return s;
  }
  ScanTrackSnapshot feed(double t, double y, double robot_x = 0.0)
  {
    tracker->enqueue(scan(t, y, robot_x));
    return tracker->snapshot("odom", t);
  }
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf;
  std::unique_ptr<ScanObstacleTracker> tracker;
};

TEST_F(ScanTrackerTest, CrossingUsesStampedFixedFrameAndFitsVelocity)
{
  ScanTrackSnapshot s;
  for (int i = 0; i < 9; ++i) {
    s = feed(1.0 + .2 * i, -.8 + .05 * i, .04 * i);
  }
  ASSERT_EQ(s.tracks.size(), 1u);
  EXPECT_TRUE(s.fresh);
  EXPECT_GT(s.tracks[0].confidence, .65);
  EXPECT_NEAR(s.tracks[0].vx, 0.0, .07);
  EXPECT_NEAR(s.tracks[0].vy, .25, .07);
  EXPECT_NEAR(s.stamp, 2.6, 1e-8);
}

TEST_F(ScanTrackerTest, MovingRobotDoesNotMakeStaticObjectMove)
{
  ScanTrackSnapshot s;
  for (int i = 0; i < 9; ++i) {
    s = feed(1.0 + .2 * i, -.5, .04 * i);
  }
  ASSERT_EQ(s.tracks.size(), 1u);
  EXPECT_LT(std::hypot(s.tracks[0].vx, s.tracks[0].vy), .06);
}

TEST_F(ScanTrackerTest, MissingObservationCoastsAndNeverConfirmsClear)
{
  for (int i = 0; i < 8; ++i) {
    feed(1.0 + .2 * i, -.8 + .05 * i);
  }
  auto empty = scan(2.6, 0);
  empty.ranges.assign(600, std::numeric_limits<float>::infinity());
  tracker->enqueue(empty);
  auto s = tracker->snapshot("odom", 2.6);
  ASSERT_EQ(s.tracks.size(), 1u);
  EXPECT_FALSE(s.tracks[0].observed);
  EXPECT_TRUE(s.unresolved);
  EXPECT_GT(s.tracks[0].vy, .15);
  s = tracker->snapshot("odom", 4.0);
  EXPECT_FALSE(s.fresh);
  EXPECT_TRUE(s.unresolved);
}

TEST_F(ScanTrackerTest, BadTfAndOldMessagesDoNotBecomeFreshObservations)
{
  feed(1.0, -.5);
  auto s = scan(1.2, -.5);
  s.header.frame_id = "missing_laser";
  tracker->enqueue(s);
  auto result = tracker->snapshot("odom", 1.5);
  EXPECT_FALSE(result.fresh);
  EXPECT_GT(result.tf_failures, 0u);
  tracker->enqueue(scan(.8, -.5));
  result = tracker->snapshot("odom", 1.6);
  EXPECT_DOUBLE_EQ(result.stamp, 1.0);
}

TEST_F(ScanTrackerTest, InvalidRangesAndLongWallsAreRejected)
{
  auto s = scan(1, 0);
  s.ranges.assign(600, std::numeric_limits<float>::quiet_NaN());
  for (size_t i = 100; i < 500; ++i) {
    const double a = s.angle_min + i * s.angle_increment;
    s.ranges[i] = 1.6 / std::cos(a);
  }
  tracker->enqueue(s);
  EXPECT_TRUE(tracker->snapshot("odom", 1).tracks.empty());
}

TEST_F(ScanTrackerTest, StopResetsPerceptionAndReactivationRequiresNewScan)
{
  feed(1, -.5);
  tracker->reset();
  const auto s = tracker->snapshot("odom", 1);
  EXPECT_FALSE(s.fresh);
  EXPECT_TRUE(s.tracks.empty());
}

TEST_F(ScanTrackerTest, ReacquiredMovingTrackRemainsUnresolvedUntilFitIsReliable)
{
  for (int i = 0; i < 8; ++i) {
    feed(1.0 + .2 * i, -.8 + .05 * i);
  }
  const auto s = feed(2.8, -.35);  // .4 s gap resets fit, not known motion history.
  ASSERT_EQ(s.tracks.size(), 1u);
  EXPECT_TRUE(s.tracks[0].observed);
  EXPECT_LT(s.tracks[0].confidence, .65);
  EXPECT_TRUE(s.unresolved);
}

TEST_F(ScanTrackerTest, ChangingVisibleSurfaceIsUnknownDespiteSmallRegressionResidual)
{
  ScanTrackSnapshot result;
  for (int k = 0; k < 7; ++k) {
    auto s = scan(1 + .2 * k, 0);
    // Fixed wall x=1.6, whose visible upper edge moves while the lower edge stays.
    s.ranges.assign(600, INFINITY);
    for (size_t i = 0; i < s.ranges.size(); ++i) {
      const double a = s.angle_min + i * s.angle_increment;
      const double y = 1.6 * std::tan(a);
      if (y >= -.25 && y <= .35 - .05 * k) {s.ranges[i] = 1.6 / std::cos(a);}
    }
    tracker->enqueue(s);
    result = tracker->snapshot("odom", 1 + .2 * k);
  }
  ASSERT_EQ(result.tracks.size(), 1u);
  EXPECT_LT(result.tracks[0].residual, .02);
  EXPECT_EQ(result.tracks[0].motion, ScanMotion::Unknown);
  EXPECT_GT(result.tracks[0].support_change, .1);
}

TEST_F(ScanTrackerTest, ExpiredOrdinaryClusterDoesNotOwnPermanentRisk)
{
  for (int i = 0; i < 8; ++i) {
    feed(1 + .2 * i, -.8 + .05 * i);
  }
  auto empty = scan(3.4, 0);
  empty.ranges.assign(600, INFINITY);
  tracker->enqueue(empty);
  const auto s = tracker->snapshot("odom", 3.4);
  EXPECT_TRUE(s.fresh);
  EXPECT_TRUE(s.tracks.empty());
  EXPECT_FALSE(s.unresolved);  // Monitor owns the old risk area, not this cluster.
}
}  // namespace
}  // namespace my_dwa_controller
