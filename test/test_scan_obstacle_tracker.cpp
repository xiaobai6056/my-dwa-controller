#include <gtest/gtest.h>
#include "scan_fixture.hpp"

#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>
#include <limits>
#include <memory>
#include <vector>

#include "my_dwa_controller/scan_obstacle_tracker.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace my_dwa_controller
{
namespace
{
constexpr float kInvalid = std::numeric_limits<float>::infinity();
constexpr double kTolerance = 1e-6;

sensor_msgs::msg::LaserScan scanAt(
  const std::vector<float> & ranges, int64_t stamp = 1000000000LL,
  const std::string & frame = "scan_world")
{
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = frame;
  scan.header.stamp = rclcpp::Time(stamp);
  scan.angle_min = 0.0;
  scan.angle_increment = 0.01;
  scan.range_min = 0.12;
  scan.range_max = 3.5;
  scan.ranges = ranges;
  return scan;
}

class ScanTrackerTest : public ::testing::Test
{
protected:
  std::shared_ptr<tf2_ros::Buffer> tf = std::make_shared<tf2_ros::Buffer>(
    std::make_shared<rclcpp::Clock>(RCL_ROS_TIME));

  void transform(int64_t stamp, double x, double y, double yaw = 0.0)
  {
    geometry_msgs::msg::TransformStamped value;
    value.header.frame_id = "scan_world";
    value.child_frame_id = "laser";
    value.header.stamp = rclcpp::Time(stamp);
    value.transform.translation.x = x;
    value.transform.translation.y = y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    value.transform.rotation = tf2::toMsg(q);
    ASSERT_TRUE(tf->setTransform(value, "unit_test"));
  }
};

TEST_F(ScanTrackerTest, DelayedTfPreservesBaselineAndProcessesConsecutiveScansInOrder)
{
  ScanObstacleTracker tracker(tf);
  transform(1000000000LL, 0.0, 0.0);
  tracker.update(cornerScan(1000000000LL, "laser"), "scan_world");
  const auto baseline = tracker.snapshot();
  tracker.enqueue(
    std::make_shared<sensor_msgs::msg::LaserScan>(
      cornerScan(1200000000LL, "laser")));
  tracker.enqueue(
    std::make_shared<sensor_msgs::msg::LaserScan>(
      cornerScan(1400000000LL, "laser")));
  tracker.processPending("scan_world", rclcpp::Time(1400000000LL, RCL_ROS_TIME));
  EXPECT_EQ(tracker.snapshot(), baseline);
  transform(1400000000LL, 0.2, -0.08);
  tracker.processPending("scan_world", rclcpp::Time(1400000000LL, RCL_ROS_TIME));
  ASSERT_TRUE(tracker.snapshot());
  EXPECT_EQ(tracker.snapshot()->stamp.nanosec, 400000000u);
  const auto & obstacle = tracker.snapshot()->obstacles.at(0);
  EXPECT_TRUE(obstacle.velocity_valid);
  EXPECT_NEAR(obstacle.vx, 0.5, kTolerance);
  EXPECT_NEAR(obstacle.vy, -0.2, kTolerance);
}

TEST_F(ScanTrackerTest, MissingEarlierFrameBlocksReadyLaterFrameUntilExpiry)
{
  ScanObstacleTracker tracker(tf);
  tracker.enqueue(
    std::make_shared<sensor_msgs::msg::LaserScan>(
      scanAt({1, 1, 1}, 1000000000LL, "missing_laser")));
  tracker.enqueue(
    std::make_shared<sensor_msgs::msg::LaserScan>(
      scanAt({1, 1, 1}, 1200000000LL)));
  tracker.processPending("scan_world", rclcpp::Time(1200000000LL, RCL_ROS_TIME));
  EXPECT_FALSE(tracker.snapshot());
  tracker.processPending("scan_world", rclcpp::Time(1360000000LL, RCL_ROS_TIME));
  ASSERT_TRUE(tracker.snapshot());
  EXPECT_EQ(tracker.snapshot()->stamp.nanosec, 200000000u);
  EXPECT_FALSE(tracker.snapshot()->obstacles.at(0).velocity_valid);
}

TEST_F(ScanTrackerTest, QueueIsBoundedAndOverflowDropsOldestPendingScans)
{
  ScanObstacleTracker tracker(tf);
  for (int i = 0; i < 8; ++i) {
    tracker.enqueue(
      std::make_shared<sensor_msgs::msg::LaserScan>(
        cornerScan(1000000000LL + i * 10000000LL, "laser")));
  }
  transform(1000000000LL, 0.0, 0.0);
  transform(1020000000LL, 0.0, 0.0);
  tracker.processPending("scan_world", rclcpp::Time(1070000000LL, RCL_ROS_TIME));
  EXPECT_FALSE(tracker.snapshot());  // First three have been evicted from the five-slot queue.
  transform(1030000000LL, 0.0, 0.0);
  tracker.processPending("scan_world", rclcpp::Time(1070000000LL, RCL_ROS_TIME));
  ASSERT_TRUE(tracker.snapshot());
  EXPECT_EQ(tracker.snapshot()->stamp.nanosec, 30000000u);
  EXPECT_FALSE(tracker.snapshot()->obstacles.at(0).velocity_valid);
  transform(1070000000LL, 0.04, 0.0);
  tracker.processPending("scan_world", rclcpp::Time(1070000000LL, RCL_ROS_TIME));
  EXPECT_EQ(tracker.snapshot()->stamp.nanosec, 70000000u);
  EXPECT_TRUE(tracker.snapshot()->obstacles.at(0).velocity_valid);
}

TEST_F(ScanTrackerTest, ExpiredPendingScanDoesNotBecomeFreshWhenTfArrives)
{
  ScanObstacleTracker tracker(tf);
  tracker.enqueue(
    std::make_shared<sensor_msgs::msg::LaserScan>(
      scanAt({1, 1, 1}, 1000000000LL, "laser")));
  tracker.processPending("scan_world", rclcpp::Time(1360000000LL, RCL_ROS_TIME));
  transform(1000000000LL, 0.0, 0.0);
  tracker.processPending("scan_world", rclcpp::Time(1360000000LL, RCL_ROS_TIME));
  EXPECT_FALSE(tracker.snapshot());
}

TEST_F(ScanTrackerTest, PausedRosClockDoesNotExpireScansByWallTime)
{
  ScanObstacleTracker tracker(tf);
  tracker.enqueue(
    std::make_shared<sensor_msgs::msg::LaserScan>(
      scanAt({1, 1, 1}, 1000000000LL, "laser")));
  tracker.processPending("scan_world", rclcpp::Time(1000000000LL, RCL_ROS_TIME));
  std::this_thread::sleep_for(std::chrono::milliseconds(370));
  transform(1000000000LL, 0.0, 0.0);
  tracker.processPending("scan_world", rclcpp::Time(1000000000LL, RCL_ROS_TIME));
  ASSERT_TRUE(tracker.snapshot());
  EXPECT_EQ(tracker.snapshot()->stamp.sec, 1);
  EXPECT_FALSE(tracker.snapshot()->obstacles.at(0).velocity_valid);
}

TEST_F(ScanTrackerTest, LateArrivalCannotEraseNewerAssociation)
{
  ScanObstacleTracker tracker(tf);
  tracker.update(cornerScan(1200000000LL), "scan_world");
  const auto baseline = tracker.snapshot();
  tracker.enqueue(
    std::make_shared<sensor_msgs::msg::LaserScan>(
      cornerScan(1000000000LL)));
  tracker.processPending("scan_world", rclcpp::Time(1200000000LL, RCL_ROS_TIME));
  EXPECT_EQ(tracker.snapshot(), baseline);
  tracker.enqueue(
    std::make_shared<sensor_msgs::msg::LaserScan>(
      cornerScan(1400000000LL)));
  tracker.processPending("scan_world", rclcpp::Time(1400000000LL, RCL_ROS_TIME));
  EXPECT_TRUE(tracker.snapshot()->obstacles.at(0).velocity_valid);
}

TEST_F(ScanTrackerTest, QueuedInvalidFrameAndZeroStampCannotUseLatestTf)
{
  transform(2000000000LL, 0.1, 0.0);
  ScanObstacleTracker tracker(tf);
  for (const auto & frame : {std::string("/laser"), std::string("")}) {
    tracker.enqueue(
      std::make_shared<sensor_msgs::msg::LaserScan>(
        scanAt({1, 1, 1}, 1000000000LL, frame)));
    tracker.processPending("scan_world", rclcpp::Time(1000000000LL, RCL_ROS_TIME));
    EXPECT_FALSE(tracker.snapshot());
  }
  tracker.enqueue(std::make_shared<sensor_msgs::msg::LaserScan>(scanAt({1, 1, 1}, 0, "laser")));
  tracker.processPending("scan_world", rclcpp::Time(2000000000LL, RCL_ROS_TIME));
  EXPECT_FALSE(tracker.snapshot());
  tracker.enqueue(std::make_shared<sensor_msgs::msg::LaserScan>(scanAt({1, 1, 1})));
  tracker.processPending("/scan_world", rclcpp::Time(1000000000LL, RCL_ROS_TIME));
  EXPECT_FALSE(tracker.snapshot());
}

TEST_F(ScanTrackerTest, RosClockRewindRestartsAssociationWithoutAcceptingLateMessages)
{
  ScanObstacleTracker tracker(tf);
  for (int64_t stamp : {1000000000LL, 1200000000LL}) {
    tracker.enqueue(std::make_shared<sensor_msgs::msg::LaserScan>(cornerScan(stamp)));
    tracker.processPending("scan_world", rclcpp::Time(stamp, RCL_ROS_TIME));
  }
  ASSERT_TRUE(tracker.snapshot()->obstacles.at(0).velocity_valid);
  tracker.processPending("scan_world", rclcpp::Time(800000000LL, RCL_ROS_TIME));
  EXPECT_FALSE(tracker.snapshot());
  for (int64_t stamp : {800000000LL, 950000000LL}) {
    tracker.enqueue(std::make_shared<sensor_msgs::msg::LaserScan>(cornerScan(stamp)));
    tracker.processPending("scan_world", rclcpp::Time(stamp, RCL_ROS_TIME));
    ASSERT_TRUE(tracker.snapshot());
    EXPECT_EQ(rclcpp::Time(tracker.snapshot()->stamp).nanoseconds(), stamp);
    EXPECT_EQ(tracker.snapshot()->obstacles.at(0).velocity_valid, stamp == 950000000LL);
  }
}

TEST_F(ScanTrackerTest, FutureScanWaitsForRosTimeWithoutReplacingPreviousAssociation)
{
  ScanObstacleTracker tracker(tf);
  tracker.update(cornerScan(1000000000LL), "scan_world");
  const auto previous = tracker.snapshot();
  tracker.enqueue(
    std::make_shared<sensor_msgs::msg::LaserScan>(
      cornerScan(1201000000LL)));
  tracker.processPending("scan_world", rclcpp::Time(1200000000LL, RCL_ROS_TIME));
  EXPECT_EQ(tracker.snapshot(), previous);
  tracker.processPending("scan_world", rclcpp::Time(1300000000LL, RCL_ROS_TIME));
  ASSERT_TRUE(tracker.snapshot());
  EXPECT_EQ(rclcpp::Time(tracker.snapshot()->stamp).nanoseconds(), 1201000000LL);
  EXPECT_TRUE(tracker.snapshot()->obstacles.at(0).velocity_valid);
}

TEST_F(ScanTrackerTest, SeparatesDistantClustersAndFiltersSmallOnes)
{
  ScanObstacleTracker tracker(tf);
  tracker.update(scanAt({1, 1, 1, 2, 2, 2, kInvalid, 3}), "scan_world");
  ASSERT_TRUE(tracker.snapshot());
  const auto & obstacles = tracker.snapshot()->obstacles;
  ASSERT_EQ(obstacles.size(), 2u);
  EXPECT_LT(obstacles[0].aabb.max_x, obstacles[1].aabb.min_x);
  EXPECT_FALSE(obstacles[0].velocity_valid);
  EXPECT_EQ(tracker.snapshot()->frame_id, "scan_world");
  EXPECT_EQ(tracker.snapshot()->stamp.sec, 1);
}

TEST_F(ScanTrackerTest, RetainedPointsStayInTheirOriginalClustersAcrossInvalidBeams)
{
  ScanObstacleTracker tracker(tf);
  tracker.update(scanAt({1, 1, 1, kInvalid, 1, 1, 1}), "scan_world");
  const auto snapshot = tracker.snapshot();
  ASSERT_TRUE(snapshot);
  ASSERT_EQ(snapshot->obstacles.size(), 2u);
  const auto & first = snapshot->obstacles[0].points;
  const auto & second = snapshot->obstacles[1].points;
  ASSERT_EQ(first.size(), 3u);
  ASSERT_EQ(second.size(), 3u);
  EXPECT_NEAR(first[0][0], 1.0, kTolerance);
  EXPECT_NEAR(first[2][1], std::sin(0.02), kTolerance);
  EXPECT_NEAR(second[0][1], std::sin(0.04), kTolerance);
  EXPECT_NEAR(second[2][1], std::sin(0.06), kTolerance);
}

TEST_F(ScanTrackerTest, InvalidRangesSplitOtherwiseClosePoints)
{
  ScanObstacleTracker tracker(tf);
  for (float invalid : {kInvalid, std::numeric_limits<float>::quiet_NaN(), 0.0f, 4.0f}) {
    tracker.update(scanAt({1, 1, invalid, 1, 1}), "scan_world");
    ASSERT_TRUE(tracker.snapshot());
    EXPECT_TRUE(tracker.snapshot()->obstacles.empty());
    tracker.reset();
  }
}

TEST_F(ScanTrackerTest, TransformsPointsBeforeComputingAxisAlignedBounds)
{
  const double pi = std::acos(-1.0);
  transform(1000000000LL, 4.0, -2.0, pi / 2.0);
  ScanObstacleTracker tracker(tf, 10.0);
  auto scan = scanAt({1, 2, 3}, 1000000000LL, "laser");
  scan.angle_increment = pi / 2.0;
  tracker.update(scan, "scan_world");
  ASSERT_TRUE(tracker.snapshot());
  ASSERT_EQ(tracker.snapshot()->obstacles.size(), 1u);
  const auto & box = tracker.snapshot()->obstacles[0].aabb;
  EXPECT_NEAR(box.min_x, 2.0, kTolerance);
  EXPECT_NEAR(box.max_x, 4.0, kTolerance);
  EXPECT_NEAR(box.min_y, -5.0, kTolerance);
  EXPECT_NEAR(box.max_y, -1.0, kTolerance);
}

TEST_F(ScanTrackerTest, RetainsOrderedBeamPointsInSnapshotFrameAtExactScanStamp)
{
  const double pi = std::acos(-1.0);
  transform(1000000000LL, 4.0, -2.0, pi / 2.0);
  transform(1200000000LL, 8.0, 9.0, pi / 2.0);
  ScanObstacleTracker tracker(tf, 10.0);
  auto scan = scanAt({1, 2, 3}, 1000000000LL, "laser");
  scan.angle_increment = pi / 2.0;
  tracker.update(scan, "scan_world");
  const auto snapshot = tracker.snapshot();
  ASSERT_TRUE(snapshot);
  EXPECT_EQ(snapshot->frame_id, "scan_world");
  EXPECT_EQ(snapshot->stamp, scan.header.stamp);
  ASSERT_EQ(snapshot->obstacles.size(), 1u);
  const auto & points = snapshot->obstacles[0].points;
  ASSERT_EQ(points.size(), 3u);
  EXPECT_NEAR(points[0][0], 4.0, kTolerance);
  EXPECT_NEAR(points[0][1], -1.0, kTolerance);
  EXPECT_NEAR(points[1][0], 2.0, kTolerance);
  EXPECT_NEAR(points[1][1], -2.0, kTolerance);
  EXPECT_NEAR(points[2][0], 4.0, kTolerance);
  EXPECT_NEAR(points[2][1], -5.0, kTolerance);
}

TEST_F(ScanTrackerTest, UsesExactScanTimeAndEstimatesBothVelocityComponents)
{
  transform(1000000000LL, 0.0, 0.0);
  transform(1200000000LL, 0.1, -0.04);
  transform(1400000000LL, 0.2, -0.08);
  ScanObstacleTracker tracker(tf);
  tracker.update(cornerScan(1000000000LL, "laser"), "scan_world");
  ASSERT_TRUE(tracker.snapshot());
  EXPECT_NEAR(tracker.snapshot()->obstacles[0].aabb.max_x, 1.0, kTolerance);
  tracker.update(cornerScan(1200000000LL, "laser"), "scan_world");
  ASSERT_TRUE(tracker.snapshot());
  ASSERT_EQ(tracker.snapshot()->obstacles.size(), 1u);
  const auto & obstacle = tracker.snapshot()->obstacles[0];
  EXPECT_TRUE(obstacle.velocity_valid);
  EXPECT_NEAR(obstacle.vx, 0.5, kTolerance);
  EXPECT_NEAR(obstacle.vy, -0.2, kTolerance);
}

TEST_F(ScanTrackerTest, SensorTranslationDoesNotBecomeObstacleVelocity)
{
  transform(1000000000LL, 0.0, 0.0);
  transform(1200000000LL, 0.2, 0.0);
  ScanObstacleTracker tracker(tf, 0.12, 1);
  tracker.update(scanAt({2}, 1000000000LL, "laser"), "scan_world");
  tracker.update(scanAt({1.8}, 1200000000LL, "laser"), "scan_world");
  ASSERT_TRUE(tracker.snapshot());
  const auto & obstacle = tracker.snapshot()->obstacles[0];
  EXPECT_FALSE(obstacle.velocity_valid);
  EXPECT_NEAR(obstacle.aabb.min_x, 2.0, kTolerance);
  EXPECT_NEAR(obstacle.vx, 0.0, kTolerance);
  EXPECT_NEAR(obstacle.vy, 0.0, kTolerance);
}

TEST_F(ScanTrackerTest, NewUnmatchedClustersHaveNoInheritedVelocity)
{
  ScanObstacleTracker tracker(tf);
  tracker.update(scanAt({1, 1, 1}), "scan_world");
  tracker.update(scanAt({3, 3, 3}, 1200000000LL), "scan_world");
  ASSERT_TRUE(tracker.snapshot());
  ASSERT_EQ(tracker.snapshot()->obstacles.size(), 1u);
  const auto & obstacle = tracker.snapshot()->obstacles[0];
  EXPECT_FALSE(obstacle.velocity_valid);
  EXPECT_TRUE(std::isfinite(obstacle.vx));
  EXPECT_TRUE(std::isfinite(obstacle.vy));
}

TEST_F(ScanTrackerTest, PreviousClusterCanOnlySupportOneCurrentVelocity)
{
  ScanObstacleTracker tracker(tf);
  tracker.update(uScan(1000000000LL, 0.6), "scan_world");
  auto current = uScan(1200000000LL, 0.6);
  current.ranges[110] = kInvalid;
  tracker.update(current, "scan_world");
  const auto snapshot = tracker.snapshot();
  ASSERT_TRUE(snapshot);
  ASSERT_EQ(snapshot->obstacles.size(), 2u);
  EXPECT_TRUE(snapshot->obstacles[0].velocity_valid);
  EXPECT_FALSE(snapshot->obstacles[1].velocity_valid);
}

TEST_F(ScanTrackerTest, ChoosesNearestUnusedClusterWhenScanOrderReverses)
{
  transform(1000000000LL, 0.0, 0.0);
  transform(1200000000LL, 0.02, 0.01);
  auto previous = uScan(1000000000LL, 0.3);
  previous.header.frame_id = "laser";
  previous.ranges[110] = kInvalid;
  auto current = previous;
  current.header.stamp = rclcpp::Time(1200000000LL);
  current.angle_min = previous.angle_min + 220 * previous.angle_increment;
  current.angle_increment = -previous.angle_increment;
  std::reverse(current.ranges.begin(), current.ranges.end());
  ScanObstacleTracker tracker(tf);
  tracker.update(previous, "scan_world");
  tracker.update(current, "scan_world");
  const auto snapshot = tracker.snapshot();
  ASSERT_TRUE(snapshot);
  ASSERT_EQ(snapshot->obstacles.size(), 2u);
  for (const auto & obstacle : snapshot->obstacles) {
    ASSERT_TRUE(obstacle.velocity_valid);
    EXPECT_NEAR(obstacle.vx, 0.1, 1e-3);
    EXPECT_NEAR(obstacle.vy, 0.05, 1e-3);
  }
}

TEST_F(ScanTrackerTest, ChangingTargetFrameDoesNotReusePreviousCoordinates)
{
  geometry_msgs::msg::TransformStamped value;
  value.header.frame_id = "another_world";
  value.child_frame_id = "scan_world";
  value.transform.rotation.w = 1.0;
  ASSERT_TRUE(tf->setTransform(value, "unit_test", true));
  ScanObstacleTracker tracker(tf);
  tracker.update(scanAt({1, 1, 1}), "scan_world");
  tracker.update(scanAt({1, 1, 1}, 1200000000LL), "another_world");
  ASSERT_TRUE(tracker.snapshot());
  ASSERT_EQ(tracker.snapshot()->obstacles.size(), 1u);
  EXPECT_EQ(tracker.snapshot()->frame_id, "another_world");
  EXPECT_FALSE(tracker.snapshot()->obstacles[0].velocity_valid);
}

TEST_F(ScanTrackerTest, LongGapKeepsCurrentGeometryAsNewVelocityBaseline)
{
  ScanObstacleTracker tracker(tf, 0.12, 1);
  tracker.update(cornerScan(), "scan_world");
  tracker.update(cornerScan(1500000001LL, "scan_world", 1.1, 1.0), "scan_world");
  ASSERT_TRUE(tracker.snapshot());
  ASSERT_EQ(tracker.snapshot()->obstacles.size(), 1u);
  const auto & current = tracker.snapshot()->obstacles[0];
  EXPECT_FALSE(current.velocity_valid);
  EXPECT_DOUBLE_EQ(current.vx, 0.0);
  EXPECT_DOUBLE_EQ(current.vy, 0.0);
  EXPECT_NEAR(current.aabb.max_x, 1.1, kTolerance);
  EXPECT_EQ(tracker.snapshot()->stamp.nanosec, 500000001u);

  tracker.update(cornerScan(1700000001LL, "scan_world", 1.2, 1.0), "scan_world");
  ASSERT_TRUE(tracker.snapshot());
  ASSERT_EQ(tracker.snapshot()->obstacles.size(), 1u);
  const auto & next = tracker.snapshot()->obstacles[0];
  EXPECT_TRUE(next.velocity_valid);
  EXPECT_NEAR(next.vx, 0.5, kTolerance);
  EXPECT_NEAR(next.vy, 0.0, kTolerance);
}

TEST_F(ScanTrackerTest, AssociationAcceptsIntervalsThroughHalfASecond)
{
  for (int64_t interval : {499999999LL, 500000000LL}) {
    ScanObstacleTracker tracker(tf, 0.12, 1);
    tracker.update(cornerScan(), "scan_world");
    tracker.update(cornerScan(1000000000LL + interval, "scan_world", 1.1, 1.0), "scan_world");
    ASSERT_TRUE(tracker.snapshot());
    ASSERT_EQ(tracker.snapshot()->obstacles.size(), 1u);
    const auto & current = tracker.snapshot()->obstacles[0];
    EXPECT_TRUE(current.velocity_valid);
    EXPECT_NEAR(current.vx, 0.2, kTolerance);
    EXPECT_NEAR(current.vy, 0.0, kTolerance);
  }
}

TEST_F(ScanTrackerTest, SuccessfulEmptyFrameIsPresentAndEndsAssociation)
{
  ScanObstacleTracker tracker(tf);
  tracker.update(scanAt({1, 1, 1}), "scan_world");
  const auto scan = scanAt({kInvalid}, 1200000000LL);
  tracker.update(scan, "scan_world");
  const auto empty = tracker.snapshot();
  ASSERT_TRUE(empty);
  // No surviving clusters is not spatial clearance evidence. Consumers retain
  // the source timestamp to decide freshness; reading does not refresh it.
  EXPECT_TRUE(empty->obstacles.empty());
  EXPECT_EQ(empty->stamp, scan.header.stamp);
  EXPECT_EQ(empty->frame_id, "scan_world");
  EXPECT_EQ(tracker.snapshot(), empty);
  EXPECT_EQ(tracker.snapshot()->stamp, scan.header.stamp);
  tracker.update(scanAt({1, 1, 1}, 1400000000LL), "scan_world");
  ASSERT_TRUE(tracker.snapshot());
  EXPECT_FALSE(tracker.snapshot()->obstacles[0].velocity_valid);
}

TEST_F(ScanTrackerTest, TfFailureIncludingEmptyScanIsUnavailableAndClearsAssociation)
{
  ScanObstacleTracker tracker(tf);
  tracker.update(scanAt({1, 1, 1}), "scan_world");
  tracker.update(scanAt({}, 1200000000LL, "missing_laser"), "scan_world");
  EXPECT_FALSE(tracker.snapshot());
  tracker.update(scanAt({1, 1, 1}, 1400000000LL), "scan_world");
  ASSERT_TRUE(tracker.snapshot());
  EXPECT_FALSE(tracker.snapshot()->obstacles[0].velocity_valid);
}

TEST_F(ScanTrackerTest, ZeroRepeatedAndReversedTimesFailWithoutLeakingVelocity)
{
  for (int64_t stamp : {0LL, 1000000000LL, 900000000LL}) {
    ScanObstacleTracker tracker(tf);
    tracker.update(scanAt({1, 1, 1}), "scan_world");
    tracker.update(scanAt({1, 1, 1}, stamp), "scan_world");
    EXPECT_FALSE(tracker.snapshot());
    tracker.update(scanAt({1, 1, 1}, 1200000000LL), "scan_world");
    ASSERT_TRUE(tracker.snapshot());
    EXPECT_FALSE(tracker.snapshot()->obstacles[0].velocity_valid);
  }
}

TEST_F(ScanTrackerTest, InvalidMetadataDoesNotLookLikeAnEmptyObservation)
{
  ScanObstacleTracker tracker(tf);
  auto scan = scanAt({1, 1, 1});
  scan.angle_increment = std::numeric_limits<float>::quiet_NaN();
  tracker.update(scan, "scan_world");
  EXPECT_FALSE(tracker.snapshot());
  scan = scanAt({1, 1, 1});
  scan.header.stamp.nanosec = 1000000000u;
  tracker.update(scan, "scan_world");
  EXPECT_FALSE(tracker.snapshot());
  scan = scanAt({1, 1, 1});
  scan.header.stamp.sec = -1;
  tracker.update(scan, "scan_world");
  EXPECT_FALSE(tracker.snapshot());
  tracker.update(scanAt({1, 1, 1}), "");
  EXPECT_FALSE(tracker.snapshot());
}

TEST_F(ScanTrackerTest, ResetRemovesLatestAndPreviousFrame)
{
  ScanObstacleTracker tracker(tf);
  tracker.update(scanAt({1, 1, 1}), "scan_world");
  tracker.reset();
  EXPECT_FALSE(tracker.snapshot());
  tracker.update(scanAt({1, 1, 1}, 1200000000LL), "scan_world");
  ASSERT_TRUE(tracker.snapshot());
  EXPECT_FALSE(tracker.snapshot()->obstacles[0].velocity_valid);
}

TEST_F(ScanTrackerTest, RejectsInvalidConstructionParameters)
{
  EXPECT_THROW(ScanObstacleTracker(nullptr), std::invalid_argument);
  EXPECT_THROW(ScanObstacleTracker(tf, 0.0), std::invalid_argument);
  EXPECT_THROW(ScanObstacleTracker(tf, 0.12, 0), std::invalid_argument);
  EXPECT_THROW(ScanObstacleTracker(tf, 0.12, 3, kInvalid), std::invalid_argument);
}

}  // namespace
}  // namespace my_dwa_controller
