#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "my_dwa_controller/path_conflict_evaluator.hpp"

namespace my_dwa_controller
{
namespace
{
constexpr double kTolerance = 1e-9;
// Synthetic physics for formula tests only; these are not robot configuration.
const BrakingModel kSyntheticBraking{0.4, 0.2};

Pose2D point(double x, double y, double theta = 0.0)
{
  Pose2D value;
  value.x = x;
  value.y = y;
  value.theta = theta;
  return value;
}

class PathConflictTest : public ::testing::Test
{
protected:
  PathConflictEvaluator evaluator{0.22, 0.05, 1.7, 0.05, 0.35};
  std::vector<Pose2D> straight{point(0, 0, 0), point(5, 0, 0)};
  ScanSnapshot scan;

  void SetUp() override
  {
    scan.stamp = rclcpp::Time(1000000000LL, RCL_ROS_TIME);
    scan.frame_id = "conflict_world";
  }

  void obstacle(const ObstacleAabb & box, double vx = 0.0, double vy = 0.0, bool known = true)
  {
    scan.obstacles.push_back({box, vx, vy, known, {}});
  }

  PathConflictResult assess(
    int64_t now = 1000000000LL, std::optional<BrakingModel> braking = std::nullopt,
    double maximum = 0.26)
  {
    return evaluator.evaluate(
      &scan, straight, point(0, 0, 0), scan.frame_id,
      rclcpp::Time(now, RCL_ROS_TIME), maximum, braking);
  }
};

TEST_F(PathConflictTest, UsesScanAgeForCurrentPredictedPosition)
{
  obstacle({2, 2.1, -0.1, 0.1}, 1.0, 0.0);
  const auto at_scan = assess();
  const auto delayed = assess(1200000000LL);
  ASSERT_TRUE(at_scan.valid);
  ASSERT_TRUE(delayed.valid);
  ASSERT_TRUE(at_scan.conflict_distance);
  ASSERT_TRUE(delayed.conflict_distance);
  EXPECT_NEAR(*at_scan.conflict_distance, 1.73, kTolerance);
  EXPECT_NEAR(*delayed.conflict_distance, 1.93, kTolerance);
  EXPECT_FALSE(delayed.speed_limit);
}

TEST_F(PathConflictTest, PredictsBothVelocityComponents)
{
  PathConflictEvaluator exact(0.0, 0.0, 1.0, 0.25, 0.35);
  obstacle({1, 1, 1, 1}, 1.0, -1.0);
  const auto result = exact.evaluate(
    &scan, straight, point(0, 0, 0), scan.frame_id,
    rclcpp::Time(1000000000LL, RCL_ROS_TIME), 0.26);
  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.conflict_distance);
  // Conservative swept box for t=[0.75,1] reaches y=0 and starts at x=1.75.
  EXPECT_NEAR(*result.conflict_distance, 1.75, kTolerance);
}

TEST_F(PathConflictTest, CrossingAndDepartingMotionDifferAtTheSameObservedPosition)
{
  obstacle({2, 2.2, 0.9, 1.1}, 0.0, -1.0);
  const auto crossing = assess();
  ASSERT_TRUE(crossing.valid);
  ASSERT_TRUE(crossing.conflict_distance);
  EXPECT_NEAR(*crossing.conflict_distance, 1.73, kTolerance);
  scan.obstacles[0].vy = 1.0;
  const auto departing = assess();
  EXPECT_TRUE(departing.valid);
  EXPECT_FALSE(departing.conflict_distance);
}

TEST_F(PathConflictTest, FarLateralObstacleDoesNotConflict)
{
  obstacle({1, 1.2, 2, 2.2}, 0.0, 0.1);
  const auto result = assess();
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.conflict_distance);
}

TEST_F(PathConflictTest, SweepsBetweenTimeSamplesInsteadOfMissingAFastCrossing)
{
  PathConflictEvaluator coarse(0.0, 0.0, 0.1, 0.1, 0.35);
  obstacle({2, 2.1, 0.9, 1.0}, 0.0, -20.0);
  const auto result = coarse.evaluate(
    &scan, straight, point(0, 0, 0), scan.frame_id,
    rclcpp::Time(1000000000LL, RCL_ROS_TIME), 0.26);
  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.conflict_distance);
  EXPECT_NEAR(*result.conflict_distance, 2.0, kTolerance);
}

TEST_F(PathConflictTest, UsesBentPathAndArcDistanceRatherThanStraightLine)
{
  straight = {point(0, 0, 0), point(2, 0, 0), point(2, 2, 0)};
  obstacle({1.95, 2.05, 1, 1.1});
  const auto result = assess();
  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.conflict_distance);
  EXPECT_NEAR(*result.conflict_distance, 2.73, kTolerance);
  scan.obstacles.clear();
  obstacle({0.9, 1.1, 0.9, 1.1});  // On the endpoint chord, away from both real legs.
  const auto side = assess();
  EXPECT_TRUE(side.valid);
  EXPECT_FALSE(side.conflict_distance);
}

TEST_F(PathConflictTest, ClipsAtRobotProjectionAndExcludesRearObstacle)
{
  straight = {point(-2, 0, 0), point(0, 0, 0), point(4, 0, 0)};
  obstacle({-1, -0.8, -0.1, 0.1});
  const auto rear = evaluator.evaluate(
    &scan, straight, point(1, 0, 0), scan.frame_id,
    rclcpp::Time(1000000000LL, RCL_ROS_TIME), 0.26);
  EXPECT_TRUE(rear.valid);
  EXPECT_FALSE(rear.conflict_distance);
  scan.obstacles.clear();
  obstacle({2, 2.1, -0.1, 0.1});
  const auto ahead = evaluator.evaluate(
    &scan, straight, point(1, 0.2, 0), scan.frame_id,
    rclcpp::Time(1000000000LL, RCL_ROS_TIME), 0.26);
  ASSERT_TRUE(ahead.conflict_distance);
  EXPECT_NEAR(*ahead.conflict_distance, 0.73, kTolerance);
}

TEST_F(PathConflictTest, FindsEarliestConflictRegardlessOfObstacleOrder)
{
  obstacle({4, 4.1, -0.1, 0.1});
  obstacle({2, 2.1, -0.1, 0.1});
  const auto result = assess();
  ASSERT_TRUE(result.conflict_distance);
  EXPECT_NEAR(*result.conflict_distance, 1.73, kTolerance);
}

TEST_F(PathConflictTest, UnknownMotionKeepsObservedHitButCannotProveFutureSafety)
{
  obstacle({2, 2.1, -0.1, 0.1}, 1000.0, 1000.0, false);
  const auto current = assess(1000000000LL, kSyntheticBraking);
  EXPECT_FALSE(current.valid);
  ASSERT_TRUE(current.conflict_distance);
  EXPECT_NEAR(*current.conflict_distance, 1.73, kTolerance);
  EXPECT_FALSE(current.speed_limit);
  scan.obstacles[0].aabb = {2, 2.1, 1, 1.1};
  const auto unknown = assess();
  EXPECT_FALSE(unknown.valid);
  EXPECT_FALSE(unknown.conflict_distance);
  EXPECT_FALSE(unknown.speed_limit);
}

TEST_F(PathConflictTest, FreshEmptyScanIsDistinctFromMissingStaleOrFutureScan)
{
  const auto empty = assess();
  EXPECT_TRUE(empty.valid);  // No detected clusters, not a region-clear certificate.
  EXPECT_FALSE(empty.conflict_distance);
  EXPECT_FALSE(empty.speed_limit);
  EXPECT_FALSE(assess(1350000001LL).valid);
  EXPECT_TRUE(assess(1350000000LL).valid);
  EXPECT_FALSE(assess(999999999LL).valid);
  const auto missing = evaluator.evaluate(
    nullptr, straight, point(0, 0, 0), scan.frame_id,
    rclcpp::Time(1000000000LL, RCL_ROS_TIME), 0.26);
  EXPECT_FALSE(missing.valid);
  EXPECT_FALSE(missing.speed_limit);
}

TEST_F(PathConflictTest, NoPhysicalModelMeansNoGeneratedSpeedLimit)
{
  obstacle({2, 2.1, -0.1, 0.1});
  const auto result = assess();
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.conflict_distance);
  EXPECT_FALSE(result.speed_limit);
}

TEST_F(PathConflictTest, SyntheticBrakingIsMonotonicAndUsesInflatedBoundaryOnlyOnce)
{
  double previous_cap = -1.0;
  for (double x : {0.27, 0.28, 0.3, 0.5, 1.0, 2.0}) {
    scan.obstacles.clear();
    obstacle({x, x + 0.1, -0.1, 0.1});
    const auto result = assess(1000000000LL, kSyntheticBraking, 1.0);
    ASSERT_TRUE(result.valid);
    ASSERT_TRUE(result.conflict_distance);
    ASSERT_TRUE(result.speed_limit);
    const double distance = std::max(0.0, x - 0.27);
    const double offset = 0.4 * 0.2;
    const double expected = std::min(
      1.0, std::sqrt(offset * offset + 2.0 * 0.4 * distance) - offset);
    EXPECT_NEAR(*result.conflict_distance, distance, kTolerance);
    EXPECT_NEAR(*result.speed_limit, expected, kTolerance);
    EXPECT_GE(*result.speed_limit, previous_cap);
    EXPECT_LE(*result.speed_limit, 1.0);
    EXPECT_TRUE(std::isfinite(*result.speed_limit));
    previous_cap = *result.speed_limit;
  }
}

TEST_F(PathConflictTest, SyntheticSpeedLimitRespectsMaximumAndHandlesZeroDistanceAndDelay)
{
  obstacle({4, 4.1, -0.1, 0.1});
  auto result = assess(1000000000LL, kSyntheticBraking);
  ASSERT_TRUE(result.speed_limit);
  EXPECT_DOUBLE_EQ(*result.speed_limit, 0.26);
  scan.obstacles[0].aabb = {-0.1, 0.1, -0.1, 0.1};
  result = assess(1000000000LL, BrakingModel{0.4, 0.0});
  ASSERT_TRUE(result.speed_limit);
  EXPECT_DOUBLE_EQ(*result.speed_limit, 0.0);
  EXPECT_TRUE(std::isfinite(*result.speed_limit));
}

TEST_F(PathConflictTest, HandlesSinglePointAndRepeatedPathVertices)
{
  straight = {point(0, 0, 0), point(0, 0, 0), point(2, 0, 0), point(2, 0, 0)};
  obstacle({1, 1.1, -0.1, 0.1});
  EXPECT_TRUE(assess().conflict_distance);
  straight = {point(0, 0, 0)};
  scan.obstacles[0].aabb = {-0.1, 0.1, -0.1, 0.1};
  const auto result = assess();
  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.conflict_distance);
  EXPECT_DOUBLE_EQ(*result.conflict_distance, 0.0);
}

TEST_F(PathConflictTest, InvalidFramesGeometryTimeAndModelsYieldNoSafeCap)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  auto invalid = evaluator.evaluate(
    &scan, straight, point(0, 0, 0), "wrong_frame",
    rclcpp::Time(1000000000LL, RCL_ROS_TIME), 0.26);
  EXPECT_FALSE(invalid.valid);
  EXPECT_FALSE(invalid.speed_limit);
  obstacle({2, 1, 0, 1});
  EXPECT_FALSE(assess().valid);
  scan.obstacles[0].aabb = {1, 2, 0, 1};
  scan.obstacles[0].vx = nan;
  EXPECT_FALSE(assess().valid);
  scan.obstacles.clear();
  straight[0].x = nan;
  EXPECT_FALSE(assess().valid);
  straight.clear();
  EXPECT_FALSE(assess().valid);
  straight = {point(0, 0, 0), point(5, 0, 0)};
  EXPECT_FALSE(assess(1000000000LL, BrakingModel{0.0, 0.2}).valid);
  EXPECT_FALSE(assess(1000000000LL, BrakingModel{0.4, -0.1}).valid);
  EXPECT_FALSE(assess(1000000000LL, kSyntheticBraking, nan).valid);
  scan.stamp.nanosec = 1000000000u;
  EXPECT_FALSE(assess().valid);
}

TEST_F(PathConflictTest, FiniteExtremeSyntheticBrakingInputsDoNotOverflowSpeedCap)
{
  obstacle({1, 1.1, -0.1, 0.1});
  const double huge = std::numeric_limits<double>::max();
  const auto result = assess(1000000000LL, BrakingModel{huge, huge});
  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.speed_limit);
  EXPECT_TRUE(std::isfinite(*result.speed_limit));
  EXPECT_GE(*result.speed_limit, 0.0);
  EXPECT_LE(*result.speed_limit, 0.26);
}

TEST_F(PathConflictTest, InvalidConstructorInputsAreRejected)
{
  EXPECT_THROW(PathConflictEvaluator(-0.1, 0.05, 1.7, 0.05, 0.35), std::invalid_argument);
  EXPECT_THROW(PathConflictEvaluator(0.22, 0.05, 0.0, 0.05, 0.35), std::invalid_argument);
  EXPECT_THROW(PathConflictEvaluator(0.22, 0.05, 1.7, 0.0, 0.35), std::invalid_argument);
  EXPECT_THROW(PathConflictEvaluator(0.22, 0.05, 1.7, 0.05, 0.0), std::invalid_argument);
}

}  // namespace
}  // namespace my_dwa_controller
