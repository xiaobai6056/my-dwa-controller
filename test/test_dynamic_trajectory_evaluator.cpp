#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "my_dwa_controller/dynamic_trajectory_evaluator.hpp"
#include "my_dwa_controller/trajectory_predictor.hpp"

namespace my_dwa_controller
{
namespace
{

constexpr double kTolerance = 1e-9;

Pose2D point(double x, double y, double theta = 0.0)
{
  Pose2D pose;
  pose.x = x;
  pose.y = y;
  pose.theta = theta;
  return pose;
}

class DynamicTrajectoryTest : public ::testing::Test
{
protected:
  DynamicTrajectoryEvaluator evaluator{0.22, 0.05, 1.7, 0.05, 0.35};
  TrajectoryPredictor predictor{1.7, 0.05};
  Trajectory trajectory = predictor.predict(point(0, 0), {1.0, 0.0});
  ScanSnapshot scan;

  void SetUp() override
  {
    scan.stamp = rclcpp::Time(1000000000LL, RCL_ROS_TIME);
    scan.frame_id = "candidate_world";
  }

  void obstacle(const ObstacleAabb & box, double vx = 0.0, double vy = 0.0, bool known = true)
  {
    // Explicit measured points for synthetic fixtures, not an implicit filled box.
    scan.obstacles.push_back(
      {box, vx, vy, known,
        {{box.min_x, box.min_y}, {box.min_x, box.max_y},
          {box.max_x, box.min_y}, {box.max_x, box.max_y}}});
  }

  DynamicTrajectoryResult assess(int64_t now = 1000000000LL)
  {
    return evaluator.evaluate(
      trajectory, &scan, scan.frame_id, rclcpp::Time(now, RCL_ROS_TIME));
  }

  static void expectInvalid(const DynamicTrajectoryResult & result)
  {
    EXPECT_FALSE(result.valid);
    EXPECT_FALSE(result.collision);
    EXPECT_FALSE(result.min_clearance);
  }
};

TEST_F(DynamicTrajectoryTest, AabbWithoutScanPointsCannotEstablishDynamicSafety)
{
  obstacle({1, 1.1, 2, 2.1});
  scan.obstacles.back().points.clear();
  expectInvalid(assess());
}

TEST_F(DynamicTrajectoryTest, UShapedPointsDoNotFillTheirAabbInterior)
{
  obstacle({-1, 1, -1, 1});
  auto & points = scan.obstacles.back().points;
  for (int i = 0; i <= 20; ++i) {
    const double value = -1.0 + 0.1 * i;
    points.push_back({value, 1.0});
    points.push_back({-1.0, value});
    points.push_back({value, -1.0});
  }
  trajectory = predictor.predict(point(0, 0), {0.4, 0.0});
  const auto result = assess();
  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.min_clearance);
  EXPECT_FALSE(result.collision);
  EXPECT_NEAR(*result.min_clearance, 0.73, kTolerance);
}

TEST_F(DynamicTrajectoryTest, StaticObstacleHasExpectedMinimumClearance)
{
  obstacle({0.8, 1.0, 0.6, 0.8});
  const auto result = assess();
  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.min_clearance);
  EXPECT_NEAR(*result.min_clearance, 0.6 - 0.27, kTolerance);
  EXPECT_FALSE(result.collision);
  obstacle({1.2, 1.3, 0.4, 0.5});
  const auto nearer = assess();
  std::reverse(scan.obstacles.begin(), scan.obstacles.end());
  const auto reordered = assess();
  ASSERT_TRUE(nearer.min_clearance);
  ASSERT_TRUE(reordered.min_clearance);
  EXPECT_NEAR(*nearer.min_clearance, 0.13, kTolerance);
  EXPECT_DOUBLE_EQ(*nearer.min_clearance, *reordered.min_clearance);
}

TEST_F(DynamicTrajectoryTest, FindsMinimumInsideIntervalRatherThanOnlyAtSamples)
{
  DynamicTrajectoryEvaluator coarse(0.22, 0.05, 1.0, 1.0, 0.35);
  const auto two_points = TrajectoryPredictor(1.0, 1.0).predict(point(0, 0), {1.0, 0.0});
  obstacle({0.5, 0.5, 0.4, 0.4});
  const auto result = coarse.evaluate(
    two_points, &scan, scan.frame_id, rclcpp::Time(1000000000LL, RCL_ROS_TIME));
  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.min_clearance);
  EXPECT_NEAR(*result.min_clearance, 0.13, kTolerance);
  EXPECT_FALSE(result.collision);
}

TEST_F(DynamicTrajectoryTest, EuclideanCornerClearanceDoesNotUseSquareExpansion)
{
  trajectory = predictor.predict(point(0.8, 0.8), {0.0, 0.0});
  obstacle({1, 1, 1, 1});
  const auto result = assess();
  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.min_clearance);
  EXPECT_NEAR(*result.min_clearance, std::hypot(0.2, 0.2) - 0.27, kTolerance);
  EXPECT_FALSE(result.collision);  // Inside square expansion, outside rounded clearance.
}

TEST_F(DynamicTrajectoryTest, DiagonalIntervalFindsClosestMeasuredPoint)
{
  DynamicTrajectoryEvaluator diagonal(0.22, 0.05, 1.0, 1.0, 0.35);
  const auto crossing_line = TrajectoryPredictor(1.0, 1.0).predict(
    point(0, 0, std::atan2(1.0, 1.0)), {std::sqrt(2.0), 0.0});
  obstacle({0.0, 0.1, 0.9, 1.0});
  const auto result = diagonal.evaluate(
    crossing_line, &scan, scan.frame_id, rclcpp::Time(1000000000LL, RCL_ROS_TIME));
  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.min_clearance);
  EXPECT_NEAR(*result.min_clearance, 0.8 / std::sqrt(2.0) - 0.27, kTolerance);
  EXPECT_FALSE(result.collision);
}

TEST_F(DynamicTrajectoryTest, ClearanceBoundaryAndCollisionSubtractMarginOnlyOnce)
{
  trajectory = predictor.predict(point(0, 0), {0.0, 0.0});
  for (double y : {0.0, 0.269, 0.27, 0.271, 0.5}) {
    SCOPED_TRACE(y);
    scan.obstacles.clear();
    obstacle({0, 0, y, y});
    const auto result = assess();
    ASSERT_TRUE(result.valid);
    ASSERT_TRUE(result.min_clearance);
    EXPECT_NEAR(*result.min_clearance, y - 0.27, kTolerance);
    EXPECT_EQ(result.collision, *result.min_clearance <= 0.0);
    EXPECT_EQ(result.collision, y <= 0.27);
  }
}

TEST_F(DynamicTrajectoryTest, DetectsFastCrossingBetweenClearEndpointSamples)
{
  DynamicTrajectoryEvaluator coarse(0.22, 0.05, 1.0, 1.0, 0.35);
  const auto stopped = TrajectoryPredictor(1.0, 1.0).predict(point(0, 0), {0.0, 0.0});
  obstacle({-0.05, 0.05, 1.0, 1.1}, 0.0, -2.0);
  const auto result = coarse.evaluate(
    stopped, &scan, scan.frame_id, rclcpp::Time(1000000000LL, RCL_ROS_TIME));
  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.collision);
  ASSERT_TRUE(result.min_clearance);
  EXPECT_NEAR(*result.min_clearance, 0.05 - 0.27, kTolerance);
}

TEST_F(DynamicTrajectoryTest, FastMeasuredPointCrossesInsideActualFiftyMillisecondInterval)
{
  trajectory = predictor.predict(point(0, 0), {0.0, 0.0});
  obstacle({0, 0, 0.5, 0.5}, 0.0, -20.0);
  // At t=0 and t=0.05 the measured point is 0.5 m away. It crosses at 0.025.
  const auto result = assess();
  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.min_clearance);
  EXPECT_TRUE(result.collision);
  EXPECT_NEAR(*result.min_clearance, -0.27, kTolerance);
}

TEST_F(DynamicTrajectoryTest, PointSpacingPreservesPhysicalRadiusButDoesNotInventASurface)
{
  trajectory = predictor.predict(point(0, 0), {0.0, 0.0});
  for (double y : {0.22, 0.2695}) {
    SCOPED_TRACE(y);
    scan.obstacles.clear();
    obstacle({-0.06, 0.06, y, y});
    scan.obstacles.back().points = {{-0.06, y}, {0.06, y}};
    const auto result = assess();
    ASSERT_TRUE(result.valid);
    ASSERT_TRUE(result.min_clearance);
    EXPECT_NEAR(*result.min_clearance, std::hypot(0.06, y) - 0.27, kTolerance);
    EXPECT_EQ(result.collision, y == 0.22);
  }
  // A 0.12 m beam gap is covered at physical radius 0.22 by the 0.27 point
  // radius. Close to the outer margin, a hypothetical connecting surface can
  // be closer than either actual point; offline acceptance quantifies this.
}

TEST_F(DynamicTrajectoryTest, NonFiniteMeasuredPointsCannotEstablishDynamicSafety)
{
  obstacle({1, 1, 1, 1});
  for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    scan.obstacles.back().points = {{1.0, invalid}};
    expectInvalid(assess());
  }
}

TEST_F(DynamicTrajectoryTest, SpatialCrossingDistinguishesEarlySimultaneousAndLateArrival)
{
  obstacle({1, 1, 1, 1}, 0.0, -1.0);  // Crosses (1,0) at t=1.
  for (double arrival : {0.3, 1.0, 1.7}) {
    SCOPED_TRACE(arrival);
    trajectory = predictor.predict(point(1.0 - arrival, 0), {1.0, 0.0});
    const auto result = assess();
    ASSERT_TRUE(result.valid);
    ASSERT_TRUE(result.min_clearance);
    EXPECT_NEAR(
      *result.min_clearance, std::abs(arrival - 1.0) / std::sqrt(2.0) - 0.27,
      kTolerance);
    EXPECT_EQ(result.collision, arrival == 1.0);
  }
}

TEST_F(DynamicTrajectoryTest, ActualPredictorTurnsDependOnObstacleSide)
{
  for (double side : {-1.0, 1.0}) {
    SCOPED_TRACE(side);
    const auto toward = predictor.predict(point(0, 0), {1.0, side});
    const auto away = predictor.predict(point(0, 0), {1.0, -side});
    const auto & center = toward.poses[24];
    scan.obstacles.clear();
    obstacle({center.x - 0.04, center.x + 0.04, center.y - 0.04, center.y + 0.04});
    const auto straight = assess();
    trajectory = toward;
    const auto toward_result = assess();
    trajectory = away;
    const auto away_result = assess();
    ASSERT_TRUE(straight.valid);
    ASSERT_TRUE(toward_result.valid);
    ASSERT_TRUE(away_result.valid);
    ASSERT_TRUE(straight.min_clearance);
    ASSERT_TRUE(toward_result.min_clearance);
    ASSERT_TRUE(away_result.min_clearance);
    EXPECT_TRUE(toward_result.collision);
    EXPECT_FALSE(straight.collision);
    EXPECT_FALSE(away_result.collision);
    EXPECT_LT(*toward_result.min_clearance, *straight.min_clearance);
    EXPECT_LT(*straight.min_clearance, *away_result.min_clearance);
    trajectory = predictor.predict(point(0, 0), {1.0, 0.0});
  }
}

TEST_F(DynamicTrajectoryTest, DepartingObstacleDoesNotCreateUnfoundedCollision)
{
  obstacle({1, 1.1, 0.8, 0.9});
  const auto stationary = assess();
  scan.obstacles[0].vy = 1.0;
  const auto departing = assess();
  ASSERT_TRUE(departing.valid);
  ASSERT_TRUE(stationary.min_clearance);
  ASSERT_TRUE(departing.min_clearance);
  EXPECT_FALSE(departing.collision);
  EXPECT_GT(*departing.min_clearance, *stationary.min_clearance);
}

TEST_F(DynamicTrajectoryTest, ScanAgeIsIncludedInBothVelocityComponents)
{
  DynamicTrajectoryEvaluator shorter(0.22, 0.05, 0.5, 0.25, 0.35);
  const auto stopped = TrajectoryPredictor(0.5, 0.25).predict(point(0, 0), {0.0, 0.0});
  obstacle({1, 1, 1, 1}, -1.0, -1.0);
  const auto initial = shorter.evaluate(
    stopped, &scan, scan.frame_id, rclcpp::Time(1000000000LL, RCL_ROS_TIME));
  const auto delayed = shorter.evaluate(
    stopped, &scan, scan.frame_id, rclcpp::Time(1200000000LL, RCL_ROS_TIME));
  ASSERT_TRUE(initial.valid);
  ASSERT_TRUE(delayed.valid);
  ASSERT_TRUE(initial.min_clearance);
  ASSERT_TRUE(delayed.min_clearance);
  EXPECT_NEAR(*initial.min_clearance, std::hypot(0.5, 0.5) - 0.27, kTolerance);
  EXPECT_NEAR(*delayed.min_clearance, std::hypot(0.3, 0.3) - 0.27, kTolerance);
}

TEST_F(DynamicTrajectoryTest, UnknownMotionChecksInitialGeometryWithoutAssumingItIsStatic)
{
  obstacle({1, 1, 0, 0}, std::numeric_limits<double>::quiet_NaN(), 1000.0, false);
  const auto future_unknown = assess();
  EXPECT_FALSE(future_unknown.valid);
  EXPECT_FALSE(future_unknown.collision);  // Not safety: only initial geometry was checked.
  ASSERT_TRUE(future_unknown.min_clearance);
  EXPECT_NEAR(*future_unknown.min_clearance, 0.73, kTolerance);
  scan.obstacles[0].aabb = {0, 0, 0, 0};
  scan.obstacles[0].points = {{0, 0}};
  const auto current = assess();
  EXPECT_FALSE(current.valid);
  EXPECT_TRUE(current.collision);
  ASSERT_TRUE(current.min_clearance);
  EXPECT_NEAR(*current.min_clearance, -0.27, kTolerance);
}

TEST_F(DynamicTrajectoryTest, FreshEmptyIsDistinctFromMissingExpiredAndFutureScan)
{
  const auto empty = assess();
  EXPECT_TRUE(empty.valid);  // No extracted obstacle, not a region-clear certificate.
  EXPECT_FALSE(empty.collision);
  EXPECT_FALSE(empty.min_clearance);
  EXPECT_TRUE(assess(1350000000LL).valid);
  expectInvalid(assess(1350000001LL));
  expectInvalid(assess(999999999LL));
  expectInvalid(
    evaluator.evaluate(
      trajectory, nullptr, scan.frame_id, rclcpp::Time(1000000000LL, RCL_ROS_TIME)));
  expectInvalid(
    evaluator.evaluate(
      trajectory, &scan, scan.frame_id, rclcpp::Time(1000000000LL, RCL_SYSTEM_TIME)));
}

TEST_F(DynamicTrajectoryTest, InvalidInputsCannotProduceAnApparentlySafeEvaluation)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  expectInvalid(
    evaluator.evaluate(
      trajectory, &scan, "wrong_frame", rclcpp::Time(1000000000LL, RCL_ROS_TIME)));
  auto saved = trajectory;
  trajectory.poses[0].x = nan;
  expectInvalid(assess());
  trajectory = saved;
  trajectory.poses.back().theta = std::numeric_limits<double>::infinity();
  expectInvalid(assess());
  trajectory = saved;
  trajectory.command.v = nan;
  expectInvalid(assess());
  trajectory = saved;
  obstacle({1, 0, 0, 1});
  expectInvalid(assess());
  scan.obstacles[0].aabb = {0, 1, 0, nan};
  expectInvalid(assess());
  scan.obstacles[0].aabb = {0, 1, 0, 1};
  scan.obstacles[0].vy = nan;
  expectInvalid(assess());
  scan.obstacles.clear();
  scan.stamp.sec = -1;
  expectInvalid(assess());
  scan.stamp.sec = 0;
  expectInvalid(assess());
  scan.stamp.sec = 1;
  scan.stamp.nanosec = 1000000000u;
  expectInvalid(assess());
  scan.stamp.nanosec = 0;
  scan.frame_id = "/invalid";
  expectInvalid(assess());
  scan.frame_id.clear();
  expectInvalid(assess());
}

TEST_F(DynamicTrajectoryTest, UsesPredictorShortenedDecimalAndSubStepEndpoints)
{
  // A fictitious full final dt would predict a collision in these cases.
  for (const auto & times : std::array<std::array<double, 2>, 4>{{
      {{1.0, 0.3}}, {{0.9, 0.3}}, {{0.05, 0.1}}, {{1e-12, 1e-13}}}})
  {
    const double horizon = times[0], dt = times[1];
    SCOPED_TRACE(::testing::Message() << "horizon=" << horizon << " dt=" << dt);
    DynamicTrajectoryEvaluator exact(0.0, 0.0, horizon, dt, 0.35);
    const auto stopped = TrajectoryPredictor(horizon, dt).predict(point(0, 0), {0.0, 0.0});
    const double x = horizon * 1.05;
    scan.obstacles.clear();
    obstacle({x, x, 0, 0}, -1.0, 0.0);
    const auto result = exact.evaluate(
      stopped, &scan, scan.frame_id, rclcpp::Time(1000000000LL, RCL_ROS_TIME));
    ASSERT_TRUE(result.valid);
    ASSERT_TRUE(result.min_clearance);
    EXPECT_NEAR(*result.min_clearance, x - horizon, horizon * 1e-12);
    EXPECT_FALSE(result.collision);
  }
}

TEST_F(DynamicTrajectoryTest, RejectsTrajectoryWithMissingOrExtraTimeSamples)
{
  const auto saved = trajectory;
  trajectory.poses.pop_back();
  expectInvalid(assess());
  trajectory = saved;
  trajectory.poses.push_back(trajectory.poses.back());
  expectInvalid(assess());
  trajectory.poses.resize(1);
  expectInvalid(assess());
  trajectory.poses.clear();
  expectInvalid(assess());
}

TEST_F(DynamicTrajectoryTest, ValidityCoversOnlyTheConfiguredPredictionHorizon)
{
  trajectory = predictor.predict(point(0, 0), {0.0, 0.0});
  obstacle({2.1, 2.1, 0, 0}, -1.0, 0.0);
  const auto within = assess();
  ASSERT_TRUE(within.valid);
  ASSERT_TRUE(within.min_clearance);
  EXPECT_NEAR(*within.min_clearance, 0.13, kTolerance);
  EXPECT_FALSE(within.collision);  // Does not claim anything beyond t=1.7.
  DynamicTrajectoryEvaluator longer(0.22, 0.05, 2.2, 0.05, 0.35);
  const auto longer_trajectory = TrajectoryPredictor(2.2, 0.05).predict(point(0, 0), {0.0, 0.0});
  const auto beyond = longer.evaluate(
    longer_trajectory, &scan, scan.frame_id, rclcpp::Time(1000000000LL, RCL_ROS_TIME));
  ASSERT_TRUE(beyond.valid);
  EXPECT_TRUE(beyond.collision);
  expectInvalid(
    evaluator.evaluate(
      longer_trajectory, &scan, scan.frame_id, rclcpp::Time(1000000000LL, RCL_ROS_TIME)));
}

TEST_F(DynamicTrajectoryTest, OutputsRemainFiniteForExtremeFiniteCoordinates)
{
  const double huge = std::numeric_limits<double>::max();
  trajectory = predictor.predict(point(huge, 0), {0.0, 0.0});
  obstacle({huge, huge, 0, 0});
  const auto touching = assess();
  ASSERT_TRUE(touching.valid);
  ASSERT_TRUE(touching.min_clearance);
  EXPECT_TRUE(std::isfinite(*touching.min_clearance));
  EXPECT_TRUE(touching.collision);
  scan.obstacles[0].aabb = {-huge, -huge, 0, 0};
  scan.obstacles[0].points = {{-huge, 0}};
  expectInvalid(assess());  // Unrepresentable distance is not reported as Infinity.
}

TEST_F(DynamicTrajectoryTest, RejectsInvalidConstructionParameters)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(DynamicTrajectoryEvaluator(-0.1, 0.05, 1.7, 0.05, 0.35), std::invalid_argument);
  EXPECT_THROW(DynamicTrajectoryEvaluator(0.22, -0.05, 1.7, 0.05, 0.35), std::invalid_argument);
  EXPECT_THROW(DynamicTrajectoryEvaluator(nan, 0.05, 1.7, 0.05, 0.35), std::invalid_argument);
  EXPECT_THROW(DynamicTrajectoryEvaluator(0.22, 0.05, 0.0, 0.05, 0.35), std::invalid_argument);
  EXPECT_THROW(DynamicTrajectoryEvaluator(0.22, 0.05, 1.7, 0.0, 0.35), std::invalid_argument);
  EXPECT_THROW(DynamicTrajectoryEvaluator(0.22, 0.05, 1.7, 0.05, 0.0), std::invalid_argument);
}


class SpatialScreenTest : public ::testing::Test
{
protected:
  DynamicTrajectoryEvaluator evaluator{0.22, 0.05, 1.7, 0.05, 0.35};
  TrajectoryPredictor predictor{1.7, 0.05};
  Trajectory trajectory = predictor.predict(point(0, 0), {0.0, 0.0});
  ScanSnapshot scan;

  void SetUp() override
  {
    scan.stamp = rclcpp::Time(1000000000LL, RCL_ROS_TIME);
    scan.frame_id = "candidate_world";
  }

  void add(
    std::vector<std::array<double, 2>> points, double vx = 0.0,
    double vy = 0.0, bool known = true)
  {
    scan.obstacles.push_back({{-100, 100, -100, 100}, vx, vy, known, std::move(points)});
  }

  std::optional<ScanSnapshot> screen(int64_t now = 1000000000LL)
  {
    return evaluator.prepareScan(
      {&trajectory}, &scan, scan.frame_id, rclcpp::Time(now, RCL_ROS_TIME));
  }

  void expectSameConflict(const ScanSnapshot & filtered, int64_t now = 1000000000LL)
  {
    const auto time = rclcpp::Time(now, RCL_ROS_TIME);
    const auto raw = evaluator.evaluate(trajectory, &scan, scan.frame_id, time);
    const auto result = evaluator.evaluate(trajectory, &filtered, scan.frame_id, time);
    ASSERT_TRUE(raw.valid);
    ASSERT_TRUE(raw.collision);
    EXPECT_EQ(result.valid, raw.valid);
    EXPECT_EQ(result.collision, raw.collision);
    ASSERT_TRUE(result.min_clearance);
    EXPECT_DOUBLE_EQ(*result.min_clearance, *raw.min_clearance);
  }
};

TEST_F(SpatialScreenTest, KeepsFutureTrajectoryBoundaryAndDropsFarPoints)
{
  trajectory = predictor.predict(point(0, 0), {1.0, 0.0});
  add({{1.7, 0.27}, {1.0, 0.0}, {-20, 20}});
  const auto filtered = screen();
  ASSERT_TRUE(filtered);
  ASSERT_EQ(filtered->obstacles.size(), 1u);
  ASSERT_EQ(filtered->obstacles[0].points.size(), 2u);
  EXPECT_EQ(filtered->obstacles[0].points[0], scan.obstacles[0].points[0]);
  EXPECT_EQ(filtered->obstacles[0].points[1], scan.obstacles[0].points[1]);
  EXPECT_EQ(filtered->stamp, scan.stamp);
  EXPECT_EQ(filtered->frame_id, scan.frame_id);
  expectSameConflict(*filtered);
}

TEST_F(SpatialScreenTest, PreservesCollisionInsideContinuousInterval)
{
  add({{0.0, -0.5}, {20, 20}}, 0.0, 20.0);
  const auto filtered = screen();
  ASSERT_TRUE(filtered);
  ASSERT_EQ(filtered->obstacles.size(), 1u);
  ASSERT_EQ(filtered->obstacles[0].points.size(), 1u);
  expectSameConflict(*filtered);
  EXPECT_DOUBLE_EQ(filtered->obstacles[0].vy, 20.0);
}

TEST_F(SpatialScreenTest, UsesEachOriginalObstacleVelocity)
{
  add({{2.0, 0.0}}, -2.0, 0.0);
  add({{2.0, 0.0}}, 2.0, 0.0);
  const auto filtered = screen();
  ASSERT_TRUE(filtered);
  ASSERT_EQ(filtered->obstacles.size(), 1u);
  EXPECT_DOUBLE_EQ(filtered->obstacles[0].vx, -2.0);
  expectSameConflict(*filtered);
}

TEST_F(SpatialScreenTest, AccountsForScanAgeBeforePredictionStarts)
{
  add({{2.0, 0.0}}, -1.0, 0.0);
  const auto fresh = screen();
  ASSERT_TRUE(fresh);
  EXPECT_TRUE(fresh->obstacles.empty());
  const auto aged = screen(1200000000LL);
  ASSERT_TRUE(aged);
  ASSERT_EQ(aged->obstacles.size(), 1u);
  expectSameConflict(*aged, 1200000000LL);
}

TEST_F(SpatialScreenTest, UnionsEveryCandidateAndUsesConfiguredRadius)
{
  add({{0, 1.45}, {20, 20}});
  const auto second = predictor.predict(point(0, 1.0), {0, 0});
  const DynamicTrajectoryEvaluator wider(0.4, 0.1, 1.7, 0.05, 0.35);
  const auto filtered = wider.prepareScan(
    {&trajectory, &second}, &scan, scan.frame_id, rclcpp::Time(scan.stamp));
  ASSERT_TRUE(filtered);
  ASSERT_EQ(filtered->obstacles.size(), 1u);
  ASSERT_EQ(filtered->obstacles[0].points.size(), 1u);
  EXPECT_TRUE(
    wider.evaluate(
      second, &*filtered, scan.frame_id,
      rclcpp::Time(scan.stamp)).collision);
}

TEST_F(SpatialScreenTest, EmptyScreenIsNotAnOriginalEmptyOrInvalidInput)
{
  add({{20, 20}});
  const auto filtered = screen();
  ASSERT_TRUE(filtered);
  EXPECT_TRUE(filtered->obstacles.empty());
  EXPECT_EQ(scan.obstacles.size(), 1u);
  const auto raw = evaluator.evaluate(trajectory, &scan, scan.frame_id, rclcpp::Time(scan.stamp));
  ASSERT_TRUE(raw.min_clearance);
  const auto result = evaluator.evaluate(
    trajectory, &*filtered, scan.frame_id, rclcpp::Time(scan.stamp));
  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.collision);
  EXPECT_FALSE(result.min_clearance);
}

TEST_F(SpatialScreenTest, UnknownVelocityCannotDisappearAndBecomeValid)
{
  add({{20, 20}}, 0, 0, false);
  EXPECT_FALSE(screen());
  const auto raw = evaluator.evaluate(trajectory, &scan, scan.frame_id, rclcpp::Time(scan.stamp));
  EXPECT_FALSE(raw.valid);
}

TEST_F(SpatialScreenTest, BypassesInvalidTimeFrameAndMissingInput)
{
  add({{20, 20}});
  EXPECT_FALSE(screen(999999999LL));
  EXPECT_FALSE(screen(1350000001LL));
  EXPECT_FALSE(
    evaluator.prepareScan(
      {&trajectory}, nullptr, scan.frame_id, rclcpp::Time(
        scan.stamp)));
  EXPECT_FALSE(evaluator.prepareScan({&trajectory}, &scan, "other", rclcpp::Time(scan.stamp)));
  EXPECT_FALSE(evaluator.prepareScan({&trajectory}, &scan, "/bad", rclcpp::Time(scan.stamp)));
  EXPECT_FALSE(
    evaluator.prepareScan(
      {&trajectory}, &scan, scan.frame_id, rclcpp::Time(int64_t{1000000000}, RCL_SYSTEM_TIME)));
  scan.stamp.sec = 0;
  scan.stamp.nanosec = 0;
  EXPECT_FALSE(screen());
}

TEST_F(SpatialScreenTest, OriginalMalformedGeometryCannotBeFilteredAway)
{
  add({{20, 20}});
  scan.obstacles[0].points[0][0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(screen());
  scan.obstacles[0].points.clear();
  EXPECT_FALSE(screen());
  scan.obstacles[0].points = {{20, 20}};
  scan.obstacles[0].aabb.min_x = 101;
  EXPECT_FALSE(screen());
  scan.obstacles[0].aabb.min_x = -100;
  scan.obstacles[0].vx = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(screen());
}

TEST_F(SpatialScreenTest, BypassesInvalidTrajectoryAndTimeMesh)
{
  add({{20, 20}});
  EXPECT_FALSE(evaluator.prepareScan({}, &scan, scan.frame_id, rclcpp::Time(scan.stamp)));
  EXPECT_FALSE(evaluator.prepareScan({nullptr}, &scan, scan.frame_id, rclcpp::Time(scan.stamp)));
  const auto good = trajectory;
  trajectory.poses.pop_back();
  EXPECT_FALSE(screen());
  trajectory = good;
  trajectory.poses.push_back(trajectory.poses.back());
  EXPECT_FALSE(screen());
  trajectory = good;
  trajectory.poses[0].x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(screen());
  trajectory = good;
  trajectory.command.v = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(screen());
}

TEST_F(SpatialScreenTest, KeepsOutwardNumericalBoundaryAndConfirmedEmptyValidity)
{
  add({{0.27, 0}, {0.27 + 1e-11, 0}, {0.27 + 1e-6, 0}});
  const auto filtered = screen();
  ASSERT_TRUE(filtered);
  ASSERT_EQ(filtered->obstacles.size(), 1u);
  EXPECT_EQ(filtered->obstacles[0].points.size(), 2u);
  expectSameConflict(*filtered);
  scan.obstacles.clear();
  const auto empty = screen();
  ASSERT_TRUE(empty);
  EXPECT_TRUE(
    evaluator.evaluate(
      trajectory, &*empty, scan.frame_id, rclcpp::Time(scan.stamp)).valid);
}

TEST_F(SpatialScreenTest, NumericalExtremesBypassRatherThanPromoteSafety)
{
  const double huge = std::numeric_limits<double>::max();
  trajectory = predictor.predict(point(-huge, 0), {0, 0});
  add({{huge, 0}});
  EXPECT_FALSE(screen());
  EXPECT_FALSE(
    evaluator.evaluate(
      trajectory, &scan, scan.frame_id, rclcpp::Time(scan.stamp)).valid);
}

}  // namespace
}  // namespace my_dwa_controller
