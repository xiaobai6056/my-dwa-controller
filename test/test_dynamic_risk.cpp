#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include "my_dwa_controller/dynamic_risk_evaluator.hpp"
#include "my_dwa_controller/trajectory_predictor.hpp"

namespace my_dwa_controller
{
namespace
{
class DynamicRiskTest : public ::testing::Test
{
protected:
  DynamicRiskEvaluator evaluator{0.22, 0.05, 1.7, 0.05, 0.35};
  Trajectory straight = TrajectoryPredictor(1.7, 0.05).predict(Pose2D{}, {0.4, 0.0});
  Trajectory stopped = TrajectoryPredictor(1.7, 0.05).predict(Pose2D{}, {0.0, 0.0});
  ScanSnapshot scan;
  const rclcpp::Time now{1200000000LL, RCL_ROS_TIME};

  void SetUp() override
  {
    scan.stamp = rclcpp::Time(1000000000LL, RCL_ROS_TIME);
    scan.frame_id = "odom";
  }

  void point(double x, double y, double vx = 0.0, double vy = 0.0, bool known = true)
  {
    ObstacleObservation observation;
    observation.aabb = {x, x, y, y};
    observation.points.push_back({x, y});
    observation.vx = vx;
    observation.vy = vy;
    observation.velocity_valid = known;
    scan.obstacles.push_back(observation);
  }

  PreparedDynamicRisk prepare(bool screen = true)
  {
    return evaluator.prepare({&straight, &stopped}, &scan, "odom", now, screen);
  }
};

TEST_F(DynamicRiskTest, UnknownClusterDoesNotEraseKnownEvidenceOrOriginalPoints)
{
  point(0.4, 0.0);
  point(0.1, 0.0, 0.0, 0.0, false);
  const auto prepared = prepare();
  ASSERT_TRUE(prepared.input_available);
  EXPECT_EQ(prepared.predicted_clusters, 1u);
  EXPECT_EQ(prepared.unknown_clusters, 1u);
  EXPECT_EQ(scan.obstacles.size(), 2u);
  EXPECT_EQ(scan.obstacles[1].points.size(), 1u);
  const auto result = evaluator.evaluate(straight, prepared);
  ASSERT_TRUE(result.input_available);
  EXPECT_NEAR(result.cost, 1.0, 1e-12);
}

TEST_F(DynamicRiskTest, UnknownOnlyHasNoPredictionAndIsNotStaticEvidence)
{
  point(0.0, 0.0, 0.0, 0.0, false);
  const auto prepared = prepare();
  ASSERT_TRUE(prepared.input_available);
  EXPECT_EQ(prepared.predicted_clusters, 0u);
  EXPECT_EQ(prepared.unknown_clusters, 1u);
  const auto result = evaluator.evaluate(straight, prepared);
  EXPECT_TRUE(result.input_available);
  EXPECT_DOUBLE_EQ(result.cost, 0.0);
  EXPECT_FALSE(result.min_clearance);
}

TEST_F(DynamicRiskTest, ContinuousIntervalInteriorIsChecked)
{
  // Both interval endpoints are outside the radius, but the interval crosses.
  point(0.01, 0.0, 0.0, -20.0);
  scan.obstacles[0].points[0][1] = 4.5;
  scan.obstacles[0].aabb.min_y = scan.obstacles[0].aabb.max_y = 4.5;
  const auto result = evaluator.evaluate(straight, prepare());
  ASSERT_TRUE(result.input_available);
  EXPECT_GT(result.cost, 0.99);
}

TEST_F(DynamicRiskTest, ScanAgeAndEachClusterVelocityAffectPrediction)
{
  point(0.0, 0.47, 0.0, -1.0);
  point(5.0, 5.0, 0.4, 0.2);
  const auto aged = evaluator.evaluate(stopped, prepare());
  ASSERT_TRUE(aged.input_available);
  EXPECT_NEAR(aged.cost, 1.0, 1e-12);
  scan.obstacles[0].vy = 1.0;
  const auto receding = evaluator.evaluate(stopped, prepare());
  EXPECT_DOUBLE_EQ(receding.cost, 0.0);
}

TEST_F(DynamicRiskTest, RiskIsContinuousMonotonicAndDoesNotRewardFarClearance)
{
  double previous = -1.0;
  for (double y : {0.8, 0.27, 0.20, 0.10, 0.0}) {
    scan.obstacles.clear();
    point(0.0, y);
    const auto result = evaluator.evaluate(stopped, prepare());
    ASSERT_TRUE(result.input_available);
    EXPECT_GE(result.cost, previous);
    EXPECT_NEAR(result.cost, std::max(0.0, (0.27 - y) / 0.27), 1e-12);
    previous = result.cost;
  }
}

TEST_F(DynamicRiskTest, PointDensityAndFarGeometryDoNotChangeRisk)
{
  point(0.4, 0.10);
  const double expected = evaluator.evaluate(straight, prepare()).cost;
  scan.obstacles[0].points.resize(200, {0.4, 0.10});
  point(100, 100);
  EXPECT_DOUBLE_EQ(evaluator.evaluate(straight, prepare()).cost, expected);
}

TEST_F(DynamicRiskTest, ScreeningPreservesCostsAndConflictingClearance)
{
  for (double vx : {-0.6, 0.0, 0.7}) {
    scan.obstacles.clear();
    point(0.7, 0.4, vx, -0.3);
    point(10, 10, -0.2, 0.0);
    const auto screened = prepare(), full = prepare(false);
    ASSERT_TRUE(screened.input_available);
    EXPECT_LT(screened.retained_points, full.retained_points);
    for (const auto * trajectory : {&straight, &stopped}) {
      const auto a = evaluator.evaluate(*trajectory, screened);
      const auto b = evaluator.evaluate(*trajectory, full);
      EXPECT_EQ(a.input_available, b.input_available);
      EXPECT_DOUBLE_EQ(a.cost, b.cost);
      if (b.cost > 0.0) {
        EXPECT_EQ(a.min_clearance, b.min_clearance);
      }
    }
  }
}

TEST_F(DynamicRiskTest, EmptyScreenDoesNotTurnInvalidInputIntoAvailableEvidence)
{
  point(10, 10);
  auto prepared = prepare();
  EXPECT_TRUE(prepared.input_available);
  EXPECT_EQ(prepared.retained_points, 0u);
  EXPECT_FALSE(evaluator.evaluate(straight, prepared).min_clearance);
  point(20, 20, 0.0, 0.0, false);
  scan.obstacles.back().points[0][0] = std::numeric_limits<double>::quiet_NaN();
  prepared = prepare();
  EXPECT_FALSE(prepared.input_available);
  EXPECT_FALSE(evaluator.evaluate(straight, prepared).input_available);
}

TEST_F(DynamicRiskTest, MissingExpiredFutureAndWrongFrameAreUnavailable)
{
  EXPECT_FALSE(evaluator.prepare({&straight}, nullptr, "odom", now).input_available);
  for (const int64_t stamp : {0LL, 849999999LL, 1200000001LL}) {
    scan.stamp = rclcpp::Time(stamp, RCL_ROS_TIME);
    EXPECT_FALSE(prepare().input_available);
  }
  scan.stamp = rclcpp::Time(1000000000LL, RCL_ROS_TIME);
  scan.frame_id = "map";
  EXPECT_FALSE(prepare().input_available);
}

TEST_F(DynamicRiskTest, InvalidTrajectoryAndKnownVelocityAreUnavailable)
{
  point(10, 10);
  scan.obstacles[0].vx = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(prepare().input_available);
  scan.obstacles.clear();
  straight.poses.pop_back();
  EXPECT_FALSE(prepare().input_available);
}
}  // namespace
}  // namespace my_dwa_controller
