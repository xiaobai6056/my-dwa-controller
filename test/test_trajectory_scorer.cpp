#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <vector>

#include "my_dwa_controller/trajectory_scorer.hpp"

namespace my_dwa_controller
{
namespace
{
constexpr double kTolerance = 1e-12;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

Pose2D makePose(double x, double y, double theta = 0.0)
{
  Pose2D pose;
  pose.x = x;
  pose.y = y;
  pose.theta = theta;
  return pose;
}

Trajectory makeTrajectory(std::initializer_list<Pose2D> poses, double speed = 0.5)
{
  return {{speed, 0.0}, poses};
}

TEST(TrajectoryScorer, ComputesAllCostsAndDefaultWeightedTotal)
{
  const TrajectoryScorer scorer(1.0);
  const auto trajectory = makeTrajectory({makePose(1.0, 1.0), makePose(3.0, 1.0)});
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(4.0, 0.0)};
  const auto result = scorer.score(trajectory, path, 126.0);
  EXPECT_DOUBLE_EQ(result.obstacle_cost, 0.5);
  EXPECT_DOUBLE_EQ(result.speed_cost, 0.5);
  EXPECT_NEAR(result.goal_cost, 2.6, kTolerance);
  EXPECT_DOUBLE_EQ(result.path_cost, 2.0);
  const double heading = std::abs(std::atan2(-1.0, 0.6)) / std::acos(-1.0);
  EXPECT_NEAR(result.heading_cost, heading, kTolerance);
  EXPECT_NEAR(result.total_cost, 0.1 + 0.6 * 2.6 + 0.4 + 0.05 * heading, kTolerance);
}

TEST(TrajectoryScorer, UsesCustomWeightsAndReferenceDistancesWithoutRenormalizing)
{
  const TrajectoryScorer scorer(2.0, {2.0, 3.0, 4.0, 5.0, 0.0}, 2.0, 4.0, 4.0);
  const auto trajectory = makeTrajectory({makePose(1.0, 1.0), makePose(3.0, 1.0)}, 0.5);
  const auto result = scorer.score(trajectory, {makePose(0.0, 0.0), makePose(4.0, 0.0)}, 63.0);
  EXPECT_DOUBLE_EQ(result.obstacle_cost, 0.25);
  EXPECT_DOUBLE_EQ(result.speed_cost, 0.75);
  EXPECT_NEAR(result.goal_cost, std::sqrt(2.0) / 2.0, kTolerance);
  EXPECT_DOUBLE_EQ(result.path_cost, 0.25);
  EXPECT_NEAR(result.total_cost, 4.0 + 2.0 * std::sqrt(2.0), kTolerance);
}

TEST(TrajectoryScorer, HasZeroPathDeviationBetweenSparsePathVertices)
{
  const TrajectoryScorer scorer(1.0);
  const auto trajectory = makeTrajectory(
    {makePose(1.0, 0.0, 2.0), makePose(2.0, 0.0), makePose(3.0, 0.0, -2.0)});
  EXPECT_DOUBLE_EQ(
    scorer.score(trajectory, {makePose(0.0, 0.0), makePose(4.0, 0.0)}, 0.0).path_cost, 0.0);
}

TEST(TrajectoryScorer, MeasuresDistanceToSegmentInteriorAndClampedEndpoints)
{
  const TrajectoryScorer scorer(1.0, ScoreWeights{}, 1.0, 1.0);
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(4.0, 0.0)};
  EXPECT_DOUBLE_EQ(scorer.score(makeTrajectory({makePose(2.0, 3.0)}), path, 0.0).path_cost, 3.0);
  EXPECT_DOUBLE_EQ(scorer.score(makeTrajectory({makePose(-3.0, 4.0)}), path, 0.0).path_cost, 5.0);
  EXPECT_DOUBLE_EQ(scorer.score(makeTrajectory({makePose(7.0, 4.0)}), path, 0.0).path_cost, 5.0);
}

TEST(TrajectoryScorer, UsesNearestSegmentOfWholePolyline)
{
  const TrajectoryScorer scorer(1.0, ScoreWeights{}, 1.0, 1.0);
  const std::vector<Pose2D> path{
    makePose(0.0, 0.0), makePose(2.0, 0.0), makePose(2.0, 4.0)};
  const auto result = scorer.score(makeTrajectory({makePose(3.0, 3.0)}), path, 0.0);
  EXPECT_DOUBLE_EQ(result.path_cost, 1.0);
  EXPECT_NEAR(result.goal_cost, std::hypot(2.4, 3.0), kTolerance);
}

TEST(TrajectoryScorer, AveragesEveryTrajectoryPointIncludingRepeatedAndInitialPoints)
{
  const TrajectoryScorer scorer(1.0, ScoreWeights{}, 1.0, 1.0);
  const auto trajectory = makeTrajectory(
    {makePose(1.0, 1.0), makePose(1.0, 1.0), makePose(2.0, 4.0)});
  EXPECT_DOUBLE_EQ(
    scorer.score(trajectory, {makePose(0.0, 0.0), makePose(4.0, 0.0)}, 0.0).path_cost, 2.0);
}

TEST(TrajectoryScorer, HandlesSinglePointPathAndZeroLengthSegments)
{
  const TrajectoryScorer scorer(1.0, ScoreWeights{}, 1.0, 1.0);
  const auto trajectory = makeTrajectory({makePose(3.0, 4.0)});
  const auto single = scorer.score(trajectory, {makePose(0.0, 0.0)}, 0.0);
  const auto repeated = scorer.score(trajectory, std::vector<Pose2D>(3, makePose(0.0, 0.0)), 0.0);
  EXPECT_DOUBLE_EQ(single.path_cost, 5.0);
  EXPECT_DOUBLE_EQ(single.goal_cost, 5.0);
  EXPECT_DOUBLE_EQ(repeated.path_cost, 5.0);
  EXPECT_DOUBLE_EQ(repeated.goal_cost, 5.0);
}

TEST(TrajectoryScorer, IncreasingObstacleCostIncreasesOnlyObstacleTermAndTotal)
{
  const TrajectoryScorer scorer(1.0);
  const auto trajectory = makeTrajectory({makePose(1.0, 0.0)});
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(2.0, 0.0)};
  const auto low = scorer.score(trajectory, path, 0.0);
  const auto high = scorer.score(trajectory, path, 252.0);
  EXPECT_DOUBLE_EQ(high.obstacle_cost, 1.0);
  EXPECT_DOUBLE_EQ(low.obstacle_cost, 0.0);
  EXPECT_DOUBLE_EQ(high.speed_cost, low.speed_cost);
  EXPECT_DOUBLE_EQ(high.goal_cost, low.goal_cost);
  EXPECT_DOUBLE_EQ(high.path_cost, low.path_cost);
  EXPECT_NEAR(high.total_cost - low.total_cost, ScoreWeights{}.obstacle, kTolerance);
}

TEST(TrajectoryScorer, IncreasingSpeedDecreasesOnlySpeedTermAndTotal)
{
  const TrajectoryScorer scorer(1.0);
  auto trajectory = makeTrajectory({makePose(1.0, 0.0)}, 0.0);
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(2.0, 0.0)};
  const auto slow = scorer.score(trajectory, path, 126.0);
  trajectory.command.v = 1.0;
  const auto fast = scorer.score(trajectory, path, 126.0);
  EXPECT_DOUBLE_EQ(slow.speed_cost, 1.0);
  EXPECT_DOUBLE_EQ(fast.speed_cost, 0.0);
  EXPECT_DOUBLE_EQ(fast.obstacle_cost, slow.obstacle_cost);
  EXPECT_DOUBLE_EQ(fast.goal_cost, slow.goal_cost);
  EXPECT_DOUBLE_EQ(fast.path_cost, slow.path_cost);
  EXPECT_NEAR(slow.total_cost - fast.total_cost, 0.1, kTolerance);
}

TEST(TrajectoryScorer, RejectsReverseAndAboveMaximumSpeeds)
{
  const TrajectoryScorer scorer(0.5);
  const std::vector<Pose2D> path{makePose(0.0, 0.0)};
  const std::array<double, 6> invalid{{
    -1.0, -std::numeric_limits<double>::max(), std::nextafter(0.0, -1.0),
    std::nextafter(0.5, 1.0), 1.0, std::numeric_limits<double>::max()}};
  for (double speed : invalid) {
    SCOPED_TRACE(::testing::Message() << "speed=" << speed);
    EXPECT_THROW(
      scorer.score(makeTrajectory({makePose(0.0, 0.0)}, speed), path, 0.0),
      std::invalid_argument);
  }
}

TEST(TrajectoryScorer, AcceptsForwardSpeedRangeIncludingBothEndpoints)
{
  const TrajectoryScorer scorer(0.5);
  const std::vector<Pose2D> path{makePose(0.0, 0.0)};
  const std::array<double, 3> speeds{{0.0, 0.25, 0.5}};
  const std::array<double, 3> expected{{1.0, 0.5, 0.0}};
  for (std::size_t i = 0; i < speeds.size(); ++i) {
    EXPECT_DOUBLE_EQ(
      scorer.score(makeTrajectory({makePose(0.0, 0.0)}, speeds[i]), path, 0.0).speed_cost,
      expected[i]);
  }
}

TEST(TrajectoryScorer, AllowsIndividualZeroWeightsAndCopiesConfiguration)
{
  ScoreWeights weights{0.0, 1.0, 0.0, 0.0, 0.0};
  const TrajectoryScorer scorer(1.0, weights);
  weights.speed = 9.0;
  const auto result = scorer.score(
    makeTrajectory({makePose(1.0, 0.0)}, 0.25), {makePose(0.0, 0.0)}, 252.0);
  EXPECT_DOUBLE_EQ(result.total_cost, 0.75);
  EXPECT_DOUBLE_EQ(result.obstacle_cost, 1.0);
  EXPECT_DOUBLE_EQ(result.goal_cost, 1.0);
  EXPECT_DOUBLE_EQ(result.path_cost, 2.0);
}

TEST(TrajectoryScorer, RejectsEmptyTrajectoryAndReferencePath)
{
  const TrajectoryScorer scorer(1.0);
  EXPECT_THROW(scorer.score(Trajectory{}, {makePose(0.0, 0.0)}, 0.0), std::invalid_argument);
  EXPECT_THROW(
    scorer.score(makeTrajectory({makePose(0.0, 0.0)}), {}, 0.0), std::invalid_argument);
  EXPECT_THROW(scorer.score(Trajectory{}, {}, 0.0), std::invalid_argument);
}

TEST(TrajectoryScorer, RejectsInvalidSpeedLimitAndReferenceDistances)
{
  const std::array<double, 5> invalid{{0.0, -1.0, kInfinity, -kInfinity, std::nan("")}};
  for (double value : invalid) {
    EXPECT_THROW(TrajectoryScorer{value}, std::invalid_argument);
    EXPECT_THROW((TrajectoryScorer{1.0, ScoreWeights{}, value, 0.5}), std::invalid_argument);
    EXPECT_THROW((TrajectoryScorer{1.0, ScoreWeights{}, 1.0, value}), std::invalid_argument);
  }
}

TEST(TrajectoryScorer, RejectsNegativeNonFiniteAndAllZeroWeights)
{
  const std::array<double ScoreWeights::*, 5> fields{{
    &ScoreWeights::obstacle, &ScoreWeights::speed, &ScoreWeights::goal, &ScoreWeights::path,
    &ScoreWeights::heading}};
  for (auto field : fields) {
    for (double value : {-1.0, kInfinity, -kInfinity, std::nan("")}) {
      ScoreWeights weights;
      weights.*field = value;
      EXPECT_THROW((TrajectoryScorer{1.0, weights}), std::invalid_argument);
    }
  }
  EXPECT_THROW((TrajectoryScorer{1.0, {0.0, 0.0, 0.0, 0.0, 0.0}}), std::invalid_argument);
}

TEST(TrajectoryScorer, RejectsInvalidMeanObstacleCost)
{
  const TrajectoryScorer scorer(1.0);
  for (double cost : {-0.1, 252.1, 253.0, kInfinity, -kInfinity, std::nan("")}) {
    EXPECT_THROW(
      scorer.score(makeTrajectory({makePose(0.0, 0.0)}), {makePose(0.0, 0.0)}, cost),
      std::invalid_argument);
  }
}

TEST(TrajectoryScorer, RejectsAllNonFinitePoseComponentsInTrajectoryAndPath)
{
  const TrajectoryScorer scorer(1.0);
  const std::array<double Pose2D::*, 3> fields{{&Pose2D::x, &Pose2D::y, &Pose2D::theta}};
  for (auto field : fields) {
    for (double value : {kInfinity, -kInfinity, std::nan("")}) {
      auto invalid = makePose(0.0, 0.0);
      invalid.*field = value;
      EXPECT_THROW(
        scorer.score(makeTrajectory({invalid, makePose(0.0, 0.0)}), {makePose(0.0, 0.0)}, 0.0),
        std::invalid_argument);
      EXPECT_THROW(
        scorer.score(makeTrajectory({makePose(0.0, 0.0)}), {makePose(0.0, 0.0), invalid}, 0.0),
        std::invalid_argument);
    }
  }
}

TEST(TrajectoryScorer, RejectsNonFiniteCommandComponents)
{
  const TrajectoryScorer scorer(1.0);
  const std::array<double Velocity2D::*, 2> fields{{&Velocity2D::v, &Velocity2D::omega}};
  for (auto field : fields) {
    for (double value : {kInfinity, -kInfinity, std::nan("")}) {
      auto trajectory = makeTrajectory({makePose(0.0, 0.0)});
      trajectory.command.*field = value;
      EXPECT_THROW(scorer.score(trajectory, {makePose(0.0, 0.0)}, 0.0), std::invalid_argument);
    }
  }
}

TEST(TrajectoryScorer, RejectsUnrepresentableGeometryAndNormalizedCosts)
{
  const TrajectoryScorer scorer(1.0);
  EXPECT_THROW(
    scorer.score(makeTrajectory({makePose(1e308, 0.0)}), {makePose(0.0, 0.0)}, 0.0),
    std::overflow_error);
  const double tiny = std::numeric_limits<double>::min();
  const auto trajectory = makeTrajectory({makePose(10.0, 0.0)});
  const std::vector<Pose2D> path{makePose(0.0, 0.0)};
  const TrajectoryScorer tiny_goal(1.0, ScoreWeights{}, tiny, 0.5);
  const TrajectoryScorer tiny_path(1.0, ScoreWeights{}, 1.0, tiny);
  EXPECT_THROW(tiny_goal.score(trajectory, path, 0.0), std::overflow_error);
  EXPECT_THROW(tiny_path.score(trajectory, path, 0.0), std::overflow_error);
}

TEST(TrajectoryScorer, RejectsWeightedTotalOverflow)
{
  const double maximum = std::numeric_limits<double>::max();
  const TrajectoryScorer scorer(1.0, {maximum, maximum, 0.0, 0.0});
  EXPECT_THROW(
    scorer.score(makeTrajectory({makePose(0.0, 0.0)}, 0.0), {makePose(0.0, 0.0)}, 252.0),
    std::overflow_error);
}

TEST(TrajectoryScorer, FollowLookaheadUsesPointSixMetersInsteadOfLocalTail)
{
  const TrajectoryScorer scorer(1.0);
  const auto result = scorer.score(
    makeTrajectory({makePose(0.0, 0.0), makePose(0.2, 0.0)}),
    {makePose(0.0, 0.0), makePose(2.0, 0.0)}, 0.0);
  EXPECT_NEAR(result.goal_cost, 0.4, kTolerance);
}

TEST(TrajectoryScorer, FollowLookaheadInterpolatesArcLengthAcrossCorner)
{
  const TrajectoryScorer scorer(1.0);
  const auto result = scorer.score(
    makeTrajectory({makePose(0.0, 0.0)}),
    {makePose(0.0, 0.0), makePose(0.3, 0.0), makePose(0.3, 1.0)}, 0.0);
  EXPECT_NEAR(result.goal_cost, std::hypot(0.3, 0.3), kTolerance);
}

TEST(TrajectoryScorer, FollowLookaheadRetainsDistanceNormalization)
{
  const TrajectoryScorer scorer(1.0, ScoreWeights{}, 2.0, 0.5);
  const auto result = scorer.score(
    makeTrajectory({makePose(0.0, 0.0)}),
    {makePose(0.0, 0.0), makePose(2.0, 0.0)}, 0.0);
  EXPECT_NEAR(result.goal_cost, 0.3, kTolerance);
}


TEST(TrajectoryScorer, ConfigurableFollowLookaheadKeepsOtherCostFormulas)
{
  const TrajectoryScorer scorer(1.0, {0.1, 0.1, 0.6, 0.2, 0.0}, 2.0, 0.5, 0.2, 0.6);
  const auto result = scorer.score(
    makeTrajectory({makePose(0.0, 1.0), makePose(0.1, 1.0)}, 0.5),
    {makePose(0.0, 0.0), makePose(2.0, 0.0)}, 126.0);
  EXPECT_DOUBLE_EQ(result.obstacle_cost, 0.5);
  EXPECT_DOUBLE_EQ(result.speed_cost, 0.5);
  EXPECT_DOUBLE_EQ(result.path_cost, 2.0);
  EXPECT_NEAR(result.goal_cost, std::hypot(0.1, 1.0) / 2.0, kTolerance);
}

TEST(TrajectoryScorer, HeadingPrefersUsefulLeftAndRightPureRotations)
{
  const double pi = std::acos(-1.0);
  const TrajectoryScorer scorer(1.0, {0.0, 0.0, 0.0, 0.0, 1.0});
  for (double direction : {-1.0, 1.0}) {
    const std::vector<Pose2D> path{
      makePose(0.0, 0.0, -direction), makePose(0.0, direction * 2.0, -direction)};
    const auto useful = scorer.score(
      makeTrajectory(
        {makePose(0.0, 0.0), makePose(0.0, 0.0, direction * pi / 2.0)}, 0.0), path, 0.0);
    const auto wrong = scorer.score(
      makeTrajectory(
        {makePose(0.0, 0.0), makePose(0.0, 0.0, -direction * pi / 2.0)}, 0.0), path, 0.0);
    const auto stopped = scorer.score(makeTrajectory({makePose(0.0, 0.0)}, 0.0), path, 0.0);
    EXPECT_NEAR(useful.heading_cost, 0.0, kTolerance);
    EXPECT_NEAR(wrong.heading_cost, 1.0, kTolerance);
    EXPECT_NEAR(stopped.heading_cost, 0.5, kTolerance);
    EXPECT_LT(useful.total_cost, stopped.total_cost);
    EXPECT_LT(stopped.total_cost, wrong.total_cost);
  }
}

TEST(TrajectoryScorer, HeadingUsesEndpointProjectionThenArcLookaheadOnCurve)
{
  const double pi = std::acos(-1.0);
  const TrajectoryScorer scorer(1.0);
  const std::vector<Pose2D> path{
    makePose(0.0, 0.0), makePose(0.5, 0.0), makePose(0.5, 2.0)};
  const auto aligned = scorer.score(
    makeTrajectory(
      {makePose(0.0, 0.0), makePose(0.4, 0.0, std::atan2(0.5, 0.1))}), path, 0.0);
  EXPECT_NEAR(aligned.heading_cost, 0.0, kTolerance);
  const TrajectoryScorer short_heading(1.0, ScoreWeights{}, 1.0, 0.5, 0.6, 0.05);
  const auto straight = short_heading.score(
    makeTrajectory({makePose(0.4, 0.0, pi / 2.0)}), path, 0.0);
  EXPECT_NEAR(straight.heading_cost, 0.5, kTolerance);
}

TEST(TrajectoryScorer, HeadingWrapsAcrossBothPiBoundaries)
{
  const double pi = std::acos(-1.0);
  const TrajectoryScorer scorer(1.0);
  for (double sign : {-1.0, 1.0}) {
    const double reference = sign * (pi - 0.01);
    const std::vector<Pose2D> path{
      makePose(0.0, 0.0), makePose(2.0 * std::cos(reference), 2.0 * std::sin(reference))};
    const auto result = scorer.score(
      makeTrajectory({makePose(0.0, 0.0, -sign * (pi - 0.01))}), path, 0.0);
    EXPECT_NEAR(result.heading_cost, 0.02 / pi, kTolerance);
  }
}

TEST(TrajectoryScorer, ExhaustedHeadingReferenceUsesForwardGeometryNotUTurnOrStoredYaw)
{
  const double pi = std::acos(-1.0);
  const TrajectoryScorer scorer(1.0);
  const std::vector<Pose2D> path{
    makePose(0.0, 0.0, pi), makePose(0.2, 0.0, pi), makePose(0.2, 0.0, pi)};
  for (double x : {0.2, 0.5}) {
    const auto forward = scorer.score(makeTrajectory({makePose(x, 0.0, 0.0)}), path, 0.0);
    const auto backward = scorer.score(makeTrajectory({makePose(x, 0.0, pi)}), path, 0.0);
    EXPECT_NEAR(forward.heading_cost, 0.0, kTolerance);
    EXPECT_NEAR(backward.heading_cost, 1.0, kTolerance);
  }
}

TEST(TrajectoryScorer, DirectionlessPathIsNeutralAndTerminalGateDisablesOnlyHeading)
{
  const TrajectoryScorer scorer(1.0);
  const auto trajectory = makeTrajectory({makePose(0.0, 0.0, 1.0)}, 0.0);
  EXPECT_DOUBLE_EQ(scorer.score(trajectory, {makePose(0.0, 0.0)}, 0.0).heading_cost, 0.0);
  const std::vector<Pose2D> path{makePose(0.0, 0.0), makePose(2.0, 0.0)};
  const auto enabled = scorer.score(trajectory, path, 126.0);
  const auto disabled = scorer.score(trajectory, path, 126.0, false);
  EXPECT_GT(enabled.heading_cost, 0.0);
  EXPECT_DOUBLE_EQ(disabled.heading_cost, 0.0);
  EXPECT_DOUBLE_EQ(enabled.obstacle_cost, disabled.obstacle_cost);
  EXPECT_DOUBLE_EQ(enabled.speed_cost, disabled.speed_cost);
  EXPECT_DOUBLE_EQ(enabled.path_cost, disabled.path_cost);
  EXPECT_DOUBLE_EQ(enabled.goal_cost, disabled.goal_cost);
  EXPECT_NEAR(
    enabled.total_cost - disabled.total_cost, ScoreWeights{}.heading * enabled.heading_cost,
    kTolerance);
}

TEST(TrajectoryScorer, RejectsInvalidLookaheadsAndHeadingWeight)
{
  for (double bad : {0.0, -0.1, kInfinity, std::nan("")}) {
    EXPECT_THROW(
      (TrajectoryScorer{1.0, ScoreWeights{}, 1.0, 0.5, bad, 0.6}),
      std::invalid_argument);
    EXPECT_THROW(
      (TrajectoryScorer{1.0, ScoreWeights{}, 1.0, 0.5, 0.6, bad}),
      std::invalid_argument);
  }
  for (double bad : {-0.1, kInfinity, std::nan("")}) {
    auto weights = ScoreWeights{};
    weights.heading = bad;
    EXPECT_THROW((TrajectoryScorer{1.0, weights}), std::invalid_argument);
  }
}

}  // 匿名命名空间
}  // 命名空间 my_dwa_controller
