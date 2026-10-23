#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "my_dwa_controller/trajectory_predictor.hpp"

namespace my_dwa_controller
{
namespace
{
constexpr double kTolerance = 1e-12;
constexpr double kPi = 3.14159265358979323846;

Pose2D makePose(double x, double y, double theta)
{
  Pose2D pose;
  pose.x = x;
  pose.y = y;
  pose.theta = theta;
  return pose;
}

void expectPose(const Pose2D & pose, double x, double y, double theta)
{
  EXPECT_NEAR(pose.x, x, kTolerance);
  EXPECT_NEAR(pose.y, y, kTolerance);
  EXPECT_NEAR(pose.theta, theta, kTolerance);
}

TEST(TrajectoryPredictor, ZeroVelocityPreservesEveryPose)
{
  const TrajectoryPredictor predictor(1.0, 0.25);
  const auto trajectory = predictor.predict(makePose(2.0, -3.0, 0.7), {0.0, 0.0});

  ASSERT_EQ(trajectory.poses.size(), 5u);
  for (const auto & pose : trajectory.poses) {
    expectPose(pose, 2.0, -3.0, 0.7);
  }
}

TEST(TrajectoryPredictor, StraightMotionIncludesInitialPoseAndEndpoint)
{
  const TrajectoryPredictor predictor(2.0, 0.5);
  const auto trajectory = predictor.predict(makePose(1.0, -2.0, 0.0), {1.5, 0.0});

  ASSERT_EQ(trajectory.poses.size(), 5u);
  expectPose(trajectory.poses.front(), 1.0, -2.0, 0.0);
  expectPose(trajectory.poses.back(), 4.0, -2.0, 0.0);
}

TEST(TrajectoryPredictor, UsesInitialHeadingInInputFrame)
{
  const TrajectoryPredictor predictor(1.0, 0.25);
  const auto trajectory = predictor.predict(makePose(-1.0, 2.0, kPi / 2.0), {2.0, 0.0});

  ASSERT_EQ(trajectory.poses.size(), 5u);
  expectPose(trajectory.poses.back(), -1.0, 4.0, kPi / 2.0);
}

TEST(TrajectoryPredictor, SupportsReverseMotion)
{
  const TrajectoryPredictor predictor(1.0, 0.25);
  const auto trajectory = predictor.predict(makePose(1.0, 2.0, 0.0), {-2.0, 0.0});

  ASSERT_EQ(trajectory.poses.size(), 5u);
  expectPose(trajectory.poses.back(), -1.0, 2.0, 0.0);
}

TEST(TrajectoryPredictor, RotationKeepsPositionAndUnwrappedAngle)
{
  const TrajectoryPredictor predictor(1.0, 0.25);
  const auto trajectory = predictor.predict(makePose(2.0, -1.0, 3.0), {0.0, 2.0});

  ASSERT_EQ(trajectory.poses.size(), 5u);
  for (std::size_t i = 0; i < trajectory.poses.size(); ++i) {
    expectPose(trajectory.poses[i], 2.0, -1.0, 3.0 + 0.5 * i);
  }
}

TEST(TrajectoryPredictor, LeftCurveUsesForwardEulerHeading)
{
  const TrajectoryPredictor predictor(1.0, 0.5);
  const auto trajectory = predictor.predict(makePose(0.0, 0.0, 0.0), {1.0, 1.0});

  ASSERT_EQ(trajectory.poses.size(), 3u);
  // Euler 积分使用每步开始前的朝向：先为 0，再为 0.5 rad。
  expectPose(trajectory.poses[1], 0.5, 0.0, 0.5);
  expectPose(trajectory.poses.back(), 0.9387912809451864, 0.2397127693021015, 1.0);
  EXPECT_GT(trajectory.poses.back().x, 0.0);
  EXPECT_GT(trajectory.poses.back().y, 0.0);
}

TEST(TrajectoryPredictor, RightCurveHasNegativeLateralDisplacement)
{
  const TrajectoryPredictor predictor(1.0, 0.5);
  const auto trajectory = predictor.predict(makePose(0.0, 0.0, 0.0), {1.0, -1.0});

  ASSERT_EQ(trajectory.poses.size(), 3u);
  expectPose(trajectory.poses.back(), 0.9387912809451864, -0.2397127693021015, -1.0);
  EXPECT_LT(trajectory.poses.back().y, 0.0);
}

TEST(TrajectoryPredictor, DivisibleBinaryStepContainsAllNodes)
{
  const TrajectoryPredictor predictor(1.0, 0.25);
  const auto trajectory = predictor.predict(makePose(0.0, 0.0, 0.0), {1.0, 0.0});

  ASSERT_EQ(trajectory.poses.size(), 5u);
  for (std::size_t i = 0; i < trajectory.poses.size(); ++i) {
    expectPose(trajectory.poses[i], 0.25 * i, 0.0, 0.0);
  }
}

TEST(TrajectoryPredictor, DecimalStepDoesNotAddSpuriousTail)
{
  for (const auto & times : std::array<std::array<double, 2>, 3>{{
      {{0.3, 0.1}}, {{0.9, 0.3}}, {{1.0, 0.1}}}})
  {
    SCOPED_TRACE(::testing::Message() << "sim_time=" << times[0] << ", dt=" << times[1]);
    const TrajectoryPredictor predictor(times[0], times[1]);
    const auto trajectory = predictor.predict(makePose(0.0, 0.0, 0.0), {1.0, 0.0});
    const std::size_t expected_steps = static_cast<std::size_t>(std::round(times[0] / times[1]));

    ASSERT_EQ(trajectory.poses.size(), expected_steps + 1);
    expectPose(trajectory.poses.back(), times[0], 0.0, 0.0);
  }
}

TEST(TrajectoryPredictor, NonDivisibleStepUsesOnlyRemainingTime)
{
  const TrajectoryPredictor predictor(1.0, 0.3);
  const auto trajectory = predictor.predict(makePose(0.0, 0.0, 0.0), {2.0, 0.0});

  ASSERT_EQ(trajectory.poses.size(), 5u);
  expectPose(trajectory.poses[1], 0.6, 0.0, 0.0);
  expectPose(trajectory.poses[2], 1.2, 0.0, 0.0);
  expectPose(trajectory.poses[3], 1.8, 0.0, 0.0);
  expectPose(trajectory.poses.back(), 2.0, 0.0, 0.0);
  for (const auto & pose : trajectory.poses) {
    EXPECT_LE(pose.x, 2.0);
  }
}

TEST(TrajectoryPredictor, CurvedRemainderUsesHeadingBeforeFinalStep)
{
  const TrajectoryPredictor predictor(0.75, 0.5);
  const auto trajectory = predictor.predict(makePose(0.0, 0.0, 0.0), {1.0, 1.0});

  ASSERT_EQ(trajectory.poses.size(), 3u);
  // 先积分 0.5 s，再以 0.5 rad 的朝向积分剩余的 0.25 s。
  expectPose(trajectory.poses.back(), 0.7193956404725932, 0.1198563846510508, 0.75);
}

TEST(TrajectoryPredictor, HorizonShorterThanStepStillContainsBothEndpoints)
{
  const TrajectoryPredictor predictor(0.05, 0.1);
  const auto trajectory = predictor.predict(makePose(1.0, 2.0, 0.0), {2.0, 1.0});

  ASSERT_EQ(trajectory.poses.size(), 2u);
  expectPose(trajectory.poses.front(), 1.0, 2.0, 0.0);
  expectPose(trajectory.poses.back(), 1.1, 2.0, 0.05);
}

TEST(TrajectoryPredictor, TinyPositiveHorizonIsNotDiscarded)
{
  const TrajectoryPredictor predictor(1e-12, 1e-13);
  const auto trajectory = predictor.predict(makePose(0.0, 0.0, 0.0), {1.0, 0.0});

  ASSERT_EQ(trajectory.poses.size(), 11u);
  EXPECT_NEAR(trajectory.poses.back().x, 1e-12, 1e-26);
}

TEST(TrajectoryPredictor, RetainsCandidateCommand)
{
  const TrajectoryPredictor predictor(1.0, 0.25);
  const Velocity2D command{-0.4, 0.7};
  const auto trajectory = predictor.predict(makePose(0.0, 0.0, 0.0), command);

  EXPECT_DOUBLE_EQ(trajectory.command.v, command.v);
  EXPECT_DOUBLE_EQ(trajectory.command.omega, command.omega);
}

TEST(TrajectoryPredictor, RejectsInvalidTimeParameters)
{
  const double infinity = std::numeric_limits<double>::infinity();
  const std::array<double, 6> invalid_times{{
    0.0, -0.0, -0.1, infinity, -infinity, std::numeric_limits<double>::quiet_NaN()}};

  for (const double value : invalid_times) {
    SCOPED_TRACE(::testing::Message() << "time=" << value);
    EXPECT_THROW((TrajectoryPredictor{value, 0.1}), std::invalid_argument);
    EXPECT_THROW((TrajectoryPredictor{1.0, value}), std::invalid_argument);
  }
}

TEST(TrajectoryPredictor, RejectsEveryNonFinitePoseComponent)
{
  const TrajectoryPredictor predictor(1.0, 0.1);
  const double infinity = std::numeric_limits<double>::infinity();
  const std::array<double, 3> non_finite{{
    infinity, -infinity, std::numeric_limits<double>::quiet_NaN()}};
  const std::array<double Pose2D::*, 3> components{{&Pose2D::x, &Pose2D::y, &Pose2D::theta}};

  for (std::size_t i = 0; i < components.size(); ++i) {
    for (const double value : non_finite) {
      SCOPED_TRACE(::testing::Message() << "pose component=" << i << ", value=" << value);
      auto pose = makePose(0.0, 0.0, 0.0);
      pose.*components[i] = value;
      EXPECT_THROW(predictor.predict(pose, {0.0, 0.0}), std::invalid_argument);
    }
  }
}

TEST(TrajectoryPredictor, RejectsEveryNonFiniteVelocityComponent)
{
  const TrajectoryPredictor predictor(1.0, 0.1);
  const double infinity = std::numeric_limits<double>::infinity();
  const std::array<double, 3> non_finite{{
    infinity, -infinity, std::numeric_limits<double>::quiet_NaN()}};

  for (const double value : non_finite) {
    SCOPED_TRACE(::testing::Message() << "velocity=" << value);
    EXPECT_THROW(predictor.predict(makePose(0.0, 0.0, 0.0), {value, 0.0}), std::invalid_argument);
    EXPECT_THROW(predictor.predict(makePose(0.0, 0.0, 0.0), {0.0, value}), std::invalid_argument);
  }
}

}  // 匿名命名空间
}  // 命名空间 my_dwa_controller
